/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610_alt

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <string.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include "pmw3610.h"

/* Linux input keycodes for arrow keys */
#define ARROWS_KEY_UP    103
#define ARROWS_KEY_DOWN  108
#define ARROWS_KEY_LEFT  105
#define ARROWS_KEY_RIGHT 106

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_ALT_LOG_LEVEL);

enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,
    ASYNC_INIT_STEP_CLEAR_OB1,
    ASYNC_INIT_STEP_CHECK_OB1,
    ASYNC_INIT_STEP_CONFIGURE,
    ASYNC_INIT_STEP_COUNT
};

static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP]  = 10 + CONFIG_PMW3610_ALT_INIT_POWER_UP_EXTRA_DELAY_MS,
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200,
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP]  = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Layer helpers ////////

static bool pmw3610_layer_match(const uint8_t *layers, size_t len) {
    if (len == 0 || layers == NULL) {
        return false;
    }
    uint8_t active = (uint8_t)zmk_keymap_highest_layer_active();
    for (size_t i = 0; i < len; i++) {
        if (layers[i] == active) {
            return true;
        }
    }
    return false;
}

/* arrows_profiles: flat array of uint16 groups [layer, up, down, left, right, tick, remainder, no_diagonal, one_shot]
 * Returns pointer to the matching group (9 elements), or NULL if no match. */
static const uint16_t *pmw3610_arrows_profile_match(const struct pixart_config *config) {
    if (config->arrows_profiles == NULL || config->arrows_profiles_count == 0) {
        return NULL;
    }
    uint16_t active = (uint16_t)zmk_keymap_highest_layer_active();
    for (size_t i = 0; i < config->arrows_profiles_count; i++) {
        const uint16_t *p = &config->arrows_profiles[i * 9];
        if (p[0] == active) {
            return p;
        }
    }
    return NULL;
}

/* arrows-alt-profiles: matches any active layer (not just highest).
 * Used when arrows_alt_mode_g is true. */
static bool arrows_alt_mode_g = false;

void pmw3610_arrows_alt_mode_set(bool val) {
    arrows_alt_mode_g = val;
}

static const uint16_t *pmw3610_arrows_alt_profile_match(const struct pixart_config *config) {
    if (config->arrows_alt_profiles == NULL || config->arrows_alt_profiles_count == 0) {
        return NULL;
    }
    for (size_t i = 0; i < config->arrows_alt_profiles_count; i++) {
        const uint16_t *p = &config->arrows_alt_profiles[i * 9];
        if (zmk_keymap_layer_active((uint8_t)p[0])) {
            return p;
        }
    }
    return NULL;
}

//////// SPI helpers ////////

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {
    const struct pixart_config *cfg = dev->config;
    const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf[] = {
        { .buf = NULL,  .len = sizeof(addr) },
        { .buf = value, .len = len },
    };
    const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
    return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
    return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
    const struct pixart_config *cfg = dev->config;
    uint8_t write_buf[] = { addr | SPI_WRITE_BIT, value };
    const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf) };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));
    int err = pmw3610_write_reg(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    return 0;
}

//////// CPI ////////

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi,
                           bool swap_xy, bool inv_x, bool inv_y) {
#if IS_ENABLED(CONFIG_PMW3610_ALT_ORIENTATION_180)
    inv_x = true;
    inv_y = true;
#endif

#if CONFIG_PMW3610_ALT_CPI_DIVIDOR > 1
    cpi = cpi / CONFIG_PMW3610_ALT_CPI_DIVIDOR;
    if (cpi < PMW3610_MIN_CPI) {
        cpi = PMW3610_MIN_CPI;
    }
#endif

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    uint8_t value = 0x00;
    uint8_t cpi_val = cpi / 200;
    value = (value & 0xE0) | (cpi_val & 0x1F);

    LOG_INF("Setting cpi: %d, swap_xy: %s inv_x: %s inv_y: %s",
            cpi, swap_xy ? "yes" : "no", inv_x ? "yes" : "no", inv_y ? "yes" : "no");

#if IS_ENABLED(CONFIG_PMW3610_ALT_SWAP_XY)
    value |= (1 << 7);
#else
    if (swap_xy) { value |= (1 << 7); } else { value &= ~(1 << 7); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_X)
    value |= (1 << 6);
#else
    if (inv_x) { value |= (1 << 6); } else { value &= ~(1 << 6); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_Y)
    value |= (1 << 5);
#else
    if (inv_y) { value |= (1 << 5); } else { value &= ~(1 << 5); }
#endif

    LOG_INF("Setting CPI reg value 0x%x", value);

    uint8_t addr[] = { 0x7F, PMW3610_REG_RES_STEP, 0x7F };
    uint8_t data[] = { 0xFF, value,                0x00 };
    int err = 0;

    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));
    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    return err;
}

//////// Timing ////////

static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range", sample_time);
        return -EINVAL;
    }
    uint8_t value = sample_time / mintime;
    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }
    return err;
}

static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime, mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        maxtime = 8160;
        mintime = 32;
        break;
    case PMW3610_REG_REST1_DOWNSHIFT:
        maxtime = 255 * 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS;
        break;
    case PMW3610_REG_REST2_DOWNSHIFT:
        maxtime = 255 * 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS;
        break;
    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    uint8_t value = time / mintime;
    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }
    return err;
}

//////// Performance ////////

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
    const struct pixart_config *config = dev->config;
    int err = 0;

#if defined(CONFIG_PMW3610_ALT_POLLING_RATE_125)
    /* 125 Hz: Performance register lower nibble = 0x00 */
    uint8_t perf = 0x00;
#else
    /* 250 Hz (default): Performance register lower nibble = 0x0D */
    uint8_t perf = 0x0D;
#endif

    /* force_awake: always keep sensor in RUN mode regardless of ZMK idle state */
    if (config->force_awake) {
        perf |= 0xF0;
    }

    uint8_t value;
    err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
    if (err) {
        LOG_ERR("Can't read performance %d", err);
        return err;
    }

    if (perf != value) {
        err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);
        if (err) {
            LOG_ERR("Can't write performance %d", err);
            return err;
        }
    }
    LOG_INF("Set performance register: 0x%02x (polling=%s, force_awake=%s)",
            perf,
            IS_ENABLED(CONFIG_PMW3610_ALT_POLLING_RATE_125) ? "125Hz" : "250Hz",
            config->force_awake ? "always-on" : "off");
    return err;
}

//////// IRQ ////////

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

//////// Async init ////////

static int pmw3610_async_init_power_up(const struct device *dev) {
    int ret = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    return ret < 0 ? ret : 0;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }
    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    uint8_t product_id = 0x01;
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }
    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }
    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    int err = 0;
    const struct pixart_config *config = dev->config;

    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = pmw3610_read_reg(dev, reg, buf);
    }

    if (!err) err = pmw3610_set_performance(dev, true);
    if (!err) err = pmw3610_set_cpi(dev, config->cpi, config->swap_xy, config->inv_x, config->inv_y);
    if (!err) err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,  CONFIG_PMW3610_ALT_RUN_DOWNSHIFT_TIME_MS);
    if (!err) err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, CONFIG_PMW3610_ALT_REST1_DOWNSHIFT_TIME_MS);
    if (!err) err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, CONFIG_PMW3610_ALT_REST2_DOWNSHIFT_TIME_MS);
    if (!err) err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS);
    if (!err) err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS);
    if (!err) err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, CONFIG_PMW3610_ALT_REST3_SAMPLE_TIME_MS);

    if (err) {
        LOG_ERR("Config the sensor failed");
    }
    return err;
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("PMW3610 initialization failed in step %d", data->async_init_step);
    } else {
        data->async_init_step++;
        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true;
            LOG_INF("PMW3610 initialized");
            pmw3610_set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

//////// Linux keycode → ZMK HID encoded (keyboard page 0x07) ////////

/* input_report(INPUT_EV_KEY) is NOT processed by ZMK's pointing input handler.
 * Must use ZMK's keycode_state_changed event system instead. */
static uint32_t linux_key_to_zmk(uint16_t linux_key) {
#define HID_KB(hid)       (((uint32_t)0x07 << 16) | (hid))
#define HID_CONSUMER(hid) (((uint32_t)0x0C << 16) | (hid))
    switch (linux_key) {
        case 102: return HID_KB(0x4A); /* Home */
        case 103: return HID_KB(0x52); /* Up */
        case 104: return HID_KB(0x4B); /* PageUp */
        case 105: return HID_KB(0x50); /* Left */
        case 106: return HID_KB(0x4F); /* Right */
        case 107: return HID_KB(0x4D); /* End */
        case 108: return HID_KB(0x51); /* Down */
        case 109: return HID_KB(0x4E); /* PageDown */
        case 2:   return HID_KB(0x1E); /* 1 */
        case 3:   return HID_KB(0x1F); /* 2 */
        case 4:   return HID_KB(0x20); /* 3 */
        case 5:   return HID_KB(0x21); /* 4 */
        case 6:   return HID_KB(0x22); /* 5 */
        case 7:   return HID_KB(0x23); /* 6 */
        case 8:   return HID_KB(0x24); /* 7 */
        case 9:   return HID_KB(0x25); /* 8 */
        case 10:  return HID_KB(0x26); /* 9 */
        case 11:  return HID_KB(0x27); /* 0 */
        case 28:  return HID_KB(0x28); /* Enter */
        case 14:  return HID_KB(0x2A); /* Backspace */
        case 52:  return HID_KB(0x37); /* . */
        case 15:  return HID_KB(0x2B); /* Tab */
        case 111: return HID_KB(0x4C); /* Delete */
        /* Special combos: modifier + key */
        case 1000: return (0x08U << 24) | HID_KB(0x1D); /* Cmd+Z */
        case 1001: return (0x08U << 24) | HID_KB(0x1B); /* Cmd+X */
        case 1002: return (0x08U << 24) | HID_KB(0x06); /* Cmd+C */
        case 1003: return (0x08U << 24) | HID_KB(0x19); /* Cmd+V */
        case 1010: return (0x02U << 24) | HID_KB(0x2B); /* Shift+Tab */
        case 1020: return (0x02U << 24) | HID_KB(0x52); /* Shift+Up */
        case 1021: return (0x02U << 24) | HID_KB(0x51); /* Shift+Down */
        case 1022: return (0x02U << 24) | HID_KB(0x4F); /* Shift+Right */
        case 1023: return (0x02U << 24) | HID_KB(0x50); /* Shift+Left */
        case 1040: return (0x04U << 24) | HID_KB(0x50); /* Opt+Left  (word-left)  */
        case 1041: return (0x04U << 24) | HID_KB(0x51); /* Opt+Down  (para-down)  */
        case 1042: return (0x04U << 24) | HID_KB(0x52); /* Opt+Up    (para-up)    */
        case 1043: return (0x04U << 24) | HID_KB(0x4F); /* Opt+Right (word-right) */
        case 1004: return (0x0AU << 24) | HID_KB(0x1D); /* Cmd+Shift+Z  (Redo)            */
        case 1005: return (0x08U << 24) | HID_KB(0x04); /* Cmd+A        (全選択)           */
        case 1060: return (0x01U << 24) | HID_KB(0x52); /* Ctrl+Up      (Mission Control)  */
        case 1061: return (0x01U << 24) | HID_KB(0x51); /* Ctrl+Down    (App Expose)       */
        case 1062: return (0x01U << 24) | HID_KB(0x50); /* Ctrl+Left    (前のデスクトップ)  */
        case 1063: return (0x01U << 24) | HID_KB(0x4F); /* Ctrl+Right   (次のデスクトップ)  */
        case 1064: return (0x0AU << 24) | HID_KB(0x2B); /* Cmd+Shift+Tab                  */
        case 1065: return (0x08U << 24) | HID_KB(0x2B); /* Cmd+Tab      (アプリ切り替え)   */
        case 1066: return HID_KB(0x6A); /* F15 (左のデスクトップ) */
        case 1067: return HID_KB(0x69); /* F14 (右のデスクトップ) */
        case 1068: return (0x0CU << 24) | HID_KB(0x29); /* Cmd+Opt+Esc  (強制終了)          */
        case 1069: return (0x09U << 24) | HID_KB(0x14); /* Ctrl+Cmd+Q   (画面ロック)        */
        /* 1070/1071: swapper-style Cmd+Tab/Cmd+Shift+Tab — handled in pmw3610_send_arrow_key */
        case 58:  return HID_KB(0x91); /* LANG2 (英数) */
        case 90:  return HID_KB(0x90); /* LANG1 (かな) */
        /* Consumer page (0x0C): Linux standard media keycodes */
        case 114: return HID_CONSUMER(0xEA); /* KEY_VOLUMEDOWN  → C_VOL_DN */
        case 115: return HID_CONSUMER(0xE9); /* KEY_VOLUMEUP    → C_VOL_UP */
        case 163: return HID_CONSUMER(0xB5); /* KEY_NEXTSONG    → C_NEXT   */
        case 164: return HID_CONSUMER(0xCD); /* KEY_PLAYPAUSE    → C_PP     */
        case 165: return HID_CONSUMER(0xB6); /* KEY_PREVIOUSSONG→ C_PREV   */
        case 166: return HID_CONSUMER(0xB7); /* KEY_STOPCD       → C_STOP   */
        default:  return 0;
    }
#undef HID_KB
#undef HID_CONSUMER
}

static void pmw3610_raise_key(uint16_t linux_key) {
    uint32_t usage = linux_key_to_zmk(linux_key);
    if (usage == 0) return;
    raise_zmk_keycode_state_changed_from_encoded(usage, true, k_uptime_get());
    raise_zmk_keycode_state_changed_from_encoded(usage, false, k_uptime_get());
}

//////// Arrows swapper (Cmd held across ticks, state-machine) ////////

/* LGUI key usage (keyboard page 0x07, keycode 0xE3) — pressed as a key, not modifier byte */
#define SWAPPER_LGUI_USAGE       ((0x07U << 16) | 0xE3U)
#define SWAPPER_TAB_USAGE        ((0x07U << 16) | 0x2BU)
#define SWAPPER_SHIFT_TAB_USAGE  ((0x02U << 24) | (0x07U << 16) | 0x2BU)
/* Cmd auto-releases this many ms after the last swapper tick */
#define ARROWS_SWAPPER_TIMEOUT_MS  1000
/* Delay from Cmd press to first Tab (ms) */
#define SWAPPER_CMD_TAB_DELAY_MS   30

/* Immediately abort swapper: cancel pending works, release Cmd if held, reset state. */
static void pmw3610_swapper_abort(struct pixart_data *data) {
    k_work_cancel_delayable(&data->arrows_swapper_tab_work);
    k_work_cancel_delayable(&data->arrows_swapper_release_work);
    if (data->arrows_swapper_state != SWAPPER_IDLE) {
        raise_zmk_keycode_state_changed_from_encoded(SWAPPER_LGUI_USAGE, false, k_uptime_get());
        data->arrows_swapper_state = SWAPPER_IDLE;
        data->arrows_swapper_key   = 0;
        LOG_DBG("swapper: aborted → IDLE");
    }
}

static void pmw3610_swapper_release_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, arrows_swapper_release_work);
    if (data->arrows_swapper_state != SWAPPER_IDLE) {
        raise_zmk_keycode_state_changed_from_encoded(SWAPPER_LGUI_USAGE, false, k_uptime_get());
        data->arrows_swapper_state = SWAPPER_IDLE;
        data->arrows_swapper_key   = 0;
        LOG_DBG("swapper: Cmd released → IDLE");
    }
}

static void pmw3610_swapper_tab_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, arrows_swapper_tab_work);

    if (data->arrows_swapper_state != SWAPPER_PENDING) {
        return;
    }

    uint32_t tab_usage = (data->arrows_swapper_key == 1070) ? SWAPPER_TAB_USAGE : SWAPPER_SHIFT_TAB_USAGE;
    raise_zmk_keycode_state_changed_from_encoded(tab_usage, true, k_uptime_get());
    raise_zmk_keycode_state_changed_from_encoded(tab_usage, false, k_uptime_get());

    data->arrows_swapper_state = SWAPPER_ACTIVE;
    LOG_DBG("swapper: Tab sent → ACTIVE");
}

/* key 1070: Swapper Cmd+Tab (forward app switch)
 * key 1071: Swapper Cmd+Shift+Tab (reverse app switch)
 *
 * State machine:
 *   IDLE    → press Cmd, store key, schedule tab_work in 30ms → PENDING
 *   PENDING → overwrite key (reverse), reschedule tab_work
 *   ACTIVE  → send Tab immediately
 * Timeout (600ms): release_work fires → IDLE */
static void pmw3610_swapper_fire(struct pixart_data *data, uint16_t key) {
    switch (data->arrows_swapper_state) {
    case SWAPPER_IDLE:
        raise_zmk_keycode_state_changed_from_encoded(SWAPPER_LGUI_USAGE, true, k_uptime_get());
        data->arrows_swapper_key   = key;
        data->arrows_swapper_state = SWAPPER_PENDING;
        k_work_schedule(&data->arrows_swapper_tab_work, K_MSEC(SWAPPER_CMD_TAB_DELAY_MS));
        LOG_DBG("swapper: Cmd pressed → PENDING");
        break;

    case SWAPPER_PENDING:
        data->arrows_swapper_key = key;
        k_work_reschedule(&data->arrows_swapper_tab_work, K_MSEC(SWAPPER_CMD_TAB_DELAY_MS));
        LOG_DBG("swapper: PENDING key overwrite key=%u", key);
        break;

    case SWAPPER_ACTIVE: {
        uint32_t tab_usage = (key == 1070) ? SWAPPER_TAB_USAGE : SWAPPER_SHIFT_TAB_USAGE;
        raise_zmk_keycode_state_changed_from_encoded(tab_usage, true, k_uptime_get());
        raise_zmk_keycode_state_changed_from_encoded(tab_usage, false, k_uptime_get());
        LOG_DBG("swapper: ACTIVE Tab sent key=%u", key);
        break;
    }
    }

    k_work_reschedule(&data->arrows_swapper_release_work, K_MSEC(ARROWS_SWAPPER_TIMEOUT_MS));
}

//////// Arrows helper ////////

static void pmw3610_send_arrow_key(const struct device *dev, uint16_t key) {
    if (key == 1070 || key == 1071) {
        pmw3610_swapper_fire(dev->data, key);
    } else if (key >= 2000 && key <= 2003) {
        /* Scroll wheel emit: bypass HID keycode path, send REL_WHEEL/REL_HWHEEL */
        switch (key) {
        case 2000: input_report_rel(dev, INPUT_REL_WHEEL,   1, true, K_FOREVER); break; /* up    */
        case 2001: input_report_rel(dev, INPUT_REL_WHEEL,  -1, true, K_FOREVER); break; /* down  */
        case 2002: input_report_rel(dev, INPUT_REL_HWHEEL, -1, true, K_FOREVER); break; /* left  */
        case 2003: input_report_rel(dev, INPUT_REL_HWHEEL,  1, true, K_FOREVER); break; /* right */
        }
    } else {
        ARG_UNUSED(dev);
        pmw3610_raise_key(key);
    }
}

//////// Numpad 2-stroke input ////////

/*
 * 1st stroke:  上→group 0 (1-3)  左→group 1 (4-6)  右→group 2 (7-9)  下→group 3
 * 2nd stroke (group 0-2):  左→col 0  上→col 1  右→col 2
 * 2nd stroke (group 3/下):  下→0  上→.  左→Enter  右→Backspace
 *
 * Linux keycode: 1=2, 2=3, 3=4, 4=5, 5=6, 6=7, 7=8, 8=9, 9=10, 0=11
 *                Enter=28, Backspace=14, .=52
 */
static const uint16_t numpad_keys[3][3] = {
    {2, 3, 4},    /* group 0 (上): 1, 2, 3 */
    {5, 6, 7},    /* group 1 (左): 4, 5, 6 */
    {8, 9, 10},   /* group 2 (右): 7, 8, 9 */
};
/* group 3 (下): [下, 上, 左, 右] */
static const uint16_t numpad_down_keys[4] = {11, 52, 14, 28}; /* 0, ., BS, Enter */

static void pmw3610_numpad_send(const struct device *dev, uint16_t keycode) {
    ARG_UNUSED(dev);
    pmw3610_raise_key(keycode);
    LOG_DBG("numpad key=%d", keycode);
}

static void pmw3610_numpad_reset(struct pixart_data *data) {
    data->numpad_state = 0;
    data->numpad_group = 0;
    data->numpad_dx    = 0;
    data->numpad_dy    = 0;
    k_work_cancel_delayable(&data->numpad_timeout_work);
}

static void pmw3610_numpad_timeout_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, numpad_timeout_work);
    LOG_DBG("numpad timeout: reset");
    pmw3610_numpad_reset(data);
}

static int pmw3610_numpad_process(const struct device *dev, int16_t x, int16_t y) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int tick = config->numpad_tick;

    /* クールダウン中は入力を無視してアキュムレータもリセット */
    if (k_uptime_get() < data->numpad_cooldown_until) {
        data->numpad_dx = 0;
        data->numpad_dy = 0;
        return 0;
    }

    data->numpad_dx += x;
    data->numpad_dy += y;

    /* stroke検出: どちらかの軸がtickを超えたら方向確定 */
    if (abs(data->numpad_dx) < tick && abs(data->numpad_dy) < tick) {
        return 0;
    }

    /* 斜め入力を無視: 両軸が近い値なら捨てる */
    int adx = abs(data->numpad_dx);
    int ady = abs(data->numpad_dy);
    if (adx > tick / 2 && ady > tick / 2) {
        data->numpad_dx = 0;
        data->numpad_dy = 0;
        data->numpad_cooldown_until = k_uptime_get() + 180;
        return 0;
    }

    /* 主軸を判定 */
    bool horiz = abs(data->numpad_dx) >= abs(data->numpad_dy);
    int dir; /* 0=上 1=下 2=左 3=右 */
    if (horiz) {
        dir = data->numpad_dx > 0 ? 3 : 2;
    } else {
        dir = data->numpad_dy > 0 ? 1 : 0;
    }

    /* アキュムレータをリセット */
    data->numpad_dx = 0;
    data->numpad_dy = 0;

    if (data->numpad_state == 0) {
        /* 1ストローク目: 短いクールダウン（勢い防止のみ）→すぐ2ストローク受付 */
        data->numpad_cooldown_until = k_uptime_get() + 80;
        /* 全方向でgroup決定→2ストローク待ち */
        if (dir == 0)      data->numpad_group = 0;  /* 上: 1-3 */
        else if (dir == 2) data->numpad_group = 1;  /* 左: 4-6 */
        else if (dir == 3) data->numpad_group = 2;  /* 右: 7-9 */
        else               data->numpad_group = 3;  /* 下: 0/./Enter/BS */
        data->numpad_state = 1;
        k_work_schedule(&data->numpad_timeout_work,
                        K_MSEC(config->numpad_timeout_ms));
    } else {
        /* 2ストローク目後: 長めのクールダウンで誤入力防止 */
        data->numpad_cooldown_until = k_uptime_get() + 200;
        /* 2ストローク目 */
        if (data->numpad_group <= 2) {
            /* group 0-2: 左=col0 上=col1 右=col2 */
            int col = -1;
            if (dir == 2)      col = 0;  /* 左 */
            else if (dir == 0) col = 1;  /* 上 */
            else if (dir == 3) col = 2;  /* 右 */
            if (col >= 0) {
                pmw3610_numpad_send(dev, numpad_keys[data->numpad_group][col]);
            }
        } else {
            /* group 3 (下): 下=0 上=. 左=Enter 右=BS */
            int idx = -1;
            if (dir == 1)      idx = 0;  /* 下: 0 */
            else if (dir == 0) idx = 1;  /* 上: . */
            else if (dir == 2) idx = 2;  /* 左: Enter */
            else if (dir == 3) idx = 3;  /* 右: Backspace */
            if (idx >= 0) {
                pmw3610_numpad_send(dev, numpad_down_keys[idx]);
            }
        }
        pmw3610_numpad_reset(data);
    }
    return 0;
}

//////// Inertia scroll ////////

/* Compute decay% for current speed using 2-point linear interpolation.
 * speed < slow_spd  → decay_slow (stops quickly)
 * speed > fast_spd  → decay_fast (long glide)
 * in between        → linear blend
 */
static int pmw3610_adaptive_decay(const struct pixart_config *config, int32_t vx, int32_t vy) {
    int spd = MAX(abs(vx), abs(vy)) / 256;  /* convert from fixed-point to scroll-units/tick */
    if (spd >= config->scroll_inertia_fast_spd) {
        return config->scroll_inertia_decay_fast;
    }
    if (spd <= config->scroll_inertia_slow_spd) {
        return config->scroll_inertia_decay;
    }
    /* linear blend between slow and fast */
    int range = config->scroll_inertia_fast_spd - config->scroll_inertia_slow_spd;
    int t     = spd - config->scroll_inertia_slow_spd;
    return config->scroll_inertia_decay +
           (config->scroll_inertia_decay_fast - config->scroll_inertia_decay) * t / range;
}

static void pmw3610_inertia_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, inertia_work);
    const struct device *dev = data->dev;
    const struct pixart_config *config = dev->config;

    /* Speed-adaptive decay */
    int decay = pmw3610_adaptive_decay(config, data->inertia_vx, data->inertia_vy);
    data->inertia_vx = data->inertia_vx * decay / 100;
    data->inertia_vy = data->inertia_vy * decay / 100;

    int16_t vx = (int16_t)(data->inertia_vx / 256);
    int16_t vy = (int16_t)(data->inertia_vy / 256);
    bool emitted = false;

    if (vx != 0) {
        input_report_rel(dev, INPUT_REL_HWHEEL, vx, false, K_FOREVER);
        emitted = true;
    }
    if (vy != 0) {
        input_report_rel(dev, INPUT_REL_WHEEL, vy, false, K_FOREVER);
        emitted = true;
    }
    if (emitted) {
        input_report_rel(dev, INPUT_REL_WHEEL, 0, true, K_FOREVER);
    }

    /* Reschedule while velocity is still meaningful */
    if (abs(data->inertia_vx) >= 256 || abs(data->inertia_vy) >= 256) {
        k_work_schedule(&data->inertia_work, K_MSEC(config->scroll_inertia_tick_ms));
    }
}

//////// Pointer inertia ////////

static void pmw3610_pointer_inertia_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, pointer_inertia_work);
    const struct device *dev = data->dev;
    const struct pixart_config *config = dev->config;
    int spd = MAX(abs(data->pointer_inertia_vx), abs(data->pointer_inertia_vy)) / 256;
    int decay;
    if (spd >= config->pointer_inertia_fast_spd) {
        decay = config->pointer_inertia_decay_fast;
    } else if (spd <= config->pointer_inertia_slow_spd) {
        decay = config->pointer_inertia_decay;
    } else {
        int range = config->pointer_inertia_fast_spd - config->pointer_inertia_slow_spd;
        int t     = spd - config->pointer_inertia_slow_spd;
        decay = config->pointer_inertia_decay +
                (config->pointer_inertia_decay_fast - config->pointer_inertia_decay) * t / range;
    }
    data->pointer_inertia_vx = data->pointer_inertia_vx * decay / 100;
    data->pointer_inertia_vy = data->pointer_inertia_vy * decay / 100;
    int16_t vx = (int16_t)(data->pointer_inertia_vx / 256);
    int16_t vy = (int16_t)(data->pointer_inertia_vy / 256);
    if (vx != 0 || vy != 0) {
        bool have_x = vx != 0;
        bool have_y = vy != 0;
        if (have_x) input_report(dev, config->evt_type, config->x_input_code, vx, !have_y, K_FOREVER);
        if (have_y) input_report(dev, config->evt_type, config->y_input_code, vy, true, K_FOREVER);
    }
    if (abs(data->pointer_inertia_vx) >= 256 || abs(data->pointer_inertia_vy) >= 256) {
        k_work_schedule(&data->pointer_inertia_work, K_MSEC(config->pointer_inertia_tick_ms));
    } else {
        data->pointer_inertia_vx = 0;
        data->pointer_inertia_vy = 0;
    }
}

//////// Arrows auto-repeat ////////

static void pmw3610_arrows_repeat_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, arrows_repeat_work);
    const struct device *dev = data->dev;
    const struct pixart_config *config = dev->config;

    if (!data->arrows_repeating || data->arrows_last_key == 0) {
        return;
    }

    /* no-repeat layers: skip */
    if (pmw3610_layer_match(config->arrows_no_repeat_layers,
                             config->arrows_no_repeat_layers_len)) {
        data->arrows_repeating = false;
        data->arrows_last_key  = 0;
        return;
    }

    pmw3610_send_arrow_key(dev, data->arrows_last_key);
    k_work_schedule(&data->arrows_repeat_work, K_MSEC(config->arrows_repeat_rate_ms));
}

//////// Report data ////////

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int err = pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
    if (err) {
        return err;
    }

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

    int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
    int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);
    LOG_DBG("x/y: %d/%d", x, y);

#ifdef CONFIG_PMW3610_ALT_SMART_ALGORITHM
    int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8)
                    + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        pmw3610_write(dev, 0x32, 0x00);
        data->sw_smart_flag = false;
    }
    if (!data->sw_smart_flag && shutter > 45) {
        pmw3610_write(dev, 0x32, 0x80);
        data->sw_smart_flag = true;
    }
#endif

#if CONFIG_PMW3610_ALT_MOVEMENT_THRESHOLD > 0
    if (abs(x) < CONFIG_PMW3610_ALT_MOVEMENT_THRESHOLD) x = 0;
    if (abs(y) < CONFIG_PMW3610_ALT_MOVEMENT_THRESHOLD) y = 0;
    if (x == 0 && y == 0) return 0;
#endif

    /* ======================================================
     * SCROLL LAYER: emit WHEEL/HWHEEL instead of REL_X/Y
     * ====================================================== */
    if (pmw3610_layer_match(config->scroll_layers, config->scroll_layers_len)) {
        /* Cancel any ongoing inertia - user is scrolling again */
        if (config->scroll_inertia) {
            k_work_cancel_delayable(&data->inertia_work);
            data->inertia_vx = 0;
            data->inertia_vy = 0;
        }
        /* Cancel pointer inertia */
        if (config->pointer_inertia) {
            k_work_cancel_delayable(&data->pointer_inertia_work);
            data->pointer_inertia_vx = 0;
            data->pointer_inertia_vy = 0;
        }
        /* Cancel arrows auto-repeat so it doesn't fire while on scroll layer */
        k_work_cancel_delayable(&data->arrows_repeat_work);
        data->arrows_repeating = false;
        data->arrows_last_key  = 0;
        /* Abort swapper so Cmd+Tab doesn't fire unexpectedly on scroll layer */
        pmw3610_swapper_abort(data);
        /* Reset numpad state from a previous numpad session */
        pmw3610_numpad_reset(data);

        /* Flick detection: push raw delta into ring buffer */
        data->flick_hist_x[data->flick_idx] = x;
        data->flick_hist_y[data->flick_idx] = y;
        data->flick_idx = (data->flick_idx + 1) & 3;  /* mod 4 */

        data->scroll_dx += x;
        data->scroll_dy += y;
        data->snipe_dx = 0;
        data->snipe_dy = 0;
        data->arrows_dx = 0;
        data->arrows_dy = 0;
        data->arrows_one_shot_x_done = false;
        data->arrows_one_shot_y_done = false;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;

        bool scrolled = false;
        int16_t hwheel = 0;
        int16_t wheel  = 0;

        /* Scroll acceleration: factor *256 fixed-point */
        int32_t accel_factor = 256; /* 1.0 */
        if (config->scroll_accel && config->scroll_accel_threshold > 0) {
            int speed = MAX(abs(x), abs(y));
            int mult  = config->scroll_accel_max_mult > 1 ? config->scroll_accel_max_mult : 1;
            /* factor = 1 + (mult-1) * min(1, speed/threshold), in *256 */
            int32_t ratio = (speed * 256) / config->scroll_accel_threshold;
            if (ratio > 256) ratio = 256;
            accel_factor = 256 + ((mult - 1) * ratio);
        }

        if (abs(data->scroll_dx) >= CONFIG_PMW3610_ALT_SCROLL_TICK) {
            hwheel = (int16_t)((data->scroll_dx / CONFIG_PMW3610_ALT_SCROLL_TICK)
                               * accel_factor / 256);
            if (hwheel == 0) hwheel = data->scroll_dx > 0 ? 1 : -1;
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_SCROLL_X)
            hwheel = -hwheel;
#endif
            input_report_rel(dev, INPUT_REL_HWHEEL, hwheel, false, K_FOREVER);
            data->scroll_dx %= CONFIG_PMW3610_ALT_SCROLL_TICK;
            scrolled = true;
        }

        if (abs(data->scroll_dy) >= CONFIG_PMW3610_ALT_SCROLL_TICK) {
            wheel = (int16_t)((data->scroll_dy / CONFIG_PMW3610_ALT_SCROLL_TICK)
                              * accel_factor / 256);
            if (wheel == 0) wheel = data->scroll_dy > 0 ? 1 : -1;
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_SCROLL_Y)
            wheel = -wheel;
#endif
            input_report_rel(dev, INPUT_REL_WHEEL, wheel, false, K_FOREVER);
            data->scroll_dy %= CONFIG_PMW3610_ALT_SCROLL_TICK;
            scrolled = true;
        }

        if (scrolled) {
            /* sync event */
            input_report_rel(dev, INPUT_REL_WHEEL, 0, true, K_FOREVER);

            if (config->scroll_inertia) {
                /* Compute 4-sample average to detect flick */
                int32_t avg_x = ((int32_t)data->flick_hist_x[0] + data->flick_hist_x[1] +
                                  data->flick_hist_x[2] + data->flick_hist_x[3]) / 4;
                int32_t avg_y = ((int32_t)data->flick_hist_y[0] + data->flick_hist_y[1] +
                                  data->flick_hist_y[2] + data->flick_hist_y[3]) / 4;
                bool flick = (abs(avg_x) >= config->scroll_flick_threshold ||
                              abs(avg_y) >= config->scroll_flick_threshold);

                int32_t new_vx = (int32_t)hwheel * 256;
                int32_t new_vy = (int32_t)wheel  * 256;
                if (flick) {
                    new_vx = new_vx * config->scroll_flick_boost / 256;
                    new_vy = new_vy * config->scroll_flick_boost / 256;
                    LOG_DBG("Flick detected avg_x=%d avg_y=%d boost=%d",
                            (int)avg_x, (int)avg_y, config->scroll_flick_boost);
                }
                data->inertia_vx = new_vx;
                data->inertia_vy = new_vy;
                k_work_schedule(&data->inertia_work,
                                K_MSEC(config->scroll_inertia_tick_ms * 2));
            }
        }
        return 0;
    }

    /* ======================================================
     * SNIPE LAYER: divide movement, carry over remainder
     * arrows_alt_mode が ON の時は arrows 経路を優先するため SNIPE をスキップ
     * ====================================================== */
    if (!arrows_alt_mode_g &&
        pmw3610_layer_match(config->snipe_layers, config->snipe_layers_len)) {
        pmw3610_swapper_abort(data);
        k_work_cancel_delayable(&data->inertia_work);
        data->inertia_vx = 0;
        data->inertia_vy = 0;
        if (config->pointer_inertia) {
            k_work_cancel_delayable(&data->pointer_inertia_work);
            data->pointer_inertia_vx = 0;
            data->pointer_inertia_vy = 0;
        }
        k_work_cancel_delayable(&data->arrows_repeat_work);
        data->arrows_repeating = false;
        data->arrows_last_key  = 0;
        pmw3610_numpad_reset(data);
        data->scroll_dx = 0;
        data->scroll_dy = 0;
        data->arrows_dx = 0;
        data->arrows_dy = 0;
        data->arrows_one_shot_x_done = false;
        data->arrows_one_shot_y_done = false;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;

        data->snipe_dx += x;
        data->snipe_dy += y;

        int16_t rx = (int16_t)(data->snipe_dx / CONFIG_PMW3610_ALT_SNIPE_CPI_DIVIDOR);
        int16_t ry = (int16_t)(data->snipe_dy / CONFIG_PMW3610_ALT_SNIPE_CPI_DIVIDOR);
        data->snipe_dx %= CONFIG_PMW3610_ALT_SNIPE_CPI_DIVIDOR;
        data->snipe_dy %= CONFIG_PMW3610_ALT_SNIPE_CPI_DIVIDOR;

        bool have_x = rx != 0;
        bool have_y = ry != 0;

        if (have_x) {
            input_report(dev, config->evt_type, config->x_input_code, rx, !have_y, K_FOREVER);
        }
        if (have_y) {
            input_report(dev, config->evt_type, config->y_input_code, ry, true, K_FOREVER);
        }
        return 0;
    }

    /* ======================================================
     * ARROWS LAYER: convert movement to arrow key presses
     * Features: per-layer key mapping, diagonal, accel, auto-repeat
     * Profile layout: [layer, key_up, key_down, key_left, key_right]
     * ====================================================== */
    const uint16_t *profile = pmw3610_arrows_profile_match(config);
    if (arrows_alt_mode_g) {
        const uint16_t *alt_profile = pmw3610_arrows_alt_profile_match(config);
        if (alt_profile) {
            profile = alt_profile;
        }
    }
    if (profile != NULL) {
        /* profile[0]=layer, [1]=up, [2]=down, [3]=left, [4]=right, [5]=tick(0=global), [6]=remainder, [7]=no_diagonal, [8]=one_shot */
        uint16_t key_up    = profile[1];
        uint16_t key_down  = profile[2];
        uint16_t key_left  = profile[3];
        uint16_t key_right = profile[4];
        bool use_remainder  = (profile[6] != 0);
        bool no_diagonal    = (profile[7] != 0);
        bool one_shot       = (profile[8] != 0);

        k_work_cancel_delayable(&data->inertia_work);
        data->inertia_vx = 0;
        data->inertia_vy = 0;
        pmw3610_numpad_reset(data);
        data->scroll_dx = 0;
        data->scroll_dy = 0;
        data->snipe_dx  = 0;
        data->snipe_dy  = 0;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;

        /* 方向が変わったらアキュムレータをリセット（ボール反転による逆キー防止） */
        if (data->arrows_dx != 0 && x != 0 &&
            ((data->arrows_dx > 0) != (x > 0))) {
            data->arrows_dx = 0;
            data->arrows_one_shot_x_done = false;
        }
        if (data->arrows_dy != 0 && y != 0 &&
            ((data->arrows_dy > 0) != (y > 0))) {
            data->arrows_dy = 0;
            data->arrows_one_shot_y_done = false;
        }

        data->arrows_dx += x;
        data->arrows_dy += y;

        /* Acceleration: reduce effective tick based on raw input magnitude */
        int tick = (profile[5] != 0) ? (int)profile[5] : config->arrows_tick;
        if (config->arrows_accel) {
            int mag = MAX(abs(x), abs(y));
            if (mag >= config->arrows_accel_threshold) {
                int div = 1 + (mag - config->arrows_accel_threshold) *
                              (config->arrows_accel_max_div - 1) /
                              config->arrows_accel_threshold;
                div = MIN(div, config->arrows_accel_max_div);
                tick = MAX(1, tick / div);
            }
        }

        bool fired_x = false;
        bool fired_y = false;
        uint16_t emit_x = 0;
        uint16_t emit_y = 0;

        if (abs(data->arrows_dx) >= tick) {
            emit_x = data->arrows_dx > 0 ? key_right : key_left;
            data->arrows_dx = use_remainder ? (data->arrows_dx % tick) : 0;
            fired_x = true;
        }
        if (abs(data->arrows_dy) >= tick) {
            emit_y = data->arrows_dy > 0 ? key_down : key_up;
            data->arrows_dy = use_remainder ? (data->arrows_dy % tick) : 0;
            fired_y = true;
        }

        /* Diagonal suppression: per-profile flag or global arrows-diagonal */
        if ((no_diagonal || !config->arrows_diagonal) && fired_x && fired_y) {
            if (abs(x) >= abs(y)) {
                fired_y = false;
                emit_y = 0;
            } else {
                fired_x = false;
                emit_x = 0;
            }
        }

        /* One-shot suppression: once fired in a direction, ignore until direction reverses */
        if (one_shot) {
            if (fired_x && data->arrows_one_shot_x_done) {
                fired_x = false;
                emit_x = 0;
            }
            if (fired_y && data->arrows_one_shot_y_done) {
                fired_y = false;
                emit_y = 0;
            }
        }

        uint16_t primary_key = 0;
        if (fired_x && emit_x != 0) {
            pmw3610_send_arrow_key(dev, emit_x);
            if (one_shot) data->arrows_one_shot_x_done = true;
            primary_key = emit_x;
        }
        if (fired_y && emit_y != 0) {
            pmw3610_send_arrow_key(dev, emit_y);
            if (one_shot) data->arrows_one_shot_y_done = true;
            if (!primary_key) primary_key = emit_y;
        }

        /* Auto-repeat: track held direction */
        bool no_repeat = pmw3610_layer_match(config->arrows_no_repeat_layers,
                                              config->arrows_no_repeat_layers_len);
        if (!no_repeat && config->arrows_repeat_delay_ms > 0 && primary_key != 0) {
            if (primary_key != data->arrows_last_key) {
                k_work_cancel_delayable(&data->arrows_repeat_work);
                data->arrows_repeating  = false;
                data->arrows_last_key   = primary_key;
                k_work_schedule(&data->arrows_repeat_work,
                                K_MSEC(config->arrows_repeat_delay_ms));
            } else if (!data->arrows_repeating) {
                data->arrows_repeating = true;
            }
        } else if (primary_key == 0 && data->arrows_last_key != 0) {
            data->arrows_last_key  = 0;
            data->arrows_repeating = false;
            k_work_cancel_delayable(&data->arrows_repeat_work);
        }

        return 0;
    }

    /* ======================================================
     * NUMPAD LAYER: 2-stroke number input (0-9)
     * 1st stroke: 上=1-3グループ 左=4-6グループ 右=7-9グループ 下=0
     * 2nd stroke: 左=1/4/7  上=2/5/8  右=3/6/9
     * ====================================================== */
    if (pmw3610_layer_match(config->numpad_layers, config->numpad_layers_len)) {
        pmw3610_swapper_abort(data);
        k_work_cancel_delayable(&data->inertia_work);
        data->inertia_vx = 0;
        data->inertia_vy = 0;
        if (config->pointer_inertia) {
            k_work_cancel_delayable(&data->pointer_inertia_work);
            data->pointer_inertia_vx = 0;
            data->pointer_inertia_vy = 0;
        }
        k_work_cancel_delayable(&data->arrows_repeat_work);
        data->arrows_repeating = false;
        data->arrows_last_key  = 0;
        data->scroll_dx = 0;
        data->scroll_dy = 0;
        data->snipe_dx  = 0;
        data->snipe_dy  = 0;
        data->arrows_dx = 0;
        data->arrows_dy = 0;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;
        return pmw3610_numpad_process(dev, x, y);
    }

    /* ======================================================
     * NORMAL CURSOR
     * ====================================================== */
    pmw3610_swapper_abort(data);
    k_work_cancel_delayable(&data->inertia_work);
    data->inertia_vx = 0;
    data->inertia_vy = 0;
    k_work_cancel_delayable(&data->arrows_repeat_work);
    data->arrows_repeating = false;
    data->arrows_last_key  = 0;
    pmw3610_numpad_reset(data);
    data->snipe_dx  = 0;
    data->snipe_dy  = 0;
    data->arrows_dx = 0;
    data->arrows_dy = 0;

    if (config->pointer_inertia) {
        k_work_cancel_delayable(&data->pointer_inertia_work);
        data->pointer_inertia_vx = 0;
        data->pointer_inertia_vy = 0;
        data->pointer_flick_hist_x[data->pointer_flick_idx] = x;
        data->pointer_flick_hist_y[data->pointer_flick_idx] = y;
        data->pointer_flick_idx = (data->pointer_flick_idx + 1) & 3;
    }

    /* 2-sample accumulation: HID レポート頻度を 62.5Hz に抑えて BLE 圧迫を防止。
     * 250Hz ポーリング (Kconfig 既定) を 125Hz レポートへ落とすために書かれた処理で、
     * POLLING_RATE_125 と併用すると 62.5Hz まで二重に半減する。
     * no-sample-accumulation でスキップ可能 (既定は従来通り蓄積する)。 */
    if (!config->no_sample_accumulation) {
        int64_t curr_time = k_uptime_get();
        if (data->last_poll_time == 0 || curr_time - data->last_poll_time > 128) {
            data->last_poll_time = curr_time;
            data->last_x = x;
            data->last_y = y;
            return 0;
        } else {
            x += data->last_x;
            y += data->last_y;
            data->last_poll_time = 0;
            data->last_x = 0;
            data->last_y = 0;
        }
    }

    /* ======================================================
     * IIR FILTER: Motion noise reduction
     * Smooths raw sensor deltas to reduce jitter
     * ====================================================== */
    if (config->iir_filter_alpha > 0 && config->iir_filter_alpha <= 1024) {
        int32_t alpha = config->iir_filter_alpha;
        int32_t new_x = ((int32_t)x * alpha + data->filtered_x * (1024 - alpha)) / 1024;
        int32_t new_y = ((int32_t)y * alpha + data->filtered_y * (1024 - alpha)) / 1024;
        data->filtered_x = new_x;
        data->filtered_y = new_y;
        x = (int16_t)new_x;
        y = (int16_t)new_y;
    }

    /* ======================================================
     * SPEED-BASED CPI: Dynamically adjust sensor sensitivity
     * Low speed (trembling) → lower CPI for precision
     * High speed (flicking) → higher CPI for faster tracking
     * ====================================================== */
    if (config->speed_based_cpi) {
        int32_t speed = abs(x) + abs(y);
        uint32_t new_cpi = config->speed_cpi_medium;

        if (speed < config->speed_cpi_low_threshold) {
            new_cpi = config->speed_cpi_low;
        } else if (speed >= config->speed_cpi_high_threshold) {
            new_cpi = config->speed_cpi_high;
        }

        if (new_cpi != data->current_cpi) {
            data->current_cpi = new_cpi;
            pmw3610_set_cpi(dev, new_cpi, config->swap_xy, config->inv_x, config->inv_y);
        }

        data->recent_speed_x = x;
        data->recent_speed_y = y;
    }

    bool have_x = x != 0;
    bool have_y = y != 0;

    if (have_x) {
        input_report(dev, config->evt_type, config->x_input_code, x, !have_y, K_FOREVER);
    }
    if (have_y) {
        input_report(dev, config->evt_type, config->y_input_code, y, true, K_FOREVER);
    }

    if (config->pointer_inertia && (have_x || have_y)) {
        int32_t avg_x = ((int32_t)data->pointer_flick_hist_x[0] + data->pointer_flick_hist_x[1] +
                          data->pointer_flick_hist_x[2] + data->pointer_flick_hist_x[3]) / 4;
        int32_t avg_y = ((int32_t)data->pointer_flick_hist_y[0] + data->pointer_flick_hist_y[1] +
                          data->pointer_flick_hist_y[2] + data->pointer_flick_hist_y[3]) / 4;
        bool flick = (config->pointer_flick_threshold == 0) ||
                     (abs(avg_x) >= config->pointer_flick_threshold ||
                      abs(avg_y) >= config->pointer_flick_threshold);
        if (flick) {
            int32_t new_vx = (int32_t)x * 256 * config->pointer_flick_boost / 256;
            int32_t new_vy = (int32_t)y * 256 * config->pointer_flick_boost / 256;
            data->pointer_inertia_vx = new_vx;
            data->pointer_inertia_vy = new_vy;
            k_work_schedule(&data->pointer_inertia_work, K_MSEC(config->pointer_inertia_tick_ms * 2));
        }
    }

    return 0;
}

//////// Callbacks ////////

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    pmw3610_set_interrupt(dev, false);
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;
    pmw3610_report_data(dev);
    pmw3610_set_interrupt(dev, true);
}

static int pmw3610_init_irq(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));
    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }
    return err;
}

static int pmw3610_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("%s is not ready", config->spi.bus->name);
        return -ENODEV;
    }

    data->dev = dev;
    data->sw_smart_flag = false;
    data->scroll_dx = 0;
    data->scroll_dy = 0;
    data->snipe_dx     = 0;
    data->snipe_dy     = 0;
    data->arrows_dx    = 0;
    data->arrows_dy    = 0;
    data->arrows_last_key  = 0;
    data->arrows_repeating = false;
    data->numpad_dx    = 0;
    data->numpad_dy    = 0;
    data->numpad_state = 0;
    data->numpad_group = 0;
    data->inertia_vx   = 0;
    data->inertia_vy   = 0;
    data->flick_idx    = 0;
    memset(data->flick_hist_x, 0, sizeof(data->flick_hist_x));
    memset(data->flick_hist_y, 0, sizeof(data->flick_hist_y));
    data->last_poll_time = 0;
    data->last_x       = 0;
    data->last_y       = 0;

    /* IIR filter and speed-based CPI initialization */
    data->filtered_x   = 0;
    data->filtered_y   = 0;
    data->current_cpi  = config->cpi;
    data->recent_speed_x = 0;
    data->recent_speed_y = 0;

    data->arrows_swapper_state = SWAPPER_IDLE;
    data->arrows_swapper_key   = 0;

    k_work_init(&data->trigger_work, pmw3610_work_callback);
    k_work_init_delayable(&data->inertia_work, pmw3610_inertia_work_handler);
    k_work_init_delayable(&data->pointer_inertia_work, pmw3610_pointer_inertia_work_handler);
    k_work_init_delayable(&data->arrows_repeat_work, pmw3610_arrows_repeat_handler);
    k_work_init_delayable(&data->arrows_swapper_tab_work, pmw3610_swapper_tab_handler);
    k_work_init_delayable(&data->arrows_swapper_release_work, pmw3610_swapper_release_handler);
    k_work_init_delayable(&data->numpad_timeout_work, pmw3610_numpad_timeout_handler);

    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    k_work_init_delayable(&data->init_work, pmw3610_async_init);
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

//////// Sensor API ////////

static int pmw3610_alt_attr_set(const struct device *dev, enum sensor_channel chan,
                                enum sensor_attribute attr, const struct sensor_value *val) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    if (unlikely(chan != SENSOR_CHAN_ALL)) return -ENOTSUP;
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case PMW3610_ALT_ATTR_CPI:
        err = pmw3610_set_cpi(dev, PMW3610_SVALUE_TO_CPI(*val),
                              config->swap_xy, config->inv_x, config->inv_y);
        break;
    case PMW3610_ALT_ATTR_RUN_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST1_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST2_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST1_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST2_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST3_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;
    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }
    return err;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .attr_set = pmw3610_alt_attr_set,
};

//////// Device instantiation ////////

#define PMW3610_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
                          SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PMW3610_LAYERS_DEFINE(n, prop)                                              \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prop),                                    \
        (static const uint8_t prop##_##n[] = DT_INST_PROP(n, prop);),              \
        ())

#define PMW3610_LAYERS_PTR(n, prop)                                                 \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prop), (prop##_##n), (NULL))

#define PMW3610_LAYERS_LEN(n, prop)                                                 \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prop),                                    \
        (DT_INST_PROP_LEN(n, prop)), (0))

#define PMW3610_DEFINE(n)                                                           \
    PMW3610_LAYERS_DEFINE(n, scroll_layers)                                         \
    PMW3610_LAYERS_DEFINE(n, snipe_layers)                                          \
    PMW3610_LAYERS_DEFINE(n, numpad_layers)                                         \
    PMW3610_LAYERS_DEFINE(n, arrows_no_repeat_layers)                               \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, arrows_profiles),                         \
        (static const uint16_t arrows_profiles_##n[] = DT_INST_PROP(n, arrows_profiles);), \
        ())                                                                         \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, arrows_alt_profiles),                     \
        (static const uint16_t arrows_alt_profiles_##n[] = DT_INST_PROP(n, arrows_alt_profiles);), \
        ())                                                                         \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, cpi_layers),                              \
        (static const uint16_t cpi_layers_##n[] = DT_INST_PROP(n, cpi_layers);),    \
        ())                                                                         \
    static struct pixart_data data##n;                                              \
    static const struct pixart_config config##n = {                                 \
        .spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, 0),                       \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                           \
        .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                       \
        .swap_xy = DT_PROP(DT_DRV_INST(n), swap_xy),                               \
        .inv_x = DT_PROP(DT_DRV_INST(n), invert_x),                                \
        .inv_y = DT_PROP(DT_DRV_INST(n), invert_y),                                \
        .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                             \
        .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                     \
        .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                     \
        .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                       \
        .force_awake_4ms_mode = DT_PROP(DT_DRV_INST(n), force_awake_4ms_mode),     \
        .scroll_layers = PMW3610_LAYERS_PTR(n, scroll_layers),                     \
        .scroll_layers_len = PMW3610_LAYERS_LEN(n, scroll_layers),                 \
        .snipe_layers = PMW3610_LAYERS_PTR(n, snipe_layers),                       \
        .snipe_layers_len = PMW3610_LAYERS_LEN(n, snipe_layers),                   \
        .arrows_no_repeat_layers = PMW3610_LAYERS_PTR(n, arrows_no_repeat_layers), \
        .arrows_no_repeat_layers_len = PMW3610_LAYERS_LEN(n, arrows_no_repeat_layers), \
        .numpad_layers = PMW3610_LAYERS_PTR(n, numpad_layers),                     \
        .numpad_layers_len = PMW3610_LAYERS_LEN(n, numpad_layers),                 \
        .numpad_tick = DT_PROP_OR(DT_DRV_INST(n), numpad_tick, 15),               \
        .numpad_timeout_ms = DT_PROP_OR(DT_DRV_INST(n), numpad_timeout_ms, 1000), \
        .arrows_profiles = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, arrows_profiles),  \
            (arrows_profiles_##n), (NULL)),                                         \
        .arrows_profiles_count = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, arrows_profiles), \
            (DT_INST_PROP_LEN(n, arrows_profiles) / 9), (0)),                      \
        .arrows_alt_profiles = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, arrows_alt_profiles), \
            (arrows_alt_profiles_##n), (NULL)),                                     \
        .arrows_alt_profiles_count = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, arrows_alt_profiles), \
            (DT_INST_PROP_LEN(n, arrows_alt_profiles) / 9), (0)),                  \
        .cpi_layers = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, cpi_layers),            \
            (cpi_layers_##n), (NULL)),                                              \
        .cpi_layers_count = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, cpi_layers),      \
            (DT_INST_PROP_LEN(n, cpi_layers) / 2), (0)),                           \
        .arrows_tick             = DT_PROP_OR(DT_DRV_INST(n), arrows_tick, 10),             \
        .arrows_diagonal         = DT_PROP(DT_DRV_INST(n), arrows_diagonal),                 \
        .arrows_accel            = DT_PROP(DT_DRV_INST(n), arrows_accel),                    \
        .arrows_accel_max_div    = DT_PROP_OR(DT_DRV_INST(n), arrows_accel_max_div, 4),      \
        .arrows_accel_threshold  = DT_PROP_OR(DT_DRV_INST(n), arrows_accel_threshold, 20),  \
        .arrows_repeat_delay_ms  = DT_PROP_OR(DT_DRV_INST(n), arrows_repeat_delay_ms, 300), \
        .arrows_repeat_rate_ms   = DT_PROP_OR(DT_DRV_INST(n), arrows_repeat_rate_ms, 50),   \
        .scroll_inertia = DT_PROP(DT_DRV_INST(n), scroll_inertia),                        \
        .scroll_inertia_decay      = DT_PROP_OR(DT_DRV_INST(n), scroll_inertia_decay, 75),    \
        .scroll_inertia_decay_fast = DT_PROP_OR(DT_DRV_INST(n), scroll_inertia_decay_fast, 92), \
        .scroll_inertia_slow_spd   = DT_PROP_OR(DT_DRV_INST(n), scroll_inertia_slow_spd, 2),  \
        .scroll_inertia_fast_spd   = DT_PROP_OR(DT_DRV_INST(n), scroll_inertia_fast_spd, 6),  \
        .scroll_inertia_tick_ms    = DT_PROP_OR(DT_DRV_INST(n), scroll_inertia_tick_ms, 16),  \
        .scroll_flick_threshold    = DT_PROP_OR(DT_DRV_INST(n), scroll_flick_threshold, 6),   \
        .scroll_flick_boost        = DT_PROP_OR(DT_DRV_INST(n), scroll_flick_boost, 512),     \
        .scroll_accel              = DT_PROP(DT_DRV_INST(n), scroll_accel),                   \
        .scroll_accel_max_mult     = DT_PROP_OR(DT_DRV_INST(n), scroll_accel_max_mult, 4),    \
        .scroll_accel_threshold    = DT_PROP_OR(DT_DRV_INST(n), scroll_accel_threshold, 20),  \
        .pointer_inertia           = DT_PROP(DT_DRV_INST(n), pointer_inertia),                \
        .pointer_inertia_decay     = DT_PROP_OR(DT_DRV_INST(n), pointer_inertia_decay, 80),   \
        .pointer_inertia_decay_fast = DT_PROP_OR(DT_DRV_INST(n), pointer_inertia_decay_fast, 97), \
        .pointer_inertia_slow_spd  = DT_PROP_OR(DT_DRV_INST(n), pointer_inertia_slow_spd, 5), \
        .pointer_inertia_fast_spd  = DT_PROP_OR(DT_DRV_INST(n), pointer_inertia_fast_spd, 30), \
        .pointer_inertia_tick_ms   = DT_PROP_OR(DT_DRV_INST(n), pointer_inertia_tick_ms, 16), \
        .pointer_flick_threshold   = DT_PROP_OR(DT_DRV_INST(n), pointer_flick_threshold, 8),  \
        .pointer_flick_boost       = DT_PROP_OR(DT_DRV_INST(n), pointer_flick_boost, 512),    \
        .no_sample_accumulation    = DT_PROP(DT_DRV_INST(n), no_sample_accumulation),         \
        .iir_filter_alpha          = DT_PROP_OR(DT_DRV_INST(n), iir_filter_alpha, 614),       \
        .speed_based_cpi           = DT_PROP_OR(DT_DRV_INST(n), speed_based_cpi, 0),          \
        .speed_cpi_low_threshold   = DT_PROP_OR(DT_DRV_INST(n), speed_cpi_low_threshold, 5),  \
        .speed_cpi_high_threshold  = DT_PROP_OR(DT_DRV_INST(n), speed_cpi_high_threshold, 30),\
        .speed_cpi_low             = DT_PROP_OR(DT_DRV_INST(n), speed_cpi_low, 1200),         \
        .speed_cpi_medium          = DT_PROP_OR(DT_DRV_INST(n), speed_cpi_medium, 2200),      \
        .speed_cpi_high            = DT_PROP_OR(DT_DRV_INST(n), speed_cpi_high, 3200),        \
    };                                                                              \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n,             \
                          POST_KERNEL, CONFIG_INPUT_PMW3610_INIT_PRIORITY,          \
                          &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)

#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *pmw3610_devs[] = {
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610_alt, GET_PMW3610_DEV)
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }
    bool enable = state_ev->state == ZMK_ACTIVITY_ACTIVE ? 1 : 0;
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        pmw3610_set_performance(pmw3610_devs[i], enable);
    }
    return 0;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);

//////// CPI layers: per-layer CPI override ////////

static int pmw3610_apply_layer_cpi(const struct device *dev) {
    const struct pixart_config *cfg = dev->config;
    if (cfg->cpi_layers == NULL || cfg->cpi_layers_count == 0) return 0;
    uint16_t target_cpi = cfg->cpi;
    int target_layer = -1;
    /* Pick highest active matching layer (later entries win when tied highest) */
    for (size_t i = 0; i < cfg->cpi_layers_count; i++) {
        uint8_t layer = (uint8_t)cfg->cpi_layers[i * 2];
        uint16_t lcpi = cfg->cpi_layers[i * 2 + 1];
        if (zmk_keymap_layer_active(layer) && (int)layer > target_layer) {
            target_layer = layer;
            target_cpi = lcpi;
        }
    }
    return pmw3610_set_cpi(dev, target_cpi, cfg->swap_xy, cfg->inv_x, cfg->inv_y);
}

static int on_layer_state_for_cpi(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        pmw3610_apply_layer_cpi(pmw3610_devs[i]);
    }
    return 0;
}

ZMK_LISTENER(zmk_pmw3610_cpi_layers, on_layer_state_for_cpi);
ZMK_SUBSCRIPTION(zmk_pmw3610_cpi_layers, zmk_layer_state_changed);

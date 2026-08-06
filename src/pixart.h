#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pixart_data {
    const struct device          *dev;
    bool                         sw_smart_flag;

    struct gpio_callback         irq_gpio_cb;
    struct k_work                trigger_work;

    struct k_work_delayable      init_work;
    int                          async_init_step;

    bool                         ready;
    int                          err;

    /* scroll mode accumulation */
    int32_t                      scroll_dx;
    int32_t                      scroll_dy;

    /* snipe mode accumulation (remainder carry-over) */
    int32_t                      snipe_dx;
    int32_t                      snipe_dy;

    /* arrows mode accumulation */
    int32_t                      arrows_dx;
    int32_t                      arrows_dy;

    /* arrows auto-repeat */
    uint16_t                     arrows_last_key;
    bool                         arrows_repeating;
    struct k_work_delayable      arrows_repeat_work;

    /* inertia scroll: velocity in fixed-point (*256) */
    int32_t                      inertia_vx;
    int32_t                      inertia_vy;
    struct k_work_delayable      inertia_work;

    /* flick detection: ring buffer of recent raw deltas */
    int16_t                      flick_hist_x[4];
    int16_t                      flick_hist_y[4];
    uint8_t                      flick_idx;

    /* pointer inertia: velocity in fixed-point (*256) */
    int32_t                      pointer_inertia_vx;
    int32_t                      pointer_inertia_vy;
    struct k_work_delayable      pointer_inertia_work;

    /* pointer flick detection: ring buffer of recent raw deltas */
    int16_t                      pointer_flick_hist_x[4];
    int16_t                      pointer_flick_hist_y[4];
    uint8_t                      pointer_flick_idx;

    /* 2-sample accumulation (POLLING_RATE_125_SW equivalent) */
    int64_t                      last_poll_time;
    int16_t                      last_x;
    int16_t                      last_y;

    /* IIR filter state for noise reduction */
    int32_t                      filtered_x;
    int32_t                      filtered_y;

    /* Speed-based CPI tracking */
    uint32_t                     current_cpi;
    int32_t                      recent_speed_x;
    int32_t                      recent_speed_y;

    /* arrows swapper: modifier held across ticks (Cmd+Tab app switcher style) */
    enum {
        SWAPPER_IDLE = 0,
        SWAPPER_PENDING,
        SWAPPER_ACTIVE,
    }                            arrows_swapper_state;
    uint16_t                     arrows_swapper_key;
    struct k_work_delayable      arrows_swapper_tab_work;
    struct k_work_delayable      arrows_swapper_release_work;

    /* arrows one-shot: suppress repeated fires in same direction until direction reverses */
    bool                         arrows_one_shot_x_done;
    bool                         arrows_one_shot_y_done;

    /* numpad 2-stroke input state */
    int32_t                      numpad_dx;
    int32_t                      numpad_dy;
    uint8_t                      numpad_state;  /* 0=idle, 1=waiting 2nd stroke */
    uint8_t                      numpad_group;  /* 0=up(1-3) 1=left(4-6) 2=right(7-9) */
    int64_t                      numpad_cooldown_until; /* uptime ms: ignore input until this */
    struct k_work_delayable      numpad_timeout_work;
};

struct pixart_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
    bool swap_xy;
    bool inv_x;
    bool inv_y;
    uint8_t evt_type;
    uint8_t x_input_code;
    uint8_t y_input_code;
    bool force_awake;
    bool force_awake_4ms_mode;
    const uint8_t *scroll_layers;
    size_t scroll_layers_len;
    const uint8_t *snipe_layers;
    size_t snipe_layers_len;
    const uint8_t *arrows_no_repeat_layers;
    size_t arrows_no_repeat_layers_len;
    /* cpi-layers: flat array of [layer, cpi] uint16 pairs. Layer active → switch sensor CPI. */
    const uint16_t *cpi_layers;
    size_t cpi_layers_count;
    /* arrows profiles: flat array of [layer, key_up, key_down, key_left, key_right, ...] */
    const uint16_t *arrows_profiles;
    size_t arrows_profiles_count;  /* number of profiles (array_len / 5) */
    /* arrows-alt profiles: active only when arrows_alt_mode is set */
    const uint16_t *arrows_alt_profiles;
    size_t          arrows_alt_profiles_count;
    int arrows_tick;
    bool arrows_diagonal;         /* fire both axes on diagonal input */
    bool arrows_accel;            /* reduce tick as speed increases */
    int  arrows_accel_max_div;    /* max divisor for tick at high speed (default 4) */
    int  arrows_accel_threshold;  /* raw delta to start accel (default 20) */
    int  arrows_repeat_delay_ms;  /* initial auto-repeat delay ms (0=off, default 300) */
    int  arrows_repeat_rate_ms;   /* auto-repeat interval ms (default 50) */
    bool scroll_inertia;
    int  scroll_inertia_decay;       /* slow-speed decay % (default 75) */
    int  scroll_inertia_decay_fast;  /* high-speed decay % (default 92) */
    int  scroll_inertia_fast_spd;    /* speed threshold for fast decay (default 6) */
    int  scroll_inertia_slow_spd;    /* speed threshold for slow decay (default 2) */
    int  scroll_inertia_tick_ms;
    int  scroll_flick_threshold;     /* avg raw delta/sample to detect flick (default 6) */
    int  scroll_flick_boost;         /* velocity multiplier *256 on flick (default 512=2x) */
    bool scroll_accel;               /* amplify scroll delta based on speed */
    int  scroll_accel_max_mult;      /* max multiplier at high speed (default 4) */
    int  scroll_accel_threshold;     /* raw delta speed to reach max mult (default 20) */
    bool pointer_inertia;
    int  pointer_inertia_decay;      /* slow-speed decay % (default 80) */
    int  pointer_inertia_decay_fast; /* high-speed decay % (default 97) */
    int  pointer_inertia_fast_spd;   /* speed threshold for fast decay (default 30) */
    int  pointer_inertia_slow_spd;   /* speed threshold for slow decay (default 5) */
    int  pointer_inertia_tick_ms;    /* timer interval ms (default 16) */
    int  pointer_flick_threshold;    /* avg raw delta/sample to detect flick (default 8) */
    int  pointer_flick_boost;        /* velocity multiplier *256 on flick (default 512=2x) */
    /* 2-sample accumulation: halves HID report rate (written for 250Hz polling) */
    bool no_sample_accumulation;     /* skip pairing; report every sample (default false) */
    /* IIR filter for motion noise reduction (fixed-point *1024) */
    int  iir_filter_alpha;           /* filter coefficient (0-1024, default 614 ≈ 0.6) */
    /* Speed-based CPI: dynamically adjust CPI based on movement speed */
    bool speed_based_cpi;            /* enable speed-based CPI adjustment */
    int  speed_cpi_low_threshold;    /* raw delta threshold for low speed (default 5) */
    int  speed_cpi_high_threshold;   /* raw delta threshold for high speed (default 30) */
    uint32_t speed_cpi_low;          /* CPI for slow movements (default 1200) */
    uint32_t speed_cpi_medium;       /* CPI for medium movements (default 2200) */
    uint32_t speed_cpi_high;         /* CPI for fast movements (default 3200) */
    const uint8_t *numpad_layers;
    size_t numpad_layers_len;
    int  numpad_tick;          /* stroke threshold (default 15) */
    int  numpad_timeout_ms;    /* 2nd stroke timeout ms (default 1000) */
};

void pmw3610_arrows_alt_mode_set(bool val);

#ifdef __cplusplus
}
#endif

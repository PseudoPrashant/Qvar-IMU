/******************************************************************************
 * @file    main.c
 * @brief   Raw & 5-Sample Peak-to-Peak Envelope Telemetry Firmware for ISM330BX QVAR.
 *          Features:
 *            - Sensor ODR = 240 Hz (ISM330BX_XL_ODR_AT_240Hz)
 *            - Polling interval = 4 ms (250 Hz, perfectly matching 240 Hz ODR)
 *            - Hardware HPF = 1 (on-chip sub-Hz baseline wander elimination)
 *            - 5-Sample Sliding Window Peak-to-Peak Envelope Extractor:
 *                Activity[n] = max(x[n..n-4]) - min(x[n..n-4])
 *                Spans 20.0 ms = exactly 1 full cycle of 50 Hz (and >1 cycle of 60 Hz).
 *            - Dual-channel telemetry streaming: Q1 (raw) and Q1_ACT (envelope activity)
 ******************************************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "imu.h"

#define I2C_MASTER_SCL_IO           22      
#define I2C_MASTER_SDA_IO           21      
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000

/* Hardware AFE configuration */
#define RAW_QVAR_ZIN                ISM330BX_235MOhm   /* 235 MOhm input impedance (lowest, maximum noise immunity) */
#define RAW_SAMPLE_PERIOD_MS        5u                 /* 5 ms = 200 Hz sampling rate */

/* 5-Sample Sliding Window Peak-to-Peak Envelope Extractor:
 * Activity[n] = max(x[n..n-4]) - min(x[n..n-4])
 * Spans 5 * 4 ms = 20.0 ms (exactly 1 period of 50 Hz powerline hum).
 * Captures the absolute peak-to-peak amplitude regardless of phase alignment.
 */
#define ENVELOPE_WINDOW_SIZE        4u                 /* 4 samples * 5 ms = 20.0 ms (1 full cycle of 50 Hz mains) */

typedef struct {
    int16_t window[ENVELOPE_WINDOW_SIZE];
    uint8_t count;
    uint8_t head;
} qvar_envelope_extractor_t;

static qvar_envelope_extractor_t sEnvQ1 = {0};

static inline int32_t qvar_envelope_extractor_update(qvar_envelope_extractor_t *env, int16_t sample) {
    env->window[env->head] = sample;
    env->head = (env->head + 1u) % ENVELOPE_WINDOW_SIZE;
    if (env->count < ENVELOPE_WINDOW_SIZE) {
        env->count++;
    }

    int16_t min_val = env->window[0];
    int16_t max_val = env->window[0];
    for (uint8_t i = 1u; i < env->count; i++) {
        if (env->window[i] < min_val) min_val = env->window[i];
        if (env->window[i] > max_val) max_val = env->window[i];
    }
    return (int32_t)max_val - (int32_t)min_val;
}

/* 4-Stage Robust Tap Detection Engine with Adaptive Baseline Envelope Filter */
#define TAP_PRESS_DELTA_LSB       1100L   /* Contact trigger: baseline + 1100 LSB */
#define TAP_RELEASE_DELTA_LSB     600L    /* Release trigger: baseline + 600 LSB */
#define TAP_MIN_PEAK_ACT_LSB      3300L   /* Minimum activity peak required for tap */
#define TAP_MIN_DUR_SAMPLES       3u      /* 15 ms min duration at 200 Hz */
#define TAP_MAX_DUR_SAMPLES       70u     /* 350 ms max duration (rejects holds) */
#define TAP_COOLDOWN_SAMPLES      25u     /* 125 ms refractory lockout */
#define TAP_SQUELCH_TIMER_SAMPLES 50u     /* 250 ms squelch on disturbance */
#define TAP_SQUELCH_QUIET_SAMPLES 15u     /* 75 ms continuous calm required */
#define TAP_BIPOLAR_MIN_RAW       (-500)  /* Must plunge below -500 LSB */
#define TAP_BIPOLAR_MAX_RAW       (+500)  /* Must crest above +500 LSB */
#define TAP_ALPHA_RISE            0.99917f  /* Ultra-slow rise during contact */
#define TAP_ALPHA_FALL            0.9875f  /* Fast fall when calm */

typedef enum {
    TAP_STATE_IDLE = 0,
    TAP_STATE_CONTACT,
    TAP_STATE_COOLDOWN,
    TAP_STATE_SQUELCH
} tap_state_t;

typedef struct {
    tap_state_t state;
    uint32_t start_idx;
    uint32_t peak_idx;
    int32_t peak_act;
    int16_t min_raw;
    int16_t max_raw;
    uint16_t timer;
    uint16_t quiet_counter;
    uint32_t tap_count;
    float baseline_act;
    uint8_t baseline_init;
} robust_tap_detector_t;

static robust_tap_detector_t sTapDetector = {0};

static inline uint8_t robust_tap_detector_update(robust_tap_detector_t *det,
                                                uint32_t sample_idx,
                                                int16_t raw_val,
                                                int32_t act_val)
{
    uint8_t tap_confirmed = 0u;

    if (det->baseline_init == 0u) {
        det->baseline_act = (float)act_val;
        det->baseline_init = 1u;
    }

    /* Asymmetric EMA baseline tracking */
    if (det->state == TAP_STATE_IDLE) {
        if ((float)act_val > det->baseline_act) {
            det->baseline_act = TAP_ALPHA_RISE * det->baseline_act + (1.0f - TAP_ALPHA_RISE) * (float)act_val;
        } else {
            det->baseline_act = TAP_ALPHA_FALL * det->baseline_act + (1.0f - TAP_ALPHA_FALL) * (float)act_val;
        }
    } else if (det->state == TAP_STATE_SQUELCH) {
        det->baseline_act = 0.995f * det->baseline_act + 0.005f * (float)act_val;
    }

    int32_t th_press = (int32_t)det->baseline_act + TAP_PRESS_DELTA_LSB;
    int32_t th_release = (int32_t)det->baseline_act + TAP_RELEASE_DELTA_LSB;

    if (act_val < th_release) {
        det->quiet_counter++;
    } else {
        det->quiet_counter = 0u;
    }

    switch (det->state)
    {
    case TAP_STATE_IDLE:
        if (act_val >= th_press) {
            det->state = TAP_STATE_CONTACT;
            det->start_idx = sample_idx;
            det->peak_idx = sample_idx;
            det->peak_act = act_val;
            det->min_raw = raw_val;
            det->max_raw = raw_val;
        }
        break;

    case TAP_STATE_CONTACT: {
        uint32_t dur = sample_idx - det->start_idx;

        if (act_val > det->peak_act) {
            det->peak_act = act_val;
            det->peak_idx = sample_idx;
        }
        if (raw_val < det->min_raw) det->min_raw = raw_val;
        if (raw_val > det->max_raw) det->max_raw = raw_val;

        if (dur > TAP_MAX_DUR_SAMPLES) {
            det->state = TAP_STATE_SQUELCH;
            det->timer = TAP_SQUELCH_TIMER_SAMPLES;
            break;
        }

        if (act_val < th_release) {
            if (dur < TAP_MIN_DUR_SAMPLES) {
                det->state = TAP_STATE_IDLE;
            } else if (dur <= TAP_MAX_DUR_SAMPLES) {
                uint8_t is_bipolar = (det->min_raw <= TAP_BIPOLAR_MIN_RAW && det->max_raw >= TAP_BIPOLAR_MAX_RAW) ? 1u : 0u;
                uint8_t is_strong = (det->peak_act >= TAP_MIN_PEAK_ACT_LSB) ? 1u : 0u;

                if (is_bipolar != 0u && is_strong != 0u) {
                    det->tap_count++;
                    tap_confirmed = 1u;
                    det->state = TAP_STATE_COOLDOWN;
                    det->timer = TAP_COOLDOWN_SAMPLES;
                } else {
                    if (is_strong == 0u && is_bipolar != 0u) {
                        det->state = TAP_STATE_IDLE;
                    } else {
                        det->state = TAP_STATE_SQUELCH;
                        det->timer = TAP_SQUELCH_TIMER_SAMPLES;
                    }
                }
            }
        }
        break;
    }

    case TAP_STATE_COOLDOWN:
        if (det->timer > 0u) det->timer--;
        if (det->timer == 0u) {
            det->state = TAP_STATE_IDLE;
            if (act_val >= th_press) {
                det->state = TAP_STATE_CONTACT;
                det->start_idx = sample_idx;
                det->peak_idx = sample_idx;
                det->peak_act = act_val;
                det->min_raw = raw_val;
                det->max_raw = raw_val;
            }
        }
        break;

    case TAP_STATE_SQUELCH:
        if (det->timer > 0u) det->timer--;
        if (act_val >= th_release) {
            det->timer = TAP_SQUELCH_TIMER_SAMPLES;
        }
        if (det->timer == 0u && det->quiet_counter >= TAP_SQUELCH_QUIET_SAMPLES) {
            det->state = TAP_STATE_IDLE;
        }
        break;
    }

    return tap_confirmed;
}

static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main(void) {
    i2c_master_init();
    printf("\r\n=======================================================\r\n");
    printf("   ISM330BX 200 HZ RAW & ROBUST TAP DETECTOR FIRMWARE\r\n");
    printf("=======================================================\r\n");

    if (imu_init() != 0) {
        printf("[!] Error: IMU initialization failed. Check wiring (GPIO 21 SDA, GPIO 22 SCL).\r\n");
        return;
    }
    printf("[+] ISM330BX IMU initialized successfully (WHO_AM_I: 0x71).\r\n");

    if (imu_raw_qvar_start(RAW_QVAR_ZIN) != 0) {
        printf("[!] Error: Failed to start QVAR analog front-end.\r\n");
        return;
    }
    printf("[+] Raw QVAR AFE started (Zin=235M, HPF=1, ODR=240 Hz).\r\n");
    printf("[+] 4-Sample P2P Envelope & 4-Stage Robust Tap Detector active (200 Hz).\r\n");
    printf("[+] Streaming dual telemetry [Q1 (raw), Q1_ACT (envelope)] to UART0...\r\n\r\n");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(RAW_SAMPLE_PERIOD_MS);
    uint32_t sample_idx = 0;
    int16_t q1_raw = 0;

    while (1) {
        if (imu_raw_qvar_read(&q1_raw) == 0) {
            sample_idx++;
            int32_t q1_act = qvar_envelope_extractor_update(&sEnvQ1, q1_raw);

            /* 4-Stage Robust Tap Detector check */
            if (robust_tap_detector_update(&sTapDetector, sample_idx, q1_raw, q1_act) != 0u) {
                uint32_t dur_ms = (sample_idx >= sTapDetector.start_idx) ?
                                  (sample_idx - sTapDetector.start_idx) * RAW_SAMPLE_PERIOD_MS : 0u;
                printf("[IMU QVAR TAP] #%lu dur=%lu ms peak=%ld LSB base=%.0f LSB\r\n",
                       (unsigned long)sTapDetector.tap_count,
                       (unsigned long)dur_ms,
                       (long)sTapDetector.peak_act,
                       sTapDetector.baseline_act);
            }

            /* Streamlined dual telemetry format to eliminate UART buffer saturation at 200 Hz */
            printf("[IMU QVAR RAW] Q1=%d Q1_ACT=%ld #%lu\r\n",
                   (int)q1_raw, (long)q1_act, (unsigned long)sample_idx);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

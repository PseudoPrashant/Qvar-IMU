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
    printf("   ISM330BX 240 HZ RAW & ENVELOPE TELEMETRY FIRMWARE\r\n");
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
    printf("[+] 5-Sample Peak-to-Peak Envelope Extractor active (Window=20ms @ 250 Hz).\r\n");
    printf("[+] Streaming dual telemetry [Q1 (raw), Q1_ACT (envelope)] to UART0...\r\n\r\n");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(RAW_SAMPLE_PERIOD_MS);
    uint32_t sample_idx = 0;
    int16_t q1_raw = 0;

    while (1) {
        if (imu_raw_qvar_read(&q1_raw) == 0) {
            sample_idx++;
            int32_t q1_act = qvar_envelope_extractor_update(&sEnvQ1, q1_raw);

            /* Streamlined dual telemetry format to eliminate UART buffer saturation at 200 Hz */
            printf("[IMU QVAR RAW] Q1=%d Q1_ACT=%ld #%lu\r\n",
                   (int)q1_raw, (long)q1_act, (unsigned long)sample_idx);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

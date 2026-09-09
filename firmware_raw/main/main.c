/******************************************************************************
 * @file    main.c
 * @brief   Pure Raw Telemetry Firmware for ISM330BX QVAR.
 *          ZERO data processing: no hardware filters (HPF=0, LPF=0),
 *          no FIR filter, no baseline tracking, no peak detector,
 *          no debouncing, no button/wear state machines.
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
#define RAW_QVAR_ZIN                ISM330BX_2400MOhm  /* 2400 MOhm (maximum sensitivity) */
#define RAW_SAMPLE_PERIOD_MS        10u                /* 10 ms = 100 Hz sampling rate */

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
    printf("   ISM330BX PURE RAW DATA FIRMWARE (ZERO FILTERS)\r\n");
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
    printf("[+] Raw QVAR AFE started (Zin=2400M, Filters=NONE [HPF=0, LPF=0], 100 Hz).\r\n");
    printf("[+] Streaming raw 16-bit register samples directly to UART0...\r\n\r\n");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(RAW_SAMPLE_PERIOD_MS);
    uint32_t sample_idx = 0;
    int16_t q1_raw = 0;

    while (1) {
        if (imu_raw_qvar_read(&q1_raw) == 0) {
            sample_idx++;
            /* ST AN5755 voltage scaling: 78 LSB / mV */
            float mv = (float)q1_raw / 78.0f;

            /* Pure raw reading output directly to serial port */
            printf("[IMU QVAR RAW] Q1=%d Q2=NA (%.2f mV) #%lu\r\n",
                   (int)q1_raw, mv, (unsigned long)sample_idx);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "imu.h"

#define I2C_MASTER_SCL_IO           22      
#define I2C_MASTER_SDA_IO           21      
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000

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
    printf("I2C Initialized.\n");

    if (imu_init() != 0) {
        printf("IMU Initialization Failed. Check wiring and address.\n");
        return;
    }

    printf("Starting QVAR Polling Task...\n");
    while (1) {
        imu_qvar_app_task();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
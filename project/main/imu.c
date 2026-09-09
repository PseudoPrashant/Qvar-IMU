#include "imu.h"
#include "imu_internal.h"
#include "ism330bx_reg.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define I2C_MASTER_NUM I2C_NUM_0
#define ISM330BX_I2C_ADDR 0x6B

static stmdev_ctx_t sImuCtx;
static uint8_t sStarted = 0;

static int32_t imu_platform_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len) {
    uint8_t *write_buf = malloc(len + 1);
    if (!write_buf) return -1;
    write_buf[0] = reg;
    memcpy(&write_buf[1], buf, len);
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, ISM330BX_I2C_ADDR, write_buf, len + 1, pdMS_TO_TICKS(100));
    free(write_buf);
    return (err == ESP_OK) ? 0 : -1;
}

static int32_t imu_platform_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len) {
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, ISM330BX_I2C_ADDR, &reg, 1, buf, len, pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? 0 : -1;
}

static void imu_platform_delay(uint32_t millisec) {
    vTaskDelay(pdMS_TO_TICKS(millisec));
}

stmdev_ctx_t *imu_internal_ctx(void) {
    return &sImuCtx;
}

uint8_t imu_is_started(void) {
    return sStarted;
}

// Minimal initialization tailored for I2C
int imu_init(void) {
    uint8_t whoAmI = 0;

    sImuCtx.write_reg = imu_platform_write;
    sImuCtx.read_reg = imu_platform_read;
    sImuCtx.mdelay = imu_platform_delay;
    sImuCtx.handle = NULL; 

    if (ism330bx_device_id_get(&sImuCtx, &whoAmI) != 0 || whoAmI != ISM330BX_ID) {
        return -1; // Failed to find device
    }

    // Initialize required baseline registers
    ism330bx_auto_increment_set(&sImuCtx, PROPERTY_ENABLE);
    ism330bx_block_data_update_set(&sImuCtx, PROPERTY_ENABLE);

    sStarted = 1;
    return 0; // Success
}

// Stubs for power management required by qvar.c
int imu_internal_qvar_power_down_motion(void) {
    ism330bx_xl_data_rate_set(&sImuCtx, ISM330BX_XL_ODR_OFF);
    ism330bx_gy_data_rate_set(&sImuCtx, ISM330BX_GY_ODR_OFF);
    return 0;
}

void imu_internal_qvar_restore_motion_if_needed(void) {
    // The ISM330BX strictly requires the accelerometer to be in 
    // High-Performance mode for the Qvar channel to function.
    
    // 1. Set Accelerometer full scale to 2g
    ism330bx_xl_full_scale_set(&sImuCtx, ISM330BX_2g);
    
    // 2. Set Accelerometer to High-Performance mode
    ism330bx_xl_mode_set(&sImuCtx, ISM330BX_XL_HIGH_PERFORMANCE_MD);
    
    // 3. Turn on the Accelerometer at 240Hz
    ism330bx_xl_data_rate_set(&sImuCtx, ISM330BX_XL_ODR_AT_240Hz);
}

static uint8_t sInt2Pending = 0;

uint8_t imu_internal_get_int2_pending(void) {
    return sInt2Pending;
}

void imu_internal_clear_int2_pending(void) {
    sInt2Pending = 0;
}
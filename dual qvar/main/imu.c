#include "imu.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define I2C_MASTER_NUM       I2C_NUM_0
#define ISM330BX_I2C_ADDR    0x6B

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

stmdev_ctx_t *imu_get_ctx(void) {
    return &sImuCtx;
}

int imu_init(void) {
    uint8_t whoAmI = 0;

    sImuCtx.write_reg = imu_platform_write;
    sImuCtx.read_reg  = imu_platform_read;
    sImuCtx.mdelay    = imu_platform_delay;
    sImuCtx.handle    = NULL;

    if (ism330bx_device_id_get(&sImuCtx, &whoAmI) != 0 || whoAmI != ISM330BX_ID) {
        printf("[!] Failed to read WHO_AM_I (read: 0x%02X, expected: 0x%02X)\r\n", whoAmI, ISM330BX_ID);
        return -1;
    }

    // Auto-increment register address on multi-byte transfers & Block Data Update
    ism330bx_auto_increment_set(&sImuCtx, PROPERTY_ENABLE);
    ism330bx_block_data_update_set(&sImuCtx, PROPERTY_ENABLE);

    sStarted = 1;
    return 0;
}

int imu_raw_qvar_start(ism330bx_ah_qvar_zin_t zin) {
    if (!sStarted) return -1;

    // Per ST AN5755: Power down XL and GY before setting AH_QVAR_EN
    ism330bx_xl_data_rate_set(&sImuCtx, ISM330BX_XL_ODR_OFF);
    ism330bx_gy_data_rate_set(&sImuCtx, ISM330BX_GY_ODR_OFF);

    // Hardware filter configuration: HPF enabled to eliminate DC drift, LPF disabled
    ism330bx_filt_ah_qvar_conf_t filter = {0};
    filter.hpf = 1u;
    filter.lpf = 0u;
    ism330bx_filt_ah_qvar_conf_set(&sImuCtx, filter);

    // Set input impedance Zin (e.g. 2400 MOhm, 730 MOhm, 300 MOhm, 235 MOhm)
    ism330bx_ah_qvar_zin_set(&sImuCtx, zin);

    // Select Q1 and Q2 electrodes for differential mode
    ism330bx_ah_qvar_mode_t mode = {0};
    mode.ah_qvar1_en = 1u;
    mode.ah_qvar2_en = 1u; // Enable Q2 for differential sensing
    mode.swaps = 0u;
    ism330bx_ah_qvar_mode_set(&sImuCtx, mode);

    // Restore Accelerometer in High-Performance mode at 120 Hz (required by QVAR engine)
    ism330bx_xl_full_scale_set(&sImuCtx, ISM330BX_2g);
    ism330bx_xl_mode_set(&sImuCtx, ISM330BX_XL_HIGH_PERFORMANCE_MD);
    ism330bx_xl_data_rate_set(&sImuCtx, ISM330BX_XL_ODR_AT_240Hz);

    return 0;
}

int imu_raw_qvar_read(int16_t *raw_out) {
    if (!raw_out || !sStarted) return -1;
    return (ism330bx_ah_qvar_raw_get(&sImuCtx, raw_out) == 0) ? 0 : -1;
}

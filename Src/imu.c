/******************************************************************************
 * @file    imu.c
 * @brief   ISM330BX IMU driver wrapper for STM32 SPI2.
 ******************************************************************************/

#include "../Inc/imu.h"

#include "../Inc/imu_internal.h"
#include "../Inc/ism330bx_reg.h"
#include "../../SPI_Bus/Inc/spi_bus.h"
#include "main.h"
#include <stddef.h>
#include <stdio.h>

#define IMU_SPI_TIMEOUT_MS 100u
#define IMU_PRINT_PERIOD_MS 500u
#define IMU_WAKE_DURATION_MAX 3u

static spi_bus_device_t sImuSpi;
static stmdev_ctx_t sImuCtx;
static uint8_t sStarted;
static volatile uint8_t sInt1Pending;
static volatile uint8_t sInt2Pending;
static volatile uint32_t sInt1IrqCount;
static volatile uint32_t sInt2IrqCount;
static imu_feature_config_t sFeatureConfig;
static imu_motion_config_t sMotionConfig;
static uint8_t sMotionConfigValid;
static uint8_t sQvarMotionPowerDownActive;

extern SPI_HandleTypeDef hspi2;

/* Write IMU registers through the shared SPI bus adapter used by ST driver. */
static int32_t imu_platform_write(void *handle,
                                  uint8_t reg,
                                  const uint8_t *buf,
                                  uint16_t len)
{
    const spi_bus_device_t *device = (const spi_bus_device_t *)handle;

    return (spi_bus_reg_write(device, reg, buf, len) == HAL_OK) ? 0 : -1;
}

/* Read IMU registers through the shared SPI bus adapter used by ST driver. */
static int32_t imu_platform_read(void *handle,
                                 uint8_t reg,
                                 uint8_t *buf,
                                 uint16_t len)
{
    const spi_bus_device_t *device = (const spi_bus_device_t *)handle;

    return (spi_bus_reg_read(device, reg, buf, len) == HAL_OK) ? 0 : -1;
}

/* Provide millisecond delay callback for the ST ISM330BX register driver. */
static void imu_platform_delay(uint32_t millisec)
{
    HAL_Delay(millisec);
}

/* Return the initialized ST register-driver context used by feature modules. */
stmdev_ctx_t *imu_internal_ctx(void)
{
    return &sImuCtx;
}

/* Enable or disable only the wake-up route bits on one interrupt line route. */
static void imu_apply_wakeup_route(ism330bx_pin_int_route_t *route,
                                   uint8_t enable)
{
    route->six_d = 0u;
    route->double_tap = 0u;
    route->free_fall = 0u;
    route->wake_up = enable;
    route->single_tap = 0u;
    route->sleep_change = 0u;
    route->sleep_status = 0u;
}

/* Convert wrapper accelerometer full-scale selection to ST driver enum. */
static ism330bx_xl_full_scale_t imu_accel_fs_to_driver(imu_accel_fs_t fs)
{
    switch (fs)
    {
    case IMU_ACCEL_FS_4G:
        return ISM330BX_4g;

    case IMU_ACCEL_FS_8G:
        return ISM330BX_8g;

    case IMU_ACCEL_FS_2G:
    default:
        return ISM330BX_2g;
    }
}

/* Convert wrapper gyroscope full-scale selection to ST driver enum. */
static ism330bx_gy_full_scale_t imu_gyro_fs_to_driver(imu_gyro_fs_t fs)
{
    switch (fs)
    {
    case IMU_GYRO_FS_125DPS:
        return ISM330BX_125dps;

    case IMU_GYRO_FS_500DPS:
        return ISM330BX_500dps;

    case IMU_GYRO_FS_1000DPS:
        return ISM330BX_1000dps;

    case IMU_GYRO_FS_2000DPS:
        return ISM330BX_2000dps;

    case IMU_GYRO_FS_4000DPS:
        return ISM330BX_4000dps;

    case IMU_GYRO_FS_250DPS:
    default:
        return ISM330BX_250dps;
    }
}

/* Convert wrapper accelerometer output data rate to ST driver enum. */
static ism330bx_xl_data_rate_t imu_accel_odr_to_driver(imu_accel_odr_t odr)
{
    switch (odr)
    {
    case IMU_ACCEL_ODR_OFF:
        return ISM330BX_XL_ODR_OFF;
    case IMU_ACCEL_ODR_1HZ875:
        return ISM330BX_XL_ODR_AT_1Hz875;
    case IMU_ACCEL_ODR_7HZ5:
        return ISM330BX_XL_ODR_AT_7Hz5;
    case IMU_ACCEL_ODR_15HZ:
        return ISM330BX_XL_ODR_AT_15Hz;
    case IMU_ACCEL_ODR_30HZ:
        return ISM330BX_XL_ODR_AT_30Hz;
    case IMU_ACCEL_ODR_60HZ:
        return ISM330BX_XL_ODR_AT_60Hz;
    case IMU_ACCEL_ODR_240HZ:
        return ISM330BX_XL_ODR_AT_240Hz;
    case IMU_ACCEL_ODR_480HZ:
        return ISM330BX_XL_ODR_AT_480Hz;
    case IMU_ACCEL_ODR_960HZ:
        return ISM330BX_XL_ODR_AT_960Hz;
    case IMU_ACCEL_ODR_1920HZ:
        return ISM330BX_XL_ODR_AT_1920Hz;
    case IMU_ACCEL_ODR_3840HZ:
        return ISM330BX_XL_ODR_AT_3840Hz;
    case IMU_ACCEL_ODR_120HZ:
    default:
        return ISM330BX_XL_ODR_AT_120Hz;
    }
}

/* Convert wrapper gyroscope output data rate to ST driver enum. */
static ism330bx_gy_data_rate_t imu_gyro_odr_to_driver(imu_gyro_odr_t odr)
{
    switch (odr)
    {
    case IMU_GYRO_ODR_OFF:
        return ISM330BX_GY_ODR_OFF;
    case IMU_GYRO_ODR_7HZ5:
        return ISM330BX_GY_ODR_AT_7Hz5;
    case IMU_GYRO_ODR_15HZ:
        return ISM330BX_GY_ODR_AT_15Hz;
    case IMU_GYRO_ODR_30HZ:
        return ISM330BX_GY_ODR_AT_30Hz;
    case IMU_GYRO_ODR_60HZ:
        return ISM330BX_GY_ODR_AT_60Hz;
    case IMU_GYRO_ODR_240HZ:
        return ISM330BX_GY_ODR_AT_240Hz;
    case IMU_GYRO_ODR_480HZ:
        return ISM330BX_GY_ODR_AT_480Hz;
    case IMU_GYRO_ODR_960HZ:
        return ISM330BX_GY_ODR_AT_960Hz;
    case IMU_GYRO_ODR_1920HZ:
        return ISM330BX_GY_ODR_AT_1920Hz;
    case IMU_GYRO_ODR_3840HZ:
        return ISM330BX_GY_ODR_AT_3840Hz;
    case IMU_GYRO_ODR_120HZ:
    default:
        return ISM330BX_GY_ODR_AT_120Hz;
    }
}

/* Convert wrapper accelerometer power/performance mode to ST driver enum. */
static ism330bx_xl_mode_t imu_accel_mode_to_driver(imu_accel_mode_t mode)
{
    switch (mode)
    {
    case IMU_ACCEL_MODE_LOW_POWER_2_AVG:
        return ISM330BX_XL_LOW_POWER_2_AVG_MD;

    case IMU_ACCEL_MODE_LOW_POWER_4_AVG:
        return ISM330BX_XL_LOW_POWER_4_AVG_MD;

    case IMU_ACCEL_MODE_LOW_POWER_8_AVG:
        return ISM330BX_XL_LOW_POWER_8_AVG_MD;

    case IMU_ACCEL_MODE_HIGH_PERFORMANCE:
    default:
        return ISM330BX_XL_HIGH_PERFORMANCE_MD;
    }
}

/* Convert wrapper gyroscope power/performance mode to ST driver enum. */
static ism330bx_gy_mode_t imu_gyro_mode_to_driver(imu_gyro_mode_t mode)
{
    switch (mode)
    {
    case IMU_GYRO_MODE_SLEEP:
        return ISM330BX_GY_SLEEP_MD;

    case IMU_GYRO_MODE_LOW_POWER:
        return ISM330BX_GY_LOW_POWER_MD;

    case IMU_GYRO_MODE_HIGH_PERFORMANCE:
    default:
        return ISM330BX_GY_HIGH_PERFORMANCE_MD;
    }
}

/* Convert wrapper interrupt pin output type to ST driver enum. */
static ism330bx_int_pin_mode_t imu_int_pin_mode_to_driver(imu_int_pin_mode_t mode)
{
    return (mode == IMU_INT_PIN_OPEN_DRAIN) ? ISM330BX_OPEN_DRAIN : ISM330BX_PUSH_PULL;
}

/* Convert wrapper interrupt active level to ST driver enum. */
static ism330bx_pin_polarity_t imu_int_polarity_to_driver(imu_int_polarity_t polarity)
{
    return (polarity == IMU_INT_ACTIVE_LOW) ? ISM330BX_ACTIVE_LOW : ISM330BX_ACTIVE_HIGH;
}

/* Convert wrapper interrupt latch/pulse mode to ST driver enum. */
static ism330bx_int_notification_t imu_int_latch_to_driver(imu_int_latch_t latch)
{
    return (latch == IMU_INT_LATCHED) ? ISM330BX_ALL_INT_LATCHED : ISM330BX_ALL_INT_PULSED;
}

/* Put XL/GY ODRs in power-down before enabling AH_QVAR, as required by ISM330BX. */
HAL_StatusTypeDef imu_internal_qvar_power_down_motion(void)
{
    if (ism330bx_xl_data_rate_set(&sImuCtx, ISM330BX_XL_ODR_OFF) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_gy_data_rate_set(&sImuCtx, ISM330BX_GY_ODR_OFF) != 0)
    {
        return HAL_ERROR;
    }

    sQvarMotionPowerDownActive = 1u;
    printf("[IMU] Qvar mode: accel/gyro ODR OFF before AH_QVAR_EN\r\n");
    return HAL_OK;
}

/* Restore the requested XL/GY motion configuration after leaving AH_QVAR mode. */
void imu_internal_qvar_restore_motion_if_needed(void)
{
    if ((sQvarMotionPowerDownActive == 0u) || (sMotionConfigValid == 0u) ||
        (sStarted == 0u))
    {
        return;
    }

    sQvarMotionPowerDownActive = 0u;
    if (imu_configure_motion(&sMotionConfig) != HAL_OK)
    {
        printf("[IMU] Qvar mode: motion restore FAIL\r\n");
        return;
    }
    printf("[IMU] Qvar mode: accel/gyro ODR restored after AH_QVAR_EN\r\n");
}

/* Fill an IMU config with the board default settings used for bring-up. */
void imu_get_default_config(imu_config_t *config)
{
    static const imu_config_t defaultConfig = IMU_CONFIG_DEFAULT;

    if (config == NULL)
    {
        return;
    }

    *config = defaultConfig;
}

/* Read WHO_AM_I and verify that the connected sensor is ISM330BX. */
HAL_StatusTypeDef imu_probe(uint8_t *who_am_i)
{
    uint8_t value = 0u;

    if (ism330bx_device_id_get(&sImuCtx, &value) != 0)
    {
        if (who_am_i != NULL)
        {
            *who_am_i = 0u;
        }
        return HAL_ERROR;
    }

    if (who_am_i != NULL)
    {
        *who_am_i = value;
    }

    return (value == ISM330BX_ID) ? HAL_OK : HAL_ERROR;
}

/* Initialize SPI2 IMU with the board default config and check WHO_AM_I. */
HAL_StatusTypeDef imu_init(void)
{
    imu_config_t config;

    imu_get_default_config(&config);
    return imu_init_with_config(&config);
}

/* Initialize SPI2 IMU with caller-provided config and check WHO_AM_I. */
HAL_StatusTypeDef imu_init_with_config(const imu_config_t *config)
{
    uint8_t whoAmI = 0u;
    imu_config_t defaultConfig;

    if (config == NULL)
    {
        imu_get_default_config(&defaultConfig);
        config = &defaultConfig;
    }

    if (spi_bus_device_init(&sImuSpi,
                            &hspi2,
                            IMU_CS_GPIO_Port,
                            IMU_CS_Pin,
                            SPI_BUS_CS_ACTIVE_LOW,
                            IMU_SPI_TIMEOUT_MS) != HAL_OK)
    {
        printf("[IMU] SPI device init FAIL\r\n");
        return HAL_ERROR;
    }

    sImuCtx.write_reg = imu_platform_write;
    sImuCtx.read_reg = imu_platform_read;
    sImuCtx.mdelay = imu_platform_delay;
    sImuCtx.handle = &sImuSpi;
    sImuCtx.priv_data = NULL;

    if (imu_probe(&whoAmI) != HAL_OK)
    {
        printf("[IMU] WHO_AM_I FAIL read=0x%02X expected=0x%02X\r\n",
               (unsigned)whoAmI,
               (unsigned)ISM330BX_ID);
        sStarted = 0u;
        return HAL_ERROR;
    }

    printf("[IMU] WHO_AM_I PASS 0x%02X\r\n", (unsigned)whoAmI);

    if (ism330bx_spi_mode_set(&sImuCtx, ISM330BX_SPI_4_WIRE) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_ui_i2c_i3c_mode_set(&sImuCtx, ISM330BX_I2C_I3C_DISABLE) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_auto_increment_set(&sImuCtx, PROPERTY_ENABLE) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_block_data_update_set(&sImuCtx, PROPERTY_ENABLE) != 0)
    {
        return HAL_ERROR;
    }

    sStarted = 1u;

    if (imu_configure_features(&config->features) != HAL_OK)
    {
        sStarted = 0u;
        return HAL_ERROR;
    }
    if (imu_configure_interrupt_pins(&config->interrupt_pins) != HAL_OK)
    {
        sStarted = 0u;
        return HAL_ERROR;
    }
    if (imu_configure_motion(&config->motion) != HAL_OK)
    {
        sStarted = 0u;
        return HAL_ERROR;
    }
    if (config->features.qvar_enable != 0u)
    {
        if (imu_qvar_start(&config->qvar) != HAL_OK)
        {
            sStarted = 0u;
            return HAL_ERROR;
        }
    }

    HAL_Delay(50u);
    printf("[IMU] configure PASS SPI2 accel=%u gyro=%u temp=%u qvar=%u\r\n",
           (unsigned)sFeatureConfig.accel_enable,
           (unsigned)sFeatureConfig.gyro_enable,
           (unsigned)sFeatureConfig.temperature_enable,
           (unsigned)sFeatureConfig.qvar_enable);
    return HAL_OK;
}

/* Return 1 after IMU init succeeds, otherwise 0. */
uint8_t imu_is_started(void)
{
    return sStarted;
}

/* Apply accel/gyro full-scale, ODR, and power mode configuration. */
HAL_StatusTypeDef imu_configure_motion(const imu_motion_config_t *config)
{
    ism330bx_xl_data_rate_t accelOdr;
    ism330bx_gy_data_rate_t gyroOdr;

    if ((config == NULL) || (sStarted == 0u))
    {
        return HAL_ERROR;
    }

    accelOdr = (sFeatureConfig.accel_enable != 0u) ?
               imu_accel_odr_to_driver(config->accel_odr) :
               ISM330BX_XL_ODR_OFF;
    gyroOdr = (sFeatureConfig.gyro_enable != 0u) ?
              imu_gyro_odr_to_driver(config->gyro_odr) :
              ISM330BX_GY_ODR_OFF;

    if (ism330bx_xl_full_scale_set(&sImuCtx,
                                   imu_accel_fs_to_driver(config->accel_full_scale)) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_gy_full_scale_set(&sImuCtx,
                                   imu_gyro_fs_to_driver(config->gyro_full_scale)) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_xl_mode_set(&sImuCtx,
                             imu_accel_mode_to_driver(config->accel_mode)) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_gy_mode_set(&sImuCtx,
                             imu_gyro_mode_to_driver(config->gyro_mode)) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_xl_data_rate_set(&sImuCtx, accelOdr) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_gy_data_rate_set(&sImuCtx, gyroOdr) != 0)
    {
        return HAL_ERROR;
    }

    sMotionConfig = *config;
    sMotionConfigValid = 1u;

    printf("[IMU] motion accel_fs=%u accel_odr=%u accel_mode=%u gyro_fs=%u gyro_odr=%u gyro_mode=%u\r\n",
           (unsigned)config->accel_full_scale,
           (unsigned)config->accel_odr,
           (unsigned)config->accel_mode,
           (unsigned)config->gyro_full_scale,
           (unsigned)config->gyro_odr,
           (unsigned)config->gyro_mode);
    return HAL_OK;
}

/* Enable or disable wrapper-level accel, gyro, temperature, and QVAR features. */
HAL_StatusTypeDef imu_configure_features(const imu_feature_config_t *config)
{
    if ((config == NULL) || (sStarted == 0u))
    {
        return HAL_ERROR;
    }

    sFeatureConfig = *config;

    if (sFeatureConfig.qvar_enable == 0u)
    {
        (void)imu_qvar_stop();
    }

    return HAL_OK;
}

/* Configure INT pin electrical mode, polarity, latch mode, and raw DRDY route. */
HAL_StatusTypeDef imu_configure_interrupt_pins(const imu_interrupt_pin_config_t *config)
{
    ism330bx_pin_int_route_t int1Route = {0};
    ism330bx_pin_int_route_t int2Route = {0};
    uint8_t int1RawEnable;
    uint8_t int2RawEnable;

    if ((config == NULL) || (sStarted == 0u) ||
        (((uint8_t)config->raw_data_ready_route & (uint8_t)~IMU_INTERRUPT_ROUTE_BOTH) != 0u))
    {
        return HAL_ERROR;
    }

    int1RawEnable = ((config->raw_data_ready_route & IMU_INTERRUPT_ROUTE_INT1_PD0) != 0u) ?
                    1u : 0u;
    int2RawEnable = ((config->raw_data_ready_route & IMU_INTERRUPT_ROUTE_INT2_PC13) != 0u) ?
                    1u : 0u;

    if (ism330bx_int_pin_mode_set(&sImuCtx,
                                  imu_int_pin_mode_to_driver(config->pin_mode)) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_pin_polarity_set(&sImuCtx,
                                  imu_int_polarity_to_driver(config->polarity)) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_int_notification_set(&sImuCtx,
                                      imu_int_latch_to_driver(config->latch)) != 0)
    {
        return HAL_ERROR;
    }

    if (ism330bx_pin_int1_route_get(&sImuCtx, &int1Route) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_pin_int2_route_get(&sImuCtx, &int2Route) != 0)
    {
        return HAL_ERROR;
    }

    int1Route.drdy_xl = int1RawEnable;
    int1Route.drdy_gy = int1RawEnable;
    int2Route.drdy_xl = int2RawEnable;
    int2Route.drdy_gy = int2RawEnable;

    if (ism330bx_pin_int1_route_set(&sImuCtx, int1Route) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_pin_int2_route_set(&sImuCtx, int2Route) != 0)
    {
        return HAL_ERROR;
    }

    printf("[IMU] int pins mode=%u polarity=%u latch=%u raw_route=%u\r\n",
           (unsigned)config->pin_mode,
           (unsigned)config->polarity,
           (unsigned)config->latch,
           (unsigned)config->raw_data_ready_route);
    return HAL_OK;
}

/* Read enabled raw accelerometer, gyroscope, and temperature channels. */
HAL_StatusTypeDef imu_read_raw(imu_raw_data_t *raw)
{
    if ((raw == NULL) || (sStarted == 0u))
    {
        return HAL_ERROR;
    }

    raw->accel[0] = 0;
    raw->accel[1] = 0;
    raw->accel[2] = 0;
    raw->gyro[0] = 0;
    raw->gyro[1] = 0;
    raw->gyro[2] = 0;
    raw->temperature = 0;
    raw->accel_valid = 0u;
    raw->gyro_valid = 0u;
    raw->temperature_valid = 0u;

    if (sFeatureConfig.accel_enable != 0u)
    {
        if (ism330bx_acceleration_raw_get(&sImuCtx, raw->accel) != 0)
        {
            return HAL_ERROR;
        }
        raw->accel_valid = 1u;
    }
    if (sFeatureConfig.gyro_enable != 0u)
    {
        if (ism330bx_angular_rate_raw_get(&sImuCtx, raw->gyro) != 0)
        {
            return HAL_ERROR;
        }
        raw->gyro_valid = 1u;
    }
    if (sFeatureConfig.temperature_enable != 0u)
    {
        if (ism330bx_temperature_raw_get(&sImuCtx, &raw->temperature) != 0)
        {
            return HAL_ERROR;
        }
        raw->temperature_valid = 1u;
    }

    return HAL_OK;
}

/* Print enabled raw accelerometer, gyroscope, and temperature channels. */
HAL_StatusTypeDef imu_print_raw(void)
{
    imu_raw_data_t raw;

    if (imu_read_raw(&raw) != HAL_OK)
    {
        printf("[IMU] raw read FAIL\r\n");
        return HAL_ERROR;
    }

    printf("[IMU] raw A=%s(%d,%d,%d) G=%s(%d,%d,%d) T=%s%d\r\n",
           (raw.accel_valid != 0u) ? "" : "NA:",
           (int)raw.accel[0],
           (int)raw.accel[1],
           (int)raw.accel[2],
           (raw.gyro_valid != 0u) ? "" : "NA:",
           (int)raw.gyro[0],
           (int)raw.gyro[1],
           (int)raw.gyro[2],
           (raw.temperature_valid != 0u) ? "" : "NA:",
           (int)raw.temperature);
    return HAL_OK;
}

/* Configure wake-up threshold interrupt and route it to INT1, INT2, or both. */
HAL_StatusTypeDef imu_configure_wakeup_interrupt(const imu_wakeup_interrupt_config_t *config)
{
    ism330bx_act_thresholds_t thresholds = {0};
    ism330bx_act_wkup_time_windows_t windows = {0};
    ism330bx_pin_int_route_t int1Route = {0};
    ism330bx_pin_int_route_t int2Route = {0};
    uint8_t int1Enable;
    uint8_t int2Enable;

    if ((config == NULL) || (sStarted == 0u) ||
        (((uint8_t)config->route & (uint8_t)~IMU_INTERRUPT_ROUTE_BOTH) != 0u) ||
        (config->duration_samples > IMU_WAKE_DURATION_MAX))
    {
        return HAL_ERROR;
    }

    int1Enable = ((config->enable != 0u) &&
                  ((config->route & IMU_INTERRUPT_ROUTE_INT1_PD0) != 0u)) ? 1u : 0u;
    int2Enable = ((config->enable != 0u) &&
                  ((config->route & IMU_INTERRUPT_ROUTE_INT2_PC13) != 0u)) ? 1u : 0u;

    if (ism330bx_act_thresholds_get(&sImuCtx, &thresholds) != 0)
    {
        return HAL_ERROR;
    }

    thresholds.wk_ths_mg = config->threshold_mg;
    if (ism330bx_act_thresholds_set(&sImuCtx, thresholds) != 0)
    {
        return HAL_ERROR;
    }

    if (ism330bx_act_wkup_time_windows_get(&sImuCtx, &windows) != 0)
    {
        return HAL_ERROR;
    }

    windows.shock = config->duration_samples;
    if (ism330bx_act_wkup_time_windows_set(&sImuCtx, windows) != 0)
    {
        return HAL_ERROR;
    }

    if (ism330bx_int_notification_set(&sImuCtx,
                                      (config->latched != 0u) ?
                                      ISM330BX_ALL_INT_LATCHED :
                                      ISM330BX_ALL_INT_PULSED) != 0)
    {
        return HAL_ERROR;
    }

    if (ism330bx_pin_int1_route_get(&sImuCtx, &int1Route) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_pin_int2_route_get(&sImuCtx, &int2Route) != 0)
    {
        return HAL_ERROR;
    }

    imu_apply_wakeup_route(&int1Route, int1Enable);
    imu_apply_wakeup_route(&int2Route, int2Enable);

    if (ism330bx_pin_int1_route_set(&sImuCtx, int1Route) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_pin_int2_route_set(&sImuCtx, int2Route) != 0)
    {
        return HAL_ERROR;
    }

    printf("[IMU] wake interrupt %s route=%u threshold=%lu mg duration=%u %s\r\n",
           (config->enable != 0u) ? "EN" : "DIS",
           (unsigned)config->route,
           (unsigned long)config->threshold_mg,
           (unsigned)config->duration_samples,
           (config->latched != 0u) ? "latched" : "pulsed");
    return HAL_OK;
}

/* Read and clear IMU interrupt source registers after an interrupt event. */
HAL_StatusTypeDef imu_get_interrupt_sources(imu_interrupt_sources_t *sources)
{
    ism330bx_all_sources_t allSources = {0};
    ism330bx_data_ready_t dataReady = {0};

    if ((sources == NULL) || (sStarted == 0u))
    {
        return HAL_ERROR;
    }

    if (ism330bx_all_sources_get(&sImuCtx, &allSources) != 0)
    {
        return HAL_ERROR;
    }
    if (ism330bx_flag_data_ready_get(&sImuCtx, &dataReady) != 0)
    {
        return HAL_ERROR;
    }

    sources->accel_data_ready = dataReady.drdy_xl;
    sources->gyro_data_ready = dataReady.drdy_gy;
    sources->qvar_data_ready = dataReady.drdy_ah_qvar;
    sources->wake_up = allSources.wake_up;
    sources->wake_up_x = allSources.wake_up_x;
    sources->wake_up_y = allSources.wake_up_y;
    sources->wake_up_z = allSources.wake_up_z;
    sources->free_fall = allSources.free_fall;
    sources->six_d = allSources.six_d;
    sources->sleep_change = allSources.sleep_change;
    sources->sleep_state = allSources.sleep_state;
    return HAL_OK;
}

/* Mark PD0/PC13 EXTI interrupt as pending for later task-context handling. */
void imu_handle_interrupt(uint16_t gpio_pin)
{
    if (gpio_pin == GPIO_PIN_0)
    {
        sInt1IrqCount++;
        sInt1Pending = 1u;
    }
    else if (gpio_pin == GPIO_PIN_13)
    {
        sInt2IrqCount++;
        sInt2Pending = 1u;
    }
}

/* Return ISR hit counters for PD0/INT1 and PC13/INT2 debug. */
void imu_get_interrupt_counts(uint32_t *int1_count, uint32_t *int2_count)
{
    if (int1_count != NULL)
    {
        *int1_count = sInt1IrqCount;
    }
    if (int2_count != NULL)
    {
        *int2_count = sInt2IrqCount;
    }
}

/* Return 1 when PC13/INT2 has a pending EXTI event. */
uint8_t imu_internal_get_int2_pending(void)
{
    return sInt2Pending;
}

/* Clear the pending PC13/INT2 EXTI event after QVAR consumes it. */
void imu_internal_clear_int2_pending(void)
{
    sInt2Pending = 0u;
}

/* Service pending IMU interrupts and optional periodic raw debug printing. */
void imu_task_poll(void)
{
    static uint32_t lastPrintTick;
    uint8_t shouldPrint = 0u;
    uint8_t int1Pending;
    uint8_t int2Pending;
    uint8_t qvarOwnsInt2;

    if (sStarted == 0u)
    {
        return;
    }

    qvarOwnsInt2 = imu_qvar_owns_int2();

    if ((sInt1Pending != 0u) ||
        ((sInt2Pending != 0u) && (qvarOwnsInt2 == 0u)))
    {
        imu_interrupt_sources_t sources = {0};

        int1Pending = sInt1Pending;
        int2Pending = (qvarOwnsInt2 == 0u) ? sInt2Pending : 0u;
        sInt1Pending = 0u;
        if (qvarOwnsInt2 == 0u)
        {
            sInt2Pending = 0u;
        }
        sources.int1_pending = int1Pending;
        sources.int2_pending = int2Pending;
        if (imu_get_interrupt_sources(&sources) == HAL_OK)
        {
            printf("[IMU INT] pending INT1=%u INT2=%u status DRDY=(A%u,G%u,Q%u) WU=%u xyz=(%u,%u,%u) FF=%u 6D=%u sleep=%u/%u\r\n",
                   (unsigned)sources.int1_pending,
                   (unsigned)sources.int2_pending,
                   (unsigned)sources.accel_data_ready,
                   (unsigned)sources.gyro_data_ready,
                   (unsigned)sources.qvar_data_ready,
                   (unsigned)sources.wake_up,
                   (unsigned)sources.wake_up_x,
                   (unsigned)sources.wake_up_y,
                   (unsigned)sources.wake_up_z,
                   (unsigned)sources.free_fall,
                   (unsigned)sources.six_d,
                   (unsigned)sources.sleep_change,
                   (unsigned)sources.sleep_state);
            printf("[IMU INT] pin PD0=%u PC13=%u irqcnt=(INT1:%lu,INT2:%lu)\r\n",
                   (unsigned)HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0),
                   (unsigned)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13),
                   (unsigned long)sInt1IrqCount,
                   (unsigned long)sInt2IrqCount);
        }
        shouldPrint = 1u;
    }
    else if ((HAL_GetTick() - lastPrintTick) >= IMU_PRINT_PERIOD_MS)
    {
        shouldPrint = 1u;
    }

    if (shouldPrint != 0u)
    {
        lastPrintTick = HAL_GetTick();
        (void)imu_print_raw();
    }
}

#define IMU_APP_WAKEUP_ENABLE                     0u
#define IMU_APP_WAKEUP_THRESHOLD_MG               250u
#define IMU_APP_WAKEUP_DURATION_SAMPLES           1u

#if (IMU_APP_WAKEUP_ENABLE != 0u)
static uint8_t sImuAppWakeupConfigured;
#endif

static void imu_app_wakeup_task(void)
{
#if (IMU_APP_WAKEUP_ENABLE != 0u)
    if (imu_is_started() == 0u)
    {
        return;
    }

    if (sImuAppWakeupConfigured == 0u)
    {
        imu_wakeup_interrupt_config_t config = {
            1u,
            IMU_INTERRUPT_ROUTE_INT1_PD0,
            IMU_APP_WAKEUP_THRESHOLD_MG,
            IMU_APP_WAKEUP_DURATION_SAMPLES,
            1u
        };

        if (imu_configure_wakeup_interrupt(&config) == HAL_OK)
        {
            sImuAppWakeupConfigured = 1u;
            printf("[IMU TEST] wakeup threshold configured on PD0/INT1\r\n");
        }
        else
        {
            printf("[IMU TEST] wakeup threshold config FAIL\r\n");
            return;
        }
    }

    imu_task_poll();
#endif
}

/* Run IMU application-level tests/features such as QVAR wear/button polling. */
void imu_app_task(void)
{
//	imu_task_poll();
//    imu_qvar_app_task();
    imu_app_wakeup_task();
}

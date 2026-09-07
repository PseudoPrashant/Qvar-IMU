/******************************************************************************
 * @file    imu.h
 * @brief   ISM330BX IMU driver wrapper for STM32 SPI2.
 ******************************************************************************/

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "stm32u5xx_hal.h"
#include "qvar.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temperature;
    uint8_t accel_valid;
    uint8_t gyro_valid;
    uint8_t temperature_valid;
} imu_raw_data_t;

typedef enum {
    IMU_INTERRUPT_ROUTE_NONE = 0u,
    IMU_INTERRUPT_ROUTE_INT1_PD0 = 1u,
    IMU_INTERRUPT_ROUTE_INT2_PC13 = 2u,
    IMU_INTERRUPT_ROUTE_BOTH = 3u
} imu_interrupt_route_t;

typedef struct {
    uint8_t enable;
    imu_interrupt_route_t route;
    uint32_t threshold_mg;
    uint8_t duration_samples;
    uint8_t latched;
} imu_wakeup_interrupt_config_t;

typedef struct {
    uint8_t int1_pending;
    uint8_t int2_pending;
    uint8_t accel_data_ready;
    uint8_t gyro_data_ready;
    uint8_t qvar_data_ready;
    uint8_t wake_up;
    uint8_t wake_up_x;
    uint8_t wake_up_y;
    uint8_t wake_up_z;
    uint8_t free_fall;
    uint8_t six_d;
    uint8_t sleep_change;
    uint8_t sleep_state;
} imu_interrupt_sources_t;

typedef enum {
    IMU_ACCEL_FS_2G = 0u,
    IMU_ACCEL_FS_4G,
    IMU_ACCEL_FS_8G
} imu_accel_fs_t;

typedef enum {
    IMU_GYRO_FS_125DPS = 0u,
    IMU_GYRO_FS_250DPS,
    IMU_GYRO_FS_500DPS,
    IMU_GYRO_FS_1000DPS,
    IMU_GYRO_FS_2000DPS,
    IMU_GYRO_FS_4000DPS
} imu_gyro_fs_t;

typedef enum {
    IMU_ACCEL_ODR_OFF = 0u,
    IMU_ACCEL_ODR_1HZ875,
    IMU_ACCEL_ODR_7HZ5,
    IMU_ACCEL_ODR_15HZ,
    IMU_ACCEL_ODR_30HZ,
    IMU_ACCEL_ODR_60HZ,
    IMU_ACCEL_ODR_120HZ,
    IMU_ACCEL_ODR_240HZ,
    IMU_ACCEL_ODR_480HZ,
    IMU_ACCEL_ODR_960HZ,
    IMU_ACCEL_ODR_1920HZ,
    IMU_ACCEL_ODR_3840HZ
} imu_accel_odr_t;

typedef enum {
    IMU_GYRO_ODR_OFF = 0u,
    IMU_GYRO_ODR_7HZ5,
    IMU_GYRO_ODR_15HZ,
    IMU_GYRO_ODR_30HZ,
    IMU_GYRO_ODR_60HZ,
    IMU_GYRO_ODR_120HZ,
    IMU_GYRO_ODR_240HZ,
    IMU_GYRO_ODR_480HZ,
    IMU_GYRO_ODR_960HZ,
    IMU_GYRO_ODR_1920HZ,
    IMU_GYRO_ODR_3840HZ
} imu_gyro_odr_t;

typedef enum {
    IMU_ACCEL_MODE_HIGH_PERFORMANCE = 0u,
    IMU_ACCEL_MODE_LOW_POWER_2_AVG,
    IMU_ACCEL_MODE_LOW_POWER_4_AVG,
    IMU_ACCEL_MODE_LOW_POWER_8_AVG
} imu_accel_mode_t;

typedef enum {
    IMU_GYRO_MODE_HIGH_PERFORMANCE = 0u,
    IMU_GYRO_MODE_SLEEP,
    IMU_GYRO_MODE_LOW_POWER
} imu_gyro_mode_t;

typedef enum {
    IMU_INT_PIN_PUSH_PULL = 0u,
    IMU_INT_PIN_OPEN_DRAIN
} imu_int_pin_mode_t;

typedef enum {
    IMU_INT_ACTIVE_HIGH = 0u,
    IMU_INT_ACTIVE_LOW
} imu_int_polarity_t;

typedef enum {
    IMU_INT_PULSED = 0u,
    IMU_INT_LATCHED
} imu_int_latch_t;

typedef struct {
    uint8_t accel_enable;
    uint8_t gyro_enable;
    uint8_t temperature_enable;
    uint8_t qvar_enable;
} imu_feature_config_t;

typedef struct {
    imu_accel_fs_t accel_full_scale;
    imu_accel_odr_t accel_odr;
    imu_accel_mode_t accel_mode;
    imu_gyro_fs_t gyro_full_scale;
    imu_gyro_odr_t gyro_odr;
    imu_gyro_mode_t gyro_mode;
} imu_motion_config_t;

typedef struct {
    imu_int_pin_mode_t pin_mode;
    imu_int_polarity_t polarity;
    imu_int_latch_t latch;
    imu_interrupt_route_t raw_data_ready_route;
} imu_interrupt_pin_config_t;

typedef struct {
    imu_feature_config_t features;
    imu_motion_config_t motion;
    imu_interrupt_pin_config_t interrupt_pins;
    imu_qvar_config_t qvar;
} imu_config_t;

/* IMU default profile used for normal bring-up and raw data testing. */
#define IMU_CONFIG_DEFAULT {                                      \
    .features = {                                                 \
        .accel_enable = 1u,                                       \
        .gyro_enable = 1u,                                        \
        .temperature_enable = 1u,                                 \
        .qvar_enable = 0u                                         \
    },                                                            \
    .motion = {                                                   \
        .accel_full_scale = IMU_ACCEL_FS_2G,                      \
        .accel_odr = IMU_ACCEL_ODR_120HZ,                         \
        .accel_mode = IMU_ACCEL_MODE_HIGH_PERFORMANCE,            \
        .gyro_full_scale = IMU_GYRO_FS_250DPS,                    \
        .gyro_odr = IMU_GYRO_ODR_120HZ,                           \
        .gyro_mode = IMU_GYRO_MODE_HIGH_PERFORMANCE               \
    },                                                            \
    .interrupt_pins = {                                           \
        .pin_mode = IMU_INT_PIN_PUSH_PULL,                        \
        .polarity = IMU_INT_ACTIVE_HIGH,                          \
        .latch = IMU_INT_LATCHED,                                 \
        .raw_data_ready_route = IMU_INTERRUPT_ROUTE_BOTH           \
    },                                                            \
    .qvar = QVAR_CONFIG_DISABLED                                  \
}

/* IMU low-power profile for slower accel-only monitoring. */
#define IMU_CONFIG_LOW_POWER {                                    \
    .features = {                                                 \
        .accel_enable = 1u,                                       \
        .gyro_enable = 0u,                                        \
        .temperature_enable = 0u,                                 \
        .qvar_enable = 0u                                         \
    },                                                            \
    .motion = {                                                   \
        .accel_full_scale = IMU_ACCEL_FS_2G,                      \
        .accel_odr = IMU_ACCEL_ODR_15HZ,                          \
        .accel_mode = IMU_ACCEL_MODE_LOW_POWER_4_AVG,             \
        .gyro_full_scale = IMU_GYRO_FS_250DPS,                    \
        .gyro_odr = IMU_GYRO_ODR_OFF,                             \
        .gyro_mode = IMU_GYRO_MODE_LOW_POWER                      \
    },                                                            \
    .interrupt_pins = {                                           \
        .pin_mode = IMU_INT_PIN_PUSH_PULL,                        \
        .polarity = IMU_INT_ACTIVE_HIGH,                          \
        .latch = IMU_INT_LATCHED,                                 \
        .raw_data_ready_route = IMU_INTERRUPT_ROUTE_BOTH          \
    },                                                            \
    .qvar = QVAR_CONFIG_DISABLED                                  \
}

/* IMU high-performance profile for faster accel/gyro response. */
#define IMU_CONFIG_HIGH_PERF {                                    \
    .features = {                                                 \
        .accel_enable = 1u,                                       \
        .gyro_enable = 1u,                                        \
        .temperature_enable = 1u,                                 \
        .qvar_enable = 0u                                         \
    },                                                            \
    .motion = {                                                   \
        .accel_full_scale = IMU_ACCEL_FS_4G,                      \
        .accel_odr = IMU_ACCEL_ODR_240HZ,                         \
        .accel_mode = IMU_ACCEL_MODE_HIGH_PERFORMANCE,            \
        .gyro_full_scale = IMU_GYRO_FS_500DPS,                    \
        .gyro_odr = IMU_GYRO_ODR_240HZ,                           \
        .gyro_mode = IMU_GYRO_MODE_HIGH_PERFORMANCE               \
    },                                                            \
    .interrupt_pins = {                                           \
        .pin_mode = IMU_INT_PIN_PUSH_PULL,                        \
        .polarity = IMU_INT_ACTIVE_HIGH,                          \
        .latch = IMU_INT_LATCHED,                                 \
        .raw_data_ready_route = IMU_INTERRUPT_ROUTE_BOTH          \
    },                                                            \
    .qvar = QVAR_CONFIG_DISABLED                                  \
}

/* Fill an IMU config with the board default settings used for bring-up. */
void imu_get_default_config(imu_config_t *config);

/* Initialize SPI2 IMU with the board default config and check WHO_AM_I. */
HAL_StatusTypeDef imu_init(void);

/* Initialize SPI2 IMU with caller-provided config and check WHO_AM_I. */
HAL_StatusTypeDef imu_init_with_config(const imu_config_t *config);

/* Read WHO_AM_I and verify that the connected sensor is ISM330BX. */
HAL_StatusTypeDef imu_probe(uint8_t *who_am_i);

/* Return 1 after IMU init succeeds, otherwise 0. */
uint8_t imu_is_started(void);

/* Apply accel/gyro full-scale, ODR, and power mode configuration. */
HAL_StatusTypeDef imu_configure_motion(const imu_motion_config_t *config);

/* Enable or disable wrapper-level accel, gyro, temperature, and QVAR features. */
HAL_StatusTypeDef imu_configure_features(const imu_feature_config_t *config);

/* Configure INT pin electrical mode, polarity, latch mode, and raw DRDY route. */
HAL_StatusTypeDef imu_configure_interrupt_pins(const imu_interrupt_pin_config_t *config);

/* Read enabled raw accelerometer, gyroscope, and temperature channels. */
HAL_StatusTypeDef imu_read_raw(imu_raw_data_t *raw);

/* Print enabled raw accelerometer, gyroscope, and temperature channels. */
HAL_StatusTypeDef imu_print_raw(void);

/* Configure wake-up threshold interrupt and route it to INT1, INT2, or both. */
HAL_StatusTypeDef imu_configure_wakeup_interrupt(const imu_wakeup_interrupt_config_t *config);

/* Read and clear IMU interrupt source registers after an interrupt event. */
HAL_StatusTypeDef imu_get_interrupt_sources(imu_interrupt_sources_t *sources);

/* Mark PD0/PC13 EXTI interrupt as pending for later task-context handling. */
void imu_handle_interrupt(uint16_t gpio_pin);

/* Return ISR hit counters for PD0/INT1 and PC13/INT2 debug. */
void imu_get_interrupt_counts(uint32_t *int1_count, uint32_t *int2_count);

/* Service pending IMU interrupts and optional periodic raw debug printing. */
void imu_task_poll(void);

/* Run IMU application-level tests/features such as QVAR wear/button polling. */
void imu_app_task(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */

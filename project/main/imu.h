/******************************************************************************
 * @file    imu.h
 * @brief   ISM330BX IMU driver wrapper for FreeRTOS / ESP32.
 ******************************************************************************/

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
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

/* Initialize IMU over I2C and check WHO_AM_I. Returns 0 on success. */
int imu_init(void);

/* Return 1 after IMU init succeeds, otherwise 0. */
uint8_t imu_is_started(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */

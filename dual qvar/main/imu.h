/******************************************************************************
 * @file    imu.h
 * @brief   Raw IMU and QVAR I2C interface for ESP32 / FreeRTOS.
 ******************************************************************************/

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "ism330bx_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize I2C and check ISM330BX WHO_AM_I (0x71). Returns 0 on success. */
int imu_init(void);

/* Get device register access context pointer. */
stmdev_ctx_t *imu_get_ctx(void);

/* Configure and start raw QVAR sensing (Q1 enabled, ZERO hardware or software filters). */
int imu_raw_qvar_start(ism330bx_ah_qvar_zin_t zin);

/* Read instantaneous raw 16-bit QVAR1 ADC register value (zero processing). */
int imu_raw_qvar_read(int16_t *raw_out);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */

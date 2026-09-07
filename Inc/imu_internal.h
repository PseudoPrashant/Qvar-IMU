/******************************************************************************
 * @file    imu_internal.h
 * @brief   Internal hooks shared by IMU feature modules.
 ******************************************************************************/

#ifndef IMU_INTERNAL_H
#define IMU_INTERNAL_H

#include "stm32u5xx_hal.h"
#include "ism330bx_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Return the initialized ST register-driver context used by the IMU wrapper. */
stmdev_ctx_t *imu_internal_ctx(void);

/* Put accel/gyro in power-down while the AH/QVAR front end is configured. */
HAL_StatusTypeDef imu_internal_qvar_power_down_motion(void);

/* Restore accel/gyro settings after AH/QVAR front-end configuration. */
void imu_internal_qvar_restore_motion_if_needed(void);

/* Return 1 when PC13/INT2 has a pending EXTI event. */
uint8_t imu_internal_get_int2_pending(void);

/* Clear the pending PC13/INT2 EXTI event after QVAR consumes it. */
void imu_internal_clear_int2_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_INTERNAL_H */

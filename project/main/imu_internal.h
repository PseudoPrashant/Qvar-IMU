#ifndef IMU_INTERNAL_H
#define IMU_INTERNAL_H

#include "ism330bx_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

stmdev_ctx_t *imu_internal_ctx(void);
int imu_internal_qvar_power_down_motion(void);
void imu_internal_qvar_restore_motion_if_needed(void);
uint8_t imu_internal_get_int2_pending(void);
void imu_internal_clear_int2_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_INTERNAL_H */
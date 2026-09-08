/******************************************************************************
 * @file    qvar.h
 * @brief   ISM330BX QVAR electrode sensing API and app-level polling detector.
 ******************************************************************************/

#ifndef QVAR_H
#define QVAR_H

#include <stdint.h>

#ifndef HAL_OK
typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_QVAR_DEFAULT_SETTLING_MS 5u

typedef enum {
    IMU_QVAR_USE_DISABLED = 0u,
    IMU_QVAR_USE_RAW,
    IMU_QVAR_USE_WEAR,
    IMU_QVAR_USE_BUTTON
} imu_qvar_use_t;

typedef enum {
    IMU_QVAR_ZIN_2400_MOHM = 0u,
    IMU_QVAR_ZIN_730_MOHM = 1u,
    IMU_QVAR_ZIN_300_MOHM = 2u,
    IMU_QVAR_ZIN_235_MOHM = 3u
} imu_qvar_zin_t;

typedef struct {
    uint8_t enableQvar1;              /* 1: read QVAR1 electrode, 0: skip QVAR1. */
    uint8_t enableQvar2;              /* 1: read QVAR2 electrode, 0: skip QVAR2. */
    imu_qvar_use_t qvar1Use;          /* Application role for QVAR1. */
    imu_qvar_use_t qvar2Use;          /* Application role for QVAR2. */
    uint8_t hpfEnable;                /* 1: enable AH/QVAR high-pass filter. */
    uint8_t lpfEnable;                /* 1: enable AH/QVAR low-pass filter. */
    uint8_t dataReadyInterruptEnable; /* 1: route AH/QVAR DRDY to INT2/PC13. */
    imu_qvar_zin_t zin;               /* AH/QVAR input impedance selection. */
    uint8_t settlingMs;               /* Delay after electrode switching before read. */
} imu_qvar_config_t;

typedef struct {
    int16_t qvar1;       /* Latest raw QVAR1 sample. */
    int16_t qvar2;       /* Latest raw QVAR2 sample. */
    uint8_t qvar1Valid;  /* 1: qvar1 contains a valid sample. */
    uint8_t qvar2Valid;  /* 1: qvar2 contains a valid sample. */
} imu_qvar_raw_t;

/* QVAR disabled profile for normal accel/gyro-only IMU use. */
#define QVAR_CONFIG_DISABLED {                         \
    .enableQvar1 = 0u,                                 \
    .enableQvar2 = 0u,                                 \
    .qvar1Use = IMU_QVAR_USE_DISABLED,                 \
    .qvar2Use = IMU_QVAR_USE_DISABLED,                 \
    .hpfEnable = 0u,                                   \
    .lpfEnable = 1u,                                   \
    .dataReadyInterruptEnable = 0u,                    \
    .zin = IMU_QVAR_ZIN_2400_MOHM,                    \
    .settlingMs = IMU_QVAR_DEFAULT_SETTLING_MS         \
}

/* QVAR profile that reads both electrodes as raw values only. */
#define QVAR_CONFIG_RAW_BOTH {                         \
    .enableQvar1 = 1u,                                 \
    .enableQvar2 = 1u,                                 \
    .qvar1Use = IMU_QVAR_USE_RAW,                      \
    .qvar2Use = IMU_QVAR_USE_RAW,                      \
    .hpfEnable = 0u,                                   \
    .lpfEnable = 1u,                                   \
    .dataReadyInterruptEnable = 0u,                    \
    .zin = IMU_QVAR_ZIN_2400_MOHM,                    \
    .settlingMs = IMU_QVAR_DEFAULT_SETTLING_MS         \
}

/* QVAR profile for glasses: QVAR1 button and QVAR2 wear sensing. */
#define QVAR_CONFIG_WEAR_Q2_BUTTON_Q1 {                \
    .enableQvar1 = 1u,                                 \
    .enableQvar2 = 1u,                                 \
    .qvar1Use = IMU_QVAR_USE_BUTTON,                   \
    .qvar2Use = IMU_QVAR_USE_WEAR,                     \
    .hpfEnable = 0u,                                   \
    .lpfEnable = 1u,                                   \
    .dataReadyInterruptEnable = 0u,                    \
    .zin = IMU_QVAR_ZIN_2400_MOHM,                    \
    .settlingMs = IMU_QVAR_DEFAULT_SETTLING_MS         \
}

/* QVAR profile that enables only QVAR1 as a button electrode. */
#define QVAR_CONFIG_BUTTON_Q1_ONLY {                   \
    .enableQvar1 = 1u,                                 \
    .enableQvar2 = 0u,                                 \
    .qvar1Use = IMU_QVAR_USE_BUTTON,                   \
    .qvar2Use = IMU_QVAR_USE_DISABLED,                 \
    .hpfEnable = 1u,                                   \
    .lpfEnable = 1u,                                   \
    .dataReadyInterruptEnable = 0u,                    \
    .zin = IMU_QVAR_ZIN_300_MOHM,                     \
    .settlingMs = IMU_QVAR_DEFAULT_SETTLING_MS         \
}

/* QVAR profile that enables only QVAR2 as a wear electrode. */
#define QVAR_CONFIG_WEAR_Q2_ONLY {                     \
    .enableQvar1 = 0u,                                 \
    .enableQvar2 = 1u,                                 \
    .qvar1Use = IMU_QVAR_USE_DISABLED,                 \
    .qvar2Use = IMU_QVAR_USE_WEAR,                     \
    .hpfEnable = 0u,                                   \
    .lpfEnable = 1u,                                   \
    .dataReadyInterruptEnable = 0u,                    \
    .zin = IMU_QVAR_ZIN_2400_MOHM,                    \
    .settlingMs = IMU_QVAR_DEFAULT_SETTLING_MS         \
}

/* Configure QVAR data-ready interrupt; chip supports this only on INT2/PC13. */
HAL_StatusTypeDef imu_configure_qvar_data_ready_interrupt(uint8_t enable);

/* Consume a pending INT2 QVAR data-ready event; returns 1 when QVAR should be read. */
uint8_t imu_qvar_consume_data_ready_interrupt(void);

/* Return 1 when QVAR owns INT2/PC13 for data-ready handling. */
uint8_t imu_qvar_owns_int2(void);

/* Start QVAR electrode sensing with configurable QVAR1/QVAR2 use cases. */
HAL_StatusTypeDef imu_qvar_start(const imu_qvar_config_t *config);

/* Stop QVAR electrode sensing and disable the AH/QVAR chain. */
HAL_StatusTypeDef imu_qvar_stop(void);

/* Read raw QVAR1/QVAR2 values from the enabled electrodes. */
HAL_StatusTypeDef imu_qvar_read_raw(imu_qvar_raw_t *raw);

/* Print raw QVAR1/QVAR2 values from the enabled electrodes. */
HAL_StatusTypeDef imu_qvar_print_raw(void);

/* Run the QVAR polling app: Q2 wear sensing and Q1 button detection. */
void imu_qvar_app_task(void);

#ifdef __cplusplus
}
#endif

#endif /* QVAR_H */

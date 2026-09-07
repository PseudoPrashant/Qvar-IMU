/******************************************************************************
 * @file    qvar_config.h
 * @brief   User-editable QVAR app profiles and detection settings.
 ******************************************************************************/

#ifndef QVAR_CONFIG_H
#define QVAR_CONFIG_H

#include <stdint.h>
#include "qvar.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum QVAR2 wear averaging buffer size allocated by qvar.c. */
#define QVAR_APP_WEAR_AVG_BUFFER_MAX_SAMPLES 30u

typedef struct {
    uint8_t enable;                         /* 1: run QVAR app, 0: app off. */
    uint8_t rawLogEnable;                   /* 1: print raw QVAR samples. */
    imu_qvar_config_t qvarConfig;           /* Low-level QVAR electrode config. */
    uint32_t readPeriodMs;                  /* QVAR polling period. */
    uint32_t startupSettleMs;               /* Delay before baseline learning. */
    uint8_t baselineSamples;                /* Samples used for first baseline. */
    int32_t baselineTrackDiv;               /* Larger value means slower baseline. */
    uint8_t wearAvgSamples;                 /* QVAR2 moving-average sample count. */
    uint32_t buttonEventCooldownMs;         /* Delay after QVAR1 button event. */
    uint32_t buttonIdleRearmMs;             /* Quiet time before QVAR1 re-arms. */
    uint32_t wearButtonArmDelayMs;          /* Delay after wear-on before button. */
    int32_t qvar2WearThresholdRaw;          /* QVAR2 delta needed for wear-on. */
    int32_t qvar2WearReleaseRaw;            /* QVAR2 delta needed for wear-off. */
    uint32_t qvar2WearOnConfirmMs;          /* Wear-on confirmation time. */
    uint32_t qvar2WearOffConfirmMs;         /* Wear-off confirmation time. */
    int32_t qvar1ButtonThresholdRaw;        /* QVAR1 delta needed for press. */
    int32_t qvar1ButtonReleaseRaw;          /* QVAR1 delta needed for release. */
    int32_t qvar1ButtonMinPeakRaw;          /* Minimum peak for single press. */
    uint32_t qvar1ButtonMaxPressMs;         /* Max duration for single press. */
    uint32_t qvar1ButtonHoldTimeMs;         /* Duration needed for hold. */
    uint8_t qvar1ButtonTouchConfirmSamples; /* Press confirm sample count. */
    uint8_t qvar1ButtonReleaseConfirmSamples; /* Release confirm sample count. */
} qvar_app_config_t;

/* App off: QVAR app does not start sensing. */
#define QVAR_APP_CONFIG_DISABLED {                         \
    .enable = 0u,                                          \
    .rawLogEnable = 0u,                                    \
    .qvarConfig = QVAR_CONFIG_DISABLED,                    \
    .readPeriodMs = 25u,                                   \
    .startupSettleMs = 5000u,                              \
    .baselineSamples = 40u,                                \
    .baselineTrackDiv = 32L,                               \
    .wearAvgSamples = 10u,                                 \
    .buttonEventCooldownMs = 700u,                         \
    .buttonIdleRearmMs = 300u,                             \
    .wearButtonArmDelayMs = 2000u,                         \
    .qvar2WearThresholdRaw = 700L,                         \
    .qvar2WearReleaseRaw = 250L,                           \
    .qvar2WearOnConfirmMs = 500u,                          \
    .qvar2WearOffConfirmMs = 1500u,                        \
    .qvar1ButtonThresholdRaw = 900L,                       \
    .qvar1ButtonReleaseRaw = 350L,                         \
    .qvar1ButtonMinPeakRaw = 1300L,                        \
    .qvar1ButtonMaxPressMs = 400u,                         \
    .qvar1ButtonHoldTimeMs = 1200u,                        \
    .qvar1ButtonTouchConfirmSamples = 3u,                  \
    .qvar1ButtonReleaseConfirmSamples = 3u                 \
}

/* Raw debug: read and print both electrodes, no wear/button meaning. */
#define QVAR_APP_CONFIG_RAW_BOTH {                         \
    .enable = 1u,                                          \
    .rawLogEnable = 1u,                                    \
    .qvarConfig = QVAR_CONFIG_RAW_BOTH,                    \
    .readPeriodMs = 25u,                                   \
    .startupSettleMs = 0u,                                 \
    .baselineSamples = 40u,                                \
    .baselineTrackDiv = 32L,                               \
    .wearAvgSamples = 10u,                                 \
    .buttonEventCooldownMs = 700u,                         \
    .buttonIdleRearmMs = 300u,                             \
    .wearButtonArmDelayMs = 2000u,                         \
    .qvar2WearThresholdRaw = 700L,                         \
    .qvar2WearReleaseRaw = 250L,                           \
    .qvar2WearOnConfirmMs = 500u,                          \
    .qvar2WearOffConfirmMs = 1500u,                        \
    .qvar1ButtonThresholdRaw = 900L,                       \
    .qvar1ButtonReleaseRaw = 350L,                         \
    .qvar1ButtonMinPeakRaw = 1300L,                        \
    .qvar1ButtonMaxPressMs = 400u,                         \
    .qvar1ButtonHoldTimeMs = 1200u,                        \
    .qvar1ButtonTouchConfirmSamples = 3u,                  \
    .qvar1ButtonReleaseConfirmSamples = 3u                 \
}

/* Glass use case: QVAR2 wear sensing, QVAR1 button. */
#define QVAR_APP_CONFIG_WEAR_Q2_BUTTON_Q1 {                \
    .enable = 1u,                                          \
    .rawLogEnable = 1u,                                    \
    .qvarConfig = QVAR_CONFIG_WEAR_Q2_BUTTON_Q1,           \
    .readPeriodMs = 25u,                                   \
    .startupSettleMs = 5000u,                              \
    .baselineSamples = 40u,                                \
    .baselineTrackDiv = 32L,                               \
    .wearAvgSamples = 10u,                                 \
    .buttonEventCooldownMs = 700u,                         \
    .buttonIdleRearmMs = 300u,                             \
    .wearButtonArmDelayMs = 2000u,                         \
    .qvar2WearThresholdRaw = 700L,                         \
    .qvar2WearReleaseRaw = 250L,                           \
    .qvar2WearOnConfirmMs = 500u,                          \
    .qvar2WearOffConfirmMs = 1500u,                        \
    .qvar1ButtonThresholdRaw = 900L,                       \
    .qvar1ButtonReleaseRaw = 350L,                         \
    .qvar1ButtonMinPeakRaw = 1300L,                        \
    .qvar1ButtonMaxPressMs = 400u,                         \
    .qvar1ButtonHoldTimeMs = 1200u,                        \
    .qvar1ButtonTouchConfirmSamples = 3u,                  \
    .qvar1ButtonReleaseConfirmSamples = 3u                 \
}

/* QVAR1 button only; button works without wear gating. */
#define QVAR_APP_CONFIG_BUTTON_Q1_ONLY {                   \
    .enable = 1u,                                          \
    .rawLogEnable = 1u,                                    \
    .qvarConfig = QVAR_CONFIG_BUTTON_Q1_ONLY,              \
    .readPeriodMs = 25u,                                   \
    .startupSettleMs = 5000u,                              \
    .baselineSamples = 40u,                                \
    .baselineTrackDiv = 32L,                               \
    .wearAvgSamples = 10u,                                 \
    .buttonEventCooldownMs = 700u,                         \
    .buttonIdleRearmMs = 300u,                             \
    .wearButtonArmDelayMs = 0u,                            \
    .qvar2WearThresholdRaw = 700L,                         \
    .qvar2WearReleaseRaw = 250L,                           \
    .qvar2WearOnConfirmMs = 500u,                          \
    .qvar2WearOffConfirmMs = 1500u,                        \
    .qvar1ButtonThresholdRaw = 900L,                       \
    .qvar1ButtonReleaseRaw = 350L,                         \
    .qvar1ButtonMinPeakRaw = 1300L,                        \
    .qvar1ButtonMaxPressMs = 400u,                         \
    .qvar1ButtonHoldTimeMs = 1200u,                        \
    .qvar1ButtonTouchConfirmSamples = 3u,                  \
    .qvar1ButtonReleaseConfirmSamples = 3u                 \
}

/* QVAR2 wear only; QVAR1 button disabled. */
#define QVAR_APP_CONFIG_WEAR_Q2_ONLY {                     \
    .enable = 1u,                                          \
    .rawLogEnable = 1u,                                    \
    .qvarConfig = QVAR_CONFIG_WEAR_Q2_ONLY,                \
    .readPeriodMs = 25u,                                   \
    .startupSettleMs = 5000u,                              \
    .baselineSamples = 40u,                                \
    .baselineTrackDiv = 32L,                               \
    .wearAvgSamples = 10u,                                 \
    .buttonEventCooldownMs = 700u,                         \
    .buttonIdleRearmMs = 300u,                             \
    .wearButtonArmDelayMs = 0u,                            \
    .qvar2WearThresholdRaw = 700L,                         \
    .qvar2WearReleaseRaw = 250L,                           \
    .qvar2WearOnConfirmMs = 500u,                          \
    .qvar2WearOffConfirmMs = 1500u,                        \
    .qvar1ButtonThresholdRaw = 900L,                       \
    .qvar1ButtonReleaseRaw = 350L,                         \
    .qvar1ButtonMinPeakRaw = 1300L,                        \
    .qvar1ButtonMaxPressMs = 400u,                         \
    .qvar1ButtonHoldTimeMs = 1200u,                        \
    .qvar1ButtonTouchConfirmSamples = 3u,                  \
    .qvar1ButtonReleaseConfirmSamples = 3u                 \
}

/* Active QVAR app profile. Change this one macro to select behavior. */
#ifndef QVAR_APP_ACTIVE_CONFIG
#define QVAR_APP_ACTIVE_CONFIG QVAR_APP_CONFIG_WEAR_Q2_BUTTON_Q1
#endif

#ifdef __cplusplus
}
#endif

#endif /* QVAR_CONFIG_H */

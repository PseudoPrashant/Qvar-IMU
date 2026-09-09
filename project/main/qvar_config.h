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
    uint32_t qvar1ButtonDoubleTapWindowMs;  /* Max delay between taps for double-tap. */
    uint8_t qvar1ButtonTouchConfirmSamples; /* Press confirm sample count. */
    uint8_t qvar1ButtonReleaseConfirmSamples; /* Release confirm sample count. */
    int32_t tapPressThresholdRaw;           /* 4-sample peak-to-peak contact threshold (e.g. 3300 LSB). */
    int32_t tapReleaseThresholdRaw;         /* Release threshold with hysteresis (e.g. 2650 LSB). */
    uint16_t tapMinDurationSamples;         /* Min tap duration (e.g. 8 samples = 40 ms). */
    uint16_t tapMaxDurationSamples;         /* Max tap duration (e.g. 70 samples = 350 ms, rejects holds). */
    uint16_t tapCooldownSamples;            /* Refractory lockout after tap (e.g. 25 samples = 125 ms). */
    uint16_t tapSquelchTimerSamples;        /* Disturbance squelch lockout (e.g. 50 samples = 250 ms). */
    uint16_t tapSquelchQuietSamples;        /* Continuous calm required to exit squelch (e.g. 15 samples = 75 ms). */
    int16_t tapBipolarMinRaw;               /* Minimum raw excursion required (e.g. -500 LSB). */
    int16_t tapBipolarMaxRaw;               /* Maximum raw excursion required (e.g. +500 LSB). */
    uint8_t tapAdaptiveBaselineEnable;      /* 1: enable adaptive baseline envelope filter, 0: static thresholds. */
    int32_t tapPressDeltaRaw;               /* Dynamic contact delta above baseline (e.g. 1100 LSB). */
    int32_t tapReleaseDeltaRaw;             /* Dynamic release delta above baseline (e.g. 600 LSB). */
    int32_t tapMinPeakActRaw;               /* Minimum activity peak required for tap (e.g. 3300 LSB). */
    float tapAlphaRise;                     /* EMA rise factor (e.g. 0.999f = slow rise during contact). */
    float tapAlphaFall;                     /* EMA fall factor (e.g. 0.988f = fast fall when calm). */
    int32_t tapLowerThresholdRaw;           /* Band-pass floor (3500 LSB). */
    int32_t tapUpperCeilingRaw;             /* Band-pass kill-switch ceiling (5500 LSB). */
    uint16_t tapBridgeTimerSamples;         /* Bridge timer (10 samples = 50 ms). */
    uint16_t tapLockoutSamples;             /* Post-tap lockout cooldown (30 samples = 150 ms). */
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
    .qvar1ButtonThresholdRaw = 18000L,                     \
    .qvar1ButtonReleaseRaw = 9000L,                        \
    .qvar1ButtonMinPeakRaw = 25000L,                       \
    .qvar1ButtonMaxPressMs = 400u,                         \
    .qvar1ButtonHoldTimeMs = 1200u,                        \
    .qvar1ButtonDoubleTapWindowMs = 350u,                  \
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
    .qvar1ButtonDoubleTapWindowMs = 350u,                  \
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
    .qvar1ButtonDoubleTapWindowMs = 350u,                  \
    .qvar1ButtonTouchConfirmSamples = 3u,                  \
    .qvar1ButtonReleaseConfirmSamples = 3u                 \
}

/* QVAR1 button only; button works without wear gating. */
#define QVAR_APP_CONFIG_BUTTON_Q1_ONLY {                   \
    .enable = 1u,                                          \
    .rawLogEnable = 1u,                                    \
    .qvarConfig = QVAR_CONFIG_BUTTON_Q1_ONLY,              \
    .readPeriodMs = 5u,                                    \
    .startupSettleMs = 1000u,                              \
    .baselineSamples = 50u,                                \
    .baselineTrackDiv = 8L,                                \
    .wearAvgSamples = 10u,                                 \
    .buttonEventCooldownMs = 50u,                          \
    .buttonIdleRearmMs = 100u,                             \
    .wearButtonArmDelayMs = 0u,                            \
    .qvar2WearThresholdRaw = 700L,                         \
    .qvar2WearReleaseRaw = 250L,                           \
    .qvar2WearOnConfirmMs = 500u,                          \
    .qvar2WearOffConfirmMs = 1500u,                        \
    .qvar1ButtonThresholdRaw = 2800L,                      \
    .qvar1ButtonReleaseRaw = 2200L,                        \
    .qvar1ButtonMinPeakRaw = 3000L,                        \
    .qvar1ButtonMaxPressMs = 400u,                         \
    .qvar1ButtonHoldTimeMs = 800u,                         \
    .qvar1ButtonDoubleTapWindowMs = 300u,                  \
    .qvar1ButtonTouchConfirmSamples = 3u,                  \
    .qvar1ButtonReleaseConfirmSamples = 2u,                \
    .tapPressThresholdRaw = 3300L,                         \
    .tapReleaseThresholdRaw = 2650L,                       \
    .tapMinDurationSamples = 8u,                           \
    .tapMaxDurationSamples = 70u,                          \
    .tapCooldownSamples = 25u,                             \
    .tapSquelchTimerSamples = 50u,                         \
    .tapSquelchQuietSamples = 15u,                         \
    .tapBipolarMinRaw = -500,                              \
    .tapBipolarMaxRaw = 500,                               \
    .tapAdaptiveBaselineEnable = 1u,                       \
    .tapPressDeltaRaw = 1100L,                             \
    .tapReleaseDeltaRaw = 600L,                            \
    .tapMinPeakActRaw = 3300L,                             \
    .tapAlphaRise = 0.99917f,                              \
    .tapAlphaFall = 0.9875f,                               \
    .tapLowerThresholdRaw = 3500L,                         \
    .tapUpperCeilingRaw = 5500L,                           \
    .tapBridgeTimerSamples = 10u,                          \
    .tapLockoutSamples = 30u                               \
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
    .qvar1ButtonDoubleTapWindowMs = 350u,                  \
    .qvar1ButtonTouchConfirmSamples = 3u,                  \
    .qvar1ButtonReleaseConfirmSamples = 3u                 \
}

/* Active QVAR app profile. Changed to QVAR_APP_CONFIG_BUTTON_Q1_ONLY. */
#ifndef QVAR_APP_ACTIVE_CONFIG
#define QVAR_APP_ACTIVE_CONFIG QVAR_APP_CONFIG_BUTTON_Q1_ONLY
#endif

#ifdef __cplusplus
}
#endif

#endif /* QVAR_CONFIG_H */

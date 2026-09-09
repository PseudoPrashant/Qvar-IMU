/******************************************************************************
 * @file    qvar.c
 * @brief   ISM330BX QVAR electrode sensing and polling detector.
 ******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "qvar.h"
#include "qvar_config.h"
#include "imu.h"
#include "imu_internal.h"
#include "ism330bx_reg.h"
#include <stddef.h>
#include <stdio.h>

#define IMU_REG_DUMP_ENABLE                      1u

typedef struct
{
    uint8_t baseline_ready;
    uint8_t worn;
    uint8_t wear_confirming;
    uint8_t remove_confirming;
    uint8_t baseline_count;
    int32_t baseline_accum;
    int16_t baseline;
    int16_t avg_buffer[QVAR_APP_WEAR_AVG_BUFFER_MAX_SAMPLES];
    int32_t avg_sum;
    uint8_t avg_count;
    uint8_t avg_index;
    uint32_t wear_start_ms;
    uint32_t remove_start_ms;
    uint32_t last_change_ms;
} imu_app_qvar_wear_channel_t;

typedef struct
{
    uint8_t baseline_ready;
    uint8_t armed;
    uint8_t baseline_count;
    int32_t baseline_accum;
    int16_t baseline;
    int16_t v_prev1;
    int16_t v_prev2;
    int16_t rebound_crest;
    uint32_t last_peak_ms;
} imu_app_qvar_button_channel_t;

static imu_qvar_config_t sQvarConfig;
static uint8_t sQvarStarted;
static const qvar_app_config_t sQvarAppConfig = QVAR_APP_ACTIVE_CONFIG;

static uint8_t sImuAppQvarStarted;
static uint8_t sImuAppQvarBaselinePrinted;
static uint8_t sImuAppQvarStartupSettled;
static uint32_t sImuAppQvarStartMs;
static uint32_t sImuAppQvarLastReadMs;
static imu_app_qvar_button_channel_t sImuAppQvarButtonQ1;
static imu_app_qvar_wear_channel_t sImuAppQvarWearQ2;
static uint32_t sLastBaselineHeartbeatMs;

/* Validate the public QVAR impedance enum before writing CTRL7. */
static uint8_t imu_qvar_is_zin_valid(imu_qvar_zin_t zin)
{
    uint8_t isValid = 0u;

    switch (zin)
    {
    case IMU_QVAR_ZIN_2400_MOHM:
    case IMU_QVAR_ZIN_730_MOHM:
    case IMU_QVAR_ZIN_300_MOHM:
    case IMU_QVAR_ZIN_235_MOHM:
        isValid = 1u;
        break;

    default:
        isValid = 0u;
        break;
    }

    return isValid;
}

/* Convert wrapper QVAR input impedance selection to ST driver enum. */
static ism330bx_ah_qvar_zin_t imu_qvar_zin_to_driver(imu_qvar_zin_t zin)
{
    switch (zin)
    {
    case IMU_QVAR_ZIN_730_MOHM:
        return ISM330BX_730MOhm;

    case IMU_QVAR_ZIN_300_MOHM:
        return ISM330BX_300MOhm;

    case IMU_QVAR_ZIN_235_MOHM:
        return ISM330BX_235MOhm;

    case IMU_QVAR_ZIN_2400_MOHM:
    default:
        return ISM330BX_2400MOhm;
    }
}

/* Select which QVAR electrode input is connected to the AH/QVAR front end. */
static HAL_StatusTypeDef imu_qvar_select(uint8_t qvar1_enable,
                                         uint8_t qvar2_enable)
{
    ism330bx_ah_qvar_mode_t mode = {0};

    mode.ah_qvar1_en = (qvar1_enable != 0u) ? 1u : 0u;
    mode.ah_qvar2_en = (qvar2_enable != 0u) ? 1u : 0u;
    mode.swaps = 0u;

    return (ism330bx_ah_qvar_mode_set(imu_internal_ctx(), mode) == 0) ?
           HAL_OK : HAL_ERROR;
}

/* Print key interrupt route registers while bringing up QVAR INT2. */
static void imu_qvar_debug_dump_interrupt_registers(const char *tag)
{
#if (IMU_REG_DUMP_ENABLE != 0u)
    uint8_t ifCfg = 0u;
    uint8_t ctrl4 = 0u;
    uint8_t ctrl7 = 0u;
    uint8_t int1Ctrl = 0u;
    uint8_t int2Ctrl = 0u;
    stmdev_ctx_t *ctx = imu_internal_ctx();

    (void)ism330bx_read_reg(ctx, ISM330BX_IF_CFG, &ifCfg, 1);
    (void)ism330bx_read_reg(ctx, ISM330BX_CTRL4, &ctrl4, 1);
    (void)ism330bx_read_reg(ctx, ISM330BX_CTRL7, &ctrl7, 1);
    (void)ism330bx_read_reg(ctx, ISM330BX_INT1_CTRL, &int1Ctrl, 1);
    (void)ism330bx_read_reg(ctx, ISM330BX_INT2_CTRL, &int2Ctrl, 1);

    printf("[IMU REG] %s IF_CFG=0x%02X CTRL4=0x%02X CTRL7=0x%02X INT1_CTRL=0x%02X INT2_CTRL=0x%02X\r\n",
           (tag != NULL) ? tag : "",
           (unsigned)ifCfg,
           (unsigned)ctrl4,
           (unsigned)ctrl7,
           (unsigned)int1Ctrl,
           (unsigned)int2Ctrl);
#else
    (void)tag;
#endif
}

/* Configure QVAR data-ready interrupt; chip supports this only on INT2. */
HAL_StatusTypeDef imu_configure_qvar_data_ready_interrupt(uint8_t enable)
{
    /* INT2 route register holds the AH/QVAR DRDY route bit. */
    ism330bx_pin_int_route_t int2Route = {0};

    /* CTRL4 is used here to keep INT2 separate from INT1 and latched. */
    ism330bx_ctrl4_t ctrl4 = {0};

    /* CTRL7 also contains the AH/QVAR DRDY enable bit for INT2. */
    ism330bx_ctrl7_t ctrl7 = {0};

    /* Use the already-initialized ST register driver context. */
    stmdev_ctx_t *ctx = imu_internal_ctx();

    /* Do not touch sensor registers before WHO_AM_I/init has passed. */
    if (imu_is_started() == 0u)
    {
        return HAL_ERROR;
    }

    /* Read current INT2 routes to preserve unrelated interrupt sources. */
    if (ism330bx_pin_int2_route_get(ctx, &int2Route) != 0)
    {
        return HAL_ERROR;
    }

    imu_qvar_debug_dump_interrupt_registers((enable != 0u) ?
                                            "qvar-int-before" :
                                            "qvar-int-disable-before");

    /* Datasheet/ST driver: AH/QVAR data-ready is available only on INT2. */
    int2Route.drdy_ah_qvar = (enable != 0u) ? 1u : 0u;

    /* Use latched data-ready so the MCU can service it in task context. */
    if ((enable != 0u) &&
        (ism330bx_data_ready_mode_set(ctx, ISM330BX_DRDY_LATCHED) != 0))
    {
        return HAL_ERROR;
    }

    /* Write the updated INT2 route without changing other route bits. */
    if (ism330bx_pin_int2_route_set(ctx, int2Route) != 0)
    {
        return HAL_ERROR;
    }

    /* Read CTRL4 before changing only the INT line behavior bits we own. */
    if (ism330bx_read_reg(ctx, ISM330BX_CTRL4, (uint8_t *)&ctrl4, 1) != 0)
    {
        return HAL_ERROR;
    }

    /* Keep INT2 routed separately, not mirrored onto INT1. */
    ctrl4.int2_on_int1 = PROPERTY_DISABLE;

    /* Keep DRDY latched instead of short pulsed mode. */
    ctrl4.drdy_pulsed = PROPERTY_DISABLE;

    /* Write CTRL4 after the local bit updates. */
    if (ism330bx_write_reg(ctx, ISM330BX_CTRL4, (uint8_t *)&ctrl4, 1) != 0)
    {
        return HAL_ERROR;
    }

    /* Read CTRL7 before changing only the AH/QVAR DRDY bit. */
    if (ism330bx_read_reg(ctx, ISM330BX_CTRL7, (uint8_t *)&ctrl7, 1) != 0)
    {
        return HAL_ERROR;
    }

    /* Enable or disable the AH/QVAR DRDY generator for INT2. */
    ctrl7.int2_drdy_ah_qvar = (enable != 0u) ? PROPERTY_ENABLE : PROPERTY_DISABLE;

    /* Write CTRL7 after the local bit update. */
    if (ism330bx_write_reg(ctx, ISM330BX_CTRL7, (uint8_t *)&ctrl7, 1) != 0)
    {
        return HAL_ERROR;
    }

    imu_qvar_debug_dump_interrupt_registers((enable != 0u) ?
                                            "qvar-int-after" :
                                            "qvar-int-disable-after");

    printf("[IMU] Qvar DRDY INT2 %s%s\r\n",
           (enable != 0u) ? "EN" : "DIS",
           (enable != 0u) ? " latched" : "");
    return HAL_OK;
}

/* Consume a pending INT2 QVAR data-ready event; returns 1 when QVAR should be read. */
uint8_t imu_qvar_consume_data_ready_interrupt(void)
{
    if ((imu_is_started() == 0u) ||
        (sQvarStarted == 0u) ||
        (sQvarConfig.dataReadyInterruptEnable == 0u))
    {
        return 0u;
    }

    if (imu_internal_get_int2_pending() == 0u)
    {
        return 0u;
    }

    imu_internal_clear_int2_pending();
    return 1u;
}

/* Return 1 when QVAR owns INT2 for data-ready handling. */
uint8_t imu_qvar_owns_int2(void)
{
    return ((sQvarStarted != 0u) &&
            (sQvarConfig.dataReadyInterruptEnable != 0u)) ? 1u : 0u;
}

/* Start QVAR electrode sensing with configurable QVAR1/QVAR2 use cases. */
HAL_StatusTypeDef imu_qvar_start(const imu_qvar_config_t *config)
{
    /* CTRL8/CTRL9 filter configuration for AH/QVAR. */
    ism330bx_filt_ah_qvar_conf_t filter = {0};

    /* Local normalized enable flag for QVAR1. */
    uint8_t qvar1Requested;

    /* Local normalized enable flag for QVAR2. */
    uint8_t qvar2Requested;

    /* Use the already-initialized ST register driver context. */
    stmdev_ctx_t *ctx = imu_internal_ctx();

    /* Caller must provide a complete QVAR config. */
    if (config == NULL)
    {
        return HAL_ERROR;
    }

    /* Enable QVAR1 when either the enable flag or use-case asks for it. */
    qvar1Requested = ((config->enableQvar1 != 0u) ||
                      (config->qvar1Use != IMU_QVAR_USE_DISABLED)) ? 1u : 0u;

    /* Enable QVAR2 when either the enable flag or use-case asks for it. */
    qvar2Requested = ((config->enableQvar2 != 0u) ||
                      (config->qvar2Use != IMU_QVAR_USE_DISABLED)) ? 1u : 0u;

    /* Reject invalid state before writing any AH/QVAR registers. */
    if ((imu_is_started() == 0u) ||
        ((qvar1Requested == 0u) && (qvar2Requested == 0u)) ||
        (imu_qvar_is_zin_valid(config->zin) == 0u))
    {
        return HAL_ERROR;
    }

    /* Copy config once, then normalize the derived fields. */
    sQvarConfig = *config;

    /* Store normalized QVAR1 enable state. */
    sQvarConfig.enableQvar1 = qvar1Requested;

    /* Store normalized QVAR2 enable state. */
    sQvarConfig.enableQvar2 = qvar2Requested;

    /* Use the default settling time when caller passes 0. */
    if (sQvarConfig.settlingMs == 0u)
    {
        sQvarConfig.settlingMs = IMU_QVAR_DEFAULT_SETTLING_MS;
    }

    /* Map public HPF flag to the ST register driver structure. */
    filter.hpf = (sQvarConfig.hpfEnable != 0u) ? 1u : 0u;

    /* Map public LPF flag to the ST register driver structure. */
    filter.lpf = (sQvarConfig.lpfEnable != 0u) ? 1u : 0u;

    /* Datasheet/ST driver requires XL/GY power-down before AH_QVAR_EN=1. */
    if (imu_internal_qvar_power_down_motion() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Program AH/QVAR filter bits before enabling the QVAR channels. */
    if (ism330bx_filt_ah_qvar_conf_set(ctx, filter) != 0)
    {
        imu_internal_qvar_restore_motion_if_needed();
        return HAL_ERROR;
    }

    /* Program AH/QVAR input impedance before enabling the QVAR channels. */
    if (ism330bx_ah_qvar_zin_set(ctx,
                                 imu_qvar_zin_to_driver(sQvarConfig.zin)) != 0)
    {
        imu_internal_qvar_restore_motion_if_needed();
        return HAL_ERROR;
    }

    /* Enable the requested QVAR electrode channels; this sets AH_QVAR_EN. */
    if (imu_qvar_select(sQvarConfig.enableQvar1,
                        sQvarConfig.enableQvar2) != HAL_OK)
    {
        imu_internal_qvar_restore_motion_if_needed();
        return HAL_ERROR;
    }

    /* Optionally route AH/QVAR DRDY to INT2. */
    if (imu_configure_qvar_data_ready_interrupt(sQvarConfig.dataReadyInterruptEnable) != HAL_OK)
    {
        (void)imu_qvar_select(0u, 0u);
        imu_internal_qvar_restore_motion_if_needed();
        return HAL_ERROR;
    }

    /* Restore the caller's normal motion config after AH_QVAR_EN is set. */
    imu_internal_qvar_restore_motion_if_needed();

    /* Mark QVAR ready only after all register writes succeed. */
    sQvarStarted = 1u;
    printf("[IMU] Qvar start q1=%u/use%u q2=%u/use%u zin=%u hpf=%u lpf=%u drdy=%u settle=%u ms\r\n",
           (unsigned)sQvarConfig.enableQvar1,
           (unsigned)sQvarConfig.qvar1Use,
           (unsigned)sQvarConfig.enableQvar2,
           (unsigned)sQvarConfig.qvar2Use,
           (unsigned)sQvarConfig.zin,
           (unsigned)sQvarConfig.hpfEnable,
           (unsigned)sQvarConfig.lpfEnable,
           (unsigned)sQvarConfig.dataReadyInterruptEnable,
           (unsigned)sQvarConfig.settlingMs);
    return HAL_OK;
}

/* Stop QVAR electrode sensing and disable the AH/QVAR chain. */
HAL_StatusTypeDef imu_qvar_stop(void)
{
    if (imu_is_started() == 0u)
    {
        return HAL_ERROR;
    }

    sQvarStarted = 0u;

    if (imu_internal_qvar_power_down_motion() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (imu_configure_qvar_data_ready_interrupt(0u) != HAL_OK)
    {
        imu_internal_qvar_restore_motion_if_needed();
        return HAL_ERROR;
    }

    if (imu_qvar_select(0u, 0u) != HAL_OK)
    {
        imu_internal_qvar_restore_motion_if_needed();
        return HAL_ERROR;
    }

    imu_internal_qvar_restore_motion_if_needed();
    return HAL_OK;
}

/* Read raw QVAR1/QVAR2 values from the enabled electrodes. */
HAL_StatusTypeDef imu_qvar_read_raw(imu_qvar_raw_t *raw)
{
    HAL_StatusTypeDef status = HAL_OK;
    stmdev_ctx_t *ctx = imu_internal_ctx();

    if ((raw == NULL) || (imu_is_started() == 0u) || (sQvarStarted == 0u))
    {
        return HAL_ERROR;
    }

    raw->qvar1 = 0;
    raw->qvar2 = 0;
    raw->qvar1Valid = 0u;
    raw->qvar2Valid = 0u;

    if (sQvarConfig.enableQvar1 != 0u)
    {
        status = imu_qvar_select(1u, 0u);
        if (status == HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(sQvarConfig.settlingMs));
            status = (ism330bx_ah_qvar_raw_get(ctx, &raw->qvar1) == 0) ?
                     HAL_OK : HAL_ERROR;
        }
        if (status != HAL_OK)
        {
            return status;
        }
        raw->qvar1Valid = 1u;
    }

    if (sQvarConfig.enableQvar2 != 0u)
    {
        status = imu_qvar_select(0u, 1u);
        if (status == HAL_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(sQvarConfig.settlingMs));
            status = (ism330bx_ah_qvar_raw_get(ctx, &raw->qvar2) == 0) ?
                     HAL_OK : HAL_ERROR;
        }
        if (status != HAL_OK)
        {
            return status;
        }
        raw->qvar2Valid = 1u;
    }

    return imu_qvar_select(sQvarConfig.enableQvar1, sQvarConfig.enableQvar2);
}

/* Print raw QVAR1/QVAR2 values from the enabled electrodes. */
HAL_StatusTypeDef imu_qvar_print_raw(void)
{
    imu_qvar_raw_t raw;

    if (imu_qvar_read_raw(&raw) != HAL_OK)
    {
        printf("[IMU] Qvar read FAIL\r\n");
        return HAL_ERROR;
    }

    printf("[IMU QVAR RAW] Q1=%s%d Q2=%s%d\r\n",
           (raw.qvar1Valid != 0u) ? "" : "NA:",
           (int)raw.qvar1,
           (raw.qvar2Valid != 0u) ? "" : "NA:",
           (int)raw.qvar2);
    return HAL_OK;
}

/* 4-Sample Sliding Window Peak-to-Peak Envelope Extractor:
 * Activity[n] = max(x[n..n-3]) - min(x[n..n-3])
 * Spans 4 * 5 ms = 20.0 ms (exactly 1 full period of 50 Hz powerline hum at 200 Hz).
 * Captures the absolute peak-to-peak amplitude regardless of phase alignment.
 */
#define ENVELOPE_WINDOW_SIZE 4u

typedef struct {
    int16_t window[ENVELOPE_WINDOW_SIZE];
    uint8_t count;
    uint8_t head;
} qvar_envelope_extractor_t;

static qvar_envelope_extractor_t sEnvQ1 = {0};
static qvar_envelope_extractor_t sEnvQ2 = {0};

static void qvar_envelope_extractor_reset(qvar_envelope_extractor_t *env)
{
    if (env == NULL)
    {
        return;
    }
    memset(env->window, 0, sizeof(env->window));
    env->count = 0u;
    env->head = 0u;
}

static int32_t qvar_envelope_extractor_update(qvar_envelope_extractor_t *env, int16_t sample)
{
    if (env == NULL)
    {
        return 0;
    }
    env->window[env->head] = sample;
    env->head = (env->head + 1u) % ENVELOPE_WINDOW_SIZE;
    if (env->count < ENVELOPE_WINDOW_SIZE)
    {
        env->count++;
    }

    int16_t min_val = env->window[0];
    int16_t max_val = env->window[0];
    for (uint8_t i = 1u; i < env->count; i++)
    {
        if (env->window[i] < min_val) min_val = env->window[i];
        if (env->window[i] > max_val) max_val = env->window[i];
    }
    return (int32_t)max_val - (int32_t)min_val;
}

/* 4-Stage Robust Tap Detection Engine with Bipolarity Verification and Disturbance Squelch */
typedef enum {
    TAP_STATE_IDLE = 0,
    TAP_STATE_CONTACT,
    TAP_STATE_COOLDOWN,
    TAP_STATE_SQUELCH
} tap_state_t;

typedef struct {
    tap_state_t state;
    uint32_t start_idx;
    uint32_t peak_idx;
    int32_t peak_act;
    int16_t min_raw;
    int16_t max_raw;
    uint16_t timer;
    uint16_t quiet_counter;
    uint32_t tap_count;
    float baseline_act;
    uint8_t baseline_init;
} robust_tap_detector_t;

static robust_tap_detector_t sRobustTapDetector = {0};
static uint32_t sTapSampleIndex = 0u;

static void robust_tap_detector_reset(robust_tap_detector_t *det)
{
    if (det == NULL) return;
    det->state = TAP_STATE_IDLE;
    det->start_idx = 0u;
    det->peak_idx = 0u;
    det->peak_act = 0;
    det->min_raw = 0;
    det->max_raw = 0;
    det->timer = 0u;
    det->quiet_counter = 0u;
    det->baseline_act = 0.0f;
    det->baseline_init = 0u;
}

static uint8_t robust_tap_detector_update(robust_tap_detector_t *det,
                                         uint32_t sample_idx,
                                         int16_t raw_val,
                                         int32_t act_val,
                                         const qvar_app_config_t *cfg)
{
    uint8_t tap_confirmed = 0u;
    if (det == NULL || cfg == NULL) return 0u;

    /* Initialize adaptive baseline on first sample */
    if (det->baseline_init == 0u) {
        det->baseline_act = (float)act_val;
        det->baseline_init = 1u;
    }

    float a_rise = (cfg->tapAlphaRise > 0.0f) ? cfg->tapAlphaRise : 0.999f;
    float a_fall = (cfg->tapAlphaFall > 0.0f) ? cfg->tapAlphaFall : 0.988f;

    /* Update adaptive baseline in IDLE or SQUELCH */
    if (cfg->tapAdaptiveBaselineEnable != 0u) {
        if (det->state == TAP_STATE_IDLE) {
            if ((float)act_val > det->baseline_act) {
                det->baseline_act = a_rise * det->baseline_act + (1.0f - a_rise) * (float)act_val;
            } else {
                det->baseline_act = a_fall * det->baseline_act + (1.0f - a_fall) * (float)act_val;
            }
        } else if (det->state == TAP_STATE_SQUELCH) {
            /* Slowly track towards ambient activity in squelch */
            det->baseline_act = 0.995f * det->baseline_act + 0.005f * (float)act_val;
        }
    }

    int32_t p_delta = cfg->tapPressDeltaRaw ? cfg->tapPressDeltaRaw : 1100L;
    int32_t r_delta = cfg->tapReleaseDeltaRaw ? cfg->tapReleaseDeltaRaw : 600L;
    int32_t min_peak = cfg->tapMinPeakActRaw ? cfg->tapMinPeakActRaw : 3300L;

    int32_t th_press;
    int32_t th_release;
    if (cfg->tapAdaptiveBaselineEnable != 0u) {
        th_press = (int32_t)det->baseline_act + p_delta;
        th_release = (int32_t)det->baseline_act + r_delta;
    } else {
        th_press = cfg->tapPressThresholdRaw ? cfg->tapPressThresholdRaw : 3300L;
        th_release = cfg->tapReleaseThresholdRaw ? cfg->tapReleaseThresholdRaw : 2650L;
    }

    uint16_t min_dur = cfg->tapMinDurationSamples ? cfg->tapMinDurationSamples : 3u;
    uint16_t max_dur = cfg->tapMaxDurationSamples ? cfg->tapMaxDurationSamples : 70u;
    uint16_t cooldown = cfg->tapCooldownSamples ? cfg->tapCooldownSamples : 25u;
    uint16_t squelch_timer = cfg->tapSquelchTimerSamples ? cfg->tapSquelchTimerSamples : 50u;
    uint16_t squelch_quiet = cfg->tapSquelchQuietSamples ? cfg->tapSquelchQuietSamples : 15u;
    int16_t bip_min = cfg->tapBipolarMinRaw ? cfg->tapBipolarMinRaw : -500;
    int16_t bip_max = cfg->tapBipolarMaxRaw ? cfg->tapBipolarMaxRaw : 500;

    /* Track continuous quiet samples */
    if (act_val < th_release) {
        det->quiet_counter++;
    } else {
        det->quiet_counter = 0u;
    }

    switch (det->state)
    {
    case TAP_STATE_IDLE:
        if (act_val >= th_press) {
            det->state = TAP_STATE_CONTACT;
            det->start_idx = sample_idx;
            det->peak_idx = sample_idx;
            det->peak_act = act_val;
            det->min_raw = raw_val;
            det->max_raw = raw_val;
        }
        break;

    case TAP_STATE_CONTACT: {
        uint32_t dur = sample_idx - det->start_idx;

        if (act_val > det->peak_act) {
            det->peak_act = act_val;
            det->peak_idx = sample_idx;
        }
        if (raw_val < det->min_raw) det->min_raw = raw_val;
        if (raw_val > det->max_raw) det->max_raw = raw_val;

        /* Check max duration: hold or long disturbance */
        if (dur > max_dur) {
            det->state = TAP_STATE_SQUELCH;
            det->timer = squelch_timer;
            break;
        }

        /* Check for contact release */
        if (act_val < th_release) {
            if (dur < min_dur) {
                /* Glitch rejection: brief 1-2 sample spike, ignore without squelching */
                det->state = TAP_STATE_IDLE;
            } else if (dur <= max_dur) {
                uint8_t is_bipolar = (det->min_raw <= bip_min && det->max_raw >= bip_max) ? 1u : 0u;
                uint8_t is_strong = (det->peak_act >= min_peak) ? 1u : 0u;

                if (is_bipolar != 0u && is_strong != 0u) {
                    det->tap_count++;
                    tap_confirmed = 1u;
                    det->state = TAP_STATE_COOLDOWN;
                    det->timer = cooldown;
                } else {
                    if (is_strong == 0u && is_bipolar != 0u) {
                        /* Weak ripple, return to IDLE */
                        det->state = TAP_STATE_IDLE;
                    } else {
                        /* Unipolar DC blast / electrostatic transient -> enter squelch */
                        det->state = TAP_STATE_SQUELCH;
                        det->timer = squelch_timer;
                    }
                }
            }
        }
        break;
    }

    case TAP_STATE_COOLDOWN:
        if (det->timer > 0u) det->timer--;
        if (det->timer == 0u) {
            det->state = TAP_STATE_IDLE;
            if (act_val >= th_press) {
                det->state = TAP_STATE_CONTACT;
                det->start_idx = sample_idx;
                det->peak_idx = sample_idx;
                det->peak_act = act_val;
                det->min_raw = raw_val;
                det->max_raw = raw_val;
            }
        }
        break;

    case TAP_STATE_SQUELCH:
        if (det->timer > 0u) det->timer--;
        if (act_val >= th_release) {
            det->timer = squelch_timer;
        }
        /* Re-arm only after squelch timer elapsed AND calm baseline observed */
        if (det->timer == 0u && det->quiet_counter >= squelch_quiet) {
            det->state = TAP_STATE_IDLE;
        }
        break;
    }

    return tap_confirmed;
}

static int32_t imu_app_qvar_abs_delta(int16_t value, int16_t baseline)
{
    int32_t delta = (int32_t)value - (int32_t)baseline;

    return (delta < 0) ? -delta : delta;
}

static void imu_app_qvar_wear_reset(imu_app_qvar_wear_channel_t *channel)
{
    if (channel == NULL)
    {
        return;
    }

    channel->baseline_ready = 0u;
    channel->worn = 0u;
    channel->wear_confirming = 0u;
    channel->remove_confirming = 0u;
    channel->baseline_count = 0u;
    channel->baseline_accum = 0;
    channel->baseline = 0;
    channel->avg_sum = 0;
    channel->avg_count = 0u;
    channel->avg_index = 0u;
    channel->wear_start_ms = 0u;
    channel->remove_start_ms = 0u;
    channel->last_change_ms = 0u;
}

static void imu_app_qvar_button_reset(imu_app_qvar_button_channel_t *channel)
{
    if (channel == NULL)
    {
        return;
    }

    channel->baseline_ready = 0u;
    channel->armed = 1u;
    channel->baseline_count = 0u;
    channel->baseline_accum = 0;
    channel->baseline = 0;
    channel->v_prev1 = 0;
    channel->v_prev2 = 0;
    channel->rebound_crest = 0;
    channel->last_peak_ms = 0u;
}

static int16_t imu_app_qvar_track_baseline(int16_t current_baseline, int16_t value)
{
    int32_t next_baseline;

    if (sQvarAppConfig.baselineTrackDiv <= 1L)
    {
        return value;
    }

    next_baseline = (((int32_t)current_baseline *
                      (sQvarAppConfig.baselineTrackDiv - 1L)) +
                     (int32_t)value) /
                    sQvarAppConfig.baselineTrackDiv;
    return (int16_t)next_baseline;
}

static int16_t imu_app_qvar_wear_average_sample(imu_app_qvar_wear_channel_t *channel,
                                                int16_t value)
{
    uint8_t sampleLimit = sQvarAppConfig.wearAvgSamples;

    if (sampleLimit == 0u)
    {
        sampleLimit = 1u;
    }
    if (sampleLimit > QVAR_APP_WEAR_AVG_BUFFER_MAX_SAMPLES)
    {
        sampleLimit = QVAR_APP_WEAR_AVG_BUFFER_MAX_SAMPLES;
    }

    if (channel->avg_count < sampleLimit)
    {
        channel->avg_buffer[channel->avg_count] = value;
        channel->avg_sum += value;
        channel->avg_count++;
    }
    else
    {
        channel->avg_sum -= channel->avg_buffer[channel->avg_index];
        channel->avg_buffer[channel->avg_index] = value;
        channel->avg_sum += value;
        channel->avg_index++;
        if (channel->avg_index >= sampleLimit)
        {
            channel->avg_index = 0u;
        }
    }

    return (int16_t)(channel->avg_sum / (int32_t)channel->avg_count);
}

static void imu_app_qvar_wear_process_q2(imu_app_qvar_wear_channel_t *channel,
                                         int16_t value,
                                         uint8_t valid,
                                         uint32_t now_ms)
{
    int32_t delta;
    int16_t filtered_value;

    if ((channel == NULL) || (valid == 0u))
    {
        return;
    }

    if (channel->baseline_ready == 0u)
    {
        channel->baseline_accum += value;
        channel->baseline_count++;

        if (channel->baseline_count >= sQvarAppConfig.baselineSamples)
        {
            channel->baseline =
                (int16_t)(channel->baseline_accum / (int32_t)channel->baseline_count);
            channel->baseline_ready = 1u;
            printf("[IMU QVAR] Q2 wear baseline=%d on_th=%ld off_th=%ld on_ms=%lu off_ms=%lu\r\n",
                   (int)channel->baseline,
                   (long)sQvarAppConfig.qvar2WearThresholdRaw,
                   (long)sQvarAppConfig.qvar2WearReleaseRaw,
                   (unsigned long)sQvarAppConfig.qvar2WearOnConfirmMs,
                   (unsigned long)sQvarAppConfig.qvar2WearOffConfirmMs);
        }
        return;
    }

    filtered_value = imu_app_qvar_wear_average_sample(channel, value);
    delta = imu_app_qvar_abs_delta(filtered_value, channel->baseline);

    if (channel->worn == 0u)
    {
        if (delta <= sQvarAppConfig.qvar2WearReleaseRaw)
        {
            channel->baseline = imu_app_qvar_track_baseline(channel->baseline, value);
            channel->wear_confirming = 0u;
            return;
        }

        if (delta >= sQvarAppConfig.qvar2WearThresholdRaw)
        {
            if (channel->wear_confirming == 0u)
            {
                channel->wear_confirming = 1u;
                channel->wear_start_ms = now_ms;
            }
            if ((now_ms - channel->wear_start_ms) >= sQvarAppConfig.qvar2WearOnConfirmMs)
            {
                channel->worn = 1u;
                channel->wear_confirming = 0u;
                channel->remove_confirming = 0u;
                channel->last_change_ms = now_ms;
                printf("[IMU QVAR] Q2 WEAR ON\r\n");
            }
        }
        else
        {
            channel->wear_confirming = 0u;
        }
        return;
    }

    if (delta <= sQvarAppConfig.qvar2WearReleaseRaw)
    {
        if (channel->remove_confirming == 0u)
        {
            channel->remove_confirming = 1u;
            channel->remove_start_ms = now_ms;
        }
        if ((now_ms - channel->remove_start_ms) >= sQvarAppConfig.qvar2WearOffConfirmMs)
        {
            channel->worn = 0u;
            channel->remove_confirming = 0u;
            channel->wear_confirming = 0u;
            channel->last_change_ms = now_ms;
            printf("[IMU QVAR] Q2 WEAR OFF\r\n");
        }
    }
    else
    {
        channel->remove_confirming = 0u;
    }
}

static void imu_app_qvar_button_process_q1(imu_app_qvar_button_channel_t *channel,
                                           int16_t value,
                                           uint8_t valid,
                                           uint32_t now_ms,
                                           uint8_t button_allowed)
{
    int32_t delta_curr;
    int32_t delta_p1;
    int16_t v_curr;
    int16_t v_p1;
    int16_t v_p2;

    if ((channel == NULL) || (valid == 0u))
    {
        return;
    }

    if (button_allowed == 0u)
    {
        imu_app_qvar_button_reset(channel);
        return;
    }

    if (channel->baseline_ready == 0u)
    {
        channel->baseline_accum += value;
        channel->baseline_count++;

        if (channel->baseline_count >= sQvarAppConfig.baselineSamples)
        {
            channel->baseline =
                (int16_t)(channel->baseline_accum / (int32_t)channel->baseline_count);
            channel->baseline_ready = 1u;
            channel->v_prev1 = value;
            channel->v_prev2 = value;
            channel->rebound_crest = value;
            printf("[IMU QVAR] Q1 button baseline=%d press_th=%ld release=%ld peak=%ld\r\n",
                   (int)channel->baseline,
                   (long)sQvarAppConfig.qvar1ButtonThresholdRaw,
                   (long)sQvarAppConfig.qvar1ButtonReleaseRaw,
                   (long)sQvarAppConfig.qvar1ButtonMinPeakRaw);
        }
        return;
    }

    v_curr = value;
    v_p1 = channel->v_prev1;
    v_p2 = channel->v_prev2;
    delta_curr = (int32_t)v_curr - (int32_t)channel->baseline;
    delta_p1 = (int32_t)v_p1 - (int32_t)channel->baseline;

    /* Dynamic baseline tracking when idle (within +-600 LSB) */
    if ((delta_curr > -sQvarAppConfig.qvar1ButtonThresholdRaw) &&
        (delta_curr < sQvarAppConfig.qvar1ButtonThresholdRaw))
    {
        channel->baseline = imu_app_qvar_track_baseline(channel->baseline, v_curr);
    }

    uint8_t is_neg_peak = 0u;
    uint8_t is_pos_peak = 0u;

    /* Negative plunge peak: local minimum <= -600 LSB */
    if ((v_p1 <= v_p2) && (v_p1 < v_curr) &&
        (delta_p1 <= -sQvarAppConfig.qvar1ButtonMinPeakRaw))
    {
        is_neg_peak = 1u;
    }
    /* Positive crest peak: local maximum >= +600 LSB */
    else if ((v_p1 >= v_p2) && (v_p1 > v_curr) &&
             (delta_p1 >= sQvarAppConfig.qvar1ButtonMinPeakRaw))
    {
        is_pos_peak = 1u;
    }

    if ((is_neg_peak != 0u) || (is_pos_peak != 0u))
    {
        uint32_t cand_peak_ms = (now_ms >= sQvarAppConfig.readPeriodMs) ?
            (now_ms - sQvarAppConfig.readPeriodMs) : now_ms;

        if ((cand_peak_ms - channel->last_peak_ms) >= 70u)
        {
            channel->last_peak_ms = cand_peak_ms;
            printf("[IMU QVAR] Q1 PEAK (%s val=%d delta=%+ld LSB / %+.1f mV)\r\n",
                   (is_neg_peak != 0u) ? "NEG" : "POS",
                   (int)v_p1,
                   (long)delta_p1,
                   (float)delta_p1 / 78.0f);
        }
    }

    channel->v_prev2 = channel->v_prev1;
    channel->v_prev1 = v_curr;
}

static void imu_app_qvar_process_sample(const imu_qvar_raw_t *raw, uint32_t now_ms)
{
    uint8_t previousWearState;
    uint8_t buttonAllowed = 0u;
    uint8_t wearEnabled;
    uint8_t buttonEnabled;

    if (raw == NULL)
    {
        return;
    }

    wearEnabled = (sQvarConfig.qvar2Use == IMU_QVAR_USE_WEAR) ? 1u : 0u;
    buttonEnabled = (sQvarConfig.qvar1Use == IMU_QVAR_USE_BUTTON) ? 1u : 0u;
    previousWearState = sImuAppQvarWearQ2.worn;

    if (wearEnabled != 0u)
    {
        imu_app_qvar_wear_process_q2(&sImuAppQvarWearQ2,
                                     raw->qvar2,
                                     raw->qvar2Valid,
                                     now_ms);
    }

    if (buttonEnabled != 0u)
    {
        if (previousWearState != sImuAppQvarWearQ2.worn)
        {
            imu_app_qvar_button_reset(&sImuAppQvarButtonQ1);
        }

        if (wearEnabled == 0u)
        {
            buttonAllowed = 1u;
        }
        else if ((sImuAppQvarWearQ2.worn != 0u) &&
                 ((now_ms - sImuAppQvarWearQ2.last_change_ms) >=
                  sQvarAppConfig.wearButtonArmDelayMs))
        {
            buttonAllowed = 1u;
        }
        else
        {
            buttonAllowed = 0u;
        }

        imu_app_qvar_button_process_q1(&sImuAppQvarButtonQ1,
                                       raw->qvar1,
                                       raw->qvar1Valid,
                                       now_ms,
                                       buttonAllowed);
    }

    if ((sImuAppQvarBaselinePrinted == 0u) &&
        ((buttonEnabled == 0u) || (sImuAppQvarButtonQ1.baseline_ready != 0u)) &&
        ((wearEnabled == 0u) || (sImuAppQvarWearQ2.baseline_ready != 0u)))
    {
        sImuAppQvarBaselinePrinted = 1u;
        printf("[IMU QVAR] detection ready: Q1 button=%u Q2 wear=%u\r\n",
               (unsigned)buttonEnabled,
               (unsigned)wearEnabled);
    }

    if ((buttonEnabled != 0u) && (sImuAppQvarButtonQ1.baseline_ready != 0u))
    {
        if ((now_ms - sLastBaselineHeartbeatMs) >= 5000u)
        {
            sLastBaselineHeartbeatMs = now_ms;
            printf("[IMU QVAR] Q1 button baseline=%d press_th=%ld release=%ld peak=%ld\r\n",
                   (int)sImuAppQvarButtonQ1.baseline,
                   (long)sQvarAppConfig.qvar1ButtonThresholdRaw,
                   (long)sQvarAppConfig.qvar1ButtonReleaseRaw,
                   (long)sQvarAppConfig.qvar1ButtonMinPeakRaw);
        }
    }
}

static void imu_app_qvar_print_raw_sample(const imu_qvar_raw_t *raw)
{
    if (raw == NULL)
    {
        return;
    }

    if (sQvarAppConfig.rawLogEnable == 0u)
    {
        return;
    }

    printf("[IMU QVAR RAW] Q1=%s%d Q2=%s%d\r\n",
           (raw->qvar1Valid != 0u) ? "" : "NA:",
           (int)raw->qvar1,
           (raw->qvar2Valid != 0u) ? "" : "NA:",
           (int)raw->qvar2);
}

/* Run the QVAR polling app: Q2 wear sensing and Q1 button detection. */
void imu_qvar_app_task(void)
{
    uint32_t now_ms;
    imu_qvar_raw_t raw;

    if (sQvarAppConfig.enable == 0u)
    {
        return;
    }

    if (imu_is_started() == 0u)
    {
        return;
    }

    if (sImuAppQvarStarted == 0u)
    {
        imu_qvar_config_t config = sQvarAppConfig.qvarConfig;

        if (imu_qvar_start(&config) == HAL_OK)
        {
            sImuAppQvarStarted = 1u;
            sImuAppQvarStartMs = (xTaskGetTickCount() * portTICK_PERIOD_MS);
            sImuAppQvarLastReadMs = sImuAppQvarStartMs;
            sImuAppQvarStartupSettled = 0u;
            sImuAppQvarBaselinePrinted = 0u;
            imu_app_qvar_button_reset(&sImuAppQvarButtonQ1);
            imu_app_qvar_wear_reset(&sImuAppQvarWearQ2);
            qvar_envelope_extractor_reset(&sEnvQ1);
            qvar_envelope_extractor_reset(&sEnvQ2);
            robust_tap_detector_reset(&sRobustTapDetector);
            sTapSampleIndex = 0u;
            printf("[IMU TEST] Qvar polling started q1_button=%u q2_wear=%u\r\n",
                   (config.qvar1Use == IMU_QVAR_USE_BUTTON) ? 1u : 0u,
                   (config.qvar2Use == IMU_QVAR_USE_WEAR) ? 1u : 0u);
        }
        else
        {
            printf("[IMU TEST] Qvar start FAIL\r\n");
            return;
        }
    }

    now_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((sQvarAppConfig.readPeriodMs > (portTICK_PERIOD_MS / 2u)) &&
        ((now_ms - sImuAppQvarLastReadMs) < (sQvarAppConfig.readPeriodMs - (portTICK_PERIOD_MS / 2u))))
    {
        return;
    }

    sImuAppQvarLastReadMs = now_ms;
    if (imu_qvar_read_raw(&raw) != HAL_OK)
    {
        printf("[IMU] Qvar read FAIL\r\n");
        return;
    }

    int16_t q1_raw_in = raw.qvar1;
    int32_t q1_act = 0;

    /* Apply 4-sample peak-to-peak envelope extractor */
    if (raw.qvar1Valid != 0u)
    {
        q1_act = qvar_envelope_extractor_update(&sEnvQ1, raw.qvar1);
        raw.qvar1 = (int16_t)q1_act;
    }
    if (raw.qvar2Valid != 0u)
    {
        raw.qvar2 = (int16_t)qvar_envelope_extractor_update(&sEnvQ2, raw.qvar2);
    }

    imu_app_qvar_print_raw_sample(&raw);

    if (sImuAppQvarStartupSettled == 0u)
    {
        if ((now_ms - sImuAppQvarStartMs) < sQvarAppConfig.startupSettleMs)
        {
            return;
        }

        sImuAppQvarStartupSettled = 1u;
        sImuAppQvarBaselinePrinted = 0u;
        imu_app_qvar_button_reset(&sImuAppQvarButtonQ1);
        imu_app_qvar_wear_reset(&sImuAppQvarWearQ2);
        qvar_envelope_extractor_reset(&sEnvQ1);
        qvar_envelope_extractor_reset(&sEnvQ2);
        robust_tap_detector_reset(&sRobustTapDetector);
        sTapSampleIndex = 0u;
        printf("[IMU QVAR] startup settle done after %lu ms; baseline learning starts now\r\n",
               (unsigned long)sQvarAppConfig.startupSettleMs);
        return;
    }

    /* 4-Stage Robust Tap Detector Evaluation */
    sTapSampleIndex++;
    if (raw.qvar1Valid != 0u)
    {
        if (robust_tap_detector_update(&sRobustTapDetector, sTapSampleIndex, q1_raw_in, q1_act, &sQvarAppConfig))
        {
            uint32_t tap_dur_ms = (sTapSampleIndex >= sRobustTapDetector.start_idx) ?
                                  (sTapSampleIndex - sRobustTapDetector.start_idx) * 5u : 0u;
            printf("[IMU QVAR] TAP DETECTED #%lu (dur=%lu ms, peak=%ld LSB / %.1f mV, base=%.0f LSB, raw_span=[%d, %d])\r\n",
                   (unsigned long)sRobustTapDetector.tap_count,
                   (unsigned long)tap_dur_ms,
                   (long)sRobustTapDetector.peak_act,
                   (float)sRobustTapDetector.peak_act / 78.0f,
                   sRobustTapDetector.baseline_act,
                   (int)sRobustTapDetector.min_raw,
                   (int)sRobustTapDetector.max_raw);
        }
    }

    imu_app_qvar_process_sample(&raw, now_ms);
}

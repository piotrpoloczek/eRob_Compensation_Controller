/*
 * @Author:
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 *
 * Corrected + integrated:
 * - FIX extern vs static for p_file_log
 * - FIX enum order
 * - Use torque_assist.h as the boost engine
 * - Keep TS_SIGN direction fix
 * - Keep bias snapshot on enable to prevent self-motion
 * - Safe mc_mode_name array + static_assert
 * - Add robust torque sensor debug print
 * - Add 2nd assist mode with friction comp ON
 */

#include "motor_control.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <termios.h>
#include <time.h>

#include "friction_identify.h"
#include "yzdn_math.h"
#include "lpf.h"
#include "log.h"
#include "tf_2rd.h"
#include "freq_scan.h"
#include "slop.h"

#include <cmath>
#include <cstdint>

#include "torque_assist.h"

// ---------------------------------------------
// External app control flag (likely in main)
// ---------------------------------------------
extern uint8_t exit_app;

// module parameters
#define MOTOR_CURRENT_BASE       (5.4f)       // motor current base = rated current A
#define MOTOR_ENCODER_SIM        (524288.0f)  // encoder lines
#define MOTOR_ENCODER_SIM_DIV    (1.0f/MOTOR_ENCODER_SIM)
#define MOTOR_K_RADPS2SIMPS      (MOTOR_ENCODER_SIM * 1.0F / YZDN_MATH_2PI)

#define MOTOR_UNLIMITED_ACCEL    (1000.0F)    // rpm/s
#define MOTOR_LIMITED_ACCEL      (10.0f)      // rpm/s

// ======================================================
// Torque sensor assist debug options
// ======================================================
#ifndef TS_DEBUG_RATIO_CROSSCHECK
#define TS_DEBUG_RATIO_CROSSCHECK 0
#endif

#ifndef TS_RATIO_FIELD
#define TS_RATIO_FIELD torque_ratio_permil
#endif

#ifndef TS_RATED_FIELD
#define TS_RATED_FIELD rated_torque_mN_m
#endif

// ---------------- TORQUE SENSOR ASSIST ----------------

static float g_touch_sign = 0.0f;


// Enable flag (manual toggle with 't')
static uint8_t g_en_torque_boost = 0x00;

// Tare state
static int32_t g_ts_zero_mN_m = 0;
static uint8_t g_ts_tare_done = 0;

// Direction fix
static constexpr float TS_SIGN = -1.0f;

// Bias removal for "zero effort feel"
static float   g_ts_bias_Nm = 0.0f;
static uint8_t g_ts_bias_valid = 0;

// Assist config/state
static TorqueAssistConfig g_ts_cfg;
static TorqueAssistState  g_ts_state;

//////////////

// ------------------------------
// TS touch-pulse test mode
// ------------------------------

// intent detection threshold (Nm)
static float g_touch_on_Nm = 0.30f;     // tune 0.2~0.6

// consider axis "still"
static float g_touch_still_rpm = 0.6f;  // tune 0.3~1.0

// pulse parameters
static float g_touch_pulse_mA = 120.0f; // fixed assist current
static float g_touch_pulse_s  = 2.0f;   // your requirement

// internal countdown
static float g_touch_left_s = 0.0f;

/////////////

// Simple clamp/sign local
static inline float clampf_local(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float signf_local(float x)
{
    return (x > 0.0f) ? 1.0f : (x < 0.0f) ? -1.0f : 0.0f;
}

// Auto tare when still
static void torque_sensor_tare_update(float speed_rpm, int32_t torque_mN_m_raw)
{
    const float still_rpm = 0.5f;
    if (std::fabs(speed_rpm) > still_rpm) {
        return;
    }

    const int N = 500;  // ~0.5s if called at 1kHz
    static int64_t acc = 0;
    static int cnt = 0;

    if (g_ts_tare_done) return;

    acc += torque_mN_m_raw;
    cnt++;

    if (cnt >= N) {
        g_ts_zero_mN_m = (int32_t)(acc / cnt);
        g_ts_tare_done = 1;
        acc = 0;
        cnt = 0;

        ECAT_LOG("[TS] tare set to %d mN·m (%.3f N·m)\n",
                 g_ts_zero_mN_m, g_ts_zero_mN_m / 1000.0f);
    }
}

// Convert to compensated N·m
static inline float torque_sensor_comp_Nm(int32_t torque_mN_m_raw)
{
    return (float)(torque_mN_m_raw - g_ts_zero_mN_m) * 1e-3f;
}


// ------------------------------------------------------
// Control mode
// ------------------------------------------------------
typedef enum
{
  MC_MODE_OFF = 0X00,
  MC_MODE_COMP_FRIC_PT,
  MC_MODE_COMP_GRAVITY_PT,
  MC_MODE_COMP_PT,
  MC_MODE_COMP_PV,
  MC_MODE_FRIC_IDEN,
  MC_MODE_COMP_PV_IDEN_SIN,
  MC_MODE_COMP_PV_IDEN_SQUARE,
  MC_MODE_COMP_PV_IDEN_SIN_2,
  MC_MODE_COMP_PV_IDEN_SQUARE_2,

  // Assist modes (PT-based)
  MC_MODE_TS_ASSIST,        // pure assist
  MC_MODE_TS_ASSIST_FRIC,   // assist + friction comp ON (recommended)
  // --- NEW: simple intent test ---
  MC_MODE_TS_TOUCH_PULSE,

  MC_MODE_NUM,
} MOTOR_CTRL_mode_e;

static const char* mc_mode_name[] = {
    "MC_MODE_OFF",
    "MC_MODE_COMP_FRIC_PT",
    "MC_MODE_COMP_GRAVITY_PT",
    "MC_MODE_COMP_PT",
    "MC_MODE_COMP_PV",
    "MC_MODE_FRIC_IDEN",
    "MC_MODE_COMP_PV_IDEN_SIN",
    "MC_MODE_COMP_PV_IDEN_SQUARE",
    "MC_MODE_COMP_PV_IDEN_SIN_2",
    "MC_MODE_COMP_PV_IDEN_SQUARE_2",
    "MC_MODE_TS_ASSIST",
    "MC_MODE_TS_ASSIST_FRIC",
    "MC_MODE_TS_TOUCH_PULSE"
};

static_assert(sizeof(mc_mode_name)/sizeof(mc_mode_name[0]) == MC_MODE_NUM,
              "mc_mode_name size must match MC_MODE_NUM");


// motor control structure
typedef struct
{
    float dt;
    float dt_2;
    float time;
    struct timespec time_now;
    struct timespec time_pre;
    MOTOR_CTRL_mode_e mode;
    uint16_t  tick;

    // speed control parameters
    float speed_ref;     // rpm
    float speed_max;

    // motor control parameters
    float current_ref;         // mA
    float current_offset;      // mA (final)
    float current_offset_user; // mA
    float current_max;         // mA

    // acceleration limit
    float accel_limit;     // rad/s^2
    float accel_limit_rpm; // rpm/s

    // feedback
    float current_fbk;     // mA
    float speed_fbk;       // rad/s
    float speed_fbk_rpm;   // rpm
    float angle_fbk;       // rad
    float angle_fbk_deg;   // deg

    // friction and gravity compensation
    uint8_t en_fric_comp;
    uint8_t en_gravity_comp;
    uint8_t en_fric_iden;
    float current_fric;
    float current_gravity;

    // motor base
    float current_base;
    ST_LPF lpf_current;

    // motor feedback raw
    txpdo_t fbk_raw;
    rxpdo_t cmd_raw;

    // delay
    float delay;

    // speed loop model
    TF_2RD_t tf_spd_d;
    TF_2RD_t tf_spd_c;
    SLOP_t   slop_spd;

    // --- torque sensor assist ---
    uint8_t en_torque_boost;
    float torque_sens_Nm;
    float torque_user_Nm;
    float current_boost;

} MOTOR_CTRL_t, *MOTOR_CTRL_h;

static void MOTOR_CTRL_key(void);

// globals
MOTOR_CTRL_t g_motor_ctrl;

// IMPORTANT: not static (matches extern in header)
FILE *p_file_log = NULL;
static FILE *p_data = NULL;


// ------------------------------------------------------
// Init
// ------------------------------------------------------
void MOTOR_CTRL_init(void)
{
    MOTOR_CTRL_t *h = &g_motor_ctrl;

    h->current_ref = 0.0f;
    h->current_base = MOTOR_CURRENT_BASE;
    h->speed_ref = 0.0f;
    h->current_offset_user = 0.0f;
    h->current_offset = 0.0f;
    h->current_max = 5400.0f;

    lpf_init(&h->lpf_current, 0.2f);

    Friction_Identify_init();
    FREQ_SCAN_init();
    SLOP_init(&h->slop_spd, 0.0001f);

    h->en_torque_boost = 0x00;
    h->torque_sens_Nm  = 0.0f;
    h->torque_user_Nm  = 0.0f;
    h->current_boost   = 0.0f;

    h->en_fric_comp = 0x00;
    h->en_gravity_comp = 0x00;
    h->en_fric_iden = 0x00;

    h->mode = MC_MODE_OFF;

    h->cmd_raw.controlword = 0x000F;
    h->cmd_raw.torque_slope = 0x00;
    h->cmd_raw.max_torque = h->current_max / h->current_base;
    h->cmd_raw.padding = 0x00;
    h->cmd_raw.speed_limit = MOTOR_ENCODER_SIM;
    h->cmd_raw.speed_limit_2 = MOTOR_ENCODER_SIM;
    h->cmd_raw.accelerate_up = 2000000;
    h->cmd_raw.accelerate_down = 2000000;

    TF_2RD_discrete_init(&h->tf_spd_d,
                         0.0121667f, 0.0243333f, 0.0121667f,
                         1.0f, -1.7175803f, 0.7660727f);

    // log
    p_file_log = fopen("log.txt", "w");
    if (p_file_log == NULL) {
        ECAT_LOG("can not open log.txt\n");
        return;
    }

    p_data = fopen("data.txt", "w");
    if (p_data == NULL) {
        ECAT_LOG("can not open data.txt\n");
        return;
    }

    uint32_t accel_limit = MOTOR_LIMITED_ACCEL * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;
    uint32_t accel_unlimit = MOTOR_UNLIMITED_ACCEL * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;
    ECAT_LOG("accel_limit:%.1f rpm/s, %d cnt/s\n", MOTOR_LIMITED_ACCEL, accel_limit);
    ECAT_LOG("accel_unlimit:%.1f rpm/s, %d cnt/s\n", MOTOR_UNLIMITED_ACCEL, accel_unlimit);

    clock_gettime(CLOCK_MONOTONIC, &h->time_now);
    h->time_pre = h->time_now;

    // ---------------- TS reset ----------------
    g_ts_zero_mN_m = 0;
    g_ts_tare_done = 0;

    g_ts_bias_Nm = 0.0f;
    g_ts_bias_valid = 0;

    g_en_torque_boost = 0x00;

    g_ts_state.active = false;
    g_ts_state.ts_user_filt = 0.0f;

    // Touch-pulse reset
    g_touch_left_s = 0.0f;


    // ---------------- Assist tuning ----------------
    g_ts_cfg.on_Nm  = 1.00f;
    g_ts_cfg.off_Nm = 0.50f;

    g_ts_cfg.stiction_mA = 60.0f;
    g_ts_cfg.gain_mA_per_Nm = 180.0f;
    g_ts_cfg.max_mA = 600.0f;

    g_ts_cfg.filt_alpha = 0.15f;

    h->tick = 0;
    h->delay = 0.0f;
    h->time = 0.0f;
}


// ------------------------------------------------------
// Step
// ------------------------------------------------------
void MOTOR_CTRL_step(float dt)
{
    MOTOR_CTRL_h h = &g_motor_ctrl;
    h->dt = dt;

    txpdo_t *h_tx = &h->fbk_raw;
    rxpdo_t *h_rx = &h->cmd_raw;

    h->tick++;

    // ---------------- Mode logic ----------------
    switch(h->mode)
    {
        case MC_MODE_OFF:
        {
            h->cmd_raw.mode_of_operation = 0x04; // PT
            h->speed_ref = 0.0f;
            h->current_ref = 0.0f;
            h->current_offset_user = 0.0f;
            h->current_offset = 0.0f;
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x00;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        case MC_MODE_FRIC_IDEN:
        {
            h->cmd_raw.mode_of_operation = 0x03; // PV
            h->en_fric_iden = 0x01;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x00;
            Friction_Identify_step(h->dt, h->speed_fbk_rpm, h->lpf_current.output, 0.0f);
            h->speed_ref = Friction_Identify_get_spd_ref();
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        case MC_MODE_COMP_PV_IDEN_SQUARE:
        {
            h->cmd_raw.mode_of_operation = 0x03; // PV
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;

            h->delay += h->dt;
            float period = 6.0f;
            if(h->delay > period) h->delay = 0.0f;

            h->speed_ref = (h->delay > 0.5f * period) ? 2.0f : -2.0f;
            break;
        }

        case MC_MODE_COMP_PV_IDEN_SIN:
        {
            h->cmd_raw.mode_of_operation = 0x03; // PV
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;

            FREQ_SCAN_step(dt);
            h->speed_ref = FREQ_SCAN_get_input();
            break;
        }

        case MC_MODE_COMP_PV_IDEN_SQUARE_2:
        {
            h->cmd_raw.mode_of_operation = 0x03; // PV
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_LIMITED_ACCEL;

            h->delay += h->dt;
            float period = 6.0f;
            if(h->delay > period) h->delay = 0.0f;

            h->speed_ref = (h->delay > 0.5f * period) ? 2.0f : -2.0f;
            break;
        }

        case MC_MODE_COMP_PV_IDEN_SIN_2:
        {
            h->cmd_raw.mode_of_operation = 0x03; // PV
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_LIMITED_ACCEL;

            FREQ_SCAN_step(dt);
            h->speed_ref = FREQ_SCAN_get_input();
            break;
        }

        case MC_MODE_COMP_PT:
        {
            h->cmd_raw.mode_of_operation = 0x04; // PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        case MC_MODE_COMP_FRIC_PT:
        {
            h->cmd_raw.mode_of_operation = 0x04; // PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x00;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        case MC_MODE_COMP_GRAVITY_PT:
        {
            h->cmd_raw.mode_of_operation = 0x04; // PT
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        case MC_MODE_COMP_PV:
        {
            h->cmd_raw.mode_of_operation = 0x03; // PV
            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x01;
            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        // ------------------------------
        // Torque-only assist mode (pure)
        // ------------------------------
        case MC_MODE_TS_ASSIST:
        {
            h->cmd_raw.mode_of_operation = 0x04; // PT

            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x00;

            h->speed_ref = 0.0f;
            h->current_ref = 0.0f;
            h->current_offset_user = 0.0f;

            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

        // ------------------------------------------------
        // Assist + friction comp (better feel both ways)
        // ------------------------------------------------
        case MC_MODE_TS_ASSIST_FRIC:
        {
            h->cmd_raw.mode_of_operation = 0x04; // PT

            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x01;
            h->en_gravity_comp = 0x00;

            h->speed_ref = 0.0f;
            h->current_ref = 0.0f;
            h->current_offset_user = 0.0f;

            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }

                // -----------------------------------------
        // Simple intent -> fixed 2s boost test
        // -----------------------------------------
        case MC_MODE_TS_TOUCH_PULSE:
        {
            h->cmd_raw.mode_of_operation = 0x04; // PT

            h->en_fric_iden = 0x00;
            h->en_fric_comp = 0x00;
            h->en_gravity_comp = 0x00;

            h->speed_ref = 0.0f;
            h->current_ref = 0.0f;
            h->current_offset_user = 0.0f;
            h->current_offset = 0.0f;

            h->accel_limit_rpm = MOTOR_UNLIMITED_ACCEL;
            break;
        }


        default:
            break;
    }

    // ---------------- Timing ----------------
    clock_gettime(CLOCK_MONOTONIC, &h->time_now);
    h->dt_2 = h->time_now.tv_sec - h->time_pre.tv_sec +
              (h->time_now.tv_nsec - h->time_pre.tv_nsec) * 1e-9f;
    h->time_pre = h->time_now;

    // speed loop model
    float ref_slop = SLOP_step(&h->slop_spd, h->dt_2, h->accel_limit_rpm, h->speed_ref);
    float tf_out_d = TF_2RD_discrete_step(&h->tf_spd_d, h->dt_2, ref_slop);

    // keyboard
    MOTOR_CTRL_key();

    // ---------------- Feedback ----------------
    h->time          += h->dt;

    h->angle_fbk     = h_tx->actual_position * MOTOR_ENCODER_SIM_DIV * YZDN_MATH_2PI;
    h->angle_fbk_deg = h->angle_fbk * YZDN_MATH_K_RAD2DEG;

    h->speed_fbk     = h_tx->actual_velocity * MOTOR_ENCODER_SIM_DIV * YZDN_MATH_2PI;
    h->speed_fbk_rpm = h->speed_fbk * YZDN_MATH_K_RADPS2RPM;

    h->current_fbk   = h_tx->actual_torque * h->current_base;
    lpf_step(&h->lpf_current, h->current_fbk, h->dt);

    // ---------------- Torque sensor tare ----------------
    torque_sensor_tare_update(h->speed_fbk_rpm, h_tx->torque_mN_m);

    // signed + tared
    const float ts_comp_Nm = torque_sensor_comp_Nm(h_tx->torque_mN_m);
    h->torque_sens_Nm = TS_SIGN * ts_comp_Nm;

    // ---------------- Assist enable logic ----------------
    
    bool in_assist_mode =
        (h->mode == MC_MODE_TS_ASSIST) ||
        (h->mode == MC_MODE_TS_ASSIST_FRIC) ||
        (h->mode == MC_MODE_TS_TOUCH_PULSE);

    bool assist_enabled = (g_en_torque_boost != 0) || in_assist_mode;


    h->en_torque_boost = assist_enabled ? 1 : 0;

    // ---------------- Torque assist compute ----------------
    // ---------------- Torque assist compute ----------------
    float boost = 0.0f;

    const bool in_touch_pulse_mode = (h->mode == MC_MODE_TS_TOUCH_PULSE);

    if (!assist_enabled || !g_ts_tare_done) {
        boost = 0.0f;
        g_ts_state.active = false;
        g_ts_state.ts_user_filt = 0.0f;
        h->torque_user_Nm = 0.0f;

        g_touch_left_s = 0.0f;

    } else {

        // Capture bias snapshot on first enable
        if (!g_ts_bias_valid) {
            g_ts_bias_Nm = h->torque_sens_Nm;
            g_ts_bias_valid = 1;

            g_ts_state.active = false;
            g_ts_state.ts_user_filt = 0.0f;

            g_touch_left_s = 0.0f;
        }

        // user intent after bias removal
        h->torque_user_Nm = h->torque_sens_Nm - g_ts_bias_Nm;

        // When inactive and nearly still, slowly track drift
        if (!g_ts_state.active && std::fabs(h->speed_fbk_rpm) < 0.3f) {
            const float alpha = 0.002f;
            g_ts_bias_Nm = (1.0f - alpha) * g_ts_bias_Nm + alpha * h->torque_sens_Nm;
            h->torque_user_Nm = h->torque_sens_Nm - g_ts_bias_Nm;
        }

        // -----------------------------------------
        // NEW: ultra-simple intent → 2s pulse mode
        // -----------------------------------------
        if (in_touch_pulse_mode) {

            // countdown active pulse
            if (g_touch_left_s > 0.0f) {
                g_touch_left_s -= h->dt_2;
                if (g_touch_left_s < 0.0f) g_touch_left_s = 0.0f;

                g_touch_left_s = g_touch_pulse_s;
                g_touch_sign = signf_local(h->torque_user_Nm);
                boost = g_touch_sign * g_touch_pulse_mA;

            } else {

                // detect intent only when still
                if (std::fabs(h->speed_fbk_rpm) < g_touch_still_rpm &&
                    std::fabs(h->torque_user_Nm) > g_touch_on_Nm)
                {
                    g_touch_left_s = g_touch_pulse_s; // 2 seconds
                    boost = g_touch_sign * g_touch_pulse_mA;

                } else {
                    boost = 0.0f;
                }
            }

            // Do not use hysteresis assist state here
            g_ts_state.active = false;
            g_ts_state.ts_user_filt = 0.0f;

        } else {
            // existing assist engine
            boost = torque_assist_update(g_ts_state, g_ts_cfg, h->torque_user_Nm);
        }
    }

    h->current_boost = boost;

    

    // ---------------- Accel limit write ----------------
    h->accel_limit = h->accel_limit_rpm * YZDN_MATH_K_RPM2RADPS;
    float accel_raw = h->accel_limit_rpm * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;

    // slightly higher cap so assist doesn't feel "stuck"
    if(accel_raw > 6000000.0f) accel_raw = 6000000.0f;

    h_rx->accelerate_up = (uint32_t)accel_raw;
    h_rx->accelerate_down = (uint32_t)accel_raw;

    // ---------------- Compensation ----------------
    h->current_fric = Friction_Identify_compensate_step(h->speed_fbk_rpm);
    h->current_gravity = Gravity_compensate_step(h->angle_fbk_deg);

    // base offset (no boost yet)
    float offset_base =
        h->current_offset_user +
        h->en_fric_comp * h->current_fric +
        h->en_gravity_comp * h->current_gravity;

    float ref_eff = h->current_ref;
    float offset_eff = offset_base;

    // Apply boost:
    // In PT mode, boost is most meaningful as extra target torque.
    // In PV mode, many drives ignore target_torque -> boost may not "feel" effective.
    if (h->cmd_raw.mode_of_operation == 0x04) {
        ref_eff += h->current_boost;
    } else {
        offset_eff += h->current_boost;
    }

    // clamp
    ref_eff    = clampf_local(ref_eff,    -h->current_max, +h->current_max);
    offset_eff = clampf_local(offset_eff, -h->current_max, +h->current_max);

    // store for log
    h->current_offset = offset_eff;

    // write PDO
    h_rx->target_torque = ref_eff / h->current_base;
    h_rx->torque_offset = offset_eff / h->current_base;

    // speed target
    h_rx->speed_target =
        h->speed_ref * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;

    // ---------------- Debug print ----------------
    if (h->tick % 500 == 0)
    {
        const float ts_user_dbg = (g_ts_bias_valid) ? h->torque_user_Nm : 0.0f;
        const float ts_raw_Nm = (float)h_tx->torque_mN_m * 1e-3f;

        ECAT_LOG("mode:%d, spd_ref:%.1f, i_ref:%.1f, i_offset:%.1f, "
                 "angle:%.1f, spd:%.1f, i:%.1f, i_fric:%.1f, i_gravity:%.1f, "
                 "TS_raw:%d mNm(%.3fNm), TS_zero:%d mNm, TS_comp:%.3fNm, "
                 "TS_signed:%.3fNm, bias:%.3fNm(%d), TS_user:%.3fNm, "
                 "assist:%d(active=%d), i_boost:%.1f\n",
                 h->mode,
                 h->speed_ref, h->current_ref, h->current_offset,
                 h->angle_fbk_deg, h->speed_fbk_rpm, h->current_fbk,
                 h->current_fric, h->current_gravity,
                 h_tx->torque_mN_m, ts_raw_Nm,
                 g_ts_zero_mN_m, ts_comp_Nm,
                 h->torque_sens_Nm,
                 g_ts_bias_Nm, g_ts_bias_valid,
                 ts_user_dbg,
                 (int)assist_enabled, (int)g_ts_state.active,
                 h->current_boost);

#if TS_DEBUG_RATIO_CROSSCHECK
        float ts_ratio_est_mNm =
            (float)(h_tx->TS_RATIO_FIELD) * (float)(h_tx->TS_RATED_FIELD) / 1000.0f;

        ECAT_LOG("[TS RATIO CHECK] 3B69=%d mNm, ratio_est=%.1f mNm "
                 "(ratio=%d, rated=%d)\n",
                 h_tx->torque_mN_m,
                 ts_ratio_est_mNm,
                 (int)h_tx->TS_RATIO_FIELD,
                 (int)h_tx->TS_RATED_FIELD);
#endif
    }

    // record data
    if (0x00 == Friction_Identify_get_finish())
    {
        fprintf(p_data, "%.3f, %.3f, %.3f, %.3f, %.1f, %.1f\n",
                h->time, h->speed_ref, h->speed_fbk_rpm, tf_out_d,
                h->current_fbk, h->lpf_current.output);
    }
}


// ------------------------------------------------------
// Exit
// ------------------------------------------------------
void MOTOR_CTRL_exit(void)
{
    ECAT_LOG("exit, save file");
    sleep(1);

    if (p_data) fclose(p_data);
    if (p_file_log) fclose(p_file_log);
}


// ------------------------------------------------------
// Keyboard input
// ------------------------------------------------------
static int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}


// ------------------------------------------------------
// Keyboard control
// ------------------------------------------------------
static void MOTOR_CTRL_key(void)
{
    if (!kbhit()) return;

    char ch = getchar();
    MOTOR_CTRL_h h = &g_motor_ctrl;

    KEY_LOG("input cmd:%c\n", ch);

    switch(ch)
    {
        case 'p':
        {
            if(h->mode < (MC_MODE_NUM - 0x01)) {
                h->mode = MOTOR_CTRL_mode_e((uint8_t)h->mode + 0x01);
            } else {
                h->mode = MC_MODE_OFF;
            }
            KEY_LOG("mc mode:%s\n", mc_mode_name[h->mode]);

            if (h->mode == MC_MODE_TS_ASSIST ||
                h->mode == MC_MODE_TS_ASSIST_FRIC ||
                h->mode == MC_MODE_TS_TOUCH_PULSE)

            {
                g_ts_bias_valid = 0;
                g_touch_left_s = 0.0f;
                g_ts_state.active = false;
                g_ts_state.ts_user_filt = 0.0f;
                KEY_LOG("TS assist mode: bias will snapshot on next step\n");
            }
            break;
        }

        case 'w':
            h->current_ref += 10.0f;
            h->speed_ref   += 1.0f;
            break;

        case 's':
            h->current_ref -= 10.0f;
            h->speed_ref   -= 1.0f;
            break;

        case 'q':
            exit_app = 0x01;
            break;

        case 'e':
            h->current_offset_user += 10.0f;
            break;

        case 'd':
            h->current_offset_user -= 10.0f;
            break;

        case 'b':
            h->current_ref = 0.0f;
            h->current_offset_user = 0.0f;
            h->current_offset = 0.0f;
            h->speed_ref = 0.0f;

            g_ts_state.active = false;
            g_ts_state.ts_user_filt = 0.0f;
            break;

        case 'f':
            h->en_fric_comp = (0x01 - h->en_fric_comp);
            KEY_LOG("en_fric_comp:%d\n", h->en_fric_comp);
            break;

        case 'c':
            h->en_fric_iden = 0x01 - h->en_fric_iden;
            KEY_LOG("en_fric_iden:%d\n", h->en_fric_iden);
            break;

        case 'j':
        {
            float gain = Friction_Identify_get_gain() + 0.01f;
            Friction_Identify_set_gain(gain);
            KEY_LOG("fric gain:%.3f\n", gain);
            break;
        }

        case 'k':
        {
            float gain = Friction_Identify_get_gain() - 0.01f;
            Friction_Identify_set_gain(gain);
            KEY_LOG("fric gain:%.3f\n", gain);
            break;
        }

        case 'n':
        {
            float gain = Friction_Identify_get_gravity_gain() + 0.01f;
            Friction_Identify_set_gravity_gain(gain);
            KEY_LOG("gravity gain:%.3f\n", gain);
            break;
        }

        case 'm':
        {
            float gain = Friction_Identify_get_gravity_gain() - 0.01f;
            Friction_Identify_set_gravity_gain(gain);
            KEY_LOG("gravity gain:%.3f\n", gain);
            break;
        }

        // -------- Torque assist toggle --------
        case 't':
        {
            g_en_torque_boost = 0x01 - g_en_torque_boost;
            h->en_torque_boost = g_en_torque_boost;

            g_ts_bias_valid = 0;
            g_ts_state.active = false;
            g_ts_state.ts_user_filt = 0.0f;

            KEY_LOG("en_torque_boost:%d\n", g_en_torque_boost);
            break;
        }

        // -------- Re-tare --------
        case 'r':
            g_ts_tare_done = 0;
            g_ts_zero_mN_m = 0;

            g_ts_bias_Nm = 0.0f;
            g_ts_bias_valid = 0;

            g_ts_state.active = false;
            g_ts_state.ts_user_filt = 0.0f;

            KEY_LOG("torque sensor re-tare requested\n");
            break;

        default:
        {
            KEY_LOG("-------------------------------\n");
            KEY_LOG("press key to control motor:\n");
            KEY_LOG("p: next mode (includes TS assist modes)\n");
            KEY_LOG("w/s: current_ref +/-10, speed_ref +/-1\n");
            KEY_LOG("e/d: user offset +/-10\n");
            KEY_LOG("j/k: fric_gain +/-0.01\n");
            KEY_LOG("n/m: gravity_gain +/-0.01\n");
            KEY_LOG("b: stop\n");
            KEY_LOG("q: exit\n");
            KEY_LOG("f: toggle friction comp\n");
            KEY_LOG("c: toggle fric iden\n");
            KEY_LOG("t: torque assist toggle (works in any mode)\n");
            KEY_LOG("r: re-tare\n");
            KEY_LOG("-------------------------------\n");
            break;
        }
    }
}


// ------------------------------------------------------
// Update feedback
// ------------------------------------------------------
void MOTOR_CTRL_set_fbk_raw(txpdo_t fbk)
{
    MOTOR_CTRL_h h = &g_motor_ctrl;
    h->fbk_raw = fbk;
}


// ------------------------------------------------------
// Read command
// ------------------------------------------------------
rxpdo_t MOTOR_CTRL_get_cmd(void)
{
    MOTOR_CTRL_h h = &g_motor_ctrl;
    return h->cmd_raw;
}

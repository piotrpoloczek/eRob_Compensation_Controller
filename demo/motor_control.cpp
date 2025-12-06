/*
 * @Author:
 * @Date: 2025-08-06 15:58:55
 * @LastEditors: 抖音@翼之道男
 *
 * Corrected by ChatGPT:
 * - Fix torque boost direction with TS_SIGN
 * - Add bias snapshot on enable to prevent self-motion
 * - Add hysteresis deadband + slow bias trim
 * - Fix small syntax issues
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

// ---------------------------------------------
// External app control flag (likely in main)
// ---------------------------------------------
extern uint8_t exit_app;

// #define MOTOR_EN_FRIC_IDEN // start friction identification

// module parameters
#define MOTOR_CURRENT_BASE       (5.4f)       // motor current base = rated current A
#define MOTOR_ENCODER_SIM        (524288.0f)  // encoder lines
#define MOTOR_ENCODER_SIM_DIV    (1.0f/MOTOR_ENCODER_SIM)
#define MOTOR_K_RADPS2SIMPS      (MOTOR_ENCODER_SIM * 1.0F / YZDN_MATH_2PI)

#define MOTOR_UNLIMITED_ACCEL    (1000.0F)    // rpm/s
#define MOTOR_LIMITED_ACCEL      (10.0f)      // rpm/s

// ---------------- TORQUE SENSOR BOOST ----------------

// Enable flag
static uint8_t g_en_torque_boost = 0x00;

// Tare state
static int32_t g_ts_zero_mN_m = 0;
static uint8_t g_ts_tare_done = 0;

// Direction fix
// If "touch left -> moves right", you need -1.
// If it becomes wrong again, flip to +1.
static constexpr float TS_SIGN = -1.0f;

// Assist tuning
static float g_ts_stiction_mA        = 30.0f;   // kick to overcome static friction
static float g_ts_gain_mA_per_Nm     = 100.0f;
static float g_ts_max_mA             = 300.0f;

// Hysteresis deadband (prevents oscillation / self-creep)
static float g_ts_deadband_on_Nm  = 1.00f;
static float g_ts_deadband_off_Nm = 0.50f;

// Bias removal for "zero effort feel"
// This is separate from tare.
// Tare removes sensor offset.
// Bias removes static load at the current pose / support state.
static float   g_ts_bias_Nm = 0.0f;
static uint8_t g_ts_bias_valid = 0;

// Internal state
static uint8_t g_ts_active = 0;

// Simple clamp
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
  MC_MODE_OFF = 0X00,            // idle
  MC_MODE_COMP_FRIC_PT,         // current loop mode after friction compensation
  MC_MODE_COMP_GRAVITY_PT,      // current loop mode after gravity compensation
  MC_MODE_COMP_PT,              // current loop mode after gravity and friction compensation
  MC_MODE_COMP_PV,              // speed loop mode after gravity and friction compensation
  MC_MODE_FRIC_IDEN,            // friction identification
  MC_MODE_COMP_PV_IDEN_SIN,     // speed loop identification sine
  MC_MODE_COMP_PV_IDEN_SQUARE,  // speed loop identification square
  MC_MODE_COMP_PV_IDEN_SIN_2,   // sine with accel limit
  MC_MODE_COMP_PV_IDEN_SQUARE_2,// square with accel limit
  MC_MODE_NUM,
} MOTOR_CTRL_mode_e;

char mc_mode_name[MC_MODE_NUM][45] = {
    "MC_MODE_OFF","MC_MODE_COMP_FRIC_PT",
    "MC_MODE_COMP_GRAVITY_PT", "MC_MODE_COMP_PT","MC_MODE_COMP_PV",
    "MC_MODE_FRIC_IDEN","MC_MODE_COMP_PV_IDEN_SIN","MC_MODE_COMP_PV_IDEN_SQUARE",
    "MC_MODE_COMP_PV_IDEN_SIN_2","MC_MODE_COMP_PV_IDEN_SQUARE_2"
};

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

    // --- torque sensor boost ---
    uint8_t en_torque_boost;
    float torque_sens_Nm;  // signed + tared
    float torque_user_Nm;  // signed + tared + bias-removed
    float current_boost;   // mA

} MOTOR_CTRL_t, *MOTOR_CTRL_h;

void MOTOR_CTRL_key(void);

MOTOR_CTRL_t g_motor_ctrl;
FILE *p_file_log = NULL;
FILE *p_data = NULL;


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

    // control mode
    h->mode = MC_MODE_OFF;

    // motor control mode configuration
    h->cmd_raw.controlword = 0x000F;
    h->cmd_raw.torque_slope = 0x00;
    h->cmd_raw.max_torque = h->current_max / h->current_base;
    h->cmd_raw.padding = 0x00;
    h->cmd_raw.speed_limit = MOTOR_ENCODER_SIM;
    h->cmd_raw.speed_limit_2 = MOTOR_ENCODER_SIM;
    h->cmd_raw.accelerate_up = 2000000;
    h->cmd_raw.accelerate_down = 2000000;

    // speed loop model
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

    // reset TS state
    g_ts_zero_mN_m = 0;
    g_ts_tare_done = 0;
    g_ts_bias_Nm = 0.0f;
    g_ts_bias_valid = 0;
    g_ts_active = 0;
    g_en_torque_boost = 0x00;
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
    // auto tare when nearly still
    torque_sensor_tare_update(h->speed_fbk_rpm, h_tx->torque_mN_m);

    // signed + tared
    h->torque_sens_Nm = TS_SIGN * torque_sensor_comp_Nm(h_tx->torque_mN_m);

    // ---------------- Torque boost compute ----------------
    float boost = 0.0f;

    if (!g_en_torque_boost || !g_ts_tare_done) {
        g_ts_active = 0;
        boost = 0.0f;
    } else {

        // If bias not valid yet, capture a snapshot
        // This is the key fix for "enable t -> motor moves by itself".
        if (!g_ts_bias_valid) {
            g_ts_bias_Nm = h->torque_sens_Nm;
            g_ts_bias_valid = 1;
            g_ts_active = 0;
        }

        // torque user intent after bias removal
        h->torque_user_Nm = h->torque_sens_Nm - g_ts_bias_Nm;

        float abs_user = std::fabs(h->torque_user_Nm);

        // hysteresis deadband
        if (!g_ts_active) {
            if (abs_user >= g_ts_deadband_on_Nm) {
                g_ts_active = 1;
            }
        } else {
            if (abs_user <= g_ts_deadband_off_Nm) {
                g_ts_active = 0;
            }
        }

        // If inactive, slowly trim bias to follow drift
        // only when near still
        if (!g_ts_active && std::fabs(h->speed_fbk_rpm) < 0.3f) {
            const float alpha = 0.002f; // slow
            g_ts_bias_Nm = (1.0f - alpha) * g_ts_bias_Nm + alpha * h->torque_sens_Nm;
            h->torque_user_Nm = h->torque_sens_Nm - g_ts_bias_Nm;
        }

        // compute boost if active
        if (g_ts_active) {
            boost = h->torque_user_Nm * g_ts_gain_mA_per_Nm;
            boost += signf_local(h->torque_user_Nm) * g_ts_stiction_mA;
            boost = clampf_local(boost, -g_ts_max_mA, +g_ts_max_mA);
        } else {
            boost = 0.0f;
        }
    }

    h->current_boost = boost;

    // ---------------- Accel limit write ----------------
    h->accel_limit = h->accel_limit_rpm * YZDN_MATH_K_RPM2RADPS;
    float accel_raw = h->accel_limit_rpm * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;
    if(accel_raw > 2000000.0f) accel_raw = 2000000.0f;

    h_rx->accelerate_up = accel_raw;
    h_rx->accelerate_down = accel_raw;

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
    // - PT mode: add to target torque
    // - PV mode: add to feedforward offset
    if (h->cmd_raw.mode_of_operation == 0x04) {
        ref_eff += h->current_boost;
    } else {
        offset_eff += h->current_boost;
    }

    // clamp
    ref_eff    = clampf_local(ref_eff,   -h->current_max, +h->current_max);
    offset_eff = clampf_local(offset_eff, -h->current_max, +h->current_max);

    // store for log
    h->current_offset = offset_eff;

    // write PDO
    h_rx->target_torque = ref_eff / h->current_base;
    h_rx->torque_offset = offset_eff / h->current_base;

    // speed target
    h_rx->speed_target = h->speed_ref * YZDN_MATH_K_RPM2RADPS * MOTOR_K_RADPS2SIMPS;

    // ---------------- Print ----------------
    if (h->tick % 500 == 0)
    {
        ECAT_LOG("mode:%d, spd_ref:%.1f, i_ref:%.1f, i_offset:%.1f, "
                 "angle:%.1f, spd:%.1f, i:%.1f, i_fric:%.1f, i_gravity:%.1f, "
                 "ts:%.3fNm, ts_user:%.3fNm, i_boost:%.1f\n",
                 h->mode,
                 h->speed_ref, h->current_ref, h->current_offset,
                 h->angle_fbk_deg, h->speed_fbk_rpm, h->current_fbk,
                 h->current_fric, h->current_gravity,
                 h->torque_sens_Nm,
                 (g_ts_bias_valid ? (h->torque_sens_Nm - g_ts_bias_Nm) : 0.0f),
                 h->current_boost);
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
int kbhit(void)
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
void MOTOR_CTRL_key(void)
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

        // -------- Torque sensor boost toggle --------
        case 't':
        {
            g_en_torque_boost = 0x01 - g_en_torque_boost;
            h->en_torque_boost = g_en_torque_boost;

            // Reset assist state + bias capture on next step
            g_ts_active = 0;
            g_ts_bias_valid = 0;

            KEY_LOG("en_torque_boost:%d\n", g_en_torque_boost);
            break;
        }

        // increase/decrease boost gain
        case 'u':
            g_ts_gain_mA_per_Nm += 5.0f;
            KEY_LOG("ts_gain: %.1f mA/Nm\n", g_ts_gain_mA_per_Nm);
            break;

        case 'i':
            g_ts_gain_mA_per_Nm -= 5.0f;
            if (g_ts_gain_mA_per_Nm < 0.0f) g_ts_gain_mA_per_Nm = 0.0f;
            KEY_LOG("ts_gain: %.1f mA/Nm\n", g_ts_gain_mA_per_Nm);
            break;

        // increase/decrease max clamp
        case 'y':
            g_ts_max_mA += 20.0f;
            KEY_LOG("ts_max: %.1f mA\n", g_ts_max_mA);
            break;

        case 'h':
            g_ts_max_mA -= 20.0f;
            if (g_ts_max_mA < 0.0f) g_ts_max_mA = 0.0f;
            KEY_LOG("ts_max: %.1f mA\n", g_ts_max_mA);
            break;

        // force re-tare
        case 'r':
            g_ts_tare_done = 0;
            g_ts_zero_mN_m = 0;
            g_ts_bias_Nm = 0.0f;
            g_ts_bias_valid = 0;
            g_ts_active = 0;
            KEY_LOG("torque sensor re-tare requested\n");
            break;

        // stiction adjust
        case 'o':
            g_ts_stiction_mA += 10.0f;
            KEY_LOG("ts_stiction: %.1f mA\n", g_ts_stiction_mA);
            break;

        case 'l':
            g_ts_stiction_mA -= 10.0f;
            if (g_ts_stiction_mA < 0.0f) g_ts_stiction_mA = 0.0f;
            KEY_LOG("ts_stiction: %.1f mA\n", g_ts_stiction_mA);
            break;

        default:
        {
            KEY_LOG("-------------------------------\n");
            KEY_LOG("press key to control motor:\n");
            KEY_LOG("p: next mode\n");
            KEY_LOG("w/s: current_ref +/-10, speed_ref +/-1\n");
            KEY_LOG("e/d: user offset +/-10\n");
            KEY_LOG("j/k: fric_gain +/-0.01\n");
            KEY_LOG("n/m: gravity_gain +/-0.01\n");
            KEY_LOG("b: stop\n");
            KEY_LOG("q: exit\n");
            KEY_LOG("f: toggle friction comp\n");
            KEY_LOG("c: toggle fric iden\n");
            KEY_LOG("t: torque sensor boost toggle\n");
            KEY_LOG("u/i: ts gain +/-\n");
            KEY_LOG("y/h: ts max +/-\n");
            KEY_LOG("o/l: stiction +/-\n");
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

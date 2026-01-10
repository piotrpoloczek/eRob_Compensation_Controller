#pragma once
#include <cmath>
#include <cstdint>

struct TorqueAssistConfig
{
    // Engage/disengage thresholds (Nm)
    float on_Nm  = 1.00f;  // engage when |torque| >= on_Nm
    float off_Nm = 0.50f;  // disengage when |torque| <= off_Nm

    // Current shaping (mA)
    float stiction_mA       = 60.0f;   // base current once active
    float gain_mA_per_Nm    = 180.0f;  // slope vs |torque|
    float max_mA            = 600.0f;  // hard cap

    // Simple torque filtering
    float filt_alpha        = 0.15f;   // 0..1 (higher = faster response)
};

struct TorqueAssistState
{
    bool  active        = false;
    float ts_user_filt  = 0.0f;
};

static inline float ta_clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float ta_signf(float x)
{
    return (x > 0.0f) ? 1.0f : (x < 0.0f) ? -1.0f : 0.0f;
}

/**
 * Update torque-only assist.
 *
 * Input:
 *  ts_user_Nm = user torque after tare & bias removal (Nm)
 *
 * Output:
 *  return boost current in mA
 */
static inline float torque_assist_update(
    TorqueAssistState &st,
    const TorqueAssistConfig &cfg,
    float ts_user_Nm)
{
    // 1) Filter
    st.ts_user_filt = (1.0f - cfg.filt_alpha) * st.ts_user_filt
                    + (cfg.filt_alpha) * ts_user_Nm;

    const float t  = st.ts_user_filt;
    const float at = std::fabs(t);

    // 2) Symmetric hysteresis on magnitude
    if (!st.active) {
        if (at >= cfg.on_Nm) st.active = true;
    } else {
        if (at <= cfg.off_Nm) st.active = false;
    }

    if (!st.active) return 0.0f;

    // 3) Map |torque| to current magnitude
    float i_mag = cfg.stiction_mA + cfg.gain_mA_per_Nm * at;
    i_mag = ta_clampf(i_mag, 0.0f, cfg.max_mA);

    // 4) Signed output
    return ta_signf(t) * i_mag;
}

#ifndef __ERBO_H__
#define __ERBO_H__

#include <cstdint>

/* ----------------------- RXPDO (master -> slave) ----------------------- */
/* Matches your 0x1600 mapping (12 entries incl. 8-bit padding) */
typedef struct {
    uint16_t controlword;      // 0x6040:0  (16)
    int16_t  target_torque;    // 0x6071:0  (16)
    int32_t  torque_slope;     // 0x6087:0  (32) 0 = no slope
    uint16_t max_torque;       // 0x6072:0  (16)
    uint8_t  mode_of_operation;// 0x6060:0  (8)
    int16_t  torque_offset;    // 0x60B2:0  (16) current/torque FF
    int32_t  speed_target;     // 0x60FF:0  (32) target speed (puls/s)
    uint32_t speed_limit;      // 0x607F:0  (32) max allowed speed (puls/s)
    uint32_t speed_limit_2;    // 0x6080:0  (32) max any-direction speed (puls/s)
    uint32_t accelerate_up;    // 0x6083:0  (32) profile accel (puls/s^2)
    uint32_t accelerate_down;  // 0x6084:0  (32) profile decel (puls/s^2)
    uint8_t  padding;          // (8)      ESC even-byte alignment because of 0x6060
} __attribute__((__packed__)) rxpdo_t;

static_assert(sizeof(rxpdo_t) == 34, "RXPDO size must be 34 bytes");

/* ----------------------- TXPDO (slave -> master) ----------------------- */
/* Order & sizes must match 0x1A00 mapping:
   0:  0x6041:0  u16   StatusWord
   2:  0x6064:0  s32   Actual Position
   6:  0x606C:0  s32   Actual Velocity
   10: 0x6077:0  s16   Actual Torque
   12: 0x3B69:0  s32   Torque sensor value (mN·m)
   16: 0x3B6A:0  s16   Torque sensor ratio (0.1%)
*/
typedef struct {
    uint16_t statusword;       // 0x6041:0
    int32_t  actual_position;  // 0x6064:0
    int32_t  actual_velocity;  // 0x606C:0
    int16_t  actual_torque;    // 0x6077:0
    int32_t  torque_mN_m;      // 0x3B69:0  DINT (mN·m)
    int16_t  torque_ratio_pm;  // 0x3B6A:0  INT  (0.1 % of rated)
} __attribute__((__packed__)) txpdo_t;

static_assert(sizeof(txpdo_t) == 18, "TXPDO size must be 18 bytes");

extern uint8_t exit_app;

#endif

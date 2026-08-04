#pragma once
#include <stdint.h>

// robot_drive — differential drive only (no balancing).
// FW/BK motor sense is inverted vs the old balboa_fsf kit frame to match
// this robot's assembly (see applyDrive in Motion.cpp).

const int16_t MOTOR_LIMIT = 300;
const int8_t MOTOR_SIGN = 1;
const int8_t ENCODER_SIGN = -1;

const uint8_t CONTROL_MS = 10;  // 100 Hz motion loop

// Functional safety: FW/BK (and continuous turn) die unless refreshed.
const uint16_t WATCHDOG_MS = 300;
const uint16_t WATCHDOG_MIN_MS = 50;
const uint16_t WATCHDOG_MAX_MS = 1000;

// Hard caps on commanded speed (PWM units after clamp).
const int16_t SPEED_MAX = 200;
const int16_t SPEED_DEFAULT = 100;
const int16_t TURN_SPEED_DEFAULT = 120;

// Max continuous travel while a FW/BK latch is held (encoder counts L+R).
// Forces a stop even if heartbeat keeps coming — operator must re-command.
// 0 = disabled; the HB watchdog is then the only deadman on held driving.
const int32_t TRAVEL_LIMIT_COUNTS = 0;

const int32_t DISTANCE_CLAMP = 200000;

// On-MCU command FIFO (FUSA: STOP / watchdog / travel_limit flush it).
const uint8_t CMD_QUEUE_DEPTH = 8;

// Optional segment duration for queued FW/BK/continuous turns (0 = until
// travel limit / finite-turn done / safety stop). Clamped in Motion.
const uint16_t DURATION_MAX_MS = 30000;

// Straight-line assist (FW/BK): P correction from encoder imbalance.
// corr = clamp(straightDiff * NUM / DEN, ±min(CORR_MAX, speed/2))
// Applied only after travelAbs >= STRAIGHT_MIN_TRAVEL.
const int16_t STRAIGHT_KP_NUM = 1;
const int16_t STRAIGHT_KP_DEN = 4;
const int16_t STRAIGHT_CORR_MAX = 20;
const int32_t STRAIGHT_MIN_TRAVEL = 40;

// DRV8838: after brake, same-direction restart often sticks. One short
// opposite PHASE pulse at segment start (not mid-segment) unsticks it.
// Keep this short to avoid a visible reverse twitch / vibration.
const int16_t PHASE_KICK_PWM = 90;
const uint16_t PHASE_KICK_MS = 8;

// Anti-stall: wheels not turning while PWM is applied means not enough
// torque (load/friction), so raise PWM in the SAME direction — never a
// reverse pulse, which just shakes the chassis. Boost lasts one segment.
const uint16_t STALL_DETECT_MS = 150;
const int16_t STALL_BOOST_STEP = 40;
const int16_t STALL_BOOST_MAX = 140;

// Serial
const uint32_t SERIAL_BAUD = 115200;
const uint8_t LINE_MAX = 96;

// Motion debug UART lines (D ...). Toggle at runtime: DBG 1 / DBG 0
const uint8_t DEBUG_DEFAULT = 0;
const uint16_t DEBUG_MID_MS = 250;

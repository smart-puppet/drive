#pragma once
#include <stdint.h>
#include "Config.h"

enum MotionMode : uint8_t {
  MODE_IDLE = 0,
  MODE_FORWARD,
  MODE_BACKWARD,
  MODE_TURN_LEFT,
  MODE_TURN_RIGHT,
};

enum StopReason : uint8_t {
  STOP_NONE = 0,
  STOP_COMMAND,
  STOP_WATCHDOG,
  STOP_TRAVEL_LIMIT,
  STOP_TURN_DONE,
  STOP_DURATION,
  STOP_BOOT,
  STOP_QUEUE_FULL,
};

struct QueuedCmd {
  MotionMode mode;
  int16_t speed;
  uint16_t ttlMs;
  int32_t counts;      // LT/RT finite target; 0 = continuous
  uint16_t durationMs; // 0 = no time limit (until travel/turn/safety)
};

struct MotionState {
  MotionMode mode;
  StopReason lastStop;
  int16_t speed;
  uint16_t watchdogMs;
  uint32_t deadlineMs;
  uint16_t durationMs;
  uint32_t durationDeadlineMs;
  int32_t turnTargetCounts;
  int32_t turnProgress;
  int32_t turnLastDiff;
  bool turnHaveDiff;
  bool estop;
  uint16_t cmdSeq;
  uint16_t stopSeq;
  uint8_t queueLen;
};

extern MotionState gMotion;

void motionSetup();
void motionTick(uint32_t nowMs);

// STOP: motors off, flush FIFO, latch estop (FUSA).
void motionStop(StopReason reason);

// Soft idle current segment (no estop, no flush); then start next queued cmd.
void motionIdle(StopReason reason);

void motionClearEstop();

// Enqueue motion (starts immediately if idle). Returns false if estop / full.
bool motionEnqueueForward(int16_t speed, uint16_t ttlMs, uint16_t durationMs);
bool motionEnqueueBackward(int16_t speed, uint16_t ttlMs, uint16_t durationMs);
bool motionEnqueueTurnLeft(int16_t speed, uint16_t ttlMs, int32_t counts,
                           uint16_t durationMs);
bool motionEnqueueTurnRight(int16_t speed, uint16_t ttlMs, int32_t counts,
                            uint16_t durationMs);

bool motionHeartbeat(uint16_t ttlMs);

uint8_t motionQueueLen();
void motionQueueFlush();

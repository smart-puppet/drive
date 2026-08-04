#include "Motion.h"
#include "Motors.h"
#include "Protocol.h"
#include "Config.h"
#include <Arduino.h>

MotionState gMotion;

static QueuedCmd sQueue[CMD_QUEUE_DEPTH];
static uint8_t sHead;
static uint8_t sTail;
static uint8_t sCount;

// Per-segment debug counters (only used when gDebug).
static uint16_t sSegTicks;
static uint32_t sSegStartMs;
static uint32_t sLastDebugMidMs;

// Per-segment anti-stall state.
static int16_t sStallBoost;
static int32_t sStallTravelMark;
static uint32_t sStallSinceMs;

static char modeChar(MotionMode m)
{
  switch (m) {
    case MODE_FORWARD: return 'F';
    case MODE_BACKWARD: return 'B';
    case MODE_TURN_LEFT: return 'L';
    case MODE_TURN_RIGHT: return 'R';
    default: return 'I';
  }
}

static char stopChar(StopReason r)
{
  switch (r) {
    case STOP_COMMAND: return 'C';
    case STOP_WATCHDOG: return 'W';
    case STOP_TRAVEL_LIMIT: return 'T';
    case STOP_TURN_DONE: return 'U';
    case STOP_DURATION: return 'D';
    case STOP_BOOT: return 'K';
    case STOP_QUEUE_FULL: return 'Q';
    default: return '-';
  }
}

static int16_t clampSpeed(int16_t speed)
{
  if (speed < 0) { speed = -speed; }
  if (speed == 0) { speed = SPEED_DEFAULT; }
  if (speed > SPEED_MAX) { speed = SPEED_MAX; }
  return speed;
}

static uint16_t clampTtl(uint16_t ttlMs)
{
  if (ttlMs == 0) { ttlMs = WATCHDOG_MS; }
  if (ttlMs < WATCHDOG_MIN_MS) { ttlMs = WATCHDOG_MIN_MS; }
  if (ttlMs > WATCHDOG_MAX_MS) { ttlMs = WATCHDOG_MAX_MS; }
  return ttlMs;
}

static uint16_t clampDuration(uint16_t durationMs)
{
  if (durationMs > DURATION_MAX_MS) { durationMs = DURATION_MAX_MS; }
  return durationMs;
}

static int16_t straightCorrection()
{
  // Left ahead (straightDiff>0) → reduce left PWM, boost right.
  if (travelAbs < STRAIGHT_MIN_TRAVEL) { return 0; }
  int32_t corr = (straightDiff * (int32_t)STRAIGHT_KP_NUM) / (int32_t)STRAIGHT_KP_DEN;
  int16_t maxC = (int16_t)(gMotion.speed / 2);
  if (maxC > STRAIGHT_CORR_MAX) { maxC = STRAIGHT_CORR_MAX; }
  if (maxC < 1) { maxC = 1; }
  if (corr > maxC) { corr = maxC; }
  if (corr < -maxC) { corr = -maxC; }
  return (int16_t)corr;
}

static void applyDrive()
{
  int16_t s = (int16_t)(gMotion.speed + sStallBoost);
  if (s > MOTOR_LIMIT) { s = MOTOR_LIMIT; }
  switch (gMotion.mode) {
    case MODE_FORWARD: {
      // Inverted vs kit sign frame — matches this robot assembly.
      const int16_t c = straightCorrection();
      motorsDriveLR((int16_t)(s + c), (int16_t)(s - c));
      break;
    }
    case MODE_BACKWARD: {
      const int16_t c = straightCorrection();
      motorsDriveLR((int16_t)(-s + c), (int16_t)(-s - c));
      break;
    }
    case MODE_TURN_LEFT:
      // Opposite wheel signs vs FW (s,s): left reverse, right forward.
      motorsDriveLR(-s, s);
      break;
    case MODE_TURN_RIGHT:
      motorsDriveLR(s, -s);
      break;
    default:
      motorsStop();
      break;
  }
}

// Brief opposite PHASE while EN stays active, then caller applyDrive() flips
// DIR to the desired sense. Needed because Balboa setSpeed(0) forces DIR=forward
// while braking — same-direction restart then has no DIR edge and often sticks.
static void phaseEdgeKick(MotionMode mode)
{
  const int16_t k = PHASE_KICK_PWM;
  switch (mode) {
    case MODE_FORWARD:
      motorsDriveLR(-k, -k);
      break;
    case MODE_BACKWARD:
      motorsDriveLR(k, k);
      break;
    case MODE_TURN_LEFT:
      motorsDriveLR(k, -k);
      break;
    case MODE_TURN_RIGHT:
      motorsDriveLR(-k, k);
      break;
    default:
      return;
  }
  delay(PHASE_KICK_MS);
}

static void syncQueueLen()
{
  gMotion.queueLen = sCount;
}

void motionQueueFlush()
{
  sHead = 0;
  sTail = 0;
  sCount = 0;
  syncQueueLen();
}

uint8_t motionQueueLen()
{
  return sCount;
}

static bool queuePush(const QueuedCmd &cmd)
{
  if (sCount >= CMD_QUEUE_DEPTH) { return false; }
  sQueue[sTail] = cmd;
  sTail = (uint8_t)((sTail + 1) % CMD_QUEUE_DEPTH);
  sCount++;
  syncQueueLen();
  return true;
}

static bool queuePop(QueuedCmd *out)
{
  if (sCount == 0) { return false; }
  *out = sQueue[sHead];
  sHead = (uint8_t)((sHead + 1) % CMD_QUEUE_DEPTH);
  sCount--;
  syncQueueLen();
  return true;
}

static void haltMotors(StopReason reason, bool latchEstop, bool flushQueue)
{
  // Snapshot before motors/encoders are cleared (needed to diagnose "PWM on,
  // travel=0" vs early stop).
  const int16_t snapML = motorLeft;
  const int16_t snapMR = motorRight;
  const int32_t snapTr = travelAbs;
  const int32_t snapSd = straightDiff;
  const uint16_t snapTk = sSegTicks;
  const uint16_t snapMs =
      (sSegStartMs == 0) ? 0 : (uint16_t)(millis() - sSegStartMs);
  const uint8_t wasActive = (gMotion.mode != MODE_IDLE) ? 1 : 0;

  gMotion.mode = MODE_IDLE;
  gMotion.speed = 0;
  gMotion.deadlineMs = 0;
  gMotion.durationMs = 0;
  gMotion.durationDeadlineMs = 0;
  gMotion.turnTargetCounts = 0;
  gMotion.turnProgress = 0;
  gMotion.turnHaveDiff = false;
  gMotion.lastStop = reason;
  if (flushQueue) {
    motionQueueFlush();
    gMotion.stopSeq++;
  }
  if (latchEstop) {
    gMotion.estop = true;
  }
  motorsStop();
  if (wasActive) {
    protocolEmitDebugEnd(stopChar(reason), snapML, snapMR, snapTr, snapSd,
                         snapTk, snapMs, flushQueue ? 1 : 0);
  }
  travelReset();
  sSegTicks = 0;
  sSegStartMs = 0;
  sLastDebugMidMs = 0;
}

static bool startCmd(const QueuedCmd &cmd, bool fromStop)
{
  if (gMotion.estop) { return false; }

  gMotion.mode = cmd.mode;
  gMotion.speed = clampSpeed(cmd.speed);
  gMotion.turnTargetCounts = 0;
  gMotion.turnProgress = 0;
  gMotion.turnHaveDiff = false;
  gMotion.lastStop = STOP_NONE;
  gMotion.cmdSeq++;

  const bool isTurn =
      (cmd.mode == MODE_TURN_LEFT || cmd.mode == MODE_TURN_RIGHT);
  if (isTurn && cmd.counts > 0) {
    gMotion.turnTargetCounts = cmd.counts;
  }

  gMotion.durationMs = clampDuration(cmd.durationMs);
  gMotion.durationDeadlineMs = 0;
  gMotion.watchdogMs = 0;
  gMotion.deadlineMs = 0;

  // From a brake/stop: force a DIR edge, then go straight to full commanded
  // PWM. Never apply 0 after the kick (that re-brakes and re-sticks).
  if (fromStop) {
    phaseEdgeKick(cmd.mode);
  }
  travelReset();
  sStallBoost = 0;
  sStallTravelMark = 0;

  const uint32_t t0 = millis();
  if (gMotion.durationMs > 0) {
    gMotion.durationDeadlineMs = t0 + gMotion.durationMs;
  } else if (isTurn && cmd.counts > 0) {
    uint16_t t = cmd.ttlMs == 0 ? 3000 : cmd.ttlMs;
    if (t < 1000) { t = 1000; }
    if (t > 8000) { t = 8000; }
    gMotion.watchdogMs = t;
    gMotion.deadlineMs = t0 + gMotion.watchdogMs;
  } else {
    gMotion.watchdogMs = clampTtl(cmd.ttlMs);
    gMotion.deadlineMs = t0 + gMotion.watchdogMs;
  }

  sSegTicks = 0;
  sSegStartMs = t0;
  sLastDebugMidMs = t0;
  sStallSinceMs = t0;
  applyDrive();  // full speed immediately
  protocolEmitDebugBegin(modeChar(cmd.mode), gMotion.speed, gMotion.durationMs,
                         cmd.counts);
  return true;
}

static void tryStartNext()
{
  if (gMotion.estop || gMotion.mode != MODE_IDLE) { return; }
  QueuedCmd cmd;
  if (!queuePop(&cmd)) { return; }
  startCmd(cmd, true);
}

static bool enqueue(MotionMode mode, int16_t speed, uint16_t ttlMs,
                    int32_t counts, uint16_t durationMs)
{
  if (gMotion.estop) { return false; }

  QueuedCmd cmd;
  cmd.mode = mode;
  cmd.speed = speed;
  cmd.ttlMs = ttlMs;
  cmd.counts = counts;
  cmd.durationMs = durationMs;

  // Timed one-shots (dur>0): always start NOW with a full fresh segment.
  // Same-direction refresh while moving: no brake (keeps DIR/EN live).
  if (durationMs > 0) {
    motionQueueFlush();
    if (gMotion.mode == mode && gMotion.mode != MODE_IDLE) {
      return startCmd(cmd, false);
    }
    if (gMotion.mode != MODE_IDLE) {
      haltMotors(STOP_DURATION, false, false);
    }
    return startCmd(cmd, true);
  }

  if (gMotion.mode == MODE_IDLE && sCount == 0) {
    return startCmd(cmd, true);
  }
  if (!queuePush(cmd)) {
    gMotion.lastStop = STOP_QUEUE_FULL;
    return false;
  }
  return true;
}

void motionSetup()
{
  gMotion.mode = MODE_IDLE;
  gMotion.lastStop = STOP_BOOT;
  gMotion.speed = 0;
  gMotion.watchdogMs = WATCHDOG_MS;
  gMotion.deadlineMs = 0;
  gMotion.durationMs = 0;
  gMotion.durationDeadlineMs = 0;
  gMotion.turnTargetCounts = 0;
  gMotion.turnProgress = 0;
  gMotion.turnLastDiff = 0;
  gMotion.turnHaveDiff = false;
  gMotion.estop = false;
  gMotion.cmdSeq = 0;
  gMotion.stopSeq = 0;
  motionQueueFlush();
  motorsStop();
}

void motionStop(StopReason reason)
{
  // FUSA path: always flush FIFO. STOP_COMMAND also latches estop.
  const bool estop = (reason == STOP_COMMAND);
  haltMotors(reason, estop, true);
}

void motionIdle(StopReason reason)
{
  // Complete current segment only — do not flush FIFO; start next.
  haltMotors(reason, false, false);
  tryStartNext();
}

void motionClearEstop()
{
  gMotion.estop = false;
  if (gMotion.lastStop == STOP_COMMAND) {
    gMotion.lastStop = STOP_NONE;
  }
}

bool motionEnqueueForward(int16_t speed, uint16_t ttlMs, uint16_t durationMs)
{
  return enqueue(MODE_FORWARD, speed, ttlMs, 0, durationMs);
}

bool motionEnqueueBackward(int16_t speed, uint16_t ttlMs, uint16_t durationMs)
{
  return enqueue(MODE_BACKWARD, speed, ttlMs, 0, durationMs);
}

bool motionEnqueueTurnLeft(int16_t speed, uint16_t ttlMs, int32_t counts,
                           uint16_t durationMs)
{
  return enqueue(MODE_TURN_LEFT, speed, ttlMs, counts, durationMs);
}

bool motionEnqueueTurnRight(int16_t speed, uint16_t ttlMs, int32_t counts,
                            uint16_t durationMs)
{
  return enqueue(MODE_TURN_RIGHT, speed, ttlMs, counts, durationMs);
}

bool motionHeartbeat(uint16_t ttlMs)
{
  if (gMotion.estop) { return false; }
  if (gMotion.mode == MODE_IDLE) { return false; }
  // Finite-turn safety TTL uses a wider window; HB still refreshes deadline.
  if (gMotion.turnTargetCounts > 0) {
    uint16_t t = ttlMs == 0 ? gMotion.watchdogMs : ttlMs;
    if (t < 500) { t = 500; }
    if (t > 5000) { t = 5000; }
    gMotion.watchdogMs = t;
  } else {
    gMotion.watchdogMs = clampTtl(ttlMs == 0 ? gMotion.watchdogMs : ttlMs);
  }
  gMotion.deadlineMs = millis() + gMotion.watchdogMs;
  return true;
}

void motionTick(uint32_t nowMs)
{
  if (gMotion.mode == MODE_IDLE) {
    tryStartNext();
    return;
  }

  sSegTicks++;

  // Untimed only: external HB deadman. Timed (dur>0) uses duration alone.
  if (gMotion.durationMs == 0) {
    if ((int32_t)(nowMs - gMotion.deadlineMs) >= 0) {
      motionStop(STOP_WATCHDOG);
      return;
    }
  }

  if (gMotion.durationDeadlineMs != 0 &&
      (int32_t)(nowMs - gMotion.durationDeadlineMs) >= 0) {
    motionIdle(STOP_DURATION);
    return;
  }

  if (TRAVEL_LIMIT_COUNTS > 0 && gMotion.durationMs == 0 &&
      (gMotion.mode == MODE_FORWARD || gMotion.mode == MODE_BACKWARD) &&
      travelAbs >= TRAVEL_LIMIT_COUNTS) {
    motionStop(STOP_TRAVEL_LIMIT);
    return;
  }

  // Finite turn: travelAbs (= Σ|dL|+|dR|) always grows while wheels move.
  // (Old dL−dR progress could stay ~0 with this encoder/motor sign frame.)
  if ((gMotion.mode == MODE_TURN_LEFT || gMotion.mode == MODE_TURN_RIGHT) &&
      gMotion.turnTargetCounts > 0) {
    gMotion.turnProgress = travelAbs;
    if (travelAbs >= gMotion.turnTargetCounts) {
      motionIdle(STOP_TURN_DONE);
      return;
    }
  }

  // Anti-stall: encoders not advancing while driving → more PWM, same sign.
  if (travelAbs > sStallTravelMark) {
    sStallTravelMark = travelAbs;
    sStallSinceMs = nowMs;
  } else if (sStallBoost < STALL_BOOST_MAX &&
             (uint32_t)(nowMs - sStallSinceMs) >= STALL_DETECT_MS) {
    sStallBoost += STALL_BOOST_STEP;
    if (sStallBoost > STALL_BOOST_MAX) { sStallBoost = STALL_BOOST_MAX; }
    sStallSinceMs = nowMs;
    protocolEmitDebugStall(travelAbs, sStallBoost);
  }

  applyDrive();

  if (gDebug && (uint32_t)(nowMs - sLastDebugMidMs) >= DEBUG_MID_MS) {
    sLastDebugMidMs = nowMs;
    int32_t tleft = 0;
    if (gMotion.durationDeadlineMs != 0) {
      tleft = (int32_t)(gMotion.durationDeadlineMs - nowMs);
    } else if (gMotion.deadlineMs != 0) {
      tleft = (int32_t)(gMotion.deadlineMs - nowMs);
    }
    protocolEmitDebugMid(modeChar(gMotion.mode), motorLeft, motorRight,
                         travelAbs, straightDiff, wheelSpeed, tleft);
  }
}

#include "Protocol.h"
#include "Motion.h"
#include "Motors.h"
#include "Config.h"
#include <Balboa32U4.h>
#include <stdlib.h>
#include <string.h>

uint8_t gDebug = DEBUG_DEFAULT;

static char lineBuf[LINE_MAX];
static uint8_t lineLen;

// Compact mode codes: I F B L R
static char modeCode(MotionMode m)
{
  switch (m) {
    case MODE_FORWARD: return 'F';
    case MODE_BACKWARD: return 'B';
    case MODE_TURN_LEFT: return 'L';
    case MODE_TURN_RIGHT: return 'R';
    default: return 'I';
  }
}

// Compact stop codes: - C W T U D K Q
static char stopCode(StopReason r)
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

// Full status only when requested (ST) / boot.
// Avoid calling this from the motion loop — readBatteryMillivolts() is slow
// and a long TX line can block the MCU until the host drains USB, dropping RX.
void protocolEmitStatus(const char *event)
{
  uint32_t now = millis();
  int32_t ttlLeft = 0;
  if (gMotion.mode != MODE_IDLE && gMotion.deadlineMs != 0) {
    ttlLeft = (int32_t)(gMotion.deadlineMs - now);
    if (ttlLeft < 0) { ttlLeft = 0; }
  }
  const uint8_t needHb =
      (gMotion.mode != MODE_IDLE && gMotion.durationMs == 0) ? 1 : 0;

  Serial.print(F("S e="));
  Serial.print(event);
  Serial.print(F(" m="));
  Serial.print(modeCode(gMotion.mode));
  Serial.print(F(" es="));
  Serial.print(gMotion.estop ? 1 : 0);
  Serial.print(F(" sp="));
  Serial.print(gMotion.speed);
  Serial.print(F(" tl="));
  Serial.print(ttlLeft);
  Serial.print(F(" q="));
  Serial.print(gMotion.queueLen);
  Serial.print(F(" ls="));
  Serial.print(stopCode(gMotion.lastStop));
  Serial.print(F(" hb="));
  Serial.print(needHb);
  Serial.print(F(" cs="));
  Serial.print(gMotion.cmdSeq);
  Serial.print(F(" ss="));
  Serial.print(gMotion.stopSeq);
  Serial.print(F(" tr="));
  Serial.print(travelAbs);
  Serial.print(F(" v="));
  Serial.print(readBatteryMillivolts());
  Serial.println();
}

// Short ACK after motion cmds: A e=<evt> m=<code> es=<0|1> q=<n> hb=<0|1>
void protocolEmitAck(const char *event)
{
  const uint8_t needHb =
      (gMotion.mode != MODE_IDLE && gMotion.durationMs == 0) ? 1 : 0;
  Serial.print(F("A e="));
  Serial.print(event);
  Serial.print(F(" m="));
  Serial.print(modeCode(gMotion.mode));
  Serial.print(F(" es="));
  Serial.print(gMotion.estop ? 1 : 0);
  Serial.print(F(" q="));
  Serial.print(gMotion.queueLen);
  Serial.print(F(" hb="));
  Serial.print(needHb);
  Serial.println();
}

void protocolEmitDebugBegin(char mode, int16_t speed, uint16_t durMs, int32_t counts)
{
  if (!gDebug) { return; }
  Serial.print(F("D beg m="));
  Serial.print(mode);
  Serial.print(F(" sp="));
  Serial.print(speed);
  Serial.print(F(" dur="));
  Serial.print(durMs);
  Serial.print(F(" c="));
  Serial.print(counts);
  Serial.print(F(" mL="));
  Serial.print(motorLeft);
  Serial.print(F(" mR="));
  Serial.println(motorRight);
}

void protocolEmitDebugMid(char mode, int16_t mL, int16_t mR, int32_t travel,
                          int32_t straight, int32_t ws, int32_t tleft)
{
  if (!gDebug) { return; }
  Serial.print(F("D mid m="));
  Serial.print(mode);
  Serial.print(F(" mL="));
  Serial.print(mL);
  Serial.print(F(" mR="));
  Serial.print(mR);
  Serial.print(F(" tr="));
  Serial.print(travel);
  Serial.print(F(" sd="));
  Serial.print(straight);
  Serial.print(F(" ws="));
  Serial.print(ws);
  Serial.print(F(" tl="));
  Serial.print(tleft);
  Serial.print(F(" v="));
  Serial.println(readBatteryMillivolts());
}

void protocolEmitDebugEnd(char reason, int16_t mL, int16_t mR, int32_t travel,
                          int32_t straight, uint16_t ticks, uint16_t activeMs,
                          uint8_t flushed)
{
  if (!gDebug) { return; }
  Serial.print(F("D end ls="));
  Serial.print(reason);
  Serial.print(F(" mL="));
  Serial.print(mL);
  Serial.print(F(" mR="));
  Serial.print(mR);
  Serial.print(F(" tr="));
  Serial.print(travel);
  Serial.print(F(" sd="));
  Serial.print(straight);
  Serial.print(F(" tk="));
  Serial.print(ticks);
  Serial.print(F(" ms="));
  Serial.print(activeMs);
  Serial.print(F(" fl="));
  Serial.print(flushed);
  Serial.print(F(" v="));
  Serial.println(readBatteryMillivolts());
}

void protocolEmitDebugStall(int32_t travel, int16_t boost)
{
  if (!gDebug) { return; }
  Serial.print(F("D stall tr="));
  Serial.print(travel);
  Serial.print(F(" boost="));
  Serial.print(boost);
  Serial.print(F(" v="));
  Serial.println(readBatteryMillivolts());
}

static void emitErr(const char *code)
{
  Serial.print(F("E "));
  Serial.println(code);
}

// Positional ints: skip spaces, parse up to maxN values into out[].
// Returns how many were present. Missing slots keep their pre-filled defaults.
static uint8_t parsePos(const char *args, int32_t *out, uint8_t maxN)
{
  uint8_t n = 0;
  const char *p = args;
  while (*p && n < maxN) {
    while (*p == ' ') { p++; }
    if (!*p) { break; }
    out[n++] = (int32_t)atol(p);
    while (*p && *p != ' ') { p++; }
  }
  return n;
}

static void handleLine(char *line)
{
  size_t n = strlen(line);
  while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == ' ')) {
    line[--n] = 0;
  }
  if (n == 0) { return; }

  char verb[16];
  const char *args = "";
  {
    size_t i = 0;
    while (line[i] && line[i] != ' ' && i < sizeof(verb) - 1) {
      char c = line[i];
      if (c >= 'a' && c <= 'z') { c = (char)(c - 'a' + 'A'); }
      verb[i++] = c;
    }
    verb[i] = 0;
    args = line[i] == ' ' ? line + i + 1 : "";
  }

  if (strcmp(verb, "DBG") == 0 || strcmp(verb, "DEBUG") == 0) {
    int32_t a[1] = {1};
    parsePos(args, a, 1);
    gDebug = (a[0] != 0) ? 1 : 0;
    Serial.print(F("D dbg="));
    Serial.println(gDebug);
    return;
  }
  if (strcmp(verb, "STOP") == 0) {
    motionStop(STOP_COMMAND);
    protocolEmitAck("stop");
    return;
  }
  if (strcmp(verb, "IDLE") == 0) {
    motionIdle(STOP_DURATION);
    protocolEmitAck("idle");
    return;
  }
  if (strcmp(verb, "CLEAR") == 0 || strcmp(verb, "ARM") == 0) {
    motionClearEstop();
    protocolEmitAck("clr");
    return;
  }
  if (strcmp(verb, "STATUS") == 0 || strcmp(verb, "ST") == 0) {
    protocolEmitStatus("st");
    return;
  }
  if (strcmp(verb, "HB") == 0 || strcmp(verb, "HEARTBEAT") == 0) {
    // HB [ttl]
    int32_t a[1] = {0};
    parsePos(args, a, 1);
    if (!motionHeartbeat((uint16_t)a[0])) {
      emitErr(gMotion.estop ? "estop" : "idle");
      return;
    }
    // No ACK on success — HB is high-rate; silence keeps TX buffer free.
    return;
  }
  if (strcmp(verb, "FW") == 0 || strcmp(verb, "FORWARD") == 0) {
    // FW [speed] [ttl] [dur]
    int32_t a[3] = {SPEED_DEFAULT, WATCHDOG_MS, 0};
    parsePos(args, a, 3);
    if (!motionEnqueueForward((int16_t)a[0], (uint16_t)a[1], (uint16_t)a[2])) {
      emitErr(gMotion.estop ? "estop" : "qfull");
      return;
    }
    protocolEmitAck("fw");
    return;
  }
  if (strcmp(verb, "BK") == 0 || strcmp(verb, "BACK") == 0 ||
      strcmp(verb, "BACKWARD") == 0) {
    // BK [speed] [ttl] [dur]
    int32_t a[3] = {SPEED_DEFAULT, WATCHDOG_MS, 0};
    parsePos(args, a, 3);
    if (!motionEnqueueBackward((int16_t)a[0], (uint16_t)a[1], (uint16_t)a[2])) {
      emitErr(gMotion.estop ? "estop" : "qfull");
      return;
    }
    protocolEmitAck("bk");
    return;
  }
  if (strcmp(verb, "LT") == 0 || strcmp(verb, "LEFT") == 0) {
    // LT [speed] [ttl] [counts] [dur]
    int32_t a[4] = {TURN_SPEED_DEFAULT, WATCHDOG_MS, 0, 0};
    parsePos(args, a, 4);
    if (!motionEnqueueTurnLeft((int16_t)a[0], (uint16_t)a[1], a[2], (uint16_t)a[3])) {
      emitErr(gMotion.estop ? "estop" : "qfull");
      return;
    }
    protocolEmitAck("lt");
    return;
  }
  if (strcmp(verb, "RT") == 0 || strcmp(verb, "RIGHT") == 0) {
    // RT [speed] [ttl] [counts] [dur]
    int32_t a[4] = {TURN_SPEED_DEFAULT, WATCHDOG_MS, 0, 0};
    parsePos(args, a, 4);
    if (!motionEnqueueTurnRight((int16_t)a[0], (uint16_t)a[1], a[2], (uint16_t)a[3])) {
      emitErr(gMotion.estop ? "estop" : "qfull");
      return;
    }
    protocolEmitAck("rt");
    return;
  }
  if (strcmp(verb, "RESET") == 0) {
    wheelsReset();
    protocolEmitAck("rst");
    return;
  }
  if (strcmp(verb, "HELP") == 0 || strcmp(verb, "?") == 0) {
    Serial.println(F("FW|BK speed ttl dur"));
    Serial.println(F("LT|RT speed ttl counts dur"));
    Serial.println(F("HB ttl | IDLE STOP CLEAR ST RESET | DBG 0|1"));
    return;
  }

  emitErr("unk");
}

void protocolSetup()
{
  lineLen = 0;
  lineBuf[0] = 0;
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 2000) {}
  // Drain any USB garbage from enumeration before we accept commands.
  delay(50);
  while (Serial.available() > 0) { (void)Serial.read(); }
  Serial.println(F("RDY"));
}

void protocolPoll()
{
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      if (lineLen > 0) {
        handleLine(lineBuf);
      }
      lineLen = 0;
    } else if (c == '\r') {
      // ignore
    } else if (c < 32 || c > 126) {
      // Drop binary/noise bytes; reset line so corruption cannot glue onto FW.
      lineLen = 0;
    } else if (lineLen + 1 < LINE_MAX) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
      emitErr("ovf");
    }
  }
}

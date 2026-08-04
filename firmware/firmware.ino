#include "Config.h"
#include "Motors.h"
#include "Motion.h"
#include "Protocol.h"

static uint32_t lastTickMs;

void setup()
{
  motorsSetup();
  motionSetup();
  protocolSetup();
  lastTickMs = millis();
}

void loop()
{
  protocolPoll();

  uint32_t now = millis();
  if ((uint16_t)(now - lastTickMs) >= CONTROL_MS) {
    lastTickMs = now;
    wheelsPoll();
    motionTick(now);
  }
}

#include "Motors.h"
#include "Config.h"
#include <avr/io.h>

Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
int16_t motorLeft;
int16_t motorRight;
int32_t wheelSpeed;
int32_t distanceLeft;
int32_t distanceRight;
int32_t travelAbs;
int32_t straightDiff;

static int16_t lastCountsLeft;
static int16_t lastCountsRight;

void motorsSetup()
{
  wheelsReset();
  motorsStop();
}

void motorsStop()
{
  // Brake = EN/PWM low. Do NOT call setSpeeds(0,0): Pololu's setter forces
  // DIR=forward whenever speed==0, which erases the last PHASE and makes
  // same-direction restart flaky on the DRV8838.
  motorLeft = 0;
  motorRight = 0;
  OCR1A = 0;
  OCR1B = 0;
}

void motorsDriveLR(int16_t left, int16_t right)
{
  if (left > MOTOR_LIMIT) { left = MOTOR_LIMIT; }
  if (left < -MOTOR_LIMIT) { left = -MOTOR_LIMIT; }
  if (right > MOTOR_LIMIT) { right = MOTOR_LIMIT; }
  if (right < -MOTOR_LIMIT) { right = -MOTOR_LIMIT; }
  motorLeft = left;
  motorRight = right;
  motors.setSpeeds((int16_t)(MOTOR_SIGN * left), (int16_t)(MOTOR_SIGN * right));
}

void wheelsReset()
{
  lastCountsLeft = encoders.getCountsLeft();
  lastCountsRight = encoders.getCountsRight();
  wheelSpeed = 0;
  distanceLeft = 0;
  distanceRight = 0;
  travelAbs = 0;
  straightDiff = 0;
}

void travelReset()
{
  lastCountsLeft = encoders.getCountsLeft();
  lastCountsRight = encoders.getCountsRight();
  travelAbs = 0;
  straightDiff = 0;
}

void wheelsPoll()
{
  int16_t cL = encoders.getCountsLeft();
  int16_t cR = encoders.getCountsRight();
  int32_t dL = (int32_t)ENCODER_SIGN * (cL - lastCountsLeft);
  int32_t dR = (int32_t)ENCODER_SIGN * (cR - lastCountsRight);
  lastCountsLeft = cL;
  lastCountsRight = cR;
  wheelSpeed = dL + dR;
  distanceLeft += dL;
  distanceRight += dR;
  straightDiff += dL - dR;
  int32_t aL = dL >= 0 ? dL : -dL;
  int32_t aR = dR >= 0 ? dR : -dR;
  travelAbs += aL + aR;
  if (distanceLeft > DISTANCE_CLAMP) { distanceLeft = DISTANCE_CLAMP; }
  if (distanceLeft < -DISTANCE_CLAMP) { distanceLeft = -DISTANCE_CLAMP; }
  if (distanceRight > DISTANCE_CLAMP) { distanceRight = DISTANCE_CLAMP; }
  if (distanceRight < -DISTANCE_CLAMP) { distanceRight = -DISTANCE_CLAMP; }
}

#pragma once
#include <stdint.h>
#include <Balboa32U4.h>

extern Balboa32U4Motors motors;
extern Balboa32U4Encoders encoders;
extern int16_t motorLeft;
extern int16_t motorRight;
extern int32_t wheelSpeed;
extern int32_t distanceLeft;
extern int32_t distanceRight;
extern int32_t travelAbs;  // |dL|+|dR| accumulated since last reset
extern int32_t straightDiff;  // Σ(dL − dR) since last travelReset

void motorsSetup();
void motorsStop();
void motorsDriveLR(int16_t left, int16_t right);
void wheelsReset();
void wheelsPoll();
void travelReset();

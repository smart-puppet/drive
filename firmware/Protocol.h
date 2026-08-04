#pragma once

#include <stdint.h>

extern uint8_t gDebug;  // 1 = emit D lines

void protocolSetup();
void protocolPoll();

void protocolEmitStatus(const char *event);
void protocolEmitAck(const char *event);

void protocolEmitDebugBegin(char mode, int16_t speed, uint16_t durMs, int32_t counts);
void protocolEmitDebugMid(char mode, int16_t mL, int16_t mR, int32_t travel,
                          int32_t straight, int32_t ws, int32_t tleft);
void protocolEmitDebugEnd(char reason, int16_t mL, int16_t mR, int32_t travel,
                          int32_t straight, uint16_t ticks, uint16_t activeMs,
                          uint8_t flushed);
void protocolEmitDebugStall(int32_t travel, int16_t boost);

/**
 * @file    console.h
 * @brief   Minimal serial console output helpers (no stdio, no heap).
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include "app_types.h"

void Console_PrintString(const char *text);
void Console_PrintLine(const char *text);
void Console_PrintNewLine(void);
void Console_PrintInt(int32_t value);
void Console_PrintUint(uint32_t value);
void Console_PrintFloat(float32_t value, uint8_t decimals);
void Console_PrintHex(uint32_t value);
bool Console_ReadChar(char *character);

#endif /* CONSOLE_H */

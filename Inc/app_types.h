/**
 * @file    app_types.h
 * @brief   Common fixed-width and floating-point type aliases for the application.
 *
 * MISRA C:2012 Dir 4.6: plain int/float are not used in application modules.
 */
#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/** 32-bit IEEE-754 single precision float (Cortex-M33 FPU). */
typedef float float32_t;

#endif /* APP_TYPES_H */

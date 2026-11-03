/**
 * @file    app_commands.h
 * @brief   Serial command interpreter (same command set as the NodeMCU firmware).
 *
 *   mr           MA782 configuration/status
 *   dt5..dt25    optional: re-measure encoder sign with a short drive pulse (also re-arms after a fault)
 *   ast / z      stationary check with the motor disabled
 *   g<deg>       one relative move; negative degrees = explicit reverse move
 *   r<deg>       repeatability: 10 moves, 4 s rest after each completion
 *   test / vol   volumetric schedule (10 x 10.61 deg @ 4 s start-to-start, 180 s, 10 batches)
 *   trace1/0     buffered 10 ms trace, printed after the move
 *   x            immediate disable/abort; a move in progress latches a fault
 *   clear        fault acknowledge (requires stationary output and valid MA782 config)
 *   s            status; ?  help
 */
#ifndef APP_COMMANDS_H
#define APP_COMMANDS_H

#include "app_types.h"

void Commands_PrintHelp(void);
void Commands_Poll(void);

#endif /* APP_COMMANDS_H */

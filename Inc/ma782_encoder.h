/**
 * @file    ma782_encoder.h
 * @brief   MA782 magnetic angle sensor driver (SPI mode 0, 16-bit frames).
 *
 * Frame formats (same as the validated NodeMCU firmware):
 *   angle read     : tx 0x0000, rx = 16-bit angle
 *   register read  : tx (0b010 << 13) | (addr << 8), data returned in the NEXT frame (low byte)
 *   register write : tx (0b100 << 13) | (addr << 8) | value, new value returned in the NEXT frame
 * Only volatile (RAM) configuration is written. Non-volatile memory is never written.
 */
#ifndef MA782_ENCODER_H
#define MA782_ENCODER_H

#include "app_types.h"

/** Snapshot of the MA782 configuration registers. */
typedef struct
{
    uint8_t  fw_register;
    uint8_t  rd_register;
    uint8_t  status_register;
    uint8_t  filter_window;
    uint8_t  rotation_direction;
    bool     config_ok;
} Ma782Config_t;

uint16_t Ma782_ReadAngleRaw(void);
uint8_t  Ma782_ReadRegister(uint8_t address);
uint8_t  Ma782_WriteRegister(uint8_t address, uint8_t value);
void     Ma782_ConfigureVolatile(void);
bool     Ma782_ReadConfiguration(Ma782Config_t *config);
bool     Ma782_ConfigurationPass(void);

#endif /* MA782_ENCODER_H */

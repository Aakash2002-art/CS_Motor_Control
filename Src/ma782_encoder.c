/**
 * @file    ma782_encoder.c
 * @brief   MA782 driver implementation.
 */
#include "ma782_encoder.h"
#include "app_config.h"
#include "bsp_hw.h"

#define MA782_REG_RD            (0x09U)
#define MA782_REG_FW            (0x0EU)
#define MA782_REG_STATUS        (0x1AU)

#define MA782_CMD_READ          (0x2U)   /* 0b010 */
#define MA782_CMD_WRITE         (0x4U)   /* 0b100 */
#define MA782_CMD_SHIFT         (13U)
#define MA782_ADDR_SHIFT        (8U)
#define MA782_ADDR_MASK         (0x1FU)
#define MA782_DATA_MASK         (0x00FFU)
#define MA782_FW_SHIFT          (4U)
#define MA782_FW_MASK           (0x0FU)
#define MA782_RD_SHIFT          (7U)
#define MA782_STATUS_ERR_MASK   (0x0EU)   /* ERRPAR | ERRMEM | ERRNVM */
#define MA782_FRAME_GAP_US      (1U)

/**
 * @brief Read the 16-bit angle: a frame of zeros returns the angle of the previous conversion.
 */
uint16_t Ma782_ReadAngleRaw(void)
{
    return Bsp_SpiTransferFrame16(0x0000U);
}

/**
 * @brief Read one MA782 register (command frame, then a data frame).
 */
uint8_t Ma782_ReadRegister(uint8_t address)
{
    const uint16_t command = (uint16_t)(((uint16_t)MA782_CMD_READ << MA782_CMD_SHIFT) |
                                        (uint16_t)(((uint16_t)address & MA782_ADDR_MASK) << MA782_ADDR_SHIFT));
    uint16_t response;

    (void)Bsp_SpiTransferFrame16(command);
    Bsp_DelayUs(MA782_FRAME_GAP_US);
    response = Bsp_SpiTransferFrame16(0x0000U);

    return (uint8_t)(response & MA782_DATA_MASK);
}

/**
 * @brief Write one MA782 register (volatile only; NVM is never written). Returns the read-back byte.
 */
uint8_t Ma782_WriteRegister(uint8_t address, uint8_t value)
{
    const uint16_t command = (uint16_t)(((uint16_t)MA782_CMD_WRITE << MA782_CMD_SHIFT) |
                                        (uint16_t)(((uint16_t)address & MA782_ADDR_MASK) << MA782_ADDR_SHIFT) |
                                        (uint16_t)value);
    uint16_t response;

    (void)Bsp_SpiTransferFrame16(command);
    Bsp_DelayUs(MA782_FRAME_GAP_US);
    response = Bsp_SpiTransferFrame16(0x0000U);

    return (uint8_t)(response & MA782_DATA_MASK);
}

/**
 * @brief Force the volatile configuration the algorithm relies on: filter window FW=12 and rotation direction RD=0.
 */
void Ma782_ConfigureVolatile(void)
{
    const uint8_t fw_old = Ma782_ReadRegister(MA782_REG_FW);
    const uint8_t fw_new = (uint8_t)((fw_old & 0x0FU) | (uint8_t)(APP_MA782_REQUIRED_FW << MA782_FW_SHIFT));
    const uint8_t rd_old = Ma782_ReadRegister(MA782_REG_RD);
    const uint8_t rd_new = (uint8_t)(rd_old & 0x7FU);   /* RD = 0 */

    (void)Ma782_WriteRegister(MA782_REG_FW, fw_new);
    (void)Ma782_WriteRegister(MA782_REG_RD, rd_new);
    Bsp_DelayUs(5U);
}

/**
 * @brief Read FW/RD/STATUS and decide config_ok (FW and RD as required, no parity/memory/NVM error bits).
 */
bool Ma782_ReadConfiguration(Ma782Config_t *config)
{
    bool ok = false;

    if (config != NULL)
    {
        config->fw_register = Ma782_ReadRegister(MA782_REG_FW);
        config->rd_register = Ma782_ReadRegister(MA782_REG_RD);
        config->status_register = Ma782_ReadRegister(MA782_REG_STATUS);
        config->filter_window = (uint8_t)((config->fw_register >> MA782_FW_SHIFT) & MA782_FW_MASK);
        config->rotation_direction = (uint8_t)((config->rd_register >> MA782_RD_SHIFT) & 0x01U);
        config->config_ok = ((uint32_t)config->filter_window == APP_MA782_REQUIRED_FW) &&
                            ((uint32_t)config->rotation_direction == APP_MA782_REQUIRED_RD) &&
                            ((config->status_register & MA782_STATUS_ERR_MASK) == 0U);
        ok = config->config_ok;
    }
    return ok;
}

/**
 * @brief Convenience wrapper: true if the MA782 configuration and status are valid.
 */
bool Ma782_ConfigurationPass(void)
{
    Ma782Config_t config;

    return Ma782_ReadConfiguration(&config);
}

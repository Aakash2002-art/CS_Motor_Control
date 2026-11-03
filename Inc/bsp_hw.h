/**
 * @file    bsp_hw.h
 * @brief   Board support layer: the ONLY module that touches STM32 HAL peripherals.
 *
 * Pin map (NUCLEO-U575ZI-Q):
 *   PE9  TIM1_CH1 -> DRV8411A AIN1        PE11 TIM1_CH2 -> DRV8411A AIN2
 *   PF13 EXTI13   <- DRV8411A nFAULT      PA3  ADC1_IN8 <- DRV8411A AIPROPI
 *   PA5  SPI1_SCK -> MA782 SCLK           PA6  SPI1_MISO <- MA782 MISO
 *   PA7  SPI1_MOSI-> MA782 MOSI           PA4  GPIO     -> MA782 /CS
 *   PA9  USART1_TX / PA10 USART1_RX       (ST-LINK virtual COM port)
 *
 * This file is excluded from the MISRA scope (HAL access), see README.md.
 */
#ifndef BSP_HW_H
#define BSP_HW_H

#include "app_types.h"

/** PWM output channel selector. */
typedef enum
{
    BSP_PWM_CHANNEL_AIN1 = 1,
    BSP_PWM_CHANNEL_AIN2 = 2
} BspPwmChannel_t;

/** Callback type used for interrupt-driven events. */
typedef void (*BspCallback_t)(void);

void     Bsp_SetSafeOutputs(void);
void     Bsp_Init(void);

void     Bsp_PwmSetCompare(BspPwmChannel_t channel, uint32_t compare_value);
uint32_t Bsp_PwmGetPeriodCounts(void);

uint32_t Bsp_GetMicros(void);
uint32_t Bsp_GetMillis(void);
void     Bsp_DelayUs(uint32_t delay_us);
void     Bsp_DelayMs(uint32_t delay_ms);

uint16_t Bsp_SpiTransferFrame16(uint16_t tx_word);

bool     Bsp_IsNFaultAsserted(void);
uint16_t Bsp_AdcReadCurrentRaw(void);

void     Bsp_UartWrite(const char *text, uint16_t length);
bool     Bsp_UartReadByte(uint8_t *byte);

void     Bsp_SetControlTickCallback(BspCallback_t callback);
void     Bsp_SetFaultCallback(BspCallback_t callback);
void     Bsp_ControlTickStart(void);
void     Bsp_ControlTickSuspend(void);
void     Bsp_ControlTickResume(void);

void     Bsp_WatchdogStart(void);
void     Bsp_WatchdogRefresh(void);

/* Interrupt entry points called from stm32u5xx_it.c */
void     Bsp_Tim6IrqHandler(void);
void     Bsp_Usart1IrqHandler(void);
void     Bsp_NFaultExtiIrqHandler(void);

#endif /* BSP_HW_H */

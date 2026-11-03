/**
 ******************************************************************************
 * @file    main.c
 * @brief   Program entry: clock setup, then hand over to the application.
 *          Clock tree derived from the STM32Cube NUCLEO-U575ZI-Q template
 *          (160 MHz from MSI/PLL, all bus prescalers 1).
 ******************************************************************************
 */
#include "main.h"
#include "app_main.h"
#include "bsp_hw.h"

static void SystemClock_Config(void);
static void CACHE_Enable(void);

int main(void)
{
    HAL_Init();

    /* SysTick must preempt the 2 kHz control interrupt so HAL timeouts used inside it can expire. */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0U, 0U);

    /* Bridge inputs low (coast) as early as possible after reset. */
    Bsp_SetSafeOutputs();

    CACHE_Enable();
    SystemClock_Config();

    App_Init();
    App_Run();

    return 0;
}

/**
 * @brief  System clock: PLL from MSI 4 MHz, SYSCLK = HCLK = PCLK1/2/3 = 160 MHz.
 *         Timer clocks therefore equal 160 MHz (used by bsp_hw.c).
 */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef clk_init = {0};
    RCC_OscInitTypeDef osc_init = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        for (;;)
        {
        }
    }
    if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
    {
        for (;;)
        {
        }
    }
    __HAL_RCC_PWR_CLK_DISABLE();

    osc_init.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    osc_init.MSIState = RCC_MSI_ON;
    osc_init.MSIClockRange = RCC_MSIRANGE_4;
    osc_init.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    osc_init.PLL.PLLState = RCC_PLL_ON;
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    osc_init.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
    osc_init.PLL.PLLM = 1U;
    osc_init.PLL.PLLN = 80U;
    osc_init.PLL.PLLR = 2U;
    osc_init.PLL.PLLP = 2U;
    osc_init.PLL.PLLQ = 2U;
    osc_init.PLL.PLLFRACN = 0U;
    osc_init.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)
    {
        for (;;)
        {
        }
    }

    clk_init.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
                          RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3);
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
    clk_init.APB3CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_4) != HAL_OK)
    {
        for (;;)
        {
        }
    }
}

static void CACHE_Enable(void)
{
    (void)HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY);
    (void)HAL_ICACHE_Enable();
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    for (;;)
    {
    }
}
#endif

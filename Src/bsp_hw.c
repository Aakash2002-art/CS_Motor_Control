/**
 * @file    bsp_hw.c
 * @brief   Board support layer implementation (STM32U575ZI HAL wrappers).
 *
 * Peripheral allocation:
 *   TIM1  : 20 kHz edge-aligned PWM, CH1/CH2 -> AIN1/AIN2
 *   TIM2  : 32-bit free-running 1 MHz microsecond time base
 *   TIM6  : 2 kHz control tick interrupt
 *   SPI1  : MA782, 16-bit frames, mode 0, 5 MHz
 *   ADC1  : IN8 (PA3) motor current from AIPROPI
 *   USART1: ST-LINK virtual COM port, interrupt-driven RX ring buffer
 *   IWDG  : independent watchdog
 */
#include "bsp_hw.h"
#include "app_config.h"
#include "stm32u5xx_hal.h"

/* ------------------------------------------------------------------------- */
/* Pin definitions                                                            */
/* ------------------------------------------------------------------------- */
#define PWM_AIN1_PORT       GPIOE
#define PWM_AIN1_PIN        GPIO_PIN_9
#define PWM_AIN2_PORT       GPIOE
#define PWM_AIN2_PIN        GPIO_PIN_11
#define NFAULT_PORT         GPIOF
#define NFAULT_PIN          GPIO_PIN_13
#define CURRENT_PORT        GPIOA
#define CURRENT_PIN         GPIO_PIN_3
#define SPI_SCK_PIN         GPIO_PIN_5
#define SPI_MISO_PIN        GPIO_PIN_6
#define SPI_MOSI_PIN        GPIO_PIN_7
#define SPI_PORT            GPIOA
#define SPI_CS_PORT         GPIOA
#define SPI_CS_PIN          GPIO_PIN_4
#define UART_PORT           GPIOA
#define UART_TX_PIN         GPIO_PIN_9
#define UART_RX_PIN         GPIO_PIN_10

#define SPI_TIMEOUT_MS      (2U)
#define UART_TX_TIMEOUT_MS  (200U)
#define SPI_PRESCALER       (SPI_BAUDRATEPRESCALER_32)   /* 160 MHz / 32 = 5 MHz */
#define ADC_TIMEOUT_MS      (2U)
#define UART_RX_RING_SIZE   (128U)
#define TIMER_1MHZ_PRESCALER (159U)                      /* 160 MHz / 160 = 1 MHz */
#define LSI_PRESCALER_DIV   (32U)

/* ------------------------------------------------------------------------- */
/* Module state                                                               */
/* ------------------------------------------------------------------------- */
static TIM_HandleTypeDef  g_htim1;
static TIM_HandleTypeDef  g_htim2;
static TIM_HandleTypeDef  g_htim6;
static SPI_HandleTypeDef  g_hspi1;
static ADC_HandleTypeDef  g_hadc1;
static UART_HandleTypeDef g_huart1;
static IWDG_HandleTypeDef g_hiwdg;

static uint32_t g_pwm_period_counts = 0U;
static BspCallback_t g_control_tick_callback = NULL;
static BspCallback_t g_fault_callback = NULL;

static volatile uint8_t  g_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t g_rx_head = 0U;
static volatile uint16_t g_rx_tail = 0U;
static uint8_t           g_rx_byte = 0U;

/* ------------------------------------------------------------------------- */
/* Private helpers                                                            */
/* ------------------------------------------------------------------------- */
static void Bsp_ErrorTrap(void)
{
    /* A peripheral failed to initialise: keep the bridge off and stop here;
       the independent watchdog (if running) will reset the device. */
    Bsp_SetSafeOutputs();
    for (;;)
    {
    }
}

/**
 * @brief Enable the GPIO clocks and park the outputs safely: AIN1/AIN2 low (coast) and MA782 chip select high.
 */
static void Bsp_InitGpioClocks(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
}

/**
 * @brief TIM1 PWM on PE9/PE11 at APP_PWM_FREQUENCY_HZ. Compares start at the coast pattern before the pins are handed to the timer.
 *         With APP_COMPLEMENTARY_DRIVE, AIN2 uses PWM mode 2 so it is the inverse of AIN1.
 */
static void Bsp_InitPwm(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};
    TIM_MasterConfigTypeDef master = {0};
    uint32_t timer_clock_hz;

    __HAL_RCC_TIM1_CLK_ENABLE();

    /* APB2 prescaler is 1 in SystemClock_Config, so the timer clock equals PCLK2. */
    timer_clock_hz = HAL_RCC_GetPCLK2Freq();
    g_pwm_period_counts = timer_clock_hz / APP_PWM_FREQUENCY_HZ;

    g_htim1.Instance = TIM1;
    g_htim1.Init.Prescaler = 0U;
    g_htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_htim1.Init.Period = g_pwm_period_counts - 1U;
    g_htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_htim1.Init.RepetitionCounter = 0U;
    g_htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&g_htim1) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&g_htim1, &master) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }

    /* Both outputs start at 0 % (low = coast). */
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if ((HAL_TIM_PWM_ConfigChannel(&g_htim1, &oc, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&g_htim1, &oc, TIM_CHANNEL_2) != HAL_OK))
    {
        Bsp_ErrorTrap();
    }

#if (APP_COMPLEMENTARY_DRIVE != 0)
    /* AIN2 = inverse of AIN1 for the same compare: AIN2 is high while CNT >= compare. */
    oc.OCMode = TIM_OCMODE_PWM2;
    oc.Pulse = g_pwm_period_counts;   /* PWM2 with compare = period keeps AIN2 low (coast) */
    if (HAL_TIM_PWM_ConfigChannel(&g_htim1, &oc, TIM_CHANNEL_2) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }
#endif

    __HAL_TIM_ENABLE_OCxPRELOAD(&g_htim1, TIM_CHANNEL_1);
    __HAL_TIM_ENABLE_OCxPRELOAD(&g_htim1, TIM_CHANNEL_2);

    if ((HAL_TIM_PWM_Start(&g_htim1, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&g_htim1, TIM_CHANNEL_2) != HAL_OK))
    {
        Bsp_ErrorTrap();
    }

    /* Hand the pins to the timer only after both compares are at zero. */
    gpio.Pin = PWM_AIN1_PIN | PWM_AIN2_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gpio);
}

/**
 * @brief TIM2: free-running 32-bit 1 MHz counter used for all microsecond timing and delays.
 */
static void Bsp_InitTimeBase(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    g_htim2.Instance = TIM2;
    g_htim2.Init.Prescaler = TIMER_1MHZ_PRESCALER;
    g_htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_htim2.Init.Period = 0xFFFFFFFFU;
    g_htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&g_htim2) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }
    if (HAL_TIM_Base_Start(&g_htim2) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }
}

/**
 * @brief TIM6: update interrupt every APP_CONTROL_TICK_US (2 kHz control tick).
 */
static void Bsp_InitControlTick(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    g_htim6.Instance = TIM6;
    g_htim6.Init.Prescaler = TIMER_1MHZ_PRESCALER;
    g_htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_htim6.Init.Period = APP_CONTROL_TICK_US - 1U;
    g_htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&g_htim6) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }
    HAL_NVIC_SetPriority(TIM6_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

/**
 * @brief SPI1 master, mode 0, 16-bit frames, MSB first, software chip select (PA4) for the MA782.
 */
static void Bsp_InitSpi(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_SPI1_CLK_ENABLE();

    gpio.Pin = SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(SPI_PORT, &gpio);

    /* Chip select is a plain GPIO, idle high. */
    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
    gpio.Pin = SPI_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = 0U;
    HAL_GPIO_Init(SPI_CS_PORT, &gpio);

    g_hspi1.Instance = SPI1;
    g_hspi1.Init.Mode = SPI_MODE_MASTER;
    g_hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    g_hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
    g_hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    g_hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    g_hspi1.Init.NSS = SPI_NSS_SOFT;
    g_hspi1.Init.BaudRatePrescaler = SPI_PRESCALER;
    g_hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    g_hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    g_hspi1.Init.CRCPolynomial = 7U;
    g_hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    g_hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    g_hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    g_hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    g_hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    g_hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    g_hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    g_hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    g_hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&g_hspi1) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }
}

/**
 * @brief ADC1 single conversion on the DRV8411A IPROPI pin (motor current mirror).
 */
static void Bsp_InitAdc(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clk = {0};
    ADC_ChannelConfTypeDef channel = {0};

    clk.PeriphClockSelection = RCC_PERIPHCLK_ADCDAC;
    clk.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableVddA();

    gpio.Pin = CURRENT_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CURRENT_PORT, &gpio);

    g_hadc1.Instance = ADC1;
    g_hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    g_hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    g_hadc1.Init.GainCompensation = 0U;
    g_hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    g_hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    g_hadc1.Init.LowPowerAutoWait = DISABLE;
    g_hadc1.Init.ContinuousConvMode = DISABLE;
    g_hadc1.Init.NbrOfConversion = 1U;
    g_hadc1.Init.DiscontinuousConvMode = DISABLE;
    g_hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_hadc1.Init.DMAContinuousRequests = DISABLE;
    g_hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    g_hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&g_hadc1) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }
    if (HAL_ADCEx_Calibration_Start(&g_hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }

    channel.Channel = ADC_CHANNEL_8;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_68CYCLES;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&g_hadc1, &channel) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }
}

/**
 * @brief USART1 115200 8N1 (ST-LINK virtual COM port), interrupt-driven receive into a ring buffer.
 */
static void Bsp_InitUart(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_USART1_CLK_ENABLE();

    gpio.Pin = UART_TX_PIN | UART_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(UART_PORT, &gpio);

    g_huart1.Instance = USART1;
    g_huart1.Init.BaudRate = APP_UART_BAUD;
    g_huart1.Init.WordLength = UART_WORDLENGTH_8B;
    g_huart1.Init.StopBits = UART_STOPBITS_1;
    g_huart1.Init.Parity = UART_PARITY_NONE;
    g_huart1.Init.Mode = UART_MODE_TX_RX;
    g_huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    g_huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    g_huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    g_huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&g_huart1) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }

    HAL_NVIC_SetPriority(USART1_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (HAL_UART_Receive_IT(&g_huart1, &g_rx_byte, 1U) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }
}

/**
 * @brief nFAULT input PF13: falling-edge EXTI interrupt, internal pull-up as a backup to the EVM pull-up.
 */
static void Bsp_InitNFault(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* nFAULT is open drain, low = fault. An external pull-up is required;
       the internal pull-up is only a backup. */
    gpio.Pin = NFAULT_PIN;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NFAULT_PORT, &gpio);

    HAL_NVIC_SetPriority(EXTI13_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(EXTI13_IRQn);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */
void Bsp_SetSafeOutputs(void)
{
    GPIO_InitTypeDef gpio = {0};

    Bsp_InitGpioClocks();

    /* AIN1/AIN2 driven low (coast) and the MA782 chip select high. */
    HAL_GPIO_WritePin(GPIOE, PWM_AIN1_PIN | PWM_AIN2_PIN, GPIO_PIN_RESET);
    gpio.Pin = PWM_AIN1_PIN | PWM_AIN2_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);

    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
    gpio.Pin = SPI_CS_PIN;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SPI_CS_PORT, &gpio);
}

/**
 * @brief Initialise all board peripherals in a safe order.
 */
void Bsp_Init(void)
{
    Bsp_InitGpioClocks();
    Bsp_InitTimeBase();
    Bsp_InitPwm();
    Bsp_InitSpi();
    Bsp_InitAdc();
    Bsp_InitUart();
    Bsp_InitControlTick();
    Bsp_InitNFault();
}

/**
 * @brief Write the TIM1 compare value of one bridge input channel.
 */
void Bsp_PwmSetCompare(BspPwmChannel_t channel, uint32_t compare_value)
{
    if (channel == BSP_PWM_CHANNEL_AIN1)
    {
        __HAL_TIM_SET_COMPARE(&g_htim1, TIM_CHANNEL_1, compare_value);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&g_htim1, TIM_CHANNEL_2, compare_value);
    }
}

/**
 * @brief Timer counts per PWM period (the 100 % duty reference).
 */
uint32_t Bsp_PwmGetPeriodCounts(void)
{
    return g_pwm_period_counts;
}

/**
 * @brief Free-running microsecond counter (wraps at 2^32, use unsigned subtraction for intervals).
 */
uint32_t Bsp_GetMicros(void)
{
    return __HAL_TIM_GET_COUNTER(&g_htim2);
}

/**
 * @brief Millisecond tick from the HAL.
 */
uint32_t Bsp_GetMillis(void)
{
    return HAL_GetTick();
}

/**
 * @brief Busy-wait on the microsecond counter.
 */
void Bsp_DelayUs(uint32_t delay_us)
{
    const uint32_t start_us = Bsp_GetMicros();

    while ((Bsp_GetMicros() - start_us) < delay_us)
    {
        /* busy wait on the free-running 1 MHz counter */
    }
}

/**
 * @brief Busy-wait delay in milliseconds (blocks; never call while a move is running).
 */
void Bsp_DelayMs(uint32_t delay_ms)
{
    Bsp_DelayUs(delay_ms * 1000U);
}

/**
 * @brief One 16-bit full-duplex SPI frame with the chip select asserted around it. Returns the received word
 *         (0 if the HAL transfer fails).
 */
uint16_t Bsp_SpiTransferFrame16(uint16_t tx_word)
{
    uint16_t rx_word = 0U;
    uint16_t tx_copy = tx_word;

    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);
    Bsp_DelayUs(1U);
    if (HAL_SPI_TransmitReceive(&g_hspi1, (uint8_t *)&tx_copy, (uint8_t *)&rx_word, 1U,
                                SPI_TIMEOUT_MS) != HAL_OK)
    {
        rx_word = 0U;
    }
    Bsp_DelayUs(1U);
    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
    Bsp_DelayUs(1U);

    return rx_word;
}

/**
 * @brief True when the open-drain nFAULT line is pulled low by the DRV8411A.
 */
bool Bsp_IsNFaultAsserted(void)
{
    return (HAL_GPIO_ReadPin(NFAULT_PORT, NFAULT_PIN) == GPIO_PIN_RESET);
}

/**
 * @brief Blocking single ADC conversion of the IPROPI voltage (12-bit raw).
 */
uint16_t Bsp_AdcReadCurrentRaw(void)
{
    uint16_t result = 0U;

    if (HAL_ADC_Start(&g_hadc1) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(&g_hadc1, ADC_TIMEOUT_MS) == HAL_OK)
        {
            result = (uint16_t)HAL_ADC_GetValue(&g_hadc1);
        }
        (void)HAL_ADC_Stop(&g_hadc1);
    }
    return result;
}

/**
 * @brief Blocking transmit of a text buffer on USART1.
 */
void Bsp_UartWrite(const char *text, uint16_t length)
{
    (void)HAL_UART_Transmit(&g_huart1, (const uint8_t *)text, length, UART_TX_TIMEOUT_MS);
}

/**
 * @brief Pop one received byte from the RX ring buffer; false if empty.
 */
bool Bsp_UartReadByte(uint8_t *byte)
{
    bool available = false;

    if (g_rx_head != g_rx_tail)
    {
        *byte = g_rx_ring[g_rx_tail];
        g_rx_tail = (uint16_t)((g_rx_tail + 1U) % UART_RX_RING_SIZE);
        available = true;
    }
    return available;
}

/**
 * @brief Register the function called from the 2 kHz TIM6 interrupt.
 */
void Bsp_SetControlTickCallback(BspCallback_t callback)
{
    g_control_tick_callback = callback;
}

/**
 * @brief Register the function called from the nFAULT falling-edge interrupt.
 */
void Bsp_SetFaultCallback(BspCallback_t callback)
{
    g_fault_callback = callback;
}

/**
 * @brief Start the TIM6 interrupt (the control loop begins running).
 */
void Bsp_ControlTickStart(void)
{
    (void)HAL_TIM_Base_Start_IT(&g_htim6);
}

/**
 * @brief Mask the control-tick interrupt so blocking SPI/stationary routines own the bus.
 *         Always paired with Bsp_ControlTickResume; do not hold it for more than a few milliseconds during a move.
 */
void Bsp_ControlTickSuspend(void)
{
    HAL_NVIC_DisableIRQ(TIM6_IRQn);
}

/**
 * @brief Unmask the control-tick interrupt.
 */
void Bsp_ControlTickResume(void)
{
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

/**
 * @brief Start the independent watchdog (reload from APP_WATCHDOG_RELOAD_MS; frozen while debugging).
 */
void Bsp_WatchdogStart(void)
{
    /* LSI is nominally 32 kHz: /32 prescaler gives a 1 kHz watchdog clock. */
    g_hiwdg.Instance = IWDG;
    g_hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    g_hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    g_hiwdg.Init.Reload = (APP_WATCHDOG_RELOAD_MS * 32000U) / (LSI_PRESCALER_DIV * 1000U);
    g_hiwdg.Init.EWI = 0U;
    __HAL_DBGMCU_FREEZE_IWDG();
    if (HAL_IWDG_Init(&g_hiwdg) != HAL_OK)
    {
        Bsp_ErrorTrap();
    }
}

/**
 * @brief Kick the independent watchdog.
 */
void Bsp_WatchdogRefresh(void)
{
    (void)HAL_IWDG_Refresh(&g_hiwdg);
}

/* ------------------------------------------------------------------------- */
/* Interrupt entry points and HAL callbacks                                   */
/* ------------------------------------------------------------------------- */
void Bsp_Tim6IrqHandler(void)
{
    HAL_TIM_IRQHandler(&g_htim6);
}

/**
 * @brief USART1 interrupt entry (called from stm32u5xx_it.c).
 */
void Bsp_Usart1IrqHandler(void)
{
    HAL_UART_IRQHandler(&g_huart1);
}

/**
 * @brief EXTI13 interrupt entry (called from stm32u5xx_it.c).
 */
void Bsp_NFaultExtiIrqHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(NFAULT_PIN);
}

/**
 * @brief HAL callback: TIM6 update -> control tick callback.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM6) && (g_control_tick_callback != NULL))
    {
        g_control_tick_callback();
    }
}

/**
 * @brief HAL callback: nFAULT falling edge -> fault callback.
 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t gpio_pin)
{
    if ((gpio_pin == NFAULT_PIN) && (g_fault_callback != NULL))
    {
        g_fault_callback();
    }
}

/**
 * @brief HAL callback: store the received byte in the ring buffer and re-arm the receive interrupt.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        const uint16_t next_head = (uint16_t)((g_rx_head + 1U) % UART_RX_RING_SIZE);

        if (next_head != g_rx_tail)
        {
            g_rx_ring[g_rx_head] = g_rx_byte;
            g_rx_head = next_head;
        }
        (void)HAL_UART_Receive_IT(&g_huart1, &g_rx_byte, 1U);
    }
}

/**
 * @brief HAL callback: clear UART error flags and keep receiving.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* Overrun/noise/framing: clear and keep receiving. */
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        (void)HAL_UART_Receive_IT(&g_huart1, &g_rx_byte, 1U);
    }
}

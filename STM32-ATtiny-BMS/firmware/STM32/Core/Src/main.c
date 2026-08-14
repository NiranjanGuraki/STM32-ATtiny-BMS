/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Multi-module BMS with interrupt-driven
  *                    ADC current sensing and flag-based SOC timer updates
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* CHANGE 1: Added __attribute__((packed)) - previously missing here, present
   only on the ATtiny side. This guarantees identical byte layout across the
   ARM and AVR compilers instead of relying on lucky natural alignment. */
typedef struct __attribute__((packed))
{
    uint8_t  start;
    uint8_t  module_id;
    uint16_t cell1_mv;
    uint16_t cell2_mv;
    uint16_t cell3_mv;
    uint16_t temp_x10;
    uint8_t  fault;
    uint8_t  checksum;
} Packet_t;

typedef struct __attribute__((packed))
{
    uint8_t start;
    uint8_t balance_mask;
} BalanceCmd_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NUM_MODULES           2
#define BATTERY_CAPACITY_AH   5.0f
#define BALANCE_THRESHOLD_MV  30
#define SOC_TICK_SECONDS      0.1f   /* must match your actual TIM2 period - see MX_TIM2_Init changes */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */

uint8_t CalculateChecksum(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

/* CHANGE 6: GetBalanceMask now takes the pre-computed GLOBAL minimum across
   every module, not this module's own local minimum. This fixes the bug
   where a module could look "internally balanced" while being pack-wide
   out of balance relative to other modules. */
uint8_t GetBalanceMask(Packet_t *pkt, uint16_t globalMin)
{
    uint8_t mask = 0;

    if ((pkt->cell1_mv - globalMin) > BALANCE_THRESHOLD_MV)
        mask |= 0x01;

    if ((pkt->cell2_mv - globalMin) > BALANCE_THRESHOLD_MV)
        mask |= 0x02;

    if ((pkt->cell3_mv - globalMin) > BALANCE_THRESHOLD_MV)
        mask |= 0x04;

    return mask;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* CHANGE 2: Arrays instead of single globals, one slot per module. */
uint8_t   moduleAddrs[NUM_MODULES] = {0x10, 0x11};
Packet_t  modulePackets[NUM_MODULES];
uint8_t   moduleValid[NUM_MODULES];

BalanceCmd_t txCmd;
char msg[100];

float batteryCurrent = 0.0f;

/* TODO: soc should be loaded from flash at boot instead of hardcoded 100.0f -
   see our earlier discussion on SOC persistence. Not implemented in this pass;
   flagging so it isn't forgotten. */
float soc = 100.0f;

/* CHANGE 2 (cont): flag/counter for deferred SOC processing, set only inside
   the timer ISR, consumed in the main loop. MUST be volatile - written in an
   ISR, read in main(). */
volatile uint32_t socTickCount = 0;

/* CHANGE 5: ADC interrupt-driven state. Both MUST be volatile for the same
   reason as socTickCount - written in HAL_ADC_ConvCpltCallback (ISR context),
   read in the main loop. */
volatile uint8_t  adcReady = 0;
volatile uint16_t latestAdcValue = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* CHANGE 3: Removed the premature MX_TIM2_Init() + HAL_TIM_Base_Start_IT()
     pair that used to run here, BEFORE SystemClock_Config(). Starting a timer
     before the clock tree is configured means its prescaler/period math runs
     against the wrong (default/reset) clock speed. The timer is now
     initialized and started only once, after the clock is correctly set up
     and every other peripheral is ready - see below. */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  /* Timer started once, after clocks + all peripherals are correctly configured. */
  HAL_TIM_Base_Start_IT(&htim2);

  /* CHANGE 3 (cont): ADC armed once here - this is the "prime the pump" call.
     Single-conversion mode (ContinuousConvMode stays DISABLE in
     MX_ADC1_Init), software-triggered, interrupt-driven completion. The next
     conversion is only re-armed manually, from inside the main loop, after
     each result is consumed - see the adcReady block below. */
  HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* ============================================================
       ADC: non-blocking. Only does work when a conversion has
       actually completed (adcReady set by the ISR). Re-arms the
       next conversion right after consuming the result - never
       called unconditionally, since the ADC must finish its
       current conversion before Start_IT() can safely be called
       again in single-conversion mode.
       ============================================================ */
    if (adcReady)
    {
        adcReady = 0;

        float voltage = (latestAdcValue * 3.3f) / 4095.0f;
        batteryCurrent = voltage / 0.01f;   /* shunt + INA180 gain bundled into this constant */

        HAL_ADC_Start_IT(&hadc1);   /* re-arm next conversion */
    }

    /* ============================================================
       I2C: PHASE 1 - GATHER
       Poll every module before deciding anything. Stays blocking -
       this is a deliberate choice, not an oversight: the loop's
       control flow is inherently sequential (need data from all
       modules before a balancing decision can be made), so
       interrupt-driving I2C here would require a state machine for
       no real benefit at this module count / loop rate.
       ============================================================ */
    for (uint8_t i = 0; i < NUM_MODULES; i++)
    {
        uint8_t buf[sizeof(Packet_t)];

        HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
            &hi2c1,
            (moduleAddrs[i] << 1),
            buf,
            sizeof(Packet_t),
            100);

        if (status == HAL_OK)
        {
            Packet_t temp;
            memcpy(&temp, buf, sizeof(Packet_t));

            uint8_t crc = CalculateChecksum(buf, sizeof(Packet_t) - 1);

            if (crc == temp.checksum)
            {
                modulePackets[i] = temp;
                moduleValid[i] = 1;

                sprintf(msg, "M%d: ID=%d C1=%dmV C2=%dmV C3=%dmV Temp=%d\r\n",
                        i,
                        temp.module_id,
                        temp.cell1_mv,
                        temp.cell2_mv,
                        temp.cell3_mv,
                        temp.temp_x10);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            }
            else
            {
                moduleValid[i] = 0;   /* checksum mismatch - don't trust this round's data */
            }
        }
        else
        {
            moduleValid[i] = 0;   /* no response / timeout - don't trust this round's data */
        }
    }

    /* ============================================================
       PHASE 2 - DECIDE
       Find the true global minimum cell voltage across every
       module that reported valid data this round. This is the fix
       for the "each module only compares against its own local
       minimum" bug - without this, uniformly-high modules would
       never balance even if genuinely high relative to the pack.
       ============================================================ */
    uint16_t globalMin = 0xFFFF;
    uint8_t anyValid = 0;

    for (uint8_t i = 0; i < NUM_MODULES; i++)
    {
        if (!moduleValid[i]) continue;

        anyValid = 1;

        if (modulePackets[i].cell1_mv < globalMin) globalMin = modulePackets[i].cell1_mv;
        if (modulePackets[i].cell2_mv < globalMin) globalMin = modulePackets[i].cell2_mv;
        if (modulePackets[i].cell3_mv < globalMin) globalMin = modulePackets[i].cell3_mv;
    }

    /* ============================================================
       PHASE 3 - COMMAND
       Send each module its mask relative to the global minimum.
       Fail-safe: any module whose data wasn't valid this round
       gets mask = 0 (no balancing) rather than acting on stale or
       untrusted data - this fixes the original bug where
       GetBalanceMask ran unconditionally every loop regardless of
       I2C/checksum status.
       ============================================================ */
    for (uint8_t i = 0; i < NUM_MODULES; i++)
    {
        uint8_t mask = 0;

        if (anyValid && moduleValid[i])
        {
            mask = GetBalanceMask(&modulePackets[i], globalMin);
        }

        txCmd.start = 0xAA;
        txCmd.balance_mask = mask;

        HAL_I2C_Master_Transmit(
            &hi2c1,
            (moduleAddrs[i] << 1),
            (uint8_t*)&txCmd,
            sizeof(BalanceCmd_t),
            100);

        sprintf(msg, "M%d: Mask=0x%02X Valid=%d\r\n", i, mask, moduleValid[i]);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }

    /* ============================================================
       SOC: deferred processing. The timer ISR only increments
       socTickCount (see HAL_TIM_PeriodElapsedCallback below) - all
       the actual math happens here, in the interruptible main loop.

       Read-and-clear is wrapped in __disable_irq()/__enable_irq()
       to make it atomic: reading socTickCount and clearing it are
       two separate operations, and without this guard a timer tick
       landing exactly between them would be silently lost. This is
       a very short critical section (a few instructions), not a
       long block - safe to briefly disable interrupts here.

       Backlog is collapsed into a single lump-sum calculation
       (ticks * SOC_TICK_SECONDS) using the current instantaneous
       batteryCurrent, rather than a loop of repeated subtraction.
       This assumes current was roughly constant across the backlog
       window - an approximation that's reasonable when backlogs
       are small (expected here, since I2C timeouts are bounded and
       loop time is well under the 100ms tick period in the normal
       case), but degrades if backlogs grow large. A more precise
       fix would snapshot batteryCurrent per-tick inside the timer
       ISR itself - not implemented here to keep this at the
       complexity level we agreed on, but worth naming if asked.
       ============================================================ */
    if (socTickCount > 0)
    {
        __disable_irq();
        uint32_t ticks = socTickCount;
        socTickCount = 0;
        __enable_irq();

        soc -= (batteryCurrent * (ticks * SOC_TICK_SECONDS))
               / (BATTERY_CAPACITY_AH * 3600.0f)
               * 100.0f;

        if (soc < 0)   soc = 0;
        if (soc > 100) soc = 100;
    }

    /* CHANGE 7: The mid-loop HAL_Delay(100) has been removed. It's no longer
       needed for SOC timing pacing now that the timer ISR handles that
       independently via socTickCount. Loop cadence is now bounded by the
       I2C transactions' own timeouts (100ms each, times NUM_MODULES reads
       and writes) rather than an explicit delay. If you want a deliberate
       minimum loop period for any other reason (e.g. rate-limiting UART
       traffic), reintroduce a non-blocking millis()-style check here rather
       than a blocking HAL_Delay. */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;

  /* CHANGE 5 (cont): ContinuousConvMode stays DISABLE - deliberately kept as
     single-conversion mode. Continuous mode would make the ADC free-run and
     fire interrupts as fast as hardware allows, far faster than this ~10Hz
     current-sense use case needs. Single-conversion + manual re-arm from the
     main loop naturally paces sampling to match loop cadence instead. */
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

  /* CHANGE 4: Period changed from 4294967295 (max uint32 - effectively never
     overflows, meaning HAL_TIM_PeriodElapsedCallback essentially never fired)
     to 99, which - combined with the Prescaler above - produces a real
     100ms overflow period.

     Math: assuming a 16MHz timer input clock (verify this against YOUR
     actual SystemClock_Config output - APB1 timer clock specifically),
     Prescaler=15999 means the counter increments at
     16,000,000 / (15999+1) = 1000 Hz, i.e. once per millisecond.
     Period=99 means it counts 0..99 (100 counts) before overflowing,
     so it overflows every 100 x 1ms = 100ms - matching SOC_TICK_SECONDS
     above. If your actual timer clock differs, recalculate Period
     accordingly: Period = (TimerClockHz / (Prescaler+1)) * SOC_TICK_SECONDS - 1 */
  htim2.Init.Period = 99;

  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/* CHANGE 4 (cont): Timer ISR now only sets a flag/counter - no math happens
   here. Keeps the ISR as short as possible, per the "deferred processing"
   pattern - actual SOC calculation runs in the main loop where it can be
   interrupted by anything more time-critical (I2C, ADC). */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        socTickCount++;
    }
}

/* CHANGE 5 (cont): New ISR - fires automatically when an ADC conversion
   started via HAL_ADC_Start_IT() completes. Only stores the raw result and
   sets a flag; the voltage/current conversion math happens in the main loop,
   same deferred-processing reasoning as the timer ISR above. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        latestAdcValue = HAL_ADC_GetValue(hadc);
        adcReady = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */

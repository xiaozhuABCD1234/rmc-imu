#include "main.h"

#include <stdbool.h>

#include "bmi088.h"
#include "imu_heater.h"
#include "imu_task.h"
#include "justfloat.h"

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;
#if APP_ENABLE_IWDG
IWDG_HandleTypeDef hiwdg;
#endif

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM10_Init(void);
#if APP_ENABLE_IWDG
static void MX_IWDG_Init(void);
#endif

static void Led_Write(bool red, bool green, bool blue);
static void Led_Indicate(bool imu_ready, uint32_t tick_ms);

/*
 * The startup file calls __libc_init_array, which calls _init. With
 * -nostartfiles no crti.o is linked, so it has to be provided here.
 */
void _init(void) {
}

int main(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
#if APP_ENABLE_IWDG
  MX_IWDG_Init();
#endif

  /* Heater PWM runs from now on; the duty stays 0 until the first PI step. */
  if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  /* The sampling time base. Ticks only set a flag, the main loop does the SPI. */
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
    Error_Handler();
  }

  ImuHeater_Init(&htim10);
  (void)ImuTask_Init(&hspi1, &huart1);

  while (1) {
#if APP_ENABLE_IWDG
    HAL_IWDG_Refresh(&hiwdg);
#endif
    ImuTask_Poll();
    Led_Indicate(ImuTask_IsReady(), HAL_GetTick());
  }
}

static void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 6U;
  osc.PLL.PLLN = 168U;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 7U;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    Error_Handler();
  }

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Chip selects idle high. Both are plain GPIO, not SPI hardware NSS. */
  HAL_GPIO_WritePin(BMI088_CS_ACCEL_GPIO_PORT, BMI088_CS_ACCEL_PIN,
                    GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMI088_CS_GYRO_GPIO_PORT, BMI088_CS_GYRO_PIN,
                    GPIO_PIN_SET);

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Pin = BMI088_CS_ACCEL_PIN;
  HAL_GPIO_Init(BMI088_CS_ACCEL_GPIO_PORT, &gpio);
  gpio.Pin = BMI088_CS_GYRO_PIN;
  HAL_GPIO_Init(BMI088_CS_GYRO_GPIO_PORT, &gpio);

  /* Status LED, off at boot (output high would light it, so drive low). */
  HAL_GPIO_WritePin(LED_R_GPIO_PORT, LED_R_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_PIN, GPIO_PIN_RESET);

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = LED_R_PIN;
  HAL_GPIO_Init(LED_R_GPIO_PORT, &gpio);
  gpio.Pin = LED_G_PIN;
  HAL_GPIO_Init(LED_G_GPIO_PORT, &gpio);
  gpio.Pin = LED_B_PIN;
  HAL_GPIO_Init(LED_B_GPIO_PORT, &gpio);
}

static void MX_SPI1_Init(void) {
  /*
   * SPI1 is on APB2 (84 MHz). Prescaler 16 gives 5.25 MHz, comfortably below
   * the BMI088 limit of 10 MHz and safe on the board's short traces.
   *
   * The BMI088 auto-detects SPI mode 0 or 3 from SCK after the falling edge of
   * the chip select; mode 0 is used here.
   */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10U;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = APP_UART_BAUD_RATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef clock_source = {0};
  TIM_MasterConfigTypeDef master = {0};

  /*
   * TIM3 sits on APB1. PCLK1 = HCLK/4 = 42 MHz and, because the APB1 divider
   * is not 1, the timer counter clock is twice PCLK1: 84 MHz. A prescaler of
   * 8399 therefore gives a 10 kHz tick and the period finishes the division
   * down to IMU_TASK_RATE_HZ.
   *
   * (APB2 is HCLK/2 = 84 MHz, so TIM10 - also an APB2 timer - really does run
   *  at 168 MHz. The two timers are not on the same clock.)
   */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = (10000U / IMU_TASK_RATE_HZ) - 1U;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  clock_source.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &clock_source) != HAL_OK) {
    Error_Handler();
  }

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM10_Init(void) {
  TIM_OC_InitTypeDef pwm = {0};

  /*
   * TIM10 drives the BMI088 heating resistor on PF6 (TIM10_CH1).
   *
   * APB2 is HCLK/2, so the counter clock is 168 MHz. Prescaler 167 gives
   * 1 MHz and a period of 9999 gives a 100 Hz PWM with 10000 duty steps.
   * The heater is deliberately switched slowly: its thermal time constant is
   * seconds, so a low frequency keeps switching noise out of the IMU.
   */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 167U;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9999U;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) {
    Error_Handler();
  }

  pwm.OCMode = TIM_OCMODE_PWM1;
  pwm.Pulse = 0U;
  pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &pwm, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim10);
}

#if APP_ENABLE_IWDG
static void MX_IWDG_Init(void) {
  /*
   * LSI is nominally 32 kHz: prescaler 64 gives a 500 Hz down-counter clock
   * and a reload of 2000 gives roughly a 4 s window. The watchdog is frozen
   * while the core is halted in a debug session.
   */
  __HAL_DBGMCU_FREEZE_IWDG();
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 2000U;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
    Error_Handler();
  }
}
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  JustFloat_OnTxComplete(huart);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    ImuTask_OnSampleTick();
  }
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

static void Led_Write(bool red, bool green, bool blue) {
  HAL_GPIO_WritePin(LED_R_GPIO_PORT, LED_R_PIN,
                    red ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_PIN,
                    green ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_PIN,
                    blue ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * Red while the IMU does not answer, green while streaming, and a 1 Hz blue
 * heartbeat that proves the main loop is still running.
 */
static void Led_Indicate(bool imu_ready, uint32_t tick_ms) {
  static uint32_t last_toggle_ms;
  static bool heartbeat;
  static int8_t previous_state = -1;

  if ((tick_ms - last_toggle_ms) >= 500U) {
    last_toggle_ms = tick_ms;
    heartbeat = !heartbeat;
  }

  int8_t const state = (int8_t)((imu_ready ? 0x2 : 0x1) |
                                ((imu_ready && heartbeat) ? 0x4 : 0x0));
  if (state != previous_state) {
    previous_state = state;
    Led_Write(!imu_ready, imu_ready, imu_ready && heartbeat);
  }
}

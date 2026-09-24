#include "main.h"

DMA_HandleTypeDef hdma_usart1_tx;

void HAL_MspInit(void) {
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY, 0U);
}

/* -------------------------------------------------------------------------- */
/* SPI1: BMI088 accelerometer + gyroscope                                     */
/* -------------------------------------------------------------------------- */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
  GPIO_InitTypeDef gpio = {0};

  if (hspi->Instance != SPI1) {
    return;
  }

  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*
   * SCK = PB3, MISO = PB4, MOSI = PA7 (all AF5).
   *
   * PB3 and PB4 are JTDO and NJTRST out of reset. On STM32F4 there is no
   * SWJ_CFG register: programming these pins to an alternate function other
   * than AF0 is what releases them from the JTAG port. SWD (PA13/PA14) is
   * left untouched, so the board stays debuggable.
   */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI1;

  gpio.Pin = GPIO_PIN_7; /* SPI1_MOSI */
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4; /* SPI1_SCK, SPI1_MISO */
  HAL_GPIO_Init(GPIOB, &gpio);
}

/* -------------------------------------------------------------------------- */
/* USART1: JustFloat stream to the host (J30 pin 2)                           */
/* -------------------------------------------------------------------------- */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef gpio = {0};

  if (huart->Instance != USART1) {
    return;
  }

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_9; /* USART1_TX */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* USART1_TX is DMA2 Stream 7, channel 4 on STM32F4. */
  hdma_usart1_tx.Instance = DMA2_Stream7;
  hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
    Error_Handler();
  }
  __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* -------------------------------------------------------------------------- */
/* Timers                                                                     */
/* -------------------------------------------------------------------------- */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base) {
  if (htim_base->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Sampling ticks are the highest priority job in the system. */
    HAL_NVIC_SetPriority(TIM3_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim_pwm) {
  if (htim_pwm->Instance == TIM10) {
    __HAL_RCC_TIM10_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim) {
  GPIO_InitTypeDef gpio = {0};

  if (htim->Instance == TIM10) {
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Heat_Power: TIM10_CH1 on PF6 drives the BMI088 heating resistor. */
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF3_TIM10;
    HAL_GPIO_Init(GPIOF, &gpio);
  }
}

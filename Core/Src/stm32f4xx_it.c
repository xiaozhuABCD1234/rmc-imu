#include "main.h"

/*
 * Only the vectors this firmware actually uses are defined here. Every other
 * vector keeps the weak infinite-loop stub from startup/startup_stm32f407xx.s.
 */

void SysTick_Handler(void) {
  HAL_IncTick();
}

/* IMU sampling time base (IMU_TASK_RATE_HZ, default 1 kHz). */
void TIM3_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim3);
}

/* USART1 TX DMA (JustFloat frames). */
void DMA2_Stream7_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void USART1_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart1);
}

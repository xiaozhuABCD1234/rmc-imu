#include "main.h"

#include "bmi088.h"

/*
 * Only the vectors this firmware actually uses are defined here. Every other
 * vector keeps the weak infinite-loop stub from startup/startup_stm32f407xx.s.
 */

void SysTick_Handler(void) {
  HAL_IncTick();
}

/*
 * Gyroscope data-ready. TIM5 overflow runs at priority 0 so the high word of
 * the 64-bit timestamp is updated before this handler reads it.
 */
void EXTI9_5_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(BMI088_INT_GYRO_PIN);
}

/* 84 MHz timestamp counter overflow (once every ~51 s). */
void TIM5_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim5);
}

void TIM8_UP_TIM13_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim8);
}

/* USART1 TX DMA (IMU stream frames). */
void DMA2_Stream7_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void USART1_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart1);
}

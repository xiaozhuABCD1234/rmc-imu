#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

/* Handles referenced outside main.c. hspi1, htim10 and hiwdg stay file-local
 * to main.c and therefore need no declaration here. */
extern UART_HandleTypeDef huart1;    /* stm32f4xx_it.c: USART1_IRQHandler */
extern TIM_HandleTypeDef htim5;      /* stm32f4xx_it.c: TIM5_IRQHandler */
extern DMA_HandleTypeDef hdma_usart1_tx; /* stm32f4xx_it.c: DMA2_Stream7 */

/* ------------------------------------------------------------------------- */
/* Application configuration                                                 */
/* ------------------------------------------------------------------------- */

/*
 * Host link: USART1 on the 4-pin J30 connector (pin 2 = TXD = PA9, pin 3 =
 * GND).
 *
 * 2 Mbaud 8N1 carries the binary IMU stream (IMU_STREAM_FRAME_BYTES = 31 bytes
 * per frame) at the gyroscope's own output data rate, around 850 Hz: 26 kB/s,
 * roughly 13 % of the line, so there is plenty of margin against USB scheduling
 * jitter. The rate is exact: APB2 is 84 MHz and 84e6 / 2e6 = 42, so the baud
 * divisor has zero error.
 *
 * Bandwidth ceiling for this frame format: baud / 10 / IMU_STREAM_FRAME_BYTES
 * frames per second, i.e. about 6400 frames/s at 2 Mbaud.
 */
#define APP_UART_BAUD_RATE 2000000U

/*
 * Independent watchdog, fed from the main loop. Set to 0 when the core must
 * never be reset automatically, for example during long debug sessions.
 */
#define APP_ENABLE_IWDG 1U

/* Status LED (common anode RGB, driven through N-MOSFETs: high = on). */
#define LED_R_GPIO_PORT GPIOH
#define LED_R_PIN GPIO_PIN_12
#define LED_G_GPIO_PORT GPIOH
#define LED_G_PIN GPIO_PIN_11
#define LED_B_GPIO_PORT GPIOH
#define LED_B_PIN GPIO_PIN_10

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);

#endif /* MAIN_H */

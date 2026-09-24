#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * Application task: samples the BMI088 at a fixed rate and streams the result
 * to the host as JustFloat frames over UART.
 *
 * The sampling instants come from a timer update interrupt so the frame rate
 * does not depend on how long the main loop takes; all SPI traffic, however,
 * is done from the main loop, never from an interrupt.
 *
 * JustFloat channel map (fixed order, see JUSTFLOAT_CHANNEL_COUNT):
 *   0  gyro X        [deg/s]
 *   1  gyro Y        [deg/s]
 *   2  gyro Z        [deg/s]
 *   3  accel X       [m/s^2]
 *   4  accel Y       [m/s^2]
 *   5  accel Z       [m/s^2]
 *   6  temperature   [degC]
 *   7  uptime        [s]     (also the frame-loss indicator: it must step by
 *                             exactly 1/IMU_TASK_RATE_HZ in every frame)
 *   8  heater duty   [0..1]
 */

/*
 * Sampling rate, driven by the TIM3 update interrupt. 1 kHz keeps the gyroscope
 * on its matching 1000 Hz ODR / 116 Hz filter; the sensor allows 2000 Hz but
 * that roughly doubles the gyro noise for 0.7 ms less latency.
 */
#define IMU_TASK_RATE_HZ 1000U

/* Successful samples between two heater control updates (10 Hz). */
#define IMU_TASK_HEATER_DIVIDER 100U

/* How often a failed IMU initialisation is retried. */
#define IMU_TASK_RETRY_MS 1000U

/*
 * Consecutive failed reads after which the IMU is re-initialised. One second is
 * long enough to ride out a single glitch and short enough that the heater,
 * which is driven from the same readings, is never left running blind.
 */
#define IMU_TASK_REINIT_AFTER_FAILURES IMU_TASK_RATE_HZ

/* huart must already be initialised for 8N1 TX with a linked TX DMA channel. */
bool ImuTask_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);

/* True once the IMU answered and was configured. Drives the status LED. */
bool ImuTask_IsReady(void);

/* Call from the timer update callback. Sets the sampling flag only. */
void ImuTask_OnSampleTick(void);

/* Main loop: reads the IMU, builds a frame and feeds the heater controller. */
void ImuTask_Poll(void);

#endif /* IMU_TASK_H */

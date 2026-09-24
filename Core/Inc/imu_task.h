#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * Sampling task: reads the BMI088 once per gyroscope data-ready interrupt and
 * hands one frame to ImuStream.
 *
 * Why the data-ready pin is the sample clock
 * -----------------------------------------
 * The BMI088 converts on its own internal time base, which is independent of
 * and slower than any timer on the MCU (measured: ~850 Hz at the nominal
 * 1000 Hz GYRO_BANDWIDTH setting). Polling from a TIM3 tick therefore
 * misrepresents the data twice: about 15 % of the reads returned a repeated
 * gyroscope sample, and the frame timestamp was the polling tick, which can be
 * up to a whole sensor period away from the instant the sensor actually
 * converted.
 *
 * Driving the read from the gyroscope data-ready pin (PC5) fixes both: every
 * frame carries exactly one fresh conversion, and the timestamp is taken from a
 * free-running 84 MHz counter at the moment the edge is serviced.
 *
 * Timestamps
 * ----------
 * TIM5 runs free at 84 MHz (APB1 timer clock). Its 32-bit counter is extended
 * to 64 bits by the update interrupt, so there is no wrap to worry about, and
 * DRDY edges are timestamped with the counter read inside the interrupt. The
 * resolution is one 84 MHz tick, about 12 ns; the residual error is the
 * interrupt entry latency, which is a few tens of nanoseconds because this
 * interrupt is given the highest priority.
 *
 * Frame rate
 * ----------
 * One frame per gyroscope conversion, i.e. the sensor's own ODR. Channels are
 * no longer sampled on a fixed grid, so the host must derive the nominal period
 * from the stream rather than being told a rate.
 */

/* How often the heater controller is updated. */
#define IMU_TASK_HEATER_PERIOD_MS 100U

/* How often a failed IMU initialisation is retried. */
#define IMU_TASK_RETRY_MS 1000U

/*
 * Consecutive failed reads after which the IMU is re-initialised. One second is
 * long enough to ride out a single glitch and short enough that the heater,
 * which is driven from the same readings, is never left running blind.
 */
#define IMU_TASK_REINIT_AFTER_FAILURES 100U

/* huart must already be initialised for 8N1 TX with a linked TX DMA channel. */
bool ImuTask_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);

/* True once the IMU answered and was configured. Drives the status LED. */
bool ImuTask_IsReady(void);

/* Call from the gyroscope data-ready EXTI handler: timestamps the edge and
 * marks a sample as pending. Highest priority interrupt in the system. */
void ImuTask_OnGyroDataReady(void);

/* Call from the 84 MHz timestamp timer update interrupt. */
void ImuTask_OnTimestampOverflow(void);

/* Current value of the 64-bit timestamp counter, in nanoseconds. */
uint64_t ImuTask_NowNs(void);

/* Main loop: reads the IMU on a pending data-ready edge and feeds the heater. */
void ImuTask_Poll(void);

#endif /* IMU_TASK_H */

#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * Bosch BMI088 6-axis IMU driver for the RoboMaster Development Board Type C.
 *
 * On that board the accelerometer and the gyroscope are two independent SPI
 * slaves that share SPI1 and use separate chip selects. The sensor's INT pins
 * are not used by this polled driver.
 *
 * Two BMI088 quirks are handled here:
 *   1. The accelerometer always powers up in I2C mode; a rising edge on its
 *      chip select switches it to SPI, and the first read afterwards returns
 *      an invalid value (datasheet section 6.1).
 *   2. Accelerometer register reads return one unpredictable dummy byte before
 *      the real data. Gyroscope reads do not.
 *
 * Ranges, output data rates and the resulting conversion factors all come from
 * the register constants at the top of bmi088.c; there is no second copy of
 * them that could drift out of sync.
 */

/* ------------------------------------------------------------------------- */
/* Board wiring (RoboMaster Type C)                                          */
/* ------------------------------------------------------------------------- */
#define BMI088_CS_ACCEL_GPIO_PORT GPIOA
#define BMI088_CS_ACCEL_PIN GPIO_PIN_4 /* CS1_Accel */
#define BMI088_CS_GYRO_GPIO_PORT GPIOB
#define BMI088_CS_GYRO_PIN GPIO_PIN_0 /* CS1_Gyro */

typedef struct {
  float gyro_dps[3];   /* [0] = X, [1] = Y, [2] = Z in deg/s */
  float accel_ms2[3];  /* [0] = X, [1] = Y, [2] = Z in m/s^2 */
  float temperature_c; /* IMU die temperature in degC */
} Bmi088Sample;

/*
 * hspi must be SPI1 in master mode, 8 bit, CPOL = 0 / CPHA = 0, at most
 * 10 MHz. Returns false if either die does not answer with its chip id or if a
 * configuration write fails.
 */
bool Bmi088_Init(SPI_HandleTypeDef *hspi);

/* Reads the gyroscope, the accelerometer and the temperature sensor. */
bool Bmi088_Read(Bmi088Sample *sample);

#endif /* BMI088_H */

#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * Bosch BMI088 6-axis IMU driver for the RoboMaster Development Board Type C.
 *
 * On that board the accelerometer and the gyroscope are two independent SPI
 * slaves that share SPI1 and use separate chip selects.
 *
 * This driver deliberately returns *raw* 16-bit sensor values and does no unit
 * conversion at all: the measurement path stays integer end to end, and the
 * host applies the scale factors (the configured ranges are reported alongside
 * every sample). Only the heater needs a temperature in degC, so the single
 * conversion helper below is the only place that produces a float.
 *
 * Wired signals used here (RoboMaster Type C user manual):
 *   SPI1 SCK/MISO/MOSI  PB3/PB4/PA7      CS1_Accel / CS1_Gyro  PA4 / PB0
 *   INT1_Accel          PC4              INT1_Gyro             PC5
 *
 * Two BMI088 quirks are handled here:
 *   1. The accelerometer always powers up in I2C mode; a rising edge on its
 *      chip select switches it to SPI, and the first read afterwards returns
 *      an invalid value (datasheet section 6.1).
 *   2. Accelerometer register reads return one unpredictable dummy byte before
 *      the real data. Gyroscope reads do not.
 */

#define BMI088_CS_ACCEL_GPIO_PORT GPIOA
#define BMI088_CS_ACCEL_PIN GPIO_PIN_4 /* CS1_Accel */
#define BMI088_CS_GYRO_GPIO_PORT GPIOB
#define BMI088_CS_GYRO_PIN GPIO_PIN_0 /* CS1_Gyro */

/*
 * Gyroscope data-ready. The board routes the gyro interrupt output to PC5 and
 * the accelerometer one to PC4; only the gyroscope is used, as the sample clock
 * for the whole application.
 */
#define BMI088_INT_GPIO_PORT GPIOC
#define BMI088_INT_GYRO_PIN GPIO_PIN_5

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */

/*
 * Full-scale ranges. The accelerometer supports +-3/6/12/24 g and gyroscope
 * +-2000/1000/500/250/125 dps; the raw register values are reported to the host
 * in every frame so it can derive the scale factor without a second copy of
 * this configuration.
 */
#define BMI088_ACC_RANGE_VALUE 0x01U  /* +-6 g   -> 5460 deg LSB/g */
#define BMI088_GYRO_RANGE_VALUE 0x00U /* +-2000 dps -> 16.384 LSB/dps */

/* ------------------------------------------------------------------------- */
/* Sample                                                                    */
/* ------------------------------------------------------------------------- */
typedef struct {
  int16_t gyro_raw[3];  /* [0] = X, [1] = Y, [2] = Z, sensor LSB */
  int16_t accel_raw[3]; /* [0] = X, [1] = Y, [2] = Z, sensor LSB */
  int16_t temperature_code; /* 11-bit two's complement, 0.125 degC/LSB */
} Bmi088Sample;

/*
 * hspi must be SPI1 in master mode, 8 bit, CPOL = 0 / CPHA = 0, at most
 * 10 MHz. Returns false if either die does not answer with its chip id or if a
 * configuration write fails.
 */
bool Bmi088_Init(SPI_HandleTypeDef *hspi);

/* Reads the gyroscope, the accelerometer and the temperature sensor. */
bool Bmi088_Read(Bmi088Sample *sample);

/* Datasheet formula: temperature = code * 0.125 degC + 23 degC. */
float Bmi088_TemperatureC(int16_t temperature_code);

#endif /* BMI088_H */

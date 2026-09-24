#ifndef IMU_HEATER_H
#define IMU_HEATER_H

#include <stdbool.h>

#include "stm32f4xx_hal.h"

/*
 * The RoboMaster Type C board puts a heating resistor next to the BMI088 and
 * drives it from TIM10_CH1 (PF6). At 100 % duty it dissipates about 0.58 W and
 * raises the board roughly 15-20 degC above ambient, which is exactly what DJI
 * recommends in order to reduce the gyroscope zero-rate temperature drift.
 *
 * A small PI controller holds the IMU at IMU_HEATER_TARGET_C.
 */

/* Set 0 to build a firmware without temperature regulation. */
#define IMU_HEATER_ENABLE 1U

/* Target die temperature. 40 degC is a safe value for indoor use. */
#define IMU_HEATER_TARGET_C 40.0f

/* Upper duty limit; keeps the resistor and the board below their ratings. */
#define IMU_HEATER_MAX_DUTY 0.85f

/*
 * htim must be TIM10 configured for PWM on channel 1 at about 100 Hz. The
 * heater is switched slowly on purpose: its thermal time constant is seconds,
 * so a low frequency keeps the switching noise away from the IMU.
 */
void ImuHeater_Init(TIM_HandleTypeDef *htim);

/*
 * Runs one PI step. Call at a fixed, relatively slow rate (the module is
 * designed for dt_s = 0.1 s) with the latest IMU temperature.
 */
void ImuHeater_Control(float temperature_c, float dt_s);

/* Forces the PWM off; used when the IMU temperature becomes stale. */
void ImuHeater_Disable(void);

float ImuHeater_GetDuty(void);

#endif /* IMU_HEATER_H */

#ifndef CAMERA_TRIGGER_H
#define CAMERA_TRIGGER_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define CAMERA_TRIGGER_TIMER TIM8
#define CAMERA_TRIGGER_CHANNEL TIM_CHANNEL_1
#define CAMERA_TRIGGER_TIMER_CLOCK_HZ 100000U
#define CAMERA_TRIGGER_FREQUENCY_HZ 5U
#define CAMERA_TRIGGER_PULSE_US 100U

void CameraTrigger_Init(TIM_HandleTypeDef *htim);
void CameraTrigger_OnPeriodElapsed(TIM_HandleTypeDef *htim);
void CameraTrigger_Poll(void);
uint32_t CameraTrigger_GetSequence(void);
uint32_t CameraTrigger_GetDroppedEvents(void);

#endif /* CAMERA_TRIGGER_H */

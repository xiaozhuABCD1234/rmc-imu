#include "imu_heater.h"

#if IMU_HEATER_ENABLE

/* Duty per degC of error. */
#define IMU_HEATER_KP 0.05f
/* Duty per (degC * second) of accumulated error. */
#define IMU_HEATER_KI 0.004f

static TIM_HandleTypeDef *s_tim;
static float s_target_c = IMU_HEATER_TARGET_C;
static float s_integral;
static float s_duty;
static uint16_t s_period = 1U;

static void ImuHeater_ApplyDuty(float duty) {
  uint32_t compare;

  if (s_tim == NULL) {
    return;
  }
  if (duty < 0.0f) {
    duty = 0.0f;
  }
  if (duty > IMU_HEATER_MAX_DUTY) {
    duty = IMU_HEATER_MAX_DUTY;
  }

  compare = (uint32_t)((duty * (float)s_period) + 0.5f);
  __HAL_TIM_SET_COMPARE(s_tim, TIM_CHANNEL_1, compare);
  s_duty = duty;
}

void ImuHeater_Init(TIM_HandleTypeDef *htim) {
  s_tim = htim;
  s_target_c = IMU_HEATER_TARGET_C;
  s_integral = 0.0f;
  s_duty = 0.0f;

  if (htim != NULL) {
    s_period = (uint16_t)(__HAL_TIM_GET_AUTORELOAD(htim) + 1U);
  }
  ImuHeater_ApplyDuty(0.0f);
}

void ImuHeater_Control(float temperature_c, float dt_s) {
  float error;
  float duty;

  if ((s_tim == NULL) || (dt_s <= 0.0f)) {
    return;
  }

  error = s_target_c - temperature_c;
  s_integral += error * dt_s;

  duty = (IMU_HEATER_KP * error) + (IMU_HEATER_KI * s_integral);

  /* The heater cannot cool, so clamp at zero and undo the wind-up. */
  if (duty < 0.0f) {
    s_integral -= error * dt_s;
    duty = 0.0f;
  } else if (duty > IMU_HEATER_MAX_DUTY) {
    s_integral -= error * dt_s;
    duty = IMU_HEATER_MAX_DUTY;
  }

  ImuHeater_ApplyDuty(duty);
}

void ImuHeater_Disable(void) {
  s_integral = 0.0f;
  ImuHeater_ApplyDuty(0.0f);
}

float ImuHeater_GetDuty(void) {
  return s_duty;
}

#else /* IMU_HEATER_ENABLE */

void ImuHeater_Init(TIM_HandleTypeDef *htim) {
  (void)htim;
}

void ImuHeater_Control(float temperature_c, float dt_s) {
  (void)temperature_c;
  (void)dt_s;
}

void ImuHeater_Disable(void) {
}

float ImuHeater_GetDuty(void) {
  return 0.0f;
}

#endif /* IMU_HEATER_ENABLE */

#include "imu_task.h"

#include "bmi088.h"
#include "imu_heater.h"
#include "justfloat.h"

/* The channel map documented in imu_task.h must match the frame layout. */
#if (JUSTFLOAT_CHANNEL_COUNT != 9U)
#error "imu_task.c builds exactly 9 JustFloat channels; update both files"
#endif

static SPI_HandleTypeDef *s_spi;
static Bmi088Sample s_sample;
static float s_frame[JUSTFLOAT_CHANNEL_COUNT];

static volatile uint32_t s_ticks; /* advanced by the sampling ISR */
static uint32_t s_ticks_consumed; /* ticks that have produced a frame */
static uint32_t s_heater_tick;    /* tick of the last heater update */
static uint32_t s_last_init_ms;
static uint32_t s_read_failures;
static bool s_ready;

static void ImuTask_FillFrame(void) {
  s_frame[0] = s_sample.gyro_dps[0];
  s_frame[1] = s_sample.gyro_dps[1];
  s_frame[2] = s_sample.gyro_dps[2];
  s_frame[3] = s_sample.accel_ms2[0];
  s_frame[4] = s_sample.accel_ms2[1];
  s_frame[5] = s_sample.accel_ms2[2];
  s_frame[6] = s_sample.temperature_c;
  s_frame[7] = (float)s_ticks_consumed / (float)IMU_TASK_RATE_HZ;
  s_frame[8] = ImuHeater_GetDuty();
}

bool ImuTask_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart) {
  if ((hspi == NULL) || (huart == NULL)) {
    return false;
  }

  s_spi = hspi;
  JustFloat_Init(huart);

  s_ticks = 0U;
  s_ticks_consumed = 0U;
  s_heater_tick = 0U;
  s_read_failures = 0U;
  s_last_init_ms = HAL_GetTick();

  s_ready = Bmi088_Init(s_spi);
  return s_ready;
}

bool ImuTask_IsReady(void) {
  return s_ready;
}

void ImuTask_OnSampleTick(void) {
  ++s_ticks;
}

void ImuTask_Poll(void) {
  uint32_t ticks;

  if (!s_ready) {
    /*
     * The IMU is missing, still starting up, or has stopped answering. Keep the
     * watchdog fed and the queue drained instead of locking up, and retry once
     * a second so the board recovers on its own.
     */
    uint32_t const now = HAL_GetTick();
    if ((now - s_last_init_ms) >= IMU_TASK_RETRY_MS) {
      s_last_init_ms = now;
      s_ready = Bmi088_Init(s_spi);
      if (s_ready) {
        /* Drop whatever accumulated while the interface was being set up. */
        s_ticks_consumed = 0U;
        s_ticks = 0U;
        s_heater_tick = 0U;
        s_read_failures = 0U;
      }
    }
    JustFloat_Poll();
    return;
  }

  ticks = s_ticks;
  if (ticks == s_ticks_consumed) {
    JustFloat_Poll();
    return;
  }
  /*
   * One SPI read produces one sample, so if several periods elapsed only the
   * newest sample is kept. The uptime channel steps by the real elapsed time,
   * which is how the host notices the gap.
   */
  s_ticks_consumed = ticks;

  if (!Bmi088_Read(&s_sample)) {
    if (++s_read_failures >= IMU_TASK_REINIT_AFTER_FAILURES) {
      /*
       * No temperature feedback while the reads fail, so the heater must not
       * keep its last duty, and the IMU is probably wedged: hand it back to the
       * initialisation retry path above.
       */
      ImuHeater_Disable();
      s_read_failures = 0U;
      s_ready = false;
    }
    JustFloat_Poll();
    return;
  }
  s_read_failures = 0U;

  ImuTask_FillFrame();
  (void)JustFloat_Send(s_frame);

  /*
   * Feed the heater from the real elapsed tick count rather than a fixed
   * divisor, so a skipped sample cannot stall the controller or bias its dt.
   */
  if ((s_ticks_consumed - s_heater_tick) >= IMU_TASK_HEATER_DIVIDER) {
    float const dt_s = (float)(s_ticks_consumed - s_heater_tick) /
                       (float)IMU_TASK_RATE_HZ;
    s_heater_tick = s_ticks_consumed;
    ImuHeater_Control(s_sample.temperature_c, dt_s);
  }

  JustFloat_Poll();
}

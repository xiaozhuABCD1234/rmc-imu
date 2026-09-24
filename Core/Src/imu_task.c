#include "imu_task.h"

#include <string.h>

#include "bmi088.h"
#include "imu_heater.h"
#include "imu_stream.h"

/* The timestamp counter and the ranges reported in every frame must match what
 * the driver actually configures. */
#if (BMI088_ACC_RANGE_VALUE > 3U)
#error "ACC_RANGE register value must be 0..3"
#endif

#if (BMI088_GYRO_RANGE_VALUE > 4U)
#error "GYRO_RANGE register value must be 0..4"
#endif

/* TIM5 runs from the APB1 timer clock. */
#define IMU_TASK_TIMESTAMP_HZ 84000000U

#define IMU_TASK_NS_PER_MS 1000000ULL
#define IMU_TASK_HEATER_PERIOD_NS \
  ((uint64_t)IMU_TASK_HEATER_PERIOD_MS * IMU_TASK_NS_PER_MS)

static SPI_HandleTypeDef *s_spi;

/* --- hardware timestamp ------------------------------------------------ */
/* s_timer_high is the upper half of the 64-bit counter; TIM5->CNT is the lower
 * half and is read at the instant of the data-ready edge. */
static volatile uint32_t s_timer_high;
static volatile uint64_t s_capture_ns;
static volatile uint8_t s_drdy_pending;
static volatile uint32_t s_drdy_overruns;

/* --- task state -------------------------------------------------------- */
static uint64_t s_heater_ns;
static uint8_t s_heater_started;
static uint32_t s_last_init_ms;
static uint32_t s_read_failures;
static uint8_t s_read_error_pending;
static bool s_ready;

/* ---------------------------------------------------------------------- */

static uint64_t ImuTask_ReadTicks(void) {
  uint32_t high_before;
  uint32_t high_after;
  uint32_t low;

  /* The counter can overflow between reading the two halves, so repeat the
   * read if the high word changed underneath us. */
  do {
    high_before = s_timer_high;
    low = TIM5->CNT;
    high_after = s_timer_high;
  } while (high_before != high_after);

  return ((uint64_t)high_after << 32) | (uint64_t)low;
}

static uint64_t ImuTask_TicksToNs(uint64_t ticks) {
  /*
   * 1 us is 84 ticks, so ns = ticks * 1000 / 84 (truncated to well under 1 ns).
   * The product stays inside 64 bits for the first ~6 years of uptime.
   */
  return (ticks * 1000ULL) / (uint64_t)(IMU_TASK_TIMESTAMP_HZ / 1000000U);
}

uint64_t ImuTask_NowNs(void) {
  return ImuTask_TicksToNs(ImuTask_ReadTicks());
}

void ImuTask_OnGyroDataReady(void) {
  if (s_drdy_pending != 0U) {
    /* The main loop had not consumed the previous sample yet. */
    ++s_drdy_overruns;
  }
  s_capture_ns = ImuTask_TicksToNs(ImuTask_ReadTicks());
  s_drdy_pending = 1U;
}

void ImuTask_OnTimestampOverflow(void) {
  ++s_timer_high;
}

bool ImuTask_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart) {
  if ((hspi == NULL) || (huart == NULL)) {
    return false;
  }

  s_spi = hspi;
  ImuStream_Init(huart);

  s_timer_high = 0U;
  s_capture_ns = 0U;
  s_drdy_pending = 0U;
  s_drdy_overruns = 0U;
  s_heater_ns = 0U;
  s_heater_started = 0U;
  s_read_error_pending = 0U;
  s_read_failures = 0U;
  s_last_init_ms = HAL_GetTick();

  s_ready = Bmi088_Init(s_spi);
  return s_ready;
}

bool ImuTask_IsReady(void) {
  return s_ready;
}

static void ImuTask_HandleReadFailure(void) {
  s_read_error_pending = 1U;

  if (++s_read_failures >= IMU_TASK_REINIT_AFTER_FAILURES) {
    /*
     * No temperature feedback while the reads fail, so the heater must not keep
     * its last duty, and the IMU is probably wedged: hand it back to the
     * initialisation retry path.
     */
    ImuHeater_Disable();
    s_read_failures = 0U;
    s_ready = false;
  }
}

void ImuTask_Poll(void) {
  Bmi088Sample raw;
  ImuStreamSample sample;
  uint64_t capture_ns;
  uint32_t overruns;

  if (!s_ready) {
    /*
     * The IMU is missing, still starting up, or has stopped answering. Keep the
     * watchdog fed and the queue drained instead of locking up, and retry once
     * a second so the board recovers on its own.
     */
    const uint32_t now = HAL_GetTick();
    if ((now - s_last_init_ms) >= IMU_TASK_RETRY_MS) {
      s_last_init_ms = now;
      s_ready = Bmi088_Init(s_spi);
      if (s_ready) {
        s_drdy_pending = 0U;
        s_drdy_overruns = 0U;
        s_read_failures = 0U;
        s_read_error_pending = 0U;
        s_heater_started = 0U;
      }
    }
    ImuStream_Poll();
    return;
  }

  if (s_drdy_pending == 0U) {
    ImuStream_Poll();
    return;
  }

  /* Claim the pending sample before touching SPI: a new edge arriving during
   * the read sets the flag again and counts as an overrun. */
  s_drdy_pending = 0U;
  capture_ns = s_capture_ns;
  overruns = s_drdy_overruns;
  s_drdy_overruns = 0U;

  if (!Bmi088_Read(&raw)) {
    ImuTask_HandleReadFailure();
    ImuStream_Poll();
    return;
  }
  s_read_failures = 0U;

  sample.timestamp_ns = capture_ns;
  memcpy(sample.gyro_raw, raw.gyro_raw, sizeof(sample.gyro_raw));
  memcpy(sample.accel_raw, raw.accel_raw, sizeof(sample.accel_raw));
  sample.temperature_code = raw.temperature_code;
  sample.gyro_range = BMI088_GYRO_RANGE_VALUE;
  sample.accel_range = BMI088_ACC_RANGE_VALUE;
  sample.flags = 0U;
  if (overruns > 0U) {
    sample.flags |= IMU_STREAM_FLAG_SAMPLE_LOST;
  }
  if (s_read_error_pending != 0U) {
    sample.flags |= IMU_STREAM_FLAG_READ_ERROR;
    s_read_error_pending = 0U;
  }
  (void)ImuStream_Send(&sample);

  /* Heater: fixed control period, but dt from the real elapsed time so a
   * skipped sample cannot bias the integral. */
  if (s_heater_started == 0U) {
    s_heater_ns = capture_ns;
    s_heater_started = 1U;
  } else if ((capture_ns - s_heater_ns) >= IMU_TASK_HEATER_PERIOD_NS) {
    const float dt_s = (float)(capture_ns - s_heater_ns) * 1e-9f;
    s_heater_ns = capture_ns;
    ImuHeater_Control(Bmi088_TemperatureC(raw.temperature_code), dt_s);
  }

  ImuStream_Poll();
}

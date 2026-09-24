#include "bmi088.h"

/* ------------------------------------------------------------------------- */
/* Register addresses (BMI088 datasheet BST-BMI088-DS001)                    */
/* ------------------------------------------------------------------------- */
/* Accelerometer */
#define BMI088_ACC_CHIP_ID 0x00U
#define BMI088_ACC_CHIP_ID_VALUE 0x1EU
#define BMI088_ACC_DATA 0x12U
#define BMI088_TEMP_MSB 0x22U
#define BMI088_ACC_CONF 0x40U
#define BMI088_ACC_RANGE 0x41U
#define BMI088_ACC_PWR_CONF 0x7CU
#define BMI088_ACC_PWR_CTRL 0x7DU
#define BMI088_ACC_SOFTRESET 0x7EU

/* Gyroscope */
#define BMI088_GYRO_CHIP_ID 0x00U
#define BMI088_GYRO_CHIP_ID_VALUE 0x0FU
#define BMI088_GYRO_DATA 0x02U
#define BMI088_GYRO_RANGE 0x0FU
#define BMI088_GYRO_BANDWIDTH 0x10U
#define BMI088_GYRO_SOFTRESET 0x14U
#define BMI088_GYRO_INT_CTRL 0x15U
#define BMI088_INT3_INT4_IO_CONF 0x16U
#define BMI088_INT3_INT4_IO_MAP 0x18U

#define BMI088_SOFTRESET_COMMAND 0xB6U

/* ------------------------------------------------------------------------- */
/* Configuration. Changing a range here is enough: the conversion factors    */
/* below are derived from these values.                                      */
/* ------------------------------------------------------------------------- */

/*
 * ACC_CONF (0x40), see datasheet 5.3.10: acc_bwp occupies bits [7:4] and its
 * legal values are 0x08 (4x oversampling), 0x09 (2x) and 0x0A (normal); bit 7
 * must be 1, which 0x0A already contains. acc_odr is [3:0]: 0x0B is 800 Hz and
 * 0x0C is 1600 Hz.
 */
#define BMI088_ACC_BWP_NORMAL 0x0AU
#define BMI088_ACC_ODR_VALUE 0x0CU
#define BMI088_ACC_CONF_VALUE \
  (uint8_t)((BMI088_ACC_BWP_NORMAL << 4U) | BMI088_ACC_ODR_VALUE)

/*
 * GYRO_BANDWIDTH (0x10): 0x02 = 1000 Hz ODR / 116 Hz filter, matching the
 * recommended 1 kHz sampling rate. 0x01 = 2000 Hz / 230 Hz and 0x00 =
 * 2000 Hz / 532 Hz trade roughly double the noise for lower latency.
 */
#define BMI088_GYRO_BANDWIDTH_VALUE 0x02U

/* ACC_PWR_CTRL: 0x04 = accelerometer (and its temperature sensor) on. */
#define BMI088_ACC_PWR_CTRL_ON 0x04U
/* ACC_PWR_CONF: 0x00 = active mode, 0x03 = suspend. */
#define BMI088_ACC_PWR_CONF_ACTIVE 0x00U

/* Timing from the datasheet. */
#define BMI088_SPI_TIMEOUT_MS 20U
#define BMI088_WRITE_IDLE_US 2U     /* tIDLE_wacc between write accesses */
#define BMI088_GYRO_RESET_MS 30U    /* gyro soft reset recovery */
#define BMI088_ACCEL_BOOT_MS 1U     /* accel boot time after POR / soft reset */
#define BMI088_ACCEL_STARTUP_MS 50U /* accel power-up after ACC_PWR_CTRL = 4 */
#define BMI088_ACCEL_SETTLE_MS 5U   /* settling after a power mode change */
#define BMI088_CS_PULSE_MS 1U       /* CSB1 low pulse for the SPI mode switch */

#define BMI088_CS_ACCEL_LOW()                                        \
  HAL_GPIO_WritePin(BMI088_CS_ACCEL_GPIO_PORT, BMI088_CS_ACCEL_PIN,  \
                    GPIO_PIN_RESET)
#define BMI088_CS_ACCEL_HIGH()                                       \
  HAL_GPIO_WritePin(BMI088_CS_ACCEL_GPIO_PORT, BMI088_CS_ACCEL_PIN,  \
                    GPIO_PIN_SET)
#define BMI088_CS_GYRO_LOW()                                         \
  HAL_GPIO_WritePin(BMI088_CS_GYRO_GPIO_PORT, BMI088_CS_GYRO_PIN,    \
                    GPIO_PIN_RESET)
#define BMI088_CS_GYRO_HIGH()                                        \
  HAL_GPIO_WritePin(BMI088_CS_GYRO_GPIO_PORT, BMI088_CS_GYRO_PIN,    \
                    GPIO_PIN_SET)

/* ------------------------------------------------------------------------- */
/* Module state                                                              */
/* ------------------------------------------------------------------------- */
static SPI_HandleTypeDef *s_spi;

/* ------------------------------------------------------------------------- */
/* Bit-level helpers                                                         */
/* ------------------------------------------------------------------------- */

/*
 * The DWT cycle counter gives a microsecond delay that also works from an
 * interrupt context, unlike HAL_Delay() which needs the SysTick interrupt.
 */
static void Bmi088_EnableCycleCounter(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Bmi088_DelayUs(uint32_t microseconds) {
  uint32_t const start = DWT->CYCCNT;
  uint32_t const wait = microseconds * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start) < wait) {
  }
}

/*
 * One chip select period. HAL_SPI_TransmitReceive() is used for writes too:
 * on STM32F4 the transmit-only path of the HAL leaves the received data
 * unread, which can trip the overrun flag on the next transfer.
 */
static bool Bmi088_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t length) {
  return HAL_SPI_TransmitReceive(s_spi, (uint8_t *)tx, rx, length,
                                 BMI088_SPI_TIMEOUT_MS) == HAL_OK;
}

/* ------------------------------------------------------------------------- */
/* Register access                                                           */
/* ------------------------------------------------------------------------- */

/* Gyroscope: address byte, then data bytes. */
static bool Bmi088_GyroRead(uint8_t reg, uint8_t *data, uint16_t length) {
  uint8_t tx[1U + 8U] = {0};
  uint8_t rx[1U + 8U] = {0};
  uint16_t i;

  if (length > 8U) {
    return false;
  }
  tx[0] = (uint8_t)(reg | 0x80U); /* bit 7 = 1 selects a read */

  BMI088_CS_GYRO_LOW();
  bool const ok = Bmi088_Transfer(tx, rx, (uint16_t)(length + 1U));
  BMI088_CS_GYRO_HIGH();

  if (!ok) {
    return false;
  }
  for (i = 0U; i < length; ++i) {
    data[i] = rx[i + 1U];
  }
  return true;
}

/* Accelerometer: address byte, one unpredictable dummy byte, then data. */
static bool Bmi088_AccelRead(uint8_t reg, uint8_t *data, uint16_t length) {
  uint8_t tx[2U + 8U] = {0};
  uint8_t rx[2U + 8U] = {0};
  uint16_t i;

  if (length > 8U) {
    return false;
  }
  tx[0] = (uint8_t)(reg | 0x80U);

  BMI088_CS_ACCEL_LOW();
  bool const ok = Bmi088_Transfer(tx, rx, (uint16_t)(length + 2U));
  BMI088_CS_ACCEL_HIGH();

  if (!ok) {
    return false;
  }
  for (i = 0U; i < length; ++i) {
    data[i] = rx[i + 2U];
  }
  return true;
}

static bool Bmi088_Write(uint8_t reg, uint8_t value, bool accel) {
  uint8_t tx[2];
  uint8_t rx[2];
  bool ok;

  tx[0] = (uint8_t)(reg & 0x7FU); /* bit 7 = 0 selects a write */
  tx[1] = value;

  if (accel) {
    BMI088_CS_ACCEL_LOW();
    ok = Bmi088_Transfer(tx, rx, 2U);
    BMI088_CS_ACCEL_HIGH();
  } else {
    BMI088_CS_GYRO_LOW();
    ok = Bmi088_Transfer(tx, rx, 2U);
    BMI088_CS_GYRO_HIGH();
  }
  Bmi088_DelayUs(BMI088_WRITE_IDLE_US);
  return ok;
}

static bool Bmi088_GyroWrite(uint8_t reg, uint8_t value) {
  return Bmi088_Write(reg, value, false);
}

static bool Bmi088_AccelWrite(uint8_t reg, uint8_t value) {
  return Bmi088_Write(reg, value, true);
}

/*
 * Bring the accelerometer die from its power-on I2C mode into SPI mode: pull
 * CSB1 low, release it, then do the dummy register read the datasheet
 * prescribes (whose result is documented to be invalid).
 */
static void Bmi088_AccelEnterSpiMode(void) {
  uint8_t discarded = 0U;

  BMI088_CS_ACCEL_LOW();
  HAL_Delay(BMI088_CS_PULSE_MS);
  BMI088_CS_ACCEL_HIGH();
  HAL_Delay(BMI088_CS_PULSE_MS);

  (void)Bmi088_AccelRead(BMI088_ACC_CHIP_ID, &discarded, 1U);
}

/*
 * True only when the accelerometer answered with its expected chip id. The
 * first read after a mode switch is invalid, and a soft reset can drop the die
 * back to I2C, so the read is retried with a fresh mode switch in between.
 */
static bool Bmi088_ProbeAccel(void) {
  uint8_t chip_id = 0U;
  uint8_t attempt;

  for (attempt = 0U; attempt < 3U; ++attempt) {
    if (!Bmi088_AccelRead(BMI088_ACC_CHIP_ID, &chip_id, 1U)) {
      return false;
    }
    if (chip_id == BMI088_ACC_CHIP_ID_VALUE) {
      return true;
    }
    Bmi088_AccelEnterSpiMode();
  }
  return false;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */
bool Bmi088_Init(SPI_HandleTypeDef *hspi) {
  uint8_t gyro_id = 0U;

  if (hspi == NULL) {
    return false;
  }
  s_spi = hspi;
  Bmi088_EnableCycleCounter();

  /* Park both chip selects high before the first access. */
  BMI088_CS_ACCEL_HIGH();
  BMI088_CS_GYRO_HIGH();
  HAL_Delay(BMI088_ACCEL_BOOT_MS);

  Bmi088_AccelEnterSpiMode();

  if (!Bmi088_GyroWrite(BMI088_GYRO_SOFTRESET, BMI088_SOFTRESET_COMMAND)) {
    return false;
  }
  if (!Bmi088_AccelWrite(BMI088_ACC_SOFTRESET, BMI088_SOFTRESET_COMMAND)) {
    return false;
  }
  /* 30 ms for the gyroscope, 1 ms for the accelerometer. */
  HAL_Delay(BMI088_GYRO_RESET_MS);

  /* A soft reset may drop the accelerometer back to I2C mode; re-enter SPI. */
  if (!Bmi088_ProbeAccel()) {
    return false;
  }
  if (!Bmi088_GyroRead(BMI088_GYRO_CHIP_ID, &gyro_id, 1U) ||
      (gyro_id != BMI088_GYRO_CHIP_ID_VALUE)) {
    return false;
  }

  /* Gyroscope: +-2000 dps, 1000 Hz ODR / 116 Hz filter. */
  if (!Bmi088_GyroWrite(BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_VALUE) ||
      !Bmi088_GyroWrite(BMI088_GYRO_BANDWIDTH, BMI088_GYRO_BANDWIDTH_VALUE)) {
    return false;
  }

  /*
   * Route the gyroscope data-ready to INT3 (the die's pin that the board wires
   * to PC5) as push-pull active high, and enable it: 0x15 data_en = 1,
   * 0x16 int3 = output / push-pull / active high, 0x18 int3_data = 1.
   */
  if (!Bmi088_GyroWrite(BMI088_INT3_INT4_IO_CONF, 0x00U) ||
      !Bmi088_GyroWrite(BMI088_INT3_INT4_IO_MAP, 0x01U) ||
      !Bmi088_GyroWrite(BMI088_GYRO_INT_CTRL, 0x80U)) {
    return false;
  }

  /* Accelerometer: switch on, wait for the start-up time, then go active. */
  if (!Bmi088_AccelWrite(BMI088_ACC_PWR_CTRL, BMI088_ACC_PWR_CTRL_ON)) {
    return false;
  }
  HAL_Delay(BMI088_ACCEL_STARTUP_MS);
  if (!Bmi088_AccelWrite(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_CONF_ACTIVE)) {
    return false;
  }
  HAL_Delay(BMI088_ACCEL_SETTLE_MS);

  if (!Bmi088_AccelWrite(BMI088_ACC_RANGE, BMI088_ACC_RANGE_VALUE) ||
      !Bmi088_AccelWrite(BMI088_ACC_CONF, BMI088_ACC_CONF_VALUE)) {
    return false;
  }
  HAL_Delay(BMI088_ACCEL_SETTLE_MS);

  return true;
}

bool Bmi088_Read(Bmi088Sample *sample) {
  uint8_t raw[6];
  uint8_t temp_raw[2];
  uint16_t temperature_code;

  if ((s_spi == NULL) || (sample == NULL)) {
    return false;
  }

  if (!Bmi088_GyroRead(BMI088_GYRO_DATA, raw, 6U)) {
    return false;
  }
  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    sample->gyro_raw[axis] =
        (int16_t)(((uint16_t)raw[axis * 2U + 1U] << 8) | raw[axis * 2U]);
  }

  if (!Bmi088_AccelRead(BMI088_ACC_DATA, raw, 6U)) {
    return false;
  }
  for (uint8_t axis = 0U; axis < 3U; ++axis) {
    sample->accel_raw[axis] =
        (int16_t)(((uint16_t)raw[axis * 2U + 1U] << 8) | raw[axis * 2U]);
  }

  if (!Bmi088_AccelRead(BMI088_TEMP_MSB, temp_raw, 2U)) {
    return false;
  }
  /*
   * 11-bit two's complement, 0.125 degC/LSB, 23 degC at code 0:
   * Temp_uint11 = TEMP_MSB * 8 + TEMP_LSB / 32.
   */
  temperature_code =
      (uint16_t)(((uint16_t)temp_raw[0] * 8U) + (temp_raw[1] / 32U));
  sample->temperature_code = (temperature_code > 1023U)
                                 ? (int16_t)(temperature_code - 2048U)
                                 : (int16_t)temperature_code;

  return true;
}

float Bmi088_TemperatureC(int16_t temperature_code) {
  return ((float)temperature_code * 0.125f) + 23.0f;
}

#include "drv_spi_mpu6xxx.h"

#include <stdio.h>

#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_spi_gyro.h"
#include "drv_time.h"
#include "project.h"

#define MPU6000_ID (0x68)
#define MPU6500_ID (0x70)

#define ICM20601_ID (0xAC)
#define ICM20602_ID (0x12)
#define ICM20608_ID (0xAF)
#define ICM20649_ID (0xE1)
#define ICM20689_ID (0x98)

#define SPI_SPEED_INIT MHZ_TO_HZ(0.5)

extern spi_bus_device_t gyro_bus;

<<<<<<< HEAD
static uint32_t mpu6xxx_fast_divider() {
  switch (gyro_type) {
  default:
  case GYRO_TYPE_ICM20649:
    return MHZ_TO_HZ(7);
=======
static void mpu6xxx_reinit_slow() {
  spi_dma_wait_for_ready(GYRO_SPI_PORT);
  LL_SPI_Disable(PORT.channel);

  // SPI Config
  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef spi_init;
  spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi_init.Mode = LL_SPI_MODE_MASTER;
  spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  spi_init.NSS = LL_SPI_NSS_SOFT;
  spi_init.BaudRate = spi_find_divder(SPI_SPEED_INIT);
  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);
}

static void mpu6xxx_reinit_fast() {
  spi_dma_wait_for_ready(GYRO_SPI_PORT);
  LL_SPI_Disable(PORT.channel);

  // SPI Config
  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef spi_init;
  spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi_init.Mode = LL_SPI_MODE_MASTER;
  spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  spi_init.NSS = LL_SPI_NSS_SOFT;

  switch (gyro_type) {
  default:
  case GYRO_TYPE_ICM20649:
    spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(7));
    break;
>>>>>>> 53276eff (auto detect gyro type)

  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
<<<<<<< HEAD
    return MHZ_TO_HZ(8);

  case GYRO_TYPE_ICM20602:
    return MHZ_TO_HZ(10.5);

  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
    return MHZ_TO_HZ(21);
=======
    spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(8));
    break;

  case GYRO_TYPE_ICM20602:
    spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(10.5));
    break;

  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
    spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(21));
    break;
>>>>>>> 53276eff (auto detect gyro type)
  }

  return SPI_SPEED_INIT;
}

uint8_t mpu6xxx_detect() {
  time_delay_ms(100);

  const uint8_t id = mpu6xxx_read(MPU_RA_WHO_AM_I);
  switch (id) {
  case MPU6000_ID:
    return GYRO_TYPE_MPU6000;
  case MPU6500_ID:
    return GYRO_TYPE_MPU6500;
  case ICM20601_ID:
    return GYRO_TYPE_ICM20601;
  case ICM20602_ID:
    return GYRO_TYPE_ICM20602;
  case ICM20608_ID:
    return GYRO_TYPE_ICM20608;
  case ICM20649_ID:
    return GYRO_TYPE_ICM20649;
  case ICM20689_ID:
    return GYRO_TYPE_ICM20689;
  default:
    return GYRO_TYPE_INVALID;
  }
}

<<<<<<< HEAD
void mpu6xxx_configure() {
=======
uint8_t mpu6xxx_detect() {
  mpu6xxx_init();

  const uint8_t id = mpu6xxx_read(MPU_RA_WHO_AM_I);
  switch (id) {
  case MPU6000_ID:
    return GYRO_TYPE_MPU6000;
  case MPU6500_ID:
    return GYRO_TYPE_MPU6500;
  case ICM20601_ID:
    return GYRO_TYPE_ICM20601;
  case ICM20602_ID:
    return GYRO_TYPE_ICM20602;
  case ICM20608_ID:
    return GYRO_TYPE_ICM20608;
  case ICM20649_ID:
    return GYRO_TYPE_ICM20649;
  case ICM20689_ID:
    return GYRO_TYPE_ICM20689;
  default:
    return GYRO_TYPE_INVALID;
  }
}

void mpu6xxx_configure() {
  mpu6xxx_init();

>>>>>>> 53276eff (auto detect gyro type)
  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_BIT_H_RESET); // reg 107 soft reset  MPU_BIT_H_RESET
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_SIGNAL_PATH_RESET, MPU_RESET_SIGNAL_PATHWAYS);
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROX); // reg 107 set pll clock to 1 for x axis reference
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_USER_CTRL, MPU_BIT_I2C_IF_DIS); // reg 106 to 16 enabling spi
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_PWR_MGMT_2, MPU_BITS_STDBY_MODE_OFF); // reg 108 disable standbye mode to 0
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_SMPLRT_DIV, MPU_BIT_SMPLRT_DIVIDER_OFF); // reg 25 sample rate divider to 0
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_CONFIG, MPU_BITS_DLPF_CFG_256HZ); // reg 26 dlpf to 0 - 8khz
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_ACCEL_CONFIG, MPU_BITS_FS_16G); // reg 28 accel scale to 16G
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_GYRO_CONFIG, MPU_BITS_FS_2000DPS); // reg 27 gyro scale to 2000deg/s
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_INT_ENABLE, MPU_BIT_INT_STATUS_DATA); // reg 56 data ready enable interrupt to 1
  time_delay_us(1500);
}

// blocking dma read of a single register
uint8_t mpu6xxx_read(uint8_t reg) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_INIT);

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  spi_txn_t *txn = spi_txn_init(&gyro_bus, NULL);
  spi_txn_add_seg(txn, buffer, buffer, 2);
  spi_txn_submit(txn);

  spi_txn_continue_ex(&gyro_bus, true);
  spi_txn_wait(&gyro_bus);

  return buffer[1];
}

// blocking dma write of a single register
void mpu6xxx_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_INIT);

  spi_txn_t *txn = spi_txn_init(&gyro_bus, NULL);
  spi_txn_add_seg_const(txn, reg);
  spi_txn_add_seg_const(txn, data);
  spi_txn_submit(txn);

  spi_txn_wait(&gyro_bus);
}

void mpu6xxx_read_data(uint8_t reg, uint8_t *data, uint32_t size) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, mpu6xxx_fast_divider());

  spi_txn_t *txn = spi_txn_init(&gyro_bus, NULL);
  spi_txn_add_seg_const(txn, reg | 0x80);
  spi_txn_add_seg(txn, data, NULL, size);
  spi_txn_submit(txn);

  spi_txn_wait(&gyro_bus);
}

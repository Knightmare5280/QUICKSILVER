
#include "config.h"
#include "config_helper.h"

#define IFLIGHTF411RX1280

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART2_PA3PA2     \
  SOFT_SERIAL_PORT(1, PIN_A9, PIN_A9)

#define USE_SOFT_SERIAL

// LEDS
#define LED_NUMBER 1
#define LED1PIN PIN_C13
#define LED1_INVERT

// GYRO
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_A4
#define GYRO_INT PIN_B10
#define GYRO_ORIENTATION GYRO_ROTATE_90_CCW

// RADIO
#define USE_SX128X
#define USE_SX128X_BUSY_EXTI
#define SX12XX_SPI_PORT SPI_PORT3
#define SX12XX_NSS_PIN PIN_A15
#define SX12XX_DIO0_PIN PIN_C14
#define SX12XX_BUSY_PIN PIN_A13
#define SX12XX_RESET_PIN PIN_A8

// OSD
#define USE_MAX7456
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_B0
#define VBAT_DIVIDER_R1 10000
#define VBAT_DIVIDER_R2 1000

#define IBAT_PIN PIN_B1
#define IBAT_SCALE 400

// MOTOR PINS
// S3_OUT
#define MOTOR_PIN0 MOTOR_PIN_PB6
// S4_OUT
#define MOTOR_PIN1 MOTOR_PIN_PB7
// S1_OUT
#define MOTOR_PIN2 MOTOR_PIN_PA0
// S2_OUT
#define MOTOR_PIN3 MOTOR_PIN_PA1

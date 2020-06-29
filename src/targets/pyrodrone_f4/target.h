#include "config.h"
#include "config_helper.h"

#define Pyrodrone_F4

#define F4
#define F405

//PORTS
#define SPI_PORTS \
  SPI1_PA5PA6PA7  \
  SPI2_PB13B14B15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2     \
  USART3_PB11PB10   \
  USART4_PA1PA0     \
  USART5_PD2PC12    \
  USART6_PC7PC6

//LEDS
#define LED_NUMBER 1
#define LED1PIN GPIO_Pin_4
#define LED1PORT GPIOB
#define LED1_INVERT
#define BUZZER_PIN GPIO_Pin_5
#define BUZZER_PIN_PORT GPIOB
#define BUZZER_INVERT
#define FPV_PIN GPIO_Pin_15
#define FPV_PORT GPIOA

//GYRO
#define MPU6XXX_SPI_PORT SPI_PORT1
#define MPU6XXX_NSS PIN_A4
#define MPU6XXX_INT PIN_C4
#define USE_DUMMY_I2C
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x73
#define GYRO_ID_3 0x78
#define GYRO_ID_4 0x72
#define SENSOR_ROTATE_90_CW

//RADIO
#define USART1_INVERTER_PIN GPIO_Pin_3
#define USART1_INVERTER_PORT GPIOC

#ifdef SERIAL_RX
#define RX_USART USART_PORT1
#define SOFTSPI_NONE
#endif
#ifndef SOFTSPI_NONE
#define RADIO_CHECK
#define SPI_MISO_PIN GPIO_Pin_10
#define SPI_MISO_PORT GPIOA
#define SPI_MOSI_PIN GPIO_Pin_9
#define SPI_MOSI_PORT GPIOA
#define SPI_CLK_PIN GPIO_Pin_3
#define SPI_CLK_PORT GPIOA
#define SPI_SS_PIN GPIO_Pin_2
#define SPI_SS_PORT GPIOA
#endif

//OSD
#define ENABLE_OSD
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

//VOLTAGE DIVIDER
#define BATTERYPIN GPIO_Pin_2
#define BATTERYPORT GPIOC
#define BATTERY_ADC_CHANNEL ADC_Channel_12
#ifndef VOLTAGE_DIVIDER_R1
#define VOLTAGE_DIVIDER_R1 10000
#endif
#ifndef VOLTAGE_DIVIDER_R2
#define VOLTAGE_DIVIDER_R2 1000
#endif
#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3.3
#endif

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PC9
#define MOTOR_PIN1 MOTOR_PIN_PA8
#define MOTOR_PIN2 MOTOR_PIN_PB1
#define MOTOR_PIN3 MOTOR_PIN_PB0

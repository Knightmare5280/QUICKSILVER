#include "config.h"
#include "config_helper.h"

#define BetaFPVF411RX

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2     \
  SOFT_SERIAL_PORT(1, PIN_A8, PIN_A8)

#define USE_SOFT_SERIAL

// LEDS
#define LED_NUMBER 1
#define LED1PIN PIN_C14
#define LED1_INVERT
#define BUZZER_PIN PIN_A14
//#define BUZZER_INVERT

//--------------------------------------------------------------
// RGB LEDs
//--------------------------------------------------------------
#define RGB_LED_NUMBER 2
//--------------------------------------------------------------
// DMA => TIM3-CC4 => DMA1, Channel5, Stream2 (See drv_dma.c)
//--------------------------------------------------------------
#define RGB_LED_DMA        DMA_DEVICE_TIM3_CH4
//--------------------------------------------------------------
// pin / port for the RGB led
//--------------------------------------------------------------
#define RGB_PIN LL_GPIO_PIN_1
#define RGB_PORT GPIOB
#define RGB_PIN_CLOCK LL_AHB1_GRP1_PERIPH_GPIOB
//--------------------------------------------------------------
// Timer for the data signal => TIM2
//--------------------------------------------------------------
#define  RGB_TIM_CLOCK     LL_APB1_GRP1_PERIPH_TIM3
#define  RGB_TIMER         TIM3
#define  RGB_TIM_CHAN      LL_TIM_CHANNEL_CH4
#define  RGB_TIM_CCR       CCR4
#define  RGB_TIM_AF        GPIO_AF2_TIM3


// GYRO
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_A4
#define GYRO_INT PIN_B6

// RADIO
#define USE_SX128X
#define SX12XX_SPI_PORT SPI_PORT3
#define SX12XX_NSS_PIN PIN_A15
#define SX12XX_DIO0_PIN PIN_C13
#define SX12XX_BUSY_PIN PIN_A13
#define SX12XX_RESET_PIN PIN_B9

#ifdef SERIAL_RX
#define RX_USART USART_PORT1
#endif

// OSD
#define USE_MAX7456
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_A1
#define VBAT_DIVIDER_R1 10000
#define VBAT_DIVIDER_R2 1000

#define IBAT_PIN PIN_B0

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PB10
#define MOTOR_PIN1 MOTOR_PIN_PB7
#define MOTOR_PIN2 MOTOR_PIN_PB8
#define MOTOR_PIN3 MOTOR_PIN_PA0

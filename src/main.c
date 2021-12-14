#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "debug.h"
#include "drv_adc.h"
#include "drv_fmc.h"
#include "drv_gpio.h"
#include "drv_motor.h"
#include "drv_rgb_led.h"
#include "drv_serial.h"
#include "drv_spi.h"
#include "drv_spi_soft.h"
#include "drv_time.h"
#include "drv_usb.h"
#include "failloop.h"
#include "flash.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "flight/gestures.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/sixaxis.h"
#include "io/blackbox.h"
#include "io/buzzer.h"
#include "io/led.h"
#include "io/rgb_led.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd_render.h"
#include "profile.h"
#include "project.h"
#include "reset.h"
#include "rx.h"
#include "scheduler.h"
#include "util/util.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#include "drv_serial_4way.h"
#include "drv_serial_soft.h"
#endif

int random_seed = 0;

__attribute__((__used__)) void memory_section_init() {
#ifdef USE_FAST_RAM
  extern uint8_t _fast_ram_start;
  extern uint8_t _fast_ram_end;
  extern uint8_t _fast_ram_data;
  memcpy(&_fast_ram_start, &_fast_ram_data, (size_t)(&_fast_ram_end - &_fast_ram_start));
#endif
#ifdef USE_DMA_RAM
  extern uint8_t _dma_ram_start;
  extern uint8_t _dma_ram_end;
  extern uint8_t _dma_ram_data;

#ifdef STM32H7
  HAL_MPU_Disable();

  MPU_Region_InitTypeDef mpu_init;
  mpu_init.Enable = MPU_REGION_ENABLE;
  mpu_init.BaseAddress = (uint32_t)&_dma_ram_start;
  mpu_init.Size = MPU_REGION_SIZE_256KB;
  mpu_init.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu_init.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  mpu_init.IsCacheable = MPU_ACCESS_CACHEABLE;
  mpu_init.IsShareable = MPU_ACCESS_SHAREABLE;
  mpu_init.Number = MPU_REGION_NUMBER0;
  mpu_init.TypeExtField = MPU_TEX_LEVEL1;
  mpu_init.SubRegionDisable = 0x00;
  mpu_init.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&mpu_init);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
#endif

  memcpy(&_dma_ram_start, &_dma_ram_data, (size_t)(&_dma_ram_end - &_dma_ram_start));
#endif
}

__attribute__((__used__)) int main() {
  scheduler_init();

  // init timer so we can use delays etc
  time_init();

  // load default profile
  profile_set_defaults();

  // setup filters early
  filter_global_init();
  pid_init();

  // load flash saved variables
  flash_load();

  // wait for flash to stabilze
  time_delay_us(100);

  // init some hardware things
  gpio_init();
  ledon(255); // Turn on LED during boot so that if a delay is used as part of using programming pins for other functions, the FC does not appear inactive while programming times out

  debug_pin_init();
  buzzer_init();

  usb_init();
  spi_init();

  rx_spektrum_bind();

  // init motors and send values so escs init
  motor_init();
  motor_set_all(MOTOR_OFF);
  motor_update();

  // wait for devices to wake up
  time_delay_ms(300);

  if (!sixaxis_init()) {
    // gyro not found
    failloop(FAILLOOP_GYRO);
  }

  // give adc time to settle
  time_delay_ms(100);

  adc_init();
  vbat_init();

  vtx_init();
  rx_init();
  rgb_init();

  osd_init();

  // give the gyro some time to settle
  time_delay_ms(100);

  sixaxis_gyro_cal();

#ifdef ENABLE_BLACKBOX
  blackbox_init();
#endif

  imu_init();

  osd_clear();

  while (1) {
    scheduler_update();
  }
}

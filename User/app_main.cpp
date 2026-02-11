#include "app_main.h"

#include "cdc_uart.hpp"
#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_dac.hpp"
#include "stm32_flash.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "stm32_watchdog.hpp"
#include "flash_map.hpp"

using namespace LibXR;

/* User Code Begin 1 */
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern PCD_HandleTypeDef hpcd_USB_DEVICE;
extern PCD_HandleTypeDef hpcd_USB_FS;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3;

/* DMA Resources */
static uint16_t adc1_buf[32];
static uint16_t adc2_buf[64];
static uint8_t usart3_tx_buf[128];
static uint8_t usart3_rx_buf[128];

extern "C" void app_main(void) {
  // clang-format on
  // NOLINTEND
  /* User Code Begin 2 */
  
  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32Timebase timebase;
  PlatformInit();
  STM32PowerManager power_manager;

  /* GPIO Configuration */
  STM32GPIO SPI3_CS(SPI3_CS_GPIO_Port, SPI3_CS_Pin);
  STM32GPIO LCD_CS(LCD_CS_GPIO_Port, LCD_CS_Pin);
  STM32GPIO LCD_BLK(LCD_BLK_GPIO_Port, LCD_BLK_Pin);
  STM32GPIO R_EN(R_EN_GPIO_Port, R_EN_Pin);
  STM32GPIO LCD_RES(LCD_RES_GPIO_Port, LCD_RES_Pin);
  STM32GPIO LCD_DC(LCD_DC_GPIO_Port, LCD_DC_Pin);
  STM32GPIO PC6(GPIOC, GPIO_PIN_6);
  STM32GPIO PC7(GPIOC, GPIO_PIN_7);
  STM32GPIO PC8(GPIOC, GPIO_PIN_8);
  STM32GPIO PC9(GPIOC, GPIO_PIN_9);
  STM32GPIO SPI1_CSN(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin);

  STM32ADC adc1(&hadc1, adc1_buf, {ADC_CHANNEL_11, ADC_CHANNEL_12}, 3.3);
  auto adc1_adc_channel_11 = adc1.GetChannel(0);
  UNUSED(adc1_adc_channel_11);
  auto adc1_adc_channel_12 = adc1.GetChannel(1);
  UNUSED(adc1_adc_channel_12);

  STM32ADC adc2(&hadc2, adc2_buf, {ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8}, 3.3);
  auto adc2_adc_channel_5 = adc2.GetChannel(0);
  UNUSED(adc2_adc_channel_5);
  auto adc2_adc_channel_6 = adc2.GetChannel(1);
  UNUSED(adc2_adc_channel_6);
  auto adc2_adc_channel_7 = adc2.GetChannel(2);
  UNUSED(adc2_adc_channel_7);
  auto adc2_adc_channel_8 = adc2.GetChannel(3);
  UNUSED(adc2_adc_channel_8);

  STM32PWM pwm_tim1_ch1n(&htim1, TIM_CHANNEL_1, true);
  STM32PWM pwm_tim1_ch2n(&htim1, TIM_CHANNEL_2, true);
  STM32PWM pwm_tim1_ch3n(&htim1, TIM_CHANNEL_3, true);

  STM32SPI spi1(&hspi1, {nullptr, 0}, {nullptr, 0}, 3);

  STM32SPI spi3(&hspi3, {nullptr, 0}, {nullptr, 0}, 3);

  STM32UART usart3(&huart3,
              usart3_rx_buf, usart3_tx_buf, 5);

  STM32CANFD fdcan1(&hfdcan1, 5);

  /* Terminal Configuration */

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  while(true) {
    Thread::Sleep(UINT32_MAX);
  }
  /* User Code End 3 */
}
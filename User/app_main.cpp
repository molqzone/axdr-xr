#include "app_main.h"

#include <array>
#include <cstdarg>
#include <cstdio>

#include "cdc_uart.hpp"
#include "controller.hpp"
#include "cordic.h"
#include "current.hpp"
#include "dumb/dumb_current.hpp"
#include "dumb/dumb_position.hpp"
#include "flash_map.hpp"
#include "inverter.hpp"
#include "libxr.hpp"
#include "main.h"
#include "position.hpp"
#include "rat.h"
#include "rat_bridge.hpp"
#include "rat_gen.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_cordic.hpp"
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

using namespace LibXR;

/* User Code Begin 1 */
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern PCD_HandleTypeDef hpcd_USB_FS;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3;

/* DMA Resources */
static uint16_t adc1_buf[32];
static uint16_t adc2_buf[64];
static uint8_t spi1_tx_buf[32];
static uint8_t spi1_rx_buf[32];
static uint8_t usart3_tx_buf[128];
static uint8_t usart3_rx_buf[128];

namespace
{
using BenchPosition = LibXR::FOC::DumbPosition;
using BenchCurrent = LibXR::FOC::DumbCurrent;

// @rat, plot
struct RatFocHeartbeat
{
  uint32_t tick_ms = 0u;
  float phase = 0.0f;
  float iq_target = 1.0f;
};

struct BenchInverter
{
  float duty_acc = 0.0f;
  LibXR::FOC::DutyUVW last_duty = {};

  [[nodiscard]] LibXR::ErrorCode Enable(bool)
  {
    return LibXR::ErrorCode::OK;
  }

  [[nodiscard]] LibXR::ErrorCode Disable(bool)
  {
    return LibXR::ErrorCode::OK;
  }

  [[nodiscard]] LibXR::ErrorCode SetDuty(LibXR::FOC::DutyUVW duty, bool)
  {
    last_duty = duty;
    duty_acc += duty.u + duty.v + duty.w;
    return LibXR::ErrorCode::OK;
  }
};

template <typename Fn>
[[nodiscard]] uint32_t MeasureCycles(Fn&& fn, uint32_t loops)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  __DSB();
  __ISB();

  DWT->CYCCNT = 0U;
  uint32_t start = DWT->CYCCNT;
  for (uint32_t i = 0; i < loops; ++i)
  {
    fn();
  }
  uint32_t end = DWT->CYCCNT;

  __DSB();
  __ISB();
  if (primask == 0U)
  {
    __enable_irq();
  }
  return end - start;
}

void EnableDwtCycleCounter()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR)
  DWT->LAR = 0xC5ACCE55U;
#endif
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void RunFocPathBenchmark()
{
  using PositionWrapper = LibXR::FOC::Position<BenchPosition>;
  using CurrentWrapper = LibXR::FOC::Current<BenchCurrent>;
  using InverterWrapper = LibXR::FOC::Inverter<BenchInverter>;
  using FocController = LibXR::FOC::Controller<PositionWrapper, CurrentWrapper, InverterWrapper>;

  struct BenchRig
  {
    BenchPosition pos = {};
    BenchCurrent cur = {};
    BenchInverter inv = {};
    PositionWrapper pos_wrap;
    CurrentWrapper cur_wrap;
    InverterWrapper inv_wrap;
    FocController ctrl;

    BenchRig()
        : pos_wrap(pos), cur_wrap(cur), inv_wrap(inv, 24.0f), ctrl(pos_wrap, cur_wrap, inv_wrap)
    {
      pos.SetAngle(0.0f);
      pos.SetDelta(0.00035f);
      cur.SetPhase(0.0f);
      cur.SetDelta(0.00042f);
      cur.SetAmplitude(0.8f);
    }
  };

  FocController::Configuration cfg = {};
  cfg.pole_pairs = 7.0f;
  cfg.electrical_offset = 0.0f;
  cfg.output_voltage_limit = 12.0f;
  cfg.d_axis.kp = 0.2f;
  cfg.d_axis.ki = 4.0f;
  cfg.d_axis.integral_limit = 3.0f;
  cfg.d_axis.output_limit = 6.0f;
  cfg.q_axis.kp = 0.2f;
  cfg.q_axis.ki = 4.0f;
  cfg.q_axis.integral_limit = 3.0f;
  cfg.q_axis.output_limit = 6.0f;

  auto config_controller = [&](FocController& ctrl)
  {
    ctrl.SetConfig(cfg);
    ctrl.SetTargetIq(1.0f);
    ctrl.SetTargetId(0.0f);
  };

  constexpr std::array<uint32_t, 7> WAIT_CYCLES_LIST = {0U, 4U, 8U, 16U, 32U, 64U, 128U};

  constexpr uint32_t WARMUP_LOOPS = 2000U;
  constexpr uint32_t TEST_LOOPS = 20000U;
  constexpr float DT = 0.00005f;  // 20kHz control period

  BenchRig sw_rig;
  config_controller(sw_rig.ctrl);
  for (uint32_t i = 0; i < WARMUP_LOOPS; ++i)
  {
    (void)sw_rig.ctrl.Step(DT, false);
  }

  EnableDwtCycleCounter();
  uint32_t sw_total = MeasureCycles(
      [&]()
      {
        (void)sw_rig.ctrl.Step(DT, false);
      },
      TEST_LOOPS);

  uint32_t sw_avg = sw_total / TEST_LOOPS;

  auto print_line = [](const char* fmt, ...)
  {
    char line[192] = {};
    va_list args;
    va_start(args, fmt);
    int len = std::vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (len > 0)
    {
      if (len >= static_cast<int>(sizeof(line)))
      {
        len = static_cast<int>(sizeof(line) - 1U);
      }
      (void)HAL_UART_Transmit(&huart3, reinterpret_cast<uint8_t*>(line),
                              static_cast<uint16_t>(len), 1000U);
    }
  };

  LibXR::STDIO::Printf("[FOC BENCH] loops=%lu\r\n", static_cast<unsigned long>(TEST_LOOPS));
  LibXR::STDIO::Printf("[FOC BENCH] step(cmsis): total=%lu avg=%lu cycles duty_acc=%.6f\r\n",
                       static_cast<unsigned long>(sw_total), static_cast<unsigned long>(sw_avg),
                       sw_rig.inv.duty_acc);

  print_line("[FOC BENCH] loops=%lu\r\n", static_cast<unsigned long>(TEST_LOOPS));
  print_line("[FOC BENCH] step(cmsis): total=%lu avg=%lu cycles duty_acc=%.6f\r\n",
             static_cast<unsigned long>(sw_total), static_cast<unsigned long>(sw_avg),
             static_cast<double>(sw_rig.inv.duty_acc));

  uint32_t best_wait_cycles = 0U;
  uint32_t best_cd_total = 0U;
  uint32_t best_cd_avg = 0xFFFFFFFFU;
  float best_duty_acc = 0.0f;

  for (uint32_t wait_cycles : WAIT_CYCLES_LIST)
  {
    BenchRig cd_rig;
    config_controller(cd_rig.ctrl);

    LibXR::Math::STM32CordicConfig cordic_cfg = {};
    cordic_cfg.hcordic = &hcordic;
    cordic_cfg.precision = LL_CORDIC_PRECISION_6CYCLES;
    cordic_cfg.trig_scale = LL_CORDIC_SCALE_0;
    cordic_cfg.atan2_scale = LL_CORDIC_SCALE_0;
    cordic_cfg.wait_cycles = wait_cycles;

    LibXR::Math::STM32Cordic cordic(cordic_cfg);
    cordic.ConfigureSinCosMode();

    for (uint32_t i = 0; i < WARMUP_LOOPS; ++i)
    {
      (void)cd_rig.ctrl.StepWithTrigNoConfig(DT, false, cordic);
    }

    uint32_t cd_total = MeasureCycles(
        [&]()
        {
          (void)cd_rig.ctrl.StepWithTrigNoConfig(DT, false, cordic);
        },
        TEST_LOOPS);

    uint32_t cd_avg = cd_total / TEST_LOOPS;
    float speedup = (cd_avg > 0U) ? (static_cast<float>(sw_avg) / static_cast<float>(cd_avg))
                                  : 0.0f;

    LibXR::STDIO::Printf(
        "[FOC BENCH] step_with_trig_noconfig(cordic,wait=%lu): total=%lu avg=%lu cycles duty_acc=%.6f speedup(cmsis/cordic)=%.3f\r\n",
        static_cast<unsigned long>(wait_cycles), static_cast<unsigned long>(cd_total),
        static_cast<unsigned long>(cd_avg), cd_rig.inv.duty_acc, speedup);
    print_line(
        "[FOC BENCH] step_with_trig_noconfig(cordic,wait=%lu): total=%lu avg=%lu cycles duty_acc=%.6f speedup(cmsis/cordic)=%.3f\r\n",
        static_cast<unsigned long>(wait_cycles), static_cast<unsigned long>(cd_total),
        static_cast<unsigned long>(cd_avg), static_cast<double>(cd_rig.inv.duty_acc),
        static_cast<double>(speedup));

    if (cd_avg < best_cd_avg)
    {
      best_wait_cycles = wait_cycles;
      best_cd_total = cd_total;
      best_cd_avg = cd_avg;
      best_duty_acc = cd_rig.inv.duty_acc;
    }
  }

  if (best_cd_avg != 0xFFFFFFFFU)
  {
    float best_speedup = (best_cd_avg > 0U)
                             ? (static_cast<float>(sw_avg) / static_cast<float>(best_cd_avg))
                             : 0.0f;
    float delta_percent = (sw_avg > 0U)
                              ? ((static_cast<float>(sw_avg) - static_cast<float>(best_cd_avg)) *
                                 100.0f / static_cast<float>(sw_avg))
                              : 0.0f;

    LibXR::STDIO::Printf(
        "[FOC BENCH] best_cordic_wait=%lu total=%lu avg=%lu cycles duty_acc=%.6f speedup(cmsis/cordic)=%.3f\r\n",
        static_cast<unsigned long>(best_wait_cycles), static_cast<unsigned long>(best_cd_total),
        static_cast<unsigned long>(best_cd_avg), best_duty_acc, best_speedup);

    print_line(
        "[FOC BENCH] best_cordic_wait=%lu total=%lu avg=%lu cycles duty_acc=%.6f speedup(cmsis/cordic)=%.3f\r\n",
        static_cast<unsigned long>(best_wait_cycles), static_cast<unsigned long>(best_cd_total),
        static_cast<unsigned long>(best_cd_avg), static_cast<double>(best_duty_acc),
        static_cast<double>(best_speedup));

    if (best_cd_avg < sw_avg)
    {
      LibXR::STDIO::Printf("[FOC BENCH] verdict: CORDIC faster by %.2f%%\r\n", delta_percent);
      print_line("[FOC BENCH] verdict: CORDIC faster by %.2f%%\r\n",
                 static_cast<double>(delta_percent));
    }
    else if (best_cd_avg > sw_avg)
    {
      float slower_percent = -delta_percent;
      LibXR::STDIO::Printf("[FOC BENCH] verdict: CORDIC slower by %.2f%%\r\n", slower_percent);
      print_line("[FOC BENCH] verdict: CORDIC slower by %.2f%%\r\n",
                 static_cast<double>(slower_percent));
    }
    else
    {
      LibXR::STDIO::Printf("[FOC BENCH] verdict: tie\r\n");
      print_line("[FOC BENCH] verdict: tie\r\n");
    }
  }
}
}  // namespace

extern "C" void app_main(void) {
  // clang-format on
  // NOLINTEND
  /* User Code Begin 2 */
  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32Timebase timebase;
  PlatformInit(2, 1024);
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

  STM32SPI spi1(&hspi1, spi1_rx_buf, spi1_tx_buf, 2);

  STM32SPI spi3(&hspi3, {nullptr, 0}, {nullptr, 0}, 3);

  STM32UART usart3(&huart3,
              usart3_rx_buf, usart3_tx_buf, 5);

  STM32CANFD fdcan1(&hfdcan1, 5);

  /* Terminal Configuration */

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  RatBridge::Init();
  rat_info("axdr-xr boot");
  constexpr bool RUN_FOC_BENCHMARK_ON_BOOT = false;
  if (RUN_FOC_BENCHMARK_ON_BOOT)
  {
    RunFocPathBenchmark();
  }

  RatFocHeartbeat rat_sample = {};
  uint32_t loop_counter = 0u;
  uint32_t emit_countdown = 0u;
  uint32_t schema_countdown = 0u;
  constexpr uint32_t RAT_PERIOD_LOOPS = 20u;
  constexpr uint32_t RAT_SCHEMA_SYNC_INTERVAL_LOOPS = 1000u;
  constexpr float RAT_PHASE_STEP = 0.05f;
  constexpr float TWO_PI = 6.28318530717958647692f;
  while (true)
  {
    if (schema_countdown == 0u)
    {
      RatBridge::SyncSchema();
      schema_countdown = RAT_SCHEMA_SYNC_INTERVAL_LOOPS;
    }
    if (schema_countdown > 0u)
    {
      --schema_countdown;
    }

    if (emit_countdown == 0u)
    {
      rat_sample.tick_ms = loop_counter;
      rat_sample.phase += RAT_PHASE_STEP;
      if (rat_sample.phase >= TWO_PI)
      {
        rat_sample.phase -= TWO_PI;
      }
      rat_sample.iq_target = 1.0f;
      RAT_EMIT(RAT_ID_RATFOCHEARTBEAT, rat_sample);
      emit_countdown = RAT_PERIOD_LOOPS;
    }
    if (emit_countdown > 0u)
    {
      --emit_countdown;
    }
    ++loop_counter;
    Thread::Sleep(1u);
  }

  /* User Code End 3 */
}

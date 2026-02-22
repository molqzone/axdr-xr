#include "app_main.h"

#include <cstdarg>
#include <cstdio>

#include "cdc_uart.hpp"
#include "controller.hpp"
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
#include "st/stm32_inverter.hpp"
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
  float electrical_angle = 0.0f;
  float iq_target = 1.0f;
  float duty_u = 0.0f;
  float duty_v = 0.0f;
  float duty_w = 0.0f;
  uint32_t step_avg_cycles = 0u;
  uint32_t target_cycles = 0u;
  uint32_t target_met = 0u;
};

struct DumbFocRuntime
{
  using PositionWrapper = LibXR::FOC::Position<BenchPosition>;
  using CurrentWrapper = LibXR::FOC::Current<BenchCurrent>;
  using InverterWrapper = LibXR::FOC::Inverter<LibXR::FOC::STM32Inverter>;
  using FocController = LibXR::FOC::Controller<PositionWrapper, CurrentWrapper, InverterWrapper>;

  struct BenchmarkResult
  {
    uint32_t step_avg_cycles = 0u;
  };

  explicit DumbFocRuntime(TIM_HandleTypeDef* htim, float bus_voltage)
      : inverter_handle(htim, bus_voltage), position_wrap(position),
        current_wrap(current),
        inverter_wrap(inverter_handle, bus_voltage),
        controller(position_wrap, current_wrap, inverter_wrap)
  {
  }

  [[nodiscard]] LibXR::ErrorCode Init()
  {
    position.SetAngle(0.0f);
    position.SetDelta(0.00490f);
    current.SetPhase(0.0f);
    current.SetDelta(0.00042f);
    current.SetAmplitude(0.8f);

    LibXR::FOC::STM32Inverter::Configuration inverter_cfg = {};
    inverter_cfg.frequency_hz = 20000U;
    inverter_cfg.center_aligned = true;
    inverter_cfg.complementary_output = true;
    inverter_cfg.deadtime = 0U;
    const LibXR::ErrorCode inverter_init_ret = inverter_handle.Init(inverter_cfg);
    if (inverter_init_ret != LibXR::ErrorCode::OK)
    {
      return inverter_init_ret;
    }

    FocController::Configuration cfg = {};
    cfg.pole_pairs = 1.0f;
    cfg.electrical_offset = 0.0f;
    cfg.output_voltage_limit = 12.0f;
    cfg.d_axis.kp = 0.0f;
    cfg.d_axis.ki = 0.0f;
    cfg.d_axis.integral_limit = 0.0f;
    cfg.d_axis.output_limit = 0.0f;
    cfg.q_axis.kp = 0.2f;
    cfg.q_axis.ki = 0.0f;
    cfg.q_axis.integral_limit = 0.0f;
    cfg.q_axis.output_limit = 6.0f;
    controller.SetConfig(cfg);
    controller.SetTargetIq(2.0f);
    controller.SetTargetId(0.0f);

    const LibXR::ErrorCode enable_ret = controller.EnablePowerStage(false);
    if (enable_ret != LibXR::ErrorCode::OK)
    {
      return enable_ret;
    }

    return CalibrateStepCycles(2000U, 20000U, 0.00005f);
  }

  [[nodiscard]] LibXR::ErrorCode StepMany(uint32_t step_count, float dt)
  {
    if (step_count == 0U)
    {
      return LibXR::ErrorCode::OK;
    }

    for (uint32_t i = 0; i < step_count; ++i)
    {
      const bool capture_result = (i + 1U == step_count);
      const LibXR::ErrorCode step_ret = StepOne(dt, capture_result);
      if (step_ret != LibXR::ErrorCode::OK)
      {
        return step_ret;
      }
    }
    return LibXR::ErrorCode::OK;
  }

  [[nodiscard]] const BenchmarkResult& GetBenchmark() const { return benchmark; }

  BenchPosition position = {};
  BenchCurrent current = {};
  LibXR::FOC::STM32Inverter inverter_handle;
  PositionWrapper position_wrap;
  CurrentWrapper current_wrap;
  InverterWrapper inverter_wrap;
  FocController controller;
  FocController::StepResult last_step = {};

 private:
  static void EnableDwtCycleCounterLocal()
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR)
    DWT->LAR = 0xC5ACCE55U;
#endif
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  void ResetLoopState()
  {
    position.SetAngle(0.0f);
    current.SetPhase(0.0f);
    controller.ResetIntegral();
  }

  template <typename StepFn>
  [[nodiscard]] LibXR::ErrorCode RunBenchmarkStep(StepFn&& step_fn, uint32_t warmup_loops,
                                                  uint32_t test_loops, uint32_t& avg_cycles)
  {
    for (uint32_t i = 0; i < warmup_loops; ++i)
    {
      const LibXR::ErrorCode warmup_ret = step_fn();
      if (warmup_ret != LibXR::ErrorCode::OK)
      {
        return warmup_ret;
      }
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DSB();
    __ISB();

    DWT->CYCCNT = 0U;
    uint32_t start = DWT->CYCCNT;
    LibXR::ErrorCode measure_ret = LibXR::ErrorCode::OK;
    for (uint32_t i = 0; i < test_loops; ++i)
    {
      measure_ret = step_fn();
      if (measure_ret != LibXR::ErrorCode::OK)
      {
        break;
      }
    }
    uint32_t end = DWT->CYCCNT;

    __DSB();
    __ISB();
    if (primask == 0U)
    {
      __enable_irq();
    }

    if (measure_ret != LibXR::ErrorCode::OK)
    {
      return measure_ret;
    }

    avg_cycles = (test_loops > 0U) ? ((end - start) / test_loops) : 0U;
    return LibXR::ErrorCode::OK;
  }

  [[nodiscard]] LibXR::ErrorCode CalibrateStepCycles(uint32_t warmup_loops, uint32_t test_loops,
                                                     float dt)
  {
    EnableDwtCycleCounterLocal();
    ResetLoopState();
    const LibXR::ErrorCode sw_ret = RunBenchmarkStep(
        [&]() { return controller.Step(dt, false); }, warmup_loops, test_loops,
        benchmark.step_avg_cycles);
    if (sw_ret != LibXR::ErrorCode::OK)
    {
      return sw_ret;
    }

    ResetLoopState();
    return LibXR::ErrorCode::OK;
  }

  [[nodiscard]] LibXR::ErrorCode StepOne(float dt, bool capture_result)
  {
    if (capture_result)
    {
      return controller.StepInto(dt, false, last_step);
    }
    return controller.Step(dt, false);
  }

  BenchmarkResult benchmark = {};
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

void RunFocBenchmark()
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
      pos.SetDelta(0.00490f);
      cur.SetPhase(0.0f);
      cur.SetDelta(0.00042f);
      cur.SetAmplitude(0.8f);
    }
  };

  FocController::Configuration cfg = {};
  cfg.pole_pairs = 1.0f;
  cfg.electrical_offset = 0.0f;
  cfg.output_voltage_limit = 12.0f;
  cfg.d_axis.kp = 0.0f;
  cfg.d_axis.ki = 0.0f;
  cfg.d_axis.integral_limit = 0.0f;
  cfg.d_axis.output_limit = 0.0f;
  cfg.q_axis.kp = 0.2f;
  cfg.q_axis.ki = 0.0f;
  cfg.q_axis.integral_limit = 0.0f;
  cfg.q_axis.output_limit = 6.0f;

  auto config_controller = [&](FocController& ctrl)
  {
    ctrl.SetConfig(cfg);
    ctrl.SetTargetIq(2.0f);
    ctrl.SetTargetId(0.0f);
  };

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
  LibXR::STDIO::Printf("[FOC BENCH] step(sw only): total=%lu avg=%lu cycles duty_acc=%.6f\r\n",
                       static_cast<unsigned long>(sw_total), static_cast<unsigned long>(sw_avg),
                       sw_rig.inv.duty_acc);

  print_line("[FOC BENCH] loops=%lu\r\n", static_cast<unsigned long>(TEST_LOOPS));
  print_line("[FOC BENCH] step(sw only): total=%lu avg=%lu cycles duty_acc=%.6f\r\n",
             static_cast<unsigned long>(sw_total), static_cast<unsigned long>(sw_avg),
             static_cast<double>(sw_rig.inv.duty_acc));
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
    RunFocBenchmark();
  }

  constexpr uint32_t FOC_TARGET_STEP_CYCLES = 900U;
  constexpr float FOC_DT = 0.00005f;  // 20kHz
  constexpr uint32_t FOC_SUBSTEPS_PER_LOOP = 20u;
  DumbFocRuntime foc_runtime(&htim1, 24.0f);
  const LibXR::ErrorCode foc_init_ret = foc_runtime.Init();
  const bool foc_runtime_ready = (foc_init_ret == LibXR::ErrorCode::OK);
  bool foc_runtime_faulted = false;
  uint32_t step_avg_cycles = 0u;
  uint32_t target_met = 0u;
  if (foc_runtime_ready)
  {
    const auto& BENCH = foc_runtime.GetBenchmark();
    step_avg_cycles = BENCH.step_avg_cycles;
    target_met = (BENCH.step_avg_cycles <= FOC_TARGET_STEP_CYCLES) ? 1U : 0U;

    rat_info("foc cycles step=%lu target=%lu met=%lu",
             static_cast<unsigned long>(step_avg_cycles),
             static_cast<unsigned long>(FOC_TARGET_STEP_CYCLES),
             static_cast<unsigned long>(target_met));

    rat_info("dumb foc runtime ready");
  }
  else
  {
    rat_info("dumb foc runtime init failed=%d", static_cast<int>(foc_init_ret));
  }

  RatFocHeartbeat rat_sample = {};
  uint32_t loop_counter = 0u;
  uint32_t emit_countdown = 0u;
  uint32_t schema_countdown = 0u;
  constexpr uint32_t RAT_PERIOD_LOOPS = 5u;
  constexpr uint32_t RAT_SCHEMA_SYNC_INTERVAL_LOOPS = 5000u;
  while (true)
  {
    if (foc_runtime_ready && !foc_runtime_faulted)
    {
      const LibXR::ErrorCode step_ret = foc_runtime.StepMany(FOC_SUBSTEPS_PER_LOOP, FOC_DT);
      if (step_ret != LibXR::ErrorCode::OK)
      {
        foc_runtime_faulted = true;
        rat_info("dumb foc runtime fault=%d", static_cast<int>(step_ret));
      }
    }

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
      if (foc_runtime_ready)
      {
        const auto& STEP = foc_runtime.last_step;
        rat_sample.electrical_angle = STEP.electrical_angle;
        rat_sample.iq_target = STEP.target_current_dq.q;
        rat_sample.duty_u = STEP.duty.u;
        rat_sample.duty_v = STEP.duty.v;
        rat_sample.duty_w = STEP.duty.w;
      }
      else
      {
        rat_sample.electrical_angle = 0.0f;
        rat_sample.iq_target = 0.0f;
        rat_sample.duty_u = 0.0f;
        rat_sample.duty_v = 0.0f;
        rat_sample.duty_w = 0.0f;
      }
      rat_sample.step_avg_cycles = step_avg_cycles;
      rat_sample.target_cycles = FOC_TARGET_STEP_CYCLES;
      rat_sample.target_met = target_met;
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

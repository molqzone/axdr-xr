#pragma once

#include <cmath>
#include <cstdint>

#include "main.h"
#include "libxr_def.hpp"

#if defined(HAL_CORDIC_MODULE_ENABLED) && defined(CORDIC)
#include "stm32g4xx_ll_cordic.h"

#include "cordic.hpp"

namespace LibXR::Math
{

struct STM32CordicConfig
{
  CORDIC_HandleTypeDef* hcordic = nullptr;
  uint32_t precision = LL_CORDIC_PRECISION_6CYCLES;
  uint32_t trig_scale = LL_CORDIC_SCALE_0;
  uint32_t atan2_scale = LL_CORDIC_SCALE_0;
  // 0 means single-check fast-fail (no spin). Increase only when explicitly needed.
  uint32_t wait_cycles = 0U;
};

class STM32Cordic : public Cordic
{
 public:
  explicit STM32Cordic(const STM32CordicConfig& config = {})
      : config_(config),
        instance_((config.hcordic != nullptr) ? config.hcordic->Instance : nullptr)
  {
    ASSERT(instance_ != nullptr);
  }

  void ConfigureSinCosMode() const
  {
    LL_CORDIC_Config(instance_, LL_CORDIC_FUNCTION_COSINE, config_.precision,
                     config_.trig_scale, LL_CORDIC_NBWRITE_1, LL_CORDIC_NBREAD_2,
                     LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
  }

  void ConfigureSinMode() const
  {
    LL_CORDIC_Config(instance_, LL_CORDIC_FUNCTION_SINE, config_.precision,
                     config_.trig_scale, LL_CORDIC_NBWRITE_1, LL_CORDIC_NBREAD_1,
                     LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
  }

  void ConfigureCosMode() const
  {
    LL_CORDIC_Config(instance_, LL_CORDIC_FUNCTION_COSINE, config_.precision,
                     config_.trig_scale, LL_CORDIC_NBWRITE_1, LL_CORDIC_NBREAD_1,
                     LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
  }

  void ConfigureAtan2Mode() const
  {
    LL_CORDIC_Config(instance_, LL_CORDIC_FUNCTION_PHASE, config_.precision,
                     config_.atan2_scale, LL_CORDIC_NBWRITE_2, LL_CORDIC_NBREAD_1,
                     LL_CORDIC_INSIZE_32BITS, LL_CORDIC_OUTSIZE_32BITS);
  }

  [[nodiscard]] TrigPair SinCos(float radians) const override
  {
    ConfigureSinCosMode();
    return SinCosNoConfig(radians);
  }

  [[nodiscard]] float Sin(float radians) const override
  {
    ConfigureSinMode();
    return SinNoConfig(radians);
  }

  [[nodiscard]] float Cos(float radians) const override
  {
    ConfigureCosMode();
    return CosNoConfig(radians);
  }

  [[nodiscard]] float Atan2(float y, float x) const override
  {
    ConfigureAtan2Mode();
    return Atan2NoConfig(y, x);
  }

  [[nodiscard]] TrigPair SinCosNoConfig(float radians) const
  {
    return SinCosFastNoConfig(WrapPiFast(radians));
  }

  [[nodiscard]] TrigPair SinCosFastNoConfig(float wrapped_radians) const
  {
    return SinCosQ31NoConfig(RadiansToQ31Wrapped(wrapped_radians));
  }

  [[nodiscard]] TrigPair SinCosQ31NoConfig(int32_t angle_q31) const
  {
    LL_CORDIC_WriteData(instance_, static_cast<uint32_t>(angle_q31));
    uint32_t cos_q31_raw = 0U;
    uint32_t sin_q31_raw = 0U;
    if (!ReadWhenReady(cos_q31_raw) || !ReadWhenReady(sin_q31_raw))
    {
      return {};
    }

    return {Q31ToFloat(static_cast<int32_t>(sin_q31_raw)),
            Q31ToFloat(static_cast<int32_t>(cos_q31_raw))};
  }

  [[nodiscard]] float SinNoConfig(float radians) const
  {
    return SinFastNoConfig(WrapPiFast(radians));
  }

  [[nodiscard]] float SinFastNoConfig(float wrapped_radians) const
  {
    return SinQ31NoConfig(RadiansToQ31Wrapped(wrapped_radians));
  }

  [[nodiscard]] float SinQ31NoConfig(int32_t angle_q31) const
  {
    LL_CORDIC_WriteData(instance_, static_cast<uint32_t>(angle_q31));
    uint32_t sin_q31_raw = 0U;
    if (!ReadWhenReady(sin_q31_raw))
    {
      return 0.0f;
    }
    return Q31ToFloat(static_cast<int32_t>(sin_q31_raw));
  }

  [[nodiscard]] float CosNoConfig(float radians) const
  {
    return CosFastNoConfig(WrapPiFast(radians));
  }

  [[nodiscard]] float CosFastNoConfig(float wrapped_radians) const
  {
    return CosQ31NoConfig(RadiansToQ31Wrapped(wrapped_radians));
  }

  [[nodiscard]] float CosQ31NoConfig(int32_t angle_q31) const
  {
    LL_CORDIC_WriteData(instance_, static_cast<uint32_t>(angle_q31));
    uint32_t cos_q31_raw = 0U;
    if (!ReadWhenReady(cos_q31_raw))
    {
      return 0.0f;
    }
    return Q31ToFloat(static_cast<int32_t>(cos_q31_raw));
  }

  [[nodiscard]] float Atan2NoConfig(float y, float x) const
  {
    if (x == 0.0f && y == 0.0f)
    {
      return 0.0f;
    }

    const float abs_x = std::fabs(x);
    const float abs_y = std::fabs(y);
    const float scale = (abs_x > abs_y) ? abs_x : abs_y;
    if (scale <= 1e-20f)
    {
      return 0.0f;
    }

    const int32_t x_q31 = FloatToQ31(x / scale);
    const int32_t y_q31 = FloatToQ31(y / scale);
    LL_CORDIC_WriteData(instance_, static_cast<uint32_t>(x_q31));
    LL_CORDIC_WriteData(instance_, static_cast<uint32_t>(y_q31));

    uint32_t phase_q31_raw = 0U;
    if (!ReadWhenReady(phase_q31_raw))
    {
      return 0.0f;
    }
    return Q31ToFloat(static_cast<int32_t>(phase_q31_raw)) * kPi;
  }

 private:
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kInvPi = 0.31830988618379067154f;
  static constexpr float kQ31Scale = 2147483648.0f;
  static constexpr float kQ31ToFloatScale = 1.0f / kQ31Scale;
  static constexpr float kTwoPi = 6.28318530717958647692f;

  [[nodiscard]] static int32_t FloatToQ31(float normalized)
  {
    constexpr float kUpper = 0.9999999995343387f;  // (2^31 - 1) / 2^31
    if (normalized > kUpper)
    {
      normalized = kUpper;
    }
    else if (normalized < -1.0f)
    {
      normalized = -1.0f;
    }
    return static_cast<int32_t>(normalized * kQ31Scale);
  }

  [[nodiscard]] static int32_t RadiansToQ31Wrapped(float wrapped_radians)
  {
    return static_cast<int32_t>(wrapped_radians * (kInvPi * kQ31Scale));
  }

  [[nodiscard]] static float Q31ToFloat(int32_t value)
  {
    return static_cast<float>(value) * kQ31ToFloatScale;
  }

  [[nodiscard]] static float WrapPiFast(float radians)
  {
    if (radians >= kPi)
    {
      return radians - kTwoPi;
    }
    if (radians < -kPi)
    {
      return radians + kTwoPi;
    }
    return radians;
  }

  [[nodiscard]] bool ReadWhenReady(uint32_t& out) const
  {
    if (config_.wait_cycles == 0U)
    {
      if (LL_CORDIC_IsActiveFlag_RRDY(instance_) != 0U)
      {
        out = LL_CORDIC_ReadData(instance_);
        return true;
      }
      return false;
    }

    uint32_t wait = config_.wait_cycles;
    while (wait-- > 0U)
    {
      if (LL_CORDIC_IsActiveFlag_RRDY(instance_) != 0U)
      {
        out = LL_CORDIC_ReadData(instance_);
        return true;
      }
    }
    return false;
  }

  STM32CordicConfig config_ = {};
  CORDIC_TypeDef* instance_ = nullptr;
};

#define XRMATH_HAS_STM32_CORDIC 1

}  // namespace LibXR::Math

#endif

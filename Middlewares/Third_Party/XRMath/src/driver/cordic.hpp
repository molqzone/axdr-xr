#pragma once

#include <cstdint>

namespace LibXR::Math
{

struct TrigPair
{
  float sin = 0.0f;
  float cos = 1.0f;
};

class Cordic
{
 public:
  virtual ~Cordic() = default;

  [[nodiscard]] virtual TrigPair SinCos(float radians) const = 0;
  [[nodiscard]] virtual float Sin(float radians) const = 0;
  [[nodiscard]] virtual float Cos(float radians) const = 0;
  [[nodiscard]] virtual float Atan2(float y, float x) const = 0;
};

}  // namespace LibXR::Math

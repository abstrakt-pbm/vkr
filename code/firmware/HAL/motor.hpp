#pragma once

#include <robot/hal/hal_motor.hpp>

class HilIMotorHAL : public HAL::IMotorHAL {
public:
  ~HilIMotorHAL() = default;

  bool SetRawVoltage(float voltage) override;

  float GetCurrentRawVoltage() const override;
  float GetMinVoltageToStart() const override;
  float GetMaxVoltage() const override;

  bool IsAlive() const override;

  float GetMaxVelocity() const;

private:
  double m_time_step;
  float m_current_voltage;
};

#pragma once

#include <hal/hal_motor.hpp>

#include <gmock/gmock.h>

namespace HAL {

class MockIMotorHAL : public IMotorHAL {
public:
	bool SetRawVoltage(float voltage) override;
	float GetCurrentRawVoltage() const override;
	float GetMinVoltageToStart() const override;
	float GetMaxVoltage() const override;

	bool IsAlive() const override;
    
private:
    float voltage_feedback_ = 0.0f;
    float min_start_ = 0.5f;
    float max_voltage_ = 12.0f;
    bool alive_status_ = true;
    
};

} // namespace HAL

#pragma once

#include <robot/hal/hal_motor.hpp>

#include <memory>

namespace webots {
	class Robot;
	class Motor;
}

class WebotsIMotorHAL : public HAL::IMotorHAL {
public:
	WebotsIMotorHAL(std::shared_ptr<webots::Robot> robot, 
                    const std::string& motor_name);
    ~WebotsIMotorHAL() = default;

	bool SetRawVoltage(float voltage) override;

	float GetCurrentRawVoltage() const override;
	float GetMinVoltageToStart() const override;
	float GetMaxVoltage() const override;

	bool IsAlive() const override;

	float GetMaxVelocity() const;
private:
	webots::Motor* m_motor;
    std::shared_ptr<webots::Robot> m_robot;
    double m_time_step;
    float m_current_voltage;
};


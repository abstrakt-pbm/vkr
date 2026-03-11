#pragma once

#include <webots/Robot.hpp>

#include <memory>

namespace webots {
    class Motor;
    class PositionSensor;
    class Gyro;
} // namespace webots

class WebotsHAL final {
public:
	WebotsHAL();
	~WebotsHAL() = default;

	bool step();
    void set_motor_pwm(float left, float right);
    void get_encoder_ticks(int32_t& left, int32_t& right);

private:
	webots::Motor* left_motor_ = nullptr;
    webots::Motor* right_motor_ = nullptr;
    webots::PositionSensor* left_encoder_ = nullptr;
    webots::PositionSensor* right_encoder_ = nullptr;
    webots::Gyro* gyro_ = nullptr;
	std::unique_ptr<webots::Robot> m_webots_robot;
};


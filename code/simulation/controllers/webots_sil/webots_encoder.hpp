#pragma once

#include <robot/hal/hal_encoder.hpp>

#include <string>
#include <memory>

namespace webots {
	class Robot;
	class PositionSensor;
}

class WebotsIEncoderHAL final : public HAL::IEncoderHAL  {
public:
	WebotsIEncoderHAL(std::shared_ptr<webots::Robot> robot,
				   const std::string& sensor_name,
				   double wheel_radius_meters);
	~WebotsIEncoderHAL() = default;

	float GetRawLinearVelocity() const override;
	bool IsAlive() const override;
private:
	webots::PositionSensor *m_encoder;
	std::shared_ptr<webots::Robot> m_robot;
	double m_time_step;

	double m_wheel_radius_meters;
	mutable double m_prev_radians;
};

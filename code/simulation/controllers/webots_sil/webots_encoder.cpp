#include <webots_sil/webots_encoder.hpp>

#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>

WebotsIEncoderHAL::WebotsIEncoderHAL(std::shared_ptr<webots::Robot> robot,
				   const std::string& sensor_name, double wheel_radius_meters)
	:	m_encoder(nullptr),
	m_robot(std::move(robot)),
	m_time_step(0.0f),
	m_wheel_radius_meters(wheel_radius_meters),
	m_prev_radians(0.0f) {
	if (m_robot && m_wheel_radius_meters > 0.0f) {
		m_time_step = m_robot->getBasicTimeStep();
		m_encoder = robot->getPositionSensor(sensor_name);
		if (m_encoder) {
			m_encoder->enable(static_cast<int>(m_time_step));
		}
	}
}


float WebotsIEncoderHAL::GetRawLinearVelocity() const {
	if (IsAlive() && m_wheel_radius_meters > 0.0f) {
		double current_radians = m_encoder->getValue();
		double delta_radians = current_radians - m_prev_radians;
		float dt_sec = static_cast<float>(m_time_step / 1000.0);
		float angular_velocity_rad_sec = static_cast<float>(delta_radians / dt_sec);

		m_prev_radians = current_radians;

		return angular_velocity_rad_sec * m_wheel_radius_meters;
	}
	return 0.0f;
}

bool WebotsIEncoderHAL::IsAlive() const {
	 return m_robot && m_encoder && m_encoder->getSamplingPeriod() > 0;
}


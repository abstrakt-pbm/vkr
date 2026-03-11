#pragma once

#include <robot/hal/hal_imu.hpp>

#include <memory>

namespace webots {
	class Robot;
	class InertialUnit;
	class Accelerometer;
}

class WebotsIImuHAL final : public HAL::IImuHAL {
public:
	WebotsIImuHAL(std::shared_ptr<webots::Robot> robot, 
			   const std::string& imu_name,
			   const std::string& accelerometer_name);
	~WebotsIImuHAL() = default;

	float GetRawGyroX() const override;
	float GetRawGyroY() const override;
	float GetRawGyroZ() const override;

	float GetAccelerationX() const override;
	float GetAccelerationY() const override;
	float GetAccelerationZ() const override;

	bool IsAlive() const override;
private:
	webots::InertialUnit *m_imu;
	webots::Accelerometer *m_accelerometer;
    std::shared_ptr<webots::Robot> m_robot;
    double m_time_step;
    mutable double m_prev_time;
};


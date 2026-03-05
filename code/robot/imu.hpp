#pragma once

namespace HAL {
	class IImuHAL;
} // namespace HAL

namespace Robot {
class IMU {
public:
	IMU(HAL::IImuHAL &imu_hal);
	float get_linear_acceleration();
	float get_angular_velocity();
	float get_gyro_z();

	bool IsAlive();

private:
	HAL::IImuHAL &m_imu_hal;
};
} // namespace Robot


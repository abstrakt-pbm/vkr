#pragma once

namespace HAL {
	class IImuHAL;
} // namespace HAL

namespace Robot {
class IMU {
public:
	IMU(HAL::IImuHAL &imu_hal);

	float GetGyroX();
	float GetGyroY();
	float GetGyroZ();

	float GetAccelerationX();
	float GetAccelerationY();
	float GetAccelerationZ();

	bool IsAlive();

	void UpdateState();
private:
	float m_filtered_gyro_x;
	float m_filtered_gyro_y;
	float m_filtered_gyro_z;

	float m_filtered_acceleration_x;
	float m_filtered_acceleration_y;
	float m_filtered_acceleration_z;

	HAL::IImuHAL &m_imu_hal;
};
} // namespace Robot


#pragma once

#include <robot/hal/hal_imu.hpp>

namespace HAL {
class MockIImuHAL : public IImuHAL {
public:
	float GetRawGyroX() const override;
	float GetRawGyroY() const override;
	float GetRawGyroZ() const override;

	float GetAccelerationX() const override;
	float GetAccelerationY() const override;
	float GetAccelerationZ() const override;

	bool IsAlive() const override;

	// Для тестов 
	void SetRawGyroX(float gyro);
	void SetRawGyroY(float gyro);
	void SetRawGyroZ(float gyro);

	void SetAccelerationX(float accel);
	void SetAccelerationY(float accel);
	void SetAccelerationZ(float accel);
	
	void SetIsAlive(bool is_alive);

private:
	float m_gyro_x;
	float m_gyro_y;
	float m_gyro_z;

	float m_acceleration_x;
	float m_acceleration_y;
	float m_acceleration_z;
	
	bool m_is_alive = true;
};
} // namespace HAL


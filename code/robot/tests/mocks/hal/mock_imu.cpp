#include "mock_imu.hpp"

namespace HAL {

float MockIImuHAL::GetRawGyroX() const {
	return m_gyro_x;
}

float MockIImuHAL::GetRawGyroY() const {
	return m_gyro_y;
}

float MockIImuHAL::GetRawGyroZ() const {
	return m_gyro_z;
}

float MockIImuHAL::GetAccelerationX() const {
	return m_acceleration_x;
}

float MockIImuHAL::GetAccelerationY() const {
	return m_acceleration_y;
}

float MockIImuHAL::GetAccelerationZ() const {
	return m_acceleration_z;
}

bool MockIImuHAL::IsAlive() const {
	return m_is_alive;
}

void MockIImuHAL::SetIsAlive(bool is_alive) {
	m_is_alive = is_alive;
}

void MockIImuHAL::SetRawGyroX(float gyro) {
	m_gyro_x = gyro;
}

void MockIImuHAL::SetRawGyroY(float gyro) {
	m_gyro_y = gyro;
}

void MockIImuHAL::SetRawGyroZ(float gyro) {
	m_gyro_z = gyro;
}

void MockIImuHAL::SetAccelerationX(float accel) {
	m_acceleration_x = accel;
}
void MockIImuHAL::SetAccelerationY(float accel) {
	m_acceleration_y = accel;
}

void MockIImuHAL::SetAccelerationZ(float accel) {
	m_acceleration_z = accel;
}

} // namespace HAL


#include <robot/imu.hpp>
#include <hal/hal_imu.hpp>

namespace {
    constexpr float kGyroAlpha = 0.08f;
    constexpr float kAccelAlpha = 0.12f;
}

namespace Robot {
IMU::IMU(HAL::IImuHAL &imu_hal)
	:m_filtered_gyro_x(0.0f),
	m_filtered_gyro_y(0.0f),
	m_filtered_gyro_z(0.0f),
	m_filtered_acceleration_x(0.0f),
	m_filtered_acceleration_y(0.0f),
	m_filtered_acceleration_z(0.0f),
	m_imu_hal(imu_hal)
{}

float IMU::GetGyroX() {
	return m_filtered_gyro_x;
}

float IMU::GetGyroY() {
	return m_filtered_gyro_y;
}

float IMU::GetGyroZ() {
	return m_filtered_gyro_z;
}

float IMU::GetAccelerationX() {
	return m_filtered_acceleration_x;
}

float IMU::GetAccelerationY() {
	return m_filtered_acceleration_y;
}

float IMU::GetAccelerationZ() {
	return m_filtered_acceleration_z;
}

bool IMU::IsAlive() {
	return m_imu_hal.IsAlive();
}

void IMU::UpdateState() {
	if (m_imu_hal.IsAlive()) {
		float raw_gx = m_imu_hal.GetRawGyroX();
        float raw_gy = m_imu_hal.GetRawGyroY();
        float raw_gz = m_imu_hal.GetRawGyroZ();
        
        m_filtered_gyro_x = (1.0f - kGyroAlpha) * m_filtered_gyro_x + kGyroAlpha * raw_gx;
        m_filtered_gyro_y = (1.0f - kGyroAlpha) * m_filtered_gyro_y + kGyroAlpha * raw_gy;
        m_filtered_gyro_z = (1.0f - kGyroAlpha) * m_filtered_gyro_z + kGyroAlpha * raw_gz;
        
        float raw_ax = m_imu_hal.GetAccelerationX();
        float raw_ay = m_imu_hal.GetAccelerationY();
        float raw_az = m_imu_hal.GetAccelerationZ();
        
        m_filtered_acceleration_x = (1.0f - kAccelAlpha) * m_filtered_acceleration_x + kAccelAlpha * raw_ax;
        m_filtered_acceleration_y = (1.0f - kAccelAlpha) * m_filtered_acceleration_y + kAccelAlpha * raw_ay;
        m_filtered_acceleration_z = (1.0f - kAccelAlpha) * m_filtered_acceleration_z + kAccelAlpha * raw_az;
	} else {
		m_filtered_gyro_x = 0;
		m_filtered_gyro_y = 0;
		m_filtered_gyro_z = 0;
		m_filtered_acceleration_x = 0;
		m_filtered_acceleration_y = 0;
		m_filtered_acceleration_z = 0;
	}
}
} // namespace Robot


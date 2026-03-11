#include <webots_sil/webots_imu.hpp>

#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>

WebotsIImuHAL::WebotsIImuHAL(std::shared_ptr<webots::Robot> robot, 
							 const std::string& imu_name,
							 const std::string& accelerometer_name)
    : m_imu(nullptr),
	  m_accelerometer(nullptr),
      m_robot(std::move(robot)),
      m_time_step(0.0),
      m_prev_time(0.0) {
    if (m_robot) {
        m_time_step = m_robot->getBasicTimeStep();
        m_imu = m_robot->getInertialUnit(imu_name);
        if (m_imu) {
            m_imu->enable(static_cast<int>(m_time_step));
        }

		m_accelerometer = m_robot->getAccelerometer(accelerometer_name);
		if (m_accelerometer) {
            m_accelerometer->enable(static_cast<int>(m_time_step));
		}
    }
}

float WebotsIImuHAL::GetRawGyroX() const {
    if (IsAlive()) {
        const double* values = m_imu->getRollPitchYaw();
        return static_cast<float>(values[0]);
    }
    return 0.0f;
}

float WebotsIImuHAL::GetRawGyroY() const {
    if (IsAlive()) {
        const double* values = m_imu->getRollPitchYaw();
        return static_cast<float>(values[1]);
    }
    return 0.0f;
}

float WebotsIImuHAL::GetRawGyroZ() const {
    if (IsAlive()) {
        const double* values = m_imu->getRollPitchYaw();
        return static_cast<float>(values[2]);
    }
    return 0.0f;
}


float WebotsIImuHAL::GetAccelerationX() const {
    if (IsAlive()) {
        const double* values = m_accelerometer->getValues();
        return static_cast<float>(values[0]);
    }
    return 0.0f;
}

float WebotsIImuHAL::GetAccelerationY() const {
    if (IsAlive()) {
        const double* values = m_accelerometer->getValues();
        return static_cast<float>(values[1]);
    }
    return 0.0f;
}

float WebotsIImuHAL::GetAccelerationZ() const {
    if (IsAlive()) {
        const double* values = m_accelerometer->getValues();
        return static_cast<float>(values[2]);
    }
    return 0.0f;
}

bool WebotsIImuHAL::IsAlive() const {
    return m_robot && m_imu && m_accelerometer
	&& m_imu->getSamplingPeriod() > 0 
	&& m_accelerometer->getSamplingPeriod() > 0;
}


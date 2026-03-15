#include <robot/motor.hpp>

#include <hal/hal_motor.hpp>

#include <cmath>
#include <algorithm>

#include <logger.hpp>

namespace Robot {
Motor::Motor(HAL::IMotorHAL &motor_hal,
			 float ramp_coeff,
			 float max_voltage,
			 float voltage_to_start)
	:
	m_motor_hal(motor_hal),
	m_ramp_coeff(ramp_coeff),
	m_max_voltage(max_voltage),
    m_min_voltage_to_start(voltage_to_start),
	m_current_voltage(0.1f) {
	m_motor_hal.SetRawVoltage(0);
}

void Motor::SetVoltage(float target_voltage, float dt) {
	float min_voltage_to_start = m_motor_hal.GetMinVoltageToStart();
	float max_voltage = m_motor_hal.GetMaxVoltage();

	/*
	if (std::abs(target_voltage) >  1e-6f &&  std::abs(target_voltage) < m_min_voltage_to_start) {
		SetRawVoltage(m_min_voltage_to_start);
	}
	
	if (std::abs(target_voltage) < 1e-6f) {
		SetRawVoltage(0);
		return;
	}
	*/

	target_voltage = std::clamp(target_voltage,
							 -m_max_voltage, m_max_voltage);

	LOG_INFO("Motor: target voltage = %.3f", target_voltage);
	float delta = m_ramp_coeff * dt;
	float current_voltage = m_motor_hal.GetCurrentRawVoltage();

	float smooth_voltage = 0.0f;
	if (std::abs(current_voltage - target_voltage) < delta ){
		smooth_voltage = target_voltage;
	} else {
		if (target_voltage > current_voltage) {
			smooth_voltage = target_voltage + delta;
		} else {
			smooth_voltage = target_voltage - delta;
		}
	}
	LOG_INFO("Motor: smooth voltage = %.3f", smooth_voltage);
	SetRawVoltage(smooth_voltage);
}

void Motor::SetRawVoltage(float voltage) {
	float min_voltage_to_start = m_motor_hal.GetMinVoltageToStart();
	float max_voltage = m_motor_hal.GetMaxVoltage();

	voltage = std::clamp(voltage, -max_voltage, max_voltage);

	/*
	if (std::abs(voltage) >  1e-6f &&  std::abs(voltage) < min_voltage_to_start) {
		if (GetCurrentVoltage() == 0.0f && voltage > 0) {
        	voltage = min_voltage_to_start;
		} else if (GetCurrentVoltage() == 0.0f && voltage < 0){
        	voltage = -min_voltage_to_start ;
		}
	}
	*/

	//вызов на нужные пины мотора
	if (m_motor_hal.IsAlive()) {
		m_motor_hal.SetRawVoltage(voltage);
		m_current_voltage = voltage;
	}
}

float Motor::GetCurrentVoltage() {
	return m_motor_hal.GetCurrentRawVoltage();
}


bool Motor::IsAlive() {
	return m_motor_hal.IsAlive();
}

float Motor::GetMaxVoltage() const {
	return m_max_voltage;
}

}


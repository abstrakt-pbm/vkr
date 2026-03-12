#include "robot.hpp"
#include <robot/robot.hpp>

#include <robot/motor.hpp>
#include <robot/imu.hpp>
#include <robot/encoder.hpp>

#include <cmath>
#include <algorithm>
#include <iostream>

namespace Robot {

constexpr float M_MIN_DT = 1e-6f;
constexpr float M_MAX_DT = 0.05f;
constexpr float kMaxWheelSpeed = 5.0f;
constexpr float kAlpha = 0.2f;

Robot::Robot(IMU &imu,
		Motor &l_motor,
		Encoder &l_encoder,
		Motor &r_motor,
		Encoder &r_encoder,
		ActuatorLimits &acturator_limits,
		RobotKinematics &robot_kinematics)
	: m_imu(imu),
	m_l_motor(l_motor),
	m_left_encoder(l_encoder),
	m_r_motor(r_motor),
	m_right_encoder(r_encoder),
	m_limits(acturator_limits),
	m_robot_kinematics(robot_kinematics)
{
	m_ekf.Reset();
}

RobotState Robot::FetchCurrentRobotState(float dt) {
    RobotState state = m_last_state;
    
    // Валидация дельты времени
    if (dt < M_MIN_DT || dt > M_MAX_DT) {
 		return state;
	}

    // Валидация энкодеров
    if (!m_left_encoder.IsAlive() || !m_right_encoder.IsAlive()) {
        enterSafeStopMode();
		return RobotState{0.0f, 0.0f, 0.0f,
					0.0f, 0.0f, 0.0f};
	}
    
	// Отфильтрованая скорость, датчики обновлены перед FetchCurrentRobotState
	// TODO: ввести объект Wheel который будет аргерировать Encoder + Motor
	// Сразу будет отдавать скорость с нормальным знаком
	int left_wheel_direction = (m_l_motor.GetCurrentVoltage() >= 0) ? 1 : -1; 
    float v_left = m_left_encoder.GetCurrentVelocity() * left_wheel_direction;
	int right_wheel_direction = (m_l_motor.GetCurrentVoltage() >= 0) ? 1 : -1; 
    float v_right = m_right_encoder.GetCurrentVelocity() * right_wheel_direction;
    
    if (!std::isfinite(v_left) || !std::isfinite(v_right)) {
 		return state;
	}

	// Получаем линейную и угловую скорость на основе энкодеров
	float linear_robot_speed_enc = (static_cast<float>(v_left + v_right)) / 2;
	float angle_robot_speed_env = (v_right - v_left) / m_robot_kinematics.m_track_width;

	printf("current linear_speed %.3f m/s\n", linear_robot_speed_enc);

    m_ekf.Predict(v_left,
				  v_right);

    float w_imu = 0.0f;
    if (m_imu.IsAlive()) {
        w_imu = m_imu.GetGyroZ();
    } else {
        w_imu = angle_robot_speed_env;
	}

	m_ekf.Update(linear_robot_speed_enc,
			 angle_robot_speed_env,
			 w_imu);

    state.current_linear_speed = m_ekf.GetLinearVelocity();
	state.current_angular_speed = m_ekf.GetAngularVelocity();

	m_odometry.Update(state.current_linear_speed,
				   state.current_angular_speed, dt);
	
    m_last_state = state;
    return state;
}

void Robot::TransferToNewState(const ControlEffort &control_effort, float dt) {
	if (m_is_in_safe_mode) {
		return;
	}

    SaturatedEffort saturated_effort = m_limits.ApplyLimits(control_effort);
	ControlEffort safe_effort = saturated_effort.effort;
    
    if (m_l_motor.IsAlive() && m_r_motor.IsAlive()) {
        float i_left = safe_effort.left_motor_voltage;
        float i_right = safe_effort.right_motor_voltage;
        
		// TODO: Переделать на нормальное определение максимального вольтажа
		float kMaxVotage = m_l_motor.GetMaxVoltage();
        if (i_left > kMaxVotage || i_right > kMaxVotage) {
            enterSafeStopMode();
            return;
        }
    } else {
		enterSafeStopMode();
        return;
	}
    
    m_l_motor.SetVoltage(safe_effort.left_motor_voltage, dt);
    m_r_motor.SetVoltage(safe_effort.right_motor_voltage, dt);
    
    // 6. Logging last values
    // m_left_voltage_last = safe_effort.left_motor_voltage;
    // m_right_voltage_last = safe_effort.right_motor_voltage;
}


void Robot::enterSafeStopMode() {
	m_is_in_safe_mode = true;
	float max_dt = M_MAX_DT;
	while (m_l_motor.GetCurrentVoltage() > 1e-6f || m_r_motor.GetCurrentVoltage()> 1e-6f) {
		m_l_motor.SetVoltage(0.0f, max_dt);
		m_r_motor.SetVoltage(0.0f, max_dt);
	}
	m_l_motor.SetRawVoltage(0.0f);
	m_r_motor.SetRawVoltage(0.0f);
}

void Robot::UpdateSensors() {
	m_left_encoder.UpdateState();
	m_right_encoder.UpdateState();
	m_imu.UpdateState();
}

bool Robot::IsInSafeMode() {
	return m_is_in_safe_mode;
}

ActuatorLimits &Robot::GetLimits() {
	return m_limits;
}

Odometry &Robot::GetOdometry() {
	return m_odometry;
}

SaturatedEffort ActuatorLimits::ApplyLimits(const ControlEffort& requested_effort) {
	return SaturatedEffort{requested_effort, false};
}


} // namespace Robot


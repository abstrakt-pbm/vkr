#include <robot/robot.hpp>

#include <cmath>
#include <algorithm>

namespace Robot {

constexpr float M_PI = 3.14;
constexpr float M_MIN_DT = 1e-6f;
constexpr float M_MAX_DT = 0.05f;
constexpr float kMaxWheelSpeed = 5.0f;
constexpr float kAlpha = 0.2f;

RobotState Robot::FetchCurrentRobotState(float dt) {
    RobotState state = m_last_state;
    
    // Валидация дельты времени
    if (dt < M_MIN_DT || dt > M_MAX_DT) {
 		return state;
	}

    // Валидация энкодеров
    if (!m_left_encoder.IsAlive() || !m_right_encoder.IsAlive()) {
		return state;
	}
    
	// Сырая скорость
    float v_left = m_left_encoder.GetCurrentVelocity();
    float v_right = m_right_encoder.GetCurrentVelocity();
    
    if (!std::isfinite(v_left) || !std::isfinite(v_right)) {
 		return state;
	}
    
    v_left  = std::clamp(v_left,  -kMaxWheelSpeed, kMaxWheelSpeed);
    v_right = std::clamp(v_right, -kMaxWheelSpeed, kMaxWheelSpeed);
    
	// Фильтр низких частот
    m_l_filtered_velocity = (1.0f - kAlpha) * m_l_filtered_velocity + kAlpha * v_left;
    m_r_filtered_velocity = (1.0f - kAlpha) * m_r_filtered_velocity + kAlpha * v_right;

	// Получаем линейную и угловую скорость на основе энкодеров
	float linear_robot_speed_enc = (m_l_filtered_velocity + m_r_filtered_velocity) / 2;
	float angle_robot_speed_env = (m_r_filtered_velocity - m_l_filtered_velocity) / m_track_width;


    m_ekf.Predict(m_l_filtered_velocity,
				  m_r_filtered_velocity);

    float w_imu = 0.0f;
    if (imu.IsAlive()) {
        w_imu = imu.get_gyro_z();
		m_ekf.Update(linear_robot_speed_enc,
			 angle_robot_speed_env,
			 w_imu);
    }

    state.current_linear_speed = m_ekf.GetLinearVelocity();
	state.current_angular_speed = m_ekf.GetAngularVelocity();
	
    m_last_state = state;
    return state;
}

void Robot::TransferToNewState(const ControlEffort &control_effort, float dt) {
	if (m_is_in_safe_mode) {
		return;
	}

    SaturatedEffort saturated_effort = m_limits.ApplyLimits(control_effort);
	ControlEffort safe_effort = saturated_effort.effort;
    
    if (l_motor.IsAlive() && r_motor.IsAlive()) {
        float i_left = safe_effort.left_motor_voltage;
        float i_right = safe_effort.right_motor_voltage;
        
		float kMaxVotage = m_limits.GetMaxVoltage();
        if (i_left > kMaxVotage || i_right > kMaxVotage) {
            enterSafeStopMode();
            return;
        }
    }
    
    // 4. Dead-time compensation (для BLDC, опционально)
    // apply_deadtime_compensation(safe_effort);
    
    // 5. PWM output (hardware)
    l_motor.SetVoltage(safe_effort.left_motor_voltage, dt);
    r_motor.SetVoltage(safe_effort.right_motor_voltage, dt);
    
    // 6. Logging last values
    // m_left_voltage_last = safe_effort.left_motor_voltage;
    // m_right_voltage_last = safe_effort.right_motor_voltage;
}


void Robot::enterSafeStopMode() {
	m_is_in_safe_mode = true;
	float max_dt = M_MAX_DT;
	while (l_motor.GetCurrentVoltage() > 1e-6f || r_motor.GetCurrentVoltage()> 1e-6f) {
		l_motor.SetVoltage(0.0f, max_dt);
		r_motor.SetVoltage(0.0f, max_dt);
	}
	l_motor.SetRawVoltage(0.0f);
	r_motor.SetRawVoltage(0.0f);
}

// Motor
Motor::Motor(float ramp_coeff,
			 float max_voltage,
			 float voltage_to_start)
	:m_ramp_coeff(ramp_coeff),
	m_max_voltage(max_voltage),
    m_min_voltage_to_start(voltage_to_start),
	m_current_voltage(0.0f) {}

void Motor::SetVoltage(float target_voltage, float dt) {
	if (target_voltage < m_min_voltage_to_start) {
		SetRawVoltage(0);
		return;
	}

	target_voltage = std::clamp(target_voltage, m_min_voltage_to_start, m_max_voltage);

	float delta = m_ramp_coeff * dt;

	if (target_voltage > m_current_voltage) {
		SetRawVoltage(m_current_voltage + delta);
	} else {
		SetRawVoltage(m_current_voltage - delta);
	}

}

void Motor::SetRawVoltage(float voltage) {
	voltage = std::clamp(voltage, -m_max_voltage, m_max_voltage);
    if (std::abs(voltage) < m_min_voltage_to_start) {
        voltage = 0.0f;
    }
	//вызов на нужные пины мотора
	m_current_voltage = voltage;
}

float Motor::GetCurrentVoltage() {
	return m_current_voltage;
}

} // namespace Robot


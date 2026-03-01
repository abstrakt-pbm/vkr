#include <inc/robot/robot.hpp>
#include <cmath>
#include <algorithm>

namespace Robot {

RobotState Robot::FetchCurrentRobotState(float dt) {
    RobotState state = m_last_state;
    
    // 0. Timing validation
    if (dt < 1e-6f || dt > 0.05f) return state;

    // 1. Encoder health & data
    if (!l_encoder.is_healthy() || !r_encoder.is_healthy()) return state;
    
    float v_left = l_encoder.get_current_velocity();
    float v_right = r_encoder.get_current_velocity();
    
    if (!std::isfinite(v_left) || !std::isfinite(v_right)) return state;
    
    // 2. Physics clamp
    constexpr float kMaxWheelSpeed = 5.0f;
    v_left  = std::clamp(v_left,  -kMaxWheelSpeed, kMaxWheelSpeed);
    v_right = std::clamp(v_right, -kMaxWheelSpeed, kMaxWheelSpeed);
    
    // 3. EMA smoothing
    constexpr float kAlpha = 0.2f;
    m_vl_filt = (1.0f - kAlpha) * m_vl_filt + kAlpha * v_left;
    m_vr_filt = (1.0f - kAlpha) * m_vr_filt + kAlpha * v_right;
    
    // 4. Linear velocity
    state.current_linear_speed = std::clamp(
        (m_vl_filt + m_vr_filt) * 0.5f, -3.0f, 3.0f);
    
    // 5. Angular velocity: IMU first, encoders fallback
    float w_imu = 0.0f;
    if (imu.is_healthy()) {
        w_imu = imu.get_gyro_z();
    }
    
    if (std::isfinite(w_imu)) {
        state.current_angular_speed = w_imu;
    } else {
        state.current_angular_speed = (m_vr_filt - m_vl_filt) / m_track_width;
    }
    
    // 6. Odometry integration
    m_total_dist += state.current_linear_speed * dt;
    m_total_angle += state.current_angular_speed * dt;
    
    // Normalize angle [-PI, PI]
    m_total_angle = std::fmod(m_total_angle + M_PI, 2.0f * M_PI) - M_PI;
    
    state.total_linear_distance = m_total_dist;
    state.total_angular_distance = m_total_angle;
    
    // 7. Save & return
    m_last_state = state;
    return state;
}

void Robot::TransferToNewState(const ControlEffort &control_effort, float dt) {
    ControlEffort safe_effort = m_limits.ApplyFinalSafetyLimits(control_effort);
    
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
    apply_deadtime_compensation(safe_effort);
    
    // 5. PWM output (hardware)
    l_motor.SetVoltage(safe_effort.left_motor_voltage, dt);
    r_motor.SetVoltage(safe_effort.right_motor_voltage, dt);
    
    // 6. Logging last values
    m_left_voltage_last = safe_effort.left_motor_voltage;
    m_right_voltage_last = safe_effort.right_motor_voltage;
}

// Motor
Motor::Motor(float ramp_coeff,
			 float max_voltage,
			 float voltage_to_start)
	:m_ramp_coeff(ramp_coeff),
	m_max_voltage(max_voltage),
    m_min_voltage_to_start(voltage_to_start),
	m_voltage(0.0f) {}

void Motor::SetVoltage(float target_voltage, float dt) {
	if (std::abs(target_voltage - m_voltage) < 1e-6f) {
		return;
	}

	target_voltage = std::clamp(target_voltage, m_min_voltage_to_start, m_max_voltage);

	float delta = m_ramp_coeff * dt;

	if (target_voltage > m_voltage) {
		SetRawVoltage(m_voltage + delta);
	} else {
		SetRawVoltage(m_voltage - delta);
	}

}

void Motor::SetRawVoltage(float voltage) {
	//вызов на нужные пины мотора
	m_voltage = voltage;
}

float Motor::GetCurrentVoltage() {
	return m_voltage;
}

} // namespace Robot


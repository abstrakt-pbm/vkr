#pragma once

namespace Robot {

class IMU;
class Encoder;
class Motor;

class RobotEKF {
public:
	void Predict(float v_left_enc, float v_right_enc);
    void Update(float v_enc, float omega_enc, float gyro_z);

    // Основное для PID/ff
    float GetLinearVelocity() const;
    float GetAngularVelocity() const;
    void Reset();

	void SetQv(float process_noise_v); // Проскальзывание
    void SetEncNoise(float enc_noise_v, float enc_noise_omega);
    void SetGyroNoise(float gyro_noise);

	bool IsAlive();
private:	
	struct State {
		float linear_velocity;
		float angular_velocity;
	};
};

// Effort for change robot state
class ControlEffort {
public:
	float left_motor_voltage;
	float right_motor_voltage;
};

class MotionCommand {
public:
	float linear_velocity;
	float angular_velocity;
};

class RobotState {
	public:
	float current_linear_speed;
	float current_angular_speed;

	float current_linear_accel;
    float current_angular_accel;

	float left_motor_current_voltage;
    float right_motor_current_voltage;
};

class SaturatedEffort {
public:
	ControlEffort effort;
    bool is_saturated;
};

// Safe Robot
class ActuatorLimits {
public:
	SaturatedEffort ApplyLimits(const ControlEffort& requested_effort);

private:
};

class Robot {
public:
	Robot(IMU &imu,
		Motor &l_motor,
		Encoder &l_encoder,
		Motor &r_motor,
		Encoder &r_encoder,
	   	ActuatorLimits &acturator_limits);

	ActuatorLimits &GetLimits();

	RobotState FetchCurrentRobotState(float dt);
	void TransferToNewState(const ControlEffort &control_effort, float dt);
	void UpdateSensors();

	void enterSafeStopMode();

	IMU &m_imu;
	Motor &m_l_motor;
	Encoder &m_left_encoder;
	Motor &m_r_motor;
	Encoder &m_right_encoder;

private:
	RobotState m_last_state;
	ActuatorLimits &m_limits;
	
	// Robot Antropomethry
	float m_track_width = 0.3f;

	RobotEKF m_ekf;

	bool m_is_in_safe_mode;
};

} // namespace Robot


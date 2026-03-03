#pragma once

namespace Robot {

class RobotEKF {
public:
	void Predict(float v_left_enc, float v_right_enc);
    void Update(float v_enc, float omega_enc, float gyro_z);

    // Основное для PID/ff
    float GetLinearVelocity() const;
    float GetAngularVelocity() const;

    void Reset();

	void SetQv(float process_noise_v);     // Проскальзывание
    void SetEncNoise(float enc_noise_v, float enc_noise_omega);
    void SetGyroNoise(float gyro_noise);

	bool IsAlive();
private:	
	struct State {
		float linear_velocity;
		float angular_velocity;
	};
};


class Motor {
public:
	Motor(float ramp_coeff,
	   float max_voltage,
	   float voltage_to_start);
	// Безопасная функция, внутри обеспечивает рампообразное снижение/повышение напряжения за указаный промежуток времени dt
	void SetVoltage(float target_voltage, float dt);
	// Опасная функция, использовать аккуратно,
	// если разница между текущим напряжением и конечным высока, может убить мотор 
	void SetRawVoltage(float voltage);
	// Возвращает текущий установленный на моторе вольтаж
	float GetCurrentVoltage();
	// Проверка на вменяемость
	bool IsAlive();
private:

	float m_current_voltage;
	float m_ramp_coeff;
	float m_max_voltage;
	float m_min_voltage_to_start;
};

class IMU {
public:
	float get_linear_acceleration();
	float get_angular_velocity();
	float get_gyro_z();

	bool IsAlive();
};

class Encoder {
public:
	float GetCurrentVelocity();
	bool IsAlive();
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
	float GetMaxVoltage();
};

class Robot {
public:
	Robot(IMU &imu,
		Motor &l_motor,
		Encoder &l_encoder,
		Motor &r_motor,
		Encoder &r_encoder);

	ActuatorLimits &GetLimits();

	RobotState FetchCurrentRobotState(float dt);
	void TransferToNewState(const ControlEffort &control_effort, float dt);
	void enterSafeStopMode();

	IMU &imu;
	Motor &l_motor;
	Encoder &m_left_encoder;
	Motor &r_motor;
	Encoder &m_right_encoder;

private:
	RobotState m_last_state;
	ActuatorLimits &m_limits;
	
	// Robot State
	float m_l_filtered_velocity;
	float m_r_filtered_velocity;

	// Robot Antropomethry
	float m_track_width;

	RobotEKF m_ekf;

	bool m_is_in_safe_mode;
};

} // namespace Robot


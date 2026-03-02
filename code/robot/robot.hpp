#pragma once


namespace Robot {

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
	float get_current_velocity();
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
	Encoder &l_encoder;
	Motor &r_motor;
	Encoder &r_encoder;

private:
	RobotState m_last_state;
	ActuatorLimits &m_limits;
	
	// Robot Antropomethry
	float m_track_width;
};

} // namespace Robot


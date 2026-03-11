#pragma once

#include "odometry.hpp"
#include <robot/robot_ekf.hpp>
#include <robot/odometry.hpp>

namespace Robot {
class IMU;
class Encoder;
class Motor;


class RobotState {
	public:
	float current_linear_speed;
	float current_angular_speed;

	float current_linear_accel;
    float current_angular_accel;

	float left_motor_current_voltage;
    float right_motor_current_voltage;
};

class ControlEffort {
public:
	float left_motor_voltage;
	float right_motor_voltage;
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
};

class RobotKinematics {
public:
	float m_track_width = 0.3f;
};

class Robot {
public:
	Robot(IMU &imu,
		Motor &l_motor,
		Encoder &l_encoder,
		Motor &r_motor,
		Encoder &r_encoder,
	   	ActuatorLimits &acturator_limits,
		RobotKinematics &robot_kinematics);

	ActuatorLimits &GetLimits();
	Odometry &GetOdometry();

	RobotState FetchCurrentRobotState(float dt);
	void TransferToNewState(const ControlEffort &control_effort,
						 float dt);
	void UpdateSensors();

	void enterSafeStopMode();
	bool IsInSafeMode();

//private:
	IMU &m_imu;
	Motor &m_l_motor;
	Encoder &m_left_encoder;
	Motor &m_r_motor;
	Encoder &m_right_encoder;

	RobotState m_last_state;
	ActuatorLimits &m_limits;
	RobotKinematics &m_robot_kinematics;

	RobotEKF m_ekf;
	Odometry m_odometry;

	bool m_is_in_safe_mode = false;
};

} // namespace Robot


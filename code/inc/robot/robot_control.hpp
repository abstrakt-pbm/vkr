#pragma once

namespace Math {
class PID;
} // namespace Math

namespace Robot {
class RobotState;
class MotionCommand; 
} // namespace Robot

namespace RobotControl {

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
	ApplyLimits(const ControlEffort& requested_effort);
};


//FeedForwareded math model
class FFModel {
	public:
	ControlEffort GetControllEffort();	
};


class RobotController {
public:
    RobotController(FFModel &ffmodel,
					Math::PID &linear_velocity_pid,
					Math::PID &angle_velocity_pid,
					ActuatorLimits &m_limits);

    // Основной шаг управления, вызываемый в цикле main
    // Принимает состояние, цель и дельту времени (dt)
	ControlEffort GetAdjustedControlEffort(const Robot::RobotState& robot_state,
                                           const Robot::MotionCommand& motion_command,
                                           float dt);

private:
    FFModel &m_model;
	Math::PID &m_linear_velocity_pid;
	Math::PID &m_angle_velocity_pid;
	ActuatorLimits &m_limits;
};

} // namespace RobotControl


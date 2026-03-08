#pragma once

#include <robot/robot.hpp>

namespace Math {
class PID;
} // namespace Math


namespace RobotControl {

class MotionCommand {
public:
	float linear_velocity;
	float angular_velocity;
};

//FeedForwareded math model
class FFModel {
	public:
	Robot::ControlEffort GetControlEffort(const MotionCommand &cmd);	
};

class RobotController {
public:
    RobotController(Robot::Robot &robot,
					FFModel &ffmodel,
					Math::PID &linear_velocity_pid,
					Math::PID &angle_velocity_pid);

    // Основной шаг управления, вызываемый в цикле main
    // Принимает состояние, цель и дельту времени (dt)
	Robot::ControlEffort GetAdjustedControlEffort(const MotionCommand& motion_command, float dt);

private:
	Robot::Robot &m_robot;
	FFModel &m_ff_model;

	Math::PID &m_linear_velocity_pid;
	Math::PID &m_angle_velocity_pid;

	Robot::ControlEffort m_last_safe_effort;
};

} // namespace RobotControl


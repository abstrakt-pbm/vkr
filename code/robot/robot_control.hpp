#pragma once

#include <robot/robot.hpp>

namespace Math {
class PID;
} // namespace Math


namespace RobotControl {

//FeedForwareded math model
class FFModel {
	public:
	Robot::ControlEffort GetControlEffort(const Robot::MotionCommand &cmd);	
};


class RobotController {
public:
    RobotController(FFModel &ffmodel,
					Math::PID &linear_velocity_pid,
					Math::PID &angle_velocity_pid);

    // Основной шаг управления, вызываемый в цикле main
    // Принимает состояние, цель и дельту времени (dt)
	Robot::ControlEffort GetAdjustedControlEffort(const Robot::RobotState& robot_state,
                                           const Robot::MotionCommand& motion_command,
                                           float dt);

private:
    FFModel &m_model;
	Math::PID &m_linear_velocity_pid;
	Math::PID &m_angle_velocity_pid;
	Robot::ActuatorLimits m_limits;
	Robot::ControlEffort m_last_safe_effort;
};

} // namespace RobotControl


#pragma once

#include <robot/robot.hpp>

namespace Math {
class PID;
} // namespace Math


namespace RobotControl {

class FFModel;

class MotionCommand {
public:
	float linear_velocity;
	float angular_velocity;
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
	MotionCommand  VelocityRamp(const MotionCommand& target_cmd, float dt);

private:
	Robot::Robot &m_robot;
	FFModel &m_ff_model;

	Math::PID &m_linear_velocity_pid;
	Math::PID &m_angle_velocity_pid;

	float m_smooth_linear_cmd;
    float m_smooth_angular_cmd;

    // Константы максимального ускорения (подбираются по формуле из предыдущего ответа)
    // 2.0 м/с^2 = разгон до 1 м/с за 0.5 секунды
    float m_max_linear_accel;
    float m_max_angular_accel;

	Robot::ControlEffort m_last_safe_effort;
};

} // namespace RobotControl


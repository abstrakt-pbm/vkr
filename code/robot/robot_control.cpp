#include <robot/robot_control.hpp>

#include <pid.hpp>
#include <ffmodel.hpp>

#include <robot/robot.hpp>

#include <algorithm>
#include <cmath>

using namespace Robot;

namespace RobotControl {

constexpr float MAX_DT = 0.5f;
//constexpr float MIN_DT = 1e-6f;
constexpr float MIN_DT = 1e-6f;

RobotController::RobotController(Robot::Robot &robot,
								 FFModel& ffmodel, 
                                 Math::PID& linear_pid, 
                                 Math::PID& angular_pid)
    : m_robot(robot),
	m_ff_model(ffmodel),
	m_linear_velocity_pid(linear_pid),
	m_angle_velocity_pid(angular_pid),
	m_last_safe_effort{0.0f, 0.0f} 
{}

ControlEffort RobotController::GetAdjustedControlEffort(const MotionCommand& cmd, float dt) {
	if (dt > MAX_DT || m_robot.IsInSafeMode()) {
		// Слишком большая дельта, робот перестаёт быть квазистатичным, дальнейшая математика не работает, робот не безопасный,
		// поэтому останавливаем робота
		m_robot.enterSafeStopMode();
        m_linear_velocity_pid.Reset();
        m_angle_velocity_pid.Reset();
        return Robot::ControlEffort{0.0f, 0.0f};
	}

	if (dt < MIN_DT) {
		// Выглядит как проблема, но врятли состояние сильно изменится за такую короткую дельту
		// поэтому вернём последнее состояние
		// (также защита от деление на ноль)
		return m_last_safe_effort;
	}

	const Robot::RobotState	state = m_robot.FetchCurrentRobotState(dt);

    // 1. ВЫЧИСЛЕНИЕ ОШИБОК (с deadband против шума энкодеров)
    float err_v = cmd.linear_velocity - state.current_linear_speed;
    float err_w = cmd.angular_velocity - state.current_angular_speed;

    // 2. FEEDFORWARD (нелинейная модель моторов + статическое трение)
    ControlEffort ff_effort = m_ff_model.GetControlEffort(cmd);

    // 3. FEEDBACK (ПИД-регулирование напряжений)
    float pid_linear = m_linear_velocity_pid.Step(cmd.linear_velocity, state.current_linear_speed, dt);
    float pid_angular = m_angle_velocity_pid.Step(cmd.angular_velocity, state.current_angular_speed, dt);

    // 4. МИКСЕР УСИЛИЙ (Вольты)
	// pid_linear: Вольты для разгона/торможения всего робота
	// pid_angular: Вольты для создания разницы скоростей между колесами
	float left_pid_volts  = pid_linear - pid_angular;
	float right_pid_volts = pid_linear + pid_angular;

    // 5. СУММИРОВАНИЕ УСИЛИЙ (FF + FB)
    ControlEffort total_effort;
    total_effort.left_motor_voltage  = ff_effort.left_motor_voltage  + left_pid_volts;
    total_effort.right_motor_voltage = ff_effort.right_motor_voltage + right_pid_volts;

    // 6. ПРИМЕНЕНИЕ ФИЗИЧЕСКИХ ЛИМИТОВ (Proportional Desaturation)
	ActuatorLimits &limits = m_robot.GetLimits();
    SaturatedEffort safe_effort = limits.ApplyLimits(total_effort);

    // 7. ✅ ТРЕКИНГОВЫЙ ANTI-WINDUP (Tracking Back-Calculation)
    // ИСПРАВЛЕНО: правильный знак! Сколько "недодали" актуаторам
    if (safe_effort.is_saturated) {
        // Положительная разница: желаемое - реальное
        float delta_left_v  = total_effort.left_motor_voltage  - safe_effort.effort.left_motor_voltage;
        float delta_right_v = total_effort.right_motor_voltage - safe_effort.effort.right_motor_voltage;
        
        // Обратная кинематика для разницы напряжений
        float delta_linear_effort = (delta_left_v + delta_right_v) / 2.0f;
        float delta_angular_effort = (delta_right_v - delta_left_v) / 2.0f;
        
        // ПИДы получают ПОЛОЖИТЕЛЬНУЮ обратную связь для коррекции интеграла
        m_linear_velocity_pid.ApplyBackCalculation(delta_linear_effort, dt);
        m_angle_velocity_pid.ApplyBackCalculation(delta_angular_effort, dt);
    }

    // Сохраняем безопасное усилие для деградации
    m_last_safe_effort = safe_effort.effort;
    return safe_effort.effort;
}
} // namespace RobotControl


#include <robot/robot_control.hpp>
#include <algorithm>
#include <cmath>

namespace RobotControl {

// ✅ ССЫЛКИ — гарантия безопасности от nullptr (Tier-1 Safety-Critical)
RobotController::RobotController(FFModel& ffmodel, 
                                 Math::PID& linear_pid, 
                                 Math::PID& angular_pid,
                                 ActuatorLimits& limits)
    : m_model(ffmodel),
      m_linear_velocity_pid(linear_pid),
      m_angle_velocity_pid(angular_pid),
      m_limits(limits),
      m_last_safe_effort{0.0f, 0.0f} 
{}

ControlEffort RobotController::GetAdjustedControlEffort(const Robot::RobotState& state,
                                                        const Robot::MotionCommand& cmd,
                                                        float dt) {
    // 0. ЖЕСТКАЯ ВАЛИДАЦИЯ ВРЕМЕНИ (RTOS Safety)
    // Защита от деления на ноль И срыва дедлайнов
    if (dt < 1e-6f || dt > MAX_DT) {
        // Мягкая деградация: НЕ обнуляем резко (уничтожение редукторов!)
        if (dt > 0.5f) {  // Полный отказ → Emergency Stop
            m_linear_velocity_pid.reset();
            m_angle_velocity_pid.reset();
            robot.enterSafeStopMode();  // Только при критическом сбое
        }
        // Иначе продолжаем с прошлым усилием (Coasting-like)
        return m_last_safe_effort{0,0}; 
    }

    // 1. ВЫЧИСЛЕНИЕ ОШИБОК (с deadband против шума энкодеров)
    float err_v = cmd.linear_velocity - state.current_linear_speed;
    float err_w = cmd.angular_velocity - state.current_angular_speed;
    
    // Deadband 0.01 м/с против микро-шума
   	err_v = std::copysign(std::max(0.0f, std::abs(err_v) - 0.01f), err_v);
	err_w = std::copysign(std::max(0.0f, std::abs(err_w) - 0.01f), err_w);

    // 2. FEEDFORWARD (нелинейная модель моторов + статическое трение)
    ControlEffort ff_effort = m_model.GetControlEffort(cmd);

    // 3. FEEDBACK (ПИД-регулирование напряжений)
    float pid_linear = m_linear_velocity_pid.step(err_v, dt);
    float pid_angular = m_angle_velocity_pid.step(err_w, dt);

    // 4. КИНЕМАТИЧЕСКИЙ МИКСЕР УСИЛИЙ (правильные размерности!)
    // ПИДы выдают вольты → миксим просто ±
    float left_pid_volts  = pid_linear - robot.velocity_to_voltage(pid_angular);
    float right_pid_volts = pid_linear + robot.velocity_to_voltage(pid_angular);

    // 5. СУММИРОВАНИЕ УСИЛИЙ (FF + FB)
    ControlEffort total_effort;
    total_effort.left_motor_voltage  = ff_effort.left_motor_voltage  + left_pid_volts;
    total_effort.right_motor_voltage = ff_effort.right_motor_voltage + right_pid_volts;

    // 6. ПРИМЕНЕНИЕ ФИЗИЧЕСКИХ ЛИМИТОВ (Proportional Desaturation)
    SaturatedEffort safe_effort = m_limits.ApplyLimits(total_effort);

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
        m_linear_velocity_pid.apply_back_calculation(delta_linear_effort, dt);
        m_angle_velocity_pid.apply_back_calculation(delta_angular_effort, dt);
    }

    // Сохраняем безопасное усилие для деградации
    m_last_safe_effort = safe_effort.effort;
    return safe_effort.effort;
}


#include <robot/robot_control.hpp>

#include <ffmodel.hpp>
#include <pid.hpp>

#include <logger.hpp>
#include <robot/robot.hpp>

#include <cmath>

using namespace Robot;

namespace RobotControl {

constexpr float MAX_DT = 0.5f;
constexpr float MIN_DT = 1e-6f;

RobotController::RobotController(Robot::Robot &robot, FFModel &ffmodel,
                                 Math::PID &linear_pid, Math::PID &angular_pid)
    : m_robot(robot), m_ff_model(ffmodel), m_linear_velocity_pid(linear_pid),
      m_angle_velocity_pid(angular_pid), m_smooth_linear_cmd(0.0f),
      m_smooth_angular_cmd(0.0f), m_max_linear_accel(200.0f),
      m_max_angular_accel(200.0f), m_last_safe_effort{0.0f, 0.0f} {}

ControlEffort
RobotController::GetAdjustedControlEffort(const MotionCommand &cmd, float dt) {
  if (dt > MAX_DT || m_robot.IsInSafeMode()) {
    LOG_ERROR("RobotController: Delta miss: %.3f", dt);
    // Слишком большая дельта, робот перестаёт быть квазистатичным, дальнейшая
    // математика не работает, робот не безопасный, поэтому останавливаем робота
    m_robot.enterSafeStopMode();
    m_linear_velocity_pid.Reset();
    m_angle_velocity_pid.Reset();
    m_smooth_linear_cmd = 0.0f;
    m_smooth_angular_cmd = 0.0f;
    return Robot::ControlEffort{0.0f, 0.0f};
  }

  if (dt < MIN_DT) {
    LOG_ERROR("RobotController: Delta miss: %.3f", dt);
    // Выглядит как проблема, но врятли состояние сильно изменится за такую
    // короткую дельту поэтому вернём последнее состояние (также защита от
    // деление на ноль)
    return m_last_safe_effort;
  }

  const Robot::RobotState state = m_robot.FetchCurrentRobotState(dt);

  MotionCommand smooth_cmd = VelocityRamp(cmd, dt);
  // 1. Получаем управление роботом от математической модели робота
  ControlEffort ff_effort = m_ff_model.GetControlEffort(smooth_cmd);
  // 2. Снижаем ошибку ПИД регулированием
  float pid_linear = m_linear_velocity_pid.Step(smooth_cmd.linear_velocity,
                                                state.current_linear_speed, dt);
  float pid_angular = m_angle_velocity_pid.Step(
      smooth_cmd.angular_velocity, state.current_angular_speed, dt);

  // 3. МИКСЕР УСИЛИЙ (Вольты)
  // pid_linear: Вольты для разгона/торможения всего робота
  // pid_angular: Вольты для создания разницы скоростей между колесами
  float left_pid_volts = pid_linear - pid_angular;
  float right_pid_volts = pid_linear + pid_angular;

  // 4. СУММИРОВАНИЕ УСИЛИЙ (FF + PID)
  ControlEffort total_effort;
  total_effort.left_motor_voltage =
      ff_effort.left_motor_voltage + left_pid_volts;
  total_effort.right_motor_voltage =
      ff_effort.right_motor_voltage + right_pid_volts;

  // 5. ПРИМЕНЕНИЕ ФИЗИЧЕСКИХ ЛИМИТОВ (Proportional Desaturation)
  ActuatorLimits &limits = m_robot.GetLimits();
  SaturatedEffort safe_effort = limits.ApplyLimits(total_effort);

  // 6. ТРЕКИНГОВЫЙ ANTI-WINDUP (Tracking Back-Calculation)
  if (safe_effort.is_saturated) {
    // Положительная разница: желаемое - реальное
    float delta_left_v =
        total_effort.left_motor_voltage - safe_effort.effort.left_motor_voltage;
    float delta_right_v = total_effort.right_motor_voltage -
                          safe_effort.effort.right_motor_voltage;

    // Обратная кинематика для разницы напряжений
    float delta_linear_effort = (delta_left_v + delta_right_v) / 2.0f;
    float delta_angular_effort = (delta_right_v - delta_left_v) / 2.0f;

    // ПИДы получают ПОЛОЖИТЕЛЬНУЮ обратную связь для коррекции интеграла
    m_linear_velocity_pid.ApplyBackCalculation(delta_linear_effort, dt);
    m_angle_velocity_pid.ApplyBackCalculation(delta_angular_effort, dt);
  }

  m_last_safe_effort = safe_effort.effort;
  return safe_effort.effort;
}

MotionCommand RobotController::VelocityRamp(const MotionCommand &target_cmd,
                                            float dt) {
  MotionCommand smooth_cmd;

  // --- Обработка линейной скорости ---
  float linear_delta = target_cmd.linear_velocity - m_smooth_linear_cmd;
  float linear_step = m_max_linear_accel * dt;

  // Ограничиваем изменение скорости за один такт (dt)
  if (std::abs(linear_delta) <= linear_step) {
    m_smooth_linear_cmd = target_cmd.linear_velocity; // Достигли цели
  } else {
    // Делаем шаг в сторону цели
    m_smooth_linear_cmd += (linear_delta > 0.0f ? linear_step : -linear_step);
  }
  smooth_cmd.linear_velocity = m_smooth_linear_cmd;

  // --- Обработка угловой скорости ---
  float angular_delta = target_cmd.angular_velocity - m_smooth_angular_cmd;
  float angular_step = m_max_angular_accel * dt;

  if (std::abs(angular_delta) <= angular_step) {
    m_smooth_angular_cmd = target_cmd.angular_velocity;
  } else {
    m_smooth_angular_cmd +=
        (angular_delta > 0.0f ? angular_step : -angular_step);
  }
  smooth_cmd.angular_velocity = m_smooth_angular_cmd;

  LOG_INFO("RobotController: Smooth CMD linear velocity: %.3f m/s",
           smooth_cmd.linear_velocity);
  LOG_INFO("RobotController: Smooth CMD angular velocity: %.3f r/s",
           smooth_cmd.angular_velocity);
  return smooth_cmd;
}
} // namespace RobotControl

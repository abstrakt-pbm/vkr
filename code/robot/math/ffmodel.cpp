#include <ffmodel.hpp>

#include <robot/robot.hpp>
#include <robot/robot_control.hpp>

#include <logger.hpp>

#include <algorithm>

namespace RobotControl {

FFModel::FFModel(float base_width, float wheel_radius, float kS, float kV,
                 float max_voltage)
    : m_base_width(base_width), m_wheel_radius(wheel_radius), m_kS(kS),
      m_kV(kV), m_max_voltage(max_voltage) {}

float FFModel::CalculateMotorVoltage(float target_omega) const {
  // Deadband: если целевая скорость нулевая, напряжение 0 (робот стоит)
  if (std::abs(target_omega) < 1e-3f) {
    return 0.0f;
  }

  // Steady-state Voltage Balance: V = kS * sgn(w) + kV * w
  float voltage =
      (m_kS * std::copysign(1.0f, target_omega)) + (m_kV * target_omega);

  return std::clamp(voltage, -m_max_voltage, m_max_voltage);
}

Robot::ControlEffort FFModel::GetControlEffort(const MotionCommand &cmd) const {
  // 1. Кинематика дифф-привода: расчет линейных скоростей колес (м/с)
  float v_left =
      cmd.linear_velocity - (cmd.angular_velocity * m_base_width / 2.0f);
  float v_right =
      cmd.linear_velocity + (cmd.angular_velocity * m_base_width / 2.0f);

  // 2. Перевод в угловые скорости колес (рад/с)
  float omega_left = v_left / m_wheel_radius;
  float omega_right = v_right / m_wheel_radius;

  // 3. Расчет требуемого напряжения для каждого мотора
  float voltage_left = CalculateMotorVoltage(omega_left);
  float voltage_right = CalculateMotorVoltage(omega_right);

  LOG_INFO("FFModel: ang_velocity_left = %.3f ang_velocity_right = %.3f",
           omega_left, omega_right);

  LOG_INFO("FFModel: voltage_left = %.3f voltage_right = %.3f", voltage_left,
           voltage_right);

  // 4. Возврат результата
  return Robot::ControlEffort{voltage_left, voltage_right};
}

} // namespace RobotControl

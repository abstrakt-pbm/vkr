#include <webots_sil/webots_motor.hpp>

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <utility>

namespace {

constexpr float kMaxVoltage = 12.0f;
constexpr float kMinVoltageToStart = 0.06f;

constexpr float kWheelRadius = 0.025f;
constexpr float kMaxLinearWheelVelocity = 1.2f;

// omega_max = v_max / r = 1.2 / 0.025 = 48 рад/с
constexpr float kMaxAngularWheelVelocity =
    kMaxLinearWheelVelocity / kWheelRadius;

} // namespace

WebotsIMotorHAL::WebotsIMotorHAL(std::shared_ptr<webots::Robot> robot,
                                 const std::string &motor_name)
    : m_motor(nullptr), m_robot(std::move(robot)), m_time_step(0.0),
      m_current_voltage(0.0f) {
  std::cout << "[M] Init " << motor_name << '\n';

  if (!m_robot) {
    std::cerr << "[M] Robot is null\n";
    return;
  }

  m_time_step = m_robot->getBasicTimeStep();

  m_motor = m_robot->getMotor(motor_name);

  if (!m_motor) {
    std::cerr << "[M] Motor not found: " << motor_name << '\n';
    return;
  }

  std::cout << "[M] Motor initialized: " << motor_name
            << ", maxTorque=" << m_motor->getMaxTorque()
            << ", maxVelocity=" << m_motor->getMaxVelocity() << '\n';

  /*
   * Переключение RotationalMotor в режим непрерывного
   * управления скоростью.
   */
  m_motor->setPosition(std::numeric_limits<double>::infinity());

  m_motor->setVelocity(0.0);

  /*
   * Используем максимальный момент, заданный
   * в узле RotationalMotor файла .wbt.
   */
  const double max_torque = m_motor->getMaxTorque();

  if (std::isfinite(max_torque) && max_torque > 0.0) {
    m_motor->setAvailableTorque(max_torque);
  }
}

bool WebotsIMotorHAL::SetRawVoltage(float voltage) {
  if (!IsAlive() || !std::isfinite(voltage)) {
    return false;
  }

  /*
   * Дополнительная защита HAL.
   * Основное ограничение уже может выполняться
   * в ActuatorLimits.
   */
  const float applied_voltage =
      std::clamp(voltage, -GetMaxVoltage(), GetMaxVoltage());

  m_current_voltage = applied_voltage;

  /*
   * Простая линейная эмуляция:
   *
   * voltage = 0 В   -> velocity = 0 рад/с
   * voltage = 12 В  -> velocity = 48 рад/с
   * voltage = -12 В -> velocity = -48 рад/с
   */
  const float voltage_ratio = applied_voltage / GetMaxVoltage();

  float target_velocity = voltage_ratio * GetMaxVelocity();

  /*
   * Учитываем ограничение maxVelocity,
   * установленное в узле Webots.
   */
  const double webots_max_velocity = m_motor->getMaxVelocity();

  if (std::isfinite(webots_max_velocity) && webots_max_velocity > 0.0) {
    const float velocity_limit = static_cast<float>(webots_max_velocity);

    target_velocity =
        std::clamp(target_velocity, -velocity_limit, velocity_limit);
  }

  m_motor->setVelocity(static_cast<double>(target_velocity));

#ifdef WEBOTS_MOTOR_DEBUG
  std::cout << "[M] " << std::fixed << std::setprecision(3) << applied_voltage
            << " V -> target " << target_velocity << " rad/s\n";
#endif

  return true;
}

float WebotsIMotorHAL::GetCurrentRawVoltage() const {
  return m_current_voltage;
}

float WebotsIMotorHAL::GetMinVoltageToStart() const {
  return kMinVoltageToStart;
}

float WebotsIMotorHAL::GetMaxVoltage() const { return kMaxVoltage; }

float WebotsIMotorHAL::GetMaxVelocity() const {
  return kMaxAngularWheelVelocity;
}

bool WebotsIMotorHAL::IsAlive() const {
  return m_robot != nullptr && m_motor != nullptr;
}

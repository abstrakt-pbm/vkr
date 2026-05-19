#pragma once

#include <cstdint>

#include <robot_hil/protocol.hpp>
#include <robot_hil/transport.hpp>

namespace robot_hil {

struct InputState {
  std::uint16_t seq = 0;

  std::uint32_t time_us = 0;
  std::uint32_t dt_us = 0;

  std::uint8_t valid_mask = 0;

  std::int64_t left_encoder_ticks = 0;
  std::int64_t right_encoder_ticks = 0;

  float imu_yaw_rad = 0.0f;
  float imu_omega_z_rad_s = 0.0f;

  float target_v_mps = 0.0f;
  float target_w_rad_s = 0.0f;

  bool valid = false;
};

struct OutputState {
  std::uint16_t seq = 0;

  float left_motor_voltage = 0.0f;
  float right_motor_voltage = 0.0f;

  float debug_v_mps = 0.0f;
  float debug_w_rad_s = 0.0f;

  bool left_motor_updated = false;
  bool right_motor_updated = false;

  bool alive = false;
  bool controller_fault = false;
};

class Session {
public:
  explicit Session(ITransport &transport);

  bool ReceiveInput(std::uint32_t timeout_ms);
  bool SendOutput(std::uint32_t timeout_ms);

  const InputState &Input() const;
  const OutputState &Output() const;

  bool IsInputValid() const;

  float GetDtSeconds() const;

  void SetMotorVoltage(MotorId motor_id, float voltage);

  float GetMotorVoltage(MotorId motor_id) const;

  void SetDebugVelocity(float v_mps, float w_rad_s);

  void SetControllerFault();

private:
  void Invalidate();
  void ResetOutputForFrame(std::uint16_t seq);

  ControlFrame BuildControlFrame() const;

private:
  ITransport &m_transport;

  InputState m_input{};
  OutputState m_output{};
};

} // namespace robot_hil

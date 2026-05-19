#include <robot_hil/session.hpp>

namespace robot_hil {

Session::Session(ITransport &transport) : m_transport(transport) {}

bool Session::ReceiveInput(std::uint32_t timeout_ms) {
  SensorFrame frame{};

  if (!m_transport.ReceiveSensorFrame(frame, timeout_ms)) {
    Invalidate();
    return false;
  }

  if (frame.magic != kSensorMagic) {
    Invalidate();
    return false;
  }

  if (frame.version != kProtocolVersion) {
    Invalidate();
    return false;
  }

  if (!VerifyPacketCrc(frame)) {
    Invalidate();
    return false;
  }

  m_input.seq = frame.seq;
  m_input.time_us = frame.time_us;
  m_input.dt_us = frame.dt_us;
  m_input.valid_mask = frame.valid_mask;

  m_input.left_encoder_ticks = frame.left_encoder_ticks;
  m_input.right_encoder_ticks = frame.right_encoder_ticks;

  m_input.imu_yaw_rad = frame.imu_yaw_rad;
  m_input.imu_omega_z_rad_s = frame.imu_omega_z_rad_s;

  m_input.target_v_mps = frame.target_v_mps;
  m_input.target_w_rad_s = frame.target_w_rad_s;

  m_input.valid = true;

  ResetOutputForFrame(frame.seq);

  return true;
}

bool Session::SendOutput(std::uint32_t timeout_ms) {
  if (!m_input.valid || !m_output.alive) {
    return false;
  }

  ControlFrame frame = BuildControlFrame();

  FillPacketCrc(frame);

  return m_transport.SendControlFrame(frame, timeout_ms);
}

const InputState &Session::Input() const { return m_input; }

const OutputState &Session::Output() const { return m_output; }

bool Session::IsInputValid() const { return m_input.valid; }

float Session::GetDtSeconds() const {
  return static_cast<float>(m_input.dt_us) * 1.0e-6f;
}

void Session::SetMotorVoltage(MotorId motor_id, float voltage) {
  if (motor_id == MotorId::Left) {
    m_output.left_motor_voltage = voltage;
    m_output.left_motor_updated = true;
    return;
  }

  m_output.right_motor_voltage = voltage;
  m_output.right_motor_updated = true;
}

float Session::GetMotorVoltage(MotorId motor_id) const {
  if (motor_id == MotorId::Left) {
    return m_output.left_motor_voltage;
  }

  return m_output.right_motor_voltage;
}

void Session::SetDebugVelocity(float v_mps, float w_rad_s) {
  m_output.debug_v_mps = v_mps;
  m_output.debug_w_rad_s = w_rad_s;
}

void Session::SetControllerFault() { m_output.controller_fault = true; }

void Session::Invalidate() {
  m_input.valid = false;
  m_output.alive = false;
}

void Session::ResetOutputForFrame(std::uint16_t seq) {
  m_output.seq = seq;

  m_output.left_motor_voltage = 0.0f;
  m_output.right_motor_voltage = 0.0f;

  m_output.debug_v_mps = 0.0f;
  m_output.debug_w_rad_s = 0.0f;

  m_output.left_motor_updated = false;
  m_output.right_motor_updated = false;

  m_output.controller_fault = false;
  m_output.alive = true;
}

ControlFrame Session::BuildControlFrame() const {
  ControlFrame frame{};

  frame.magic = kControlMagic;
  frame.version = kProtocolVersion;
  frame.seq = m_output.seq;

  frame.left_motor_voltage = m_output.left_motor_voltage;
  frame.right_motor_voltage = m_output.right_motor_voltage;

  frame.debug_v_mps = m_output.debug_v_mps;
  frame.debug_w_rad_s = m_output.debug_w_rad_s;

  frame.status = kControlOk;

  if (!m_output.left_motor_updated) {
    frame.status |= kControlLeftMotorMissing;
  }

  if (!m_output.right_motor_updated) {
    frame.status |= kControlRightMotorMissing;
  }

  if (m_input.dt_us == 0u) {
    frame.status |= kControlInvalidDt;
  }

  if (m_output.controller_fault) {
    frame.status |= kControlControllerFault;
  }

  return frame;
}

} // namespace robot_hil

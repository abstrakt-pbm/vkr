#include <robot/encoder.hpp>

#include <hal/hal_encoder.hpp>

#include <cstdio>

#include <logger.hpp>

namespace {
constexpr float kAlpha = 1.0f;
} // namespace

namespace Robot {

Encoder::Encoder(HAL::IEncoderHAL &encoder_hal)
    : m_filtered_velocity_latest(0.0f), m_encoder_hal(encoder_hal) {}

void Encoder::UpdateState() {
  if (m_encoder_hal.IsAlive()) {
    float raw_current_veloctiy = m_encoder_hal.GetRawLinearVelocity();
    m_filtered_velocity_latest = (1.0f - kAlpha) * m_filtered_velocity_latest +
                                 kAlpha * raw_current_veloctiy;

    LOG_INFO("Encoder: raw=%.3f filtered_new=%.3f kAlpha=%.2f",
             raw_current_veloctiy, m_filtered_velocity_latest, kAlpha);

  } else {
    m_filtered_velocity_latest = 0.0f;
  }
}

float Encoder::GetCurrentVelocity() const {
  LOG_INFO("Encoder: current velocity %.3f", m_filtered_velocity_latest);
  return m_filtered_velocity_latest;
}

bool Encoder::IsAlive() const { return m_encoder_hal.IsAlive(); }

} // namespace Robot

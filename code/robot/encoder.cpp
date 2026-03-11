#include <robot/encoder.hpp>

#include <hal/hal_encoder.hpp>

#include <cstdio>

namespace {
	constexpr float kAlpha = 0.05f;
} //

namespace Robot {

Encoder::Encoder(HAL::IEncoderHAL &encoder_hal)
	: m_filtered_velocity_latest(0.0f),
	m_encoder_hal(encoder_hal) {}

void Encoder::UpdateState() {
	if (m_encoder_hal.IsAlive()) {
		float raw_current_veloctiy = m_encoder_hal.GetRawLinearVelocity();
			printf("[FILTER] raw=%.3f filtered_new=%.3f kAlpha=%.2f\n", 
         raw_current_veloctiy, (1-kAlpha)*m_filtered_velocity_latest + kAlpha*raw_current_veloctiy, kAlpha);
    	m_filtered_velocity_latest = (1.0f - kAlpha) * m_filtered_velocity_latest + kAlpha * raw_current_veloctiy;
	} else {
		m_filtered_velocity_latest = 0.0f;
	}
}

float Encoder::GetCurrentVelocity() const {
	return m_filtered_velocity_latest;
}

bool Encoder::IsAlive() const {
	return m_encoder_hal.IsAlive();
}

} // namespace Robot


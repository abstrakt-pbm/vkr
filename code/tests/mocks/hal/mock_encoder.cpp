#include "mock_encoder.hpp"

namespace HAL {

float MockIEncoderHAL::GetRawLinearVelocity() const {
	return m_raw_linear_velocity;
}

bool MockIEncoderHAL::IsAlive() const {
	return m_is_alive;
}

void MockIEncoderHAL::SetIsAlive(bool isAlive) {
	m_is_alive = isAlive;
}
void MockIEncoderHAL::SetRawVelocity(float velocity) {
	m_raw_linear_velocity = velocity;
}
	
} // namespace HAL


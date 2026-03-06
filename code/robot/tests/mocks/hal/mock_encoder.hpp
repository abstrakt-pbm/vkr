#pragma once

#include <hal/hal_encoder.hpp>

namespace HAL {

class MockIEncoderHAL : public IEncoderHAL {
public:
	float GetRawLinearVelocity() const override;
	bool IsAlive() const override;

	// Для тестов
	void SetIsAlive(bool isAlive);
	void SetRawVelocity(float velocity);

private:
	bool m_is_alive;
	float m_raw_linear_velocity;
};

} // namespace HAL


#pragma once

namespace HAL {
	class IEncoderHAL;
} // namespace HAL

namespace Robot {
class Encoder {
public:
	Encoder(HAL::IEncoderHAL &encoder_hal);
	// Возвращает скорость отфильтрованую черезь фильтр низких частот
	float GetCurrentVelocity();
	bool IsAlive();
private:
	HAL::IEncoderHAL &m_encoder_hal;

	bool m_is_alive;
};
} // namespace Robot


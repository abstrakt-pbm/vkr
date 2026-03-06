#pragma once

namespace HAL {
	class IEncoderHAL;
} // namespace HAL

namespace Robot {
class Encoder {
public:
	Encoder(HAL::IEncoderHAL &encoder_hal);
	// Обновление состояния энкодера,
	// получает с HAL сырую скорость и прогоняет через фильтр низких частот
	void UpdateState();
	// Возвращает скорость отфильтрованую черезь фильтр низких частот
	float GetCurrentVelocity() const;
	// Проверка на роботоспособность энкодера
	bool IsAlive() const;
private:
	float m_filtered_velocity_latest;

	HAL::IEncoderHAL &m_encoder_hal;
};
} // namespace Robot


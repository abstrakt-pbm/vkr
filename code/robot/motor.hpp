#pragma once

namespace HAL {
	class IMotorHAL;
}

namespace Robot{
class Motor {
public:
	Motor(HAL::IMotorHAL &motor_hal,
	   float ramp_coeff,
	   float max_voltage,
	   float voltage_to_start);
	// Безопасная функция, внутри обеспечивает рампообразное снижение/повышение напряжения за указаный промежуток времени dt
	void SetVoltage(float target_voltage, float dt);
	// Опасная функция, использовать аккуратно,
	// если разница между текущим напряжением и конечным высока, может убить мотор 
	void SetRawVoltage(float voltage);
	// Возвращает текущий установленный на моторе вольтаж
	float GetCurrentVoltage();
	// Проверка на вменяемость
	bool IsAlive();
	float GetMaxVoltage() const;
private:
	float m_current_voltage;
	float m_ramp_coeff;
	float m_max_voltage;
	float m_min_voltage_to_start;

	HAL::IMotorHAL &m_motor_hal;
};
} // namespace Robot


#include <pid.hpp>

#include <cmath>
#include <algorithm>
#include <cstdio>

namespace {
	constexpr float kAlpha = 0.2f;
} //

namespace Math {
PID::PID(float kp, float ki, float kd, float k_back, float output_limit)
	:
	m_kp(kp),
	m_ki(ki),
	m_kd(kd),
	m_D_filtered(0),
	m_k_back(k_back),
	m_integrator(0),
	m_last_error(0),
	m_last_measurement(0),
	m_first_run(true),
	m_output_limit(output_limit) {}

float PID::Step(float setpoint, float measurement, float dt) {
	if (dt <= 1e-6f || !std::isfinite(setpoint) || !std::isfinite(measurement)) {
        return 0.0f;
    }

    float error = setpoint - measurement;

    // DEADZONE против шума энкодеров (±0.05 рад/с = ±3°/с)
    constexpr float DEADZONE = 0.08f;
    if (std::abs(error) < DEADZONE) {
        error = 0.0f;  // Не реагируем на шум!
    }

    // 2. Инициализация при первом запуске (избегаем рывка D-компоненты)
    if (m_first_run) { 
        m_last_measurement = measurement;
        m_first_run = false;
    }

    // Proportional
    m_p_term = m_kp * error;

    // Integral (Используем метод трапеций для большей точности)
    m_integrator += m_ki * error * dt;

    // Derivative (От измерения, а не от ошибки для защиты от Derivative Kick)
    float delta_measurement = measurement - m_last_measurement;
    float d_raw = -m_kd * (delta_measurement) / dt;
    
    // Low-Pass фильтр для D-компоненты 
    // m_alpha от 0.0 до 1.0 (чем меньше значение, тем сильнее сглаживание шума энкодера)
    m_d_term = kAlpha * d_raw + (1.0f - kAlpha) * m_d_term;

    float raw_output = m_p_term + m_integrator + m_d_term;
    float output = std::clamp(raw_output, -m_output_limit, m_output_limit);

    m_last_measurement = measurement;

	printf("[PID] SP:%7.2f | PV:%7.2f | Err:%7.2f || P:%7.2f | I:%7.2f | D:%7.2f || Out:%7.2f\n", 
           setpoint, measurement, error, 
           m_p_term, m_integrator, m_d_term, 
           output);

    return output;
}

void PID::ApplyBackCalculation(float delta_effort, float dt) {
    // delta_effort = (Желаемое усилие - Примененное усилие)
    // Если мы "передали" лишнего, нужно уменьшить интегратор
	m_integrator -= m_k_back * delta_effort * dt;
}

void PID::Reset() {
	m_integrator = 0.0f;
	m_last_error = 0.0f;
	m_last_measurement = 0.0f;
	m_first_run = true;
}

void PID::SetPIDCoef(float kp, float ki, float kd) {
	m_kp = kp;
	m_ki = ki;
	m_kd = kd;
}

float PID::GetIntegrator() const{
	return m_integrator;
}

float PID::GetP() const{
	return m_kp;
}

float PID::GetI() const{
	return m_ki;
}

float PID::GetD() const{
	return m_kd;
}

float PID::GetPTerm() const {
	return m_p_term;
}

float PID::GetITerm() const {
	return m_integrator;
}

float PID::GetDTerm() const {
	return m_d_term;
}

} // namespace Math


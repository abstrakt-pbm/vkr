#include <pid.hpp>

#include <cmath>
#include <algorithm>

namespace {
	constexpr float M_SATTURATION_ERROR = 1e-4f;
}

namespace Math {
PID::PID(float kp, float ki, float kd, float k_back, float output_limit)
: m_kp(kp), m_ki(ki), m_kd(kd), m_k_back(k_back), m_output_limit(output_limit) {}

float PID::Step(float error, float dt) {
	float P = m_kp * error;
	m_integrator += m_ki * error * dt; 
	float D = m_kd * (error - m_last_error) / dt;

	float raw_output = P + m_integrator + D;
    
    // ВАЖНО: Если мы используем внешний Back-Calculation из контроллера, 
    // внутренний clamp() и внутренний ApplyBackCalculation нужно отключить.
    // Если ты хочешь оставить внутренний лимит как "предохранитель" - можно, 
    // но внутренний Back-Calculation точно нужно убрать.
    
	float output = std::clamp(raw_output, -m_output_limit, m_output_limit);
    
    // Внутренний anti-windup удален!

	m_last_error = error;
	return output; // Можно возвращать raw_output, если m_output_limit задан как бесконечность
}

void PID::ApplyBackCalculation(float delta_effort, float dt) {
    // delta_effort = (Желаемое усилие - Примененное усилие)
    // Если мы "передали" лишнего, нужно уменьшить интегратор
	m_integrator -= m_k_back * delta_effort * dt;
}

void PID::Reset() {
	m_integrator = 0.0f;
	m_last_error = 0.0f;
}

void PID::SetPIDCoef(float kp, float ki, float kd) {
	m_kp = kp;
	m_ki = ki;
	m_kd = kd;
}

float PID::GetIntegrator() {
	return m_integrator;
}

float PID::GetP() {
	return m_kp;
}

float PID::GetI() {
	return m_ki;
}

float PID::GetD() {
	return m_kd;
}

} // namespace Math


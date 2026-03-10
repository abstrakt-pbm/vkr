#pragma once

namespace Math {

class PID{
public:
	PID(float kp,
	 float ki,
	 float kd,
	 float k_back,
	 float output_limit);

	float Step(float error, float dt);
	void ApplyBackCalculation(float delta_linear_effort, float dt);
	void Reset();
	void SetPIDCoef(float kp, float ki, float kd);
private:
	float m_kp;
	float m_ki;
	float m_kd;
	float m_k_back;

	float m_integrator;
	float m_last_error;

	float m_output_limit = 3.0f;
};
} // namespace Math


#pragma once

namespace Math {

class PID{
public:
	PID(float kp,
	 float ki,
	 float kd,
	 float k_back,
	 float output_limit);

	float Step(float setpoint, float measurement, float dt);
	void ApplyBackCalculation(float delta_linear_effort, float dt);
	void Reset();
	void SetPIDCoef(float kp, float ki, float kd);

	float GetIntegrator() const;
	float GetP() const;
	float GetI() const;
	float GetD() const;
	float GetPTerm() const;
	float GetITerm() const;
	float GetDTerm() const;
private:
	float m_kp;
	float m_ki;
	float m_kd;
	float m_D_filtered;
	float m_k_back;

	float m_p_term = 0.0f;
    float m_d_term = 0.0f;
	float m_integrator;
	float m_last_error;
	float m_last_measurement;
	bool m_first_run;

	float m_output_limit = 3.0f;
};
} // namespace Math


#pragma once

namespace Math {

class PID{
public:
	float step(float error, float dt);
	void apply_back_calculation(float delta_linear_effort, float dt);
	void reset();
};


} // namespace Math


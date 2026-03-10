#pragma once

namespace Robot {

class Odometry;

class Position {
public:
	float GetX() const;
	float GetY() const;
	float GetAngle() const;
	float GetNormalizedAngle() const;

private:
	float m_current_x = 0;
	float m_current_y = 0;

	float m_current_angle = 0;

	friend class Odometry; 
};

class Odometry {
public:
	// Recalculate m_current_possition
	void Update(float linear_velocity,
			 float angle_velocity, float dt); 

	// Return's copy of m_current_possition
	Position GetCurrentPosition() const;
private:
	Position m_current_possition;
};

}


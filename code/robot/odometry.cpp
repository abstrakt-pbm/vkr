#include <robot/odometry.hpp>

#include <cmath>

namespace Robot {
float Position::GetX() const {
	return m_current_x;
}

float Position::GetY() const {
	return m_current_y;
}

float Position::GetAngle() const {
	return m_current_angle;
}

float Position::GetNormalizedAngle() const {
    float angle = fmod(m_current_angle, 2.0f * M_PI);
    return (angle > M_PI) ? angle - 2.0f * M_PI : angle;
}

void Odometry::Update(float linear_velocity,
					  float angle_velocity,
					  float dt) {
	m_current_possition.m_current_x += linear_velocity * dt * std::cos(m_current_possition.m_current_angle);
    m_current_possition.m_current_y += linear_velocity * dt * std::sin(m_current_possition.m_current_angle);
    m_current_possition.m_current_angle += angle_velocity * dt;
}

Position Odometry::GetCurrentPosition() const {
	// Return copy of Possition by RVO
	return m_current_possition;
}


} // namespace Robot


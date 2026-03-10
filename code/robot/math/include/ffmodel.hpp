#pragma once

namespace Robot {
	class ControlEffort;
}

namespace RobotControl {
class MotionCommand;

class FFModel {
public:
    FFModel(float base_width, float wheel_radius, float kS, float kV, float max_voltage = 12.0f);

    Robot::ControlEffort GetControlEffort(const MotionCommand& cmd) const;

private:
    float m_base_width;
    float m_wheel_radius;
    
    // Параметры модели DC-мотора
    float m_kS; // Напряжение статического трения (Вольт)
    float m_kV; // Коэффициент противоЭДС: Вольт / (рад/с)
    
    float m_max_voltage;
    
    float CalculateMotorVoltage(float target_omega) const;
};

} // namespace RobotControl


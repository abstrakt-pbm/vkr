#include <webots_sil/webots_motor.hpp>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#include <iostream>
#include <iomanip>

WebotsIMotorHAL::WebotsIMotorHAL(std::shared_ptr<webots::Robot> robot, 
                                 const std::string& motor_name)
    : m_motor(nullptr), m_robot(std::move(robot)), m_time_step(0.0), m_current_voltage(0.0f) {
    
    std::cout << "[M] Init " << motor_name << std::endl;
    
    if (m_robot) {
        m_time_step = m_robot->getBasicTimeStep();
        m_motor = m_robot->getMotor(motor_name);
        if (m_motor) {
            std::cout << "[M✓] " << motor_name << " torque=" << m_motor->getMaxTorque() << std::endl;
            m_motor->setPosition(INFINITY);
            m_motor->setVelocity(0);
            m_motor->setAvailableTorque(m_motor->getMaxTorque());
        } else {
            std::cout << "[M✗] " << motor_name << std::endl;
        }
    }
}

bool WebotsIMotorHAL::SetRawVoltage(float voltage) {
    if (!IsAlive()) {
        std::cout << "[M] DEAD " << voltage << std::endl;
        return false;
    }
    
    m_current_voltage = voltage;
    
    double max_v = GetMaxVoltage();
    double max_t = m_motor->getMaxTorque();
    
    // Ваша оригинальная логика БЕЗ clamp
    double target_torque = (voltage / max_v) * max_t;
    m_motor->setTorque(target_torque);
    
    std::cout << "[M] " << std::fixed << std::setprecision(1) 
              << voltage << "V → " << target_torque << "Nm" << std::endl;
    
    return true;
}

float WebotsIMotorHAL::GetCurrentRawVoltage() const { return m_current_voltage; }
float WebotsIMotorHAL::GetMinVoltageToStart() const { return 0.5f; }
float WebotsIMotorHAL::GetMaxVoltage() const { return 12.0f; }
bool WebotsIMotorHAL::IsAlive() const { return m_robot && m_motor; }


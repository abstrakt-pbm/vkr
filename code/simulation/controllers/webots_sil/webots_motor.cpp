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

    		m_motor->setControlPID(0.0f, 0.0f, 0.0f);

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
    
    float max_v = GetMaxVoltage();
    float max_vel = GetMaxVelocity();  // Нужно определить максимальную скорость
    
    // Переход с torque на velocity
    float velocity_ratio = voltage / max_v;
    float target_velocity = velocity_ratio * max_vel;
    
    m_motor->setVelocity(target_velocity);
    
    std::cout << "[M] " << std::fixed << std::setprecision(1) 
              << voltage << "V → " << target_velocity << "rad/s" << std::endl;
    
    return true;
}

float WebotsIMotorHAL::GetCurrentRawVoltage() const { return m_current_voltage; }
float WebotsIMotorHAL::GetMinVoltageToStart() const { return 0.5f; }
float WebotsIMotorHAL::GetMaxVoltage() const { return 12.0f; }


// Добавить метод для максимальной скорости мотора
float  WebotsIMotorHAL::GetMaxVelocity() const {
    // Обычно для DC моторов в Webots ~6.28 рад/с (2 об/с) или по спецификации вашего мотора
    // Можно получить из .proto файла мотора или hardcode
    return 1.2 / 0.025;  // 2 об/с = 2 * 2π рад/с 
}

bool WebotsIMotorHAL::IsAlive() const { return m_robot && m_motor; }

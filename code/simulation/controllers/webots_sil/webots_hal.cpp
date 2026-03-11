#include "webots_hal.hpp"

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>

#include <cmath>

WebotsHAL::WebotsHAL() {
    // Инициализация робота
    m_webots_robot = std::make_unique<webots::Robot>();
    int time_step = static_cast<int>(m_webots_robot->getBasicTimeStep());

    // Захват моторов
    left_motor_ = m_webots_robot->getMotor("left_motor");
    right_motor_ = m_webots_robot->getMotor("right_motor");
    
    // Настройка моторов для скорости (не позиции)
    left_motor_->setPosition(INFINITY);
    right_motor_->setPosition(INFINITY);
    left_motor_->setVelocity(0.0);
    right_motor_->setVelocity(0.0);

    // Захват энкодеров
    left_encoder_ = m_webots_robot->getPositionSensor("left_sensor");
    right_encoder_ = m_webots_robot->getPositionSensor("right_sensor");
    left_encoder_->enable(time_step);
    right_encoder_->enable(time_step);

    // Захват гироскопа
    gyro_ = m_webots_robot->getGyro("gyro");
    if (gyro_) {
        gyro_->enable(time_step);
    }
}

bool WebotsHAL::step() {
    return m_webots_robot->step(32) != -1;  // 32мс = типичный timestep Webots
}

void WebotsHAL::set_motor_pwm(float left, float right) {
    // Конвертация PWM [-100..100] -> скорость [-max..max]
    double max_speed = left_motor_->getMaxVelocity();
    left_motor_->setVelocity((left / 100.0) * max_speed);
    right_motor_->setVelocity((right / 100.0) * max_speed);
}

void WebotsHAL::get_encoder_ticks(int32_t& left, int32_t& right) {
    // Webots: радианы -> тики энкодера (4000 тиков/оборот)
    constexpr double TICKS_PER_RAD = 4000.0 / (2.0 * M_PI);
    
    left = static_cast<int32_t>(left_encoder_->getValue() * TICKS_PER_RAD);
    right = static_cast<int32_t>(right_encoder_->getValue() * TICKS_PER_RAD);
}


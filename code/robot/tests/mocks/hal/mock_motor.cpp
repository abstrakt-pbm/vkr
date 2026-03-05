#include "mock_motor.hpp"

#include <algorithm>  // std::clamp
#include <cstdio>     // printf (debug)

namespace HAL {

bool MockIMotorHAL::SetRawVoltage(float voltage) {
    voltage_feedback_ = std::clamp(voltage, 0.0f, max_voltage_);
    printf("[MockHAL] SetRawVoltage(%.2fV)\n", voltage_feedback_);  // DEBUG
    return true;  // Всегда success для тестов
}

float MockIMotorHAL::GetCurrentRawVoltage() const {
    return voltage_feedback_;
}

float MockIMotorHAL::GetMinVoltageToStart() const {
    return min_start_;
}

float MockIMotorHAL::GetMaxVoltage() const {
    return max_voltage_;
}

bool MockIMotorHAL::IsAlive() const {
    return alive_status_;
}

} // namespace HAL


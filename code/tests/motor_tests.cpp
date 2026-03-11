#include <robot/robot.hpp>      // Motor класс
#include <robot/motor.hpp>      // Motor класс
#include <hal/hal_motor.hpp>    // IMotorHAL интерфейс
#include "mocks/hal/mock_motor.hpp"  // MockIMotorHAL

#include <gtest/gtest.h>

using namespace Robot;

class MotorTest : public ::testing::Test {
protected:
   	HAL::MockIMotorHAL mock_hal;

};

// ✅ ЕДИНСТВЕННЫЙ ТЕСТ: полный сценарий ramp + feedback
TEST_F(MotorTest, RampReachesTargetWithFeedback) {

    Motor motor(mock_hal, 2.0f /*ramp*/, 12.0f /*max*/, 0.0f /*start*/);
	mock_hal.SetRawVoltage(12.0f);
    EXPECT_NEAR(mock_hal.GetCurrentRawVoltage(), 12.0f, 0.01f);
    
    int brake_steps = 0;
    while (motor.GetCurrentVoltage() > 0.1f) {
        motor.SetVoltage(0.0f, 0.5f);  // dt=0.5s
        brake_steps++;
    }
    EXPECT_LE(motor.GetCurrentVoltage(), 0.1f);
    EXPECT_EQ(brake_steps, 12);  // 12V / 1V_step
    
    int accel_steps = 0;
    while (motor.GetCurrentVoltage() < 11.9f) {
        motor.SetVoltage(12.0f, 0.5f);
        accel_steps++;
    }
    EXPECT_GE(motor.GetCurrentVoltage(), 11.9f);
    EXPECT_EQ(accel_steps, 12);
    

}

TEST_F(MotorTest, RampUpFromZero) {
    Motor motor(mock_hal, 2.0f /*ramp*/, 12.0f /*max*/, 0.0f /*start*/);
    motor.SetVoltage(6.0f, 0.5f);  // 50% PWM
    for(int i=0; i<7; i++) motor.SetVoltage(6.0f, 0.5f);
    EXPECT_NEAR(motor.GetCurrentVoltage(), 6.0f, 0.1f);
}

TEST_F(MotorTest, RampDownFromMax) {
    Motor motor(mock_hal, 2.0f /*ramp*/, 12.0f /*max*/, 0.0f /*start*/);
    mock_hal.SetRawVoltage(12.0f);
    motor.SetVoltage(0.0f, 0.5f);
    for(int i=0; i<12; i++) motor.SetVoltage(0.0f, 0.5f);
    EXPECT_NEAR(motor.GetCurrentVoltage(), 0.0f, 0.1f);
}

TEST_F(MotorTest, RampAboveMax) {
    Motor motor(mock_hal, 2.0f /*ramp*/, 12.0f /*max*/, 0.0f /*start*/);
    motor.SetVoltage(20.0f, 0.5f);  // >12V
	for(int i = 0; i < 15; i++) {
        motor.SetVoltage(20.0f, 0.5f);
    }
    EXPECT_NEAR(motor.GetCurrentVoltage(), 12.0f, 0.2f);
}

TEST_F(MotorTest, RampBelowZero) {
    Motor motor(mock_hal, 2.0f /*ramp*/, 12.0f /*max*/, 0.0f /*start*/);
    mock_hal.SetRawVoltage(6.0f);
    motor.SetVoltage(-5.0f, 0.5f);
    for(int i=0; i<10; i++) motor.SetVoltage(-5.0f, 0.5f);
    EXPECT_NEAR(motor.GetCurrentVoltage(), 0.0f, 0.1f);
}

TEST_F(MotorTest, ZeroDtNoChange) {
    Motor motor(mock_hal, 2.0f /*ramp*/, 12.0f /*max*/, 0.0f /*start*/);
    float before = motor.GetCurrentVoltage();
    motor.SetVoltage(12.0f, 0.0f);  // dt=0
    motor.SetVoltage(12.0f, 0.0f);
    EXPECT_FLOAT_EQ(motor.GetCurrentVoltage(), before);  // Не меняется!
}


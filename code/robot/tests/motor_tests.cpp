#include <robot/robot.hpp>      // Motor класс
#include <robot/motor.hpp>      // Motor класс
#include <hal/hal_motor.hpp>    // IMotorHAL интерфейс
#include "mocks/hal/mock_motor.hpp"  // MockIMotorHAL

#include <gtest/gtest.h>

using ::testing::Return;
using ::testing::FloatNear;
using ::testing::AtLeast;
using ::testing::_;
using ::testing::Exactly;
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
    
    // 🚨 ТОРМОЖЕНИЕ: 12V → 0V
    int brake_steps = 0;
    while (motor.GetCurrentVoltage() > 0.1f) {
        motor.SetVoltage(0.0f, 0.5f);  // dt=0.5s
        brake_steps++;
        printf("[BRAKE] Step %d: %.1fV\n", brake_steps, mock_hal.GetCurrentRawVoltage());
    }
    EXPECT_LE(motor.GetCurrentVoltage(), 0.1f);
    EXPECT_EQ(brake_steps, 12);  // 12V / 1V_step
    
    // 🚀 РАЗГОН: 0V → 12V
    int accel_steps = 0;
    while (motor.GetCurrentVoltage() < 11.9f) {
        motor.SetVoltage(12.0f, 0.5f);
        accel_steps++;
        printf("[ACCEL] Step %d: %.1fV\n", accel_steps, mock_hal.GetCurrentRawVoltage());
    }
    EXPECT_GE(motor.GetCurrentVoltage(), 11.9f);
    EXPECT_EQ(accel_steps, 12);
    
    printf("✅ TORможение: %d шагов, РАЗГОН: %d шагов\n", brake_steps, accel_steps);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}

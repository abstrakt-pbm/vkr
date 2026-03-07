#include <robot/robot.hpp> // Motor класс
#include <robot/motor.hpp> // Motor класс
#include <robot/imu.hpp>
#include <robot/encoder.hpp>
#include <hal/hal_motor.hpp> // IMotorHAL интерфейс
#include <hal/hal_imu.hpp> // IMotorHAL интерфейс
#include <hal/hal_encoder.hpp> // IMotorHAL интерфейс
#include "mocks/hal/mock_motor.hpp" // MockIMotorHAL
#include "mocks/hal/mock_imu.hpp" // MockIMotorHAL
#include "mocks/hal/mock_encoder.hpp" // MockIMotorHAL

#include <gtest/gtest.h>

using namespace Robot;
using namespace HAL;

class RobotTest : public ::testing::Test {
protected:
    MockIImuHAL imu_hal;
    MockIEncoderHAL encoder_hal_l;
    MockIEncoderHAL encoder_hal_r;
    MockIMotorHAL motor_hal_l;
    MockIMotorHAL motor_hal_r;

	IMU imu{imu_hal};
    Encoder mock_l_enc{encoder_hal_l};
    Encoder mock_r_enc{encoder_hal_r};

	Motor mock_l_motor{motor_hal_l, 2.0f, 12.0f, 0.0f};
    Motor mock_r_motor{motor_hal_r, 2.0f, 12.0f, 0.0f};
    
	ActuatorLimits limits{};

	Robot::Robot robot{imu, 
		mock_l_motor, 
		mock_l_enc, 
		mock_r_motor, 
		mock_r_enc,
		limits};
    
    void SetUp() override {
        // Стандартное состояние
        encoder_hal_l.SetRawVelocity(0.3f);
        encoder_hal_r.SetRawVelocity(0.3f);
        imu_hal.SetRawGyroZ(0.0f);
    }
};

TEST_F(RobotTest, FetchCurrentRobotState_ReturnsFiltered) {
    // GIVEN: нормальные сенсоры
    encoder_hal_l.SetRawVelocity(0.30f);
    encoder_hal_r.SetRawVelocity(0.30f);
    // Опционально: можно задать IMU
	//imu_hal.SetIsAlive(true);
    imu_hal.SetRawGyroZ(0.0f);
    
    // WHEN: Даем фильтру время на "сходимость" (например 100 тиков = 1 секунда)
	robot.UpdateSensors();
	RobotState state;
   for(size_t i = 0; i < 100; ++i) {
        // Вызываем обновление робота. Внутри он должен прочитать 
        // данные с Mock-датчиков и прогнать их через EKF.Update()
		//
		robot.UpdateSensors();
        state = robot.FetchCurrentRobotState(0.01f); 
    }
    
    // THEN: После 100 итераций фильтр должен сойтись к реальному значению
    EXPECT_NEAR(state.current_linear_speed,  0.30f, 0.005f);     // (0.31+0.29)/2
    EXPECT_NEAR(state.current_angular_speed, 0.0f,  0.01f);      // (0.29-0.31) / base ≈ 0
    EXPECT_NEAR(state.left_motor_current_voltage,  0.0f, 0.1f);  // motors off (PID не вызывался)
    EXPECT_NEAR(state.right_motor_current_voltage, 0.0f, 0.1f);
}

/*
TEST_F(RobotTest, TransferToNewState_SafeLimitsApplied) {
    // GIVEN: экстремальное управление
    ControlEffort effort{15.0f, 15.0f};  // >12V
    
    // WHEN
    robot.TransferToNewState(effort, 0.01f);
    
    // THEN: limits применены
    EXPECT_LE(mock_l_motor.GetVoltage(), 12.0f);
    EXPECT_LE(mock_r_motor.GetVoltage(), 12.0f);
}

TEST_F(RobotTest, TransferToNewState_MotorDead_SafeMode) {
    // GIVEN: левый мотор мёртв
    mock_l_motor.SetAlive(false);
    
    ControlEffort effort{10.0f, 10.0f};
    
    // WHEN
    robot.TransferToNewState(effort, 0.01f);
    
    // THEN: safe stop
    EXPECT_EQ(mock_l_motor.GetVoltage(), 0.0f);
    EXPECT_EQ(mock_r_motor.GetVoltage(), 0.0f);
    EXPECT_TRUE(robot.m_is_in_safe_mode);
}

TEST_F(RobotTest, UpdateSensors_CallsAllHAL) {
    // GIVEN: mock ожидания
    EXPECT_CALL(mock_hal, UpdateEncoders()).Times(1);
    EXPECT_CALL(mock_hal, UpdateIMU()).Times(1);
    
    // WHEN
    robot.UpdateSensors();
    
    // THEN: все сенсоры обновлены
}

TEST_F(RobotTest, EnterSafeStopMode_ZeroVoltages) {
    // GIVEN: активные моторы
    mock_l_motor.SetVoltage(5.0f);
    mock_r_motor.SetVoltage(6.0f);
    
    // WHEN
    robot.enterSafeStopMode();
    
    // THEN: 0V + safe mode
    EXPECT_EQ(mock_l_motor.GetVoltage(), 0.0f);
    EXPECT_TRUE(robot.m_is_in_safe_mode);
}

TEST_F(RobotTest, GetLimits_ReturnsReference) {
    ActuatorLimits limits{12.0f, -12.0f};
    // mock_limits setup...
    
    EXPECT_EQ(&robot.GetLimits(), &m_limits);
}
*/

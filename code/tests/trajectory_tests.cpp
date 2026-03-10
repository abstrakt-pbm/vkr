#include <robot/robot.hpp>

#include <robot/motor.hpp>
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
using namespace RobotControl;
using namespace Math;

// НЕ наследуем RobotTest — копируем структуру!
class RobotTrajectoryTest : public ::testing::Test {
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
    RobotKinematics robot_kinematics{};

    Robot::Robot robot{imu, mock_l_motor, mock_l_enc, mock_r_motor, mock_r_enc, limits, robot_kinematics};
    
    constexpr static float DT_REAL = 0.001f;  // Твоя 1кГц
    constexpr static int STEPS_05M = 500;     // 0.5м / 1м/с / 1мс
    
    void SetUp() override {
        encoder_hal_l.SetRawVelocity(0.3f);
        encoder_hal_r.SetRawVelocity(0.3f);
        imu_hal.SetRawGyroZ(0.0f);
    }
    
    RobotState DriveStraight(int steps) {
        RobotState state;
        for(int i = 0; i < steps; i++) {
            encoder_hal_l.SetRawVelocity(0.5f);
            encoder_hal_r.SetRawVelocity(0.5f);
            imu_hal.SetRawGyroZ(0.0f);
            
            robot.UpdateSensors();
            state = robot.FetchCurrentRobotState(DT_REAL);
        }
        return state;
    }
};

TEST_F(RobotTrajectoryTest, HighFrequency_LPFConvergence) {
    // kAlpha=0.2 → 95% сходимости за ~5с (5000*1мс)
    for(int i = 0; i < 5000; i++) {
        encoder_hal_l.SetRawVelocity(0.3f);
        encoder_hal_r.SetRawVelocity(0.3f);
        imu_hal.SetRawGyroZ(0.0f);
        robot.UpdateSensors();
        robot.FetchCurrentRobotState(DT_REAL);
    }
    
    // Steady-state проверка
    RobotState state;
    float sum_v = 0.0f;
    for(int i = 0; i < 1000; i++) {
        encoder_hal_l.SetRawVelocity(0.3f);
        encoder_hal_r.SetRawVelocity(0.3f);
        imu_hal.SetRawGyroZ(0.0f);
        robot.UpdateSensors();
        state = robot.FetchCurrentRobotState(DT_REAL);
        
        sum_v += state.current_linear_speed;
    }
    float avg_v = sum_v / 1000;
    EXPECT_NEAR(avg_v, 0.3f, 0.12f);  // kAlpha=0.2 steady
}

TEST_F(RobotTrajectoryTest, SquareTrajectory_NoDrift) {
    constexpr float DT_REAL = 0.001f;
    constexpr int STEPS_SIDE = 500;     // 0.5м @1м/с
    constexpr int STEPS_TURN = 160;     // 0.16с
    constexpr float W_TURN = 9.8175f;   // рад/с = π/2 за 0.16с ✓
    
    RobotState state;
    
    // Квадрат 0.5×0.5м через EKF → Odometry
    for(int side = 0; side < 4; ++side) {
        // Прямой ход: encoders дают v=1м/с
        for(int i = 0; i < STEPS_SIDE; ++i) {
            encoder_hal_l.SetRawVelocity(1.0f);
            encoder_hal_r.SetRawVelocity(1.0f);
            imu_hal.SetRawGyroZ(0.0f);  // ω=0
            
            robot.UpdateSensors();
            state = robot.FetchCurrentRobotState(DT_REAL);
        }
        
        // Поворот 90°: gyro dominant
        for(int i = 0; i < STEPS_TURN; ++i) {
            encoder_hal_l.SetRawVelocity(0.0f);
            encoder_hal_r.SetRawVelocity(0.0f);
            imu_hal.SetRawGyroZ(W_TURN);  // ✓ 9.82 рад/с = 90°!
            
            robot.UpdateSensors();
            state = robot.FetchCurrentRobotState(DT_REAL);
        }
    }
    
    // Финальное состояние после квадрата
    EXPECT_NEAR(state.current_linear_speed,  0.0f, 0.05f);
    EXPECT_NEAR(state.current_angular_speed, 0.0f, 0.1f);
    
	
	Position position =robot.GetOdometry().GetCurrentPosition();
    // Pose замкнулся (одометрия + EKF)
    EXPECT_NEAR(position.GetX(), 0.0f, 0.05f);     // + НОВОЕ!
    EXPECT_NEAR(position.GetY(), 0.0f, 0.05f);     // + НОВОЕ!
    EXPECT_NEAR(position.GetNormalizedAngle(), 0.0f, 0.1f);  // + НОВОЕ! (normalized)
    
    // Моторы safe (нет команд)
    EXPECT_NEAR(state.left_motor_current_voltage,  0.0f, 0.1f);
    EXPECT_NEAR(state.right_motor_current_voltage, 0.0f, 0.1f);
}


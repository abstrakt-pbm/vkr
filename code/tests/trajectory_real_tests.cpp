#include <gtest/gtest.h>

#include <robot/robot.hpp>
#include <robot/robot_control.hpp>

#include <robot/motor.hpp>
#include <robot/imu.hpp>
#include <robot/encoder.hpp>
#include <hal/hal_motor.hpp>
#include <hal/hal_imu.hpp>
#include <hal/hal_encoder.hpp>
#include "mocks/hal/mock_motor.hpp"
#include "mocks/hal/mock_imu.hpp"
#include "mocks/hal/mock_encoder.hpp"

#include <pid.hpp>
#include <ffmodel.hpp>

using namespace Robot;
using namespace RobotControl;
using namespace HAL;
using namespace Math;

// Создаем mock классы прямо в тесте (пока mocks не готовы)

class RobotControllerTest : public ::testing::Test {
protected:
    MockIImuHAL imu_hal;
    MockIEncoderHAL encoder_hal_l;
    MockIEncoderHAL encoder_hal_r;
    MockIMotorHAL motor_hal_l;
    MockIMotorHAL motor_hal_r;

    IMU imu{imu_hal};
    Encoder enc_l{encoder_hal_l};
    Encoder enc_r{encoder_hal_r};
    Motor motor_l{motor_hal_l, 2.0f, 12.0f, 0.0f};
    Motor motor_r{motor_hal_r, 2.0f, 12.0f, 0.0f};

    ActuatorLimits limits{};
    RobotKinematics kinematics{};

    Robot::Robot robot{
        imu, motor_l, enc_l, motor_r, enc_r, limits, kinematics
    };

    // Инициализируем новую FFModel (base_width=0.3, wheel_radius=0.05, kS=1.0, kV=0.02)
    FFModel ff_model{0.3f, 0.05f, 1.0f, 0.02f, 12.0f};
    
	PID lin_pid{0.4f, 0.05f, 0.02f, 1.0f, 3.0f};  // P↓ I↑ D↑
	PID ang_pid{1.2f, 0.15f, 0.08f, 1.0f, 2.5f};

    RobotController controller{robot, ff_model, lin_pid, ang_pid};
   
    static constexpr float DT = 0.001f;
    static constexpr float POS_TOL = 0.08f;
    static constexpr float V_TOL = 0.08f;
    
    void SetUp() override {
        // Инициализация HAL в SetUp
        imu_hal.SetRawGyroZ(0.0f);
        encoder_hal_l.SetRawVelocity(0.0f);
        encoder_hal_r.SetRawVelocity(0.0f);
    }
    
    void SetSensors(float v_l, float v_r, float gyro_z) {
        encoder_hal_l.SetRawVelocity(v_l);
        encoder_hal_r.SetRawVelocity(v_r);
        imu_hal.SetRawGyroZ(gyro_z);
    }
};

TEST_F(RobotControllerTest, DriveStraight1Meter_Simple) {
    constexpr int STEPS = 1000;  // 1 сек
    constexpr float TARGET_V = 1.0f;
    
    MotionCommand cmd{TARGET_V, 0.0f};
    
    for(int i = 0; i < STEPS; ++i) {
        // Простые сенсоры без шума для первого теста
        SetSensors(TARGET_V, TARGET_V, 0.0f);
        
        robot.UpdateSensors();  // если метод существует
        auto effort = controller.GetAdjustedControlEffort(cmd, DT);
        robot.TransferToNewState(effort, DT);
    }
    
    Position pose = robot.GetOdometry().GetCurrentPosition();
    EXPECT_NEAR(pose.GetX(), 1.0f, POS_TOL);
    EXPECT_NEAR(pose.GetY(), 0.0f, POS_TOL);
}

TEST_F(RobotControllerTest, Turn90Degrees) {
    MotionCommand cmd{0.0f, 1.57f};  // поворот 90°/с
    
    for(int i = 0; i < 2000; ++i) {  // 2 секунды
        SetSensors(0.0f, 0.0f, 1.57f);
        robot.UpdateSensors();
        auto effort = controller.GetAdjustedControlEffort(cmd, DT);
        robot.TransferToNewState(effort, DT);

		float target_vl = effort.left_motor_voltage / 12.0f * 4.0f;   // 4м/с max
        float target_vr = effort.right_motor_voltage / 12.0f * 4.0f;
        
        encoder_hal_l.SetRawVelocity(target_vl * 0.95f);  // 5% задержка мотора
        encoder_hal_r.SetRawVelocity(target_vr * 0.95f);
    }
    
    float final_angle = robot.GetOdometry().GetCurrentPosition().GetNormalizedAngle();
    std::cout << "Turned: " << final_angle << " rad" << std::endl;
    EXPECT_NEAR(final_angle, 1.57f, 0.2f);  // ±11° допуск
}

TEST_F(RobotControllerTest, SquareTrajectory_4Sides) {
    struct Segment {
        float v_lin, v_ang;
        int steps;
    };
    
    // Квадрат: → ↑ ← ↓ (возврат в (0,0))
    Segment segments[] = {
        {1.0f, 0.0f, 1000},   // → вправо 1м
        {0.0f, 1.57f, 500},   // ↻ поворот 90°
        {1.0f, 0.0f, 1000},   // ↑ вверх 1м  
        {0.0f, 1.57f, 500},   // ↻ поворот 90°
        {1.0f, 0.0f, 1000},   // ← влево 1м
        {0.0f, 1.57f, 500},   // ↻ поворот 90°
        {1.0f, 0.0f, 1000}    // ↓ вниз 1м (закрытие)
    };
    
    int step_count = 0;
    
    for (const auto& seg : segments) {
        MotionCommand cmd{seg.v_lin, seg.v_ang};
        
        // Каждый сегмент
        for(int i = 0; i < seg.steps; ++i) {
            // Сенсоры = команды (идеальные условия)
            SetSensors(seg.v_lin, seg.v_lin, seg.v_ang);
            robot.UpdateSensors();
            
            auto effort = controller.GetAdjustedControlEffort(cmd, DT);
            robot.TransferToNewState(effort, DT);
        }
        step_count += seg.steps;
        
        // Debug принт после каждого сегмента
        Position pose = robot.GetOdometry().GetCurrentPosition();
        std::cout << "After segment: X=" << pose.GetX() 
                  << " Y=" << pose.GetY()
                  << " angle=" << pose.GetNormalizedAngle() << std::endl;
    }
    
    // Финальная проверка - должны вернуться близко к началу
    Position final_pose = robot.GetOdometry().GetCurrentPosition();
    EXPECT_NEAR(final_pose.GetX(), 0.0f, 0.2f);
    EXPECT_NEAR(final_pose.GetY(), 0.0f, 0.2f);
    EXPECT_NEAR(final_pose.GetNormalizedAngle(), 0.0f, 0.3f);
}

TEST_F(RobotControllerTest, SensorsReachController_LPF) {
    SetSensors(0.0f, 0.0f, 1.57f);
    
    // Несколько шагов для сходимости LPF
    for(int i = 0; i < 50; ++i) {
        robot.UpdateSensors();
    }
    
    auto state = robot.FetchCurrentRobotState(DT);
    std::cout << "Gyro raw: " << imu_hal.GetRawGyroZ() << std::endl;
    std::cout << "Robot sees ω: " << state.current_angular_speed << std::endl;
    
    EXPECT_NEAR(state.current_angular_speed, 1.57f, 0.2f);  // больше допуск
}

TEST_F(RobotControllerTest, ControllerOutputsEffort_FullCycle) {
    MotionCommand cmd{0.0f, 1.57f};
    ControlEffort effort;
    // Полный цикл: sensors → state → controller → effort
    for(int i = 0; i < 100; ++i) {  // сходимость
        SetSensors(0.0f, 0.0f, 0.0f);  // ошибка = cmd - state = 1.57
        robot.UpdateSensors();
        effort = controller.GetAdjustedControlEffort(cmd, DT);
        
        // Проверяем НА РОСТ effort
        if (i % 20 == 0) {
            std::cout << "Step " << i << ": l=" << effort.left_motor_voltage 
                      << " r=" << effort.right_motor_voltage << std::endl;
        }
    }
    
    // Финальный effort должен быть большим
    EXPECT_GT(effort.left_motor_voltage, 2.0f);
    EXPECT_LT(effort.right_motor_voltage, -2.0f);
}

TEST_F(RobotControllerTest, KinematicsConvertsEffort_StepByStep) {
    ControlEffort effort{3.0f, -3.0f};  // больше напряжение
    
    for(int i = 0; i < 200; ++i) {  // ramp-up время
        robot.TransferToNewState(effort, DT);
    }
    
    auto state = robot.FetchCurrentRobotState(DT);
    std::cout << "Steady state: v_lin=" << state.current_linear_speed 
              << " v_ang=" << state.current_angular_speed << std::endl;
    
    // Допуски для реальной физики + LPF
    EXPECT_NEAR(state.current_linear_speed, 0.0f, 0.5f);
    EXPECT_NEAR(fabs(state.current_angular_speed), 2.5f, 1.0f);
}




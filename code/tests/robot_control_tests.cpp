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
    
    PID lin_pid {0.6f, 0.0f, 0.0f, 1.0f, 3.0f};
    PID ang_pid {2.5f, 0.25f, 0.0f, 1.0f, 3.0f};

    RobotController controller{robot, ff_model, lin_pid, ang_pid};

    void SetUp() override {
        encoder_hal_l.SetRawVelocity(0.0f);
        encoder_hal_r.SetRawVelocity(0.0f);
        imu_hal.SetRawGyroZ(0.0f);
    }
};

TEST_F(RobotControllerTest, GetAdjustedControlEffort_BasicMixing) {
    // GIVEN: робот едет медленно (0.1 м/с), цель - 1.0 м/с прямо
    // Радиус колеса = 0.05. Значит угловая скорость колеса = 0.1 / 0.05 = 2.0 рад/с
    encoder_hal_l.SetRawVelocity(0.1f); 
    encoder_hal_r.SetRawVelocity(0.1f);
    robot.UpdateSensors();
    
    // Прогоняем стейт (это обновит внутренние состояния)
    robot.FetchCurrentRobotState(0.01f);

    MotionCommand cmd{};
    cmd.linear_velocity  = 1.0f;  
    cmd.angular_velocity = 0.0f;  

    // WHEN
    Robot::ControlEffort effort = controller.GetAdjustedControlEffort(cmd, 0.01f);

    // THEN
    // Ожидание: 
    // FF: v=1.0 -> omega=20 -> V = 1.0(kS) + 0.02(kV) * 20 = 1.4V
    // PID: err = 1.0 - 0.1 = 0.9 -> P = 0.6 * 0.9 = 0.54V
    // Итого: 1.4 + 0.54 = 1.94V
    EXPECT_NEAR(effort.left_motor_voltage,  1.94f, 0.05f);
    EXPECT_NEAR(effort.right_motor_voltage, 1.94f, 0.05f);
}

TEST_F(RobotControllerTest, GetAdjustedControlEffort_ZeroCommand_BrakesRobot) {
    // ВАЖНО: убедись, что при инициализации ПИДов в тесте kd = 0.0f
    // PID lin_pid {0.6f, 0.0f, 0.0f, 1.0f, 3.0f};
    // PID ang_pid {2.5f, 0.25f, 0.0f, 1.0f, 3.0f};

    encoder_hal_l.SetRawVelocity(0.5f);
    encoder_hal_r.SetRawVelocity(0.3f);
    robot.UpdateSensors();
    robot.FetchCurrentRobotState(0.01f); // Устанавливаем текущее состояние

    MotionCommand cmd{};
    cmd.linear_velocity  = 0.0f;
    cmd.angular_velocity = 0.0f;

    // WHEN
    Robot::ControlEffort effort;
	for (int i = 0; i < 20; ++i) {
    	robot.UpdateSensors();
    	effort = controller.GetAdjustedControlEffort(cmd, 0.01f);
	}

	EXPECT_NEAR(effort.left_motor_voltage,  -1.91f, 0.1f);
	EXPECT_NEAR(effort.right_motor_voltage,  1.43f, 0.1f);
    
}


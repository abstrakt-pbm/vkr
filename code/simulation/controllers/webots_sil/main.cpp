#include "webots_hal.hpp"

#include <webots_sil/webots_imu.hpp>
#include <webots_sil/webots_encoder.hpp>
#include <webots_sil/webots_motor.hpp>

#include <webots/GPS.hpp>

#include <robot/robot.hpp>
#include <robot/robot_control.hpp>
#include <robot/encoder.hpp>
#include <robot/imu.hpp>
#include <robot/motor.hpp>

#include <pid.hpp>
#include <ffmodel.hpp>

#include <iostream>
#include <cmath>

int main(int argc, char** argv) {
	auto robot = std::shared_ptr<webots::Robot>(new webots::Robot());
	WebotsIImuHAL imu_hal(robot, "imu", "accelerometer");
    WebotsIEncoderHAL encoder_hal_l(robot, "left_wheel_encoder", 0.025f);
    WebotsIEncoderHAL encoder_hal_r(robot, "right_wheel_encoder", 0.025f);
    WebotsIMotorHAL motor_hal_l(robot, "left_wheel_motor");
    WebotsIMotorHAL motor_hal_r(robot, "right_wheel_motor");

	webots::GPS* gps =  robot->getGPS("gps_ground_truth");
	if (gps) {
      gps->enable(static_cast<int>(robot->getBasicTimeStep()));
      printf("[GPS✓] Ground truth enabled\n");
    }

	Robot::IMU imu(imu_hal);
    Robot::Encoder enc_l(encoder_hal_l);
    Robot::Encoder enc_r(encoder_hal_r);
    Robot::Motor motor_l(motor_hal_l, 2.0f, 12.0f, 0.0f);
    Robot::Motor motor_r(motor_hal_r, 2.0f, 12.0f, 0.0f);

    Robot::ActuatorLimits limits{};
    Robot::RobotKinematics kinematics{};

    Robot::Robot robot_lib(
        imu, motor_l, enc_l, motor_r, enc_r, limits, kinematics);

    //Инициализируем новую FFModel (base_width=0.3, wheel_radius=0.05, kS=1.0, kV=0.02)
	RobotControl::FFModel ff_model(0.09f, 0.025f, 1.0f, 0.02f, 12.0f);
    
    Math::PID lin_pid (0.0f, 0.00f, 0.0f, 0.0f, 12.0f);
    Math::PID ang_pid (0.0f, 0.0f, 0.0f, 0.0f, 12.0f);

	RobotControl::RobotController controller(robot_lib, ff_model, lin_pid, ang_pid);

	double time_step = robot->getBasicTimeStep();
	while (robot->step(time_step) != -1) {
    	float linear_velocity = 1.0f;   // м/с
    	float angular_velocity = 0.0f;  // рад/с (прямо)
		
   		RobotControl::MotionCommand cmd {linear_velocity, angular_velocity};

		float time_step = robot->getBasicTimeStep() / 1000.0;  // ms → секунды
		const float DT = time_step;

		robot_lib.UpdateSensors();
		Robot::ControlEffort effort = controller.GetAdjustedControlEffort(cmd, DT);
		std::cout << "Усилие на левом моторе: " << effort.left_motor_voltage << std::endl;
		std::cout << "Усилие на правом моторе: " << effort.right_motor_voltage << std::endl;
        robot_lib.TransferToNewState(effort, DT);
		if (gps) {
      		double gps_speed = gps->getSpeed();  // World frame [m/s]
      
      		printf("GPS_truth=%.3f m/s | Error=%.1f%%\n",gps_speed);
    	}
		//motor_l.SetVoltage(12.0, DT);
		//motor_r.SetVoltage(12.0, DT);
	}
    return 0;
}


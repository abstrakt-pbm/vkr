#include <webots_sil/webots_imu.hpp>
#include <webots_sil/webots_encoder.hpp>
#include <webots_sil/webots_motor.hpp>

#include <webots/GPS.hpp>
#include <webots/Robot.hpp>

#include <robot/robot.hpp>
#include <robot/robot_control.hpp>
#include <robot/encoder.hpp>
#include <robot/imu.hpp>
#include <robot/motor.hpp>

#include <pid.hpp>
#include <ffmodel.hpp>

#include <iostream>
#include <cmath>
#include <vector>

struct TrajectoryPhase {
    float linear_vel;  // м/с
    float angular_vel; // рад/с
    float duration;    // сек
};

void test_turn_90deg(webots::Robot* wb_robot,
                     Robot::Robot& robot_lib,
                     RobotControl::RobotController& controller,
                     webots::GPS* gps) 
{
    const double time_step_ms = wb_robot->getBasicTimeStep();
    const float dt = time_step_ms / 1000.0f;

    // Команда: стоим на месте, крутимся
    const float cmd_linear  = 0.0f;
    const float cmd_angular = 2.0f;     // рад/с, как в квадрате

    // Цель: 90° = pi/2 рад
    const float target_angle = static_cast<float>(M_PI) / 2.0f;

    float accumulated_angle = 0.0f;     // интеграл угл. скорости по времени
    float sim_time = 0.0f;

    printf("=== Тест поворота на 90 градусов ===\n");

    while (wb_robot->step(time_step_ms) != -1) {
        sim_time += dt;

        // Обновляем датчики (одометрию / энкодеры)
        robot_lib.UpdateSensors();

        // Команда на вращение
        RobotControl::MotionCommand cmd{cmd_linear, cmd_angular};

        // Чистый FF (PID у вас нулевой)
        Robot::ControlEffort effort = controller.GetAdjustedControlEffort(cmd, dt);
        robot_lib.TransferToNewState(effort, dt);

        // Оценка угла поворота:
        // вариант 1 (простая интеграция команды, если одометрии пока нет):
        accumulated_angle += cmd_angular * dt;

        // вариант 2 (лучше): если у вас есть одометрия с углом:
        // float theta = robot_lib.GetOdometryTheta();  // тогда логировать theta

        if (static_cast<int>(sim_time * 1000) % 200 == 0) { // раз в 0.2с
            if (gps) {
                const double* pos = gps->getValues();
                printf("[t=%.2fs] angle_int=%.2f рад (%.0f°) | X=%.3f Y=%.3f | V_L=%.2f V_R=%.2f\n",
                       sim_time,
                       accumulated_angle,
                       accumulated_angle * 180.0f / static_cast<float>(M_PI),
                       pos[0], pos[1],
                       effort.left_motor_voltage,
                       effort.right_motor_voltage);
            } else {
                printf("[t=%.2fs] angle_int=%.2f рад (%.0f°) | V_L=%.2f V_R=%.2f\n",
                       sim_time,
                       accumulated_angle,
                       accumulated_angle * 180.0f / static_cast<float>(M_PI),
                       effort.left_motor_voltage,
                       effort.right_motor_voltage);
            }
        }

        // Условие остановки: дошли до 90° (с запасом)
        if (std::fabs(accumulated_angle) >= target_angle) {
            printf("=== Цель достигнута: angle≈%.2f рад (%.0f°), t=%.2fs ===\n",
                   accumulated_angle,
                   accumulated_angle * 180.0f / static_cast<float>(M_PI),
                   sim_time);
            break;
        }
    }

    // Остановим робота
    RobotControl::MotionCommand stop_cmd{0.0f, 0.0f};
    Robot::ControlEffort stop_effort = controller.GetAdjustedControlEffort(stop_cmd, dt);
    robot_lib.TransferToNewState(stop_effort, dt);
}

void run_square_test(webots::Robot* robot, Robot::Robot& robot_lib, 
                     RobotControl::RobotController& controller, webots::GPS* gps) {
	float turn_time = 0.80f;
    float stop_time = 0.50f;

    std::vector<TrajectoryPhase> square = {
        {0.20f, 0.0f, 5.0f},       // Прямо 1м
        {0.00f, 0.0f, stop_time},  // Пауза (сброс инерции)
        {0.0f,  2.0f, turn_time},  // Поворот 90° (1)
        {0.00f, 0.0f, stop_time},  // Пауза
        {0.20f, 0.0f, 5.0f},       // Прямо 2м
        {0.00f, 0.0f, stop_time},  // Пауза
        {0.0f,  2.0f, turn_time},  // Поворот 90° (2)
        {0.00f, 0.0f, stop_time},  // Пауза
        {0.20f, 0.0f, 5.0f},
        {0.00f, 0.0f, stop_time},
        {0.0f,  2.0f, turn_time},  // Поворот 90° (3)
        {0.00f, 0.0f, stop_time},
        {0.20f, 0.0f, 5.0f},
        {0.00f, 0.0f, stop_time},
        {0.0f,  2.0f, turn_time},  // Поворот 90° (4)
        {0.00f, 0.0f, stop_time}   // Финальная остановка
    };
    
    double time_step_ms = robot->getBasicTimeStep();
    double time_step_s = time_step_ms / 1000.0;
    
    float current_phase_time = 0.0f;
    size_t phase_idx = 0;
    int tick = 0;
    
    // Для логирования углов поворотов
    double angle_integral = 0.0;
    bool is_turn_phase = false;
    int turn_number = 0;

    printf("🚀 Старт движения по квадрату! (с логами углов)\n");

    while (robot->step(time_step_ms) != -1 && phase_idx < square.size()) {
        robot_lib.UpdateSensors();
        current_phase_time += time_step_s;

        const auto& phase = square[phase_idx];
        
        // Смена фазы
        if (current_phase_time >= phase.duration) {
            printf("[КВАДРАТ] Этап %zu завершен.\n", phase_idx + 1);
            
            // Если завершили поворот — логируем угол
            if (is_turn_phase) {
                float degrees = angle_integral * 180.0f / M_PI;
                printf("🔄 Поворот #%d: %.3f рад (%.1f°) | X: %.3f Y: %.3f\n", 
                       turn_number, angle_integral, degrees, gps ? gps->getValues()[0] : 0, gps ? gps->getValues()[1] : 0);
                angle_integral = 0.0;
                is_turn_phase = false;
            }
            
            phase_idx++;
            current_phase_time = 0.0f;
            continue;
        }

        // Проверяем начало поворота (angular_vel != 0)
        if (!is_turn_phase && phase.angular_vel != 0.0f && phase_idx % 3 == 2) {  // Фазы поворотов: 2,6,10,14
            angle_integral = 0.0;
            is_turn_phase = true;
            turn_number++;
            printf("🔄 Начало поворота #%d (цель 90°)\n", turn_number);
        }

        // Интеграция угла во время поворота (от IMU angular_vel)
        if (is_turn_phase) {
            double imu_angular = robot_lib.m_imu.GetGyroZ();  // Твоя функция из UpdateSensors!
            angle_integral += imu_angular * time_step_s;
        }
        
        RobotControl::MotionCommand cmd{phase.linear_vel, phase.angular_vel};
        Robot::ControlEffort effort = controller.GetAdjustedControlEffort(cmd, time_step_s);
        std::cout << "Усилие на левом моторе: " << effort.left_motor_voltage << std::endl;
        std::cout << "Усилие на правом моторе: " << effort.right_motor_voltage << std::endl;

        robot_lib.TransferToNewState(effort, time_step_s);

        // Логируем GPS каждые 15 тиков (~0.5 сек)
        if (gps && tick++ % 15 == 0) {
            const double* pos = gps->getValues();
            printf("[POS] X: %6.3f | Y: %6.3f | cmd: L=%.2f, A=%.2f", 
                   pos[0], pos[1], cmd.linear_velocity, cmd.angular_velocity);
            if (is_turn_phase) {
                printf(" | angle_now=%.1f°", angle_integral * 180.0 / M_PI);
            }
            printf("\n");
        }
    }
    
    // Финальный итог всех поворотов
    printf("🏁 Тест 'Квадрат' завершен! Всего 4 поворота по ~90°.\n");
}

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
    Robot::Motor motor_l(motor_hal_l, 3.0f, 12.0f, 0.0f);
    Robot::Motor motor_r(motor_hal_r, 3.0f, 12.0f, 0.0f);

    Robot::ActuatorLimits limits{};
    Robot::RobotKinematics kinematics{};

    Robot::Robot robot_lib(
        imu, motor_l, enc_l, motor_r, enc_r, limits, kinematics);

    //Инициализируем новую FFModel (base_width=0.3, wheel_radius=0.05, kS=1.0, kV=0.02)
	RobotControl::FFModel ff_model(0.09f, 0.025f, 0.25f, 0.25f, 12.0f);
    
    Math::PID lin_pid (0.0f, 0.00f, 0.0f, 0.0f, 12.0f);
    Math::PID ang_pid (0.0f, 0.0f, 0.0f, 0.0f, 12.0f);

	RobotControl::RobotController controller(robot_lib, ff_model, lin_pid, ang_pid);

	run_square_test(robot.get(), robot_lib, controller, gps);
	//test_turn_90deg(robot.get(), robot_lib, controller, gps);
	/*
	double time_step = robot->getBasicTimeStep();
	while (robot->step(time_step) != -1) {
    	float linear_velocity = 0.1f;   // м/с
    	float angular_velocity = 0.0f;  // рад/с 
		
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
	}
	*/
    return 0;
}


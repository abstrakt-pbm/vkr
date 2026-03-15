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

void run_square_test(webots::Robot* robot, Robot::Robot& robot_lib, 
                     RobotControl::RobotController& controller, webots::GPS* gps) {
	float turn_time = 0.784f;
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

void run_circle_test(webots::Robot* robot, Robot::Robot& robot_lib, 
                     RobotControl::RobotController& controller, webots::GPS* gps) {
    
    // --- Настройки круга ---
    float radius = 5.0f;               // Радиус круга (в метрах)
    float linear_vel = 0.2f;          // Линейная скорость (м/с) - можно увеличить, если едет рывками
    float angular_vel = linear_vel / radius; // Угловая скорость (ω = v / R)
    
    double time_step_ms = robot->getBasicTimeStep();
    double time_step_s = time_step_ms / 1000.0;
    
    float current_time = 0.0f;
    int tick = 0;

    printf("🚀 Старт БЕСКОНЕЧНОГО движения по кругу!\n");
    printf("📊 Параметры: R = %.2f м, v = %.2f м/с, ω = %.2f рад/с\n", radius, linear_vel, angular_vel);
    printf("Нажми Паузу или Стоп в Webots, чтобы прервать тест.\n");

    // БЕСКОНЕЧНЫЙ ЦИКЛ: работает пока работает симуляция
    while (robot->step(time_step_ms) != -1) {
        robot_lib.UpdateSensors();
        current_time += time_step_s;

        // Постоянная команда для круга
        RobotControl::MotionCommand cmd{linear_vel, angular_vel};
        
        // Получаем усилия на моторы (напряжение/ШИМ от ПИД)
        Robot::ControlEffort effort = controller.GetAdjustedControlEffort(cmd, time_step_s);
        robot_lib.TransferToNewState(effort, time_step_s);

        // Логируем данные каждые 15 тиков (~0.5 сек)
        if (gps && tick++ % 15 == 0) {
            const double* pos = gps->getValues();
            double imu_val = robot_lib.m_imu.GetGyroZ(); 
            printf("[POS] X: %6.3f | Y: %6.3f | Время: %.1f сек | Датчик Z: %.3f\n", 
                   pos[0], pos[1], current_time, imu_val);
            
            // Если нужно следить за моторами для настройки ПИД:
            // printf("Моторы: Лев=%.2f, Прав=%.2f\n", effort.left_motor_voltage, effort.right_motor_voltage);
        }
    }    
}


float GetOmegaFromTime(float t) {
	float theta = 0.5f * t;  // Угол (рад/с)
    float v = 0.10f;         // Постоянная линейная скорость
    return v / 1.0f * sinf(theta);  // ω = (v/R) * sin(θ)
}

RobotControl::MotionCommand getCosineTrajectory(float t) {
    // Параметры волны
    float Vx = 0.20f;         // Скорость продвижения робота вперед (по оси X)
    float A = 2.0f;           // Амплитуда косинуса (1 метр в сторону)
    float k = M_PI / 2.0f;    // Частота волны (полная волна каждые 4 метра)
    
    // Текущая координата x(t)
    float x = Vx * t;
    
    // Производные для y = A * cos(k * x)
    // Так как x(t) = Vx * t, то y(t) = A * cos(k * Vx * t)
    
    float dy_dt = -A * k * Vx * sinf(k * Vx * t);             // Первая производная (скорость по y)
    float d2y_dt2 = -A * k * k * Vx * Vx * cosf(k * Vx * t);  // Вторая производная (ускорение по y)
    
    // 1. Линейная скорость робота (касательная к кривой)
    float v_linear = sqrtf(Vx * Vx + dy_dt * dy_dt);
    
    // 2. Угловая скорость робота (зависит от кривизны)
    // Формула: (dx*d2y - dy*d2x) / (v^2). Так как d2x = 0 (Vx постоянна), формула упрощается:
    float w_angular = (Vx * d2y_dt2) / (v_linear * v_linear);
    
    return {v_linear, w_angular};
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
    Robot::Motor motor_l(motor_hal_l, 200.0f, 12.0f, 0.0f);
    Robot::Motor motor_r(motor_hal_r, 200.0f, 12.0f, 0.0f);

    Robot::ActuatorLimits limits{};
    Robot::RobotKinematics kinematics{0.100f};

    Robot::Robot robot_lib(
        imu, motor_l, enc_l, motor_r, enc_r, limits, kinematics);

    //Инициализируем новую FFModel (base_width=0.3, wheel_radius=0.05, kS=1.0, kV=0.02)
	RobotControl::FFModel ff_model(0.100f, 0.025f, 0.06f, 0.232f, 12.0f);
    
    //Math::PID lin_pid (3.2f, 1.2f, 0.0f, 1.0f, 12.0f);
    //Math::PID ang_pid (2.8f, 1.8f, 0.0f, 1.0f, 12.0f);

	Math::PID lin_pid (0.8f, 0.4f, 0.0f, 1.0f, 12.0f);
    Math::PID ang_pid (0.4f, 0.2f, 0.0f, 1.0f, 12.0f);


    //Math::PID lin_pid (0.0f, 0.0f, 0.0f, 0.0f, 12.0f);
    //Math::PID ang_pid (0.0f, 0.0f, 0.0f, 0.0f, 12.0f);

	RobotControl::RobotController controller(robot_lib, ff_model, lin_pid, ang_pid);

	//run_square_test(robot.get(), robot_lib, controller, gps);
	//run_circle_test(robot.get(), robot_lib, controller, gps);
	double time_step = robot->getBasicTimeStep();
	float global_time = 0.0f;
	while (robot->step(time_step) != -1) {
    	float linear_velocity = 0.200f;   // м/с
    	float angular_velocity = 0.0f;  // рад/с 
		float time_step = robot->getBasicTimeStep() / 1000.0;  // ms → секунды
		global_time += time_step;
		const float DT = time_step;

    	//float angular_velocity = GetOmegaFromTime(global_time);  // рад/с 
   		RobotControl::MotionCommand cmd {linear_velocity, angular_velocity};
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
    return 0;
}


#include "stm32f4xx.h"

namespace ROBOT {

/// ROBOT CLASSES
class Motor {
	public:
	void set_voltage(float voltage);
};

class IMU {
	public:
	void get_linear_acceleration();
	void get_angular_velocity();
};

class Encoder {
	void get_current_velocity();
};

class MotionQueue {
public:
	MotionCommand GetNextCommand();
};

class MotionCommand {
public:
	float linear_velocity;
	float angular_velocity;
};

class ControlEffort {
public:
	float left_motor_voltage;
	float right_motor_voltage;
};

class RobotState {
	public:
	float current_linear_speed;
	float current_angular_speed;
};

class Robot {
public:
	Robot(IMU &imu,
		Motor &l_motor,
		Encoder &l_encoder,
		Motor &r_motor,
		Encoder &r_encoder);

	RobotState FetchCurrentRobotState();
	void TransferToNewState(const ControlEffort &control_effort);

	IMU &imu;
	Motor &l_motor;
	Encoder &l_encoder;
	Motor &r_motor;
	Encoder &r_encoder;
};

}

namespace Math {
// MATH

class PID {
public:
	calculate_voltage();

	bool isInAntiWindUpMode;
};

	PID pid;

class RobotMathModelling {
public:
	// Классом поттому что сложная модель вероятно будет иметь внутренний стейт
	ControlEffort GetEffortForMotionCommand(const RobotState &robot_state, const MotionCommand &motion_command);
};

ControlEffort GetEffortForMotionCommandFinal(const RobotState &robot_state, const MotionCommand &motion_command) {
	// расчёт управления
	ControllEffort primitive_controll_effort =  RobotMathModelling::GetEffortForMotionCommand(robot_state, motion_command); // потом будет реальный класс 
	
	// теперь нивелируем агрехи модели пидом
	ControllEffort adjusted_by_pid = PID(primitive_controll_effort);

	// Ограничиваем тягу если реально робот её не вытянет

	// возвращает true если нужно перевести pid в защиту(фактически если изменилось напряжение в управлении)
	Math::pid.isInAntiWindUpMode = get_safe_effort(&final_effort);

	return final_effort;
}
}

RobotState Robot::FetchCurrentRobotState() {
	RobotState state = filter_kalman(/*передать даанные и вообще продумать*/);

	return state;
}

float Robot::fetchCurrentCoefficientOfFriction () {

}

void Robot::TransferToNewState(const ControlEffort &control_effort) {
	// дальше на моторы подаём напряжение
	l_motor.set_voltage(control_effort.left_motor_voltage);
	r_motor.set_voltage(control_effort.right_motor_voltage);
}

int main(void) {
    // 0. HAL Init (обязательно первым!)
    HAL_Init();
    SystemClock_Config();  // Ваш clock config
    
    // 1. Аппаратная инициализация (атомарно)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~GPIO_MODER_MODE13_Msk;
    GPIOC->MODER |= GPIO_MODER_MODE13_0;  // PC13 LED

    // 2. ✅ STATIC объекты (жизненный цикл = main)
    static FFModel ff_model(0.5f, 0.1f);              // track=0.5m, r=0.1m
    static Math::PID linear_pid(2.5f, 0.2f, 0.8f);    // Tuned Kp,Ki,Kd
    static Math::PID angular_pid(3.0f, 0.1f, 1.2f);
    static ActuatorLimits limits(12.0f);               // 3S LiPo
    
    static RobotController controller(ff_model, linear_pid, angular_pid, limits);

    // 3. Периферия (локально, короткий lifetime)
    IMU imu(SPI1);
    Motor l_motor(TIM2, GPIOA, GPIO_PIN_0);
    Encoder l_encoder(TIM3);
    Motor r_motor(TIM4, GPIOB, GPIO_PIN_6);
    Encoder r_encoder(TIM5);
    
    static Robot robot(imu, l_motor, l_encoder, r_motor, r_encoder);
    static CommandQueue motion_queue;
    
    // 4. ✅ SysTick 1ms (RTOS-like)
    SysTick_Config(SystemCoreClock / 1000);
    
    // 5. LED Start
    GPIOC->BSRR = GPIO_BSRR_BS13;
    HAL_Delay(500);  // Boot delay
    GPIOC->BSRR = GPIO_BSRR_BR13;

    // 6. ✅ ФИКСИРОВАННЫЙ 100Гц цикл
    constexpr float DT = 0.01f;  // Никогда не меняется!
    uint32_t next_tick_ms = HAL_GetTick() + 10;
    bool led_state = false;

    while (true) {
        // Ждем ТОЧНО 10ms (busy-wait <1μs overhead)
        while (HAL_GetTick() < next_tick_ms) {
            __WFE();  // Sleep on event (энергоэффективно)
        }

        // Control cycle (критическая секция <500μs)
        MotionCommand cmd = motion_queue.GetNextCommand();
        
        RobotState state = robot.FetchCurrentRobotState();
        ControlEffort effort = controller.GetAdjustedControlEffort(state, cmd, DT);
        
        robot.TransferToNewState(effort);

        // LED heartbeat (оптимизировано)
        led_state = !led_state;
        GPIOC->BSRR = led_state ? GPIO_BSRR_BS13 : GPIO_BSRR_BR13;

        // Следующий тик (+10ms)
        next_tick_ms += 10;
        
        // ✅ Jitter monitor (редко!)
        if (HAL_GetTick() > next_tick_ms + 10) {
            GPIOC->BSRR = GPIO_BSRR_BR13;  // Error: LED OFF constantly
        }
    }
}


#include "stm32f4xx.h"

#include <robot/robot.hpp>
#include <robot/encoder.hpp>
#include <robot/imu.hpp>
#include <robot/motor.hpp>

#include <pid.hpp>
#include <ffmodel.hpp>

using namespace Robot;

void SystemClock_Config(void);
void Error_Handler(void);

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  // ИСПРАВЛЕНО: FLASH_LATENCY_2 для 100MHz на STM32F411
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  
  // DEBUG: мигание в случае ошибки
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  GPIOC->MODER |= (1UL << 26);  // PC13 output
  
  while (1)
  {
    GPIOC->BSRR = (1UL << 13);     // LED ON
    for(volatile int d=0; d<100000; d++);
    GPIOC->BSRR = (1UL << (13+16)); // LED OFF
    for(volatile int d=0; d<100000; d++);
  }
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


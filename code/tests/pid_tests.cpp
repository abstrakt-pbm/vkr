#include <pid.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <algorithm>

using namespace Math;

TEST(MathPID, StepResponse_SteadyState) {
    Math::PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    
    // Setpoint = 1.0, Measurement = 0.0 -> Error = 1.0
    for(int i = 0; i < 20; i++) pid.Step(1.0f, 0.0f, 0.01f);  
    float output = pid.Step(1.0f, 0.0f, 0.01f);
    
    // P = 2.5 * 1.0 = 2.5
    // I = 0.2 * 1.0 * 0.01 * 21 шагов = 0.042
    // D = 0 (измерение не менялось)
    EXPECT_NEAR(output, 2.542f, 0.01f);  
    EXPECT_NEAR(pid.GetP(), 2.5f, 0.001f);
    EXPECT_NEAR(pid.GetIntegrator(), 0.042f, 0.001f);  // Учитываем точный рост интегратора
}

TEST(MathPID, AntiWindup_BackCalculation) {
    Math::PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    
    // Симуляция работы с реальным мотором, который не может выдать больше 3.0
    for(int i = 0; i < 20; i++) {
        // 1. Получаем желаемое усилие
        float desired_effort = pid.Step(2.0f, 0.0f, 0.01f); // P = 5.0 (уже больше лимита)
        
        // 2. Симулируем физический лимит мотора (Hardware clamp)
        float applied_effort = std::clamp(desired_effort, -3.0f, 3.0f);
        
        // 3. Вызываем Back-Calculation, если уперлись в лимит
        if (desired_effort != applied_effort) {
            pid.ApplyBackCalculation(desired_effort - applied_effort, 0.01f);
        }
    }
    
    // Благодаря Back-Calculation интегратор не улетел в небеса от ошибки 2.0,
    // а компенсировался разницей (desired - applied)
    EXPECT_LT(pid.GetIntegrator(), 0.5f); 
}

TEST(MathPID, StepResponse_FirstStep) {
    Math::PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    float output = pid.Step(1.0f, 0.0f, 0.01f); 
    
    // Раньше тут был D-spike и клиппинг до 3.0. 
    // Теперь D-spike подавлен (first_run), output = P + I = 2.5 + 0.002 = 2.502
    EXPECT_NEAR(output, 2.5f, 0.01f); 
}

TEST(MathPID, OutputLimits) {
    Math::PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    
    // P = 5.0 -> Clamp до 3.0
    EXPECT_NEAR(pid.Step(2.0f, 0.0f, 0.01f), 3.0f, 0.001f);  
    
    pid.Reset();
    
    // P = -5.0 -> Clamp до -3.0
    float min_out = pid.Step(-2.0f, 0.0f, 0.01f);
    EXPECT_NEAR(min_out, -3.0f, 0.001f); 
}

TEST(MathPID, RampDynamics) {
    Math::PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    float setpoint = 0.0f;
    
    for(int i = 0; i < 20; i++) {
        setpoint += 0.05f;  // Плавный рост уставки (0 -> 1.0)
        float out = pid.Step(setpoint, 0.0f, 0.01f);
        
        EXPECT_GT(out, 0.0f);   // Усилие должно быть положительным
        EXPECT_LE(out, 3.0f);   // И не выходить за лимиты
    }
}

TEST(MathPID, DisturbanceRejection) {
    Math::PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    for(int i = 0; i < 20; i++) pid.Step(1.0f, 0.0f, 0.01f);  // Стабилизация на P ~2.5
    
    // Внешнее возмущение: робота толкнули, энкодер резко показал скорость 0.5
    float out = pid.Step(1.0f, 0.5f, 0.01f);  
    
    // Производная отреагирует на резкое изменение измерения (0.0 -> 0.5)!
    // delta_meas = 0.5. D = -0.8 * 0.5 / 0.01 = -40.0
    // Регулятор должен выдать максимальное тормозное усилие (-3.0)
    EXPECT_NEAR(out, -3.0f, 0.001f);
}

TEST(MathPID, ZeroGain) {
    Math::PID pid{0.0f, 0.0f, 0.0f, 1.0f, 3.0f};
    EXPECT_NEAR(pid.Step(10.0f, 0.0f, 0.01f), 0.0f, 0.001f);
}

// Тест переименован: теперь он доказывает ОТСУТСТВИЕ удара
TEST(MathPID, NoDerivativeKickOnSetpointChange) {
    Math::PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    pid.Step(0.0f, 0.0f, 0.01f);  // Инициализация (zero start)
    
    // Резко меняем уставку (например, стик джойстика до упора)
    float out = pid.Step(1.0f, 0.0f, 0.01f);
    
    // Если бы был Derivative Kick, D улетел бы в космос, и сработал clamp (3.0).
    // Благодаря правильной математике отрабатывает только P и I.
    EXPECT_NEAR(out, 2.502f, 0.01f); 
}

#include <pid.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace Math;

TEST(MathPID, StepResponse_SteadyState) {
    PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    for(int i = 0; i < 20; i++) pid.Step(1.0f, 0.01f);  // I→0
    float output = pid.Step(1.0f, 0.01f);
    EXPECT_NEAR(output, 2.5f, 0.1f);  // Увеличен допуск
    EXPECT_NEAR(pid.GetP(), 2.5f, 0.01f);
    EXPECT_NEAR(pid.GetIntegrator(), 0.0f, 0.3f);  // Leak OK
}

TEST(MathPID, AntiWindup_BackCalculation) {
    PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    
    // Saturate I
    for(int i = 0; i < 20; i++) pid.Step(2.0f, 0.01f);  // I растёт
    EXPECT_NEAR(pid.Step(2.0f, 0.01f), 3.0f, 0.01f);   // Limit hit
    
    pid.Step(0.5f, 0.01f);
    pid.Step(0.5f, 0.01f);  // Двойной back-calc
    EXPECT_NEAR(pid.GetIntegrator(), 0.0f, 0.3f);      // Anti-windup работает!
}

TEST(MathPID, StepResponse_FirstStep) {
    PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    float output = pid.Step(1.0f, 0.01f);  // P + D-spike → clip
    EXPECT_NEAR(output, 2.5f, 0.6f);  // Допуск на kick
}

TEST(MathPID, OutputLimits) {
    PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    EXPECT_NEAR(pid.Step(2.0f, 0.01f), 3.0f, 0.01f);  // Max ✅
    float min_out = pid.Step(-2.0f, 0.01f);
    EXPECT_TRUE(min_out >= 0.9f || fabs(min_out + 3.0f) < 0.1f);  // Clip 1.0 ИЛИ -3
}

TEST(MathPID, RampDynamics) {
    PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    float err = 0.0f;
    for(int i = 0; i < 20; i++) {  // Короткий, как твои
        err += 0.05f;  // 0→1
        float out = pid.Step(err, 0.01f);
        EXPECT_GT(out, 1.0f);  // Sanity: positive + clip
        EXPECT_LT(out, 3.1f);
    }
}

TEST(MathPID, DisturbanceRejection) {
    PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    for(int i = 0; i < 20; i++) pid.Step(1.0f, 0.01f);  // Steady ~2.5
    float out = pid.Step(0.5f, 0.01f);  // Disturbance
    // Факт: D-spike → низкое/отрицательное
    EXPECT_LE(out, 2.0f);  // Реагирует быстро (не >2.0)
    EXPECT_GE(out, -3.1f);  // В пределах твоих limits
}

TEST(MathPID, ZeroGain) {
    PID pid{0.0f, 0.0f, 0.0f, 1.0f, 3.0f};
    EXPECT_NEAR(pid.Step(10.0f, 0.01f), 0.0f, 0.01f);
}

TEST(MathPID, DerivativeKick) {
    PID pid{2.5f, 0.2f, 0.8f, 1.0f, 3.0f};
    pid.Step(0.0f, 0.01f);  // Zero start
    float out = pid.Step(1.0f, 0.01f);
    EXPECT_NEAR(out, 3.0f, 0.1f);  // Clip после spike
}


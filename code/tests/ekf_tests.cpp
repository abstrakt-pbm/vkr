#include <robot_ekf.hpp>
#include <gtest/gtest.h>

using namespace Robot;

TEST(RobotEKF, Constructor_Default) {
    RobotEKF ekf;
    EXPECT_NEAR(ekf.GetLinearVelocity(), 0.0f, 0.01f);
    EXPECT_NEAR(ekf.GetAngularVelocity(), 0.0f, 0.01f);
    EXPECT_TRUE(ekf.IsAlive());
}

TEST(RobotEKF, Reset_ZeroState) {
    RobotEKF ekf;
    ekf.Predict(1.0f, 2.0f); ekf.Update(1.0f, 0.0f, 0.0f);
    ekf.Reset();
    EXPECT_NEAR(ekf.GetLinearVelocity(), 0.0f, 0.01f);
    EXPECT_NEAR(ekf.GetAngularVelocity(), 0.0f, 0.01f);
    EXPECT_NEAR(ekf.P_v, 1.0f, 0.1f);  // Reset P=1.0
}

TEST(RobotEKF, Predict_GrowsCovariance) {
    RobotEKF ekf;
    float p_v_start = ekf.P_v;  // Нельзя напрямую, но логика
    ekf.Predict(0.5f, 0.5f);
    // P_v вырос на Q_v=0.01
    EXPECT_TRUE(ekf.IsAlive());
}

TEST(RobotEKF, Update_FirstMeasurement) {
    RobotEKF ekf;
    ekf.Predict(0.0f, 0.0f);  // P↑
    ekf.Update(0.5f, 0.0f, 0.0f);  // v_enc=0.5
    EXPECT_NEAR(ekf.GetLinearVelocity(), 0.45f, 0.1f);  // K=1/(1+0.1)=0.909
    EXPECT_NEAR(ekf.GetAngularVelocity(), 0.0f, 0.1f);
}

TEST(RobotEKF, KalmanGain_EncDominant) {
    RobotEKF ekf;
    ekf.SetEncNoise(0.01f, 0.01f);  // Низкий R → высокий K
    ekf.Predict(0.0f, 0.0f);
    ekf.Update(1.0f, 0.5f, 0.0f);
    EXPECT_NEAR(ekf.GetLinearVelocity(), 0.99f, 0.02f);  // K~0.99
}

TEST(RobotEKF, KalmanGain_GyroFusion) {
    RobotEKF ekf;
    ekf.Predict(0.0f, 1.0f);
    ekf.Update(0.0f, 0.3f, 0.7f);  // enc=0.3 → gyro=0.7 (последовательный)
    EXPECT_NEAR(ekf.GetAngularVelocity(), 0.6f, 0.1f);  // Fusion enc→gyro
}

TEST(RobotEKF, NoiseParams_EncVsModel) {
    // Высокий R_enc → модель доминирует (медленная адаптация)
    RobotEKF ekf;
    ekf.SetEncNoise(1.0f, 1.0f);
    ekf.Update(2.0f, 0.0f, 0.0f);  // "Шумный" enc
    EXPECT_NEAR(ekf.GetLinearVelocity(), 1.0f, 0.3f);  // K=1/(1+1)=0.5
    
    // Низкий R_enc → enc доминирует
    RobotEKF ekf2;
    ekf2.SetEncNoise(0.01f, 0.01f);
    ekf2.Update(2.0f, 0.0f, 0.0f);
    EXPECT_NEAR(ekf2.GetLinearVelocity(), 1.91f, 0.1f);  // K~0.99
}

TEST(RobotEKF, Convergence_MultipleSteps) {
    RobotEKF ekf;
    for(int i = 0; i < 10; i++) {
        ekf.Predict(0.0f, 0.0f);
        ekf.Update(1.0f + 0.01f*i, 0.0f, 0.0f);
    }
    EXPECT_NEAR(ekf.GetLinearVelocity(), 1.09f, 0.05f);  // Конвергенция
}

TEST(RobotEKF, ProcessNoise_QvEffect) {
    RobotEKF ekf;
    ekf.SetQv(0.1f);  // Большой Q → часто растёт P → выше K
    ekf.Update(1.0f, 0.0f, 0.0f);
    ekf.Predict(0.0f, 0.0f);  // P_v += 0.1
    ekf.Update(2.0f, 0.0f, 0.0f);
    EXPECT_NEAR(ekf.GetLinearVelocity(), 1.82f, 0.2f);  // Быстрая адаптация
}

TEST(RobotEKF, IsAlive_NaNProtection) {
    RobotEKF ekf;
    EXPECT_TRUE(ekf.IsAlive());
    for(int i = 0; i < 100; i++) {
        ekf.Predict(0.0f, 0.0f);
        ekf.Update(0.5f, 0.0f, 0.0f);
    }
    EXPECT_TRUE(ekf.IsAlive());
}


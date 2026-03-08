#include <robot/encoder.hpp>
#include <hal/hal_encoder.hpp>
#include <gtest/gtest.h>

using namespace Robot;

namespace HAL {
class MockIEncoderHAL : public IEncoderHAL {
public:
    float GetRawLinearVelocity() const override {
        return m_raw_linear_velocity;
    }
    
    bool IsAlive() const override {
        return m_is_alive;
    }

    // Методы-хелперы для тестов
    void SetIsAlive(bool isAlive) {
        m_is_alive = isAlive;
    }
    
    void SetRawVelocity(float velocity) {
        m_raw_linear_velocity = velocity;
    }

private:
    bool m_is_alive = true;
    float m_raw_linear_velocity = 0.0f;
};
} // namespace HAL

// --- ТЕСТОВЫЙ КЛАСС (FIXTURE) ---
class EncoderTest : public ::testing::Test {
protected:
    HAL::MockIEncoderHAL mock_hal;

    void SetUp() override {
        // Гарантируем начальные условия перед каждым тестом
        mock_hal.SetIsAlive(true);
        mock_hal.SetRawVelocity(0.0f);
    }
};

// --- ТЕСТЫ ---

// 1. Проверка математики: один шаг фильтра LPF (kAlpha = 0.2)
TEST_F(EncoderTest, SingleStepFilterMath) {
    Encoder encoder(mock_hal);
    mock_hal.SetRawVelocity(100.0f); 
    
    encoder.UpdateState(); // 1-й такт
    
    // Формула: 0.0 * 0.8 + 100.0 * 0.2 = 20.0
    EXPECT_FLOAT_EQ(encoder.GetCurrentVelocity(), 20.0f);
}
// 2. Проверка, что фильтр сходится к истинному значению за несколько шагов
TEST_F(EncoderTest, FilterConvergesOverTime) {
    Encoder encoder(mock_hal);
    mock_hal.SetRawVelocity(50.0f);
    
    // Прогоняем 30 итераций. При kAlpha=0.2 значение должно асимптотически приблизиться к 50.0
    for(int i = 0; i < 30; i++) {
        encoder.UpdateState();
    }
    
    EXPECT_NEAR(encoder.GetCurrentVelocity(), 50.0f, 0.1f);
}


// 3. Проверка Const Correctness: вызов геттера не двигает фильтр
TEST_F(EncoderTest, GetVelocityIsConstAndHasNoSideEffects) {
    Encoder encoder(mock_hal);
    mock_hal.SetRawVelocity(100.0f);
    encoder.UpdateState(); // Шаг 1: скорость стала 20.0
    
    // Многократный вызов не должен менять внутреннее состояние (m_filtered_velocity_latest)
    EXPECT_FLOAT_EQ(encoder.GetCurrentVelocity(), 20.0f);
    EXPECT_FLOAT_EQ(encoder.GetCurrentVelocity(), 20.0f);
    EXPECT_FLOAT_EQ(encoder.GetCurrentVelocity(), 20.0f);
}

// 4. Проверка безопасности (Fail-Safe) при отказе железа
TEST_F(EncoderTest, FailsafeOnHardwareDeath) {
    Encoder encoder(mock_hal);
    // Разгоняемся до 50
    mock_hal.SetRawVelocity(50.0f);
    for(int i = 0; i < 30; i++) encoder.UpdateState();
    EXPECT_NEAR(encoder.GetCurrentVelocity(), 50.0f, 0.1f);
    EXPECT_TRUE(encoder.IsAlive());
    
    // Имитируем отвал провода датчика
    mock_hal.SetIsAlive(false);
    encoder.UpdateState();
    
    // Энкодер должен понять, что он умер, и СБРОСИТЬ скорость в 0.0
    EXPECT_FALSE(encoder.IsAlive());
    EXPECT_FLOAT_EQ(encoder.GetCurrentVelocity(), 0.0f);
}

// 5. Проверка восстановления после сбоя
TEST_F(EncoderTest, HardwareRecovery) {
    Encoder encoder(mock_hal);
    // Убиваем датчик
    mock_hal.SetIsAlive(false);
    encoder.UpdateState();
    EXPECT_FALSE(encoder.IsAlive());
    
    // "Чиним" датчик и сразу едем 100
    mock_hal.SetIsAlive(true);
    mock_hal.SetRawVelocity(100.0f);
    encoder.UpdateState(); 
    
    // Так как скорость была обнулена в состоянии Dead, 
    // новый отсчет: 0.0 * 0.8 + 100.0 * 0.2 = 20.0
    EXPECT_TRUE(encoder.IsAlive());
    EXPECT_FLOAT_EQ(encoder.GetCurrentVelocity(), 20.0f);
}

// --- MAIN ---
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


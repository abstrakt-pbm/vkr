#include <robot/imu.hpp>
#include <hal/hal_imu.hpp>
#include <gtest/gtest.h>

#include "mocks/hal/mock_imu.hpp"  // MockIMotorHAL

using namespace Robot;

class IMUTest : public ::testing::Test {
protected:
    HAL::MockIImuHAL mock_hal;

    // Предполагаем, что внутри IMU.cpp: kGyroAlpha = 0.08f, kAccelAlpha = 0.12f
    // Для тестов будем ожидать именно эту математику.
    const float kGyroAlpha = 0.08f;
    const float kAccelAlpha = 0.12f;

    void SetUp() override {
        mock_hal.SetIsAlive(true);

        mock_hal.SetRawGyroX(0.0f);
        mock_hal.SetRawGyroY(0.0f);
        mock_hal.SetRawGyroZ(0.0f);

        mock_hal.SetAccelerationX(0.0f);
        mock_hal.SetAccelerationY(0.0f);
        mock_hal.SetAccelerationZ(0.0f);
    }
};

// --- ТЕСТЫ ---

// 1. Проверка математики: один шаг фильтра (Gyro и Accel имеют разные Alpha)
TEST_F(IMUTest, SingleStepFilterMath) {
    IMU imu(mock_hal);
	mock_hal.SetRawGyroX(100.0f);
    mock_hal.SetRawGyroY(0.0f);
    mock_hal.SetRawGyroZ(0.0f);

	mock_hal.SetAccelerationX(0.0f);
    mock_hal.SetAccelerationY(0.0f);
    mock_hal.SetAccelerationZ(50.0f);
    
    imu.UpdateState(); // 1-й такт
    
    // Gyro: 0.0 * 0.92 + 100.0 * 0.08 = 8.0
    // Accel: 0.0 * 0.88 + 50.0 * 0.12 = 6.0
    EXPECT_FLOAT_EQ(imu.GetGyroX(), 8.0f);
    EXPECT_FLOAT_EQ(imu.GetAccelerationZ(), 6.0f);
    
    // Остальные оси должны остаться нулями
    EXPECT_FLOAT_EQ(imu.GetGyroY(), 0.0f);
    EXPECT_FLOAT_EQ(imu.GetAccelerationX(), 0.0f);
}

// 2. Проверка независимости осей: шум по одной оси не влияет на другие
TEST_F(IMUTest, AxesAreIndependent) {
    IMU imu(mock_hal);
    mock_hal.SetRawGyroX(0.0f);
    mock_hal.SetRawGyroY(200.0f);
    mock_hal.SetRawGyroZ(0.0f);

    for(int i = 0; i < 100; i++) imu.UpdateState();
    
    EXPECT_NEAR(imu.GetGyroY(), 200.0f, 1.0f);
    EXPECT_FLOAT_EQ(imu.GetGyroX(), 0.0f);
    EXPECT_FLOAT_EQ(imu.GetGyroZ(), 0.0f);
}

// 3. Проверка сходимости фильтра
TEST_F(IMUTest, FilterConvergesOverTime) {
    IMU imu(mock_hal);
	mock_hal.SetRawGyroX(0.0f);
    mock_hal.SetRawGyroY(0.0f);
    mock_hal.SetRawGyroZ(50.0f);

	mock_hal.SetAccelerationX(0.0f);
    mock_hal.SetAccelerationY(0.0f);
    mock_hal.SetAccelerationZ(1.0f);

    // Прогоняем 100 итераций.
    for(int i = 0; i < 100; i++) {
        imu.UpdateState();
    }
    
    EXPECT_NEAR(imu.GetGyroZ(), 50.0f, 0.1f);
    EXPECT_NEAR(imu.GetAccelerationZ(), 1.0f, 0.01f);
}

// 4. Проверка Const Correctness (Геттеры не меняют состояние)
TEST_F(IMUTest, GettersHaveNoSideEffects) {
    IMU imu(mock_hal);
	mock_hal.SetRawGyroX(100.0f);
    mock_hal.SetRawGyroY(0.0f);
    mock_hal.SetRawGyroZ(0.0f);

    imu.UpdateState(); // Шаг 1: GyroX = 8.0
    
    EXPECT_FLOAT_EQ(imu.GetGyroX(), 8.0f);
    EXPECT_FLOAT_EQ(imu.GetGyroX(), 8.0f);
    EXPECT_FLOAT_EQ(imu.GetGyroX(), 8.0f);
}

// 5. Проверка безопасности (Fail-Safe) при отвале I2C
TEST_F(IMUTest, FailsafeOnHardwareDeath) {
    IMU imu(mock_hal);
    
    // Выходим на рабочие значения (робот едет и поворачивает)
	mock_hal.SetRawGyroX(0.0f);
    mock_hal.SetRawGyroY(0.0f);
    mock_hal.SetRawGyroZ(10.0f);

	mock_hal.SetAccelerationX(1.0f);
    mock_hal.SetAccelerationY(0.0f);
    mock_hal.SetAccelerationZ(1.0f);

    for(int i = 0; i < 100; i++) imu.UpdateState();
    
    EXPECT_NEAR(imu.GetGyroZ(), 10.0f, 0.1f);
    EXPECT_NEAR(imu.GetAccelerationX(), 1.0f, 0.1f);
    EXPECT_TRUE(imu.IsAlive());
    
    // Имитируем отвал I2C провода MPU6050
    mock_hal.SetIsAlive(false);
    imu.UpdateState();
    
    // Ожидаемое поведение:
    // 1. IsAlive() возвращает false
    // 2. Accel сбрасывается в 0 немедленно (чтобы не дать ложный collision)
    // 3. Gyro затухает плавно (например, умножаясь на 0.98), а не обрубается в 0, 
    //    чтобы Калман не сошел с ума от резкого скачка.
    EXPECT_FALSE(imu.IsAlive());
    EXPECT_FLOAT_EQ(imu.GetAccelerationX(), 0.0f);
    EXPECT_FLOAT_EQ(imu.GetAccelerationZ(), 0.0f);
    
    EXPECT_FLOAT_EQ(imu.GetGyroZ(), 0.0f); 
}

// 6. Восстановление после сбоя
TEST_F(IMUTest, HardwareRecovery) {
    IMU imu(mock_hal);
    
    // Убиваем датчик (он должен сбросить Accel в нули)
    mock_hal.SetIsAlive(false);
    imu.UpdateState();
    EXPECT_FALSE(imu.IsAlive());
    
    // Чиним датчик, задаем удар по оси X (2.0g)
    mock_hal.SetIsAlive(true);
	mock_hal.SetAccelerationX(2.0f);
    mock_hal.SetAccelerationY(0.0f);
    mock_hal.SetAccelerationZ(0.0f);

    imu.UpdateState(); 
    
    // AccelX: 0.0 * 0.88 + 2.0 * 0.12 = 0.24
    EXPECT_TRUE(imu.IsAlive());
    EXPECT_FLOAT_EQ(imu.GetAccelerationX(), 0.24f);
}

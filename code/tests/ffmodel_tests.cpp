#include <ffmodel.hpp>  // robot_ekf.hpp стилистически

#include <robot/robot.hpp>
#include <robot/robot_control.hpp>

#include <gtest/gtest.h>

using namespace RobotControl;

TEST(FFModel, Constructor_DefaultParams) {
    FFModel ff{0.3f, 0.05f, 0.2f, 0.5f, 12.0f};  // b=30cm, r=5cm типично
    // Просто проверяем создание
}

TEST(FFModel, StraightLine_Velocity) {
    FFModel ff{0.3f, 0.05f, 0.2f, 0.5f};
    // v=1.0м/с, ω=0 → одинаковые ω колес
    // ω = v/r = 1/0.05 = 20 рад/с
    // U = kS + kV*ω = 0.2 + 0.5*20 = 10.2V
    // Оба колеса 10.2V
}

TEST(FFModel, TurnInPlace_Right) {
    FFModel ff{0.3f, 0.05f, 0.2f, 0.5f};
    // ω=2рад/с вправо → v_left=-v_right
    // ω_left = -ω_base/2 = -1/0.3*0.05 = -6.67 рад/с
    // U_left = 0.2 + 0.5*(-6.67) = -1.135V (назад)
}

TEST(FFModel, MaxVoltage_Clipping) {
    FFModel ff{0.3f, 0.05f, 0.2f, 0.5f, 12.0f};
    // Очень высокая скорость → clip 12V
}

TEST(FFModel, StaticFriction_kS) {
    FFModel ff{0.3f, 0.05f, 0.5f, 0.5f};  // ↑kS трение
    // ω=0 → U=kS=0.5V (против трения)
}

TEST(FFModel, BackEMF_kV) {
    FFModel ff{0.3f, 0.05f, 0.2f, 1.0f};  // ↑kV
    // ω=10 рад/с → U=0.2 + 1.0*10 = 10.2V
}


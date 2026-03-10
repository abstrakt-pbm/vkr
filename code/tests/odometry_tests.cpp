#include <robot/odometry.hpp>  // Position + Odometry
#include <gtest/gtest.h>
#include <cmath>

class OdometryTest : public ::testing::Test {
protected:
    static constexpr float DT = 0.001f;
    static constexpr float PI = 3.1415926535f;
    static constexpr float TOLERANCE = 0.01f;
    
    Robot::Odometry odom;
};

TEST_F(OdometryTest, DriveStraight1Meter) {
    constexpr float v = 0.5f;  // м/с
    constexpr int steps = 2000;  // 1м
    
    for(int i = 0; i < steps; ++i)
        odom.Update(v, 0.0f, DT);
    
    auto pos = odom.GetCurrentPosition();
    float dist = sqrt(pos.GetX()*pos.GetX() + pos.GetY()*pos.GetY());
    EXPECT_NEAR(dist, 1.0f, TOLERANCE);
    EXPECT_NEAR(pos.GetY(), 0.0f, TOLERANCE);
    EXPECT_NEAR(pos.GetNormalizedAngle(), 0.0f, 0.01f);
}

TEST_F(OdometryTest, Turn90Deg) {
    constexpr float w = PI / (2.0f * 160 * DT);  // рад/с
    for(int i = 0; i < 160; ++i)
        odom.Update(0.0f, w, DT);
    
    auto pos = odom.GetCurrentPosition();
    EXPECT_NEAR(pos.GetX(), 0.0f, TOLERANCE);
    EXPECT_NEAR(pos.GetY(), 0.0f, TOLERANCE);
    EXPECT_NEAR(pos.GetNormalizedAngle(), PI/2.0f, 0.01f);
}

TEST_F(OdometryTest, SquareReturnsHome) {
    constexpr float v = 0.5f;
    constexpr int steps_side = 1000;  // 0.5м
    constexpr int steps_turn = 160;
    constexpr float w = PI / (2.0f * steps_turn * DT);
    
    for(int side = 0; side < 4; ++side) {
        for(int i = 0; i < steps_side; ++i) odom.Update(v, 0.0f, DT);
        for(int i = 0; i < steps_turn; ++i) odom.Update(0.0f, w, DT);
    }
    
    auto pos = odom.GetCurrentPosition();
    EXPECT_NEAR(pos.GetX(), 0.0f, TOLERANCE);
    EXPECT_NEAR(pos.GetY(), 0.0f, TOLERANCE);
    EXPECT_NEAR(pos.GetNormalizedAngle(), 0.0f, 0.05f);
}


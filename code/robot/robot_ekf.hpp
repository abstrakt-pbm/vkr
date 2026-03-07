#pragma once

#include <cmath>
#include <cstdlib>

namespace Robot {

class RobotEKF {
public:
    RobotEKF() { Reset(); } // Конструктор по умолчанию

    void Predict(float v_left_enc, float v_right_enc);
    void Update(float v_enc, float omega_enc, float gyro_z);

    // Основное для PID/ff
    float GetLinearVelocity() const;
    float GetAngularVelocity() const;
    void Reset();

    void SetQv(float process_noise_v); 
    void SetEncNoise(float enc_noise_v, float enc_noise_omega);
    void SetGyroNoise(float gyro_noise);

    bool IsAlive();

private:	
    float linear_velocity = 0.0f;
    float angular_velocity = 0.0f;

    // Оценка ошибки (Ковариация P)
    float P_v = 1.0f;
    float P_omega = 1.0f;

    // Шум процесса модели (Q - учитывает проскальзывание колес, люфты)
    float Q_v = 0.01f;
    float Q_omega = 0.01f;

    // Шум измерений датчиков (R - подбирается на стенде)
    float R_enc_v = 0.1f;
    float R_enc_omega = 0.1f;
    float R_gyro = 0.05f; // Гироскоп обычно имеет меньше шума на коротких интервалах
};
} // namespace Robot


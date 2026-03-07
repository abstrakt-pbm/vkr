#include <robot/robot_ekf.hpp>

namespace Robot {

void RobotEKF::Predict(float v_left_enc, float v_right_enc) {
    // В базовом фильтре скоростей для диф-привода без управляющего воздействия (u), 
    // мы используем модель постоянной скорости (v_k = v_k-1).
    // Главная задача Predict - увеличить неопределенность P перед шагом Update.
    
    // Примечание: v_left_enc и v_right_enc здесь могут использоваться, 
    // если ты захочешь сделать Predict на основе кинематики (odometry).
    // Но так как Update уже берет данные с энкодеров, здесь мы просто растим ошибку P:
    
    P_v += Q_v;
    P_omega += Q_omega;
}

void RobotEKF::Update(float v_enc, float omega_enc, float gyro_z) {
    // === 1. Обновление линейной скорости (v) ===
    // Используем данные с энкодеров (v_enc)
    float K_v = P_v / (P_v + R_enc_v); // Вычисляем Kalman Gain
    linear_velocity = linear_velocity + K_v * (v_enc - linear_velocity); // Коррекция
    P_v = (1.0f - K_v) * P_v; // Обновление ковариации ошибки

    // === 2. Обновление угловой скорости (omega) ===
    // А) Слияние с данными энкодеров (кинематика вращения)
    float K_omega_enc = P_omega / (P_omega + R_enc_omega);
    angular_velocity = angular_velocity + K_omega_enc * (omega_enc - angular_velocity);
    P_omega = (1.0f - K_omega_enc) * P_omega;

    // Б) Слияние с данными гироскопа (IMU)
    // Последовательное обновление дает отличный fusion двух независимых датчиков
    float K_omega_gyro = P_omega / (P_omega + R_gyro);
    angular_velocity = angular_velocity + K_omega_gyro * (gyro_z - angular_velocity);
    P_omega = (1.0f - K_omega_gyro) * P_omega;
}

float RobotEKF::GetLinearVelocity() const {
    return linear_velocity;
}

float RobotEKF::GetAngularVelocity() const {
    return angular_velocity;
}

void RobotEKF::Reset() {
	linear_velocity = 0.0f;
    angular_velocity = 0.0f;
    
    // Явно задаем параметры здесь, это железобетонно запишет их в память!
    P_v = 1.0f;     
    P_omega = 1.0f; 

    Q_v = 0.01f;
    Q_omega = 0.01f;

    R_enc_v = 0.1f;
    R_enc_omega = 0.1f;
    R_gyro = 0.05f;
}

void RobotEKF::SetQv(float process_noise_v) {
    Q_v = process_noise_v;
    Q_omega = process_noise_v; // Для простоты можно использовать один параметр для обоих
}

void RobotEKF::SetEncNoise(float enc_noise_v, float enc_noise_omega) {
    R_enc_v = enc_noise_v;
    R_enc_omega = enc_noise_omega;
}

void RobotEKF::SetGyroNoise(float gyro_noise) {
    R_gyro = gyro_noise;
}

bool RobotEKF::IsAlive() {
    // Защита от взрыва фильтра (если P или velocity улетят в бесконечность)
    return !std::isnan(linear_velocity) && !std::isnan(angular_velocity) &&
           !std::isnan(P_v) && !std::isnan(P_omega);
}

} // namespace Robot


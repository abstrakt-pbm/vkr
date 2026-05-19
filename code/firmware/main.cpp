#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include <cmath>
#include <cstdint>

#include <robot/hal/hal_encoder.hpp>
#include <robot/hal/hal_imu.hpp>
#include <robot/hal/hal_motor.hpp>

#include <robot/encoder.hpp>
#include <robot/imu.hpp>
#include <robot/motor.hpp>
#include <robot/robot.hpp>
#include <robot/robot_control.hpp>

#include <ffmodel.hpp>
#include <pid.hpp>

#include <robot_hil/protocol.hpp>
#include <robot_hil/session.hpp>
#include <robot_hil_stm32/uart_transport.hpp>

namespace {

constexpr bool kDebugTextEnabled =
    true; // true для miniterm, false для реального HIL-хоста

constexpr float kPi = 3.14159265358979323846f;

// Параметры синхронизированы с simulation/controllers/webots_sil/main.cpp.
constexpr float kWheelRadiusM = 0.025f;
constexpr float kTrackWidthM = 0.100f;

constexpr float kEncoderTicksPerRevolution = 2048.0f;

constexpr float kMaxMotorVoltage = 12.0f;
constexpr float kMinVoltageToStart = 0.0f;
constexpr float kMotorRampCoeff = 200.0f;

constexpr float kFeedForwardKs = 0.06f;
constexpr float kFeedForwardKv = 0.232f;

constexpr float kLinearKp = 0.8f;
constexpr float kLinearKi = 0.4f;
constexpr float kLinearKd = 0.0f;

constexpr float kAngularKp = 0.4f;
constexpr float kAngularKi = 0.2f;
constexpr float kAngularKd = 0.0f;

constexpr float kPidBackCalculation = 1.0f;
constexpr float kPidOutputLimit = 12.0f;

constexpr std::uint32_t kHilTimeoutMs = 100;

UART_HandleTypeDef huart1{};

GPIO_TypeDef *LedPort() { return GPIOC; }

constexpr std::uint16_t kLedPin = GPIO_PIN_13;

void BusyDelay(volatile std::uint32_t count) {
  while (count-- != 0) {
    __NOP();
  }
}

void LedInit() {
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef gpio{};
  gpio.Pin = kLedPin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(LedPort(), &gpio);

  // Black Pill PC13 обычно active-low:
  // SET   -> LED off
  // RESET -> LED on
  HAL_GPIO_WritePin(LedPort(), kLedPin, GPIO_PIN_SET);
}

void LedOn() { HAL_GPIO_WritePin(LedPort(), kLedPin, GPIO_PIN_RESET); }

void LedOff() { HAL_GPIO_WritePin(LedPort(), kLedPin, GPIO_PIN_SET); }

void LedToggle() { HAL_GPIO_TogglePin(LedPort(), kLedPin); }

void ErrorBlink() {
  __disable_irq();

  LedInit();

  while (true) {
    LedToggle();
    BusyDelay(500'000);
  }
}

float ClampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }

  if (value > max_value) {
    return max_value;
  }

  return value;
}

void DebugSendBytes(const std::uint8_t *data, std::uint16_t size) {
  if (!kDebugTextEnabled || data == nullptr || size == 0) {
    return;
  }

  HAL_UART_Transmit(&huart1, const_cast<std::uint8_t *>(data), size, 100);
}

void DebugSendString(const char *str) {
  if (!kDebugTextEnabled || str == nullptr) {
    return;
  }

  std::uint16_t len = 0;

  while (str[len] != '\0') {
    ++len;
  }

  DebugSendBytes(reinterpret_cast<const std::uint8_t *>(str), len);
}

void SystemClock_Config() {
  RCC_OscInitTypeDef osc{};
  RCC_ClkInitTypeDef clk{};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  // HSI = 16 MHz.
  // VCO input  = 16 / 8 = 2 MHz.
  // VCO output = 2 * 100 = 200 MHz.
  // SYSCLK     = 200 / 2 = 100 MHz.
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM = 8;
  osc.PLL.PLLN = 100;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    ErrorBlink();
  }

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) {
    ErrorBlink();
  }
}

void MX_USART1_UART_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitTypeDef gpio{};

  // USART1:
  // PA9  = TX
  // PA10 = RX
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;

  HAL_GPIO_Init(GPIOA, &gpio);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK) {
    ErrorBlink();
  }
}

class HilMotorHAL final : public HAL::IMotorHAL {
public:
  HilMotorHAL(robot_hil::Session &session, robot_hil::MotorId motor_id,
              float max_voltage, float min_voltage_to_start)
      : m_session(session), m_motor_id(motor_id), m_max_voltage(max_voltage),
        m_min_voltage_to_start(min_voltage_to_start) {}

  bool SetRawVoltage(float voltage) override {
    if (!IsAlive()) {
      return false;
    }

    if (!std::isfinite(voltage)) {
      return false;
    }

    m_current_voltage = ClampFloat(voltage, -m_max_voltage, m_max_voltage);
    m_session.SetMotorVoltage(m_motor_id, m_current_voltage);

    return true;
  }

  float GetCurrentRawVoltage() const override { return m_current_voltage; }

  float GetMinVoltageToStart() const override { return m_min_voltage_to_start; }

  float GetMaxVoltage() const override { return m_max_voltage; }

  bool IsAlive() const override {
    return m_session.IsInputValid() && m_max_voltage > 0.0f &&
           std::isfinite(m_max_voltage);
  }

private:
  robot_hil::Session &m_session;
  robot_hil::MotorId m_motor_id;

  float m_current_voltage = 0.0f;
  float m_max_voltage = 12.0f;
  float m_min_voltage_to_start = 0.0f;
};

class HilEncoderHAL final : public HAL::IEncoderHAL {
public:
  HilEncoderHAL(robot_hil::Session &session, robot_hil::MotorId wheel_id,
                float wheel_radius_m, float ticks_per_revolution)
      : m_session(session), m_wheel_id(wheel_id),
        m_wheel_radius_m(wheel_radius_m),
        m_ticks_per_revolution(ticks_per_revolution) {}

  void LatchFromCurrentFrame() {
    const auto &input = m_session.Input();

    const std::int64_t current_ticks = m_wheel_id == robot_hil::MotorId::Left
                                           ? input.left_encoder_ticks
                                           : input.right_encoder_ticks;

    const float dt = m_session.GetDtSeconds();

    if (!m_has_previous_sample || dt <= 0.0f || !std::isfinite(dt)) {
      m_previous_ticks = current_ticks;
      m_linear_velocity_mps = 0.0f;
      m_has_previous_sample = true;
      return;
    }

    const std::int64_t delta_ticks = current_ticks - m_previous_ticks;
    m_previous_ticks = current_ticks;

    const float revolutions =
        static_cast<float>(delta_ticks) / m_ticks_per_revolution;

    const float distance_m = revolutions * 2.0f * kPi * m_wheel_radius_m;

    m_linear_velocity_mps = distance_m / dt;

    if (!std::isfinite(m_linear_velocity_mps)) {
      m_linear_velocity_mps = 0.0f;
    }
  }

  float GetRawLinearVelocity() const override { return m_linear_velocity_mps; }

  bool IsAlive() const override {
    const auto &input = m_session.Input();

    return input.valid &&
           robot_hil::HasSensorFlag(input.valid_mask,
                                    robot_hil::kSensorValidEncoders);
  }

private:
  robot_hil::Session &m_session;
  robot_hil::MotorId m_wheel_id;

  float m_wheel_radius_m = 0.0f;
  float m_ticks_per_revolution = 1.0f;

  std::int64_t m_previous_ticks = 0;
  float m_linear_velocity_mps = 0.0f;
  bool m_has_previous_sample = false;
};

class HilImuHAL final : public HAL::IImuHAL {
public:
  explicit HilImuHAL(robot_hil::Session &session) : m_session(session) {}

  float GetRawGyroX() const override { return 0.0f; }

  float GetRawGyroY() const override { return 0.0f; }

  float GetRawGyroZ() const override {
    const auto &input = m_session.Input();

    if (!input.valid || !std::isfinite(input.imu_omega_z_rad_s)) {
      return 0.0f;
    }

    return input.imu_omega_z_rad_s;
  }

  float GetAccelerationX() const override { return 0.0f; }

  float GetAccelerationY() const override { return 0.0f; }

  float GetAccelerationZ() const override { return 0.0f; }

  bool IsAlive() const override {
    // Для первичного HIL IMU считаем необязательным.
    // Иначе контур может уходить в safe mode, если хост пока шлёт только
    // энкодеры.
    return m_session.IsInputValid();
  }

private:
  robot_hil::Session &m_session;
};

void SendFaultStopFrame(robot_hil::Session &session) {
  session.SetControllerFault();
  session.SetMotorVoltage(robot_hil::MotorId::Left, 0.0f);
  session.SetMotorVoltage(robot_hil::MotorId::Right, 0.0f);
  session.SetDebugVelocity(0.0f, 0.0f);
  session.SendOutput(kHilTimeoutMs);
}

bool InputHasRequiredFields(const robot_hil::Session &session) {
  const auto &input = session.Input();

  if (!input.valid) {
    return false;
  }

  if (!robot_hil::HasSensorFlag(input.valid_mask,
                                robot_hil::kSensorValidEncoders)) {
    return false;
  }

  const float dt = session.GetDtSeconds();

  if (dt <= 0.0f || !std::isfinite(dt)) {
    return false;
  }

  if (!std::isfinite(input.target_v_mps) ||
      !std::isfinite(input.target_w_rad_s)) {
    return false;
  }

  return true;
}

} // namespace

extern "C" void SysTick_Handler(void) { HAL_IncTick(); }

extern "C" void HardFault_Handler(void) { ErrorBlink(); }

int main() {
  HAL_Init();

  SystemClock_Config();
  LedInit();
  MX_USART1_UART_Init();

  DebugSendString("\r\nHIL_STM32_BOOT_OK\r\n");
  DebugSendString("USART1 PA9/PA10, 115200 8N1\r\n");
  DebugSendString("Waiting for valid HIL input frames...\r\n");

  robot_hil_stm32::UartTransport transport(huart1);
  robot_hil::Session hil_session(transport);

  transport.FlushRx(10);

  HilEncoderHAL left_encoder_hal(hil_session, robot_hil::MotorId::Left,
                                 kWheelRadiusM, kEncoderTicksPerRevolution);

  HilEncoderHAL right_encoder_hal(hil_session, robot_hil::MotorId::Right,
                                  kWheelRadiusM, kEncoderTicksPerRevolution);

  HilImuHAL imu_hal(hil_session);

  HilMotorHAL left_motor_hal(hil_session, robot_hil::MotorId::Left,
                             kMaxMotorVoltage, kMinVoltageToStart);

  HilMotorHAL right_motor_hal(hil_session, robot_hil::MotorId::Right,
                              kMaxMotorVoltage, kMinVoltageToStart);

  Robot::IMU imu(imu_hal);

  Robot::Encoder left_encoder(left_encoder_hal);
  Robot::Encoder right_encoder(right_encoder_hal);

  Robot::Motor left_motor(left_motor_hal, kMotorRampCoeff, kMaxMotorVoltage,
                          kMinVoltageToStart);

  Robot::Motor right_motor(right_motor_hal, kMotorRampCoeff, kMaxMotorVoltage,
                           kMinVoltageToStart);

  Robot::ActuatorLimits limits{};

  Robot::RobotKinematics kinematics{};
  kinematics.m_track_width = kTrackWidthM;

  Robot::Robot robot_lib(imu, left_motor, left_encoder, right_motor,
                         right_encoder, limits, kinematics);

  RobotControl::FFModel ff_model(kTrackWidthM, kWheelRadiusM, kFeedForwardKs,
                                 kFeedForwardKv, kMaxMotorVoltage);

  Math::PID linear_pid(kLinearKp, kLinearKi, kLinearKd, kPidBackCalculation,
                       kPidOutputLimit);

  Math::PID angular_pid(kAngularKp, kAngularKi, kAngularKd, kPidBackCalculation,
                        kPidOutputLimit);

  RobotControl::RobotController controller(robot_lib, ff_model, linear_pid,
                                           angular_pid);

  std::uint32_t wait_counter = 0;

  while (true) {
    const bool received = hil_session.ReceiveInput(kHilTimeoutMs);

    if (!received) {
      ++wait_counter;

      if ((wait_counter % 10u) == 0u) {
        LedToggle();
        DebugSendString("WAIT_HIL_FRAME\r\n");
      }

      continue;
    }

    wait_counter = 0;
    LedOn();

    if (!InputHasRequiredFields(hil_session)) {
      DebugSendString("FAULT_BAD_INPUT\r\n");
      SendFaultStopFrame(hil_session);
      LedOff();
      continue;
    }

    const float dt = hil_session.GetDtSeconds();

    left_encoder_hal.LatchFromCurrentFrame();
    right_encoder_hal.LatchFromCurrentFrame();

    robot_lib.UpdateSensors();

    RobotControl::MotionCommand command{};
    command.linear_velocity = hil_session.Input().target_v_mps;
    command.angular_velocity = hil_session.Input().target_w_rad_s;

    const Robot::ControlEffort effort =
        controller.GetAdjustedControlEffort(command, dt);

    robot_lib.TransferToNewState(effort, dt);

    const Robot::RobotState state = robot_lib.m_last_state;

    hil_session.SetDebugVelocity(state.current_linear_speed,
                                 state.current_angular_speed);

    if (robot_lib.IsInSafeMode()) {
      DebugSendString("FAULT_SAFE_MODE\r\n");
      hil_session.SetControllerFault();
    }

    hil_session.SendOutput(kHilTimeoutMs);

    DebugSendString("FRAME_OK\r\n");

    LedToggle();
  }
}

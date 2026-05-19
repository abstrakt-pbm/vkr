#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include <robot_hil/protocol.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846f;

constexpr float kWheelRadiusM = 0.025f;
constexpr float kTrackWidthM = 0.100f;
constexpr float kEncoderTicksPerRevolution = 2048.0f;

constexpr float kMaxMotorVoltage = 12.0f;
constexpr float kWebotsMaxLinearWheelSpeedMps = 1.2f;
constexpr float kWebotsMaxWheelAngularSpeed =
    kWebotsMaxLinearWheelSpeedMps / kWheelRadiusM;

constexpr std::uint32_t kUartTimeoutMs = 100;

constexpr bool kInvertLeftMotor = false;
constexpr bool kInvertRightMotor = false;
constexpr bool kInvertLeftEncoder = false;
constexpr bool kInvertRightEncoder = false;

struct TrajectoryPhase {
  float linear_vel;
  float angular_vel;
  float duration;
};

enum class ScenarioType {
  Straight,
  TurnInPlace,
  Circle,
  Square,
};

struct ScenarioConfig {
  ScenarioType type;
  std::string name;

  float duration_s = 0.0f;
  float linear_vel = 0.0f;
  float angular_vel = 0.0f;
  float radius = 0.0f;

  std::vector<TrajectoryPhase> phases;
};

struct MotionCommand {
  float linear_velocity = 0.0f;
  float angular_velocity = 0.0f;
};

struct TelemetrySample {
  float t = 0.0f;

  float v_cmd = 0.0f;
  float w_cmd = 0.0f;

  float v_meas = 0.0f;
  float w_meas = 0.0f;

  float e_v = 0.0f;
  float e_w = 0.0f;

  float left_voltage = 0.0f;
  float right_voltage = 0.0f;

  double x_gt = 0.0;
  double y_gt = 0.0;
  double yaw_gt = 0.0;

  bool seq_ok = false;
  std::uint8_t status = 0;
};

struct ErrorSummary {
  float mean_abs_ev = 0.0f;
  float mean_abs_ew = 0.0f;

  float max_abs_ev = 0.0f;
  float max_abs_ew = 0.0f;

  float rms_ev = 0.0f;
  float rms_ew = 0.0f;

  float final_position_error = 0.0f;
};

struct Args {
  std::string port = "/dev/ttyUSB0";
  int baudrate = 115200;

  std::string scenario = "square";
  std::string csv = "";

  std::string left_motor = "left_wheel_motor";
  std::string right_motor = "right_wheel_motor";

  std::string left_encoder = "left_wheel_encoder";
  std::string right_encoder = "right_wheel_encoder";

  std::string imu = "imu";
  std::string gps = "gps_ground_truth";
};

float Clamp(float value, float min_value, float max_value) {
  return std::max(min_value, std::min(max_value, value));
}

float WrapToPi(float angle) {
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }

  while (angle <= -kPi) {
    angle += 2.0f * kPi;
  }

  return angle;
}

float WebotsVelocityFromVoltage(float voltage) {
  const float clamped = Clamp(voltage, -kMaxMotorVoltage, kMaxMotorVoltage);
  const float velocity_ratio = clamped / kMaxMotorVoltage;

  return velocity_ratio * kWebotsMaxWheelAngularSpeed;
}

std::int64_t WheelRadiansToTicks(double wheel_position_rad, bool invert) {
  if (invert) {
    wheel_position_rad = -wheel_position_rad;
  }

  const double revolutions =
      wheel_position_rad / (2.0 * static_cast<double>(kPi));

  return static_cast<std::int64_t>(
      std::llround(revolutions * kEncoderTicksPerRevolution));
}

std::string StatusToString(std::uint8_t status) {
  if (status == robot_hil::kControlOk) {
    return "OK";
  }

  std::string result;

  auto add = [&result](const char *text) {
    if (!result.empty()) {
      result += "|";
    }

    result += text;
  };

  if (robot_hil::HasControlStatus(status,
                                  robot_hil::kControlLeftMotorMissing)) {
    add("LEFT_MOTOR_MISSING");
  }

  if (robot_hil::HasControlStatus(status,
                                  robot_hil::kControlRightMotorMissing)) {
    add("RIGHT_MOTOR_MISSING");
  }

  if (robot_hil::HasControlStatus(status, robot_hil::kControlInvalidDt)) {
    add("INVALID_DT");
  }

  if (robot_hil::HasControlStatus(status, robot_hil::kControlControllerFault)) {
    add("CONTROLLER_FAULT");
  }

  const std::uint8_t known =
      static_cast<std::uint8_t>(robot_hil::kControlLeftMotorMissing) |
      static_cast<std::uint8_t>(robot_hil::kControlRightMotorMissing) |
      static_cast<std::uint8_t>(robot_hil::kControlInvalidDt) |
      static_cast<std::uint8_t>(robot_hil::kControlControllerFault);

  const std::uint8_t unknown =
      static_cast<std::uint8_t>(status & static_cast<std::uint8_t>(~known));

  if (unknown != 0u) {
    add("UNKNOWN");
  }

  return result;
}

void SaveCsv(const std::string &filename,
             const std::vector<TelemetrySample> &log) {
  std::ofstream out(filename);

  if (!out.is_open()) {
    std::cerr << "Failed to open CSV file: " << filename << "\n";
    return;
  }

  out << "t,"
      << "v_cmd,w_cmd,"
      << "v_meas,w_meas,"
      << "e_v,e_w,"
      << "left_voltage,right_voltage,"
      << "x_gt,y_gt,yaw_gt,"
      << "seq_ok,status\n";

  out << std::fixed << std::setprecision(6);

  for (const auto &s : log) {
    out << s.t << "," << s.v_cmd << "," << s.w_cmd << "," << s.v_meas << ","
        << s.w_meas << "," << s.e_v << "," << s.e_w << "," << s.left_voltage
        << "," << s.right_voltage << "," << s.x_gt << "," << s.y_gt << ","
        << s.yaw_gt << "," << static_cast<int>(s.seq_ok) << ","
        << static_cast<int>(s.status) << "\n";
  }
}

ErrorSummary ComputeErrorSummary(const std::vector<TelemetrySample> &log) {
  ErrorSummary summary{};

  if (log.empty()) {
    return summary;
  }

  float sum_abs_ev = 0.0f;
  float sum_abs_ew = 0.0f;

  float sum_sq_ev = 0.0f;
  float sum_sq_ew = 0.0f;

  for (const auto &sample : log) {
    const float abs_ev = std::fabs(sample.e_v);
    const float abs_ew = std::fabs(sample.e_w);

    sum_abs_ev += abs_ev;
    sum_abs_ew += abs_ew;

    sum_sq_ev += sample.e_v * sample.e_v;
    sum_sq_ew += sample.e_w * sample.e_w;

    summary.max_abs_ev = std::max(summary.max_abs_ev, abs_ev);
    summary.max_abs_ew = std::max(summary.max_abs_ew, abs_ew);
  }

  const float n = static_cast<float>(log.size());

  summary.mean_abs_ev = sum_abs_ev / n;
  summary.mean_abs_ew = sum_abs_ew / n;

  summary.rms_ev = std::sqrt(sum_sq_ev / n);
  summary.rms_ew = std::sqrt(sum_sq_ew / n);

  const double x0 = log.front().x_gt;
  const double y0 = log.front().y_gt;
  const double x1 = log.back().x_gt;
  const double y1 = log.back().y_gt;

  summary.final_position_error = static_cast<float>(
      std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)));

  return summary;
}

void PrintSummary(const std::string &scenario_name,
                  const ErrorSummary &summary) {
  std::cout << std::fixed << std::setprecision(6);

  std::cout << "\n=== " << scenario_name << " HIL summary ===\n";
  std::cout << "mean |e_v| = " << summary.mean_abs_ev << " m/s\n";
  std::cout << "mean |e_w| = " << summary.mean_abs_ew << " rad/s\n";
  std::cout << "max |e_v| = " << summary.max_abs_ev << " m/s\n";
  std::cout << "max |e_w| = " << summary.max_abs_ew << " rad/s\n";
  std::cout << "rms e_v = " << summary.rms_ev << " m/s\n";
  std::cout << "rms e_w = " << summary.rms_ew << " rad/s\n";
  std::cout << "final pos error = " << summary.final_position_error << " m\n";
}

MotionCommand CommandFromScenario(const ScenarioConfig &scenario, float t,
                                  std::size_t phase_idx) {
  switch (scenario.type) {
  case ScenarioType::Straight:
    return {scenario.linear_vel, 0.0f};

  case ScenarioType::TurnInPlace:
    return {0.0f, scenario.angular_vel};

  case ScenarioType::Circle:
    return {scenario.linear_vel, scenario.angular_vel};

  case ScenarioType::Square:
    if (phase_idx >= scenario.phases.size()) {
      return {0.0f, 0.0f};
    }

    (void)t;
    return {scenario.phases[phase_idx].linear_vel,
            scenario.phases[phase_idx].angular_vel};
  }

  return {0.0f, 0.0f};
}

bool AdvanceSquarePhaseIfNeeded(const ScenarioConfig &scenario,
                                std::size_t &phase_idx, float &phase_time,
                                float dt) {
  if (scenario.type != ScenarioType::Square || scenario.phases.empty()) {
    return false;
  }

  phase_time += dt;

  while (phase_idx < scenario.phases.size() &&
         phase_time >= scenario.phases[phase_idx].duration) {
    phase_time -= scenario.phases[phase_idx].duration;
    ++phase_idx;
  }

  return phase_idx >= scenario.phases.size();
}

ScenarioConfig MakeStraightScenario(float v_cmd, float duration_s) {
  ScenarioConfig cfg{};
  cfg.type = ScenarioType::Straight;
  cfg.name = "straight";
  cfg.linear_vel = v_cmd;
  cfg.duration_s = duration_s;

  return cfg;
}

ScenarioConfig MakeTurnScenario(float w_cmd, float duration_s) {
  ScenarioConfig cfg{};
  cfg.type = ScenarioType::TurnInPlace;
  cfg.name = "turn_in_place";
  cfg.angular_vel = w_cmd;
  cfg.duration_s = duration_s;

  return cfg;
}

ScenarioConfig MakeCircleScenario(float radius, float v_cmd, float duration_s) {
  ScenarioConfig cfg{};
  cfg.type = ScenarioType::Circle;
  cfg.name = "circle";
  cfg.radius = radius;
  cfg.linear_vel = v_cmd;
  cfg.angular_vel = v_cmd / radius;
  cfg.duration_s = duration_s;

  return cfg;
}

ScenarioConfig MakeSquareScenario() {
  ScenarioConfig cfg{};
  cfg.type = ScenarioType::Square;
  cfg.name = "square";

  const float turn_time = 0.784f;
  const float stop_time = 0.50f;

  cfg.phases = {
      {0.20f, 0.0f, 5.0f},      {0.00f, 0.0f, stop_time},
      {0.00f, 2.0f, turn_time}, {0.00f, 0.0f, stop_time},

      {0.20f, 0.0f, 5.0f},      {0.00f, 0.0f, stop_time},
      {0.00f, 2.0f, turn_time}, {0.00f, 0.0f, stop_time},

      {0.20f, 0.0f, 5.0f},      {0.00f, 0.0f, stop_time},
      {0.00f, 2.0f, turn_time}, {0.00f, 0.0f, stop_time},

      {0.20f, 0.0f, 5.0f},      {0.00f, 0.0f, stop_time},
      {0.00f, 2.0f, turn_time}, {0.00f, 0.0f, stop_time},
  };

  return cfg;
}

ScenarioConfig MakeScenarioByName(const std::string &name) {
  if (name == "straight") {
    return MakeStraightScenario(0.2f, 10.0f);
  }

  if (name == "turn" || name == "turn_in_place") {
    return MakeTurnScenario(1.0f, 10.0f);
  }

  if (name == "circle") {
    return MakeCircleScenario(5.0f, 0.2f, 158.0f);
  }

  if (name == "square") {
    return MakeSquareScenario();
  }

  throw std::runtime_error("unknown scenario: " + name);
}

class SerialPort {
public:
  SerialPort(const std::string &path, int baudrate) {
    m_fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (m_fd < 0) {
      throw std::runtime_error("open(" + path +
                               ") failed: " + std::strerror(errno));
    }

    Configure(baudrate);
    ::tcflush(m_fd, TCIOFLUSH);
  }

  ~SerialPort() {
    if (m_fd >= 0) {
      ::close(m_fd);
    }
  }

  SerialPort(const SerialPort &) = delete;
  SerialPort &operator=(const SerialPort &) = delete;

  bool WriteAll(const std::uint8_t *data, std::size_t size,
                std::uint32_t timeout_ms) {
    if (data == nullptr) {
      return false;
    }

    std::size_t written = 0;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (written < size) {
      const auto now = std::chrono::steady_clock::now();

      if (now >= deadline) {
        return false;
      }

      const long remaining_ms = static_cast<long>(
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
              .count());

      fd_set write_set;
      FD_ZERO(&write_set);
      FD_SET(m_fd, &write_set);

      timeval tv{};
      tv.tv_sec = remaining_ms / 1000;
      tv.tv_usec = (remaining_ms % 1000) * 1000;

      const int ready = ::select(m_fd + 1, nullptr, &write_set, nullptr, &tv);

      if (ready <= 0) {
        return false;
      }

      const ssize_t n = ::write(m_fd, data + written, size - written);

      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }

        return false;
      }

      written += static_cast<std::size_t>(n);
    }

    return true;
  }

  bool ReadExact(std::uint8_t *data, std::size_t size,
                 std::uint32_t timeout_ms) {
    if (data == nullptr) {
      return false;
    }

    std::size_t read_total = 0;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (read_total < size) {
      const auto now = std::chrono::steady_clock::now();

      if (now >= deadline) {
        return false;
      }

      const long remaining_ms = static_cast<long>(
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
              .count());

      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(m_fd, &read_set);

      timeval tv{};
      tv.tv_sec = remaining_ms / 1000;
      tv.tv_usec = (remaining_ms % 1000) * 1000;

      const int ready = ::select(m_fd + 1, &read_set, nullptr, nullptr, &tv);

      if (ready <= 0) {
        return false;
      }

      const ssize_t n = ::read(m_fd, data + read_total, size - read_total);

      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }

        return false;
      }

      if (n == 0) {
        return false;
      }

      read_total += static_cast<std::size_t>(n);
    }

    return true;
  }

private:
  static speed_t BaudToTermios(int baudrate) {
    switch (baudrate) {
    case 9600:
      return B9600;

    case 19200:
      return B19200;

    case 38400:
      return B38400;

    case 57600:
      return B57600;

    case 115200:
      return B115200;

    default:
      throw std::runtime_error("unsupported baudrate: " +
                               std::to_string(baudrate));
    }
  }

  void Configure(int baudrate) {
    termios tty{};

    if (::tcgetattr(m_fd, &tty) != 0) {
      throw std::runtime_error("tcgetattr failed: " +
                               std::string(std::strerror(errno)));
    }

    ::cfmakeraw(&tty);

    const speed_t speed = BaudToTermios(baudrate);

    ::cfsetispeed(&tty, speed);
    ::cfsetospeed(&tty, speed);

    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);

    tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(m_fd, TCSANOW, &tty) != 0) {
      throw std::runtime_error("tcsetattr failed: " +
                               std::string(std::strerror(errno)));
    }
  }

private:
  int m_fd = -1;
};

class HilHostLink {
public:
  explicit HilHostLink(SerialPort &serial) : m_serial(serial) {}

  bool SendSensorFrame(robot_hil::SensorFrame frame, std::uint32_t timeout_ms) {
    robot_hil::FillPacketCrc(frame);

    return m_serial.WriteAll(reinterpret_cast<const std::uint8_t *>(&frame),
                             sizeof(frame), timeout_ms);
  }

  bool ReceiveControlFrame(robot_hil::ControlFrame &frame,
                           std::uint32_t timeout_ms) {
    std::array<std::uint8_t, sizeof(robot_hil::ControlFrame)> buffer{};

    if (!SyncToControlMagic(timeout_ms, buffer[0], buffer[1])) {
      return false;
    }

    if (!m_serial.ReadExact(buffer.data() + 2, buffer.size() - 2, timeout_ms)) {
      return false;
    }

    std::memcpy(&frame, buffer.data(), sizeof(frame));

    if (frame.magic != robot_hil::kControlMagic) {
      return false;
    }

    if (frame.version != robot_hil::kProtocolVersion) {
      return false;
    }

    if (!robot_hil::VerifyPacketCrc(frame)) {
      return false;
    }

    return true;
  }

private:
  bool SyncToControlMagic(std::uint32_t timeout_ms, std::uint8_t &first,
                          std::uint8_t &second) {
    const std::uint8_t magic_lo =
        static_cast<std::uint8_t>(robot_hil::kControlMagic & 0xFFu);

    const std::uint8_t magic_hi =
        static_cast<std::uint8_t>((robot_hil::kControlMagic >> 8u) & 0xFFu);

    bool matched_low = false;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
      std::uint8_t byte = 0;

      if (!m_serial.ReadExact(&byte, 1, 10)) {
        continue;
      }

      if (!matched_low) {
        matched_low = byte == magic_lo;
        continue;
      }

      if (byte == magic_hi) {
        first = magic_lo;
        second = magic_hi;
        return true;
      }

      matched_low = byte == magic_lo;
    }

    return false;
  }

private:
  SerialPort &m_serial;
};

Args ParseArgs(int argc, char **argv) {
  Args args{};

  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];

    auto require_value = [&](const char *name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + name);
      }

      ++i;
      return argv[i];
    };

    if (key == "--port") {
      args.port = require_value("--port");
    } else if (key == "--baud") {
      args.baudrate = std::stoi(require_value("--baud"));
    } else if (key == "--scenario") {
      args.scenario = require_value("--scenario");
    } else if (key == "--csv") {
      args.csv = require_value("--csv");
    } else if (key == "--left-motor") {
      args.left_motor = require_value("--left-motor");
    } else if (key == "--right-motor") {
      args.right_motor = require_value("--right-motor");
    } else if (key == "--left-encoder") {
      args.left_encoder = require_value("--left-encoder");
    } else if (key == "--right-encoder") {
      args.right_encoder = require_value("--right-encoder");
    } else if (key == "--imu") {
      args.imu = require_value("--imu");
    } else if (key == "--gps") {
      args.gps = require_value("--gps");
    } else {
      throw std::runtime_error("unknown argument: " + key);
    }
  }

  return args;
}

template <typename T> T *RequireDevice(T *device, const std::string &name) {
  if (device == nullptr) {
    throw std::runtime_error("Webots device not found: " + name);
  }

  return device;
}

TelemetrySample MakeTelemetrySample(float global_time, const MotionCommand &cmd,
                                    const robot_hil::ControlFrame &control,
                                    webots::GPS *gps, float yaw) {
  TelemetrySample sample{};

  sample.t = global_time;

  sample.v_cmd = cmd.linear_velocity;
  sample.w_cmd = cmd.angular_velocity;

  sample.v_meas = gps != nullptr ? static_cast<float>(gps->getSpeed())
                                 : control.debug_v_mps;

  sample.w_meas = control.debug_w_rad_s;

  sample.e_v = sample.v_cmd - sample.v_meas;
  sample.e_w = sample.w_cmd - sample.w_meas;

  sample.left_voltage = control.left_motor_voltage;
  sample.right_voltage = control.right_motor_voltage;

  sample.yaw_gt = yaw;

  if (gps != nullptr) {
    const double *pos = gps->getValues();

    if (pos != nullptr) {
      sample.x_gt = pos[0];
      sample.y_gt = pos[1];
    }
  }

  sample.seq_ok = true;
  sample.status = control.status;

  return sample;
}

} // namespace

int main(int argc, char **argv) {
  try {
    const Args args = ParseArgs(argc, argv);
    const ScenarioConfig scenario = MakeScenarioByName(args.scenario);

    webots::Robot robot{};

    const int step_ms = static_cast<int>(robot.getBasicTimeStep());
    const float dt = static_cast<float>(step_ms) / 1000.0f;
    const std::uint32_t dt_us = static_cast<std::uint32_t>(step_ms) * 1000u;

    webots::Motor *left_motor =
        RequireDevice(robot.getMotor(args.left_motor), args.left_motor);

    webots::Motor *right_motor =
        RequireDevice(robot.getMotor(args.right_motor), args.right_motor);

    webots::PositionSensor *left_encoder = RequireDevice(
        robot.getPositionSensor(args.left_encoder), args.left_encoder);

    webots::PositionSensor *right_encoder = RequireDevice(
        robot.getPositionSensor(args.right_encoder), args.right_encoder);

    webots::InertialUnit *imu =
        RequireDevice(robot.getInertialUnit(args.imu), args.imu);

    webots::GPS *gps = robot.getGPS(args.gps);

    left_encoder->enable(step_ms);
    right_encoder->enable(step_ms);
    imu->enable(step_ms);

    if (gps != nullptr) {
      gps->enable(step_ms);
      std::cout << "[HIL] GPS ground truth enabled\n";
    } else {
      std::cout << "[HIL] GPS ground truth not found, continue without GPS\n";
    }

    left_motor->setPosition(std::numeric_limits<double>::infinity());
    right_motor->setPosition(std::numeric_limits<double>::infinity());

    left_motor->setVelocity(0.0);
    right_motor->setVelocity(0.0);

    SerialPort serial(args.port, args.baudrate);
    HilHostLink link(serial);

    std::cout << "[HIL] webots_hil started\n";
    std::cout << "[HIL] port=" << args.port << " baud=" << args.baudrate
              << " scenario=" << scenario.name << " step_ms=" << step_ms
              << "\n";

    std::vector<TelemetrySample> log;
    log.reserve(4096);

    std::uint16_t seq = 0;
    float global_time = 0.0f;
    int tick = 0;

    std::size_t phase_idx = 0;
    float phase_time = 0.0f;

    float previous_yaw = 0.0f;
    bool has_previous_yaw = false;

    while (robot.step(step_ms) != -1) {
      global_time += dt;

      if (scenario.type == ScenarioType::Square) {
        if (phase_idx >= scenario.phases.size()) {
          break;
        }
      } else {
        if (global_time > scenario.duration_s) {
          break;
        }
      }

      const MotionCommand cmd =
          CommandFromScenario(scenario, global_time, phase_idx);

      const std::int64_t left_ticks =
          WheelRadiansToTicks(left_encoder->getValue(), kInvertLeftEncoder);

      const std::int64_t right_ticks =
          WheelRadiansToTicks(right_encoder->getValue(), kInvertRightEncoder);

      const double *rpy = imu->getRollPitchYaw();
      const float yaw = rpy != nullptr ? static_cast<float>(rpy[2]) : 0.0f;

      float omega_z = 0.0f;

      if (has_previous_yaw && dt > 0.0f) {
        omega_z = WrapToPi(yaw - previous_yaw) / dt;
      }

      previous_yaw = yaw;
      has_previous_yaw = true;

      robot_hil::SensorFrame sensor{};

      sensor.magic = robot_hil::kSensorMagic;
      sensor.version = robot_hil::kProtocolVersion;
      sensor.seq = seq;
      sensor.time_us = static_cast<std::uint32_t>(global_time * 1'000'000.0f);
      sensor.dt_us = dt_us;

      sensor.valid_mask =
          static_cast<std::uint8_t>(robot_hil::kSensorValidEncoders) |
          static_cast<std::uint8_t>(robot_hil::kSensorValidImu) |
          static_cast<std::uint8_t>(robot_hil::kSensorValidCommand);

      sensor.left_encoder_ticks = left_ticks;
      sensor.right_encoder_ticks = right_ticks;

      sensor.imu_yaw_rad = yaw;
      sensor.imu_omega_z_rad_s = omega_z;

      sensor.target_v_mps = cmd.linear_velocity;
      sensor.target_w_rad_s = cmd.angular_velocity;

      if (!link.SendSensorFrame(sensor, kUartTimeoutMs)) {
        std::cerr << "[HIL] send SensorFrame failed, seq=" << seq << "\n";

        left_motor->setVelocity(0.0);
        right_motor->setVelocity(0.0);

        seq = static_cast<std::uint16_t>(seq + 1u);
        continue;
      }

      robot_hil::ControlFrame control{};

      if (!link.ReceiveControlFrame(control, kUartTimeoutMs)) {
        std::cerr << "[HIL] receive ControlFrame failed, seq=" << seq << "\n";

        left_motor->setVelocity(0.0);
        right_motor->setVelocity(0.0);

        seq = static_cast<std::uint16_t>(seq + 1u);
        continue;
      }

      const bool seq_ok = control.seq == seq;

      float left_velocity =
          WebotsVelocityFromVoltage(control.left_motor_voltage);

      float right_velocity =
          WebotsVelocityFromVoltage(control.right_motor_voltage);

      if (kInvertLeftMotor) {
        left_velocity = -left_velocity;
      }

      if (kInvertRightMotor) {
        right_velocity = -right_velocity;
      }

      left_motor->setVelocity(left_velocity);
      right_motor->setVelocity(right_velocity);

      TelemetrySample sample =
          MakeTelemetrySample(global_time, cmd, control, gps, yaw);

      sample.seq_ok = seq_ok;
      sample.status = control.status;

      log.push_back(sample);

      if (tick++ % 15 == 0) {
        std::cout << std::fixed << std::setprecision(3) << "t=" << sample.t
                  << " | seq=" << seq << " | seq_ok=" << seq_ok
                  << " | status=" << StatusToString(control.status)
                  << " | v_cmd=" << sample.v_cmd
                  << " | v_meas=" << sample.v_meas << " | e_v=" << sample.e_v
                  << " | w_cmd=" << sample.w_cmd
                  << " | w_meas=" << sample.w_meas << " | e_w=" << sample.e_w
                  << " | U_L=" << sample.left_voltage
                  << " | U_R=" << sample.right_voltage
                  << " | wl=" << left_velocity << " | wr=" << right_velocity;

        if (gps != nullptr) {
          std::cout << " | x=" << sample.x_gt << " | y=" << sample.y_gt;
        }

        std::cout << "\n";
      }

      if (scenario.type == ScenarioType::Square) {
        const bool finished =
            AdvanceSquarePhaseIfNeeded(scenario, phase_idx, phase_time, dt);

        if (finished) {
          break;
        }
      }

      seq = static_cast<std::uint16_t>(seq + 1u);
    }

    left_motor->setVelocity(0.0);
    right_motor->setVelocity(0.0);

    const ErrorSummary summary = ComputeErrorSummary(log);
    PrintSummary(scenario.name, summary);

    const std::string csv_name =
        args.csv.empty() ? (scenario.name + "_hil.csv") : args.csv;

    SaveCsv(csv_name, log);

    std::cout << "[HIL] saved CSV: " << csv_name << "\n";

    return 0;
  } catch (const std::exception &e) {
    std::cerr << "[HIL] fatal: " << e.what() << "\n";
    return 1;
  }
}

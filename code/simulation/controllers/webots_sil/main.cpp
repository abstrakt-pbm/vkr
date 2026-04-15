#include <webots_sil/webots_encoder.hpp>
#include <webots_sil/webots_imu.hpp>
#include <webots_sil/webots_motor.hpp>

#include <webots/GPS.hpp>
#include <webots/Robot.hpp>

#include <robot/encoder.hpp>
#include <robot/imu.hpp>
#include <robot/motor.hpp>
#include <robot/robot.hpp>
#include <robot/robot_control.hpp>

#include <ffmodel.hpp>
#include <pid.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

struct TrajectoryPhase {
  float linear_vel;  // м/с
  float angular_vel; // рад/с
  float duration;    // сек
};

enum class ScenarioType { Straight, TurnInPlace, Circle, Square };

struct ScenarioConfig {
  ScenarioType type;
  std::string name;
  float duration_s = 0.0f;

  // Для straight / turn / circle
  float linear_vel = 0.0f;
  float angular_vel = 0.0f;

  // Для circle
  float radius = 0.0f;

  // Для square
  std::vector<TrajectoryPhase> phases;
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
  double yaw_gt = 0.0; // если потом появится источник yaw ground truth
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

static float wrap_to_pi(float angle) {
  while (angle > static_cast<float>(M_PI)) {
    angle -= 2.0f * static_cast<float>(M_PI);
  }
  while (angle <= -static_cast<float>(M_PI)) {
    angle += 2.0f * static_cast<float>(M_PI);
  }
  return angle;
}

static void save_csv(const std::string &filename,
                     const std::vector<TelemetrySample> &log) {
  std::ofstream out(filename);
  if (!out.is_open()) {
    std::cerr << "Failed to open CSV file: " << filename << std::endl;
    return;
  }

  out << "t,"
      << "v_cmd,w_cmd,"
      << "v_meas,w_meas,"
      << "e_v,e_w,"
      << "left_voltage,right_voltage,"
      << "x_gt,y_gt,yaw_gt\n";

  out << std::fixed << std::setprecision(6);

  for (const auto &s : log) {
    out << s.t << "," << s.v_cmd << "," << s.w_cmd << "," << s.v_meas << ","
        << s.w_meas << "," << s.e_v << "," << s.e_w << "," << s.left_voltage
        << "," << s.right_voltage << "," << s.x_gt << "," << s.y_gt << ","
        << s.yaw_gt << "\n";
  }
}

static ErrorSummary
compute_error_summary(const std::vector<TelemetrySample> &log) {
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

  // Ошибка возврата в начальную точку / конечная позиционная ошибка
  const double x0 = log.front().x_gt;
  const double y0 = log.front().y_gt;
  const double x1 = log.back().x_gt;
  const double y1 = log.back().y_gt;

  summary.final_position_error = static_cast<float>(
      std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)));

  return summary;
}

static void print_summary(const std::string &scenario_name,
                          const ErrorSummary &summary) {
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "\n=== " << scenario_name << " summary ===\n";
  std::cout << "mean |e_v|      = " << summary.mean_abs_ev << " m/s\n";
  std::cout << "mean |e_w|      = " << summary.mean_abs_ew << " rad/s\n";
  std::cout << "max  |e_v|      = " << summary.max_abs_ev << " m/s\n";
  std::cout << "max  |e_w|      = " << summary.max_abs_ew << " rad/s\n";
  std::cout << "rms  e_v        = " << summary.rms_ev << " m/s\n";
  std::cout << "rms  e_w        = " << summary.rms_ew << " rad/s\n";
  std::cout << "final pos error = " << summary.final_position_error << " m\n";
}

static RobotControl::MotionCommand
command_from_scenario(const ScenarioConfig &scenario, float t,
                      std::size_t &phase_idx, float &phase_time) {

  switch (scenario.type) {
  case ScenarioType::Straight:
    return {scenario.linear_vel, 0.0f};

  case ScenarioType::TurnInPlace:
    return {0.0f, scenario.angular_vel};

  case ScenarioType::Circle:
    return {scenario.linear_vel, scenario.angular_vel};

  case ScenarioType::Square: {
    if (scenario.phases.empty()) {
      return {0.0f, 0.0f};
    }

    if (phase_idx >= scenario.phases.size()) {
      return {0.0f, 0.0f};
    }

    const auto &phase = scenario.phases[phase_idx];
    (void)t;
    return {phase.linear_vel, phase.angular_vel};
  }
  }

  return {0.0f, 0.0f};
}

static bool advance_square_phase_if_needed(const ScenarioConfig &scenario,
                                           std::size_t &phase_idx,
                                           float &phase_time, float dt) {
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

static TelemetrySample
make_telemetry_sample(float global_time, const RobotControl::MotionCommand &cmd,
                      const Robot::ControlEffort &effort,
                      Robot::Robot &robot_lib, webots::GPS *gps) {

  TelemetrySample sample{};
  sample.t = global_time;

  sample.v_cmd = cmd.linear_velocity;
  sample.w_cmd = cmd.angular_velocity;

  // Фактическая линейная скорость.
  // Для первого рабочего SIL-варианта можно брать GPS speed.
  sample.v_meas = gps ? static_cast<float>(gps->getSpeed()) : 0.0f;

  // Фактическая угловая скорость.
  // Берём из оценённого состояния системы.
  sample.w_meas = robot_lib.m_last_state.current_angular_speed;

  sample.e_v = sample.v_cmd - sample.v_meas;
  sample.e_w = sample.w_cmd - sample.w_meas;

  sample.left_voltage = effort.left_motor_voltage;
  sample.right_voltage = effort.right_motor_voltage;

  if (gps) {
    const double *pos = gps->getValues();
    sample.x_gt = pos[0];
    sample.y_gt = pos[1];
  }

  return sample;
}

static std::vector<TelemetrySample>
run_sil_scenario(webots::Robot *robot, Robot::Robot &robot_lib,
                 RobotControl::RobotController &controller, webots::GPS *gps,
                 const ScenarioConfig &scenario) {

  std::vector<TelemetrySample> log;
  log.reserve(4096);

  const int step_ms = static_cast<int>(robot->getBasicTimeStep());
  const float dt = static_cast<float>(step_ms) / 1000.0f;

  float global_time = 0.0f;
  int tick = 0;

  std::size_t phase_idx = 0;
  float phase_time = 0.0f;

  std::cout << "\n=== Start scenario: " << scenario.name << " ===\n";

  while (robot->step(step_ms) != -1) {
    global_time += dt;

    // Для square сценария проверяем завершение фаз.
    if (scenario.type == ScenarioType::Square) {
      if (phase_idx >= scenario.phases.size()) {
        break;
      }
    } else {
      if (global_time > scenario.duration_s) {
        break;
      }
    }

    RobotControl::MotionCommand cmd =
        command_from_scenario(scenario, global_time, phase_idx, phase_time);

    robot_lib.UpdateSensors();

    Robot::ControlEffort effort = controller.GetAdjustedControlEffort(cmd, dt);

    robot_lib.TransferToNewState(effort, dt);

    TelemetrySample sample =
        make_telemetry_sample(global_time, cmd, effort, robot_lib, gps);

    log.push_back(sample);

    if (tick++ % 15 == 0) {
      std::cout << std::fixed << std::setprecision(3) << "t=" << sample.t
                << " | v_cmd=" << sample.v_cmd << " | v_meas=" << sample.v_meas
                << " | e_v=" << sample.e_v << " | w_cmd=" << sample.w_cmd
                << " | w_meas=" << sample.w_meas << " | e_w=" << sample.e_w
                << " | U_L=" << sample.left_voltage
                << " | U_R=" << sample.right_voltage;

      if (gps) {
        std::cout << " | x=" << sample.x_gt << " | y=" << sample.y_gt;
      }

      std::cout << "\n";
    }

    if (scenario.type == ScenarioType::Square) {
      const bool finished =
          advance_square_phase_if_needed(scenario, phase_idx, phase_time, dt);
      if (finished) {
        break;
      }
    }
  }

  return log;
}

static ScenarioConfig make_straight_scenario(float v_cmd, float duration_s) {
  ScenarioConfig cfg{};
  cfg.type = ScenarioType::Straight;
  cfg.name = "straight";
  cfg.linear_vel = v_cmd;
  cfg.duration_s = duration_s;
  return cfg;
}

static ScenarioConfig make_turn_scenario(float w_cmd, float duration_s) {
  ScenarioConfig cfg{};
  cfg.type = ScenarioType::TurnInPlace;
  cfg.name = "turn_in_place";
  cfg.angular_vel = w_cmd;
  cfg.duration_s = duration_s;
  return cfg;
}

static ScenarioConfig make_circle_scenario(float radius, float v_cmd,
                                           float duration_s) {
  ScenarioConfig cfg{};
  cfg.type = ScenarioType::Circle;
  cfg.name = "circle";
  cfg.radius = radius;
  cfg.linear_vel = v_cmd;
  cfg.angular_vel = v_cmd / radius;
  cfg.duration_s = duration_s;
  return cfg;
}

static ScenarioConfig make_square_scenario() {
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

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  auto robot = std::shared_ptr<webots::Robot>(new webots::Robot());

  WebotsIImuHAL imu_hal(robot, "imu", "accelerometer");
  WebotsIEncoderHAL encoder_hal_l(robot, "left_wheel_encoder", 0.025f);
  WebotsIEncoderHAL encoder_hal_r(robot, "right_wheel_encoder", 0.025f);
  WebotsIMotorHAL motor_hal_l(robot, "left_wheel_motor");
  WebotsIMotorHAL motor_hal_r(robot, "right_wheel_motor");

  webots::GPS *gps = robot->getGPS("gps_ground_truth");
  if (gps) {
    gps->enable(static_cast<int>(robot->getBasicTimeStep()));
    std::cout << "[GPS] ground truth enabled\n";
  }

  Robot::IMU imu(imu_hal);
  Robot::Encoder enc_l(encoder_hal_l);
  Robot::Encoder enc_r(encoder_hal_r);
  Robot::Motor motor_l(motor_hal_l, 200.0f, 12.0f, 0.0f);
  Robot::Motor motor_r(motor_hal_r, 200.0f, 12.0f, 0.0f);

  Robot::ActuatorLimits limits{};
  Robot::RobotKinematics kinematics{0.100f};

  Robot::Robot robot_lib(imu, motor_l, enc_l, motor_r, enc_r, limits,
                         kinematics);

  RobotControl::FFModel ff_model(0.100f, 0.025f, 0.06f, 0.232f, 12.0f);

  Math::PID lin_pid(0.8f, 0.4f, 0.0f, 1.0f, 12.0f);
  Math::PID ang_pid(0.4f, 0.2f, 0.0f, 1.0f, 12.0f);

  RobotControl::RobotController controller(robot_lib, ff_model, lin_pid,
                                           ang_pid);

  // Выбирай один сценарий на запуск, чтобы логи не смешивались.
  // Потом можно сделать CLI-параметр и выбирать через argv.
  // const ScenarioConfig scenario = make_turn_scenario(1.0f, 10.0f);
  const ScenarioConfig scenario = make_straight_scenario(0.2f, 10.0f);
  // const ScenarioConfig scenario = make_circle_scenario(5.0f, 0.2f, 158.0f);
  // const ScenarioConfig scenario = make_square_scenario();

  std::vector<TelemetrySample> log =
      run_sil_scenario(robot.get(), robot_lib, controller, gps, scenario);

  const ErrorSummary summary = compute_error_summary(log);
  print_summary(scenario.name, summary);

  save_csv(scenario.name + ".csv", log);

  return 0;
}

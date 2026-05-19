#pragma once

#include <cstddef>
#include <cstdint>

namespace robot_hil {

constexpr std::uint16_t kSensorMagic = 0x5348;  // 'H''S'
constexpr std::uint16_t kControlMagic = 0x4348; // 'H''C'

constexpr std::uint8_t kProtocolVersion = 1;

enum SensorValidMask : std::uint8_t {
  kSensorValidNone = 0,
  kSensorValidEncoders = 1u << 0,
  kSensorValidImu = 1u << 1,
  kSensorValidCommand = 1u << 2,
};

enum ControlStatus : std::uint8_t {
  kControlOk = 0,
  kControlLeftMotorMissing = 1u << 0,
  kControlRightMotorMissing = 1u << 1,
  kControlInvalidDt = 1u << 2,
  kControlControllerFault = 1u << 3,
};

enum class MotorId : std::uint8_t {
  Left,
  Right,
};

#pragma pack(push, 1)

struct SensorFrame {
  std::uint16_t magic;
  std::uint8_t version;
  std::uint16_t seq;

  std::uint32_t time_us;
  std::uint32_t dt_us;

  std::uint8_t valid_mask;

  std::int64_t left_encoder_ticks;
  std::int64_t right_encoder_ticks;

  float imu_yaw_rad;
  float imu_omega_z_rad_s;

  float target_v_mps;
  float target_w_rad_s;

  std::uint16_t crc;
};

struct ControlFrame {
  std::uint16_t magic;
  std::uint8_t version;
  std::uint16_t seq;

  float left_motor_voltage;
  float right_motor_voltage;

  float debug_v_mps;
  float debug_w_rad_s;

  std::uint8_t status;

  std::uint16_t crc;
};

#pragma pack(pop)

static_assert(sizeof(SensorFrame) == 48, "Unexpected SensorFrame size");
static_assert(sizeof(ControlFrame) == 24, "Unexpected ControlFrame size");

inline bool HasSensorFlag(std::uint8_t mask, SensorValidMask flag) {
  return (mask & static_cast<std::uint8_t>(flag)) != 0u;
}

inline bool HasControlStatus(std::uint8_t status, ControlStatus flag) {
  return (status & static_cast<std::uint8_t>(flag)) != 0u;
}

inline std::uint16_t Crc16CcittFalse(const std::uint8_t *data,
                                     std::size_t size) {
  std::uint16_t crc = 0xFFFF;

  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;

    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0u) {
        crc = static_cast<std::uint16_t>((crc << 1u) ^ 0x1021u);
      } else {
        crc = static_cast<std::uint16_t>(crc << 1u);
      }
    }
  }

  return crc;
}

template <typename Packet>
inline std::uint16_t CalculatePacketCrc(const Packet &packet) {
  return Crc16CcittFalse(reinterpret_cast<const std::uint8_t *>(&packet),
                         offsetof(Packet, crc));
}

template <typename Packet> inline void FillPacketCrc(Packet &packet) {
  packet.crc = CalculatePacketCrc(packet);
}

template <typename Packet> inline bool VerifyPacketCrc(const Packet &packet) {
  return packet.crc == CalculatePacketCrc(packet);
}

} // namespace robot_hil

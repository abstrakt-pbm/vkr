#pragma once

#include <cstdint>

#include <robot_hil/protocol.hpp>

namespace robot_hil {

class ITransport {
public:
  virtual ~ITransport() = default;

  virtual bool ReceiveSensorFrame(SensorFrame &frame,
                                  std::uint32_t timeout_ms) = 0;

  virtual bool SendControlFrame(const ControlFrame &frame,
                                std::uint32_t timeout_ms) = 0;
};

} // namespace robot_hil

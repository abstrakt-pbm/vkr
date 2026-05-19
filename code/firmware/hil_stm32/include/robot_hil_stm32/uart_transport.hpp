#pragma once

#include <cstddef>
#include <cstdint>

#include <stm32f4xx_hal.h>

#include <robot_hil/protocol.hpp>
#include <robot_hil/transport.hpp>

namespace robot_hil_stm32 {

class UartTransport final : public robot_hil::ITransport {
public:
  explicit UartTransport(UART_HandleTypeDef &uart);

  bool ReceiveSensorFrame(robot_hil::SensorFrame &frame,
                          std::uint32_t timeout_ms) override;

  bool SendControlFrame(const robot_hil::ControlFrame &frame,
                        std::uint32_t timeout_ms) override;

  bool FlushRx(std::uint32_t timeout_ms);

private:
  bool ReadExact(std::uint8_t *data, std::size_t size,
                 std::uint32_t timeout_ms);

  bool WriteExact(const std::uint8_t *data, std::size_t size,
                  std::uint32_t timeout_ms);

  bool ReadByte(std::uint8_t &byte, std::uint32_t timeout_ms);

  bool SyncToMagic(std::uint16_t magic, std::uint32_t timeout_ms);

  static bool IsExpired(std::uint32_t start_tick, std::uint32_t timeout_ms);

  static std::uint32_t RemainingTimeout(std::uint32_t start_tick,
                                        std::uint32_t timeout_ms);

private:
  UART_HandleTypeDef &m_uart;
};

} // namespace robot_hil_stm32

#include <robot_hil_stm32/uart_transport.hpp>

namespace robot_hil_stm32 {

UartTransport::UartTransport(UART_HandleTypeDef &uart) : m_uart(uart) {}

bool UartTransport::ReceiveSensorFrame(robot_hil::SensorFrame &frame,
                                       std::uint32_t timeout_ms) {
  if (!SyncToMagic(robot_hil::kSensorMagic, timeout_ms)) {
    return false;
  }

  frame.magic = robot_hil::kSensorMagic;

  auto *frame_bytes = reinterpret_cast<std::uint8_t *>(&frame);

  constexpr std::size_t already_read = sizeof(frame.magic);

  constexpr std::size_t remaining_size =
      sizeof(robot_hil::SensorFrame) - already_read;

  return ReadExact(frame_bytes + already_read, remaining_size, timeout_ms);
}

bool UartTransport::SendControlFrame(const robot_hil::ControlFrame &frame,
                                     std::uint32_t timeout_ms) {
  return WriteExact(reinterpret_cast<const std::uint8_t *>(&frame),
                    sizeof(robot_hil::ControlFrame), timeout_ms);
}

bool UartTransport::FlushRx(std::uint32_t timeout_ms) {
  const std::uint32_t start_tick = HAL_GetTick();

  std::uint8_t byte = 0;

  while (!IsExpired(start_tick, timeout_ms)) {
    const HAL_StatusTypeDef status = HAL_UART_Receive(&m_uart, &byte, 1, 1);

    if (status == HAL_TIMEOUT) {
      return true;
    }

    if (status != HAL_OK) {
      return false;
    }
  }

  return true;
}

bool UartTransport::ReadExact(std::uint8_t *data, std::size_t size,
                              std::uint32_t timeout_ms) {
  if (data == nullptr && size != 0) {
    return false;
  }

  const std::uint32_t start_tick = HAL_GetTick();

  std::size_t offset = 0;

  while (offset < size) {
    const std::uint32_t remaining_timeout =
        RemainingTimeout(start_tick, timeout_ms);

    if (remaining_timeout == 0u) {
      return false;
    }

    const auto chunk_size = static_cast<std::uint16_t>(size - offset);

    const HAL_StatusTypeDef status =
        HAL_UART_Receive(&m_uart, data + offset, chunk_size, remaining_timeout);

    if (status != HAL_OK) {
      return false;
    }

    offset += chunk_size;
  }

  return true;
}

bool UartTransport::WriteExact(const std::uint8_t *data, std::size_t size,
                               std::uint32_t timeout_ms) {
  if (data == nullptr && size != 0) {
    return false;
  }

  const std::uint32_t start_tick = HAL_GetTick();

  std::size_t offset = 0;

  while (offset < size) {
    const std::uint32_t remaining_timeout =
        RemainingTimeout(start_tick, timeout_ms);

    if (remaining_timeout == 0u) {
      return false;
    }

    const auto chunk_size = static_cast<std::uint16_t>(size - offset);

    const HAL_StatusTypeDef status =
        HAL_UART_Transmit(&m_uart, const_cast<std::uint8_t *>(data + offset),
                          chunk_size, remaining_timeout);

    if (status != HAL_OK) {
      return false;
    }

    offset += chunk_size;
  }

  return true;
}

bool UartTransport::ReadByte(std::uint8_t &byte, std::uint32_t timeout_ms) {
  return HAL_UART_Receive(&m_uart, &byte, 1, timeout_ms) == HAL_OK;
}

bool UartTransport::SyncToMagic(std::uint16_t magic, std::uint32_t timeout_ms) {
  const std::uint8_t magic_lo = static_cast<std::uint8_t>(magic & 0xFFu);

  const std::uint8_t magic_hi =
      static_cast<std::uint8_t>((magic >> 8u) & 0xFFu);

  const std::uint32_t start_tick = HAL_GetTick();

  bool low_byte_matched = false;

  while (!IsExpired(start_tick, timeout_ms)) {
    const std::uint32_t remaining_timeout =
        RemainingTimeout(start_tick, timeout_ms);

    if (remaining_timeout == 0u) {
      return false;
    }

    std::uint8_t byte = 0;

    if (!ReadByte(byte, remaining_timeout)) {
      return false;
    }

    if (!low_byte_matched) {
      if (byte == magic_lo) {
        low_byte_matched = true;
      }

      continue;
    }

    if (byte == magic_hi) {
      return true;
    }

    if (byte == magic_lo) {
      low_byte_matched = true;
    } else {
      low_byte_matched = false;
    }
  }

  return false;
}

bool UartTransport::IsExpired(std::uint32_t start_tick,
                              std::uint32_t timeout_ms) {
  if (timeout_ms == HAL_MAX_DELAY) {
    return false;
  }

  return (HAL_GetTick() - start_tick) >= timeout_ms;
}

std::uint32_t UartTransport::RemainingTimeout(std::uint32_t start_tick,
                                              std::uint32_t timeout_ms) {
  if (timeout_ms == HAL_MAX_DELAY) {
    return HAL_MAX_DELAY;
  }

  const std::uint32_t elapsed = HAL_GetTick() - start_tick;

  if (elapsed >= timeout_ms) {
    return 0u;
  }

  return timeout_ms - elapsed;
}

} // namespace robot_hil_stm32

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include <cstdint>

namespace {

UART_HandleTypeDef huart1{};

constexpr std::uint16_t kLedPin = GPIO_PIN_13;

GPIO_TypeDef *LedPort() { return GPIOC; }

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

  // На большинстве STM32F411 Black Pill PC13 active-low:
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

void SystemClock_Config() {
  RCC_OscInitTypeDef osc{};
  RCC_ClkInitTypeDef clk{};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  // HSI = 16 MHz
  // PLL input  = 16 / 8 = 2 MHz
  // VCO        = 2 * 100 = 200 MHz
  // SYSCLK     = 200 / 2 = 100 MHz
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

void USART1_Init() {
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

void SendBytes(const std::uint8_t *data, std::uint16_t size) {
  if (data == nullptr || size == 0) {
    return;
  }

  const HAL_StatusTypeDef status =
      HAL_UART_Transmit(&huart1, const_cast<std::uint8_t *>(data), size, 100);

  if (status != HAL_OK) {
    ErrorBlink();
  }
}

void SendString(const char *str) {
  if (str == nullptr) {
    return;
  }

  std::uint16_t len = 0;

  while (str[len] != '\0') {
    ++len;
  }

  SendBytes(reinterpret_cast<const std::uint8_t *>(str), len);
}

char HexDigit(std::uint8_t value) {
  value &= 0x0F;

  if (value < 10) {
    return static_cast<char>('0' + value);
  }

  return static_cast<char>('A' + (value - 10));
}

void SendHexByte(std::uint8_t byte) {
  char buffer[] = {'0', 'x', HexDigit(static_cast<std::uint8_t>(byte >> 4)),
                   HexDigit(byte), '\0'};

  SendString(buffer);
}

bool IsPrintable(std::uint8_t byte) { return byte >= 32 && byte <= 126; }

bool TryReadByte(std::uint8_t *out_byte) {
  if (out_byte == nullptr) {
    return false;
  }

  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == RESET) {
    return false;
  }

  *out_byte = static_cast<std::uint8_t>(huart1.Instance->DR & 0xFFu);
  return true;
}

void SendReceivedByte(std::uint8_t byte) {
  SendString("RX: ");

  if (IsPrintable(byte)) {
    SendBytes(&byte, 1);
  } else if (byte == '\r') {
    SendString("\\r");
  } else if (byte == '\n') {
    SendString("\\n");
  } else {
    SendString(".");
  }

  SendString(" ");
  SendHexByte(byte);
  SendString("\r\n");
}

} // namespace

extern "C" void SysTick_Handler(void) { HAL_IncTick(); }

extern "C" void HardFault_Handler(void) { ErrorBlink(); }

int main() {
  HAL_Init();

  SystemClock_Config();
  LedInit();
  USART1_Init();

  SendString("\r\nBOOT_OK STM32F411 USART1 PA9_PA10 LED_PC13\r\n");
  SendString("PA9  USART1_TX -> USB-TTL RXD\r\n");
  SendString("PA10 USART1_RX <- USB-TTL TXD\r\n");
  SendString("GND             -> USB-TTL GND\r\n");
  SendString("Terminal: 115200 8N1, no flow control\r\n");
  SendString("Type characters to test RX echo.\r\n\r\n");

  std::uint32_t loop_counter = 0;

  while (true) {
    std::uint8_t byte = 0;

    if (TryReadByte(&byte)) {
      LedOn();
      SendReceivedByte(byte);
      BusyDelay(300'000);
      LedOff();
    }

    ++loop_counter;

    if (loop_counter >= 1'000'000u) {
      loop_counter = 0;

      LedToggle();
      SendString("UART_HEARTBEAT_FULL_MESSAGE_2026_05_19\r\n");
    }
  }
}

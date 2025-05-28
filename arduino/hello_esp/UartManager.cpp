#include "UartManager.h"





void UartManager::setup_uart() {
    _hardwareSerial = &Serial1; // Use Serial1 for UART communication
    _hardwareSerial->begin(1000000, SERIAL_8N1, PIN_UART1_RX, PIN_UART1_TX); // RX, TX
    _hardwareSerial->flush(); // Clear any existing data in the buffer
}

// void UartManager::enable_rx_interrupt() {
//   uart_t* uart = uartGetHwSerial(&_uartSerial);
//   uartEnableRxIntr(uart); // Enable RX interrupt
//   uartAttachRxCallback(uart, onUartRx, NULL); // Attach RX callback
// }

bool UartManager::read_byte(uint8_t* data) {
 if (_hardwareSerial->available()) {
    *data = _hardwareSerial->read();
    return true; // Data read successfully
  }
  return false; // No data available
}


// void IRAM_ATTR onUartRx(void* arg) {
//   while (_uartSerialPtr && _uartSerialPtr->available()) {
//     uint8_t byte = _uartSerialPtr->read();
//     uint16_t next_head = (_uart_rx_head + 1) % sizeof(uart_rx_buffer);

//     if (next_head != _uart_rx_tail) {
//       uart_rx_buffer[_uart_rx_head] = byte;
//       _uart_rx_head = next_head;
//     }
//   }
// }

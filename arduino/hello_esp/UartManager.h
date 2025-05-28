#ifndef __UART_MANAGER_H__
#define  __UART_MANAGER_H__

#include "HardwareSerial.h"
#include "esp32-hal-uart.h"
#include <stdint.h>


class UartManager {
  public:
    void setup_uart();
    volatile uint8_t uart_rx_buffer[256]; // Buffer for received data
    bool read_byte(uint8_t* data);
    void enable_rx_interrupt();


  private:
    HardwareSerial* _hardwareSerial;
    volatile uint16_t _uart_rx_head = 0;
    volatile uint16_t _uart_rx_tail = 0;
};


#endif

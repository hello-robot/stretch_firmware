#ifndef __UART_MANAGER_H__
#define  __UART_MANAGER_H__

#include "HardwareSerial.h"
#include "esp32-hal-uart.h"
#include <stdint.h>
#include <Transport.h>
#include "CommProtocol.h"

#define COBS_FRAME_DELIMITER 0x00
#define MAX_UART_PACKET_SIZE 256
#define FRAMING_TIMEOUT 100000



class UartManager {
  public:
    UartManager();
    void setup_uart();
    volatile uint8_t uart_rx_buffer[256]; // Buffer for received data
    void send_packet(const uint8_t *data, uint8_t len);
    bool receive_packet(uint8_t *data, uint8_t& n, int cobbs_frame_size);
    void enable_rx_interrupt();
    void send_status(uint8_t sts, const void* data, size_t data_size);


  private:
    HardwareSerial* _hardwareSerial;
    Crc16* _crc;
    COBS* _cobs;
    bool _rx_buffer_overflow = false;
    int  _rx_buffer_idx=0;
    uint8_t _rx_buffer[MAX_UART_PACKET_SIZE];
    uint8_t _tx_buffer[MAX_UART_PACKET_SIZE];
};


#endif

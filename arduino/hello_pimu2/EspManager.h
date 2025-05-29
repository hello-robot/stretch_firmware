#ifndef __ESP_MANAGER_H__
#define  __ESP_MANAGER_H__

#include "Common.h"
#include <Transport.h>

#define COBS_FRAME_DELIMITER 0x00
#define MAX_UART_PACKET_SIZE 256
#define FRAMING_TIMEOUT 100000

class EspManager {
  public:
    EspManager();
    void setup();
    void step();
    void esp_reset();
    void esp_fw_update();
    void send_packet(const uint8_t *data, uint8_t len);
    bool receive_packet(uint8_t *data, uint8_t& n, int cobbs_frame_size);
    void send_status(uint8_t sts_id, const void* data, size_t data_size);
    
  private:
    Crc16* _crc;
    COBS* _cobs;
    bool _rx_buffer_overflow = false;
    int  _rx_buffer_idx=0;
    uint8_t _rx_buffer[MAX_UART_PACKET_SIZE];
    uint8_t _tx_buffer[MAX_UART_PACKET_SIZE];

};

#endif
#ifndef __ESP_MANAGER_H__
#define  __ESP_MANAGER_H__

#include "Common.h"
#include <Transport.h>

#define COBS_FRAME_DELIMITER 0x00
#define MAX_UART_PACKET_SIZE 256
#define FRAMING_TIMEOUT 100000

enum EspRxState{
  ESP_RX_WAIT,
  ESP_RX_VALIDATE,

};

struct Esp_Status {
    float voltage_12v0;
    float voltage_20v0_aux;
    bool charger_barrel_fault; // Voltage in Volts
    bool charger_adapter_fault;
    bool cpu_sts;
};

class EspManager {
  public:
    EspManager();
    void setup();
    void step();
    void esp_reset();
    void esp_fw_update();
    void send_packet(const uint8_t *data, uint8_t len);
    void send_status(uint8_t sts, const void* data, size_t data_size);
    void rx_step();
    bool wake_ack = false;
    Esp_Status esp_sts;
    
  private:
    Crc16* _crc;
    COBS* _cobs;
    bool _rx_buffer_overflow = false;
    int  _rx_buffer_idx=0;
    uint8_t _rx_buffer[MAX_UART_PACKET_SIZE];
    uint8_t _tx_buffer[MAX_UART_PACKET_SIZE];
    EspRxState _esp_rx_state = ESP_RX_WAIT;
};

#endif
#ifndef __ESP_MANAGER_H__
#define  __ESP_MANAGER_H__

#include "Common.h"

class EspManager {
  public:
    void setup();
    void step();
    void esp_reset();
    void esp_fw_update();
    void write_packet(uint8_t *data, size_t len);
    
  private:
    bool esp_connected;
    bool _delay_started = false;
    unsigned long _delay_start_time = 0;
    unsigned long _elapsed_time = 0;
};

#endif
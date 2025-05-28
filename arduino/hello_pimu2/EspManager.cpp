#include "EspManager.h"
#include "TimeManager.h"

void EspManager::setup() {
    Serial1.begin(1000000);
}   

void EspManager::write_packet(uint8_t *data, size_t len) {
    Serial1.write(data, len);
}

void EspManager::esp_fw_update() {
    digitalWrite(ESP_RESET, HIGH);
    digitalWrite(ESP_BOOT, HIGH);
    digitalWrite(ESP_RESET, LOW);
    delayMicroseconds(500);
    digitalWrite(ESP_BOOT, LOW);
}

void EspManager::esp_reset() {
    digitalWrite(ESP_RESET, HIGH);
    delayMicroseconds(500);
    digitalWrite(ESP_RESET, LOW);
}
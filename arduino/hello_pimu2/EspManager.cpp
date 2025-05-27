#include "EspManager.h"
#include "TimeManager.h"



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
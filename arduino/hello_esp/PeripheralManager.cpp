#include "PeripheralManager.h"

void PeripheralManager::gpio_init() {
    pinMode(PIN_12V0_DISABLE, OUTPUT);
    pinMode(PIN_RPI_PWR_DISABLE, OUTPUT);
    pinMode(PIN_ESP_STS_LED, OUTPUT);
    pinMode(PIN_STS_LEDS_DISABLE, OUTPUT);
    pinMode(PIN_RPI_SD, OUTPUT);
    pinMode(PIN_LIDAR_DISABLE, OUTPUT);
    pinMode(PIN_CPU_SD, OUTPUT);
    pinMode(PIN_ROBOT_ACTIVE, OUTPUT);
    pinMode(PIN_PIMU_RESET, OUTPUT);
    pinMode(PIN_LATCH, OUTPUT);
    pinMode(PIN_AUX_20VO_EN, OUTPUT);
    pinMode(PIN_DCM_MODE_EN, OUTPUT);

    pinMode(PIN_RPI_STS, INPUT);
    pinMode(PIN_CPU_STS, INPUT);
    pinMode(PIN_BARREL_FAULT, INPUT);
    pinMode(PIN_ADAPTER_FAULT, INPUT);

    digitalWrite(PIN_ROBOT_ACTIVE, HIGH);
}

void PeripheralManager::disable_12v0(bool disable) {
    digitalWrite(PIN_12V0_DISABLE, disable ? HIGH : LOW);
}

void PeripheralManager::enable_aux_20v0(bool enable) {
    digitalWrite(PIN_LATCH, HIGH);
    digitalWrite(PIN_AUX_20VO_EN, enable ? HIGH : LOW);
    digitalWrite(PIN_LATCH, LOW);
}
void PeripheralManager::disable_lidar(bool disable) {
    digitalWrite(PIN_LIDAR_DISABLE, disable ? HIGH : LOW);
}
void PeripheralManager::pimu_reset() {
    digitalWrite(PIN_PIMU_RESET, HIGH);
    delay(100);
    digitalWrite(PIN_PIMU_RESET, LOW);
}
void PeripheralManager::pimu_bootloader_mode() {
    digitalWrite(PIN_PIMU_RESET, HIGH);
    delay(100);
    digitalWrite(PIN_PIMU_RESET, LOW);
    delay(100);
    digitalWrite(PIN_PIMU_RESET, HIGH);
    delay(100);
    digitalWrite(PIN_PIMU_RESET, LOW);
}
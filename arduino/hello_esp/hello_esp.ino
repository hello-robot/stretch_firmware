#include "UartManager.h"
#include "PeripheralManager.h"

PeripheralManager peripheral_manager;
UartManager uart_manager; // Using Serial1 for UART communication



uint8_t data;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  peripheral_manager.gpio_init();
  uart_manager.setup_uart();

}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(PIN_ESP_STS_LED, HIGH);
  delay(500);
  digitalWrite(PIN_ESP_STS_LED, LOW);
  delay(500);
  if (uart_manager.read_byte(&data)) {
    Serial.print("Received data: ");
    Serial.println(data, HEX); // Print received data in hexadecimal format
  } 

}


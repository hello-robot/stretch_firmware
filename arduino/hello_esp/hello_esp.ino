
#include "EspControl.h"

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(250000);
  setup_esp();
   // Enable UART1 wakeup for deep sleep

}

void loop() {
  
  if (current_pwr_state == STATE_ACTIVE)
  {
    toggle_led(500);
  }
  if (current_pwr_state == STATE_SLEEP) {
    // If the system is in sleep mode, we can put the ESP to sleep
    esp_light_sleep_start(); // Put ESP into light sleep
  }
  process_pimu_requests();

}


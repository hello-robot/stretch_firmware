
#include "EspControl.h"

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(9600);
  setup_esp();
   // Enable UART1 wakeup for deep sleep

}

void loop() {
  if (system_pwr_state_active)
  {
    toggle_led(500);
    process_pimu_requests();
  }

  if (!system_pwr_state_active) {
    // If the system is in sleep mode, we can put the ESP to sleep
    
    esp_light_sleep_start(); // Put ESP into deep sleep
    enter_wake();

  }
  
}


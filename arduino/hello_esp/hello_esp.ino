
#include "EspControl.h"

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(9600);
  setup_esp();

}

void loop() {
  toggle_led(500);
  process_pimu_requests();
}


#ifndef __ESP_CONTROL_H__
#define __ESP_CONTROL_H__

extern void process_pimu_requests();
extern void toggle_led(int rate_ms);
extern void setup_esp();
extern void enter_wake();

extern bool system_pwr_state_active;
#endif
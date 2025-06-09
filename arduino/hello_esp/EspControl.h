#ifndef __ESP_CONTROL_H__
#define __ESP_CONTROL_H__


typedef enum{
    STATE_ACTIVE,
    STATE_SLEEP,
    STATE_SHUTDOWN_CHRG,
    STATE_SLEEP_CHRG
}system_pwr_state;


extern void process_pimu_requests();
extern void toggle_led(int rate_ms);
extern void setup_esp();
extern void enter_wake();

extern system_pwr_state current_pwr_state;

#endif
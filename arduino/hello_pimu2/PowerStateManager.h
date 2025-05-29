#ifndef __POWER_STATE_MANAGER_H__
#define __POWER_STATE_MANAGER_H__

#include "Common.h"
#include "PeripheralManager.h"
#include "EspManager.h"
#include "LightBarManager.h"


typedef enum{
    STATE_BOOTED,
    STATE_NOT_BOOTED
} system_on_status_t;

class PowerStateManager
{
    public:
    PowerStateManager(PeripheralManager& pm, EspManager& em, LightBarManager& lb) :
    _peripheral_manager(pm), _esp_manager(em), _lightbar_manager(lb){}
    void step();
    bool check_boot_sts();
    void enter_sleep();
    void enter_wake();
    void sleep_pwr_button(uint8_t pwm);
    void power_state_setup();
    volatile bool system_pwr_state_active = false;

    private:
    PeripheralManager& _peripheral_manager;
    EspManager& _esp_manager;
    LightBarManager& _lightbar_manager;
    void set_pwr_button(bool state);
    bool _state;
    uint8_t _cnt = 255;

};

#endif
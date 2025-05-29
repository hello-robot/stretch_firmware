#ifndef __POWER_STATE_MANAGER_H__
#define __POWER_STATE_MANAGER_H__

#include "Common.h"
#include "PeripheralManager.h"
#include "EspManager.h"

typedef enum{
    PWR_STATE_ACTIVE,
    PWR_STATE_SLEEP,
} power_state_status_t;

typedef enum{
    STATE_BOOTED,
    STATE_NOT_BOOTED
} system_on_status_t;

class PowerStateManager
{
    public:
    PowerStateManager(PeripheralManager& pm, EspManager& em) : _peripheral_manager(pm), _esp_manager(em) {}
    power_state_status_t step();
    bool check_boot_sts();
    void enter_sleep();
    void power_state_setup();

    private:
    PeripheralManager& _peripheral_manager;
    EspManager& _esp_manager;
    void set_pwr_button(bool state);
    bool _active;
    power_state_status_t _state;

};

#endif
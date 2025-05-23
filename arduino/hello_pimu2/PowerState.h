#ifndef __POWERSTATE_H__
#define __POWERSTATE_H__

#include "Common.h"

typedef enum{
    PWR_STATE_ACTIVE,
    PWR_STATE_SLEEP,
} power_state_status_t;

typedef enum{
    STATE_BOOTED,
    STATE_NOT_BOOTED
} system_on_status_t;

class PowerState
{
    public:
    power_state_status_t step();
    bool check_boot_sts();

    private:
    void set_pwr_button(bool state);
    bool _active;

};

#endif
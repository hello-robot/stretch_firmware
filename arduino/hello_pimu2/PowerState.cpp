#include "PowerState.h"


power_state_status_t PowerState::step()
{
    power_state_status_t _state;
    if (!digitalRead(PWR_EN)) 
    {
        _active = true;
        _state = PWR_STATE_ACTIVE;

    }

    else if(digitalRead(PWR_EN)) 
    {
        _active = false;
        _state = PWR_STATE_SLEEP;
    }
    set_pwr_button(_active);
    return _state;
}

bool PowerState::check_boot_sts()
{
    if (digitalRead(ROBOT_ACTIVE)) return true;
    else return false;
}

void PowerState::set_pwr_button(bool state)
{
    digitalWrite(BTN_GREEN, _active);
    digitalWrite(BTN_RED, !_active);
}
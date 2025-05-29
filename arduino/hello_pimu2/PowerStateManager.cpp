#include "PowerStateManager.h"

volatile bool g_button_pressed = false;

void PowerStateManager::power_state_setup()
{
    pinMode(PWR_EN, INPUT);
    pinMode(SLEEP_EN, INPUT);
    pinMode(ROBOT_ACTIVE, INPUT);
    pinMode(BTN_GREEN, OUTPUT);
    pinMode(BTN_RED, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(SLEEP_EN), buttonISR, FALLING);

}

power_state_status_t PowerStateManager::step()
{
    if (!digitalRead(PWR_EN) || (g_button_pressed && _state == PWR_STATE_SLEEP))
    {
        g_button_pressed = false; // Reset button pressed state
        _active = true;
        _state = PWR_STATE_ACTIVE;

    }

    else if(digitalRead(PWR_EN) || (g_button_pressed && _state == PWR_STATE_ACTIVE)) 
    {
        enter_sleep();
        _active = false;
        _state = PWR_STATE_SLEEP;
    }
    set_pwr_button(_active);
    return _state;
}

bool PowerStateManager::check_boot_sts()
{
    if (digitalRead(ROBOT_ACTIVE)) return true;
    else return false;
}

void PowerStateManager::set_pwr_button(bool state)
{
    digitalWrite(BTN_GREEN, _active);
    digitalWrite(BTN_RED, !_active);
}

void PowerStateManager::enter_sleep()
{
    _peripheral_manager.peripheral_sleep_state();
    // _esp_manager.send_status(UART_PWR_SLEEP, 0, 0);
    set_pwr_button(false);
}

void buttonISR()
{
    g_button_pressed = true;
}
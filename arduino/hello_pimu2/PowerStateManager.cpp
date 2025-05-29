#include "PowerStateManager.h"

volatile bool g_button_pressed = false;
volatile bool _falling_detected = false;

void buttonISR()
{
    volatile bool pin_state = digitalRead(SLEEP_EN);
    if(!pin_state)
    {
        _falling_detected = true;
    }
    else{
        if(_falling_detected)
        {
            _falling_detected = false;
            g_button_pressed = true;
            return; // Ignore rising edge if it follows a falling edge
        }
    }
    
}

void PowerStateManager::power_state_setup()
{
    pinMode(PWR_EN, INPUT);
    pinMode(SLEEP_EN, INPUT);
    pinMode(ROBOT_ACTIVE, INPUT);
    pinMode(BTN_GREEN, OUTPUT);
    pinMode(BTN_RED, OUTPUT);
    if (!digitalRead(PWR_EN))
    {
        _state = true;
    }
    else if(digitalRead(PWR_EN))
    {
        _state = false;
    }
    set_pwr_button(_state);
    system_pwr_state_active =_state;
    __disable_irq();
    attachInterrupt(digitalPinToInterrupt(SLEEP_EN), buttonISR, CHANGE);
    __enable_irq();

}

void PowerStateManager::step()
{
    if (g_button_pressed && system_pwr_state_active)
    {
        system_pwr_state_active = false;
        g_button_pressed = false; // Reset button pressed state
        enter_sleep();
        return;
    }

    else if(g_button_pressed && !system_pwr_state_active)
    {
        system_pwr_state_active = true;
        g_button_pressed = false; // Reset button pressed state
        enter_wake();
        return;
    }

    if (!system_pwr_state_active)
    {
        if (_cnt > 0){
            sleep_pwr_button(_cnt--);
        }
    }
}

bool PowerStateManager::check_boot_sts()
{
    if (digitalRead(ROBOT_ACTIVE)) return true;
    else return false;
}

void PowerStateManager::set_pwr_button(bool state)
{
    digitalWrite(BTN_GREEN, state);
    digitalWrite(BTN_RED, !state);
}

void PowerStateManager::sleep_pwr_button(uint8_t pwm)
{
    analogWrite(BTN_RED, pwm); // Set the red button LED to a low brightness
}
// void PowerStateManager::set_up_button()
// {
//     MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC1;
//     GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN; // Set up the generic clock for TC1
//     while (GCLK->PCHCTRL[TC1_GCLK_ID].bit.CHEN == 0); // Wait for the clock to be enabled

//     TC1->COUNT16.CTRLA.bit.ENABLE = 0; // Disable TC1 before configuration
//     while (TC1->COUNT16.SYNCBUSY.bit.ENABLE); // Wait for synchronization

//     TC1->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | // Set to 16-bit mode
//                              TC_CTRLA_PRESCALER_DIV64 | // Set prescaler to 1024
//                              TC_WAVE_WAVEGEN_NPWM; // Set waveform generation to match frequency
//     TC1->COUNT16.CC[1].reg = 15625; // Set the compare value for 1Hz (assuming 1MHz clock)
// }

void PowerStateManager::enter_sleep()
{
    _peripheral_manager.peripheral_sleep_state();
    _lightbar_manager.Off();
    set_pwr_button(system_pwr_state_active);
    _esp_manager.send_status(UART_PWR_SLEEP, 0, 0);
}
void PowerStateManager::enter_wake()
{
    _peripheral_manager.peripheral_active_state();
    set_pwr_button(system_pwr_state_active);
    _esp_manager.send_status(UART_PWR_WAKE, 0, 0);
}


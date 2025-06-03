#include "PowerStateManager.h"
#include "Arduino.h"
#include "wiring_private.h"


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
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // Enable deep sleep mode
    PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_STANDBY; // Set sleep mode to standby
    while(PM->SLEEPCFG.bit.SLEEPMODE != PM_SLEEPCFG_SLEEPMODE_STANDBY); // Wait for the sleep mode to be set

    pinMode(PWR_EN, INPUT);
    pinMode(SLEEP_EN, INPUT);
    pinMode(ROBOT_ACTIVE, INPUT);
    pinMode(BTN_GREEN, OUTPUT);
    pinMode(BTN_RED, OUTPUT);
    set_up_button();
    if (!digitalRead(PWR_EN))
    {
        _state = true;
    }
    else if(digitalRead(PWR_EN))
    {
        _state = false;
    }
    system_pwr_state_active =_state;
    
    pwr_button_led(255, system_pwr_state_active); // Set the green button to 50% duty cycl

    set_pwr_button(_state);
    __disable_irq();
    attachInterrupt(digitalPinToInterrupt(SLEEP_EN), buttonISR, CHANGE);
    __enable_irq();

}

void PowerStateManager::step()
{
    if (g_button_pressed && system_pwr_state_active)
    {
        g_button_pressed = false; // Reset button pressed state
        system_pwr_state_active = false;
        enter_sleep();
        return;
    }

    else if(g_button_pressed && !system_pwr_state_active)
    {
        system_pwr_state_active = true;
        g_button_pressed = false; // Reset button pressed state
        _target_pwm = 0; // Reset target PWM to 0 when waking up
        _rled = SLEEP_PWM_BRIGHTNESS; // Reset red LED duty cycle to 255
        enter_wake();
        return;
    }

    if (!system_pwr_state_active)
    {
        pwr_button_sleep();
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

void PowerStateManager::pwr_button_sleep()
{
    _fade_cnt += 1;
    if (_fade_cnt < 5) return;
    if(_target_pwm == 0)
    {
        _rled -= 1;
        _fade_cnt = 0; // Reset fade count after each step
        if (_rled < 2)
        {
            _rled = 1; // Ensure the red LED duty cycle does not go below 1
            _target_pwm = SLEEP_PWM_BRIGHTNESS; // Reset target PWM to 255 when red LED reaches minimum
        }
    }
    if(_target_pwm == SLEEP_PWM_BRIGHTNESS)
    {
        _rled += 1;
        _fade_cnt = 0;
        if (_rled > SLEEP_PWM_BRIGHTNESS)
        {
            _rled = SLEEP_PWM_BRIGHTNESS; // Ensure the red LED duty cycle does not go below 1
            _target_pwm = 0; // Reset target PWM to 255 when red LED reaches minimum
        }
    }

    TC1->COUNT8.CC[1].reg = _rled; // Set the duty cycle for the green button
    TC1->COUNT8.CC[0].reg = 0;
    while (TC1->COUNT8.SYNCBUSY.bit.CC0 || TC1->COUNT8.SYNCBUSY.bit.CC1);

}
void PowerStateManager::pwr_button_led(uint8_t pwm, bool state)
{
    if (state)
    {
        TC1->COUNT8.CC[0].reg = pwm; // Set the duty cycle for the green button
        TC1->COUNT8.CC[1].reg = 1; // Set the duty cycle for the red button to 0
    }
    if (!state)
    {
        
        TC1->COUNT8.CC[1].reg = pwm; // Set the duty cycle for the green button
        TC1->COUNT8.CC[0].reg = 1;

    }
    while (TC1->COUNT8.SYNCBUSY.bit.CC0 || TC1->COUNT8.SYNCBUSY.bit.CC1);
}

void PowerStateManager::set_up_button()
{
    pinPeripheral(BTN_GREEN, PIO_TIMER);
    pinPeripheral(BTN_RED, PIO_TIMER);

    MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC1;
    GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN; // Set up the generic clock for TC1
    while (GCLK->PCHCTRL[TC1_GCLK_ID].bit.CHEN == 0); // Wait for the clock to be enabled

    TC1->COUNT8.CTRLA.bit.ENABLE = 0; // Disable TC1 before configuration
    while (TC1->COUNT8.SYNCBUSY.bit.ENABLE); // Wait for synchronization

    TC1->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | // Set to 8-bit mode
                            TC_CTRLA_PRESCALER_DIV64 |
                            // TC_CTRLA_RUNSTDBY | // Run in standby mode
                            TC_CTRLA_PRESCSYNC_PRESC;// Set waveform generation to match frequency

    TC1->COUNT8.WAVE.reg |= TC_WAVE_WAVEGEN_NPWM; // Set WAVEGEN to Normal Frequency
    TC1->COUNT8.PER.reg = 0xFF; 
    TC1->COUNT8.CC[0].reg = 1; 
    TC1->COUNT8.CC[1].reg = 1; 

    while (TC1->COUNT8.SYNCBUSY.bit.CC0 || TC1->COUNT8.SYNCBUSY.bit.CC1); // Wait for synchronization

    TC1->COUNT8.CTRLA.bit.ENABLE = 1; // Enable TC1
    while (TC1->COUNT8.SYNCBUSY.bit.ENABLE); // Wait for synchronization
}

void PowerStateManager::enter_sleep()
{
    _peripheral_manager.peripheral_sleep_state();
    _lightbar_manager.Off();

    pwr_button_led(SLEEP_PWM_BRIGHTNESS, system_pwr_state_active); // Set the green button to 0% duty cycle
    _esp_manager.send_status(UART_PWR_SLEEP, 0, 0);
    
}
void PowerStateManager::enter_wake()
{
    _peripheral_manager.peripheral_active_state();
    pwr_button_led(255, system_pwr_state_active); // Set the green button to 0% duty cycle
    _esp_manager.send_status(UART_PWR_WAKE, 0, 0);
}


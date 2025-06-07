#include "PowerStateManager.h"
#include "Arduino.h"
#include "wiring_private.h"


volatile bool g_button_pressed = false;
volatile bool _falling_detected = false;
volatile bool _state = false;
volatile bool _fading_up = false;
volatile uint8_t _target_pwm = 0;
volatile uint8_t _fade_cnt = 0;
volatile uint8_t _rled = SLEEP_PWM_BRIGHTNESS; // Red LED duty cycle

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
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_STANDBY;
    while (PM->SLEEPCFG.bit.SLEEPMODE != PM_SLEEPCFG_SLEEPMODE_STANDBY);

    USB->DEVICE.CTRLA.bit.ENABLE = 0;
    while (USB->DEVICE.SYNCBUSY.bit.ENABLE) {;}
    USB->DEVICE.CTRLA.bit.RUNSTDBY = 0;
    USB->DEVICE.CTRLA.bit.ENABLE = 1;
    while (!USB->DEVICE.SYNCBUSY.bit.ENABLE) {;}

    // OSC32KCTRL->OSCULP32K.bit.EN1K = 1;
    // OSC32KCTRL->OSCULP32K.bit.EN32K = 0;
    // OSC32KCTRL->RTCCTRL.reg |= OSC32KCTRL_RTCCTRL_RTCSEL_ULP1K;

    // EIC->CTRLA.bit.CKSEL = 1;

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
        _state = system_pwr_state_active;
        enter_sleep();
        sleep_mode_set = true;
        return;
    }

    else if(g_button_pressed && !system_pwr_state_active)
    {
        sleep_mode_set = false;
        system_pwr_state_active = true;
        _state = system_pwr_state_active;
        g_button_pressed = false; // Reset button pressed state
        enter_wake();
        return;
    }

}

bool PowerStateManager::check_boot_sts()
{
    //Esp will have this pin high when the system is booted
    if (digitalRead(ROBOT_ACTIVE)) return true;
    else return false;
}

void PowerStateManager::set_up_button()
{
    pinPeripheral(BTN_GREEN, PIO_TIMER);
    pinPeripheral(BTN_RED, PIO_TIMER);

    MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC1;
    //Set GLCK 2 to run off the 32kHz oscillator and in standby mode
    GCLK->GENCTRL[0x02].reg = GCLK_GENCTRL_SRC_OSCULP32K | // Set the source to the 32kHz oscillator
                              GCLK_GENCTRL_GENEN |
                              GCLK_GENCTRL_DIV(1) |
                              GCLK_GENCTRL_RUNSTDBY; // Enable the clock generator
    
    while (GCLK->SYNCBUSY.bit.GENCTRL2);

    GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_CHEN; // Set up the 32khz clock for TC1
    while (GCLK->PCHCTRL[TC1_GCLK_ID].bit.CHEN == 0); // Wait for the clock to be enabled

    TC1->COUNT8.CTRLA.bit.ENABLE = 0; // Disable TC1 before configuration
    while (TC1->COUNT8.SYNCBUSY.bit.ENABLE); // Wait for synchronization

    TC1->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | // Set to 8-bit mode
                            TC_CTRLA_PRESCALER_DIV1 |
                            TC_CTRLA_RUNSTDBY | // Run in standby mode
                            TC_CTRLA_PRESCSYNC_PRESC;// Set waveform generation to match frequency

    TC1->COUNT8.WAVE.reg |= TC_WAVE_WAVEGEN_NPWM; // Set WAVEGEN to Normal Frequency
    TC1->COUNT8.PER.reg = 0xFF;
    while (TC1->COUNT8.SYNCBUSY.bit.PER); // Wait for synchronization

    TC1->COUNT8.CC[0].reg = 5; 
    TC1->COUNT8.CC[1].reg = 5; 
    while (TC1->COUNT8.SYNCBUSY.bit.CC0 || TC1->COUNT8.SYNCBUSY.bit.CC1); // Wait for synchronization

    TC1->COUNT8.INTENSET.bit.OVF = 1; // Enable overflow interrupt

    NVIC_SetPriority(TC1_IRQn, 4); // Set the priority of the TC1 interrupt
    NVIC_EnableIRQ(TC1_IRQn);
}

void PowerStateManager::enableTC1()
{
    TC1->COUNT8.CTRLA.bit.ENABLE = 1; // Enable TC1
    while (TC1->COUNT8.SYNCBUSY.bit.ENABLE); // Wait for synchronization
}

void PowerStateManager::enter_sleep()
{
    imu_b.imu_sleep_mode();
    _peripheral_manager.peripheral_sleep_state();
    _lightbar_manager.disableDMAC();
    _esp_manager.send_status(UART_PWR_SLEEP, 0, 0); 

}
void PowerStateManager::enter_wake()
{
    _peripheral_manager.peripheral_active_state();
    _lightbar_manager.enableDMAC();
    _esp_manager.send_status(UART_PWR_WAKE, 0, 0);
    imu_b.imu_wake_up();
    
}

void TC1_Handler() {                // gets called with FsMg frequency

  if (TC1->COUNT8.INTFLAG.bit.OVF == 1) 
  {
    TC1->COUNT8.INTFLAG.reg = TC_INTFLAG_OVF;    
    if (_state)
    {
        TC1->COUNT8.CC[0].reg = 255; // Set the duty cycle for the green button
        TC1->COUNT8.CC[1].reg = 5; // Set the duty cycle for the red button to 0
        while (TC1->COUNT8.SYNCBUSY.bit.CC0 || TC1->COUNT8.SYNCBUSY.bit.CC1);
    }
    else if (!_state)
    {
        _fade_cnt += 1;
        if (_fade_cnt < 4) return;
        if(_fading_up)
        {
            if(_rled < SLEEP_PWM_BRIGHTNESS)
            {
                _rled += 1; // Increment the red LED duty cycle
            }
            else
            {
                _rled = SLEEP_PWM_BRIGHTNESS; // Ensure the red LED duty cycle does not exceed maximum brightness
                _fading_up = false; // Stop fading up when the maximum brightness is reached
            }
             _fade_cnt = 0;
        }
        else
        {
            if(_rled > 5)
            {
                _rled -= 1; // Decrement the red LED duty cycle
            }
            else
            {
                _rled = 5;
                _fading_up = true; // Start fading up when the minimum brightness is reached
            }
             _fade_cnt = 0;
        }
        
        TC1->COUNT8.CC[1].reg = _rled; // Set the duty cycle for the green button
        TC1->COUNT8.CC[0].reg = 5; // Set the duty cycle for the red button to 0
        while (TC1->COUNT8.SYNCBUSY.bit.CC0 || TC1->COUNT8.SYNCBUSY.bit.CC1);
    }
       
  }
}

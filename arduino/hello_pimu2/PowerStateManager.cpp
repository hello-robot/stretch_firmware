#include "PowerStateManager.h"
#include "Arduino.h"
#include "wiring_private.h"


volatile bool g_button_pressed = false;
volatile bool _falling_detected = false;
volatile system_pwr_state _state = STATE_SLEEP;
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

    OSC32KCTRL->OSCULP32K.bit.EN1K = 1;
    OSC32KCTRL->OSCULP32K.bit.EN32K = 0;
    OSC32KCTRL->RTCCTRL.reg |= OSC32KCTRL_RTCCTRL_RTCSEL_ULP1K;

    EIC->CTRLA.bit.CKSEL = 1;

    set_up_button();

    //Check if user pressed power button and battery is above 5%, enter active state
    if (!digitalRead(PWR_EN) && _battery_manager.battery_soc >= 5)
    {
        current_pwr_state = STATE_ACTIVE;
        _state = current_pwr_state;
        if (!check_boot_sts())
        {
            _esp_manager.send_status(UART_PWR_WAKE,0, 0);
            _peripheral_manager.fast_actuator_control(true);
        }
        else{
            //Set pins high since we are in active mode
            _peripheral_manager.set_actuator_active();
        }
    }
    //if battery less than 5% enter sleep state
    else if(!digitalRead(PWR_EN) && _battery_manager.battery_soc < 5)
    {
        _esp_manager.send_status(UART_PWR_SLEEP,0, 0);
        _peripheral_manager.fast_actuator_control(false);
    }
    //Check to see if user did not press the pwr button go into shutdown charge mode
    else if(digitalRead(PWR_EN))
    {

        current_pwr_state = STATE_SHUTDOWN_CHRG;
        enter_chrg_sleep(current_pwr_state);
        _state = current_pwr_state;
        // _lightbar_manager.disableDMAC();
    }

    __disable_irq();
    attachInterrupt(digitalPinToInterrupt(SLEEP_EN), buttonISR, CHANGE);
      __enable_irq();
    
}

void PowerStateManager::step()
{
    //If button is not pressed enter shudown charge state
    if (digitalRead(PWR_EN) && current_pwr_state != STATE_SHUTDOWN_CHRG && current_pwr_state != STATE_USER_FEEDBACK)
    {
        g_button_pressed = false;
        enter_chrg_sleep(current_pwr_state);
        current_pwr_state = STATE_SHUTDOWN_CHRG;
        _state = current_pwr_state;
        
    }
    if (current_pwr_state == STATE_SHUTDOWN_CHRG)
    {
        if (!digitalRead(PWR_EN))
        {
            g_button_pressed = false;
            current_pwr_state = STATE_ACTIVE;
            _state = current_pwr_state;
            enter_sd_to_wake();
            light_bar_indication = false;
            return;
        }
    }

    if(_battery_manager.battery_soc == 0 && current_pwr_state == STATE_ACTIVE && _battery_manager.bms_ready)
    {
        current_pwr_state = STATE_SLEEP;
        _state = current_pwr_state;
        enter_sleep();
        return;
    }
    

    //If button is pressed and state is active or if soc is 0 
    if (g_button_pressed && current_pwr_state == STATE_ACTIVE)
    {
        //If not charging go to lowest power mode
        g_button_pressed = false; // Reset button pressed state
        if (!_battery_manager.flag_charger_is_charging )
        {  
            current_pwr_state = STATE_SLEEP;
            _state = current_pwr_state;
            enter_sleep();
            return;
        }
        else if (_battery_manager.flag_charger_is_charging)
        {
            enter_chrg_sleep(current_pwr_state);
            current_pwr_state = STATE_SLEEP_CHRG;
            _state = current_pwr_state;
            return;
        }
    }
    
    //if button is pressed and state is in either sleep or sleep charge enter active state
    else if(g_button_pressed && (current_pwr_state == STATE_SLEEP || current_pwr_state == STATE_SLEEP_CHRG))
    {
        g_button_pressed = false; // Reset button pressed state
        if (!_battery_manager.battery_soc == 0 || _battery_manager.flag_charger_is_charging)
        {
            enter_wake(current_pwr_state);
            current_pwr_state = STATE_ACTIVE;
            _state = current_pwr_state;
            return;
        }
        else if (_battery_manager.battery_soc == 0 && !_battery_manager.flag_charger_is_charging){
            light_bar_indication = true;
            light_bar_st_time = time_manager.get_elapsed_time_ms();
            _lightbar_manager.enableDMAC();
            current_pwr_state = STATE_USER_FEEDBACK;
            feedback_next_pwr_state = STATE_SLEEP;
            return;
        }

    }

    //if system is sleep and charger starts charging enter sleep charge state
    if (current_pwr_state == STATE_SLEEP && _battery_manager.flag_charger_is_charging)
    {
        enter_chrg_sleep(current_pwr_state);
        // current_pwr_state = STATE_SLEEP_CHRG;
        return;
    }
    //if ssytem is sleep charge state and charger is disconnected enter sleep mode
    if (current_pwr_state == STATE_SLEEP_CHRG && !_battery_manager.flag_charger_is_charging)
    {
        current_pwr_state = STATE_SLEEP;
        enter_sleep();
        return;
    }
    
    if (current_pwr_state == STATE_USER_FEEDBACK){
        if (time_manager.get_elapsed_time_ms() - light_bar_st_time >= 3000)
        {
            current_pwr_state = feedback_next_pwr_state;
            _lightbar_manager.Off();
            _lightbar_manager.disableDMAC();
            light_bar_indication = false;
        }
        else
        {
            if (light_bar_indication && (feedback_next_pwr_state == STATE_SLEEP_CHRG || feedback_next_pwr_state == STATE_SHUTDOWN_CHRG))
            {
                _lightbar_manager.sleep_chrg();
                _esp_manager.send_status(UART_STS_SD_CHRG,0, 0);
            }
            if (light_bar_indication && feedback_next_pwr_state == STATE_SLEEP)
            {
                //Does not work
                _lightbar_manager.low_battery_fault();
            }
            
        }
        return;
        
    }
}

bool PowerStateManager::check_boot_sts()
{
    //Esp will have this pin high when the system is booted
    if (digitalRead(ROBOT_ACTIVE)) return true;
    else return false;
}

void PowerStateManager::enter_sleep()
{
    // imu_b.imu_sleep_mode();
    _peripheral_manager.peripheral_sleep_state();
    _lightbar_manager.disableDMAC();
    _esp_manager.send_status(UART_PWR_SLEEP,0,0); 

}
void PowerStateManager::enter_wake(system_pwr_state st)
{
    _peripheral_manager.peripheral_active_state();
    _lightbar_manager.enableDMAC();
    _esp_manager.send_status(UART_PWR_WAKE,0, 0);
    _peripheral_manager.set_actuator_active();

}
void PowerStateManager::enter_chrg_sleep(system_pwr_state st)
{
    _peripheral_manager.peripheral_sd_state();
    _esp_manager.send_status(UART_STS_SD_CHRG,0, 0);

    switch (st)
    {
        case STATE_SLEEP:
            current_pwr_state = STATE_USER_FEEDBACK;
            light_bar_st_time = time_manager.get_elapsed_time_ms();
            light_bar_indication = true;
            feedback_next_pwr_state = STATE_SLEEP_CHRG;
            _lightbar_manager.enableDMAC();
            break;

        case STATE_SHUTDOWN_CHRG:
            current_pwr_state = STATE_USER_FEEDBACK;
            light_bar_st_time = time_manager.get_elapsed_time_ms();
            light_bar_indication = true;
            feedback_next_pwr_state = STATE_SHUTDOWN_CHRG;
            break;

        case STATE_SLEEP_CHRG:
        case STATE_ACTIVE:
            light_bar_indication = false;
            _lightbar_manager.Off();
             _lightbar_manager.disableDMAC();
            break;

        default:
            break;
    }          
    

}

void PowerStateManager::enter_sd_to_wake()
{
    _lightbar_manager.enableDMAC();
    _peripheral_manager.peripheral_active_state();
     _esp_manager.send_status(UART_PWR_WAKE,0, 0);
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

void TC1_Handler() {                // gets called with FsMg frequency

  if (TC1->COUNT8.INTFLAG.bit.OVF == 1) 
  {
    TC1->COUNT8.INTFLAG.reg = TC_INTFLAG_OVF;
    switch (_state)
    {
        case STATE_ACTIVE:
            TC1->COUNT8.CC[0].reg = 255; // Set the duty cycle for the green button
            TC1->COUNT8.CC[1].reg = 5; // Set the duty cycle for the red button to 0
            while (TC1->COUNT8.SYNCBUSY.bit.CC0 || TC1->COUNT8.SYNCBUSY.bit.CC1);
            break;
        case STATE_SLEEP:
        case STATE_SLEEP_CHRG:
            _fade_cnt += 1;
            if (_fade_cnt < 8) return;
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
            break;

        case STATE_SHUTDOWN_CHRG:
            TC1->COUNT8.CC[0].reg = 5; // Set the duty cycle for the green button
            TC1->COUNT8.CC[1].reg = 5; // Set the duty cycle for the red button to 0
            while (TC1->COUNT8.SYNCBUSY.bit.CC0 || TC1->COUNT8.SYNCBUSY.bit.CC1);
            break;
        default:
            break;

    }    
       
  }
}

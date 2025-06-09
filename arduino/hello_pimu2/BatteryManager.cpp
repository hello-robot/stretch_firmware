#include "BatteryManager.h"

INA228 ina228(INA228_ADDRESS); // Create an instance of the INA228 class with the default I2C address



void BatteryManager::init() {
    ina228.begin(Wire, 1000000); // Initialize the INA228 with the Wire library and a clock speed of 400kHz
    ina228.init();
    ina228.set_shunt_measurment_time();
    ina228.set_conversion_delay();
    ina228.set_adc_range(ADC_RANGE_163); // Set the ADC range to 163mV
    ina228.set_alert_dialog();
    ina228.set_oc_limit(15.0f); //Set overcurrent limit to 1A
    ina228.set_neg_oc_limit(-10.0f); // Set the negative overcurrent limit to -1A
    
}

void BatteryManager::step(float chrg_current, float adapter_v){
    voltage_battery = ina228.read_vbus();
    get_currents(chrg_current);
    charging_state(adapter_v);
    battery_soc = get_battery_soc(voltage_battery, flag_charger_connected);
}

void BatteryManager::get_currents(float chrg_current) {
    float c = ina228.read_current();
    if (c < 0 && flag_charger_connected)
    {
        current_sys = chrg_current + c;
        current_battery = c;
        current_charger = chrg_current;
    }
    else if (c > 0 && !flag_charger_connected)
    {
        current_sys = c;
        current_charger = 0;
        current_battery = current_sys;
    }
}



void BatteryManager::charging_state(float adapter_v)
{   
    if (adapter_v >= 34)
    {
        flag_charger_connected = true; //if the 36V charger input is present
    }
    else if (adapter_v < 34)
    {
        flag_charger_connected = false;
        //when adapter is disconnected default for charger to be on
        charger_control(true);
        _chrg_done = false;
    }

    //Check to see if charger is charging
    if (flag_charger_connected && !_flag_charger_disabled)
    {
        flag_charger_is_charging = true;
    }
    else if (flag_charger_connected || !_flag_charger_disabled)
    {
        flag_charger_is_charging = false;
    }

    if (voltage_battery >= 28.8 && flag_charger_is_charging && !_chrg_done)
    {
        charger_control(false);
        _chrg_done = true;
    }
    else if (voltage_battery <= 26.0 && _chrg_done)
    {
        charger_control(true);
        _chrg_done = false;
    }

}

void BatteryManager::charger_control(bool en)
{
    if (en)
    {
        digitalWrite(CHARGER_DISABLE, LOW);
        _flag_charger_disabled = false;
    }
    else
    {
        digitalWrite(CHARGER_DISABLE, HIGH);
        _flag_charger_disabled = true;
    }
}

int BatteryManager::get_battery_soc(float voltage, bool charger_connected) {
    int new_soc = current_soc;

    // Allow downward SoC transitions
    if (voltage <= 23)
        new_soc = 0;
    if (voltage <= 24.0 && current_soc > 10)
        new_soc = 10;
    if (voltage <= 24.5 && current_soc > 20)
        new_soc = 20;
    if (voltage <= 24.8 && current_soc > 25)
        new_soc = 25;
    if (voltage <= 25.1 && current_soc > 50)
        new_soc = 50;
    if (voltage <= 25.3 && current_soc > 75)
        new_soc = 75;


    // Allow upward SoC transitions only if charging
    if (flag_charger_is_charging) {
        if (voltage > 28.6)
            new_soc = 100;
        else if (voltage >= 27.5 && new_soc < 75)
            new_soc = 75;
        else if (voltage >= 26.5 && new_soc < 50)
            new_soc = 50;
        else if (voltage >= 25.5 && new_soc < 25)
            new_soc = 25;
        else if (voltage >= 24.5 && new_soc < 20)
            new_soc = 20;
        else if (voltage >= 23.0 && new_soc < 10)
            new_soc = 10;
    }
    current_soc = new_soc;
    return new_soc;
}
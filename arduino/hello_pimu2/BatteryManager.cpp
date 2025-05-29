#include "BatteryManager.h"

INA228 ina228(INA228_ADDRESS); // Create an instance of the INA228 class with the default I2C address



void BatteryManager::init() {
    ina228.begin(Wire, 400000); // Initialize the INA228 with the Wire library and a clock speed of 400kHz
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
    uint8_t cflag1 = digitalRead(CHARGER_CONNECTED);
    uint8_t cflag2 = digitalRead(CHARGER_STATE);

    if (adapter_v > 34 && !flag_charger_disabled)
    {
        flag_charger_connected = true;
        
    }
    else
    {
        flag_charger_connected = false;
    }

}

void BatteryManager::charger_control(bool en)
{
    if (en)
    {
        digitalWrite(CHARGER_DISABLE, LOW);
        flag_charger_disabled = false;
    }
    else
    {
        digitalWrite(CHARGER_DISABLE, HIGH);
        flag_charger_disabled = true;
    }
}

int BatteryManager::get_battery_soc(float voltage, bool charger_connected) {
    int new_soc = current_soc;

    // Allow downward SoC transitions
    if (voltage <= 23.3)
        new_soc = 10;
    else if (voltage <= 23.5 && current_soc > 20)
        new_soc = 20;
    else if (voltage <= 24.5 && current_soc > 25)
        new_soc = 25;
    else if (voltage <= 25.5 && current_soc > 50)
        new_soc = 50;
    else if (voltage <= 26.5 && current_soc > 75)
        new_soc = 75;

    // Allow upward SoC transitions only if charging
    if (charger_connected) {
        if (voltage > 27.6)
            new_soc = 100;
        else if (voltage >= 26.5 && new_soc < 75)
            new_soc = 75;
        else if (voltage >= 25.5 && new_soc < 50)
            new_soc = 50;
        else if (voltage >= 24.5 && new_soc < 25)
            new_soc = 25;
        else if (voltage >= 23.5 && new_soc < 20)
            new_soc = 20;
        else if (voltage >= 23.0 && new_soc < 10)
            new_soc = 10;
    }
    current_soc = new_soc;
    return new_soc;
}
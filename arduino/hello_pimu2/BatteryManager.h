#ifndef __BATTERYMANAGER_H__
#define __BATTERYMANAGER_H__

#include "INA228.h"
#include "AnalogManager.h"

#define CHARGING_CURRENT 5.57f

class BatteryManager
{
public:
    void init();
    void step(float chrg_current,float adapter_v);
    void get_currents(float chrg_current);
    void charging_state(float adapter_v);
    void charger_control(bool en);
    int get_battery_soc(float voltage, bool charger_connected);

    float voltage_battery;
    float current_sys;
    float current_battery;
    float current_charger;

    bool flag_charger_disabled = false;
    bool flag_charger_connected = false;

    int battery_soc;
    int current_soc = 100;
    

    
};

#endif
#ifndef __BATTERYMANAGER_H__
#define __BATTERYMANAGER_H__

#include "INA228.h"
#include "AnalogManager.h"

class BatteryManager
{
public:
    void init();
    void step();
    float read_vbus();

    float voltage;
    float sys_current;
    float battery_current;

    
};

#endif
#ifndef __CHARGER_MANAGER_H__
#define  __CHARGER_MANAGER_H__
#include "Common.h"

#define CHARGER_SAMPLE_RATE 500


class ChargerManager{
  public:
    bool step(float vbat, float sys_current, float charge_current, int board_variant);
    void hotplug_check(float vd);
    void unplug_check(float v, float c);
    bool charging_sts_flag = true;
    bool hotplug_sts_flag = false;
    bool unplug_sts_flag = false;
    bool t1 = false;
    bool t2 = false;
};

#endif
#ifndef __CHARGER_MANAGER_H__
#define  __CHARGER_MANAGER_H__
#include "Common.h"

#define CHARGER_SAMPLE_RATE 1000


class ChargerManager{
  public:
    bool step(float vbat, float sys_current, float charge_current, int board_variant);
};

#endif
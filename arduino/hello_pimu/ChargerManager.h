#ifndef __CHARGER_MANAGER_H__
#define  __CHARGER_MANAGER_H__
#include "Common.h"

#define CHARGER_SAMPLE_RATE 1000

typedef enum{
  CHARGER_FALSE = 0,
  CHARGER_TRUE = 1,
  CHARGER_NOT_PLUGGED =2,
} chargerState;

class ChargerManager{
  public:
    chargerState step(float vbat, float sys_current, float charge_current);
};

#endif
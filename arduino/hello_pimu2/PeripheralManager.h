#ifndef __PERIPHERALMANAGER_H__
#define __PERIPHERALMANAGER_H__

#include "Common.h"

#define LIFT_MOTOR 1
#define OMNI_0_MOTOR 2
#define OMNI_1_MOTOR 3
#define OMNI_2_MOTOR 4
#define ARM_MOTOR 5
#define EOA_MOTOR 6
#define ALL 7

class PeripheralManager
{
    public:
    void fast_actuator_control(bool en);
    void rpc_actuator_control(uint8_t actuator, uint8_t enable);
    void peripheral_sleep_state();
    void peripheral_active_state();
};

#endif
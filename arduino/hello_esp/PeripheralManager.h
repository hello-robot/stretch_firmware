#ifndef __PERIPHERAL_MANAGER_H__
#define  __PERIPHERAL_MANAGER_H__

#include <Arduino.h>

class PeripheralManager {
    public:
        void gpio_init();
        void disable_12v0(bool disable);
        void enable_aux_20v0(bool enable);
        void disable_lidar(bool disable);
        void pimu_reset();
        void pimu_bootloader_mode();
        void peripheral_sleep_state();
        void peripheral_wakeup_state();
};
#endif
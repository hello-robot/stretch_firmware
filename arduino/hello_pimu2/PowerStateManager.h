#ifndef __POWER_STATE_MANAGER_H__
#define __POWER_STATE_MANAGER_H__


#include "Common.h"
#include "PeripheralManager.h"
#include "EspManager.h"
#include "LightBarManager.h"
#include "IMU_BNO085.h"
#include "BatteryManager.h"
#include "TimeManager.h"


#define SLEEP_PWM_BRIGHTNESS (uint8_t)25 // Target PWM for the power button LE



typedef enum{
    STATE_BOOTED,
    STATE_NOT_BOOTED
} system_on_status_t;

typedef enum{
    STATE_ACTIVE,
    STATE_SLEEP,
    STATE_SHUTDOWN_CHRG,
    STATE_SLEEP_CHRG,
    STATE_USER_FEEDBACK
}system_pwr_state;



class PowerStateManager
{
    public:
    PowerStateManager(PeripheralManager& pm, EspManager& em, LightBarManager& lb, BatteryManager& bm ) :
    _peripheral_manager(pm), _esp_manager(em), _lightbar_manager(lb), _battery_manager(bm){}
    void step();
    bool check_boot_sts();
    void enter_sleep();
    void enter_wake(system_pwr_state st);
    void power_state_setup();
    void enter_chrg_sleep(system_pwr_state st);
    void enter_sd_to_wake();
    unsigned long light_bar_st_time = 0;
    bool light_bar_indication = false;
    

    void enableTC1();
    system_pwr_state current_pwr_state;
    system_pwr_state feedback_next_pwr_state;
    volatile bool sleep_mode_set = false;

    private:
    PeripheralManager& _peripheral_manager;
    EspManager& _esp_manager;
    LightBarManager& _lightbar_manager;
    BatteryManager& _battery_manager;
    void set_up_button();

};

#endif
#ifndef __BATTERYMANAGER_H__
#define __BATTERYMANAGER_H__

#include "INA228.h"
#include "AnalogManager.h"
#include "Common.h"
#include "Transport.h"
#include "TimeManager.h"

#define CHARGING_CURRENT 5.57f

#define BMS_SAMPLE_RATE 1000
#define BMS_FRAMING_TIMEOUT 100000



enum BmsCommState {
    BMS_START,
    BMS_RX_IDLE,
    BMS_TX_IDLE,
    BMS_PARSE
};

class BatteryManager
{
public:
    void init();
    void step(float chrg_current,float adapter_v);
    void get_currents(float chrg_current);
    void charging_state(float adapter_v);
    void charger_enable(bool en);
    int get_battery_soc(float voltage, bool charger_connected);
    

    float voltage_battery;
    float current_sys;
    float current_battery;
    float current_charger;

    bool flag_charger_connected = false;
    bool flag_charger_is_charging = false;

    int battery_soc;
    int current_soc = 100;

    private:
        bool _flag_charger_enabled = true;
        bool _chrg_done = false;
        Crc16* _crc;
        COBS* _cobs;
        uint8_t _rx_buffer[256];
        uint8_t _tx_buffer[256];
        uint8_t rx_len = 0;
        BmsCommState bms_state = BMS_START;
        void _bms_step(float charging_current);
        bool _validate_crc(uint8_t* buf, uint8_t len);
        void _send_bms_read_packet(uint16_t reg_add, uint16_t reg_count);
        void _get_bms_data(uint8_t *buf, float charging_current);
        
};

#endif
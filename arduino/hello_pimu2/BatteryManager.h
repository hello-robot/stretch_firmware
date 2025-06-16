#ifndef __BATTERYMANAGER_H__
#define __BATTERYMANAGER_H__

#include "AnalogManager.h"
#include "Common.h"
#include "Transport.h"
#include "TimeManager.h"

#define BMS_SAMPLE_RATE 1000 //in ms
#define BMS_FRAMING_TIMEOUT 100 //in ms

#define BMS_VOLTAGE_ADDR 0x00

#define SOC_LED_0_POS 0x00
#define SOC_LED_1_POS 0x01
#define SOC_LED_2_POS 0x02
#define SOC_LED_3_POS 0x03
#define ALARM_LED_POS 0x04
#define CHARG_CUR_LIM_STATE_POS 0x04
#define PRE_DISCHARGE_STATE_POS 0x05
#define DISCHARGE_MOS_STATE_POS 0x06
#define CHARG_MOS_STATE_POS 0x07

#define SOC_LED_0_MSK (0x01 << SOC_LED_0_POS)
#define SOC_LED_1_MSK (0x01 << SOC_LED_1_POS)
#define SOC_LED_2_MSK (0x01 << SOC_LED_2_POS)
#define SOC_LED_3_MSK (0x01 << SOC_LED_3_POS)
#define ALARM_LED_MSK (0x01 << ALARM_LED_POS)

#define CHARG_CUR_LIM_STATE_MSK (0x01 << CHARG_CUR_LIM_STATE_POS)
#define PRE_DISCHARGE_STATE_MSK (0x01 << PRE_DISCHARGE_STATE_POS)
#define DISCHARGE_MOS_STATE_MSK (0x01 << DISCHARGE_MOS_STATE_POS)
#define CHARG_MOS_STATE_MSK (0x01 << CHARG_MOS_STATE_POS)



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
    void charging_state(float adapter_v);
    void charger_enable(bool en);

    float voltage_battery;
    float current_sys;
    float current_battery;
    uint8_t battery_soc;
    uint8_t battery_soh;
    uint16_t battery_cycles;
    int8_t battery_cell_temp_1;
    int8_t battery_cell_temp_2;
    int8_t battery_cell_temp_3;
    int8_t battery_ambient_temp;
    int8_t battery_mosfet_temp;
    bool charging_mos_status;
    bool discharging_mos_state;
    bool pre_dischargin_mos_state;
    bool chrg_current_limit_mos_state;
    bool soc_led_0;
    bool soc_led_1;
    bool soc_led_2;
    bool soc_led_3;
    bool alarm_led;

    bool flag_charger_connected = false;
    bool flag_charger_is_charging = false;
    bool bms_ready = false;

    private:
        bool _flag_charger_enabled = true;
        bool _chrg_done = false;
        Crc16* _crc;
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
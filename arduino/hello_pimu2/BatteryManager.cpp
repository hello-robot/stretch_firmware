#include "BatteryManager.h"
unsigned long _bms_last_sample_time=0;
unsigned long response_start_time = 0;


void BatteryManager::init() {
    _crc = &crc;
    pinMode(PIN_TX_EN, OUTPUT);
    bms_state = BMS_START;
    Serial2.begin(9600);
    bms_startup();
}

void BatteryManager::bms_startup()
{
    digitalWrite(PIN_TX_EN, HIGH);
    _send_bms_read_packet(BMS_SOC_ADDR, 0x01);
    while (!(SERCOM4->USART.INTFLAG.bit.TXC));
    digitalWrite(PIN_TX_EN, LOW);
    unsigned long st = micros();
    while ((micros() - st) < 100000)
    {
        while (Serial2.available())
        {
            _rx_buffer[rx_len++] = Serial2.read();
            if (rx_len >= 5 && rx_len == _rx_buffer[2] + 5)
            {  
                break;
            }
            if (rx_len >= sizeof(_rx_buffer)) {
                    // Safety: avoid buffer overflow
                    break;
                }
        }
    }
    if (_validate_crc(_rx_buffer, rx_len))
    {
        battery_soc = _rx_buffer[4];
        bms_ready = true;
    }

}

void BatteryManager::step(float chrg_current, float adapter_v){
    _bms_step(chrg_current);
    if (bms_ready)
    {
        charging_state(adapter_v);
    }
    

}


void BatteryManager::charging_state(float adapter_v)
{   
    if (adapter_v >= 34 && _flag_charger_enabled && !digitalRead(CHARGER_STATE))
    {
        flag_charger_connected = true; //if the 36V charger input is present
        flag_charger_is_charging = true;
    }
    else if (adapter_v < 34)
    {
        flag_charger_connected = false;
        flag_charger_is_charging = false;
        //when adapter is disconnected default for charger to be on
        charger_enable(true);
        user_charger_control = false;
    }


    if (voltage_battery >= 28 && battery_soc > 99)
    {
        charger_enable(false);
  
    }
    else if (voltage_battery < 26.8 && !user_charger_control)
    {
        charger_enable(true);
    }
    
}

void BatteryManager::charger_enable(bool en)
{
    if (en)
    {
        digitalWrite(CHARGER_DISABLE, LOW);
        _flag_charger_enabled = true;
    }
    else
    {
        digitalWrite(CHARGER_DISABLE, HIGH);
        _flag_charger_enabled = false;
        flag_charger_is_charging = false;
    }
}


void BatteryManager::_bms_step(float charging_current)
{
    unsigned long t = time_manager.get_elapsed_time_ms();
    switch (bms_state)
    {
        case BMS_START:
        if (t - _bms_last_sample_time >= 1000)
        {
            digitalWrite(PIN_TX_EN, HIGH);
            _send_bms_read_packet(BMS_VOLTAGE_ADDR, 0x11);
            response_start_time = t;
            rx_len = 0;
            bms_state = BMS_TX_IDLE;
        }
        break;
    
        case BMS_TX_IDLE:
        if (SERCOM4->USART.INTFLAG.bit.TXC)
        {
            digitalWrite(PIN_TX_EN, LOW);
            bms_state = BMS_RX_IDLE;
        }
        break;

        case BMS_RX_IDLE:
        _read_byte(t);
        if (t - response_start_time > BMS_FRAMING_TIMEOUT)
        {
            bms_state = BMS_START;
            _bms_last_sample_time = t;
        }
        break;
    
        case BMS_PARSE:
        if (_validate_crc(_rx_buffer, rx_len))
        {
            _get_bms_data(_rx_buffer, charging_current);
        }
        bms_state = BMS_START;
        _bms_last_sample_time = t;
        break;
    }
}

void BatteryManager::_read_byte(unsigned long st)
{
    while (Serial2.available())
    {
        _rx_buffer[rx_len++] = Serial2.read();
        if (rx_len >= 5 && rx_len == _rx_buffer[2] + 5)
        {
            bms_state = BMS_PARSE;
            break;
        }

        if (rx_len >= sizeof(_rx_buffer)) {
            // Safety: avoid buffer overflow
            bms_state = BMS_START;
            _bms_last_sample_time = st;
            break;
        }
    }
}

bool BatteryManager::_validate_crc(uint8_t* buf, uint8_t len)
{
    if (len < 3) return false;
    uint16_t crc_calc = _crc->Modbus(buf, 0, len - 2);
    uint16_t crc_recv = (buf[len - 1] << 8) | buf[len - 2];
    return crc_calc == crc_recv;
}

void BatteryManager::_send_bms_read_packet(uint16_t reg_add, uint16_t reg_count)
{
    
    uint8_t modbus_tx_buffer[8];
    _crc->clearCrc();
    modbus_tx_buffer[0] = 0x01; //BMS address
    modbus_tx_buffer[1] = 0x03; //Read byte
    modbus_tx_buffer[2] = (reg_add >> 8) & 0xFF; //Register address high byte
    modbus_tx_buffer[3] = reg_add & 0xFF; //Register address low byte
    modbus_tx_buffer[4] = (reg_count >> 8) & 0xFF; //Register count hi byte
    modbus_tx_buffer[5] = reg_count & 0xFF; //Register count low byte

    uint16_t crc_t = _crc->Modbus(modbus_tx_buffer, 0, 6);// start at byte 0 and go to byte 6

    modbus_tx_buffer[6] = crc_t & 0xFF;   // CRC low byte
    modbus_tx_buffer[7] = (crc_t >> 8);  // CRC high byte
    Serial2.write(modbus_tx_buffer, sizeof(modbus_tx_buffer));
}



void BatteryManager::_get_bms_data(uint8_t *buf, float charging_current)
{
    voltage_battery = ((buf[3] << 8) | buf[4]) * 0.01f;
    current_battery = -((int16_t)((buf[5] << 8) | buf[6])) * 0.1f;
    battery_soh = buf[15];
    battery_soc = buf[16];
    battery_cycles = (buf[17] << 8 | buf[18]);
    battery_cell_temp_1 = (int8_t)buf[19];
    battery_cell_temp_2 = (int8_t)buf[20];
    battery_cell_temp_3 = (int8_t)buf[21];
    battery_ambient_temp = (int8_t)buf[22];
    battery_mosfet_temp = (int8_t)buf[23];
    alarm_l1_total_v_high = (buf[31] & ALARM_L1_BAT_V_HIGH_MSK) >> ALARM_L1_BAT_V_HIGH_POS;
    alarm_l1_cell_v_high = (buf[32] & ALARM_L1_CELL_V_HIGH_MSK) >> ALARM_L1_CELL_V_HIGH_POS;
    alarm_l1_cell_vdif_high = (buf[32] & ALARM_L1_CELL_VDIF_HIGH_MSK) >> ALARM_L1_CELL_VDIF_HIGH_POS;
    chrg_current_limit_mos_state = (buf[35] & CHARG_CUR_LIM_STATE_MSK) >> CHARG_CUR_LIM_STATE_POS;
    pre_dischargin_mos_state = (buf[35]  & PRE_DISCHARGE_STATE_MSK) >> PRE_DISCHARGE_STATE_POS;
    discharging_mos_state = (buf[35]  & DISCHARGE_MOS_STATE_MSK) >> DISCHARGE_MOS_STATE_POS;
    charging_mos_status = (buf[35]  & CHARG_MOS_STATE_MSK) >> CHARG_MOS_STATE_POS;
    soc_led_0 = (buf[36] & SOC_LED_0_MSK) >> SOC_LED_0_POS;
    soc_led_1 = (buf[36] & SOC_LED_1_MSK) >> SOC_LED_1_POS;
    soc_led_2 = (buf[36] & SOC_LED_2_MSK) >> SOC_LED_2_POS;
    soc_led_3 = (buf[36] & SOC_LED_3_MSK) >> SOC_LED_3_POS;
    alarm_led = (buf[36] & ALARM_LED_MSK) >> ALARM_LED_POS;

    // SerialUSB.print("Current: ");
    // SerialUSB.print(current_battery);
    // SerialUSB.print(" Voltage: ");
    // SerialUSB.print(voltage_battery);
    // SerialUSB.printf(" SOC: %d\n", battery_soc);
    // SerialUSB.printf("Mos temp: %d ", battery_cycles);
    // SerialUSB.printf("Charg Fet Status: %d ", charging_mos_status);
    // SerialUSB.printf("Discharg Fet Status: %d\n", discharging_mos_state);
    // SerialUSB.printf("SOC LED 0: %d ", soc_led_0);
    // SerialUSB.printf("SOC LED 1: %d ", soc_led_1);
    // SerialUSB.printf("SOC LED 2: %d ", soc_led_2);
    // SerialUSB.printf("SOC LED 3: %d ", soc_led_3);
    // SerialUSB.printf("ALARM LED: %d\n", alarm_led);

    //Discharging
    if (current_battery <= 0)
    {
        current_sys = abs(current_battery);
        current_charger = 0;
    }
    if (current_battery > 0)
    {
        current_sys = charging_current - current_battery;
    }
}
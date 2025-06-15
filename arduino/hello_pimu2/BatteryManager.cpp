#include "BatteryManager.h"



INA228 ina228(INA228_ADDRESS); // Create an instance of the INA228 class with the default I2C address

BMSFlag bms_coms;

BmsCommState bms_state = BMS_START;
unsigned long _bms_last_sample_time=0;
unsigned long response_start_time = 0;


void BatteryManager::init() {
    bms_coms.bms_flag = RS485;
    switch (bms_coms.bms_flag)
    {
        case I2C:
        // ina228.begin(Wire, 1000000); // Initialize the INA228 with the Wire library and a clock speed of 400kHz
        // ina228.init();
        // ina228.set_shunt_measurment_time();
        // ina228.set_conversion_delay();
        // ina228.set_adc_range(ADC_RANGE_163); // Set the ADC range to 163mV
        // ina228.set_alert_dialog();
        // ina228.set_oc_limit(15.0f); //Set overcurrent limit to 1A
        // ina228.set_neg_oc_limit(-10.0f); // Set the negative overcurrent limit to -1A
        // pinMode(PIN_TX_EN, INPUT);
        break;

        case RS485:
        _crc = &crc;
        _cobs = &cobs;
        pinMode(PIN_TX_EN, OUTPUT);
        Serial2.begin(9600);
        break;
    }

}

void BatteryManager::step(float chrg_current, float adapter_v){
    switch (bms_coms.bms_flag)
    {
        case I2C:
        // voltage_battery = ina228.read_vbus();
        // get_currents(chrg_current);
        // charging_state(adapter_v);
        // battery_soc = get_battery_soc(voltage_battery, flag_charger_connected);
        break;

        case RS485:
        _bms_step(chrg_current);
        charging_state(adapter_v);
        break;

    }

}

void BatteryManager::get_currents(float chrg_current) {
    
    float c = ina228.read_current();
    if (flag_charger_is_charging)
    {
        current_sys = chrg_current + c;
        current_battery = c;
        current_charger = chrg_current;
    }
    else if (!flag_charger_is_charging)
    {
        current_sys = c;
        current_charger = 0;
        current_battery = current_sys;
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
        _chrg_done = false;
    }

    switch (bms_coms.bms_flag)
    {
        case I2C:
        if (voltage_battery >= 28.8 && flag_charger_is_charging && !_chrg_done && current_battery >= -0.1)
        {
            charger_enable(false);
            _chrg_done = true;
        }
        else if (voltage_battery <= 26.8 && _chrg_done)
        {
            charger_enable(true);
            _chrg_done = false;
        }
        break;

        case RS485:
        if (voltage_battery >= 28.8)
        {
            charger_enable(false);
            _chrg_done = true;            
        }
        else if (voltage_battery <= 26.5 && _chrg_done)
        {
            charger_enable(true);
            _chrg_done = false;
        }
        break;
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

int BatteryManager::get_battery_soc(float voltage, bool charger_connected) {
    int new_soc = current_soc;

    // Allow downward SoC transitions
    if (voltage <= 23)
        new_soc = 0;
    if (voltage <= 24.0 && current_soc > 10)
        new_soc = 10;
    if (voltage <= 24.5 && current_soc > 20)
        new_soc = 20;
    if (voltage <= 24.8 && current_soc > 25)
        new_soc = 25;
    if (voltage <= 25.1 && current_soc > 50)
        new_soc = 50;
    if (voltage <= 25.3 && current_soc > 75)
        new_soc = 75;


    // Allow upward SoC transitions only if charging
    if (flag_charger_is_charging) {
        if (voltage > 28.6)
            new_soc = 100;
        else if (voltage >= 27.5 && new_soc < 75)
            new_soc = 75;
        else if (voltage >= 26.5 && new_soc < 50)
            new_soc = 50;
        else if (voltage >= 25.5 && new_soc < 25)
            new_soc = 25;
        else if (voltage >= 24.5 && new_soc < 20)
            new_soc = 20;
        else if (voltage >= 23.0 && new_soc < 10)
            new_soc = 10;
    }
    current_soc = new_soc;
    return new_soc;
}

void BatteryManager::_bms_step(float charging_current)
{
    // unsigned long t = time_manager.get_elapsed_time_ms();
    const uint32_t t = micros();
    switch (bms_state)
    {
        case BMS_START:
        if (t - _bms_last_sample_time >= 1000000)
        {
            digitalWrite(PIN_TX_EN, HIGH);
            _send_bms_read_packet(0x00, 0x0A);
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
                _bms_last_sample_time = t;
                break;
            }
        }
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
    battery_soc = buf[16];
    SerialUSB.println(battery_soc);
    //Discharging
    if (current_battery < 0)
    {
        current_sys = abs(current_battery);
    }
    if (current_battery >= 0)
    {
        current_sys = charging_current - current_battery;
    }
}
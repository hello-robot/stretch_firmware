#ifndef __INA228_H__
#define __INA228_H__

#include <Arduino.h>
#include <Wire.h>

#define INA228_ADDRESS 0x40 //A0 and A1 pins are tied to GND

//INA228 Register Addresses//
#define INA228_REG_CONFIG 0x00
#define INA228_ADC_CONFIG 0x01
#define INA228_SHUNT_CAL 0x02
#define INA228_SHUNT_TEMPCO 0x03
#define INA228_VSHUNT_VOLTAGE 0x04
#define INA228_VBUS_VOLTAGE 0x05
#define INA228_DIE_TEMP 0x06
#define INA228_CURRENT 0x07
#define INA228_POWER 0x08
#define INA228_ENERGY 0x09
#define INA228_CHARGE 0x0A
#define INA228_ALERT 0x0B
#define INA228_SHUNT_OV_THRESH 0x0C
#define INA228_SHUNT_UV_THRESH 0x0D
#define INA228_BUS_OV_THRESH 0x0E
#define INA228_BUS_UV_THRESH 0x0F
#define INA228_TEMP_LIMIT 0x10
#define INA228_PWR_LIMIT 0x11
#define INA228_MANUFACTURER_ID 0x3E
#define INA228_DEVICE_ID 0x3F

//CONFIG REGISTER bit masks and positions//
#define CONFIG_ADC_RANGE_POS (4)
#define CONFIG_ADC_RANGE_MASK (0x1 << CONFIG_ADC_RANGE_POS)
#define ADC_RANGE_163 (0x0)
#define ADC_RANGE_40 (0x1)
#define CONFIG_TEMPCOMP_POS (5)
#define CONFIG_TEMPCOMP_MASK (0x1 << CONFIG_TEMPCOMP_POS)
#define CONFIG_CONVDLY_POS (6)
#define CONFIG_CONVDLY_MASK (0xFF << CONFIG_CONVDLY_POS)
#define CONFIG_RSTACC_POS (14)
#define CONFIG_RSTACC_MASK (0x1 << CONFIG_RSTACC_POS)
#define CONFIG_RST_POS (15)
#define CONFIG_RST_MASK (0x1 << CONFIG_RST_POS)

//ADC_CONFIG REGISTER bit masks and positions//
#define ADC_CONFIG_AVG_POS (0)
#define ADC_CONFIG_AVG_MASK (0x7 << ADC_CONFIG_AVG_POS)
#define ADC_CONFIG_VTCT_POS (3)
#define ADC_CONFIG_VTCT_MASK (0x7 << ADC_CONFIG_VTCT_POS)
#define ADC_CONFIG_VSHCT_POS (6)
#define ADC_CONFIG_VSHCT_MASK (0x7 << ADC_CONFIG_VSHCT_POS)
#define ADC_CONFIG_VBUSCT_POS (9)
#define ADC_CONFIG_VBUSCT_MASK (0x7 << ADC_CONFIG_VBUSCT_POS)
#define ADC_CONFIG_MODE_POS (12)
#define ADC_CONFIG_MODE_MASK (0xF << ADC_CONFIG_MODE_POS)

#define ADC_TRIGGER_ALL (0x07) //Single shot trigger for all outputs

//Shunt Resistor Register Values//
#define SHUNT_CAL_POS (0)
#define SHUNT_CAL_MASK (0x7FFF << SHUNT_CAL_POS)
#define SHUNT_CAL_VALUE 0x15F9
#define PERCISE_SHUNT_CAL_VALUE 0x57E4
#define SHUNT_OV_SETPOINT 0x20D0
#define SHUNT_UV_SETPOINT 0x0E10
#define SHUNT_RES_VAL (0.003f) //3mOhm shunt resistor value
#define SHUNT_SCALAR_ADCRANGE_0 (0.000005f) // 5uV/LSB
#define SHUNT_SCALAR_ADCRANGE_1 (0.00000125f) // 5uV/LSB
#define SHUNT_CONV (0.0003125f)
#define SHUNT_CONV_PERCISE (0.000078125f)

//ALERT REGISTER bit masks and positions//
#define ALERT_MEMSTATUS_POS (0)
#define ALERT_MEMSTATUS_MASK (0x1 << ALERT_MEMSTATUS_POS)
#define ALERT_CNVRF_POS (1)
#define ALERT_CNVRF_MASK (0x1 << ALERT_CNVRF_POS)
#define ALERT_POL_POS (2)
#define ALERT_POL_MASK (0x1 << ALERT_POL_POS)
#define ALERT_BUSUL_POS (3)
#define ALERT_BUSUL_MASK (0x1 << ALERT_BUSUL_POS)
#define ALERT_BUSOV_POS (4)
#define ALERT_BUSOV_MASK (0x1 << ALERT_BUSOV_POS)
#define ALERT_SHUNTUL_POS (5)
#define ALERT_SHUNTUL_MASK (0x1 << ALERT_SHUNTUL_POS)
#define ALERT_SHUNTOV_POS (6)
#define ALERT_SHUNTOV_MASK (0x1 << ALERT_SHUNTOV_POS)
#define ALERT_TEMP_POS (7)
#define ALERT_TEMP_MASK (0x1 << ALERT_TEMP_POS)
#define ALERT_MATHOF_POS (9)
#define ALERT_MATHOF_MASK (0x1 << ALERT_MATHOF_POS)
#define ALERT_CHARGOF_POS (10)
#define ALERT_CHARGOF_MASK (0x1 << ALERT_CHARGOF_POS)
#define ALERT_ENERGYOF_POS (11)
#define ALERT_ENERGYOF_MASK (0x1 << ALERT_ENERGYOF_POS)
#define ALERT_POLARITY_POS (12)
#define ALERT_POLARITY_MASK (0x1 << ALERT_POLARITY_POS)
#define ALERT_SLOWALERT_POS (13)
#define ALERT_SLOWALERT_MASK (0x1 << ALERT_SLOWALERT_POS)
#define ALERT_CNVR_POS (14)
#define ALERT_CNVR_MASK (0x1 << ALERT_CNVR_POS)
#define ALERT_LATCH_POS (15)
#define ALERT_LATCH_MASK (0x1 << ALERT_LATCH_POS)

//Alert Setting Values//
#define ALERT_LATCH_ENABLED (0x1)
#define ALERT_LATCH_DISABLED (0x0)
#define SLOW_ALERT_ENABLED (0x1)
#define SLOW_ALERT_DISABLED (0x0)
#define ALERT_POLARITY_NORMAL (0x0)
#define ALERT_POLARITY_INVERTED (0x1)
#define ALERT_CNVR_ENABLED (0x1)
#define ALERT_CNVR_DISABLED (0x0)


typedef enum{
    INA228_OK,
    INA228_I2C_WRITE_FAIL,
    INA228_I2C_READ_FAIL,
    INA228_INVALID_ARGUMENT
} ina228_status_t;

class INA228{
public:
    INA228(uint8_t address = INA228_ADDRESS);
    void begin(TwoWire& wirePort = Wire, uint32_t clockSpeed = 400000);
    ina228_status_t read_register(uint8_t reg, uint8_t *data, uint8_t data_size);
    ina228_status_t write_register(uint8_t reg, uint16_t *data);
    void init();
    uint16_t read_device_id();
    uint16_t read_mfg_id();
    void set_conversion_delay();
    void write_to_shunt_cal(uint16_t cal);
    void set_adc_range(uint16_t range);
    float read_vbus();
    float read_vshunt(float accuracy);
    float read_ina228_temp();
    float read_current();
    float read_power();
    float read_charge();
    float read_energy();
    void read_alert_dialog();
    void set_alert_dialog();
    void set_oc_limit(float oc_limit);
    void set_neg_oc_limit(float neg_oc_limit);
    void reset_accumlator_registers();
    void low_power_mode();
    void read_config_register();
    void set_shunt_measurment_time();

    //Alert register bit values
    uint8_t energyOF;
    uint8_t chargeOF;
    uint8_t mathOF;
    uint8_t memstat_error;
    uint8_t conversion_ready;

    //Config register bit values
    uint8_t adc_range;
    uint8_t temp_comp;
    uint8_t conv_dly;
    uint8_t rst_acc;
    uint8_t rst_bit;
    
private:
    uint8_t _address;
    TwoWire* _wire;
};
#endif
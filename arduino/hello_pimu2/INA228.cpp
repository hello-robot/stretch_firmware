#include "INA228.h"
#include <Wire.h>

INA228::INA228(uint8_t address) {
    _address = address;
    _wire = &Wire;  // Default to Wire unless changed
}

void INA228::begin(TwoWire& wirePort, uint32_t clockSpeed) {
    _wire = &wirePort;
    _wire->begin();
    _wire->setClock(clockSpeed);  // Set I2C clock speed 
}


ina228_status_t INA228::read_register(uint8_t reg, uint8_t *data, uint8_t data_size) {
    //Send START+SLAVE ADDRESS with write bit to INA228
    _wire->beginTransmission(_address);

    //Sends the register address to read from
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
        return INA228_I2C_WRITE_FAIL;
    }
    //Sends START+SAVE ADDRESS with read bit to INA228
    _wire->requestFrom(INA228_ADDRESS, data_size);

    //If we don't receive 2 bytes, return error
    if (_wire->available() != data_size) {
        return INA228_I2C_READ_FAIL;
    }
    
    if (data_size >= 1) data[0] = _wire->read();
    if (data_size >= 2) data[1] = _wire->read();
    if (data_size >= 3) data[2] = _wire->read();
    if (data_size >= 4) data[3] = _wire->read();
    if (data_size >= 5) data[4] = _wire->read();
    return INA228_OK;
}

ina228_status_t INA228::write_register(uint8_t reg, uint16_t* data) {
    _wire->beginTransmission(_address);
    _wire->write(reg); // Register address  
    _wire->write(*data >> 8); // Data to write
    _wire->write(*data & 0xFF); // Data to write
    if (_wire->endTransmission() != 0) {
        return INA228_I2C_WRITE_FAIL;
    }
    return INA228_OK;
}

void INA228::write_to_shunt_cal(uint16_t cal)
{
    uint16_t shunt_cal;
    shunt_cal &= ~SHUNT_CAL_MASK; // Clear the bits in the mask
    shunt_cal |= (cal << SHUNT_CAL_POS); // Set the bits in the mask
    write_register(INA228_SHUNT_CAL, &shunt_cal); // Write the register to clear the bits
}

void INA228::set_adc_range(uint16_t range)
{
    
    uint8_t buffer[2];
    uint16_t config;
    if (read_register(INA228_REG_CONFIG, buffer,  sizeof(buffer)) == INA228_OK) {
        config = (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
        config &= ~CONFIG_ADC_RANGE_MASK; // Clear the bits in the mask
        config |= (range << CONFIG_ADC_RANGE_POS); // Set the bits in the mask
        write_register(INA228_REG_CONFIG, &config); // Write the register to clear the bits
    }

}

void INA228::low_power_mode()
{
    uint8_t buffer[2];
    uint16_t adc_config;
    if (read_register(INA228_ADC_CONFIG, buffer, sizeof(buffer)) == INA228_OK) {
        adc_config = (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
        adc_config &= ~ADC_CONFIG_MODE_MASK; // Clear the bits in the mask
        adc_config |= (ADC_TRIGGER_ALL << ADC_CONFIG_MODE_POS); // Set the bits in the mask
        write_register(INA228_ADC_CONFIG, &adc_config); // Write the register to clear the bits
    }
}

void INA228::set_shunt_measurment_time()
{
    uint8_t buffer[2];
    uint16_t adc_config;
    if (read_register(INA228_ADC_CONFIG, buffer, sizeof(buffer)) == INA228_OK) {
        adc_config = (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
        adc_config &= ~ADC_CONFIG_VSHCT_MASK; // Clear the bits in the mask
        adc_config |= (0x04 << ADC_CONFIG_VSHCT_POS); // Set the bits in the mask
        write_register(INA228_ADC_CONFIG, &adc_config); // Write the register to clear the bits
    }
}



void INA228::init()
{
    set_adc_range(ADC_RANGE_163); // Set the ADC range to 163mV
    write_to_shunt_cal(SHUNT_CAL_VALUE); // Set the shunt calibration value
    set_alert_dialog();
    set_oc_limit(15.0f); //Set overcurrent limit to 1A
    set_neg_oc_limit(-10.0f); // Set the negative overcurrent limit to -1A
}


uint16_t INA228::read_mfg_id()
{
    uint8_t buffer[2];
    if (read_register(INA228_MANUFACTURER_ID, buffer,  sizeof(buffer)) == INA228_OK) {

        return (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
    }
    return 0; // Return 0 if read fails
}

uint16_t INA228::read_device_id()
{
    uint8_t buffer[2];
    if (read_register(INA228_DEVICE_ID, buffer, sizeof(buffer)) == INA228_OK) {

        return (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
    }
    return 0; // Return 0 if read fails
}



void INA228::read_alert_dialog()
{
    uint8_t buffer[2];
    uint16_t alert_reg;
    if (read_register(INA228_ALERT, buffer, sizeof(buffer)) == INA228_OK) {
        alert_reg = (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
        energyOF = (alert_reg & ALERT_ENERGYOF_MASK) >> ALERT_ENERGYOF_POS; // Check if the energy overflow bit is set
        chargeOF = (alert_reg & ALERT_CHARGOF_MASK) >> ALERT_CHARGOF_POS; // Check if the charge overflow bit is set
        mathOF = (alert_reg & ALERT_MATHOF_MASK) >> ALERT_MATHOF_POS; // Check if the math overflow bit is set
        memstat_error = (alert_reg & ALERT_MEMSTATUS_MASK) >> ALERT_MEMSTATUS_POS; // Check if the memory status bit is set
        conversion_ready = (alert_reg & ALERT_CNVRF_MASK) >> ALERT_CNVRF_POS; // Check if the memory status bit is set
    }

}

void INA228::set_alert_dialog()
{
    uint8_t buffer[2];
    uint16_t alert_reg;
    if (read_register(INA228_ALERT, buffer, sizeof(buffer)) == INA228_OK) {
        alert_reg = (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
        alert_reg &= ~(ALERT_LATCH_MASK | ALERT_CNVR_MASK | ALERT_SLOWALERT_MASK | ALERT_POLARITY_MASK); // Clear the bits in the mask
        alert_reg |= (ALERT_LATCH_ENABLED << ALERT_LATCH_POS); // Set the latch bit
        alert_reg |= (ALERT_CNVR_DISABLED << ALERT_CNVR_POS); // Set the CNVR bit
        alert_reg |= (SLOW_ALERT_DISABLED << ALERT_SLOWALERT_POS); // Set the slow alert bit
        alert_reg |= (ALERT_POLARITY_NORMAL << ALERT_POLARITY_POS); // Set the polarity bit
        write_register(INA228_ALERT, &alert_reg); // Write the register to set the bits
    }
}

void INA228::set_conversion_delay()
{
    uint8_t buffer[2];
    uint16_t config_reg;
    if (read_register(INA228_REG_CONFIG, buffer, sizeof(buffer)) == INA228_OK) {
        config_reg = (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
        config_reg &= ~CONFIG_CONVDLY_MASK; // Clear the conversion delay bits
        config_reg |= (0x00 << CONFIG_CONVDLY_POS); // Set the conversion delay in the proper position
        write_register(INA228_REG_CONFIG, &config_reg); // Write the register to set the bits
    }
}

void INA228::read_config_register()
{
    uint8_t buffer[2];
    uint16_t config_reg;
    if (read_register(INA228_REG_CONFIG, buffer, sizeof(buffer)) == INA228_OK) {
        config_reg = (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
        adc_range = (config_reg & CONFIG_ADC_RANGE_MASK) >> CONFIG_ADC_RANGE_POS; // Check if the energy overflow bit is set
        temp_comp = (config_reg & CONFIG_TEMPCOMP_MASK) >> CONFIG_TEMPCOMP_POS; // Check if the charge overflow bit is set
        conv_dly = (config_reg & CONFIG_CONVDLY_MASK) >> CONFIG_CONVDLY_POS; // Check if the math overflow bit is set
        rst_acc = (config_reg & CONFIG_RSTACC_MASK) >> CONFIG_RSTACC_POS; // Check if the memory status bit is set
        rst_bit = (config_reg & CONFIG_RST_MASK) >> CONFIG_RST_POS; // Check if the memory status bit is set

    }
}

void INA228::set_oc_limit(float oc_limit)
{
    //SHUNT_OV is its own register can just write directly to it
    uint16_t oc_limit_raw = (uint16_t)((SHUNT_RES_VAL*oc_limit)/SHUNT_SCALAR_ADCRANGE_0);
    write_register(INA228_SHUNT_OV_THRESH, &oc_limit_raw); // Write the register to clear the bits
}

void INA228::set_neg_oc_limit(float neg_oc_limit)
{
    //SHUNT_UV is its own register can just write directly to it
    int16_t raw = (int16_t)(((SHUNT_RES_VAL*neg_oc_limit)/SHUNT_SCALAR_ADCRANGE_0));
    uint16_t neg_oc_limit_raw = (uint16_t)raw;
    write_register(INA228_SHUNT_UV_THRESH, &neg_oc_limit_raw); // Write the register to clear the bits
}

void INA228::reset_accumlator_registers()
{
    uint8_t buffer[2];
    uint16_t config;
    if (read_register(INA228_REG_CONFIG, buffer, sizeof(buffer)) == INA228_OK) {
        config = (uint16_t)buffer[0] << 8 | buffer[1]; // Combine the two bytes into a single 16-bit value
        config &= ~CONFIG_RSTACC_MASK;
        config |= (0x1 << CONFIG_RSTACC_POS); //Set 0x0 tio RSTACC bit
        write_register(INA228_REG_CONFIG, &config); // Write the register to clear the bits
    }
}

float INA228::read_vbus()
{
    uint8_t buffer[3];

    if (read_register(INA228_VBUS_VOLTAGE, buffer, sizeof(buffer)) == INA228_OK) {
        
        uint32_t raw = ((uint32_t)buffer[0] << 16 | buffer[1] << 8 | buffer[2]) >> 4;
        int32_t vbus_raw =(int32_t)(raw << 12) >> 12; // Sign extend the 20-bit value
        return (float)vbus_raw * 0.0001953125f; // Returns in Volts
    }
    return 0; // Return 0 if read fails
}

float INA228::read_vshunt(float accuracy)
{
    uint8_t buffer[3];

    if (read_register(INA228_VSHUNT_VOLTAGE, buffer, sizeof(buffer)) == INA228_OK) {
        
        uint32_t raw = ((uint32_t)buffer[0] << 16 | buffer[1] << 8 | buffer[2]) >> 4;
        int32_t vshunt_raw =(int32_t)(raw << 12) >> 12; // Sign extend the 20-bit value

        return (float)vshunt_raw * accuracy; // Returns in millivolts
    }
    return 0; // Return 0 if read fails
}

float INA228::read_current()
{
    uint8_t buffer[3];

    if (read_register(INA228_CURRENT, buffer, sizeof(buffer)) == INA228_OK) {
        
        uint32_t raw = ((uint32_t)buffer[0] << 16 | buffer[1] << 8 | buffer[2]) >> 4;
        int32_t current_raw =(int32_t)(raw << 12) >> 12; // Sign extend the 20-bit value
        return (float)current_raw * 0.000143051f; // Returns in Amps
    }
    return 0; // Return 0 if read fails
}

float INA228::read_power()
{
    uint8_t buffer[3];

    if (read_register(INA228_POWER, buffer, sizeof(buffer)) == INA228_OK) {
        
        uint32_t raw = ((uint32_t)buffer[0] << 16 | buffer[1] << 8 | buffer[2]);
        return (float)raw * 3.2 * 0.000143051f; // Returns in W
    }
    return 0; // Return 0 if read fails
}

float INA228::read_ina228_temp()
{
    uint8_t buffer[2];

    if (read_register(INA228_DIE_TEMP, buffer, sizeof(buffer)) == INA228_OK) {
        uint16_t raw = (uint16_t)buffer[0] << 8 | buffer[1];
        int16_t temp_raw = (int16_t)raw; // Sign extend the 24-bit value
        return (float)temp_raw * 0.0078125f; // Returns in degrees Celsius
    }
    return 0; // Return 0 if read fails
}

float INA228::read_charge()
{
    uint8_t buffer[5];
    if (read_register(INA228_CHARGE, buffer, sizeof(buffer)) == INA228_OK) {
        uint64_t raw = (uint64_t)buffer[0] << 32 | buffer[1] << 24 | buffer[2] << 16 | buffer[3] << 8 | buffer[4];
        int64_t charge_raw= (int64_t)(raw << 24) >> 24; // Sign extend the 40-bit value
        return (float)charge_raw * 3.2 * 0.000143051f; // Returns in Ampere-seconds
    }
    return 0; // Return 0 if read fails
}

float INA228::read_energy()
{
    uint8_t buffer[5];
    if (read_register(INA228_ENERGY, buffer, sizeof(buffer)) == INA228_OK){
        uint64_t raw = (uint64_t)buffer[0] << 32 | buffer[1] << 24 | buffer[2] << 16 | buffer[3] << 8 | buffer[4];
        return ((float)raw * 16 * 3.2 * 0.000143051f)/3600; // Returns in Wh
    }
    return 0;
}
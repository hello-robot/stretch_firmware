#include "BatteryManager.h"
#include "Pimu.h"
INA228 ina228(INA228_ADDRESS); // Create an instance of the INA228 class with the default I2C address


void BatteryManager::init() {
    ina228.begin(Wire, 400000); // Initialize the INA228 with the Wire library and a clock speed of 400kHz
    ina228.init();
    ina228.set_shunt_measurment_time();
    ina228.set_conversion_delay();
    ina228.set_adc_range(ADC_RANGE_163); // Set the ADC range to 163mV
    ina228.set_alert_dialog();
    ina228.set_oc_limit(15.0f); //Set overcurrent limit to 1A
    ina228.set_neg_oc_limit(-10.0f); // Set the negative overcurrent limit to -1A
}

void BatteryManager::step(){
    voltage = ina228.read_vbus();
    sys_current = ina228.read_current();
}

float BatteryManager::read_vbus(){
    return ina228.read_vbus();
}
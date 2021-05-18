/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc Calc

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html  

  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#include "Accel.h"
#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

/* Assign a unique ID to this sensor at the same time */
/* Uncomment following line for default Wire bus      */
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);
//https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL343.pdf

bool accel_valid=false;
bool filter_init=true;
float ax=0;
float ay=0;
float az=0;
uint32_t accel_debug;

uint32_t single_tap_count=0;
uint32_t double_tap_count=0;

int_config g_int_config_enabled = { 0 };
int_config g_int_config_map = { 0 };
uint32_t g_ints_fired = 0;

void int1_isr(void)
{
    single_tap_count++;
    g_ints_fired++;
}
void int2_isr()
{
    double_tap_count++;
    g_ints_fired++;
}
void setupAccel(void)
{
  /* Initialise the sensor */
  attachInterrupt(digitalPinToInterrupt(ACCEL_INT1), int1_isr, RISING);
  //attachInterrupt(digitalPinToInterrupt(ACCEL_INT2), int2_isr, RISING);
  if(accel.begin())
  {
    accel_valid=true;
    accel.setRange(ADXL343_RANGE_4_G);

    g_int_config_enabled.bits.single_tap = true;
    //g_int_config_enabled.bits.double_tap = true;
    accel.enableInterrupts(g_int_config_enabled);

    g_int_config_map.bits.single_tap = ADXL343_INT1;
    //g_int_config_map.bits.double_tap = ADXL343_INT2;
    accel.mapInterrupts(g_int_config_map);
    
    //accel_debug=accel.readRegister(ADXL343_REG_DUR);
    //accel_debug=accel.readRegister(ADXL343_REG_THRESH_TAP);
    
  }
}
void setSingleTapSensitivity(uint8_t dur, uint8_t thresh)
{
  accel.writeRegister(ADXL343_REG_THRESH_TAP,thresh); 
  accel.writeRegister(ADXL343_REG_DUR,dur); 
}

void setAccelRange(uint8_t r)
{
  if (accel_valid)
  {
    if (r==2)
      accel.setRange(ADXL343_RANGE_2_G);
    if (r==4)
      accel.setRange(ADXL343_RANGE_4_G);
    if (r==8)
      accel.setRange(ADXL343_RANGE_8_G);
    if (r==16)
      accel.setRange(ADXL343_RANGE_16_G);
  }
  
}
void stepAccel(void)
{
  sensors_event_t event;
  if (accel_valid)
  {
     accel.getEvent(&event);
     ax=event.acceleration.x;
     ay=event.acceleration.y;
     az=event.acceleration.z;
  }
  else
  {
    ax=0;
    ay=0;
    az=0;
  }
  
  while (g_ints_fired)
  {
    accel.checkInterrupts(); //uint8_t source = accel.readRegister(ADXL343_REG_INT_SOURCE);//checkInterrupts();
    g_ints_fired--;
  }
}

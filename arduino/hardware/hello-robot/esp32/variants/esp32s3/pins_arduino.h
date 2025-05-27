#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>
#include "soc/soc_caps.h"

#define USB_VID 0x303a
#define USB_PID 0x1001

//Analog pins
static const uint8_t PIN_AUX_20V0_VOLT = 4;
static const uint8_t PIN_12V0_VOLT = 5;

//Ouput pins
static const uint8_t PIN_12V0_DISABLE = 6;
static const uint8_t PIN_RPI_PWR_DISABLE = 7;
static const uint8_t PIN_ESP_STS_LED = 15;
static const uint8_t PIN_STS_LEDS_DISABLE = 16;
static const uint8_t PIN_RPI_SD = 8;
static const uint8_t PIN_LIDAR_DISABLE = 9;
static const uint8_t PIN_CPU_SD = 10;
static const uint8_t PIN_ROBOT_ACTIVE = 11;
static const uint8_t PIN_PIMU_RESET = 12;
static const uint8_t PIN_LATCH = 13;
static const uint8_t PIN_AUX_20VO_EN = 35;
static const uint8_t PIN_DCM_MODE_EN = 36;

//Input pins
static const uint8_t PIN_RPI_STS = 2;
static const uint8_t PIN_CPU_STS = 1;
static const uint8_t PIN_BARREL_FAULT = 14;
static const uint8_t PIN_ADAPTER_FAULT = 21;

//UART pins
static const uint8_t PIN_UART1_RX = 18; //Comms between ESP32 and SAMD
static const uint8_t PIN_UART1_TX = 17; //Comms between ESP32 and SAMD

static const uint8_t PIN_UART0_RX = 44;
static const uint8_t PIN_UART0_TX = 43;

static const uint8_t PIN_UART2_RX = 41;
static const uint8_t PIN_UART2_TX = 42;

//I2C pins
static const uint8_t PIN_I2C0_SDA = 38;
static const uint8_t PIN_I2C0_SCL = 37;

static const uint8_t PIN_I2C1_SDA = 40;
static const uint8_t PIN_I2C1_SCL = 39;


#endif /* Pins_Arduino_h */

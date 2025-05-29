#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stdint.h>


#define UART_STS_VOLTAGE 0x01
#define UART_STS_CURRENT 0x02
#define UART_TRIGGER 0x03
#define UART_PWR_SLEEP 0x04
#define UART_PWR_WAKE 0x05

struct VoltageStatus {
    float voltage_battery; // Voltage in Volts
    float voltage_20v0;
    float voltage_5v0;
    float voltage_36v0;
};

#endif
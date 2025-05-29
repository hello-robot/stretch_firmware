#ifndef __TRANSPORT_H__
#define __TRANSPORT_H__
#include "Arduino.h"
#include "COBS.h"
#include "Crc16.h"

bool stepTransport(void (*rpc_callback)());
void setupTransport();               

extern uint8_t  rpc_in[];
extern uint8_t  rpc_out[];
extern uint16_t num_byte_rpc_out;
extern uint16_t num_byte_rpc_in;
extern COBS cobs;
extern Crc16 crc;

#endif

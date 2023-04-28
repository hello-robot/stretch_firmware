#ifndef __TRANSPORT_H__
#define __TRANSPORT_H__
#include "Arduino.h"
bool stepTransport(void (*rpc_callback)(), int version);
void setupTransport();               

extern uint8_t  rpc_in[];
extern uint8_t  rpc_out[];
extern uint16_t num_byte_rpc_out;
extern uint16_t num_byte_rpc_in;


#endif

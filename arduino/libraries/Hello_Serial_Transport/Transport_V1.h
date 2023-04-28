#ifndef __TRANSPORT_H__
#define __TRANSPORT_H__
#include "Arduino.h"
bool stepTransport(void (*rpc_callback)());
void setupTransport();               

extern uint8_t  rpc_in[]; //RPC data recieved frm
extern uint16_t num_byte_rpc_in; //num bytes of data recieved

extern uint8_t  rpc_out[]; //data response to RPC
extern uint16_t num_byte_rpc_out; //num bytes of response



#endif

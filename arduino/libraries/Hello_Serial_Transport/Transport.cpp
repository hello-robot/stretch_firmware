#include "Transport.h"
#include "COBS.h"
#include "Crc16.h"
/////////////////////////////////////////////////////////////////////////////////
/* Hello Robot RPC Serial Transport protocol

  An RPC exchange happens as:

  Host: Send array of bytes [RPC_ID1 data1...]
  Arduino: Reply with [RPC_ID2 data2...]
  Data can be up to 1024 bytes

  Each RPC Send & RPC Reply is further decomposed into a series of smaller data Blocks so that the
  serial buffers don't choke.

  Each Block is packed into a Frame which includes Cobbs Encoding/CRC handling so that data integrity is garunteed.

  RPC/Block/Frame sizes and timings are hardcoded/tuned for our particular application requirements

  So generally:
  1. Host: RPC Send -> Blocks -> Frames -> Serial
  2. Arduino: process the sent RPC request
  3. Arduino: RPC Reply -> Blocks ->Frames -> SerialUSB
  4. Host: Process the RPC reply

 */
/////////////////////////////////////////////////////////////////////////////////


#define RPC_PUSH_FRAME_FIRST_MORE 101
#define RPC_PUSH_FRAME_FIRST_ONLY 102
#define RPC_PUSH_FRAME_MORE 103
#define RPC_PUSH_FRAME_LAST  104
#define RPC_PUSH_ACK  105

#define RPC_PULL_FRAME_FIRST 108
#define RPC_PULL_FRAME_MORE 109
#define RPC_PULL_FRAME_ACK_MORE  110
#define RPC_PULL_FRAME_ACK_LAST 111

#define RPC_FRAME_DATA_MAX_BYTES  58
#define RPC_DATA_MAX_BYTES  1024

uint8_t rpc_in[RPC_DATA_MAX_BYTES+1];
uint8_t rpc_out[RPC_DATA_MAX_BYTES+1];
uint16_t num_byte_rpc_out=0;
uint16_t num_byte_rpc_in=0;
uint16_t byte_in_cnt=0;
uint16_t byte_out_cnt=0;
bool in_transaction=false;

//////////////////////////COBS Framing ///////////////////////////////////////////////////////
/*
Based on COBS.h from
https://github.com/bakercp/PacketSerial
MIT License
Copyright (c) 2017 Christopher Baker https://christopherbaker.net
*/
#define COBBS_PACKET_MARKER 0
#define COBBS_FRAME_SIZE 63
#define FRAMING_TIMEOUT 100000 //0.1s or 10hz minimum rate to send a block of 32bytes
uint8_t frame_in[COBBS_FRAME_SIZE]; //Room for encoding expansion
uint8_t frame_out[COBBS_FRAME_SIZE];
uint8_t rx_buffer[COBBS_FRAME_SIZE];
uint8_t tx_buffer[COBBS_FRAME_SIZE];
COBS cobs;
Crc16 crc;
bool rx_buffer_overflow = false;
int  rx_buffer_idx=0;

bool ready_rpc_state()
{
    num_byte_rpc_in=0;
    byte_in_cnt=0;
    num_byte_rpc_out=0;
    byte_out_cnt=0;
    in_transaction=false;
}


//Return 1 if got a valid frame
//Otherwise this will clear out any bytes in the RX buffer until FRAMING_TIMEOUT
bool receive_frame(uint8_t * buf, uint8_t & n,void (*rpc_callback2)())
{
    unsigned long t_start =micros();
    uint8_t byte_in;
    while((micros()-t_start)<FRAMING_TIMEOUT) //data may be sparse, keep polling until first byte arrives, then get whole packet
    {

		if(SerialUSB.available()>0)
		{
		rpc_callback2();
            byte_in = SerialUSB.read();
            t_start =micros();  //Restart the timer otherwise can have race condition as prior start point may be close to expiring

            if (byte_in == COBBS_PACKET_MARKER)
            {
                n = cobs.decode(rx_buffer, rx_buffer_idx, buf);
                crc.clearCrc();
                uint16_t crc1 = crc.Modbus(buf,0,n-2);
                uint16_t crc2 = (buf[n-2]<<8)|buf[n-1];
                n=n-2;
                rx_buffer_idx = 0;
                rx_buffer_overflow = false;
                return (crc1==crc2);
            }
            else
            {
                if ((rx_buffer_idx + 1) < COBBS_FRAME_SIZE)
                {
                    rx_buffer[rx_buffer_idx++] = byte_in;
                }
                else
                {
                    // The buffer will be in an overflowed state if we write
                    // so set a buffer overflowed flag.
                    rx_buffer_overflow = true;
                    //ready_rpc_state();
                }
            }
        }
    }
    return 0;
}

void send_frame(uint8_t * buf, uint8_t n)
{
        crc.clearCrc();
        uint16_t value = crc.Modbus(buf,0,n);
        buf[n++]=(value>>8)&0xff;
        buf[n++]= value&0xff;
        int nb = cobs.encode(buf,n,tx_buffer);
        tx_buffer[nb++]=COBBS_PACKET_MARKER;
        SerialUSB.write(tx_buffer, nb);
}
/////////////////////////////////////////////////////////////////////////////////

bool stepTransport(void (*rpc_callback)(),void (*rpc_callback2)())
{
  uint8_t nbytes_rx;
  uint16_t nbo;

  if (receive_frame(frame_in, nbytes_rx,rpc_callback2))
  {

    switch (frame_in[0])
    {
    //////////////////////// PUSH ///////////////////////////////////
      case RPC_PUSH_FRAME_FIRST_ONLY:
          ready_rpc_state();
          memcpy(rpc_in,frame_in+1,nbytes_rx-1);
          num_byte_rpc_in=nbytes_rx-1;
          (*rpc_callback)();
          frame_out[0]=RPC_PUSH_ACK;
          memcpy(frame_out+1,rpc_out,num_byte_rpc_out);
          send_frame(frame_out,num_byte_rpc_out+1);
          ready_rpc_state();
          break;
      case RPC_PUSH_FRAME_FIRST_MORE: //first of multi frame
          ready_rpc_state();
          in_transaction=true;
          memcpy(rpc_in,frame_in+1,nbytes_rx-1);
          byte_in_cnt=byte_in_cnt+nbytes_rx-1;
          frame_out[0]=RPC_PUSH_ACK;
          send_frame(frame_out,1);
          break;
      case RPC_PUSH_FRAME_MORE: //first of multi frame
          if (in_transaction)
          {
              memcpy(rpc_in+byte_in_cnt,frame_in+1,nbytes_rx-1);
              byte_in_cnt=byte_in_cnt+nbytes_rx-1;
              frame_out[0]=RPC_PUSH_ACK;
              send_frame(frame_out,1);
          }
          else
            ready_rpc_state();
          break;
      case RPC_PUSH_FRAME_LAST: //Last frame of multi frame, or first and only frame
          if (in_transaction)
          {
              memcpy(rpc_in+byte_in_cnt,frame_in+1,nbytes_rx-1);
              num_byte_rpc_in=nbytes_rx-1;
              (*rpc_callback)();
              frame_out[0]=RPC_PUSH_ACK;
              memcpy(frame_out+1,rpc_out,num_byte_rpc_out);
              send_frame(frame_out,num_byte_rpc_out+1);
              ready_rpc_state();
          }
          else
            ready_rpc_state();
          break;

    //////////////////////// PULL ///////////////////////////////////
      case RPC_PULL_FRAME_FIRST:
          ready_rpc_state();
          num_byte_rpc_in=nbytes_rx-1;
          memcpy(rpc_in,frame_in+1,num_byte_rpc_in); //Get single frame RPC request for a pull and process it
          (*rpc_callback)();
          in_transaction=true;
          //fall through
      case RPC_PULL_FRAME_MORE:
            if (in_transaction)
            {
                nbo = min(RPC_FRAME_DATA_MAX_BYTES,num_byte_rpc_out-byte_out_cnt);
                if (num_byte_rpc_out-byte_out_cnt<=RPC_FRAME_DATA_MAX_BYTES) //Last frame?
                {
                    frame_out[0]=RPC_PULL_FRAME_ACK_LAST;
                    memcpy(frame_out+1, rpc_out+byte_out_cnt,nbo);
                    send_frame(frame_out,nbo+1);
                    ready_rpc_state();
                    break;
                }
              //Multi-frame pull
               frame_out[0]=RPC_PULL_FRAME_ACK_MORE;
               memcpy(frame_out+1, rpc_out+byte_out_cnt,RPC_FRAME_DATA_MAX_BYTES);
               send_frame(frame_out,RPC_FRAME_DATA_MAX_BYTES+1);
               byte_out_cnt=byte_out_cnt+RPC_FRAME_DATA_MAX_BYTES;
            }
            else
                ready_rpc_state();
          break;
    };
    return true;
  }
  return false;
}


/////////////////////////////////////////////////////////////////////////////////

void setupTransport() {
    ready_rpc_state();
}



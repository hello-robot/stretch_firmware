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


#define RPC_START_NEW_RPC 100
#define RPC_ACK_NEW_RPC 101

#define RPC_SEND_BLOCK_MORE  102
#define RPC_ACK_SEND_BLOCK_MORE  103
#define RPC_SEND_BLOCK_LAST  104
#define RPC_ACK_SEND_BLOCK_LAST  105

#define RPC_GET_BLOCK  106
#define RPC_ACK_GET_BLOCK_MORE  107
#define RPC_ACK_GET_BLOCK_LAST  108

#define RPC_BLOCK_SIZE 32
#define RPC_DATA_SIZE 1024


uint8_t rpc_in[RPC_DATA_SIZE+1];
uint8_t rpc_out[RPC_DATA_SIZE+1];
uint16_t num_byte_rpc_out=0;
uint16_t num_byte_rpc_in=0;
uint16_t byte_in_cnt=0;
uint16_t byte_out_cnt=0;


//////////////////////////COBS Framing ///////////////////////////////////////////////////////
/*
Based on COBS.h from
https://github.com/bakercp/PacketSerial
MIT License
Copyright (c) 2017 Christopher Baker https://christopherbaker.net
*/
#define COBBS_PACKET_MARKER 0
#define COBBS_FRAME_SIZE RPC_BLOCK_SIZE*2
#define FRAMING_TIMEOUT 100000 //0.1s or 10hz minimum rate to send a block of 32bytes
uint8_t frame_in[COBBS_FRAME_SIZE]; //Room for encoding expansion
uint8_t frame_out[COBBS_FRAME_SIZE];
uint8_t rx_buffer[COBBS_FRAME_SIZE];
uint8_t tx_buffer[COBBS_FRAME_SIZE];
COBS cobs;
Crc16 crc;
bool rx_buffer_overflow = false;
int  rx_buffer_idx=0;


//Return 1 if got a valid frame
//Recieve the frame iteratively over a series of calls
bool receive_frame_itr(uint8_t * buf, uint8_t & n)
{
    unsigned long t_start =micros();
    uint8_t byte_in;

    while(SerialUSB.available()>0)
    {
        byte_in = SerialUSB.read();
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
            }
        }
    }
    return 0;
}

//Return 1 if got a valid frame
//Block until a frame arrives (or timeout)
bool receive_frame(uint8_t * buf, uint8_t & n)
{
    unsigned long t_start =micros();
    uint8_t byte_in;
    while((micros()-t_start)<FRAMING_TIMEOUT) //data may be sparse, keep polling until first byte arrives, then get whole packet
    {
		if(SerialUSB.available()>0)
		{
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

bool stepTransport(void (*rpc_callback)())
{
  uint8_t np;
  uint16_t nbo;

  if (receive_frame_itr(frame_in, np))
  {
    switch (frame_in[0])
    {
      case RPC_START_NEW_RPC: 
          frame_out[0]=RPC_ACK_NEW_RPC;
          send_frame(frame_out,1);
          num_byte_rpc_out=0;
          num_byte_rpc_in=0;
          byte_in_cnt=0;
          byte_out_cnt=0;
        break;
      case RPC_SEND_BLOCK_MORE: 
          memcpy(rpc_in+byte_in_cnt,frame_in+1,np-1);
          byte_in_cnt=byte_in_cnt+np-1;
          frame_out[0]=RPC_ACK_SEND_BLOCK_MORE;
          send_frame(frame_out,1);
          break;
      case RPC_SEND_BLOCK_LAST: 
          memcpy(rpc_in+byte_in_cnt,frame_in+1,np-1);
          byte_in_cnt=byte_in_cnt+np-1;
          frame_out[0]=RPC_ACK_SEND_BLOCK_LAST;
          send_frame(frame_out,1);
          num_byte_rpc_in=byte_in_cnt;
          (*rpc_callback)(); //Received a request, process it and build reply
          break;
      case RPC_GET_BLOCK:
          nbo=min(RPC_BLOCK_SIZE,num_byte_rpc_out-byte_out_cnt);
          if (byte_out_cnt+nbo==num_byte_rpc_out) //Is last block?
            frame_out[0]=RPC_ACK_GET_BLOCK_LAST;
          else
            frame_out[0]=RPC_ACK_GET_BLOCK_MORE;
          memcpy(frame_out+1,rpc_out+byte_out_cnt,nbo);
          send_frame(frame_out,nbo+1);
          byte_out_cnt=byte_out_cnt+nbo;
          break;
    };
    return true;
  }
  return false;
}


/////////////////////////////////////////////////////////////////////////////////

void setupTransport() {

}



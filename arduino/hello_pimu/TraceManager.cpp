/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "TraceManager.h"
#include "Common.h"

TraceManager trace_manager;


TraceManager::TraceManager()
{
    trace_write_idx=0;
    trace_read_idx=0;
    trace_on=false;
    reading_trace=false;
    n_trace_read=0;
}

void TraceManager::enable_trace()
{
  trace_on=true;
  memset((uint8_t*)(raw_data), 0,N_TRACE_RAW);
  memset((uint8_t*)&debug_msg,0,sizeof(DebugTrace));
  memset((uint8_t*)&print_msg,0,sizeof(PrintTrace));
  trace_write_idx=0;
}

void TraceManager::disable_trace()
{
   trace_on=false;
}

void TraceManager::update_trace_status(Pimu_Status * stat)
{
  if(trace_on && TRACE_TYPE==TRACE_TYPE_STATUS)
  {
    memcpy((uint8_t *)(raw_data+trace_write_idx*sizeof(Pimu_Status)) ,(uint8_t *)(stat) ,sizeof(Pimu_Status));
    trace_write_idx=trace_write_idx+1;
    if(trace_write_idx==N_TRACE_STATUS)
      trace_write_idx=0;
  }
}

void TraceManager::update_trace_debug()
{
  if(trace_on && TRACE_TYPE==TRACE_TYPE_DEBUG)
  {
    memcpy((uint8_t *)(raw_data+trace_write_idx*sizeof(DebugTrace)) ,(uint8_t *)(&debug_msg) ,sizeof(DebugTrace));
    trace_write_idx=trace_write_idx+1;
    if(trace_write_idx==N_TRACE_DEBUG)
      trace_write_idx=0;
  }
}

void TraceManager::update_trace_print()
{
  if(trace_on && TRACE_TYPE==TRACE_TYPE_PRINT)
  {
    print_msg.msg[N_TRACE_PRINT_LN-1]=0;//Ensure a termination to string
    memcpy((uint8_t *)(raw_data+trace_write_idx*sizeof(PrintTrace)) ,(uint8_t *)(&print_msg) ,sizeof(PrintTrace));
    trace_write_idx=trace_write_idx+1;
    if(trace_write_idx==N_TRACE_PRINT)
      trace_write_idx=0;
  }
}

int TraceManager::rpc_read(uint8_t * rpc_out)
{
  int num_byte_rpc_out=0;
      if(!reading_trace)
      {
        //Initialize new read
        reading_trace=true;
        trace_on=false; //force off
        trace_read_idx=trace_write_idx; //Pointing at the oldest in buffer
        
        if(TRACE_TYPE==TRACE_TYPE_STATUS)
          n_trace_read=N_TRACE_STATUS;

        if(TRACE_TYPE==TRACE_TYPE_DEBUG)
          n_trace_read=N_TRACE_DEBUG;

        if(TRACE_TYPE==TRACE_TYPE_PRINT)
          n_trace_read=N_TRACE_PRINT;
      }

      rpc_out[0]=RPC_REPLY_READ_TRACE;
      rpc_out[1]=(uint8_t)(n_trace_read-1>0); //Flag if done with read
      rpc_out[2]=TRACE_TYPE; //Tell what type it is
      n_trace_read--;

      if(TRACE_TYPE==TRACE_TYPE_STATUS)
      {
          memcpy(rpc_out + 3, (uint8_t *)&(raw_data[trace_read_idx*sizeof(Pimu_Status)]), sizeof(Pimu_Status)); //Collect the status data
          num_byte_rpc_out=sizeof(Pimu_Status)+3;
          trace_read_idx++;
          if(trace_read_idx==N_TRACE_STATUS)
              trace_read_idx=0; 
      }
                
      if(TRACE_TYPE==TRACE_TYPE_DEBUG)
      {
        memcpy(rpc_out + 3, (uint8_t *)&(raw_data[trace_read_idx*sizeof(DebugTrace)]), sizeof(DebugTrace)); //Collect the status data
        num_byte_rpc_out=sizeof(DebugTrace)+3;    
        trace_read_idx++;
        if(trace_read_idx==N_TRACE_DEBUG)
            trace_read_idx=0;      
      }
      
    if(TRACE_TYPE==TRACE_TYPE_PRINT)
    {
      memcpy(rpc_out + 3, (uint8_t *)&(raw_data[trace_read_idx*sizeof(PrintTrace)]), sizeof(PrintTrace)); //Collect the status data
        num_byte_rpc_out=sizeof(PrintTrace)+3;    
        trace_read_idx++;
        if(trace_read_idx==N_TRACE_PRINT)
            trace_read_idx=0;      
    }
 
    if(n_trace_read==0)
        reading_trace=false;
  return num_byte_rpc_out;
}

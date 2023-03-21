/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __TRACE_MANAGER_H__
#define  __TRACE_MANAGER_H__

#include "Common.h"


//By default Trace is a Status message
//However developer may wish to send a debug or print message
#define TRACE_TYPE_STATUS 0
#define TRACE_TYPE_DEBUG 1
#define TRACE_TYPE_PRINT 2


#define TRACE_TYPE TRACE_TYPE_STATUS

/* This class supports generating traces of data at the full control rates. 
 * There are three modes:
 * STATUS: store the entire status struct on each control cycle
 * DEBUG: store a more compact data struct on each control cycle
 * PRINT: Store a print message, timestamp, and float aperiodically
 * 
 * The mode is set with TRACE_TYPE
 * The duration of the trace, size of the buffers, etc must be tuned to match the available RAM
 * 
 * Stretch Body tools support enabling / disabling /reading of trace data
 */
 
/////////////////////////// TRACE //////////////////////////////////////
#define N_TRACE_RAW 14000  //Raw buffer. Allocate enough for min 250 Status messages / 1000 debug messages / 250 print messages
#define N_TRACE_STATUS 365 //Status message is 38 bytes ea
#define N_TRACE_DEBUG 1000 //Debug message is 14 bytes ea
#define N_TRACE_PRINT_LN 32
#define N_TRACE_PRINT 280   //Print message is 48 bytes ea

struct __attribute__ ((packed)) DebugTrace{ //14 bytes
  uint8_t u8_1;
  uint8_t u8_2;                 
  float f_1;
  float f_2;
  float f_3;    
};

struct __attribute__ ((packed)) PrintTrace{ //48 bytes
  uint64_t timestamp; //us
  char msg[N_TRACE_PRINT_LN];
  float x;
};



class TraceManager {
   public: 
    TraceManager();
    int rpc_read(uint8_t * rpc_out);
    void enable_trace();
    void disable_trace();
    void update_trace_print();
    void update_trace_debug();
    void update_trace_status(Wacc_Status * stat);

    
    DebugTrace debug_msg;
    PrintTrace print_msg;
    
    bool trace_on;
    
   private:
    uint8_t raw_data[N_TRACE_RAW];
    int trace_write_idx;
    int trace_read_idx;
    bool reading_trace;
    int n_trace_read;
    int idx_trace_print;
    
};

extern TraceManager trace_manager;

#endif

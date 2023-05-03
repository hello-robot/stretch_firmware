/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html    

  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#include <Transport.h>

#include <Arduino.h>
#include "Wacc.h"
#include "Accel.h"
#include "TimeManager.h"
#include "TraceManager.h"


//////////////////////////////////////
Wacc_Config cfg, cfg_in;
Wacc_Command cmd, cmd_in;
Wacc_Status stat, stat_out;
Wacc_Board_Info board_info;
LoadTest load_test;

float accel_LPFa = 0.0; 
float accel_LPFb = 1.0;
float ana_LPFa = 0.0; 
float ana_LPFb = 1.0;
float accel_gravity_scale=1.0;
bool dirty_config=false;
bool dirty_command=false;


/////////////////////////////////////////
void setupTimer4_and_5();
float FS_ACCEL = 50; //Run Acceleromter read Controller at 50hz
void toggle_led(int rate_ms);

void setupWacc() {  
  memset(&cfg, 0, sizeof(Wacc_Config));
  memset(&cfg_in, 0, sizeof(Wacc_Config));
  memset(&cmd, 0, sizeof(Wacc_Command));
  memset(&cmd_in, 0, sizeof(Wacc_Command));
  memset(&stat, 0, sizeof(Wacc_Status));
  memset(&stat_out, 0, sizeof(Wacc_Status));
  sprintf(board_info.board_variant, "Wacc.%d", BOARD_VARIANT);
  memcpy(&(board_info.firmware_version),FIRMWARE_VERSION,min(20,strlen(FIRMWARE_VERSION)));
  setupTimer4_and_5();
  time_manager.clock_zero();
}


uint8_t    BOARD_VARIANT;

void setupBoardVariants()
{
  //Setup board ID. Default is zero for boards prior to Mitski
  pinMode(BOARD_ID_0, INPUT);
  pinMode(BOARD_ID_1, INPUT);
  pinMode(BOARD_ID_2, INPUT);
  pinMode(BOARD_ID_0, INPUT_PULLDOWN);
  pinMode(BOARD_ID_1, INPUT_PULLDOWN);
  pinMode(BOARD_ID_2, INPUT_PULLDOWN);  
  BOARD_VARIANT=(digitalRead(BOARD_ID_2)<<2)|(digitalRead(BOARD_ID_1)<<1)|digitalRead(BOARD_ID_0);

  //Common to all variants
  pinMode(LED, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(ACCEL_INT1, INPUT);
  pinMode(ACCEL_INT2, INPUT);
  digitalWrite(LED, LOW);
  pinMode(D0, INPUT_PULLUP);
  pinMode(D1, INPUT_PULLUP);
  
}

void handleNewRPC()
{
    int ll;
  switch(rpc_in[0])
  {
    case RPC_SET_WACC_COMMAND: 
          memcpy(&cmd_in, rpc_in+1, sizeof(Wacc_Command)); 
          rpc_out[0]=RPC_REPLY_WACC_COMMAND;
          num_byte_rpc_out=1;
          dirty_command=true;
          break;
    case RPC_SET_WACC_CONFIG: 
          memcpy(&cfg_in, rpc_in+1, sizeof(Wacc_Config)); 
          rpc_out[0]=RPC_REPLY_WACC_CONFIG;
          num_byte_rpc_out=1;
          dirty_config=true;
          break;
    case RPC_GET_WACC_STATUS: 
          rpc_out[0]=RPC_REPLY_WACC_STATUS;
          //noInterrupts();
          memcpy(rpc_out + 1, (uint8_t *) (&stat_out), sizeof(Wacc_Status)); //Collect the status data
          //interrupts();
          num_byte_rpc_out=sizeof(Wacc_Status)+1;
          break; 
    case RPC_GET_WACC_BOARD_INFO:
          rpc_out[0]=RPC_REPLY_WACC_BOARD_INFO;
          memcpy(rpc_out + 1, (uint8_t *) (&board_info), sizeof(Wacc_Board_Info)); //Collect the status data
          num_byte_rpc_out=sizeof(Wacc_Board_Info)+1;
          break; 
    case RPC_READ_TRACE: 
          num_byte_rpc_out=trace_manager.rpc_read(rpc_out);
          break;

    case RPC_LOAD_TEST_PUSH: 
          memcpy(&load_test, rpc_in+1, sizeof(LoadTest)); //copy in the command
          rpc_out[0]=RPC_REPLY_LOAD_TEST_PUSH;
          num_byte_rpc_out=1;
          break;
    case RPC_LOAD_TEST_PULL:
          ll=load_test.data[0];
          for(int i=0;i<1023;i++)
            load_test.data[i]=load_test.data[i+1];
          load_test.data[1023]=ll;
          rpc_out[0]=RPC_REPLY_LOAD_TEST_PULL;
          memcpy(rpc_out + 1, (uint8_t *) (&load_test), sizeof(LoadTest));
          num_byte_rpc_out=sizeof(LoadTest)+1;
          break;
   default:
        break;
  };
}


void stepWaccRPC()
{
  toggle_led(500);
  stepTransport(handleNewRPC);
}

////////////////////////Controller///////////////////////////////////////
uint8_t board_reset_cnt=0;
uint8_t ds_cnt=0;

void stepWaccController_1000Hz()
{
    if (dirty_config)
  {
    if (cfg_in.ana_LPF!=cfg.ana_LPF) //Effort filter
    {
      ana_LPFa = exp(cfg_in.ana_LPF*-2*3.14159/FS_CTRL); // z = e^st pole mapping
      ana_LPFb = (1.0-ana_LPFa);
    }
    if (cfg_in.accel_LPF!=cfg.accel_LPF) //Effort filter
    {
      accel_LPFa = exp(cfg_in.accel_LPF*-2*3.14159/FS_CTRL); // z = e^st pole mapping
      accel_LPFb = (1.0-accel_LPFa);
    }
    if (cfg_in.accel_range_g!=cfg.accel_range_g) 
      setAccelRange(cfg_in.accel_range_g);
    
   if ((cfg_in.accel_single_tap_dur!=cfg.accel_single_tap_dur) || (cfg_in.accel_single_tap_thresh!=cfg.accel_single_tap_thresh))
      setSingleTapSensitivity(cfg_in.accel_single_tap_dur,cfg_in.accel_single_tap_thresh );
         
    memcpy(&cfg,&cfg_in,sizeof(Wacc_Config));
    accel_gravity_scale=cfg.accel_gravity_scale;
    dirty_config=false;
  }

if (dirty_command)
  {
    memcpy(&cmd,&cmd_in,sizeof(Wacc_Command));
    dirty_command=false;
    if (cmd.trigger & TRIGGER_BOARD_RESET)
    {
          board_reset_cnt=100;
    }

     if (cmd.trigger & TRIGGER_ENABLE_TRACE)
    {
      trace_manager.enable_trace();
    }
    if (cmd.trigger & TRIGGER_DISABLE_TRACE)
    {
      trace_manager.disable_trace();
    }
  }
  
  if (board_reset_cnt)
  {
    board_reset_cnt--; //Countdown to allow time for RPC to finish up
    if (board_reset_cnt==0)
      NVIC_SystemReset();
  }
  
  if (cmd.d2)
    digitalWrite(D2, HIGH);
  else
    digitalWrite(D2, LOW);

  if (cmd.d3)
    digitalWrite(D3, HIGH);
  else
    digitalWrite(D3, LOW);

  stat.ax=accel_gravity_scale*(stat.ax * accel_LPFa + accel_LPFb*ax);
  stat.ay=accel_gravity_scale*(stat.ay * accel_LPFa + accel_LPFb*ay);
  stat.az=accel_gravity_scale*(stat.az * accel_LPFa + accel_LPFb*az);
  stat.a0 = stat.a0 * ana_LPFa +  ana_LPFb* analogRead(A0);
  stat.d0=digitalRead(D0);
  stat.d1=digitalRead(D1);
  stat.d2=cmd.d2;
  stat.d3=cmd.d3;
  stat.single_tap_count=single_tap_count;
  stat.state=0;
  stat.state = trace_manager.trace_on ?     stat.state | STATE_IS_TRACE_ON: stat.state;
  stat.timestamp= time_manager.current_time_us();
  stat.debug=sizeof(Wacc_Status);//time_manager.ts_base;//(uint32_t)TC4->COUNT16.COUNT.reg;

  noInterrupts();
  memcpy((uint8_t *) (&stat_out),(uint8_t *) (&stat),sizeof(Wacc_Status));
  interrupts();


  if(TRACE_TYPE==TRACE_TYPE_DEBUG)
  {
    //Example of setting trace debug data
    trace_manager.debug_msg.f_3=stat.ax;
    trace_manager.update_trace_debug();
  }

  if(TRACE_TYPE==TRACE_TYPE_PRINT)
  {
  //Example of setting trace print data
   sprintf(trace_manager.print_msg.msg, "AX reading: %d...\n",(int)stat.debug);
   trace_manager.print_msg.x=stat.ax;
   trace_manager.print_msg.timestamp=stat.timestamp;
   trace_manager.update_trace_print();
  }

  if(TRACE_TYPE==TRACE_TYPE_STATUS)
  {
    trace_manager.update_trace_status(&stat_out);
  }
   
   
 
}

void stepWaccController_50Hz()
{
    stepAccel();
}
//////////////////////////////////////////////////////
bool led_on=false;
unsigned long t_toggle_last=0;

void toggle_led(int rate_ms)
{
  unsigned long t = millis();
  if (t-t_toggle_last>rate_ms)
  {
    t_toggle_last=t;
    if (!led_on)
    {
        digitalWrite(LED, HIGH);  //LED
    }
      else
      {
        digitalWrite(LED,LOW);   //LED
      }
     led_on=!led_on;
  }
}

////////////////////// Timer5 /////////////////////////////////////////


void TC5_Handler() {  

  if (TC5->COUNT16.INTFLAG.bit.OVF == 1) 
  {
    stepWaccController_50Hz();
    TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }
}

void TC4_Handler() {                // gets called with FsMg frequency

  if (TC4->COUNT16.INTFLAG.bit.OVF == 1) 
  {    // A counter overflow caused the interrupt
    time_manager.ts_base++;
    TC4->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    stepWaccController_1000Hz();
  }
}

#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);

void enableTCInterrupts() {   //enables the controller interrupt 
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC4
  WAIT_TC16_REGS_SYNC(TC4)                      //wait for sync
}

void disableTCInterrupts() {  //disables the controller interrupt 
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC4)                      // wait for sync
}

void setupTimer4_and_5() {  // configure the controller interrupt
  
  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC4)                      // wait for sync
  

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC4)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC4)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC4)
  
  TC5->COUNT16.CC[0].reg = (int)( round(48000000 / 64 / FS_ACCEL)); //Rate of Accel read
  WAIT_TC16_REGS_SYNC(TC5)
  TC4->COUNT16.CC[0].reg =  TC4_TICKS_PER_CYCLE;  //Rate of control loop
  WAIT_TC16_REGS_SYNC(TC4)
  
  TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0
  TC4->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC4->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC4->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  //Make TC5 low priority as it takes a while and can prevent RPC from running fast
  NVIC_SetPriority(TC4_IRQn, 4);              //TC4 loop priority
  NVIC_SetPriority(TC5_IRQn, 5);              //TC5 accel read priority

  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);
  NVIC_EnableIRQ(TC4_IRQn);
  
  // Enable TC
    enableTCInterrupts();
}



//////////////////////////////////////

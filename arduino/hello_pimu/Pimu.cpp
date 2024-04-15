/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include <Transport.h>

#include <Arduino.h>
#include "Pimu.h"
#include "IMU.h"
#include "Common.h"
#include "BeepManager.h"
#include "TimeManager.h"
#include "AnalogManager.h"
#include "SyncManager.h"
#include "RunstopManager.h"
#include "LightBarManager.h"
#include "TraceManager.h"
#include "IMU_BNO085.h"
#include "ChargerManager.h"


#define V_TO_RAW(v) v*1024/20.0 //per circuit
#define RAW_TO_V(r) (float)r*0.01953125 //20.0/1024.0
#define I_TO_RAW(i) (i*1000)*0.408*1024.0/3300 //per circuit

#define RAW_TO_I(r) (float)r*0.01438686

#define RAW_TO_CHRG_I(r) (float)r*0.01498910

float low_voltage_alert=V_TO_RAW(10.5);
int low_voltage_alert_cnt=0;
float high_current_alert= I_TO_RAW(6.0);
float over_tilt_alert_deg = 10.0;
int startup_cnt=500;


    

bool state_cliff_event=false;
bool state_fan_on=false;
bool state_buzzer_on=false;
bool state_low_voltage_alert=false;
bool state_high_current_alert=false;
bool state_over_tilt_alert=false;

bool state_boot_detected=false;

RunstopManager runstop_manager;
SyncManager sync_manager(&runstop_manager);
LightBarManager light_bar_manager;

ChargerManager charger_manager;

bool state_charger_connected = false;
bool state_charger_is_charging = false;

bool left_tilt_flag = false;
bool right_tilt_flag = false;
bool forward_tilt_flag = false;
uint8_t state_over_tilt_type = 0;



//////////////////////////////////////
Pimu_Config cfg_in, cfg;
Pimu_Trigger trg_in, trg;
Pimu_Status stat, stat_out;
Pimu_Status_Aux stat_aux;
Pimu_Motor_Sync_Reply sync_reply;
Pimu_Board_Info board_info;
IMU_Status imu_stat;
LoadTest load_test;

void setupTimer4_and_5();
void toggle_led(int rate_ms);
uint32_t cycle_cnt=0;

void resetWDT();
void setupWDT(uint8_t period);


/////////////////////////////////////////////////////////////////////////
float deg_to_rad(float x)
{
  return x*0.017453292519943295;
}
float rad_to_deg(float x)
{
  return x*57.29577951308232;
}

#define TC5_TICKS_PER_CYCLE (int)( round(48000000 / 16 / FS)) //30,000 at 100hz, 16:1 prescalar TC5 is 32bit timer
#define US_PER_TC5_CYCLE 1000000/FS //10000 at 100Hz
#define US_PER_TC5_TICK 1000000.0*16/48000000 //0.33us resolution
#define WDT_TIMEOUT_PERIOD 11 //ms range 0-11ms

////////////////////////////////////////////////////////////////////////////////////////////////////////
void handle_trigger();
void update_config();
void update_fan();
void update_imu();
void update_voltage_monitor();
void update_current_monitor();
void update_tilt_monitor();
void update_board_reset();
void update_cliff_monitor();
void update_status();
void toggle_led(int rate_ms);

////////////////////////////////////////////////////////////////////////////////////////////////////////


uint8_t    BOARD_VARIANT;
uint8_t    BOARD_VARIANT_DEDICATED_SYNC;


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

  //BOARD_VARIANT=1;//Temp for testing

  //Common to all variants
  pinMode(RUNSTOP_LED, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(FAN_FET, OUTPUT);
  pinMode(IMU_RESET,OUTPUT);
  pinMode(RUNSTOP_SW, INPUT);
  //pinMode(RUNSTOP_SW, INPUT_PULLUP);
  digitalWrite(RUNSTOP_LED, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(FAN_FET, LOW);
  digitalWrite(IMU_RESET, HIGH);
  

  if (BOARD_VARIANT==0)
  {
    pinMode(RUNSTOP_M0, OUTPUT);
    pinMode(RUNSTOP_M1, OUTPUT);
    pinMode(RUNSTOP_M2, OUTPUT);
    pinMode(RUNSTOP_M3, OUTPUT);
    BOARD_VARIANT_DEDICATED_SYNC=0;
  }
  
  if (BOARD_VARIANT>=1)
  {
    BOARD_VARIANT_DEDICATED_SYNC=1;
    light_bar_manager.setupLightBarManager();
    pinMode(RUNSTOP_OUT, OUTPUT);
    pinMode(SYNC_OUT, OUTPUT);
    pinMode(CHARGER_CONNECTED,INPUT);
    digitalWrite(RUNSTOP_OUT, LOW);
    digitalWrite(SYNC_OUT, LOW);
  }
  if (BOARD_VARIANT>=3)
  {
    pinMode(IMU_BNO085_INT, INPUT_PULLUP);
    pinMode(IMU_BNO085_RESET, OUTPUT);
  }
}

void setupPimu() {  

  memset(&cfg_in, 0, sizeof(Pimu_Config));
  memset(&cfg, 0, sizeof(Pimu_Config));
  cfg.stop_at_runstop=1; //By default acknowledge runstop, user must override via YAML otherwise
  memset(&trg_in, 0, sizeof(Pimu_Trigger));
  memset(&trg, 0, sizeof(Pimu_Trigger));
  memset(&stat, 0, sizeof(Pimu_Status));
  memset(&stat_aux, 0, sizeof(Pimu_Status_Aux));
  sprintf(board_info.board_variant, "Pimu.%d", BOARD_VARIANT);
  memcpy(&(board_info.firmware_version),FIRMWARE_VERSION,min(20,strlen(FIRMWARE_VERSION)));
  analog_manager.setupADC();
  analog_manager.factory_config();
  setupTimer4_and_5();
  setupWDT(WDT_TIMEOUT_PERIOD);
  time_manager.clock_zero();
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
void stepPimuController()
{
  cycle_cnt++;
  toggle_led(500);

  runstop_manager.step(&cfg);
  beep_manager.step();
  analog_manager.step(&stat, &cfg);
  light_bar_manager.step(state_boot_detected, runstop_manager.state_runstop_event, state_charger_is_charging, state_low_voltage_alert, runstop_manager.runstop_led_on, RAW_TO_V(analog_manager.voltage));
  update_fan();  
  update_imu();
  update_board_reset();
  
  startup_cnt=max(0,startup_cnt-1);
  if(startup_cnt==0)
  {
    update_voltage_monitor();
    update_current_monitor();
    update_tilt_monitor();
    update_cliff_monitor();
  }
  
  if (runstop_manager.state_runstop_event)
  {
    runstop_manager.toggle_led(500);
  }
  else
  {
    digitalWrite(RUNSTOP_LED, HIGH);
  }

  update_status();

}
////////////////////////////////////////////////////////////////////////////////////////////////////////

void handleNewRPC()
{
  int ll;
  switch(rpc_in[0])
  {
    case RPC_SET_PIMU_CONFIG: 
          memcpy(&cfg_in, rpc_in+1, sizeof(Pimu_Config)); //copy in the config
          noInterrupts();
          update_config();
          interrupts();
          rpc_out[0]=RPC_REPLY_PIMU_CONFIG;
          num_byte_rpc_out=1;
          break;
    case RPC_SET_PIMU_TRIGGER: 
          memcpy(&trg, rpc_in+1, sizeof(Pimu_Trigger)); //copy in
          noInterrupts();
          handle_trigger();
          interrupts();
          rpc_out[0]=RPC_REPLY_PIMU_TRIGGER;
          memcpy(rpc_out+1,(uint8_t *)&(trg_in.data),sizeof(uint32_t));
          num_byte_rpc_out=1+sizeof(uint32_t);
          break;
    case RPC_GET_PIMU_STATUS: 
          update_status();
          rpc_out[0]=RPC_REPLY_PIMU_STATUS;
          memcpy(rpc_out + 1, (uint8_t *) (&stat_out), sizeof(Pimu_Status)); //Collect the status data
          num_byte_rpc_out=sizeof(Pimu_Status)+1;
          break;

     case RPC_GET_PIMU_STATUS_AUX:
          stat_aux.foo=sync_manager.motor_sync_cnt;
          rpc_out[0]=RPC_REPLY_PIMU_STATUS_AUX;
          memcpy(rpc_out + 1, (uint8_t *) (&stat_aux), sizeof(Pimu_Status_Aux)); //Collect the status_aux data
          num_byte_rpc_out=sizeof(Pimu_Status_Aux)+1;
          break;
     case RPC_GET_PIMU_BOARD_INFO:
          rpc_out[0]=RPC_REPLY_PIMU_BOARD_INFO;
          memcpy(rpc_out + 1, (uint8_t *) (&board_info), sizeof(Pimu_Board_Info)); //Collect the status data
          num_byte_rpc_out=sizeof(Pimu_Board_Info)+1;
          state_boot_detected=true;
          break; 
     case RPC_SET_MOTOR_SYNC:
          noInterrupts();
          sync_manager.trigger_motor_sync();
          rpc_out[0]=RPC_REPLY_MOTOR_SYNC;
          sync_reply.motor_sync_cnt=sync_manager.motor_sync_cnt;
           memcpy(rpc_out + 1, (uint8_t *) (&sync_reply), sizeof(Pimu_Motor_Sync_Reply));
          num_byte_rpc_out=sizeof(Pimu_Motor_Sync_Reply)+1;
          interrupts();

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

void stepPimuRPC()
{
  stepTransport(handleNewRPC);
  resetWDT();
}


////////////////////////////
uint8_t board_reset_cnt=0;
bool first_config = 1;
int fan_on_cnt=0;
bool pulse_on=0;

////////////////////////////
void handle_trigger()
{
    if (trg.data & TRIGGER_ENABLE_TRACE)
    {
      trace_manager.enable_trace();
    }
    if (trg.data & TRIGGER_DISABLE_TRACE)
    {
      trace_manager.disable_trace();
    }
    if (trg.data & TRIGGER_LIGHTBAR_TEST)
    {
      light_bar_manager.start_test();
    }
    if (trg.data & TRIGGER_BEEP)
    {
      beep_manager.do_beep(BEEP_ID_SINGLE_SHORT);
    }
    if (trg.data & TRIGGER_BOARD_RESET)
    {
          board_reset_cnt=100;
    }
    if (trg.data & TRIGGER_RUNSTOP_RESET)
    {
          runstop_manager.deactivate_runstop();
          runstop_manager.step(&cfg);
    }
    if (trg.data & TRIGGER_RUNSTOP_ON)
    {
          runstop_manager.activate_runstop();
          runstop_manager.step(&cfg);
    }
    if (trg.data & TRIGGER_CLIFF_EVENT_RESET)
    {
          state_cliff_event = false;
    }
    if (trg.data & TRIGGER_BUZZER_ON)
    {
          state_buzzer_on=true;
          digitalWrite(BUZZER, HIGH);
    }
    if (trg.data & TRIGGER_BUZZER_OFF)
    {
          state_buzzer_on=false;
          digitalWrite(BUZZER, LOW);
    }
    if (trg.data & TRIGGER_FAN_ON)
    {
          state_fan_on=true;
          digitalWrite(FAN_FET, HIGH);
          fan_on_cnt=1200;
    }
    if (trg.data & TRIGGER_FAN_OFF)
    {
         state_fan_on=false;
          digitalWrite(FAN_FET, LOW);
    }
    if (trg.data & TRIGGER_IMU_RESET)
    {
         if (BOARD_VARIANT >= 3)
         {
          //Setting imu orientation to match other robots (clear orientation = false)
          //setting clear orientation to true clears the system orientation register
          bool clear_orientation_frs = false;
          imu_b.writeSystemOrientation(clear_orientation_frs);
         }
         else
         {
          digitalWrite(IMU_RESET, LOW);
          delay(1);
          digitalWrite(IMU_RESET, HIGH);
         }
    }
}
////////////////////////////

void update_config()
{
    analog_manager.update_config(&cfg_in, &cfg);
    setIMUConfig(&cfg_in,&cfg);

    if (cfg_in.low_voltage_alert!=cfg.low_voltage_alert) 
    {
      low_voltage_alert=V_TO_RAW(cfg_in.low_voltage_alert);
    }
    if (cfg_in.high_current_alert!=cfg.high_current_alert) 
    {
      high_current_alert=I_TO_RAW(cfg_in.high_current_alert);

    }
    if (rad_to_deg(cfg_in.over_tilt_alert)!=over_tilt_alert_deg) 
    {
      over_tilt_alert_deg=rad_to_deg(cfg_in.over_tilt_alert);
    }
    

    memcpy(&cfg,&cfg_in,sizeof(Pimu_Config));
    
}
////////////////////////////

void update_fan()
{
  //Turn off fan automatically after a bit of time
  fan_on_cnt=max(0,fan_on_cnt-1);
  if (!fan_on_cnt && state_fan_on)
  {
    state_fan_on=false;
    digitalWrite(FAN_FET, LOW);
  }
}
////////////////////////////
void update_imu()
{
  stat.timestamp= time_manager.current_time_us();  //Tag timestamp just before reading IMU
  stepIMU(&imu_stat);
  memcpy(&stat.imu,&imu_stat, sizeof(IMU_Status));
}

////////////////////////////
void update_voltage_monitor()
{
  if (BOARD_VARIANT >= 1)
  {
    state_charger_is_charging = charger_manager.step(RAW_TO_V(analog_manager.voltage), RAW_TO_I(analog_manager.current_efuse), RAW_TO_CHRG_I(analog_manager.current_charge) , BOARD_VARIANT);
    state_charger_connected = charger_manager.charger_plugged_in_flag;
    
    
    if(analog_manager.voltage<low_voltage_alert) //dropped below
      {
        state_low_voltage_alert=true;
        if (cfg.stop_at_low_voltage)
          runstop_manager.activate_runstop();
      }
      else
      {
        state_low_voltage_alert=false;
      }
  }

  if (BOARD_VARIANT==0)
  {
    //For Variant 0, do the double beep at low voltage and trigger the runstop
    if(analog_manager.voltage<low_voltage_alert && cfg.stop_at_low_voltage) //dropped below
      {
        state_low_voltage_alert=true;
        runstop_manager.activate_runstop();
        if(low_voltage_alert_cnt==0) //start new beep sequence
        {
          low_voltage_alert_cnt=(int)FS*6;
          beep_manager.do_beep(BEEP_ID_DOUBLE_SHORT);
        }
      }
      else
      {
        state_low_voltage_alert=false;
      }
      low_voltage_alert_cnt=max(0,low_voltage_alert_cnt-1);
  }
}
////////////////////////////

void update_current_monitor()
{
  if(analog_manager.current>high_current_alert && cfg.stop_at_high_current) //dropped below
    {
      state_high_current_alert=true;
      runstop_manager.activate_runstop();
    }
    else
    {
      state_high_current_alert=false;
    }
}
////////////////////////////

void update_tilt_monitor()
{
  float left_tilt;
  float right_tilt;
  float forward_tilt;

  left_tilt = analog_manager.cliff[3] - analog_manager.cliff[1];
  right_tilt = analog_manager.cliff[2] - analog_manager.cliff[0];
  forward_tilt = ((analog_manager.cliff[0] + analog_manager.cliff[1])/2) - ((analog_manager.cliff[2] + analog_manager.cliff[3])/2);

  if (isIMUOrientationValid() && cfg.stop_at_tilt > 0)
  {
    ////////////// Left Tilting Detection ////////////////////
    if (left_tilt > 300 && stat.imu.ax > 1 && stat.imu.ax < 2.5 && left_tilt_flag == false)
    {
      left_tilt_flag = true;
    }
    if (left_tilt < 50 && stat.imu.ax < 1 && left_tilt_flag == true)
    {
      left_tilt_flag = false;
    }

    /////////////////// Right Tilting Detection ////////////////////
    if (right_tilt > 300 && stat.imu.ax > -2.5 && stat.imu.ax < -0.75 && right_tilt_flag == false)
    {
      right_tilt_flag = true;
    }
    if (right_tilt < 50 && stat.imu.ax > -1 && right_tilt_flag == true)
    {
      right_tilt_flag = false;
    }

    //////// Forward Tilting Detection //////////////////////////////////
    if (forward_tilt > 300 && stat.imu.ay > 1 && stat.imu.ay < 2.5 && forward_tilt_flag == false)
    {
      forward_tilt_flag = true;
    }
    if (forward_tilt < 50 && stat.imu.ay < 1 && forward_tilt_flag == true)
    {
      forward_tilt_flag = false;
    }

    state_over_tilt_alert=(left_tilt_flag || right_tilt_flag || forward_tilt_flag);
    if (state_over_tilt_alert && cfg.stop_at_tilt == 1)
    {
      runstop_manager.activate_runstop();
    }
    state_over_tilt_type = (left_tilt_flag ? 0x01 : 0 | right_tilt_flag ? 0x02 : 0 | forward_tilt_flag ? 0x03 :0);
  }
  
}
////////////////////////////

void update_board_reset()
{
  if (board_reset_cnt)
    {
      board_reset_cnt--; //Countdown to allow time for RPC to finish up
      if (board_reset_cnt==0)
        NVIC_SystemReset();
    }
}
////////////////////////////

void update_cliff_monitor()
{
  uint8_t cliff_last = state_cliff_event;
  state_cliff_event = state_cliff_event || (analog_manager.at_cliff[0] ||analog_manager.at_cliff[1] ||analog_manager.at_cliff[2] ||analog_manager.at_cliff[3])&& cfg.stop_at_cliff; //Remains true until reset
  if (!cliff_last && state_cliff_event)
    beep_manager.do_beep(BEEP_ID_SINGLE_SHORT);
  if(state_cliff_event)
    runstop_manager.activate_runstop();
}

////////////////////////////

void update_status()
{

  //stat.debug=imu_b.irq_cnt;

  if(stat.imu.bump>cfg.bump_thresh) //Use the FW tap detector
      stat.bump_event_cnt++;
  stat.voltage=analog_manager.voltage;
  if (BOARD_VARIANT>=3) //with Variant 3 use E-fuse to monitor current.
  {
    stat.current=analog_manager.current_efuse;
    stat.current_charge=analog_manager.current_charge;
  }
  else
  {
    stat.current=analog_manager.current;
    stat.current_charge=0; //Not available
  }
  stat.temp=analog_manager.temp;

  stat.state=0;
  stat.state = analog_manager.at_cliff[0] ? stat.state|STATE_AT_CLIFF_0 : stat.state;
  stat.state = analog_manager.at_cliff[1] ? stat.state|STATE_AT_CLIFF_1 : stat.state;
  stat.state = analog_manager.at_cliff[2] ? stat.state|STATE_AT_CLIFF_2 : stat.state;
  stat.state = analog_manager.at_cliff[3] ? stat.state|STATE_AT_CLIFF_3 : stat.state;
  stat.state= state_cliff_event ? stat.state|STATE_CLIFF_EVENT : stat.state;
  stat.state= runstop_manager.state_runstop_event? stat.state|STATE_RUNSTOP_EVENT : stat.state;
  stat.state= state_fan_on ? stat.state|STATE_FAN_ON : stat.state;
  stat.state= state_buzzer_on ? stat.state|STATE_BUZZER_ON : stat.state;
  stat.state= state_low_voltage_alert ? stat.state|STATE_LOW_VOLTAGE_ALERT : stat.state;
  stat.state= state_high_current_alert ? stat.state|STATE_HIGH_CURRENT_ALERT : stat.state;
  stat.state= state_charger_connected ? stat.state|STATE_CHARGER_CONNECTED : stat.state;
  stat.state= state_boot_detected ? stat.state|STATE_BOOT_DETECTED : stat.state;
  stat.state= state_over_tilt_alert ? stat.state|STATE_OVER_TILT_ALERT : stat.state;
  stat.state = trace_manager.trace_on ?     stat.state | STATE_IS_TRACE_ON: stat.state;
  stat.state= state_charger_is_charging ? stat.state|STATE_IS_CHARGER_CHARGING : stat.state;

  stat.over_tilt_type = state_over_tilt_type;


  memcpy((uint8_t *) (&stat_out),(uint8_t *) (&stat),sizeof(Pimu_Status));

  if(TRACE_TYPE==TRACE_TYPE_DEBUG)
  {
    //Example of setting trace debug data
    trace_manager.debug_msg.f_3=stat.voltage;
    trace_manager.update_trace_debug();
  }

  if(TRACE_TYPE==TRACE_TYPE_PRINT)
  {
  //Example of setting trace print data
   sprintf(trace_manager.print_msg.msg, "Voltage: %d\n",(int)stat.voltage);
   trace_manager.print_msg.x=stat.voltage;
   trace_manager.print_msg.timestamp=stat.timestamp;
   trace_manager.update_trace_print();
  }

  if(TRACE_TYPE==TRACE_TYPE_STATUS)
  {
    trace_manager.update_trace_status(&stat_out);
  }
}

/////////////////////// Watchdog //////////////////////////////////////
// reference https://forum.arduino.cc/t/wdt-watchdog-timer-code/353610

void WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

void resetWDT() {
  // reset the WDT watchdog timer.
  // this must be called before the WDT resets the system
  WDT->CLEAR.reg= 0xA5; // reset the WDT
  WDTsync(); 
}

void systemReset() {
  // use the WDT watchdog timer to force a system reset.
  // WDT MUST be running for this to work
  WDT->CLEAR.reg= 0x00; // system reset via WDT
  WDTsync(); 
}

void setupWDT( uint8_t period) {
  // initialize the WDT watchdog timer

  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required

  WDT->CONFIG.reg = min(period,11); // see Table 17-5 Timeout Period (valid values 0-11) (ms)

  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync(); 
}

////////////////////// Timer5 /////////////////////////////////////////


void TC5_Handler() {  
  

  if (TC5->COUNT16.INTFLAG.bit.OVF == 1) 
  {
    stepPimuController();
    TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }
}

#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);
void enableTCInterrupts() {   //enables the controller interrupt ("closed loop mode")
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC4
  WAIT_TC16_REGS_SYNC(TC4)                      //wait for sync
}

void disableTCInterrupts() {  //disables the controller interrupt ("closed loop mode")
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC4)                      // wait for sync
}

void setupTimer4_and_5() {  // configure the controller interrupt
  
  // Enable GCLK for TC5 (timer counter input clock)
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

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC4)
  
  TC5->COUNT16.CC[0].reg = TC5_TICKS_PER_CYCLE; //Value to count up to
  WAIT_TC16_REGS_SYNC(TC5)
  TC4->COUNT16.CC[0].reg =  TC4_TICKS_PER_CYCLE; //(int)( round(48000000 / FsCtrl / 2)); //0x3E72; //0x4AF0;
  WAIT_TC16_REGS_SYNC(TC4)

  TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0
  TC4->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC4->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC4->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0
  

  NVIC_SetPriority(TC5_IRQn, 2);              //Set interrupt priority
  NVIC_SetPriority(TC4_IRQn, 1);              //TC4 pulse generator highest priority so timing is correct

  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);
  NVIC_EnableIRQ(TC4_IRQn);

  // Enable TC
    enableTCInterrupts();
}

////////////////////// Timer4 /////////////////////////////////////////

void TC4_Handler() {                // gets called with FsMg frequency

  if (TC4->COUNT16.INTFLAG.bit.OVF == 1) {    // A counter overflow caused the interrupt
    time_manager.ts_base++;
    sync_manager.step();
  TC4->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }
}


///////////////////// LED /////////////////////////////////////////


bool led_on=false;
unsigned long t_toggle_last=0;


void toggle_led(int rate_ms)
{
  unsigned long t = time_manager.get_elapsed_time_ms();
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

/*
/*
  -------------------------------------------------------------
  Hello Robot - Hello Stepper

  This code is derived from the Mechaduino project. 
  https://github.com/jcchurch13/Mechaduino-Firmware
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#include "HelloController.h"
#include "State.h"
#include "Utils.h"
#include "Parameters.h"
#include "VelocityGenerator.h"
#include "MotionGenerator.h"
#include "analogFastWrite.h"
#include "Transport.h"
#include <SPI.h>
#include "analogFastWrite.h"
#include <FlashStorage.h>
#include "Controller.h"

#include "SyncManager.h"
#include "TimeManager.h"
#include "TrajectoryManager.h"
#include "TraceManager.h"

#include "AnalogManager.h"

void write_to_lookup(uint8_t page_id, float * data);


Command cmd,cmd_in;
CommandReply cmd_reply;
Gains gains, gains_in;
Trigger trg, trg_in;
Status stat,stat_out;
StatusAux stat_aux;
EncCalib enc_calib_in;
MotionLimits motion_limits;
TrajectorySegment traj_seg_in;
TrajectorySegmentReply traj_seg_reply;

Stepper_Board_Info board_info;
FlashStorage(flash_gains, Gains);
FlashStorage(flash_stepper_type, uint8_t);



LoadTest load_test;
volatile int dirty_cmd=false;
volatile int sync_irq_cnt_last=0;
volatile bool dirty_gains=false;
volatile bool dirty_trigger=false;
volatile bool dirty_traj_seg=false;
volatile bool dirty_trigger_mark_on_contact=false;

volatile bool diag_pos_calibrated = 0;
volatile bool diag_runstop_on=0;
volatile bool diag_near_pos_setpoint=0;
volatile bool diag_near_vel_setpoint=0;
volatile bool diag_is_moving=0;
volatile bool diag_at_current_limit=0;
volatile bool diag_is_mg_accelerating=0;
volatile bool diag_is_mg_moving=0;
volatile bool diag_calibration_rcvd=0;
volatile bool diag_waiting_on_sync=0;


int switch_to_menu_cnt=0;
int board_reset_cnt=0;
int guarded_event_cnt=0;
int guarded_eff_cnt=0;
#define GUARDED_EFF_THRESH 50 // (ms) Must be over effort for this long to trigger a contact event
bool motion_limits_set=0;
volatile int cmd_cnt_exec=0;
volatile int cmd_cnt_rpc=0;
volatile int cmd_rpc_overflow=0;

#define N_POS_HISTORY 250 //With TC4_LOOP_RATE 1000, log 0.25s of past motions
float pos_history[N_POS_HISTORY];
int pos_history_idx=0;


//By default boot with hello_interface on
//Turn on when get RPC request RPC_SET_MENU_ON
//Turn back off when menu gets 'z' command
volatile bool hello_interface = 1;
float mark_pos=0.0;
float mark_rem=0.0;
float mark_on_contact_pos=0.0;
float ywc=0;
float uMAX_P=0;
float uMAX_N=0;
float uMAX_PF=0;
float uMAX_NF=0;
int cntr=0;
volatile uint16_t ctrl_cycle_cnt=0;

//Filter params    
volatile float efLPFa = 0.0; 
volatile float efLPFb = 1.0;
volatile float efLPF = 0;
volatile float vsLPF = 0.0;  
volatile float vsLPFa = 0.0; 
volatile float vsLPFb = 1.0;

  
bool first_filter=true;

bool led_on=false;
unsigned long t_toggle_last=0;
float hold_pos=0;
float traj_hold_pos=0;

void update_status();

VelocityGenerator vg;
MotionGenerator mg;



float vs=0;
float eff=0;
float g_eff_pos=0;
float g_eff_neg=0;
double ywd=0;
float PAY = 0;

float FsCtrl = TC4_LOOP_RATE;
bool receiving_calibration=false;
bool flip_encoder_polarity = false;
bool flip_effort_polarity = false;


bool runstop_enabled=false;
bool guarded_mode_enabled = false;

bool safety_override = false;
bool guarded_override=false;
int first_step_safety=10; //count down
uint8_t board_id=0;

int vel_watchdog=TC4_LOOP_RATE; //Watchdog counts down from 1s
bool motor_enabled_flag = true;


///////////////////////// UTIL ///////////////////////////


float deg_to_rad(float x)
{
  return x*0.017453292519943295;
}
float rad_to_deg(float x)
{
  return x*57.29577951308232;
}

float k_c2e; 
float k_e2c; 

float current_to_effort(float x)
{
  return max(-255,min(255,x*k_c2e));
}

#define NOMINAL_BUS_VOLTAGE 12.5 //V of battery fully charged, no charger attached, nominal load
#define RAW_TO_VOLTAGE 20.0/1024 //10bit adc, 0-20V per 0-3.3V reading
float voltage_calibrated=0;
float get_voltage_calibrated(float raw)
{
 return (raw*RAW_TO_VOLTAGE)-0.3; //0.3 is needed to account for leakage current of TVS
}

//This returns an effort value (-255 to 255) that is normalized by the bus voltage
//such that as the bus voltage drops, the estimate of effort "force" applied to the outside world
//goes down for a given current
//Only works for VARIANT_3 and later boards

float current_to_effort_v_norm(float x)
{
    float e=current_to_effort(x);
   if (BOARD_VARIANT>=3 && voltage_calibrated!=0)
   {
    return max(-255,min(255,e*voltage_calibrated/NOMINAL_BUS_VOLTAGE));
   }
   else
    return e;
}

float effort_to_current(float e)
{
  return (e*k_e2c);

}

void toggle_led(int rate_ms)
{
  unsigned long t = time_manager.get_elapsed_time_ms();
  if (t-t_toggle_last>rate_ms)
  {
    t_toggle_last=t;
    if (!led_on)
          digitalWrite(ledPin, HIGH);  //LED
      else
          digitalWrite(ledPin,LOW);   //LED
     led_on=!led_on;
  }
}



uint8_t    BOARD_VARIANT;
uint8_t    BOARD_VARIANT_DRV8842;
uint8_t    BOARD_VARIANT_PIN_RUNSTOP;


void setupBoardVariants()
{
  //Setup board ID. Default is zero for boards prior to Mitski
  pinMode(BOARD_ID_0, INPUT);
  pinMode(BOARD_ID_1, INPUT);
  pinMode(BOARD_ID_2, INPUT);
  pinMode(BOARD_ID_0, INPUT_PULLDOWN);
  pinMode(BOARD_ID_1, INPUT_PULLDOWN);
  pinMode(BOARD_ID_2, INPUT_PULLDOWN);  
  BOARD_VARIANT_DRV8842=0;
  BOARD_VARIANT_PIN_RUNSTOP=PIN_RS0;
  BOARD_VARIANT=(digitalRead(BOARD_ID_2)<<2)|(digitalRead(BOARD_ID_1)<<1)|digitalRead(BOARD_ID_0);

  if (BOARD_VARIANT==0)
  {
    //The board uses two  A4950ELJTR motor drivers. These are capable of 3.5A peak currents.
    // An iMax of 3.2A results in a uMax of 247. By default set at 8-bit pwm. 
    iMAX =3.2;        // Be careful adjusting this.  
    rSense = 0.1;   //Ohms per Franco board
    uMAX = (255/3.3)*(iMAX*10*rSense);
    k_c2e =(255/3.3)*10*rSense;
  }
  
  if (BOARD_VARIANT>=1)
  {
    //The board uses two DRV8842 motor drivers. These are capable of 5.0A peak currents / 3.5A RMS
    // An iMax of 4.35A results in a uMax of 252. By default set at 8-bit pwm. 
    iMAX =4.35;        // Be careful adjusting this.  
    rSense = 0.15;   //Ohms per Franco board
    uMAX = (255/3.3)*(iMAX*5*rSense);   
    k_c2e =(255/3.3)*5*rSense;
    
    BOARD_VARIANT_DRV8842=1;
    BOARD_VARIANT_PIN_RUNSTOP=PIN_RS1;
    pinMode(PIN_SYNC, INPUT);
    pinMode(MOTOR_SHUNT, OUTPUT);
    pinMode(DRV8842_NSLEEP_A, OUTPUT);
    pinMode(DRV8842_NSLEEP_B, OUTPUT);
    pinMode(DRV8842_FAULT_A, INPUT);
    pinMode(DRV8842_FAULT_B, INPUT);

    //attachInterrupt(digitalPinToInterrupt(BOARD_VARIANT_PIN_RUNSTOP), sync_manager.on_runstop_change, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(PIN_SYNC), sync_manager.on_sync_change, CHANGE);
  }
  if (BOARD_VARIANT>=3)
  {
    
    analog_manager.setupADC();//Set up analog read for S3 stepper boards
  }

  if (BOARD_VARIANT >= 4)
  {
    pinMode(DRV8842_DECAY, OUTPUT);
  }

  k_e2c = 1/k_c2e;
  pinMode(BOARD_VARIANT_PIN_RUNSTOP, INPUT);
  pinMode(BOARD_VARIANT_PIN_RUNSTOP, INPUT_PULLUP); //Default to high if no cable (disables motor)
}


void setupHelloController()
{
  memset(&cmd, 0, sizeof(Command));
  memset(&cmd_in, 0, sizeof(Command));
  memset(&gains, 0, sizeof(Gains));
  memset(&gains_in, 0, sizeof(Gains));
  memset(&trg, 0, sizeof(Trigger));
  memset(&trg_in, 0, sizeof(Trigger));
  memset(&stat, 0, sizeof(Status));
  memset(&stat_out, 0, sizeof(Status));
  memset(&stat_aux, 0, sizeof(StatusAux));
  memset(&motion_limits, 0, sizeof(MotionLimits));
  memset(&pos_history, 0, N_POS_HISTORY*sizeof(float));

  sprintf(board_info.board_variant, "Stepper.%d", BOARD_VARIANT);
  memcpy(&(board_info.firmware_version_hr),FIRMWARE_VERSION_HR,min(20,strlen(FIRMWARE_VERSION_HR)));

  //Reading stepper_type from flash memory
  uint8_t flash_st;
  flash_st = flash_stepper_type.read();
  memcpy(&(board_info.stepper_type),&flash_st,sizeof(board_info.stepper_type));

  sync_manager.setupSyncManager();
  time_manager.setupTimeManager();
   
  Gains fg;
  fg=flash_gains.read();
  memcpy(&gains_in, &fg, sizeof(Gains));
  dirty_gains=1; //force load of gains
  analogFastWrite(VREF_2, 0);     //set phase currents to zero
  analogFastWrite(VREF_1, 0);

  
}

void enableMotorDrivers()
{
  if (BOARD_VARIANT>=1)
  {
    digitalWrite(MOTOR_SHUNT, HIGH); //Turn the shunt off (for lift dof)
    digitalWrite(DRV8842_NSLEEP_A, HIGH); //Logic high enables driver
    digitalWrite(DRV8842_NSLEEP_B, HIGH); //Logic high enables driver
    motor_enabled_flag = true;
  }

}
void disableMotorDrivers()
{
  if (BOARD_VARIANT>=1)
  {
    digitalWrite(DRV8842_NSLEEP_A, LOW); //Logic high enables driver
    digitalWrite(DRV8842_NSLEEP_B, LOW); //Logic high enables driver
    digitalWrite(MOTOR_SHUNT, LOW); //Turn the shunt off (for lift dof)
    motor_enabled_flag = false;
  }
}

//Needed for lift brake circuit to run correctly 
void enableMotorDecay()
{
  if (BOARD_VARIANT >= 4)
  {
    digitalWrite(DRV8842_DECAY, HIGH);
  }
}
///////////////////////// RPC ///////////////////////////

void handleNewRPC();

void stepHelloControllerRPC()
{
  stepTransport(handleNewRPC);
  //Count down RPC cycles and switch to menu mode
  if (switch_to_menu_cnt)
  {
    switch_to_menu_cnt=switch_to_menu_cnt-1;
    if (switch_to_menu_cnt==0)
    {
      hello_interface=0;
      enableMotorDrivers();
    }
  }
}



void handleNewRPC()
{
  int ll,idx;

  switch(rpc_in[0])
  {
    case RPC_GET_STEPPER_BOARD_INFO:
          rpc_out[0]=RPC_REPLY_STEPPER_BOARD_INFO;
          memcpy(rpc_out + 1, (uint8_t *) (&board_info), sizeof(Stepper_Board_Info)); //Collect the status data
          num_byte_rpc_out=sizeof(Stepper_Board_Info)+1;
          break; 
    case RPC_SET_COMMAND:
          noInterrupts();
          memcpy(&cmd_in, rpc_in+1, sizeof(Command)); //copy in the command
          cmd_cnt_rpc++;
          cmd_reply.ctrl_cycle_cnt=ctrl_cycle_cnt;
          if(dirty_cmd)
            cmd_rpc_overflow++;
          dirty_cmd=1;
          rpc_out[0]=RPC_REPLY_COMMAND;
          memcpy(rpc_out + 1, (uint8_t *) (&cmd_reply), sizeof(CommandReply)); //Collect the status data
          num_byte_rpc_out=sizeof(CommandReply)+1;
    interrupts();
          break;
    case RPC_SET_GAINS: 
          memcpy(&gains_in, rpc_in+1, sizeof(Gains)); //copy in the command
          dirty_gains=1;
          rpc_out[0]=RPC_REPLY_GAINS;
          num_byte_rpc_out=1;
          break;
    case RPC_SET_MOTION_LIMITS: 
          memcpy(&motion_limits, rpc_in+1, sizeof(MotionLimits)); 
          rpc_out[0]=RPC_REPLY_MOTION_LIMITS;
          num_byte_rpc_out=1;
          motion_limits_set=1;
          break;
    case RPC_READ_GAINS_FROM_FLASH: 
          rpc_out[0]=RPC_REPLY_READ_GAINS_FROM_FLASH;
          Gains fg;
          fg=flash_gains.read();
          memcpy(rpc_out + 1, (uint8_t *) (&fg), sizeof(Gains)); //Collect the status data
          num_byte_rpc_out=sizeof(Gains)+1;
          break; 
    case RPC_READ_TRACE: 
          num_byte_rpc_out=trace_manager.rpc_read(rpc_out);
          break; 

    case RPC_SET_MENU_ON:
          rpc_out[0]=RPC_REPLY_MENU_ON;
          num_byte_rpc_out=1;
          switch_to_menu_cnt=5; //allow 5 rpc cycles to pass before switch to menu mode, allows any RPC replies to go out
          disableWDT();
          // enableMotorDrivers();
          break; 
    case RPC_SET_TRIGGER: 
          memcpy(&trg_in, rpc_in+1, sizeof(Trigger)); //copy in the config
          dirty_trigger=1;
          rpc_out[0]=RPC_REPLY_SET_TRIGGER;
          num_byte_rpc_out=1;
          if (trg_in.data & TRIGGER_WRITE_GAINS_TO_FLASH)
           flash_gains.write(gains);
          break;
    case RPC_SET_ENC_CALIB: 
          //if (!receiving_calibration) //Shunt motor at start
          //  disableMotorDrivers();
          receiving_calibration=true;
          memcpy(&enc_calib_in, rpc_in+1, sizeof(EncCalib)); //copy in the calibration table
          rpc_out[0]=RPC_REPLY_ENC_CALIB;
          num_byte_rpc_out=1;
          write_to_lookup(enc_calib_in.page_id, enc_calib_in.page);
          if (enc_calib_in.page_id==255)//done
          {
            board_reset_cnt=100;
          }
          break;
    case RPC_GET_STATUS:
        //Make sure that at least one ctrl cycle has executed since the last SET_CMD
        if (dirty_cmd && ctrl_cycle_cnt==cmd_reply.ctrl_cycle_cnt)
        {
            for(int i=0;i<10000;i++)
            {
                //delayMicroseconds(10);
                    if(ctrl_cycle_cnt!=cmd_reply.ctrl_cycle_cnt)
                        break;
            }
        }

        noInterrupts();
        //stat_out.debug=ctrl_cycle_cnt;
          //update_status();
          rpc_out[0]=RPC_REPLY_STATUS;
          memcpy(rpc_out + 1, (uint8_t *) (&stat_out), sizeof(Status)); //Collect the status data
          num_byte_rpc_out=sizeof(Status)+1;
        interrupts();
          break;
    case RPC_GET_STATUS_AUX:
       noInterrupts();
          rpc_out[0]=RPC_REPLY_STATUS_AUX;
          stat_aux.cmd_cnt_rpc=cmd_cnt_rpc;
          stat_aux.cmd_cnt_exec=cmd_cnt_exec;
          stat_aux.cmd_rpc_overflow=cmd_rpc_overflow;
          stat_aux.sync_irq_cnt=sync_manager.sync_irq_cnt;
          stat_aux.sync_irq_overflow=sync_manager.sync_irq_overflow;
          memcpy(rpc_out + 1, (uint8_t *) (&stat_aux), sizeof(StatusAux)); //Collect the status data
          num_byte_rpc_out=sizeof(StatusAux)+1;
          interrupts();
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
    case RPC_SET_NEXT_TRAJECTORY_SEG:
    noInterrupts();
          memcpy(&traj_seg_in, rpc_in+1, sizeof(TrajectorySegment)); //copy in the new segment
          traj_seg_reply.success=trajectory_manager.set_next_trajectory_segment(&traj_seg_in, motion_limits_set, diag_pos_calibrated, &motion_limits, &cmd_in);
          rpc_out[0]=RPC_REPLY_SET_NEXT_TRAJECTORY_SEG;
          memcpy(rpc_out + 1, (uint8_t *) (&traj_seg_reply), sizeof(TrajectorySegmentReply));
          num_byte_rpc_out=sizeof(TrajectorySegmentReply)+1;
     interrupts();
          break;
    case RPC_START_NEW_TRAJECTORY:
    noInterrupts();
          memcpy(&traj_seg_in, rpc_in+1, sizeof(TrajectorySegment)); //copy in the new segment
          traj_seg_reply.success=trajectory_manager.start_new_trajectory(&traj_seg_in, sync_manager.sync_mode_enabled, motion_limits_set, diag_pos_calibrated, &motion_limits, &cmd_in);
          rpc_out[0]=RPC_REPLY_START_NEW_TRAJECTORY;
          memcpy(rpc_out + 1, (uint8_t *) (&traj_seg_reply), sizeof(TrajectorySegmentReply)); 
          num_byte_rpc_out=sizeof(TrajectorySegmentReply)+1;
     interrupts();
          //stat.debug=trajectory_manager.dirty_seg_in;
          break;
    case RPC_RESET_TRAJECTORY: 
          rpc_out[0]=RPC_REPLY_RESET_TRAJECTORY;
          num_byte_rpc_out=1;
          traj_hold_pos=yw;
          trajectory_manager.reset();
          break;

    //RPC command to set stepper_type in flash mem
    case RPC_SET_STEPPER_TYPE: 
          memcpy(&(board_info.stepper_type),rpc_in+1,sizeof(board_info.stepper_type));
          flash_stepper_type.write(board_info.stepper_type);
          rpc_out[0]=RPC_REPLY_SET_STEPPER_TYPE;
          num_byte_rpc_out=1;
          break;

    //RPC command to read stepper_type from flash mem
    case RPC_READ_STEPPER_TYPE_FROM_FLASH:
          rpc_out[0]=RPC_REPLY_READ_STEPPER_TYPE_FROM_FLASH;
          uint8_t flash_st;
          flash_st=flash_stepper_type.read();
          memcpy(rpc_out + 1, (uint8_t *) (&flash_st), sizeof(board_info.stepper_type)); //Collect the status data
          num_byte_rpc_out=sizeof(board_info.stepper_type)+1;
          break; 
   default:
        break;
  };
}

///////////////////////// Status ///////////////////////////

void update_status()
{

//stat.debug=cmd_rpc_overflow;
  //noInterrupts();
  //stat.timestamp=time_manager.get_encoder_timestamp();
  stat.effort= eff;
  stat.pos=deg_to_rad(ywd);
  stat.vel=deg_to_rad(vs);
  stat.err=deg_to_rad(e);               //controller error (inner loop)
  stat.mode=cmd.mode; 
  stat.guarded_event = guarded_event_cnt;
  if (BOARD_VARIANT >= 3)
  {
    stat.voltage=analog_manager.voltage;
    voltage_calibrated=get_voltage_calibrated(stat.voltage);
  }
  else
  {
    stat.voltage = 0;
  }
  
  stat.diag=0;
  stat.diag= diag_pos_calibrated ?      stat.diag|DIAG_POS_CALIBRATED : stat.diag;
  stat.diag= diag_runstop_on ?          stat.diag|DIAG_RUNSTOP_ON : stat.diag;
  stat.diag= diag_near_pos_setpoint ?   stat.diag|DIAG_NEAR_POS_SETPOINT : stat.diag;
  stat.diag= diag_near_vel_setpoint ?   stat.diag|DIAG_NEAR_VEL_SETPOINT : stat.diag;
  stat.diag= diag_is_moving ?           stat.diag|DIAG_IS_MOVING : stat.diag;
  stat.diag= diag_at_current_limit ?    stat.diag|DIAG_AT_CURRENT_LIMIT : stat.diag;
  stat.diag= diag_is_mg_accelerating ?  stat.diag|DIAG_IS_MG_ACCELERATING : stat.diag;
  stat.diag= diag_is_mg_moving ?        stat.diag|DIAG_IS_MG_MOVING : stat.diag;
  stat.diag= diag_calibration_rcvd ?    stat.diag|DIAG_CALIBRATION_RCVD : stat.diag;
  stat.diag= guarded_override ?         stat.diag|DIAG_IN_GUARDED_EVENT : stat.diag;
  stat.diag = safety_override?          stat.diag| DIAG_IN_SAFETY_EVENT: stat.diag;
  stat.diag = diag_waiting_on_sync?     stat.diag| DIAG_WAITING_ON_SYNC: stat.diag;
  stat.diag = sync_manager.sync_mode_enabled?     stat.diag| DIAG_IN_SYNC_MODE: stat.diag;
  stat.diag = trajectory_manager.is_trajectory_active()? stat.diag| DIAG_TRAJ_ACTIVE: stat.diag;
  stat.diag = trajectory_manager.is_trajectory_waiting_on_sync()? stat.diag| DIAG_TRAJ_WAITING_ON_SYNC: stat.diag;
  stat.diag = trace_manager.trace_on ?     stat.diag| DIAG_IS_TRACE_ON: stat.diag;
 
  stat.traj_setpoint=trajectory_manager.q;
  stat.traj_id=trajectory_manager.get_id_current_segment();


  //stat.debug = sync_manager.runstop_trigger_cnt;
  noInterrupts();
  memcpy((uint8_t *) (&stat_out),(uint8_t *) (&stat),sizeof(Status));
  interrupts();

if (trace_manager.trace_on)
{
   if(TRACE_TYPE==TRACE_TYPE_DEBUG)
  {
    //Example of setting trace debug data
    trace_manager.debug_msg.f_3=stat.pos;
    trace_manager.update_trace_debug();
  }

  if(TRACE_TYPE==TRACE_TYPE_PRINT)
  {
  //Example of setting trace print data
   //sprintf(trace_manager.print_msg.msg, "Pos: %d\n",(int)stat.pos);
   //trace_manager.print_msg.x=stat.pos;
   //trace_manager.print_msg.timestamp=stat.timestamp;
   //trace_manager.update_trace_print();
  }

  if(TRACE_TYPE==TRACE_TYPE_STATUS)
  {
    trace_manager.update_trace_status(&stat_out);
  }
}

}




///////////////////////// Controller Loop  ///////////////////////////
//Called every control cycle waypoint TC4 interrupt

float mpos_d;
float x_des_incr=0;
#define STIFFNESS_SLEW .1
float stiffness_target=0;

float eff_max=0;
float xdes=0;


//Example of setting trace data
void update_trace()
{

  //stat.debug=mg.dt;
   if(TRACE_TYPE==TRACE_TYPE_DEBUG)
  {
    trace_manager.debug_msg.f_1= xdes;
    trace_manager.debug_msg.f_2=mg.t;
    trace_manager.debug_msg.f_3=stat.pos;
    trace_manager.debug_msg.u8_1=trace_manager.trace_write_idx;
    trace_manager.debug_msg.u8_2=trace_manager.trace_on;
    trace_manager.update_trace_debug();
  }

  if(TRACE_TYPE==TRACE_TYPE_PRINT)
  {
   sprintf(trace_manager.print_msg.msg, "Pos: %d\n",(int)stat.pos);
   trace_manager.print_msg.x=stat.pos;
   trace_manager.print_msg.timestamp=stat.timestamp;
   trace_manager.update_trace_print();
  }

  if(TRACE_TYPE==TRACE_TYPE_STATUS)
  {
    trace_manager.update_trace_status(&stat_out);
  }
}


void stepHelloController()
{

  
 
  //noInterrupts();
  stat.timestamp=time_manager.current_time_us();
  float yy = lookup[readEncoder()];
  //interrupts();
  
  sync_manager.step();
  trajectory_manager.step();
  
    if (!diag_calibration_rcvd)
    {
      if (lookup[0]!=0 && lookup[16383]!=0)
        diag_calibration_rcvd=1;
      //else
      //  disableMotorDrivers();
    }
  

    if (dirty_trigger)
    {
        memcpy((uint8_t *) (&trg),(uint8_t *) (&trg_in),sizeof(Trigger));
        dirty_trigger=0;

        if (trg.data & TRIGGER_BOARD_RESET)
          board_reset_cnt=100;

        if (trg.data & TRIGGER_ENABLE_TRACE)
        {
            trace_manager.enable_trace();
        }

        if (trg.data & TRIGGER_DISABLE_TRACE)
        {
          trace_manager.disable_trace();
        }
        
    }

    if (board_reset_cnt)
    {
      board_reset_cnt--; //Countdown to allow time for RPC to finish up
      if (board_reset_cnt==0)
      { 
        //disableMotorDrivers();
        NVIC_SystemReset();
      }
    }
    
    if (dirty_gains)
    {
    
      //RC = 1/(2*pi*Hz)
      //A = exp(-1/(RC*Fs)) = exp(-2*pi*Hz/Fs)
      //B = 1-A
      if (gains_in.pLPF!=gains.pLPF) //PID Pos D term filter
      {
        pLPFa = exp(gains_in.pLPF*-2*3.14159/FsCtrl); // z = e^st pole mapping
        pLPFb = (1.0-pLPFa);
      }
      if (gains_in.effort_LPF!=gains.effort_LPF) //Effort filter
      {
        efLPFa = exp(gains_in.effort_LPF*-2*3.14159/FsCtrl); // z = e^st pole mapping
        efLPFb = (1.0-efLPFa);
      }
      if (gains_in.vLPF!=gains.vLPF) //PID Vel 
      {
        vLPFa = exp(gains_in.vLPF*-2*3.14159/FsCtrl); // z = e^st pole mapping
        vLPFb = (1.0-vLPFa)* FsCtrl;
      }
      if (gains_in.vel_status_LPF!=gains.vel_status_LPF) //Status vel
      {
        vsLPFa = exp(gains_in.vel_status_LPF*-2*3.14159/FsCtrl); // z = e^st pole mapping
        vsLPFb = (1.0-vsLPFa)* FsCtrl;
      }
      if (BOARD_VARIANT >= 3)
      {
        analog_manager.update_config(&gains_in, &gains);
      }
      
      if (gains_in.vTe_d != gains.vTe_d)
        vg.set_error_max(gains_in.vTe_d);
      
      memcpy((uint8_t *) (&gains),(uint8_t *) (&gains_in),sizeof(Gains));

      runstop_enabled = gains.config & CONFIG_ENABLE_RUNSTOP;
      sync_manager.sync_mode_enabled = gains.config & CONFIG_ENABLE_SYNC_MODE;
      guarded_mode_enabled = gains.config & CONFIG_ENABLE_GUARDED_MODE;
      flip_encoder_polarity = gains.config & CONFIG_FLIP_ENCODER_POLARITY;
      flip_effort_polarity = gains.config & CONFIG_FLIP_EFFORT_POLARITY;


      uMAX_P = current_to_effort(gains.iMax_pos);
      uMAX_N = current_to_effort(gains.iMax_neg);
      uMAX=uMAX_P; //This is only needed for mecahduino_menu mode...
      PA = max(0,min(3.6,gains.phase_advance_d)); //Keep within 2 steps
      
      dirty_gains=0;
      first_step_safety=10; //recapture hold position in case encoder polarity has flipped
    }

    
      ////// Compute sensor data

    
     if (trg.data & TRIGGER_MARK_POS_ON_CONTACT)
     {
        dirty_trigger_mark_on_contact=true;
        mark_on_contact_pos=trg.tdata;
     }
   
     if (trg.data & TRIGGER_MARK_POS || ( dirty_trigger_mark_on_contact && guarded_override))
     {
      if (dirty_trigger_mark_on_contact && guarded_override)
      {
        dirty_trigger_mark_on_contact=0;
        mpos_d=rad_to_deg(mark_on_contact_pos);
      }
      else
        mpos_d=rad_to_deg(trg.tdata);
        
      //Reset position measurement
      //mark_pos = yy; //0-360
      hold_pos = mpos_d;
      if (flip_encoder_polarity)
        mpos_d=mpos_d*-1;
      wrap_count=(int)(mpos_d) / 360;
      mark_rem = mpos_d-360*wrap_count;
      mark_pos=yy-mark_rem;

      y_1=yy;
      yw_1=0;        
      //Reset velocity measurements
      v=0;
      vs=0;
      
      //Reset control state
      
      r=mpos_d;
      mg.safe_switch_on(mpos_d,0);
   
     }
     if (trg.data & TRIGGER_RESET_POS_CALIBRATED)
     {
      diag_pos_calibrated=false;
     }
     if (trg.data & TRIGGER_POS_CALIBRATED)
     {
      diag_pos_calibrated=true;
     }
       
    //read encoder and lookup corrected angle in calibration lookup table
    if ((yy - y_1) < -180.0) wrap_count += 1;      //Check if we've rotated more than a full revolution (have we "wrapped" around from 359 degrees to 0 or ffrom 0 to 359?)
    else if ((yy - y_1) > 180.0) wrap_count -= 1;
    if (!flip_encoder_polarity)
    {
      yw = (yy + (360.0 * wrap_count))-mark_pos;              //yw is the wrapped angle (can exceed one revolution)
      ywd = ((double)yy + (double)(360.0 * wrap_count))-mark_pos;              //ywd is double represenation
    }
    else
    {
      yw = -1*((yy + (360.0 * wrap_count))-mark_pos);              //yw is the wrapped angle (can exceed one revolution)
      ywd = -1*(((double)yy + (double)(360.0 * wrap_count))-mark_pos);              //ywd is double represenation
    }

    if (first_filter)
    {
      v=(yw-yw_1);
      vs=v;
    }
      
    v = vLPFa*v +  vLPFb*(yw-yw_1);     //compute velocity for vel PID
    vs = vsLPFa*vs +  vsLPFb*(yw-yw_1);     //compute velocity status msg


    /////////// Safety Logic ////////////
    
    //May override commanded control mode.
     uint8_t mode_last=cmd.mode;
     if (first_step_safety>0)
     {
      hold_pos=yw; //Grab position at startup
      first_step_safety--;
     }
     
     diag_runstop_on=(sync_manager.runstop_active && runstop_enabled);
     //stat.debug=diag_runstop_on;//sync_manager.runstop_active;
     //Force to safety mode on runstop or velocity watchdog
     if (diag_runstop_on || (vel_watchdog==0 && (gains.config & CONFIG_ENABLE_VEL_WATCHDOG)))
     {
        cmd_in.mode=MODE_SAFETY;
        cmd.mode=MODE_SAFETY;
        safety_override=true;
     }
      safety_override=false;

    


      /////////// Copy in new Command Data  ///////////
      
    //Determine new controller mode / controller settings
    //If sync_irq_cnt has incremented then it is a sync trigger
    diag_waiting_on_sync = sync_manager.sync_mode_enabled && ((sync_manager.sync_irq_cnt==sync_irq_cnt_last && dirty_cmd)||trajectory_manager.is_trajectory_waiting_on_sync());




    if (dirty_cmd)
    {
      if (!sync_manager.sync_mode_enabled || (sync_manager.sync_mode_enabled && (sync_manager.sync_irq_cnt != sync_irq_cnt_last))
      || (sync_manager.sync_mode_enabled && cmd_in.mode == MODE_SAFETY) ) //Don't require sync to go into safety
      {
        //New command is ready to ingest
        cmd_cnt_exec++;
        diag_waiting_on_sync=false;
        trajectory_manager.waiting_on_sync=false;

        if (guarded_override) //Reset on new command to track
          guarded_override=0;

        if (!safety_override && !guarded_override) //Bring in new desired mode
        {
          cmd.mode=cmd_in.mode; 
        }

      //Set the incremental position target if triggered
       if (cmd.mode==MODE_POS_TRAJ_INCR  &&  cmd_in.incr_trigger != cmd.incr_trigger)
        {
          x_des_incr = yw + rad_to_deg(cmd_in.x_des);
          //x_des_incr = xdes+ rad_to_deg(cmd_in.x_des);//Use xdes instead of yw so we don't add in steady state error
          //stat.debug=yw;
          
        }
        else
          cmd.x_des=cmd_in.x_des;
          
        cmd.incr_trigger=cmd_in.incr_trigger;
        cmd.i_feedforward=cmd_in.i_feedforward;
        cmd.stiffness=cmd_in.stiffness;
        cmd.i_contact_pos =cmd_in.i_contact_pos;
        cmd.i_contact_neg =cmd_in.i_contact_neg;
        g_eff_pos=current_to_effort_v_norm(abs(cmd.i_contact_pos));
        g_eff_neg=current_to_effort_v_norm(-1*abs(cmd.i_contact_neg));
      
        //If mode has changed manage smooth switchover
        if (cmd.mode!=mode_last)
        {
          
          switch(cmd.mode)
          {
            case MODE_SAFETY:
            case MODE_HOLD:
              hold_pos=yw;
              break; 
            case MODE_POS_PID:
              r=yw;
              break; 
            case MODE_VEL_PID:
              r=v;
              vg.safe_switch_on(yw,v);
              vg.setMaxAcceleration(abs(rad_to_deg(cmd_in.a_des)));
              break; 
            case MODE_VEL_TRAJ:
              vg.safe_switch_on(yw,v);
              vg.setMaxAcceleration(abs(rad_to_deg(cmd_in.a_des)));
              break; 
            case MODE_POS_TRAJ:
              mg.safe_switch_on(yw,v);
              mg.setMaxVelocity(abs(rad_to_deg(cmd_in.v_des)));
              mg.setMaxAcceleration(abs(rad_to_deg(cmd_in.a_des)));
              break; 
            case MODE_POS_TRAJ_INCR:
              mg.safe_switch_on(yw,v);
              mg.setMaxVelocity(abs(rad_to_deg(cmd_in.v_des)));
              mg.setMaxAcceleration(abs(rad_to_deg(cmd_in.a_des)));
              break; 
            case MODE_POS_TRAJ_WAYPOINT:
              eff_max=0;
              mg.safe_switch_on(yw,v);
              mg.setMaxVelocity(abs(rad_to_deg(cmd_in.v_des)));
              mg.setMaxAcceleration(abs(rad_to_deg(cmd_in.a_des)));
              trajectory_manager.q=yw;
              traj_hold_pos=yw;
              break; 
            case MODE_CURRENT:
              u=0;
              e=0;
              break; 
          };
        }
   

          
      //Handle on-the-fly updates of velocity and accel commands  
      if (cmd.mode==MODE_VEL_TRAJ)
        vg.setMaxAcceleration(abs(rad_to_deg(cmd_in.a_des)));
        
      if (cmd.mode==MODE_POS_TRAJ || cmd.mode==MODE_POS_TRAJ_INCR || cmd.mode==MODE_POS_TRAJ_WAYPOINT)
      {
        if(cmd_in.v_des!=cmd.v_des)
          mg.setMaxVelocity(abs(rad_to_deg(cmd_in.v_des)));
        if(cmd_in.a_des !=cmd.a_des)
          mg.setMaxAcceleration(abs(rad_to_deg(cmd_in.a_des)));
        
        cmd.v_des=cmd_in.v_des;
        cmd.a_des=cmd_in.a_des;
        
      }
      if (cmd.mode==MODE_VEL_TRAJ || cmd.mode==MODE_VEL_PID)
      {
        cmd.v_des=cmd_in.v_des;
        cmd.a_des=cmd_in.a_des;
      }
     if (cmd.mode==MODE_VEL_TRAJ || cmd.mode==MODE_VEL_PID)
        vg.setMaxAcceleration(abs(rad_to_deg(cmd.a_des)));
      
      dirty_cmd=0;
      vel_watchdog=TC4_LOOP_RATE;//1s
      sync_irq_cnt_last=sync_manager.sync_irq_cnt;
      }
    } //dirty_cmd

    if (cmd.mode==MODE_VEL_TRAJ || cmd.mode==MODE_VEL_PID)
      vel_watchdog=max(0,vel_watchdog-1);
    else
      vel_watchdog=TC4_LOOP_RATE;


   // sync_manager.motor_sync_triggered=false;


  if(cmd.mode!=MODE_SAFETY)
  {    
    if (cmd.stiffness>stiffness_target)
    {
      stiffness_target=min(cmd.stiffness, stiffness_target+STIFFNESS_SLEW);
    }
    if (cmd.stiffness<stiffness_target)
    {
      stiffness_target=max(cmd.stiffness,stiffness_target-STIFFNESS_SLEW);
    }
  }
  else
  {
    if (gains.safety_stiffness>stiffness_target)
    {
      stiffness_target=min(gains.safety_stiffness, stiffness_target+STIFFNESS_SLEW);
    }
    if (gains.safety_stiffness<stiffness_target)
    {
      stiffness_target=max(gains.safety_stiffness,stiffness_target-STIFFNESS_SLEW);
    }
  }
  //////////////////////////////////////////

    if (trg.data & TRIGGER_RESET_MOTION_GEN)
    {
      mg.safe_switch_on(yw,v);
      vg.safe_switch_on(yw,v);
    }

    
    diag_is_mg_accelerating=0;
    diag_is_mg_moving=0;

      ////// Now run control cycle

      //Make sure drives are on (unless in safety freewheel)
      if (cmd.mode!=MODE_SAFETY && !motor_enabled_flag)
        enableMotorDrivers();

      switch (cmd.mode)
      {
        case MODE_FREEWHEEL:
            u=0;
            e=0;
            diag_near_pos_setpoint=0;
            diag_near_vel_setpoint=0;
            diag_is_mg_accelerating=0;
            diag_is_mg_moving=0;
            break;

        case MODE_CURRENT:
            u=current_to_effort(cmd.i_feedforward);
            e=0;
            diag_near_pos_setpoint=0;
            diag_near_vel_setpoint=0;
            diag_is_mg_accelerating=0;
            diag_is_mg_moving=0;
            break;
        case MODE_SAFETY:
          if (!(gains.config & CONFIG_SAFE_MODE_HOLD)) //Safety Freewheel (base wheels and arm)
           {
            u=0;
            e=0;
            diag_near_pos_setpoint=0;
            diag_near_vel_setpoint=0;
            diag_is_mg_accelerating=0;
            diag_is_mg_moving=0;
            //This makes them easier to backdrive
            disableMotorDrivers();
            break;
           } //else do a safety float /hold
           else
           {
            e = (hold_pos - yw);
            ITerm += (gains.pKi * e);                             //Integral wind up limit
            if (ITerm > gains.pKi_limit) ITerm = gains.pKi_limit;
            else if (ITerm < -gains.pKi_limit) ITerm = -gains.pKi_limit;          
            DTerm = pLPFa*DTerm -  pLPFb*gains.pKd*(yw-yw_1);
            u = (gains.pKp * e) + ITerm + DTerm;
            //stat.debug=current_to_effort(gains.i_safety_feedforward);
            u=u*gains.safety_stiffness+current_to_effort(gains.i_safety_feedforward);
            
            //stat.debug=u;//current_to_effort(gains.i_safety_feedforward);//gains.i_safety_feedforward;//current_to_effort(gains.i_safety_feedforward);
            diag_near_pos_setpoint=abs(e)<gains.pos_near_setpoint_d;
            diag_near_vel_setpoint=0;
            diag_is_mg_accelerating=0;
            diag_is_mg_moving=0;
            break; 
           }
        case MODE_HOLD:
            e = (hold_pos - yw);
            ITerm += (gains.pKi * e);                             //Integral wind up limit
            if (ITerm > gains.pKi_limit) ITerm = gains.pKi_limit;
            else if (ITerm < -gains.pKi_limit) ITerm = -gains.pKi_limit;          
            DTerm = pLPFa*DTerm -  pLPFb*gains.pKd*(yw-yw_1);
            u = (gains.pKp * e) + ITerm + DTerm;
            u=u*stiffness_target+current_to_effort(cmd.i_feedforward);
            diag_near_pos_setpoint=abs(e)<gains.pos_near_setpoint_d;
            diag_near_vel_setpoint=0;
            diag_is_mg_accelerating=0;
            diag_is_mg_moving=0;
            break; 
        
        case MODE_POS_PID:
            if(motion_limits_set && diag_pos_calibrated)
            {
              cmd.x_des=min(max(cmd.x_des, motion_limits.pos_min), motion_limits.pos_max);
            }
            e = (rad_to_deg(cmd.x_des) - yw);
            ITerm += (gains.pKi * e);                             //Integral wind up limit
            if (ITerm > gains.pKi_limit) ITerm = gains.pKi_limit;
            else if (ITerm < -gains.pKi_limit) ITerm = -gains.pKi_limit;          
            DTerm = pLPFa*DTerm -  pLPFb*gains.pKd*(yw-yw_1);
            u = (gains.pKp * e) + ITerm + DTerm;
            u=u*stiffness_target+current_to_effort(cmd.i_feedforward);
            diag_near_pos_setpoint=abs(e)<gains.pos_near_setpoint_d;
            diag_near_vel_setpoint=0;
            diag_is_mg_accelerating=0;
            diag_is_mg_moving=0;
            break;    
        
        case MODE_VEL_PID:
            vg.update(rad_to_deg(cmd.v_des),yw);
            e = (vg.vel -v);
            ITerm += (gains.vKi * e);                 //Integral wind up limit
            if (ITerm > gains.vKi_limit) ITerm = gains.vKi_limit;
            else if (ITerm < -gains.vKi_limit) ITerm = -gains.vKi_limit;
            u = ((gains.vKp * e) + ITerm - (gains.vKd * (e-e_1)));
            u=u*stiffness_target;
            diag_near_pos_setpoint=0;
            diag_near_vel_setpoint=abs(e)<gains.vel_near_setpoint_d;
            diag_is_mg_accelerating=vg.isAccelerating();
            diag_is_mg_moving=vg.isMoving();
            break;
        
        case MODE_VEL_TRAJ:
            xdes=vg.update(rad_to_deg(cmd.v_des),yw); //get target position
            e = (xdes - yw);
            ITerm += (gains.pKi * e);                             //Integral wind up limit
            if (ITerm > gains.pKi_limit) ITerm = gains.pKi_limit;
            else if (ITerm < -gains.pKi_limit) ITerm = -gains.pKi_limit;          
            DTerm = pLPFa*DTerm -  pLPFb*gains.pKd*(yw-yw_1);
            u = (gains.pKp * e) + ITerm + DTerm;
            u=u*stiffness_target+current_to_effort(cmd.i_feedforward);
            diag_near_pos_setpoint=0;
            diag_near_vel_setpoint=abs(rad_to_deg(cmd.v_des)-v)<gains.vel_near_setpoint_d;
            diag_is_mg_accelerating=vg.isAccelerating();
            diag_is_mg_moving=vg.isMoving();
            break;    

        case MODE_POS_TRAJ_INCR:
        if (stiffness_target==0.0)
            {
              mg.follow(yw,v); //floating so force mg to track
              xdes=yw;
            }
            else
            {
              if (motion_limits_set  && diag_pos_calibrated)
              {
                  x_des_incr=min(max(x_des_incr, rad_to_deg(motion_limits.pos_min)), rad_to_deg(motion_limits.pos_max));
              }
              
              xdes=mg.update(x_des_incr); //get target position
              
            }
            e = (xdes - yw);
            ITerm += (gains.pKi * e);                             //Integral wind up limit
            if (ITerm > gains.pKi_limit) ITerm = gains.pKi_limit;
            else if (ITerm < -gains.pKi_limit) ITerm = -gains.pKi_limit;    
            
            DTerm = pLPFa*DTerm -  pLPFb*gains.pKd*(yw-yw_1);
            u = (gains.pKp * e) + ITerm + DTerm;
            u=u*stiffness_target+current_to_effort(cmd.i_feedforward);
            diag_near_pos_setpoint=abs((x_des_incr -yw))<gains.pos_near_setpoint_d;
            //stat.debug=abs((x_des_incr -yw));
            diag_near_vel_setpoint=0;
            diag_is_mg_accelerating=mg.isAccelerating();
            diag_is_mg_moving=mg.isMoving();
            break;
        case MODE_POS_TRAJ:
        
            if (stiffness_target==0.0)
            {
              mg.follow(yw,v); //floating so force mg to track
              xdes=yw;
            }
            else
            {
              if (motion_limits_set)
              {
                cmd.x_des=min(max(cmd.x_des, motion_limits.pos_min), motion_limits.pos_max);
                //stat.debug=motion_limits.pos_min;
              }
              xdes=mg.update(rad_to_deg(cmd.x_des)); //get target position
            }
            e = (xdes - yw);
            ITerm += (gains.pKi * e);                             //Integral wind up limit
            if (ITerm > gains.pKi_limit) ITerm = gains.pKi_limit;
            else if (ITerm < -gains.pKi_limit) ITerm = -gains.pKi_limit;      
            DTerm = pLPFa*DTerm -  pLPFb*gains.pKd*(yw-yw_1);
            u = (gains.pKp * e) + ITerm + DTerm;
            u=u*stiffness_target+current_to_effort(cmd.i_feedforward);
            diag_near_pos_setpoint=abs((rad_to_deg(cmd.x_des) -yw))<gains.pos_near_setpoint_d;
            //stat.debug=abs((rad_to_deg(cmd.x_des) -yw));
            diag_near_vel_setpoint=0;
            diag_is_mg_accelerating=mg.isAccelerating();
            diag_is_mg_moving=mg.isMoving();
            break;
        case MODE_POS_TRAJ_WAYPOINT:
            if (stiffness_target==0.0)
            {
              mg.follow(yw,v); //floating so force mg to track
              xdes=yw;
            }
            else
            {
              if(trajectory_manager.is_trajectory_active())
              {
                if (motion_limits_set)
                  xdes=mg.update(rad_to_deg(min(max(trajectory_manager.q, motion_limits.pos_min), motion_limits.pos_max)));
                else
                  xdes=mg.update(rad_to_deg(trajectory_manager.q)); //get target position
                traj_hold_pos=yw;
                //stat.debug++;
              }
              else
                  xdes=traj_hold_pos;
            }
            e = (xdes - yw);
            ITerm += (gains.pKi * e);                             //Integral wind up limit
            if (ITerm > gains.pKi_limit) ITerm = gains.pKi_limit;
            else if (ITerm < -gains.pKi_limit) ITerm = -gains.pKi_limit;      
            DTerm = pLPFa*DTerm -  pLPFb*gains.pKd*(yw-yw_1);
            u = (gains.pKp * e) + ITerm + DTerm;
            u=u*stiffness_target+current_to_effort(cmd.i_feedforward);
            //u=0;
            
            diag_near_pos_setpoint=abs((rad_to_deg(cmd.x_des) -yw))<gains.pos_near_setpoint_d;
            diag_near_vel_setpoint=0;
            diag_is_mg_accelerating=mg.isAccelerating();
            diag_is_mg_moving=mg.isMoving();
            break;
               
      };
  
  if(flip_effort_polarity)
  {
    u=u*-1;
    uMAX_PF=abs(uMAX_N);
    uMAX_NF=-1*abs(uMAX_P);
  }
  else
  {
    uMAX_PF=abs(uMAX_P);
    uMAX_NF=-1*abs(uMAX_N);
  }
              
  
    diag_at_current_limit = 0;  
    if (u > 0)          //Depending on direction we want to apply torque, add or subtract a phase angle of PA for max effective torque.  PA should be equal to one full step angle: if the excitation angle is the same as the current position, we would not move!  
      {                 //You can experiment with "Phase Advance" by increasing PA when operating at high speeds
          if (u > uMAX_PF)     // limit control effort
          {
            u = uMAX_PF;       //saturation limits max current command
            diag_at_current_limit = 1;
          }
       
        PAY=PA;
      }
    else
      {
        if (u < uMAX_NF)    // limit control effort
        {
          u = uMAX_NF;      //saturation limits max current command
          diag_at_current_limit = 1;
        }
        PAY=-PA;
      }


  if (first_filter)
      eff=u;
  if(flip_effort_polarity)
    eff = efLPFa*eff +  efLPFb*(-1*u);
  else
   eff = efLPFa*eff +  efLPFb*(u);

   //eff_max=max(eff_max,abs(eff));
   //stat.debug=g_eff_pos;//eff_max;//effort_to_current(eff_max);
    ///////////////////////////////
    //Guarded Mode

     if (guarded_mode_enabled)
     {
          if (eff>g_eff_pos || eff<g_eff_neg)
          {
            guarded_eff_cnt++;
            if (guarded_eff_cnt>GUARDED_EFF_THRESH)
            {
                guarded_event_cnt++;
                if (!guarded_override && (cmd.mode==MODE_POS_TRAJ ||cmd.mode==MODE_POS_TRAJ_INCR || cmd.mode==MODE_VEL_TRAJ || cmd.mode==MODE_POS_TRAJ_WAYPOINT || cmd.mode==MODE_POS_PID || cmd.mode==MODE_VEL_PID)) //Hit a new contact event, hold position
                {
                  guarded_override=1;
                  hold_pos=yw;
                  //stat.debug=deg_to_rad(hold_pos);
                  if (cmd.mode==MODE_POS_TRAJ_WAYPOINT)
                  {
                    trajectory_manager.reset();
                  }
                  cmd.mode=MODE_SAFETY;
                }
            }
         }
         else
         guarded_eff_cnt=0;//reset
     }
     else
     {
      guarded_override=0;
      guarded_event_cnt=0;
      guarded_eff_cnt=0;
     }
      
  ////////////////////
  
    U = abs((int)u);      //Effort for commutation loop

   y_1 = yy;  //copy current value of y to previous value (y_1) for next control cycle before PA angle added
  
   // e_3 = e_2;    //copy current values to previous values for next control cycle
    e_2 = e_1;    //these past values can be useful for more complex controllers/filters.  Uncomment as necessary    
    e_1 = e;
   // u_3 = u_2;
    u_2 = u_1;
    u_1 = u;
    yw_1 = yw;

  

  ////////// Handle is_moving
  float vel_is_moving = (float)(abs(ywd- pos_history[pos_history_idx]))*4;//4 as 250ms of history, convert to deg/s
  diag_is_moving = vel_is_moving>gains.vel_near_setpoint_d;
  //stat.debug= vel_is_moving;
  pos_history[pos_history_idx]=ywd;
  pos_history_idx++;
  if (pos_history_idx>=N_POS_HISTORY)
    pos_history_idx=0;
    
      
  //Cleanup
  trg.data=0; //Clear triggers
  first_filter=false;
  if (BOARD_VARIANT >= 3)
  {
    analog_manager.step();
  }
  update_status();
  update_trace();
  ctrl_cycle_cnt++;
}

///////////////////////// Commutation Loop ///////////////////////////



//Called every control cycle via interrupt (6.5Khz, sync)
//The desired effort (U) and Phase Advance (PAY) is updated in the control loop at a lower rate
//This loop updates raw encoder position (y) asynchronously from the control loop
void stepHelloCommutation()
{
  if (TC5->COUNT16.INTFLAG.bit.OVF == 1) 
  {
    noInterrupts();
    y = lookup[readEncoder()];
    interrupts();
    
    if (receiving_calibration)
    {
      
      analogFastWrite(VREF_2, 0);     //set phase currents to zero
      analogFastWrite(VREF_1, 0); 
    }
    else
      output(-(y+PAY), round(U));
    TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }
}

///////////////////////// Control Loop ///////////////////////////


void TC4_Handler() {                // gets called with FsMg frequency

  if (TC4->COUNT16.INTFLAG.bit.OVF == 1) {    // A counter overflow caused the interrupt
      TC4->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
      time_manager.ts_base++;
    
      toggle_led(500);
      if (hello_interface)
        stepHelloController();
  
  }
}


void setupMGInterrupts() {  // configure the controller interrupt

 ////////////////////////// Counter 4 ///////////////////////////
 
  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC4)                      // wait for sync

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC4)

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC4)

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC4)

  TC4->COUNT16.CC[0].reg =  TC4_COUNT_PER_CYCLE; //(int)( round(48000000 / FsCtrl / 2)); //0x3E72; //0x4AF0;
  WAIT_TC16_REGS_SYNC(TC4)

  TC4->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC4->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC4->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0


   ////////////////////////// Setup Interrupts ///////////////////////////

//Set interrupt priority so Controller (TC4) preempts Commutation (TC5) (Inverted numbering scheme)
//This ensures stable time base for controller filters
//Jitter on commutation seems to be OK for performance


  NVIC_SetPriority(TC4_IRQn, 1);      //1        see https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/cortex_handlers.c#L84
  NVIC_SetPriority(TC5_IRQn, 2);      //2        

  // Enable InterruptVector
  NVIC_EnableIRQ(TC4_IRQn);


  // Enable TC
  //  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  //  WAIT_TC16_REGS_SYNC(TC5)
}


void enableMGInterrupts() {   //enables the controller interrupt ("closed loop mode")
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC4
  WAIT_TC16_REGS_SYNC(TC4)                      //wait for sync
}

void disableMGInterrupts() {  //disables the controller interrupt ("closed loop mode")
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC4
  WAIT_TC16_REGS_SYNC(TC4)                      // wait for sync
}

/////////////////////// Watchdog //////////////////////////////////////
// reference https://forum.arduino.cc/t/wdt-watchdog-timer-code/353610

void WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

void resetWDT() {
  // reset the WDT watchdog timer.
  // this must be called before the WDT resets the system
  if(WDT->CTRL.bit.ENABLE!=1){
    setupWDT(WDT_TIMEOUT_PERIOD);
  }
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

void disableWDT(){
  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required
}

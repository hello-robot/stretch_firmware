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

#define V_TO_RAW(v) v*1024/20.0 //per circuit
#define I_TO_RAW(i) (i*1000)*0.408*1024.0/3300 //per circuit

float cliff_LPFa = 1.0; 
float cliff_LPFb = 0.0;
float voltage_LPFa = 1.0; 
float voltage_LPFb = 0.0;
float current_LPFa = 1.0; 
float current_LPFb = 0.0;
float temp_LPFa = 1.0; 
float temp_LPFb = 0.0;
float accel_LPFa = 1.0; 
float accel_LPFb = 0.0;

float low_voltage_alert=V_TO_RAW(10.0);
int low_voltage_alert_cnt=0;

float high_current_alert= I_TO_RAW(6.0);

float over_tilt_alert_deg = 10.0;
int startup_cnt=500;

float cliff[4];
bool at_cliff[4];
float voltage;
float current;
float temp;
bool first_filter=true;

bool state_runstop_event=false;
bool state_cliff_event=false;
bool state_fan_on=false;
bool state_buzzer_on=false;
bool state_low_voltage_alert=false;
bool state_high_current_alert=false;
bool state_over_tilt_alert=false;

bool dirty_config=false;
bool dirty_trigger=false;
bool dirty_motor_sync=false;

//////////////////////////////////////
Pimu_Config cfg_in, cfg;
Pimu_Trigger trg_in, trg;
Pimu_Status stat, stat_out;
Pimu_Board_Info board_info;


void setupTimer5();
void runstop_toggle_led(int rate_ms);
void toggle_led(int rate_ms);
void step_beep();

#define MODE_RUNNING 0
#define MODE_RUNSTOP_ACTIVE 1
#define MODE_RESET_ACTIVE 2
unsigned long t_low;
int runstop_mode=MODE_RUNNING;
bool alert_trigger_runstop=false;
uint32_t cycle_cnt=0;

/////////////////////////////////////////////////////////////////////////
float deg_to_rad(float x)
{
  return x*0.017453292519943295;
}
float rad_to_deg(float x)
{
  return x*57.29577951308232;
}

//////////////////////// ANALOG READ ISR ///////////////////////////////////////////
// Use the SAMD21's ISR to transfer ADC results to buffer array in memory

#define NUM_ADC_INPUTS 8
#define IDX_ANA_V_BATT 0
#define IDX_ANA_CLIFF_0 2
#define IDX_ANA_CLIFF_1 3
#define IDX_ANA_CLIFF_2 4
#define IDX_ANA_CLIFF_3 5
#define IDX_ANA_TEMP 6
#define IDX_ANA_CURRENT 7
volatile uint16_t adcResult[NUM_ADC_INPUTS] = {};         // ADC results buffer
uint8_t adc_input_id=0;
void setupADC()
{
  //ADC ISR setup
  ADC->INPUTCTRL.bit.MUXPOS = adc_input_id;                   // Set the analog input to A0
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  ADC->SAMPCTRL.bit.SAMPLEN = 0x00;                  // Set max Sampling Time Length to half divided ADC clock pulse (2.66us)
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |      // Divide Clock ADC GCLK by 512 (48MHz/512 = 93.7kHz)
                   ADC_CTRLB_RESSEL_10BIT |          // Set the ADC resolution to 10 bits (use ADC_CTRLB_RESSEL_12BIT for 12)
                   ADC_CTRLB_FREERUN;                // Set the ADC to free run
 
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization 
  NVIC_SetPriority(ADC_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for the ADC to 0 (highest)
  NVIC_EnableIRQ(ADC_IRQn);         // Connect the ADC to Nested Vector Interrupt Controller (NVIC)
  ADC->INTENSET.reg = ADC_INTENSET_RESRDY;           // Generate interrupt on result ready (RESRDY)
  ADC->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  ADC->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
  while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
}

void ADC_Handler()
{
 
  if (ADC->INTFLAG.bit.RESRDY)                       // Check if the result ready (RESRDY) flag has been set
  {
    ADC->INTFLAG.bit.RESRDY = 1;                     // Clear the RESRDY flag
    while(ADC->STATUS.bit.SYNCBUSY);                 // Wait for read synchronization
    adcResult[adc_input_id] = ADC->RESULT.reg;          // Read the result;
    adc_input_id++;
    if (adc_input_id==NUM_ADC_INPUTS)
      adc_input_id=0;
    ADC->CTRLA.bit.ENABLE = 0;                     // Disable the ADC
    while(ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization
    ADC->INPUTCTRL.bit.MUXPOS = adc_input_id;         // Set the analog input channel
    while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
    ADC->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
    while(ADC->STATUS.bit.SYNCBUSY);                   // Wait for synchronization
  }
}


///////////////////// TIMING and TIMESTAMP /////////////////////////////////////////
#define FS 100 //Loop rate in Hz
#define TC5_COUNT_PER_CYCLE (int)( round(48000000 / 16 / FS)) //30,000 at 100hz, 16:1 prescalar TC5 is 32bit timer
#define US_PER_CYCLE 1000000/FS //10000 at 50Hz
#define US_PER_TC5_TICK 1000000.0*16/48000000 //0.33us resolution

float DT_ms= 1000/FS;

//Avoid using millis() in ISR
unsigned long get_elapsed_time_ms(){
  return (unsigned long)((float)(DT_ms*(float)cycle_cnt));
  
}

uint64_t ts_base=0;

uint64_t get_TC5_timestamp()
{
  float delta = TC5->COUNT16.COUNT.reg*US_PER_TC5_TICK;
  return ts_base + (int)delta;
}

void zero_timestamp()
{
  ts_base=0;
  TC5->COUNT16.COUNT.reg=0;
}

///////////////////////////////////////////////
void setupPimu() {  

  memset(&cfg_in, 0, sizeof(Pimu_Config));
  memset(&cfg, 0, sizeof(Pimu_Config));
  cfg.stop_at_runstop=1; //By default acknowledge runstop, user must override via YAML otherwise
  memset(&trg_in, 0, sizeof(Pimu_Trigger));
  memset(&trg, 0, sizeof(Pimu_Trigger));
  memset(&stat, 0, sizeof(Pimu_Status));
  memcpy(&(board_info.board_version),BOARD_VERSION,min(20,strlen(BOARD_VERSION)));
  memcpy(&(board_info.firmware_version),FIRMWARE_VERSION,min(20,strlen(FIRMWARE_VERSION)));
  setupTimer5();
  zero_timestamp();
}


///////////////////// LED /////////////////////////////////////////


bool led_on=false;
unsigned long t_toggle_last=0;

///////////////RUNSTOP LED //////////////////////////
bool runstop_led_on=false;
unsigned long runstop_t_toggle_last=0;
void runstop_toggle_led(int rate_ms)
{
  unsigned long t = get_elapsed_time_ms();
  if (t-runstop_t_toggle_last>rate_ms)
  {
    runstop_t_toggle_last=t;
    if (!runstop_led_on)
    {
        digitalWrite(RUNSTOP_LED, HIGH);
    }
      else
      {
        digitalWrite(RUNSTOP_LED, LOW);
      }
     runstop_led_on=!runstop_led_on;
  }
}

void toggle_led(int rate_ms)
{
  unsigned long t = get_elapsed_time_ms();
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


///////////////////////////RPC ////////////////////////////////////

void handleNewRPC()
{
  switch(rpc_in[0])
  {
    case RPC_SET_PIMU_CONFIG: 
          memcpy(&cfg_in, rpc_in+1, sizeof(Pimu_Config)); //copy in the config
          dirty_config=true;
          rpc_out[0]=RPC_REPLY_PIMU_CONFIG;
          num_byte_rpc_out=1;
          break;
    case RPC_SET_PIMU_TRIGGER: 
          memcpy(&trg_in, rpc_in+1, sizeof(Pimu_Trigger)); //copy in
          dirty_trigger=true;
          rpc_out[0]=RPC_REPLY_PIMU_TRIGGER;
          memcpy(rpc_out+1,(uint8_t *)&(trg_in.data),sizeof(uint32_t));
          num_byte_rpc_out=1+sizeof(uint32_t);
          break;
    case RPC_GET_PIMU_STATUS: 
          rpc_out[0]=RPC_REPLY_PIMU_STATUS;
          memcpy(rpc_out + 1, (uint8_t *) (&stat_out), sizeof(Pimu_Status)); //Collect the status data
          num_byte_rpc_out=sizeof(Pimu_Status)+1;
          break; 
     case RPC_GET_PIMU_BOARD_INFO:
          rpc_out[0]=RPC_REPLY_PIMU_BOARD_INFO;
          memcpy(rpc_out + 1, (uint8_t *) (&board_info), sizeof(Pimu_Board_Info)); //Collect the status data
          num_byte_rpc_out=sizeof(Pimu_Board_Info)+1;
          break; 
     case RPC_SET_MOTOR_SYNC:
          rpc_out[0]=RPC_REPLY_MOTOR_SYNC;
          num_byte_rpc_out=1;
          dirty_motor_sync=1;
          break; 
   default:
        break;
  };
}

void stepPimuRPC()
{
  
  stepTransport(handleNewRPC);
}

void stepNonRT()
{
  
}

////////////////////////Controller///////////////////////////////////////
uint8_t board_reset_cnt=0;
uint8_t ds_cnt=0;
uint8_t motor_sync_cnt=0;
//100hz control rate, cnt of 3 creates sync high pulse of 33ms. 
//Stepper syncs at between 10 and 50ms pulse.
#define MOTOR_SYNC_DURATION 3
bool first_config = 1;
int runstop_on_cnt=0;
void step_runstop();
int fan_on_cnt=0;

void stepPimuController()
{
  ts_base+=US_PER_CYCLE;
  cycle_cnt++;
  toggle_led(500);
  
  if (dirty_config)
  {

    if (cfg_in.cliff_LPF!=cfg.cliff_LPF) //Effort filter
    {
      cliff_LPFa = exp(cfg_in.cliff_LPF*-2*3.14159/FS); // z = e^st pole mapping
      cliff_LPFb = (1.0-cliff_LPFa);
    }
    if (cfg_in.voltage_LPF!=cfg.voltage_LPF) //Effort filter
    {
      voltage_LPFa = exp(cfg_in.voltage_LPF*-2*3.14159/FS); // z = e^st pole mapping
      voltage_LPFb = (1.0-voltage_LPFa);
    }
    if (cfg_in.current_LPF!=cfg.current_LPF) //Effort filter
    {
      current_LPFa = exp(cfg_in.current_LPF*-2*3.14159/FS); // z = e^st pole mapping
      current_LPFb = (1.0-current_LPFa);
    }
    if (cfg_in.temp_LPF!=cfg.temp_LPF) //Effort filter
    {
      temp_LPFa = exp(cfg_in.temp_LPF*-2*3.14159/FS); // z = e^st pole mapping
      temp_LPFb = (1.0-temp_LPFa);
    }
    if (cfg_in.accel_LPF!=cfg.accel_LPF) //Effort filter
    {
      accel_LPFa = exp(cfg_in.accel_LPF*-2*3.14159/FS); // z = e^st pole mapping
      accel_LPFb = (1.0-accel_LPFa);
    }
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
    setIMUCalibration();

    if (first_config)
    {
      voltage = adcResult[IDX_ANA_V_BATT];
      current = adcResult[IDX_ANA_CURRENT];
      temp =    adcResult[IDX_ANA_TEMP];
      cliff[0] = adcResult[IDX_ANA_CLIFF_0];
      cliff[1] = adcResult[IDX_ANA_CLIFF_1];
      cliff[2] = adcResult[IDX_ANA_CLIFF_2];
      cliff[3] = adcResult[IDX_ANA_CLIFF_3];
      first_config=0;
      }
      dirty_config=false;
  }
  step_runstop();
  step_beep();
  if (dirty_trigger)
  {
    memcpy(&trg,&trg_in,sizeof(Pimu_Trigger));
    if (trg.data & TRIGGER_BEEP)
    {
      do_beep(BEEP_ID_SINGLE_SHORT);
    }
    if (trg.data & TRIGGER_BOARD_RESET)
    {
          board_reset_cnt=100;
    }
    if (trg.data & TRIGGER_RUNSTOP_RESET)
    {
          state_runstop_event=false;
          runstop_mode=MODE_RUNNING;
    }
    if (trg.data & TRIGGER_RUNSTOP_ON)
    {
          state_runstop_event=true;
          runstop_mode=MODE_RUNSTOP_ACTIVE;
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
         digitalWrite(IMU_RESET, LOW);
         delay(1);
         digitalWrite(IMU_RESET, HIGH);
    }
    if (trg.data & TRIGGER_TIMESTAMP_ZERO)
    {
         zero_timestamp();
    }
    dirty_trigger=false;
  }

  //Turn off fan automatically after a bit of time
  fan_on_cnt=max(0,fan_on_cnt-1);
  if (!fan_on_cnt && state_fan_on)
  {
    state_fan_on=false;
    digitalWrite(FAN_FET, LOW);
  }
  if (dirty_motor_sync)
  {
    dirty_motor_sync=0;
    motor_sync_cnt=MOTOR_SYNC_DURATION; 
  }
 /////////////////////////
  //Monitor voltage
  startup_cnt=max(0,startup_cnt-1);
  if(startup_cnt==0)
  {
    if(voltage<low_voltage_alert && cfg.stop_at_low_voltage) //dropped below
    {
      state_low_voltage_alert=true;
      alert_trigger_runstop=true;
      if(low_voltage_alert_cnt==0) //start new beep sequence
      {
        low_voltage_alert_cnt=(int)FS*6;
        do_beep(BEEP_ID_DOUBLE_SHORT);
      }
    }
    else
    {
      state_low_voltage_alert=false;
    }
    low_voltage_alert_cnt=max(0,low_voltage_alert_cnt-1);

   if(current>high_current_alert && cfg.stop_at_high_current) //dropped below
    {
      state_high_current_alert=true;
      alert_trigger_runstop=true;
    }
    else
    {
      state_high_current_alert=false;
    }

    if(isIMUOrientationValid() && (abs(stat.imu.pitch)>over_tilt_alert_deg || abs(stat.imu.roll)>over_tilt_alert_deg) && cfg.stop_at_tilt) //over tilt
      {
        state_over_tilt_alert=true;
        alert_trigger_runstop=true;
      }
      else
      {
        state_over_tilt_alert=false;
      }
    
  }
  

    
    if (board_reset_cnt)
    {
      board_reset_cnt--; //Countdown to allow time for RPC to finish up
      if (board_reset_cnt==0)
        NVIC_SystemReset();
    }
    


 if (state_runstop_event || state_cliff_event || motor_sync_cnt)
 {//Disable motors or generate sync pulse to trigger motors
    if (!motor_sync_cnt && (state_runstop_event || state_cliff_event))
      runstop_toggle_led(500);//indicate in runstop event
    if (motor_sync_cnt==MOTOR_SYNC_DURATION) //Timestamp low to high transition due to sync trigger
      stat.timestamp_last_sync=get_TC5_timestamp();
    digitalWrite(RUNSTOP_M0, HIGH);
    digitalWrite(RUNSTOP_M1, HIGH);
    digitalWrite(RUNSTOP_M2, HIGH);
    digitalWrite(RUNSTOP_M3, HIGH);
 }
  else 
  {//Enable motors

      digitalWrite(RUNSTOP_LED, HIGH);
      digitalWrite(RUNSTOP_M0, LOW);
      digitalWrite(RUNSTOP_M1, LOW);
      digitalWrite(RUNSTOP_M2, LOW);
      digitalWrite(RUNSTOP_M3, LOW);
  }
  motor_sync_cnt=max(motor_sync_cnt-1,0);

  stat.timestamp= get_TC5_timestamp();//update_timestamp(); //Tag timestamp just before reading IMU
  stepIMU();
  memcpy(&stat.imu,&imu_status, sizeof(IMU_Status));
  
  if (first_filter)
  {
    voltage = adcResult[IDX_ANA_V_BATT];
    current = adcResult[IDX_ANA_CURRENT];
    temp =    adcResult[IDX_ANA_TEMP];
    cliff[0] = adcResult[IDX_ANA_CLIFF_0];
    cliff[1] = adcResult[IDX_ANA_CLIFF_1];
    cliff[2] = adcResult[IDX_ANA_CLIFF_2];
    cliff[3] = adcResult[IDX_ANA_CLIFF_3];
    first_filter=false;
  }
  cliff[0]= cliff_LPFa*cliff[0] +  cliff_LPFb*adcResult[IDX_ANA_CLIFF_0];
  cliff[1]= cliff_LPFa*cliff[1] +  cliff_LPFb*adcResult[IDX_ANA_CLIFF_1];
  cliff[2]= cliff_LPFa*cliff[2] +  cliff_LPFb*adcResult[IDX_ANA_CLIFF_2];
  cliff[3]= cliff_LPFa*cliff[3] +  cliff_LPFb*adcResult[IDX_ANA_CLIFF_3];

  if (!first_config)
  {
    stat.cliff_range[0]=cliff[0]-cfg.cliff_zero[0];
    stat.cliff_range[1]=cliff[1]-cfg.cliff_zero[1];
    stat.cliff_range[2]=cliff[2]-cfg.cliff_zero[2];
    stat.cliff_range[3]=cliff[3]-cfg.cliff_zero[3];
    at_cliff[0] = stat.cliff_range[0]<cfg.cliff_thresh; //Neg is dropoff
    at_cliff[1] = stat.cliff_range[1]<cfg.cliff_thresh;
    at_cliff[2] = stat.cliff_range[2]<cfg.cliff_thresh;
    at_cliff[3] = stat.cliff_range[3]<cfg.cliff_thresh;
  }
   uint8_t cliff_last = state_cliff_event;
  state_cliff_event = state_cliff_event || (at_cliff[0] ||at_cliff[1] ||at_cliff[2] ||at_cliff[3])&& cfg.stop_at_cliff; //Remains true until reset
  if (!cliff_last && state_cliff_event)
    do_beep(BEEP_ID_SINGLE_SHORT);

  voltage = voltage * voltage_LPFa +  voltage_LPFb* adcResult[IDX_ANA_V_BATT];
  current = current * current_LPFa +  current_LPFb* adcResult[IDX_ANA_CURRENT];
  temp =    temp *    temp_LPFa +     temp_LPFb*    adcResult[IDX_ANA_TEMP];


  if(stat.imu.bump>cfg.bump_thresh)
    stat.bump_event_cnt++;
  
  
  stat.voltage=voltage;
  stat.current=current;
  stat.temp=temp;

  stat.state=0;
  stat.state = at_cliff[0] ? stat.state|STATE_AT_CLIFF_0 : stat.state;
  stat.state = at_cliff[1] ? stat.state|STATE_AT_CLIFF_1 : stat.state;
  stat.state = at_cliff[2] ? stat.state|STATE_AT_CLIFF_2 : stat.state;
  stat.state = at_cliff[3] ? stat.state|STATE_AT_CLIFF_3 : stat.state;
  stat.state= state_cliff_event ? stat.state|STATE_CLIFF_EVENT : stat.state;
  stat.state= state_runstop_event? stat.state|STATE_RUNSTOP_EVENT : stat.state;
  stat.state= state_fan_on ? stat.state|STATE_FAN_ON : stat.state;
  stat.state= state_buzzer_on ? stat.state|STATE_BUZZER_ON : stat.state;
  stat.state= state_low_voltage_alert ? stat.state|STATE_LOW_VOLTAGE_ALERT : stat.state;
  stat.state= state_high_current_alert ? stat.state|STATE_HIGH_CURRENT_ALERT : stat.state;
  stat.state= state_over_tilt_alert ? stat.state|STATE_OVER_TILT_ALERT : stat.state;
  
  memcpy((uint8_t *) (&stat_out),(uint8_t *) (&stat),sizeof(Pimu_Status));


  stat.debug=TC5->COUNT16.COUNT.reg;
}

///////////////RUNSTOP SWITCH LOGIC //////////////////////////

//Runstop is pulled high
//Depressing button pulls line low
int depressed_last=0;

void step_runstop() 
{
  int button_depressed=(digitalRead(RUNSTOP_SW)==0);

  if (!depressed_last && button_depressed  || alert_trigger_runstop) //button pushed
  {
    
    if (runstop_mode==MODE_RUNNING)
    {
      runstop_mode=MODE_RUNSTOP_ACTIVE;
      do_beep(BEEP_ID_SINGLE_SHORT);
    }
    else if (runstop_mode==MODE_RUNSTOP_ACTIVE && !alert_trigger_runstop) //already triggered, not an alert, so must be a start of a reset
    {
      t_low=get_elapsed_time_ms();
      runstop_mode=MODE_RESET_ACTIVE;
    }
    alert_trigger_runstop=false;
  }
  else 
  {
    if (runstop_mode==MODE_RESET_ACTIVE && button_depressed && get_elapsed_time_ms()-t_low>2000)
    {
      runstop_mode=MODE_RUNNING;
      do_beep(BEEP_ID_SINGLE_SHORT);
    }
    else
    if(!button_depressed && runstop_mode==MODE_RESET_ACTIVE) //Not held down long enough
    {
      runstop_mode=MODE_RUNSTOP_ACTIVE;
    }
  }
  depressed_last=button_depressed;
  
  if(runstop_mode==MODE_RUNNING)
    state_runstop_event=0;
  else
    state_runstop_event=cfg.stop_at_runstop;
  
}

////////////////////// Beep ///////////////////////////////////////////
//Step is called at 60hz, so x60
#define BEEP_4000MS 240
#define BEEP_2000MS 120
#define BEEP_1000MS 60
#define BEEP_500MS 30
#define BEEP_250MS 15

int beep1_on_cnt=0;
int beep1_off_cnt=0;
int beep2_on_cnt=0;

#define BEEP_ID_SINGLE_SHORT 1
#define BEEP_ID_SINGLE_LONG 2
#define BEEP_ID_DOUBLE_SHORT 3
#define BEEP_ID_DOUBLE_LONG 4


void do_beep(int bid)
{
    switch(bid)
    {
      case BEEP_ID_OFF:
        beep1_on_cnt=0;
        beep1_off_cnt=0;
        beep2_on_cnt=0;
        break;
      case BEEP_ID_SINGLE_SHORT:
        beep1_on_cnt=BEEP_250MS;
        beep1_off_cnt=0;
        beep2_on_cnt=0;
        break;
      case BEEP_ID_SINGLE_LONG:
        beep1_on_cnt=BEEP_500MS;
        beep1_off_cnt=0;
        beep2_on_cnt=0;
        break;
      case BEEP_ID_DOUBLE_SHORT:
        beep1_on_cnt=BEEP_250MS;
        beep1_off_cnt=BEEP_500MS;
        beep2_on_cnt=BEEP_250MS;
        break;
      case BEEP_ID_DOUBLE_LONG:
        beep1_on_cnt=BEEP_500MS;
        beep1_off_cnt=BEEP_500MS;
        beep2_on_cnt=BEEP_500MS;
        break;
      default:
        beep1_on_cnt=0;
        beep1_off_cnt=0;
        beep2_on_cnt=0;
        break;
    };
}
void step_beep()
{
    if (beep1_on_cnt>0)
    {
      digitalWrite(BUZZER, HIGH);
      beep1_on_cnt=max(0,beep1_on_cnt-1);
    }
    if (beep1_on_cnt==0 && beep1_off_cnt>0)
    {
      beep1_off_cnt=max(0,beep1_off_cnt-1);
      digitalWrite(BUZZER, LOW);
    }
    if (beep1_on_cnt==0 &&  beep1_off_cnt==0 && beep2_on_cnt>0)
    {
      digitalWrite(BUZZER, HIGH);
      beep2_on_cnt=max(0,beep2_on_cnt-1);
    }
    if (beep1_on_cnt==0 && beep1_off_cnt==0 && beep2_on_cnt==0)
    {
      digitalWrite(BUZZER, LOW);
    }
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
}

void disableTCInterrupts() {  //disables the controller interrupt ("closed loop mode")
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
}

void setupTimer5() {  // configure the controller interrupt
  
  // Enable GCLK for TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)
  
  TC5->COUNT16.CC[0].reg = TC5_COUNT_PER_CYCLE; //Value to count up to
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  NVIC_SetPriority(TC5_IRQn, 1);              //Set interrupt priority

  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable TC
    enableTCInterrupts();
}



//////////////////////////////////////

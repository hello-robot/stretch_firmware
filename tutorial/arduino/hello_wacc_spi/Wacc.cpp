/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc Spi

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#include <Transport.h>

#include <Arduino.h>
#include "Wacc.h"
#include "Accel.h"


//////////////////////////////////////
Wacc_Config cfg, cfg_in;
Wacc_Command cmd, cmd_in;
Wacc_Status stat, stat_out;
Wacc_Board_Info board_info;

float accel_LPFa = 0.0;
float accel_LPFb = 1.0;
float ana_LPFa = 0.0;
float ana_LPFb = 1.0;
float accel_gravity_scale = 1.0;
bool dirty_config = false;
bool dirty_command = false;

////////// SPI ////////////////
#include <SPI.h>

uint8_t spi_out[9];   //SPI data out
uint8_t spi_in[9];    //SPI data in
uint8_t ds_spi_cnt=0; //Down sample counter
float FS_SPI = 10;    //Rate to run transactions (Hz)

void spiTransaction()
{
  digitalWrite(HEADER_SPI_SS, LOW);
  SPI.transfer('X'); //Mark start of transaction
  memcpy(spi_out, (uint8_t *) (&cmd.calc), sizeof(Calc_Command));
  for (uint8_t idx = 0; idx < 9; idx++)
    spi_in[idx] = SPI.transfer(spi_out[idx]);
  digitalWrite(HEADER_SPI_SS, HIGH);
  memcpy((uint8_t *) (&stat.calc),spi_in+1,  sizeof(Calc_Status));
}
////////////////////////////////////
void setupTimer5();
float FS_CTRL = 700; //Run Wacc Controller at 700hz, IMU update at 70hz
float FS_ACC = 70;

void toggle_led(int rate_ms);

void setupWacc() {
  memset(&cfg, 0, sizeof(Wacc_Config));
  memset(&cfg_in, 0, sizeof(Wacc_Config));
  memset(&cmd, 0, sizeof(Wacc_Command));
  memset(&cmd_in, 0, sizeof(Wacc_Command));
  memset(&stat, 0, sizeof(Wacc_Status));
  memset(&stat_out, 0, sizeof(Wacc_Status));
  memcpy(&(board_info.board_version), BOARD_VERSION, min(20, strlen(BOARD_VERSION)));
  memcpy(&(board_info.firmware_version), FIRMWARE_VERSION, min(20, strlen(FIRMWARE_VERSION)));

  ////////// SPI ////////////////
  SPISettings settingsA(100000, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settingsA);
  ///////////////////////////////
  
  setupTimer5();
}


void handleNewRPC()
{
  switch (rpc_in[0])
  {
    case RPC_SET_WACC_COMMAND:
      memcpy(&cmd_in, rpc_in + 1, sizeof(Wacc_Command));
      rpc_out[0] = RPC_REPLY_WACC_COMMAND;
      num_byte_rpc_out = 1;
      dirty_command = true;
      break;
    case RPC_SET_WACC_CONFIG:
      memcpy(&cfg_in, rpc_in + 1, sizeof(Wacc_Config));
      rpc_out[0] = RPC_REPLY_WACC_CONFIG;
      num_byte_rpc_out = 1;
      dirty_config = true;
      break;
    case RPC_GET_WACC_STATUS:
      rpc_out[0] = RPC_REPLY_WACC_STATUS;
      memcpy(rpc_out + 1, (uint8_t *) (&stat_out), sizeof(Wacc_Status)); //Collect the status data
      num_byte_rpc_out = sizeof(Wacc_Status) + 1;
      break;
    case RPC_GET_WACC_BOARD_INFO:
      rpc_out[0] = RPC_REPLY_WACC_BOARD_INFO;
      memcpy(rpc_out + 1, (uint8_t *) (&board_info), sizeof(Wacc_Board_Info)); //Collect the status data
      num_byte_rpc_out = sizeof(Wacc_Board_Info) + 1;
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
uint8_t board_reset_cnt = 0;
uint8_t ds_cnt = 0;


//Called at 700hz
void stepWaccController()
{
  stat.timestamp = micros();

  if (dirty_config)
  {
    if (cfg_in.ana_LPF != cfg.ana_LPF) //Effort filter
    {
      ana_LPFa = exp(cfg_in.ana_LPF * -2 * 3.14159 / FS_CTRL); // z = e^st pole mapping
      ana_LPFb = (1.0 - ana_LPFa);
    }
    if (cfg_in.accel_LPF != cfg.accel_LPF) //Effort filter
    {
      accel_LPFa = exp(cfg_in.accel_LPF * -2 * 3.14159 / FS_ACC); // z = e^st pole mapping
      accel_LPFb = (1.0 - accel_LPFa);
    }
    if (cfg_in.accel_range_g != cfg.accel_range_g)
      setAccelRange(cfg_in.accel_range_g);

    if ((cfg_in.accel_single_tap_dur != cfg.accel_single_tap_dur) || (cfg_in.accel_single_tap_thresh != cfg.accel_single_tap_thresh))
      setSingleTapSensitivity(cfg_in.accel_single_tap_dur, cfg_in.accel_single_tap_thresh );

    memcpy(&cfg, &cfg_in, sizeof(Wacc_Config));
    accel_gravity_scale = cfg.accel_gravity_scale;
    dirty_config = false;
  }

  if (dirty_command)
  {
    memcpy(&cmd, &cmd_in, sizeof(Wacc_Command));
    dirty_command = false;
    if (cmd.trigger & TRIGGER_BOARD_RESET)
    {
      board_reset_cnt = 100;
    }
  }

  if (board_reset_cnt)
  {
    board_reset_cnt--; //Countdown to allow time for RPC to finish up
    if (board_reset_cnt == 0)
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

  ds_cnt++; //Downsample to 70Hz
  if (ds_cnt >= (FS_CTRL / FS_ACC))
  {
    stepAccel();
    stat.ax = accel_gravity_scale * (stat.ax * accel_LPFa + accel_LPFb * ax);
    stat.ay = accel_gravity_scale * (stat.ay * accel_LPFa + accel_LPFb * ay);
    stat.az = accel_gravity_scale * (stat.az * accel_LPFa + accel_LPFb * az);
    ds_cnt = 0;
  }

  ////////// SPI ////////////////
  if (ds_spi_cnt++ >= (FS_CTRL / FS_SPI))
  {
    ds_spi_cnt = 0;
    spiTransaction();
  }
  ////////////////////////////////////
  
  stat.a0 = stat.a0 * ana_LPFa +  ana_LPFb * analogRead(A0);
  stat.d0 = digitalRead(D0);
  stat.d1 = digitalRead(D1);
  stat.d2 = cmd.d2;
  stat.d3 = cmd.d3;
  stat.single_tap_count = single_tap_count;
  stat.state = 0;
  stat.debug = accel_debug;
 
  memcpy((uint8_t *) (&stat_out), (uint8_t *) (&stat), sizeof(Wacc_Status));
}
//////////////////////////////////////////////////////
bool led_on = false;
unsigned long t_toggle_last = 0;

void toggle_led(int rate_ms)
{
  unsigned long t = millis();
  if (t - t_toggle_last > rate_ms)
  {
    t_toggle_last = t;
    if (!led_on)
    {
      digitalWrite(LED, HIGH);  //LED
    }
    else
    {
      digitalWrite(LED, LOW);  //LED
    }
    led_on = !led_on;
  }
}

////////////////////// Timer5 /////////////////////////////////////////


void TC5_Handler() {

  if (TC5->COUNT16.INTFLAG.bit.OVF == 1)
  {
    stepWaccController();
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

  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CC[0].reg = (int)( round(48000000 / 64 / FS_CTRL)); //Count up to 6400 / rate
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

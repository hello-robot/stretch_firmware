
/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
    
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html
  
  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/


#include <Arduino.h>
#include "Common.h"
#include "Transport.h"
#include "Pimu.h"
#include "IMU.h"


void setup()        // This code runs once at startup
{     
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(FAN_FET, OUTPUT);
  pinMode(IMU_RESET,OUTPUT);
  pinMode(RUNSTOP_SW, INPUT);
  //pinMode(RUNSTOP_SW, INPUT_PULLUP);
  pinMode(RUNSTOP_LED, OUTPUT);
  pinMode(RUNSTOP_M0, OUTPUT);
  pinMode(RUNSTOP_M1, OUTPUT);
  pinMode(RUNSTOP_M2, OUTPUT);
  pinMode(RUNSTOP_M3, OUTPUT);

  digitalWrite(LED, LOW);
  digitalWrite(RUNSTOP_LED, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(IMU_RESET, HIGH);
  digitalWrite(FAN_FET, LOW);

  setupIMU();
  SerialUSB.begin(2000000);// When using SerialUSB the baudrate is ignored since running at USB rate  
  setupPimu();              // configure controller interrupt
  setupTransport();
  //do_beep(BEEP_ID_SINGLE_SHORT);

  
}

void loop()
{    
  stepPimuRPC();

  
}

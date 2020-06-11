/*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc I2C

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html      

  Copyright (c) 2020 by Hello Robot Inc. 
  --------------------------------------------------------------
*/

#include <Arduino.h>
#include "Common.h"
#include "Transport.h"
#include "Wacc.h"
#include "Accel.h"


void setup()        // This code runs once at startup
{     
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
  setupAccel();
  SerialUSB.begin(2000000);
  setupWacc();          
  setupTransport();
}

void loop()
{
  stepWaccRPC();
}

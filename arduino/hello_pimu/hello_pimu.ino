
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
  setupBoardVariants();
  setupIMU();
  SerialUSB.begin(2000000); 
  setupPimu();              
  setupTransport();
}

void loop()
{    
  stepPimuRPC();
}

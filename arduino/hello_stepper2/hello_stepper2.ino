
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


#include "Utils.h"
#include "Parameters.h"
#include "State.h"
#include "analogFastWrite.h"

#ifdef HELLO
#include "HelloController.h"
#include "Transport.h"
#endif

//////////////////////////////////////
/////////////////SETUP////////////////
//////////////////////////////////////



void setup()        // This code runs once at startup
{                         

  
  digitalWrite(ledPin,HIGH);        // turn LED on 
  setupPins();                      // configure pins
  setupTCInterrupts();              // configure controller interrupt

  SerialUSB.begin(2000000); 
  
#ifndef HELLO  
  //delay(3000);                      // This delay seems to make it easier to establish a connection when the Mechaduino is configured to start in closed loop mode.  
  serialMenu();     // Prints menu to serial monitor
#else
  setupBoardVariants();  
#endif
 
  setupSPI();                       // Sets up SPI for communicating with encoder
  digitalWrite(ledPin,LOW);         // turn LED off 
  
#ifdef HELLO
    setupTransport();
    setupHelloController();
    setMotorDecay(0); //Sets motor decay to mixed decay
    setTOFF(0);
    enableMotorDrivers(); //Turn on now that gains are loaded
    enableTCInterrupts();  //Always be running the loop
#endif
setupWDT(WDT_TIMEOUT_PERIOD);

}
  


//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////


void loop()                 // main loop
{
#ifdef HELLO
// Flash LED fast when in menu mode, slow in RPC mode
if (hello_interface)
{
  resetWDT();
  stepHelloControllerRPC();
}
else
{
  disableWDT();
  serialCheck();
  toggle_led(200);
}
#else
  disableWDT();
  serialCheck();              //must have this execute in loop for serial commands to function
#endif

}

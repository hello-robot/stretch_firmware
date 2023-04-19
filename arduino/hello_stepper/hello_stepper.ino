
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
  setupMGInterrupts();
#endif
 
  setupSPI();                       // Sets up SPI for communicating with encoder
  digitalWrite(ledPin,LOW);         // turn LED off 
  
  // spot check some of the lookup table to decide if it has been filled in
  //if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0)
  //  SerialUSB.println("WARNING: Lookup table is empty! Run calibration");

  // Uncomment the below lines as needed for your application.
  // Leave commented for initial calibration and tuning.
  
  //    configureStepDir();           // Configures setpoint to be controlled by step/dir interface
  //    configureEnablePin();         // Active low, for use wath RAMPS 1.4 or similar
  //     enableTCInterrupts();         // uncomment this line to start in closed loop 
  //    mode = 'x';                   // start in position mode
#ifdef HELLO
    setupTransport();
    setupHelloController();
    enableTCInterrupts();  //Always be running the loop
    enableMGInterrupts();
    enableMotorDrivers(); //Turn on now that gains are loaded
#endif

}
  


//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////


void loop()                 // main loop
{

#ifdef HELLO
//Flash LED fast when in menu mode, slow in RPC mode
if (hello_interface)
{
  //stepHelloControllerRPC();
  delay(1);
}
else
{
  serialCheck();
  toggle_led(200);
}
#else
  serialCheck();              //must have this execute in loop for serial commands to function
#endif


}

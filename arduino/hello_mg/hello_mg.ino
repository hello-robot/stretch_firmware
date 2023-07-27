  /*
  -------------------------------------------------------------
  Hello Robot - Hello Wacc

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html      

  Copyright (c) 2020 by Hello Robot Inc. 
  --------------------------------------------------------------
*/

#include <Arduino.h>
#include "MotionGenerator.h"



//#include "Streaming.h"

char * _print_base(char *buf, uint64_t n, int base) {
  char *str = &buf[(64+1) - 1]; // handle printing bits

  *str = '\0'; // work from least significant digit to most
  do {
    unsigned digit = n % base;
    *--str = digit < 10 ? digit + '0' : digit + 'A' - 10;
  } while ( (n /= base) );

  return (str);
};

char work1[64+1];
  
char * printll(int64_t ldata, int base = DEC)
{
  char *ptr = work1;
  
  if ( base == DEC && ldata < 0 ) {
    ldata = -ldata;
    ptr = _print_base(&ptr[1], (uint64_t)ldata, base);
    *--ptr = '-';
  }
  else {
    ptr = _print_base(ptr, (uint64_t)ldata, base);
  }

  return ptr;
}


MotionGenerator mg;

void print_int64_t(int64_t x)
{
  char * c=printll(x);
  SerialUSB.print(c);
}
void setup()        // This code runs once at startup
{     

  SerialUSB.begin(2000000);
  
}

int itr=0;
void do_print()
{
    SerialUSB.print("\n---"); SerialUSB.print(itr++); SerialUSB.print("---"); 

    SerialUSB.print("\n t: ");  print_int64_t(mg.t);
    SerialUSB.print("\ndt: ");  print_int64_t(mg.dt);
    
    SerialUSB.print("\nmaxVel: ");  print_int64_t(mg.maxVel);
    SerialUSB.print("\nmaxAcc: ");  print_int64_t(mg.maxAcc);

    SerialUSB.print("\npos: ");  print_int64_t(mg.pos);
    SerialUSB.print("\nvel: ");  print_int64_t(mg.vel);
    SerialUSB.print("\nacc: ");  print_int64_t(mg.acc);
    SerialUSB.print("\noldPos: ");  print_int64_t(mg.oldPos);
    SerialUSB.print("\noldPosRef: ");  print_int64_t(mg.oldPosRef);
    SerialUSB.print("\noldVel: ");  print_int64_t(mg.oldVel);
  
    SerialUSB.print("\ndBrk: ");  print_int64_t(mg.dBrk);
    SerialUSB.print("\ndAcc: ");  print_int64_t(mg.dAcc);
    SerialUSB.print("\ndVel: ");  print_int64_t(mg.dVel);
    SerialUSB.print("\ndDec: ");  print_int64_t(mg.dDec);
    SerialUSB.print("\ndTot: ");  print_int64_t(mg.dTot);
  
    SerialUSB.print("\ntBrk: ");  print_int64_t(mg.tBrk);
    SerialUSB.print("\ntAcc: ");  print_int64_t(mg.tAcc);
    SerialUSB.print("\ntVel: ");  print_int64_t(mg.tVel);
    SerialUSB.print("\ntDec: ");  print_int64_t(mg.tDec);
   
    SerialUSB.print("\nvelSt: ");   print_int64_t(mg.velSt);

}


void do_once()
{
  itr=0;
  mg.setMaxVelocity(25.0);
  mg.setMaxAcceleration(15.0);
  mg.safe_switch_on(0.0,0.0);
  SerialUSB.print("\n--START-");
  do_print();
  float x=mg.update(100.0);
  do_print();
  x=mg.update(100.0);
  do_print();
  
}

void loop()
{
  delay(1000);
  do_once();
  
}

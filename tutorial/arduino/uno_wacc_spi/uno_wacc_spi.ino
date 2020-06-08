/*
  -------------------------------------------------------------
  Hello Robot - Uno WACC SPI tutorial sketch

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html      

  Copyright (c) 2020 by Hello Robot Inc. 
  --------------------------------------------------------------
*/

#include <Arduino.h>
#include "SPI.h"

struct __attribute__ ((packed)) Calc_Command{
  float var1;
  float var2;
  uint8_t op;
};

struct __attribute__ ((packed)) Calc_Status{
  float result;
};


float my_calc(uint8_t op, float var1, float var2)
{
  if (op == 0)
    return var1 + var2;
  if (op == 1)
    return var1 * var2;
  if (op == 2)
    return var1 / var2;
  return 0;
}

void setup (void)
{
 pinMode(MISO, OUTPUT);
 SPCR |= _BV(SPE);
 SPI.attachInterrupt(); 
}  


// SPI interrupt routine
uint8_t buf_in[9];
uint8_t buf_in_idx=0;
uint8_t buf_out[4];
uint8_t buf_out_idx=0;

Calc_Command cmd;
Calc_Status stat;

ISR (SPI_STC_vect)
{
  byte c = SPDR;

  if ((char)c=='X')
  {
    buf_in_idx=0;
    buf_out_idx=0;
    SPDR=0;
  }
  else
  {
     buf_in[buf_in_idx++]=c;
     if(buf_out_idx<sizeof(Calc_Status))
      SPDR=buf_out[buf_out_idx++];
     else
      SPDR=0;
  }

  if(buf_in_idx==sizeof(Calc_Command))
  {
    memcpy((uint8_t *) (&cmd), buf_in, sizeof(Calc_Command));
    stat.result=my_calc(cmd.op,cmd.var1,cmd.var2);
    memcpy(buf_out, (uint8_t *) (&stat), sizeof(Calc_Status));
    buf_in_idx=0;
    buf_out_idx=0;
  }
 
}


void loop (void)
{
   
}  // end of loop

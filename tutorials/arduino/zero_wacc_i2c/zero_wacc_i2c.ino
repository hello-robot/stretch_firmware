/*
  -------------------------------------------------------------
  Hello Robot - Hello Zero I2C Slave

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html      

  Copyright (c) 2020 by Hello Robot Inc. 
  --------------------------------------------------------------
*/


#include <Arduino.h>
#include <Wire.h>

Calc_Command cmd;
Calc_Status stat;
uint8_t buf_in[9];
uint8_t buf_in_idx=0;
uint8_t buf_out[4];
uint8_t buf_out_idx=0;


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

void setup()
{
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
}

void loop()
{
  delay(100);
}

//Send back the status upon request
void requestEvent() {
  memcpy(buf_out, (uint8_t *) (&stat),  sizeof(Calc_Status));
  for (int i=0;i<sizeof(Calc_Status);i++)
       Wire.write(buf_out[i]);     
}

//Receive a command and do calculation
void receiveEvent(int bytes)
{
  while (Wire.available()) 
  {
    uint8_t x = Wire.read();       
    buf_in[buf_in_idx++]=x;
    if(buf_in_idx==sizeof(Calc_Command))
    {
      buf_in_idx=0;
      memcpy((uint8_t *) (&cmd), buf_in, sizeof(Calc_Command));
      stat.result=my_calc(cmd.op,cmd.var1,cmd.var2);
    }
  }
}

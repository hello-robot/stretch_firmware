/* Optical Flow Sensor driver
 * Copyright (c) 2022 RalBra
 * forked from Bitcraze AB -- https://github.com/bitcraze/Bitcraze_PMW3901
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY);
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM);
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Optical_Flow_Sensor.h"

#include <SPI.h>

boolean Optical_Flow_Sensor::begin(void) {
    // Setup SPI port
    SPI.begin();
    pinMode(_cs, OUTPUT);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));

    // Make sure the SPI bus is reset
    digitalWrite(_cs, HIGH);
    delay(1);
    digitalWrite(_cs, LOW);
    delay(1);
    digitalWrite(_cs, HIGH);
    delay(1);

    SPI.endTransaction();

    // Power on reset
    registerWrite(0x3A, 0x5A);
    delay(5);
    // Test the SPI communication, checking chipId and inverse chipId
    uint8_t chipId = registerRead(0x00);
    uint8_t dIpihc = registerRead(0x5F);

    if (chipId != 0x49 && dIpihc != 0xB8) return false;

    // Reading the motion registers one time
    registerRead(0x02);
    registerRead(0x03);
    registerRead(0x04);
    registerRead(0x05);
    registerRead(0x06);
    delay(1);

//  enableFrameBuffer();


    secrectSauce();
    if(_sensor == PMW3901)
      initRegistersPMW3901();
    else if(_sensor == PAA5100)
      initRegistersPAA5100();
    else
      return false;

    int product_id = registerRead(0x00);
    int revision = registerRead(0x01);
    if (product_id != 0x49 or revision != 0x00)
        return false;
    return true;
}

// Functional access

void Optical_Flow_Sensor::readMotionCount(int16_t *deltaX, int16_t *deltaY)
{
  registerRead(0x02);
  *deltaX = ((int16_t)registerRead(0x04) << 8) | registerRead(0x03);
  *deltaY = ((int16_t)registerRead(0x06) << 8) | registerRead(0x05);
}

void Optical_Flow_Sensor::enableFrameBuffer()
{

  registerWrite(0x7F, 0x07);  //Magic frame readout registers
  registerWrite(0x41, 0x1D);
  registerWrite(0x4C, 0x00);
  registerWrite(0x7F, 0x08);
  registerWrite(0x6A, 0x38);
  registerWrite(0x7F, 0x00);
  registerWrite(0x55, 0x04);
  registerWrite(0x40, 0x80);
  registerWrite(0x4D, 0x11);

  registerWrite(0x70, 0x00);   //More magic? 
  registerWrite(0x58, 0xFF);

  int temp = registerRead(0x58); //Read status register 
  int check = temp>>6; //rightshift 6 bits so only top two stay

  while(check == 0x03){ //while bits aren't set denoting ready state
    temp = registerRead(0x58); //keep reading and testing 
    check = temp>>6; //shift again 
  }  
  delayMicroseconds(50);
}

void Optical_Flow_Sensor::readFrameBuffer(char *FBuffer)
{
  int count = 0;
  uint8_t a; //temp value for reading register
  uint8_t b; //temp value for second register
  uint8_t hold; //holding value for checking bits
  uint8_t mask = 0x0c; //mask to take bits 2 and 3 from b
  uint8_t pixel = 0; //temp holding value for pixel

  for (int ii = 0; ii < 1225; ii++) { //for 1 frame of 1225 pixels (35*35)
    //check status bits 6 and 7
    //if 01 move upper 6 bits into temp value
    //if 00 or 11, reread
    //else lower 2 bits into temp value
    a = registerRead(0x58); //read register
    hold = a >> 6; //right shift to leave top two bits for ease of check.

    while ((hold == 0x03) || (hold == 0x00)) { //if data is either invalid status
      a = registerRead(0x58); //reread loop
      hold = a >> 6;
    }
    if (hold == 0x01) { //if data is upper 6 bits
      b = registerRead(0x58); //read next set to get lower 2 bits
      pixel = a; //set pixel to a
      pixel = pixel << 2; //push left to 7:2
      pixel += (b & mask); //set lower 2 from b to 1:0
      FBuffer[count++] = pixel; //put temp value in fbuffer array
      //delayMicroseconds(100);
    }
    else {}
  }
  registerWrite(0x70, 0x00);   //More magic? 
  registerWrite(0x58, 0xFF);

  int temp = registerRead(0x58); //Read status register 
  int check = temp>>6; //rightshift 6 bits so only top two stay

  while(check == 0x03){ //while bits aren't set denoting ready state
    temp = registerRead(0x58); //keep reading and testing 
    check = temp>>6; //shift again 
  }  
}

// Low level register access
void Optical_Flow_Sensor::registerWrite(uint8_t reg, uint8_t value) {
  reg |= 0x80u;

  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));

  digitalWrite(_cs, LOW);

  delayMicroseconds(50);
  SPI.transfer(reg);
  SPI.transfer(value);
  delayMicroseconds(50);

  digitalWrite(_cs, HIGH);

  SPI.endTransaction();

  delayMicroseconds(200);
}

uint8_t Optical_Flow_Sensor::registerRead(uint8_t reg) {
  reg &= ~0x80u;

  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));

  digitalWrite(_cs, LOW);

  delayMicroseconds(50);
  SPI.transfer(reg);
  delayMicroseconds(50);
  uint8_t value = SPI.transfer(0);
  delayMicroseconds(100);

  digitalWrite(_cs, HIGH);

  //delayMicroseconds(200);

  SPI.endTransaction();

  return value;
}

//here happens magic, don't ask...
void Optical_Flow_Sensor::secrectSauce()
{
  
    registerWrite(0x7f, 0x00);
    registerWrite(0x55, 0x01);
    registerWrite(0x50, 0x07);
    registerWrite(0x7f, 0x0e);
    registerWrite(0x43, 0x10);

    int temp = registerRead(0x67);
    if(temp & 0b10000000){
        registerWrite(0x48, 0x04);
    }
    else{
        registerWrite(0x48, 0x02);
    }
            
    registerWrite(0x7f, 0x00);
    registerWrite(0x51, 0x7b);
    registerWrite(0x50, 0x00);
    registerWrite(0x55, 0x00);
    registerWrite(0x7f, 0x0E);
    
    temp = registerRead(0x73);
    if (temp == 0x00){
        int c1 = registerRead(0x70);
        int c2 = registerRead(0x71);
        if (c1 <= 28)
            c1 += 14;
        if (c1 > 28)
            c1 += 11;
        c1 = max(0, min(0x3F, c1));
        c2 = (c2 * 45); // 100
        registerWrite(0x7f, 0x00);
        registerWrite(0x61, 0xad);
        registerWrite(0x51, 0x70);
        registerWrite(0x7f, 0x0e);
        registerWrite(0x70, c1);
        registerWrite(0x71, c2);
    }
}

// Performance optimisation registers
void Optical_Flow_Sensor::initRegistersPMW3901()
{
  registerWrite(0x7F, 0x00);
  registerWrite(0x61, 0xAD);
  registerWrite(0x7F, 0x03);
  registerWrite(0x40, 0x00);
  registerWrite(0x7F, 0x05);

  registerWrite(0x41, 0xB3);
  registerWrite(0x43, 0xF1);
  registerWrite(0x45, 0x14);
  registerWrite(0x5B, 0x32);
  registerWrite(0x5F, 0x34);
  registerWrite(0x7B, 0x08);
  registerWrite(0x7F, 0x06);
  registerWrite(0x44, 0x1B);
  registerWrite(0x40, 0xBF);
  registerWrite(0x4E, 0x3F);
  registerWrite(0x7F, 0x08);
  registerWrite(0x65, 0x20);
  registerWrite(0x6A, 0x18);

  registerWrite(0x7F, 0x09);
  registerWrite(0x4F, 0xAF);
  registerWrite(0x5F, 0x40);
  registerWrite(0x48, 0x80);
  registerWrite(0x49, 0x80);

  registerWrite(0x57, 0x77);
  registerWrite(0x60, 0x78);
  registerWrite(0x61, 0x78);
  registerWrite(0x62, 0x08);
  registerWrite(0x63, 0x50);
  registerWrite(0x7F, 0x0A);
  registerWrite(0x45, 0x60);
  registerWrite(0x7F, 0x00);
  registerWrite(0x4D, 0x11);
  
  registerWrite(0x55, 0x80);
  registerWrite(0x74, 0x1F);
  registerWrite(0x75, 0x1F);
  registerWrite(0x4A, 0x78);
  registerWrite(0x4B, 0x78);
  
  registerWrite(0x44, 0x08);
  registerWrite(0x45, 0x50);
  registerWrite(0x64, 0xFF);
  registerWrite(0x65, 0x1F);
  registerWrite(0x7F, 0x14);
  registerWrite(0x65, 0x60);
  registerWrite(0x66, 0x08);
  registerWrite(0x63, 0x78);
  registerWrite(0x7F, 0x15);
  registerWrite(0x48, 0x58);
  registerWrite(0x7F, 0x07);
  registerWrite(0x41, 0x0D);
  registerWrite(0x43, 0x14);
  
  registerWrite(0x4B, 0x0E);
  registerWrite(0x45, 0x0F);
  registerWrite(0x44, 0x42);
  registerWrite(0x4C, 0x80);
  registerWrite(0x7F, 0x10);
  registerWrite(0x5B, 0x02);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x41);
  registerWrite(0x70, 0x00);

  delay(100);
  registerWrite(0x32, 0x44);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x40);
  registerWrite(0x7F, 0x06);
  registerWrite(0x62, 0xf0);
  registerWrite(0x63, 0x00);
  registerWrite(0x7F, 0x0D);
  registerWrite(0x48, 0xC0);
  registerWrite(0x6F, 0xd5);
  registerWrite(0x7F, 0x00);

  registerWrite(0x5B, 0xa0);
  registerWrite(0x4E, 0xA8);
  registerWrite(0x5A, 0x50);
  registerWrite(0x40, 0x80);
}

 void Optical_Flow_Sensor::initRegistersPAA5100(){
    registerWrite(0x7f, 0x00);
    registerWrite(0x61, 0xad);

    registerWrite(0x7f, 0x03);
    registerWrite(0x40, 0x00);

    registerWrite(0x7f, 0x05);
    registerWrite(0x41, 0xb3);
    registerWrite(0x43, 0xf1);
    registerWrite(0x45, 0x14);

    registerWrite(0x5f, 0x34);
    registerWrite(0x7b, 0x08);
    registerWrite(0x5e, 0x34);
    registerWrite(0x5b, 0x11);
    registerWrite(0x6d, 0x11);
    registerWrite(0x45, 0x17);
    registerWrite(0x70, 0xe5);
    registerWrite(0x71, 0xe5);

    registerWrite(0x7f, 0x06);
    registerWrite(0x44, 0x1b);
    registerWrite(0x40, 0xbf);
    registerWrite(0x4e, 0x3f);

    registerWrite(0x7f, 0x08);
    registerWrite(0x66, 0x44);
    registerWrite(0x65, 0x20);
    registerWrite(0x6a, 0x3a);
    registerWrite(0x61, 0x05);
    registerWrite(0x62, 0x05);

    registerWrite(0x7f, 0x09);
    registerWrite(0x4f, 0xaf);
    registerWrite(0x5f, 0x40);
    registerWrite(0x48, 0x80);
    registerWrite(0x49, 0x80);
    registerWrite(0x57, 0x77);
    registerWrite(0x60, 0x78);
    registerWrite(0x61, 0x78);
    registerWrite(0x62, 0x08);
    registerWrite(0x63, 0x50);

    registerWrite(0x7f, 0x0a);
    registerWrite(0x45, 0x60);

    registerWrite(0x7f, 0x00);
    registerWrite(0x4d, 0x11);
    registerWrite(0x55, 0x80);
    registerWrite(0x74, 0x21);
    registerWrite(0x75, 0x1f);
    registerWrite(0x4a, 0x78);
    registerWrite(0x4b, 0x78);
    registerWrite(0x44, 0x08);

    registerWrite(0x45, 0x50);
    registerWrite(0x64, 0xff);
    registerWrite(0x65, 0x1f);

    registerWrite(0x7f, 0x14);
    registerWrite(0x65, 0x67);
    registerWrite(0x66, 0x08);
    registerWrite(0x63, 0x70);
    registerWrite(0x6f, 0x1c);

    registerWrite(0x7f, 0x15);
    registerWrite(0x48, 0x48);

    registerWrite(0x7f, 0x07);
    registerWrite(0x41, 0x0d);
    registerWrite(0x43, 0x14);
    registerWrite(0x4b, 0x0e);
    registerWrite(0x45, 0x0f);
    registerWrite(0x44, 0x42);
    registerWrite(0x4c, 0x80);

    registerWrite(0x7f, 0x10);
    registerWrite(0x5b, 0x02);

    registerWrite(0x7f, 0x07);
    registerWrite(0x40, 0x41);

    delay(100);

    registerWrite(0x7f, 0x00);
    registerWrite(0x32, 0x00);

    registerWrite(0x7f, 0x07);
    registerWrite(0x40, 0x40);

    registerWrite(0x7f, 0x06);
    registerWrite(0x68, 0xf0);
    registerWrite(0x69, 0x00);

    registerWrite(0x7f, 0x0d);
    registerWrite(0x48, 0xc0);
    registerWrite(0x6f, 0xd5);

    registerWrite(0x7f, 0x00);
    registerWrite(0x5b, 0xa0);
    registerWrite(0x4e, 0xa8);
    registerWrite(0x5a, 0x90);
    registerWrite(0x40, 0x80);
    registerWrite(0x73, 0x1f);

    delay(100);

    registerWrite(0x73, 0x00);
}

void Optical_Flow_Sensor::setLed(bool ledOn)
{
  delay(200);
  registerWrite(0x7f, 0x14);
  registerWrite(0x6f, ledOn ? 0x1c : 0x00);
  registerWrite(0x7f, 0x00);
}
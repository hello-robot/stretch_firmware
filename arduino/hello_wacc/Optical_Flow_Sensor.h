/* Optical Flow Sensor driver
 * Copyright (c) 2022 RalBra
 * forked from Bitcraze AB -- https://github.com/bitcraze/Bitcraze_PMW3901
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __OPTICAL_FLOW_SENSOR_H__
#define __OPTICAL_FLOW_SENSOR_H__

#include "Arduino.h"

#include <stdint.h>

#define PMW3901 1
#define PAA5100 2


class Optical_Flow_Sensor {
public:
  Optical_Flow_Sensor(uint8_t cs_pin, uint8_t sensor) 
    : _cs{cs_pin}, _sensor{sensor}{}

  boolean begin(void);

  void readMotionCount(int16_t *deltaX, int16_t *deltaY);
  void enableFrameBuffer();
  void readFrameBuffer(char *FBuffer);

  void setLed(bool ledOn);

private:
  uint8_t _cs;
  uint8_t _sensor;

  void registerWrite(uint8_t reg, uint8_t value);
  uint8_t registerRead(uint8_t reg);
  void initRegistersPMW3901(void);
  void initRegistersPAA5100(void);
  void secrectSauce(void);
};

#endif //__OPTICAL_FLOW_SENSOR_H__

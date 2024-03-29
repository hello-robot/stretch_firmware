/**

All materials released under the GNU General Public License v3.0 (GNU GPLv3).
https://www.gnu.org/licenses/gpl-3.0.html

Copyright 2020 Hello Robot Inc.


Adapted from
https://github.com/EmanuelFeru/MotionGenerator

 *
 * @author      AerDronix <aerdronix@gmail.com>
 * @web    https://aerdronix.wordpress.com/
 * @version     1.0 
 * @since       2016-12-22
 * 
 Copyright (c) 2016 AerDronix, https://aerdronix.wordpress.com/

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

 
#ifndef __VELOCITYGENERATOR_H__
#define __VELOCITYGENERATOR_H__
 
#include "Arduino.h"

/**
 See motion generator class...
 */

class VelocityGenerator {   
  public: 
 
    VelocityGenerator();
      
    void safe_switch_on(float x, float v);
    void follow(float x, float v);
    void setMaxAcceleration(float a);
    void set_error_max(float e);
    bool isAccelerating();
    bool isMoving();
    float update(float vd, float xa);
    float vel;
    
  private:
 
   
    float e_max;
    float pos;

    float acc;
    float dt;
};
#endif

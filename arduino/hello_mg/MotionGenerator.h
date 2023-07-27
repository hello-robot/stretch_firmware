#ifndef __MOTIONGENERATOR_H__
#define __MOTIONGENERATOR_H__
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
 
#include "Arduino.h"



class MotionGenerator {		
	public:	

		MotionGenerator();
			
		void safe_switch_on(float x,float v);
    void follow(float x,float v);
		float update(float aPosRef);
		void setMaxVelocity(float aMaxVel);
		void setMaxAcceleration(float aMaxAcc);
		bool isAccelerating();
    bool isMoving();
	
	int64_t t;  //time since last set point
  int64_t dt; //time per cycle
  
	//private:					
		void calculateTrapezoidalProfile(int64_t);
		short int sign(float aVal);		
		
		int64_t maxVel;
		int64_t maxAcc;

  	int64_t pos;
  	int64_t vel;
  	int64_t acc;
  	int64_t oldPos;
  	int64_t oldPosRef;
  	int64_t oldVel;
  
  	int64_t dBrk;
  	int64_t dAcc;
  	int64_t dVel;
  	int64_t dDec;
  	int64_t dTot;
  
  	int64_t tBrk;
  	int64_t tAcc;
  	int64_t tVel;
  	int64_t tDec;
   
    
    

    int64_t velSt;
  	short int signM;      	// 1 = positive change, -1 = negative change
    short int signMacc;
  	bool shape;      	// true = trapezoidal, false = triangular
		
		bool isFinished;	
    bool force_recalc;
};
#endif

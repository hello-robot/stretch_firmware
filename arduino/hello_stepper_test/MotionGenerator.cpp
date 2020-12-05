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

 
#include "MotionGenerator.h"
#include "Parameters.h"
#include "HelloController.h"

 
MotionGenerator::MotionGenerator()
{
  dt=1/FsCtrl;
	maxVel=0;
  maxAcc=0;

  pos     = 0;
  oldPos    = 0;
  oldPosRef   = 0;
  vel     = 0;
  acc     = 0;
  oldVel    = 0;
  
  dBrk    = 0;
  dAcc    = 0;
  dVel    = 0;
  dDec    = 0;
  dTot    = 0;
  
  tBrk    = 0;
  tAcc    = 0;
  tVel    = 0;
  tDec    = 0;
  
  velSt     = 0;
  signM = 1;    // 1 = positive change, -1 = negative change
  signMacc = 1; 
  shape = true;   // true = trapezoidal, false = triangular
  isFinished = false;
  force_recalc=false;
}



void MotionGenerator::setMaxVelocity(float aMaxVel) {
  maxVel = abs(aMaxVel);
  force_recalc=true;
}

void MotionGenerator::setMaxAcceleration(float aMaxAcc) {
  maxAcc = abs(aMaxAcc);
  force_recalc=true;
}


short int MotionGenerator::sign(float aVal) {
  if (aVal < 0)
    return -1;
  else if (aVal > 0)
    return 1;
  else
    return 0;
}


void  MotionGenerator::safe_switch_on(float x,float v)
{
  vel = v;
  pos = x;
  oldPosRef = 0;
  force_recalc=true;
}

void  MotionGenerator::follow(float x,float v)
{
  vel = v;
  pos = x;
  oldPosRef = x;
}

float MotionGenerator::update(float posRef) {	
		
	if (oldPosRef != posRef || force_recalc)  // reference changed
	{
		isFinished = false;
   force_recalc=false;
		// Shift state variables
		oldPosRef = posRef; 
		oldPos = pos; 
		oldVel = vel;
		t = 0;
		
		// Calculate braking time and distance (in case is neeeded)
		tBrk = abs(oldVel) / maxAcc;
		dBrk = tBrk * abs(oldVel) / 2;
		
		// Caculate Sign of motion
		signM = sign(posRef - (oldPos + sign(oldVel)*dBrk));
    signMacc=signM;
		
		if (signM != sign(oldVel))  // means brake is needed
		{
			tAcc = (maxVel / maxAcc);
			dAcc = tAcc * (maxVel / 2);
		}
		else
		{
			tBrk = 0;
			dBrk = 0;
			tAcc = abs(maxVel - abs(oldVel)) / maxAcc;
			dAcc = tAcc * (maxVel + abs(oldVel)) / 2;
     if (maxVel<abs(oldVel))//Need to decel in accel phase:
        signMacc=signMacc*-1;
		}
		
		// Calculate total distance to go after braking
		dTot = abs(posRef - oldPos + signM*dBrk);
		
		tDec = maxVel / maxAcc;
		dDec = tDec * (maxVel) / 2;
		dVel = dTot - (dAcc + dDec);
		tVel = dVel / maxVel;
		
		if (tVel > 0)    // trapezoidal shape
			shape = true;
		else             // triangular shape
		{
			shape = false;
			// Recalculate distances and periods
			if (signM != sign(oldVel))  // means brake is needed
			{
				velSt = sqrt(maxAcc*(dTot));
				tAcc = (velSt / maxAcc);
				dAcc = tAcc * (velSt / 2);
			}
			else
			{
				tBrk = 0;
				dBrk = 0;
				dTot = abs(posRef - oldPos);      // recalculate total distance
				velSt = sqrt(0.5*oldVel*oldVel + maxAcc*dTot);
				tAcc = (velSt - abs(oldVel)) / maxAcc;
				dAcc = tAcc * (velSt + abs(oldVel)) / 2;
			}
			tDec = velSt / maxAcc;
			dDec = tDec * (velSt) / 2;
		}
		
	}
	
	t = t+dt;
	calculateTrapezoidalProfile(posRef);

	return pos;
}

bool MotionGenerator::isAccelerating(){
  return acc!=0;
}
bool MotionGenerator::isMoving(){
  return vel!=0;
}

void MotionGenerator::calculateTrapezoidalProfile(float posRef) {
	
	if (shape)   // trapezoidal shape
	{
		if (t <= (tBrk+tAcc))
		{
			pos = oldPos + oldVel*t + signMacc * 0.5*maxAcc*t*t;
			vel = oldVel + signMacc * maxAcc*t;
			acc = signMacc * maxAcc;
		}
		else if (t > (tBrk+tAcc) && t < (tBrk+tAcc+tVel))
		{
			pos = oldPos + signM * (-dBrk + dAcc + maxVel*(t-tBrk-tAcc));
			vel = signM * maxVel;
			acc = 0;
		}
		else if (t >= (tBrk+tAcc+tVel) && t < (tBrk+tAcc+tVel+tDec))
		{
			pos = oldPos + signM * (-dBrk + dAcc + dVel + maxVel*(t-tBrk-tAcc-tVel) - 0.5*maxAcc*(t-tBrk-tAcc-tVel)*(t-tBrk-tAcc-tVel));
			vel = signM * (maxVel - maxAcc*(t-tBrk-tAcc-tVel));
			acc = - signM * maxAcc;
		}
		else
		{
			pos = posRef;
			vel = 0;
			acc = 0;
			isFinished = true;
		}
	}
	else            // triangular shape
	{
		if (t <= (tBrk+tAcc))
		{
			pos = oldPos + oldVel*t + signM * 0.5*maxAcc*t*t;
			vel = oldVel + signM * maxAcc*t;
			acc = signM * maxAcc;
		}
		else if (t > (tBrk+tAcc) && t < (tBrk+tAcc+tDec))
		{
			pos = oldPos + signM * (-dBrk + dAcc + velSt*(t-tBrk-tAcc) - 0.5*maxAcc*(t-tBrk-tAcc)*(t-tBrk-tAcc));
			vel = signM * (velSt - maxAcc*(t-tBrk-tAcc));
			acc = - signM * maxAcc;
		}
		else
		{
			pos = posRef;
			vel = 0;
			acc = 0;
			isFinished = true;
		}
		
	}

} 

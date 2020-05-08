/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
    
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __IMU_H__
#define  __IMU_H__
#include "Common.h"

extern IMU_Status imu_status;
extern Pimu_Config cfg;
extern float accel_LPFa; 
extern float accel_LPFb;

void setupIMU();         
void stepIMU();
void setIMUCalibration();
bool isIMUOrientationValid();
#endif

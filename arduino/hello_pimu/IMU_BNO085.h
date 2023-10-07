/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
    
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __IMU_BNO085_H__
#define  __IMU_BNO085_H__
#include "Common.h"
#include "SparkFun_BNO080_Arduino_Library.h"

#define IMU_BNO085_RESET (27u) //PA21 27
#define IMU_BNO085_INT (17u) //PA13 17

class IMU_BNO085{
  public:
    bool imu_valid;
    int irq_cnt;
    IMU_BNO085(){}
    void setupIMU();
    void stepIMU(IMU_Status * imu_status);
    void setIMUCalibration();
    bool isIMUOrientationValid();
    volatile bool dirtyRotationVector;
    volatile bool dirtyAccelerometer;
    volatile bool dirtyMagnetometer;
    volatile bool dirtyGyro;
    volatile bool dirtyLinearAcc;
    volatile bool dirtyQuat;
    BNO080 device;
    float accel_max_log[3][8];
    int accel_max_idx=0;

};

extern IMU_BNO085 imu_b;

#endif

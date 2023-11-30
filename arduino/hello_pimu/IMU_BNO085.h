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

#define FRS_WRITE_DATA_REQUEST 0xF6
#define FRS_WRITE_REQUEST 0xF7
#define FRS_SYSTEM_ORIENTATION_ID 0x2D3E
#define FRS_WRITE_RESPONSE 0xF5
#define SYSTEM_QX_ORIENTATION 0x00000000
#define SYSTEM_QY_ORIENTATION 0x00000000
#define SYSTEM_QZ_ORIENTATION 0x2D413CCD //Represents 90 degree rotation on Z axis
#define SYSTEM_QW_ORIENTATION 0x2D413CCD //Represents 90 degree rotation on Z axis


class IMU_BNO085{
  public:
    bool imu_valid;
    bool imu_orientation;
    int irq_cnt;
    IMU_BNO085(){}
    void setupIMU();
    void stepIMU(IMU_Status * imu_status);
    void setIMUCalibration();
    void writeSystemOrientation(bool resetOrientation);
    bool isIMUOrientationValid();
    volatile bool dirtyLinearAcc;
    volatile bool dirtyQuat;
    volatile bool dirtyRotationVector;
    volatile bool dirtyAccelerometer;
    volatile bool dirtyMagnetometer;
    volatile bool dirtyGyro;
    BNO080 device;
    float accel_max_log[3][8];
    int accel_max_idx=0;
};

extern IMU_BNO085 imu_b;

#endif
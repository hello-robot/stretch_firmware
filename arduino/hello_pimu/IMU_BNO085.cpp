/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "IMU_BNO085.h"
#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

IMU_BNO085 imu_b;


// ///////////////////////////////////////////////////////////////////////////////////////////////////

//In SE3, board is oriented such that
//Az+/Mz+: Point down to ground
//Ax+/Mx+: Points in direction of arm reach
//Ay+/My+: Points to back of robot

//Gz+: Base rotates CW
//Gy+: Base rotates side to side
//Gx+: Base rotates front to back

// ///////////////////////////////////////////////////////////////////////////////////////////////////


float ax, ay, az, qx, qy, qz, qw, mx,my,mz; // (qx, qy, qz, qw = i,j,k, real)
byte Accelaccuracy = 0;
float quatRadianAccuracy = 0;
byte quatAccuracy = 0;
byte magAccuracy = 0;

uint32_t systemorientation[4] = {SYSTEM_QX_ORIENTATION, SYSTEM_QY_ORIENTATION, SYSTEM_QZ_ORIENTATION, SYSTEM_QW_ORIENTATION};

//Not working currently, polling instead
void interrupt_handler()
{

  switch (imu_b.device.getReadings())
  {

    case SENSOR_REPORTID_ACCELEROMETER: 
    {
      imu_b.dirtyAccelerometer = 1;
    }
    break;

    case SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR: 
    {
        imu_b.dirtyQuat = 1;
    }
    break;

    case SENSOR_REPORTID_MAGNETIC_FIELD:
    {
      imu_b.dirtyMagnetometer = 1;
    }

    default:
        // Unhandled Input Report
        break;
  }       
}

bool frsWriteResponse()
{
    while (1)
    {
      uint8_t counter = 0;
      while(imu_b.device.receivePacket() == false)
      {
          if (counter++ > 100)
              return false;
          delay(1);
      }

      //0xF5 is Write Response first 2 byte in shtp data is status
      if (imu_b.device.shtpData[0] == FRS_WRITE_RESPONSE)
      {
          uint8_t frsStatus = imu_b.device.shtpData[1];
          if (frsStatus == 0 || frsStatus == 3 || frsStatus == 4)
          {

            return true;
          }
          else
          {
            return false;
          }
          
      }
    }
}

void IMU_BNO085::setIMUCalibration()
{

}

bool IMU_BNO085::isIMUOrientationValid()
{
    return imu_valid;
}

void IMU_BNO085::setupIMU()
{
  //https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf
  //https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/tree/main

 //Reset IMU here (toggle low, high = enabled)(10ms)
  digitalWrite(IMU_BNO085_RESET, LOW); 
  delay(10);//ms  
  digitalWrite(IMU_BNO085_RESET, HIGH);

  Wire.begin();
  imu_valid=device.begin(0x4A, Wire, IMU_BNO085_INT);
  if (imu_valid)  
  {
      Wire.setClock(400000); //Increase I2C data rate to 400kHz
      
      __enable_irq();
      attachInterrupt(digitalPinToInterrupt(IMU_BNO085_INT), interrupt_handler, FALLING);
      
      
      // device.enableLinearAccelerometer(50);  // m/s^2 no gravity, data update every 50 ms
      // device.enableRotationVector(100); //Send data update every 100 ms
  
      device.enableGyroIntegratedRotationVector(50);//RotationVector(10); //Send data update every 10ms
      device.enableAccelerometer(50); //Send data update every 50
      device.enableMagnetometer(50); //Send data update every 50 // cannot be enabled at the same time as RotationVector (will not produce data)
      // device.enableGyro(50); //Send data update every 10ms
      // device.enableTapDetector(50); //Send data update every 50ms
      dirtyRotationVector=false;
      dirtyAccelerometer=false;
      dirtyMagnetometer=false;
      dirtyGyro=false;
      dirtyLinearAcc=false;
      dirtyQuat=false;
      irq_cnt=0;

  }
}



void IMU_BNO085::writeSystemOrientation(bool resetOrientation)
{
    __disable_irq();
    uint16_t length;
    uint32_t *data = systemorientation;
    
    if (resetOrientation == true)
    {
      length = 0;
    }
    else
    {
      length = 4;
    }

    uint8_t offset = 0;
    device.shtpData[0] = FRS_WRITE_REQUEST; //FRS Read Request
    device.shtpData[1] = 0;                   //Reserved
    device.shtpData[2] = (length >> 0) & 0xFF;       //Read Offset LSB
    device.shtpData[3] = (length >> 8) & 0xFF;      //Read Offset MSB
    device.shtpData[4] = (FRS_SYSTEM_ORIENTATION_ID >> 0) & 0xFF;       //FRS Type LSB
    device.shtpData[5] = (FRS_SYSTEM_ORIENTATION_ID >> 8) & 0xFF;       //FRS Type MSB

    //Transmit packet on channel 2, 6 bytes
    device.sendPacket(CHANNEL_CONTROL, 6);

    if (frsWriteResponse() == true)
    {
      while (1)
      {
        for (uint8_t i = 0; i <= 2; i += 2)
        {

          device.shtpData[0] = FRS_WRITE_DATA_REQUEST; //FRS Read Request
          device.shtpData[1] = 0;                        //Reserved
          device.shtpData[2] = (offset+i >> 0) & 0xFF;     //Read Offset LSB
          device.shtpData[3] = (offset+i >> 8) & 0xFF;      //Read Offset MSB
          device.shtpData[4] = (data[i] >> 0) & 0xFF;
          device.shtpData[5] = (data[i]  >> 8) & 0xFF;
          device.shtpData[6] = (data[i] >> 16) & 0xFF;
          device.shtpData[7] = (data[i] >> 24) & 0xFF;
          device.shtpData[8] = (data[i+1] >> 0) & 0xFF;
          device.shtpData[9] = (data[i+1] >> 8) & 0xFF;
          device.shtpData[10] = (data[i+1] >> 16) & 0xFF;
          device.shtpData[11] = (data[i+1] >> 24) & 0xFF;
          device.sendPacket(CHANNEL_CONTROL, 12);
        }
        if (frsWriteResponse() == true)
        {
          break;
        }
        if (frsWriteResponse() == false)
        {
          break;
        }
      }
      __enable_irq();
    }
}


void IMU_BNO085::stepIMU(IMU_Status * imu_status)
{

  if (!imu_valid)
    return;
  if(dirtyQuat)
  {
    device.getQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy);
    imu_status->qx = qx;
    imu_status->qy = qy;
    imu_status->qz = qz;
    imu_status->qw = qw;

    imu_status->gx = device.getFastGyroX();
    imu_status->gy = device.getFastGyroY();
    imu_status->gz = device.getFastGyroZ();

     imu_status->roll=(device.getRoll()) * 180.0 / PI; 
    
    //Roll/Pitch/Yaw Euler
    //Move rollover point out of normal operation
    //IC is upside down so flip
    if (imu_status->roll <0)
      imu_status->roll+=180;
    else
      imu_status->roll-=180;
      
    imu_status->pitch=(device.getPitch()) * 180.0 / PI;;
    imu_status->heading=(device.getYaw()) * 180.0 / PI;

    dirtyQuat=0;
  }
  
  if(dirtyAccelerometer)
  {
    device.getAccel(ax, ay, az, Accelaccuracy);
    imu_status->ax = ax;
    imu_status->ay = ay;
    imu_status->az = az;

    float maxx,maxy,maxz;
    int i;
    accel_max_log[0][accel_max_idx]=imu_status->ax;
    accel_max_log[1][accel_max_idx]=imu_status->ay;
    accel_max_log[2][accel_max_idx]=imu_status->az;

    accel_max_idx++;
    if (accel_max_idx==8)
      accel_max_idx=0;
    
    maxx=0;
    maxy=0;
    maxz=0;
    for (i=0;i<8;i++)
    {
      maxx=max(maxx,abs(accel_max_log[0][i])); //maximum of last 8 filtered readings
      maxy=max(maxy,abs(accel_max_log[1][i]));
      maxz=max(maxz,abs(accel_max_log[2][i]));
    }
    imu_status->bump=maxx*maxx+maxy*maxy+maxz*maxz - 96.17; //magnitude minus gravity

    dirtyAccelerometer = 0;
  }

  if(dirtyMagnetometer)
  {
    device.getMag(mx, my, mz, magAccuracy);
    imu_status->mx = mx;
    imu_status->my = my;
    imu_status->mz = mz;
    dirtyMagnetometer=0;
  }

}
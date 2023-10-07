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


float ax, ay, az, qx, qy, qz, qw; // (qx, qy, qz, qw = i,j,k, real)
byte linAccuracy = 0;
float quatRadianAccuracy = 0;
byte quatAccuracy = 0;

/*
 * Not working currently, polling instead
void interrupt_handler()
{
    imu_b.irq_cnt++;
     switch (imu_b.device.getReadings())
     {
        
        case SENSOR_REPORTID_ACCELEROMETER: {
          
          imu_b.dirtyAccelerometer = 1;
        }
     
     case SENSOR_REPORTID_LINEAR_ACCELERATION: {
        imu_b.dirtyLinearAcc = 1;
      }
      break;
      
      case SENSOR_REPORTID_ROTATION_VECTOR:
      case SENSOR_REPORTID_GAME_ROTATION_VECTOR: {
         imu_b.dirtyQuat = 1;
      }
        break;
        
        case SENSOR_REPORTID_GYROSCOPE: {
          imu_b.dirtyGyro = 1;
        }
        break;

        case SENSOR_REPORTID_MAGNETIC_FIELD: {
          imu_b.dirtyMagnetometer = 1;
        }
        break;

        case SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR:{//SENSOR_REPORTID_ROTATION_VECTOR: {
          imu_b.dirtyRotationVector = 1;
        }
        break;
        default:
           // Unhandled Input Report
           break;
     }
}*/

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
      //attachInterrupt(digitalPinToInterrupt(IMU_BNO085_INT), interrupt_handler, FALLING);
      //interrupts();

      //device.enableLinearAccelerometer(50);  // m/s^2 no gravity, data update every 50 ms
      //device.enableRotationVector(100); //Send data update every 100 ms
  
      device.enableGyroIntegratedRotationVector(50);//RotationVector(10); //Send data update every 10ms
      device.enableAccelerometer(50); //Send data update every 50
      device.enableMagnetometer(50); //Send data update every 50 // cannot be enabled at the same time as RotationVector (will not produce data)
      //device.enableGyro(10); //Send data update every 10ms
      //device.enableTapDetector(50); //Send data update every 50ms
      dirtyRotationVector=false;
      dirtyAccelerometer=false;
      dirtyMagnetometer=false;
      dirtyGyro=false;
      dirtyLinearAcc=false;
      dirtyQuat=false;
      irq_cnt=0;
      //imu_b.irq_cnt++;
      //imu_b.irq_cnt++;
  }
}

void IMU_BNO085::stepIMU(IMU_Status * imu_status)
{

  if (!imu_valid)
    return;

if (device.dataAvailable() == true)
  {
    imu_status->qx = device.getQuatReal();
    imu_status->qy = device.getQuatI();
    imu_status->qz = device.getQuatJ();
    imu_status->qw = device.getQuatK();

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
    imu_status->heading=(device.getYaw()) * 180.0 / PI; ;

    imu_status->ax = device.getAccelX();
    imu_status->ay = device.getAccelY();
    imu_status->az = device.getAccelZ();
    
    imu_status->mx = device.getMagX();
    imu_status->my = device.getMagY();
    imu_status->mz = device.getMagZ();

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
  
    //imu_status->bump=(float)device.getTapDetector();
  }
    /*
  if(dirtyLinearAcc)
  {
    device.getLinAccel(ax, ay, az, linAccuracy);
    imu_status->ax = ax;
    imu_status->ay = ay;
    imu_status->az = az;
    dirtyLinearAcc=0;
  }
  if(dirtyQuat)
  {
    device.getQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy);
    imu_status->qx = qx;
    imu_status->qy = qy;
    imu_status->qz = qz;
    imu_status->qw = qw;
    dirtyQuat=0;
  }
  if(dirtyAccelerometer)
  {
    imu_status->ax = device.getAccelX();
    imu_status->ay = device.getAccelY();
    imu_status->az = device.getAccelZ();
    dirtyAccelerometer=0;
  }

  if(dirtyGyro)
  {
    imu_status->gx = device.getGyroX();
    imu_status->gy = device.getGyroY();
    imu_status->gz = device.getGyroZ();
    dirtyGyro=0;
  }

  if(dirtyMagnetometer)
  {
    imu_status->mx = device.getMagX();
    imu_status->my = device.getMagY();
    imu_status->mz = device.getMagZ();
    dirtyMagnetometer=0;
  }

  if(dirtyRotationVector)
  {
    imu_status->roll=(device.getRoll()) * 180.0 / PI; 
    imu_status->pitch=(device.getPitch()) * 180.0 / PI;;
    imu_status->heading=(device.getYaw()) * 180.0 / PI; ;

    //q= x + yI + zJ +wK
    imu_status->qx = device.getQuatReal();
    imu_status->qy = device.getQuatI();
    imu_status->qz = device.getQuatJ();
    imu_status->qw = device.getQuatK();

    imu_status->gx = device.getFastGyroX();
    imu_status->gy = device.getFastGyroY();
    imu_status->gz = device.getFastGyroZ();
    
    dirtyRotationVector=0;
  }*/

}

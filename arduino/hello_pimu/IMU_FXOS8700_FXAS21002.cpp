/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "IMU_FXOS8700_FXAS21002.h"
#include "Common.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>




IMU_FXOS8700_FXAS21002 imu_f;

// ////////////////////////////////////// IMU_FXOS8700_FXAS21002 //////////////////////////////////////////////////////////////


//The Adafruit 3463 IMU is mounted upside down in the robot

//Accel + Magnetometer
//https://www.nxp.com/docs/en/data-sheet/FXOS8700CQ.pdf

//Gyro
//https://www.nxp.com/docs/en/data-sheet/FXAS21002.pdf

//In Stretch RE1/RE2, board is oriented such that
//Az+/Mz+: Point down to ground
//Ax+/Mx+: Points in direction of arm reach
//Ay+/My+: Points to back of robot

//Gz+: Base rotates CW
//Gy+: Base rotates side to side
//Gx+: Base rotates front to back
///////////////////////////////////////////////////////////
//Sample IMU off of deterministic timer
//Code from ahrs_fusion_usb AdaFruit sketch
//See https://learn.adafruit.com/nxp-precision-9dof-breakout?view=all for background


#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002
#define NXP_FXOS8700_FXAS21002      (2)

IMU_FXOS8700_FXAS21002::IMU_FXOS8700_FXAS21002()
{
    imu_valid=false;
    imu_calibration_rcvd=false;
    imu_calibration_startup_cnt = 600;
    //Identity
    mag_softiron_matrix[0][0]=1.0;
    mag_softiron_matrix[1][1]=1.0;
    mag_softiron_matrix[2][2]=1.0;
    mag_field_strength = 47.02F;
    gravity_vector_scale = 1.0;
    rate_gyro_vector_scale=1.0;
    accel_LPFa=1.0;
    accel_LPFb=0.0;
}

void IMU_FXOS8700_FXAS21002::setIMUConfig(Pimu_Config * cfg_in, Pimu_Config * cfg)
{

    if (cfg_in->accel_LPF!=cfg->accel_LPF) //Effort filter
    {
      accel_LPFa = exp(cfg_in->accel_LPF*-2*3.14159/FS); // z = e^st pole mapping
      accel_LPFb = (1.0-accel_LPFa);
    }
    
  mag_offsets[0]=cfg_in->mag_offsets[0];
  mag_offsets[1]=cfg_in->mag_offsets[1];
  mag_offsets[2]=cfg_in->mag_offsets[2];

  gyro_zero_offsets[0]=cfg_in->gyro_zero_offsets[0];
  gyro_zero_offsets[1]=cfg_in->gyro_zero_offsets[1];
  gyro_zero_offsets[2]=cfg_in->gyro_zero_offsets[2];

  mag_softiron_matrix[0][0]=cfg_in->mag_softiron_matrix[0];
  mag_softiron_matrix[0][1]=cfg_in->mag_softiron_matrix[1];
  mag_softiron_matrix[0][2]=cfg_in->mag_softiron_matrix[2];

  mag_softiron_matrix[1][0]=cfg_in->mag_softiron_matrix[3];
  mag_softiron_matrix[1][1]=cfg_in->mag_softiron_matrix[4];
  mag_softiron_matrix[1][2]=cfg_in->mag_softiron_matrix[5];

  mag_softiron_matrix[2][0]=cfg_in->mag_softiron_matrix[6];
  mag_softiron_matrix[2][1]=cfg_in->mag_softiron_matrix[7];
  mag_softiron_matrix[2][2]=cfg_in->mag_softiron_matrix[8];

  gravity_vector_scale=cfg_in->gravity_vector_scale;
  rate_gyro_vector_scale=cfg_in->rate_gyro_vector_scale;
  imu_calibration_rcvd=true;
}


bool IMU_FXOS8700_FXAS21002::isIMUOrientationValid()
{
    return imu_calibration_rcvd && imu_calibration_startup_cnt==0; //Time for filter to settle
}

void IMU_FXOS8700_FXAS21002::setupIMU()
{
  // Initialize the sensors.
  if(!gyro.begin())
  {
    imu_valid=false;
    return ;
  }
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    imu_valid=false;
    return ;
  }
  // Filter rate of 100hz, Beta=1.0 works well
  filter.set_beta(1.0);
  filter.begin(100);
  imu_valid=true;
}


void IMU_FXOS8700_FXAS21002::stepIMU(IMU_Status * imu_status)
{

  if (imu_calibration_rcvd)
    imu_calibration_startup_cnt=max(0,imu_calibration_startup_cnt-1);

  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;


  if (!imu_valid)
    return;

  // Get new data samples

  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);
  float ax = gravity_vector_scale*accel_event.acceleration.x;
  float ay = gravity_vector_scale*accel_event.acceleration.y;
  float az = gravity_vector_scale*accel_event.acceleration.z;

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = rate_gyro_vector_scale*(gyro_event.gyro.x + gyro_zero_offsets[0]);
  float gy = rate_gyro_vector_scale*(gyro_event.gyro.y + gyro_zero_offsets[1]);
  float gz = rate_gyro_vector_scale*(gyro_event.gyro.z + gyro_zero_offsets[2]);


  imu_status->gx = gx; //Rad/sec
  imu_status->gy = gy;
  imu_status->gz = gz;

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                ax, ay, az,
                mx, my, mz);


  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();

  if (first_acc_filter)
  {
    first_acc_filter=false;
    imu_status->ax=ax;
    imu_status->ay=ay;
    imu_status->az=az;
  }

  imu_status->ax= accel_LPFa*imu_status->ax +  accel_LPFb*(ax);
  imu_status->ay= accel_LPFa*imu_status->ay +  accel_LPFb*(ay);
  imu_status->az= accel_LPFa*imu_status->az +  accel_LPFb*(az);

  // Compute bump value
  // Sensor queried at 70hz, so bump filter is magnitude of maximum readings over last ~100ms

  int i;
  float maxx,maxy,maxz;
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

  imu_status->mx = mx;
  imu_status->my = my;
  imu_status->mz = mz;
  
  //Roll/Pitch/Yaw Euler
  //Move rollover point out of normal operation
  //IC is upside down so flip
  if (roll <0)
    roll+=180;
  else
    roll-=180;

  imu_status->roll=roll;
  imu_status->pitch=pitch;
  imu_status->heading=heading;

  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  imu_status->qw=qw;
  imu_status->qx=qx;
  imu_status->qy=qy;
  imu_status->qz=qz;

  

}

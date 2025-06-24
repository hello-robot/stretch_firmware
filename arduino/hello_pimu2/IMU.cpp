/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#include "IMU.h"
#include "Pimu.h"
#include "IMU_BNO085.h"

void setupIMU()
{
    imu_b.setupIMU();
}


void stepIMU(IMU_Status * imu_status)
{
    imu_b.stepIMU(imu_status);
}


void setIMUConfig(Pimu_Config * cfg_in, Pimu_Config * cfg)
{

}

bool isIMUOrientationValid()
{
    return imu_b.isIMUOrientationValid();
}

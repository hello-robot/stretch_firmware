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
#include "IMU_FXOS8700_FXAS21002.h"
#include "IMU_BNO085.h"

void setupIMU()
{
  if (BOARD_VARIANT>=3)
    imu_b.setupIMU();
  else
    imu_f.setupIMU();
}


void stepIMU(IMU_Status * imu_status)
{
  if (BOARD_VARIANT>=3)
    imu_b.stepIMU(imu_status);
  else
    imu_f.stepIMU(imu_status);
}


void setIMUConfig(Pimu_Config * cfg_in, Pimu_Config * cfg)
{
  if (BOARD_VARIANT<3)
    imu_f.setIMUConfig(cfg_in,cfg);
}

bool isIMUOrientationValid()
{
  if (BOARD_VARIANT>=3)
    return imu_b.isIMUOrientationValid();
  else
    return imu_f.isIMUOrientationValid();
  return 0;
}

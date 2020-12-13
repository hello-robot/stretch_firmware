/*
  -------------------------------------------------------------
  Hello Robot - Hello Stepper

  This code is derived from the Mechaduino project. 
  https://github.com/jcchurch13/Mechaduino-Firmware

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc.
  --------------------------------------------------------------
*/

#ifndef __TRAJECTORY_MANAGER_H__
#define  __TRAJECTORY_MANAGER_H__

#include "Common.h"

#define NUM_SEGS_MAX 20

class  TrajectoryManager{
   public: 
    TrajectoryManager();
    void step(); //Called at 1Khz by TC4 loop
    bool add_trajectory_segment(TrajectorySegment * s);
    float q; //current position target
  private:
    TrajectorySegment segs[NUM_SEGS_MAX];
    uint8_t id_write;
    uint8_t id_read;
    uint8_t num_seg;
    TrajectorySegment seg_add;
    bool dirty_seg_add;
    uint8_t state;
    float t; 
};

extern TrajectoryManager trajectory_manager;
#endif

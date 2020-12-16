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



class  TrajectoryManager{
   public: 
    TrajectoryManager();
    void step(); //Called at 1Khz by TC4 loop
    uint8_t set_next_trajectory_segment(TrajectorySegment * s);
    uint8_t start_new_trajectory(TrajectorySegment * s,  bool wait_on_sync);
    
    float q; //current position target
    uint8_t id_curr_seg;
    float t;
    bool is_trajectory_active();
    bool waiting_on_sync;
    uint8_t state;

    TrajectorySegment seg_active;
    TrajectorySegment seg_next;
    bool seg_active_valid;
    bool seg_next_valid;
    
    
    TrajectorySegment seg_in;
    bool dirty_seg_in;
    bool start_new;
    
      private:

    
     
};

extern TrajectoryManager trajectory_manager;
#endif

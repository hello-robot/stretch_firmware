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
    bool set_next_trajectory_segment(TrajectorySegment * s);
    bool start_new_trajectory(TrajectorySegment * s,  bool wait_on_sync);
    void reset();
    bool is_trajectory_active();
    bool is_trajectory_waiting_on_sync();
    bool is_trajectory_idle();
    uint16_t get_id_current_segment(){return id_curr_seg;}
    float q; //current position target
    bool waiting_on_sync;
  private:
    
    uint16_t id_curr_seg;
    float t;
    
    uint8_t state;

    TrajectorySegment seg_active;
    TrajectorySegment seg_next;
    bool seg_active_valid;
    bool seg_next_valid;
    
    TrajectorySegment seg_in;
    bool dirty_seg_in;
    bool start_new;

     
};

extern TrajectoryManager trajectory_manager;
#endif

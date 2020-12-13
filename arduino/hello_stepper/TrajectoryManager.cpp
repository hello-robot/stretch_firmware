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

#include "TrajectoryManager.h"
#include "HelloController.h"
#include "Controller.h"


TrajectoryManager trajectory_manager;

#define TRAJ_STATE_IDLE 0
#define TRAJ_STATE_ACTIVE 1 
    
TrajectoryManager::TrajectoryManager()
{
    id_write=0;
    id_read=0;
    num_seg=0;
  
    dirty_seg_add=false;
    state=TRAJ_STATE_IDLE;
    t=0;
    q=0;
}


void TrajectoryManager::step()
{
  if (dirty_seg_add)
  {
    segs[id_write]=seg_add;
    num_seg++;
    id_write=(id_write+1)%NUM_SEGS_MAX;
    dirty_seg_add=false;
  }
 
 /* Behavior is to automatically execute sequential trajectories as long as there
  * is unexecuted segments in the buffer.
  * Then revert to idle state until more data shows up.
  */
 switch(state)
  {
    case TRAJ_STATE_IDLE:
        if (num_seg>0)
        {
          id_read=0;
          state=TRAJ_STATE_ACTIVE;
          t=0;
        }
        break;
    case TRAJ_STATE_ACTIVE:
          float t2 = t*t;
          float t3 =t2*t;
          q = segs[id_read].a0 + segs[id_read].a1*t + segs[id_read].a2*t2 + segs[id_read].a3*t3;
          if (t<segs[id_read].tf) 
            t=min(segs[id_read].tf,t+.001); //Called at 1Khz, increment time for next cycle (Todo: user timer based clock?)
          else
          { //Start next segment
            num_seg--;
            if(num_seg==0) //Finished trajectory
            {
              state=TRAJ_STATE_IDLE;
              id_write=0;
              id_read=0;
            }
            else //start next segment
            {
              id_read=(id_read+1)%NUM_SEGS_MAX;
              t=0;
            }
          }
          break; 
  };
}

//Called from RPC loop
bool TrajectoryManager::add_trajectory_segment(TrajectorySegment * s)
{
  if(num_seg==NUM_SEGS_MAX)
    return false;
  seg_add=*s;
  dirty_seg_add=true; //Flag to add on next step cycle (hanlde in irq, not RPC loop
  return true;
}

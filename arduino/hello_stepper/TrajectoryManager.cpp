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
#define TRAJ_STATE_WAITING_ON_SYNC 1
#define TRAJ_STATE_ACTIVE 2 

#define STEP_RATE (0.001)
    
TrajectoryManager::TrajectoryManager()
{
    dirty_seg_in=false;
    state=TRAJ_STATE_IDLE;
    t=0;
    q=0;
    start_new=false;
    seg_active_valid=false;
    seg_next_valid=false;
    strncpy(seg_load_error_message, "", 100);
    id_curr_seg=0;
    waiting_on_sync=false;
}

bool TrajectoryManager::is_trajectory_active()
{
  return state==TRAJ_STATE_ACTIVE;
}

bool TrajectoryManager::is_trajectory_waiting_on_sync()
{
  return state==TRAJ_STATE_WAITING_ON_SYNC;
}
bool TrajectoryManager::is_trajectory_idle()
{
  return state==TRAJ_STATE_IDLE;
}
    
void TrajectoryManager::reset()
{
  state=TRAJ_STATE_IDLE;
  seg_active_valid=false;
  id_curr_seg=0;
}

void TrajectoryManager::step()
{
  
  if (dirty_seg_in) //Transfer in new data on sync signal, starts the trajectory
  {
    if (start_new)
    {
      start_new=false;
      seg_active=seg_in;
      seg_active_valid=true;
    }
    else
    {
      if(seg_in.tf>0) //A tf of 0 marks the end of a trajectory
      {
        seg_next=seg_in;
        seg_next_valid=true;
      }
      else
        seg_next_valid=false;
    }
    dirty_seg_in=false;
  }
 
 /* Behavior is to automatically execute sequential trajectories as long as there
  * is unexecuted segments in the buffer.
  * Then revert to idle state until more data shows up.
  */
 
 if(state==TRAJ_STATE_IDLE)
 {
    id_curr_seg=0;//reserved for not loaded
    if (seg_active_valid) 
    {
      if (waiting_on_sync)
        state=TRAJ_STATE_WAITING_ON_SYNC;
      else
        state=TRAJ_STATE_ACTIVE;
      t=0;
    }
 }
 
 if(state==TRAJ_STATE_WAITING_ON_SYNC)
 {
    id_curr_seg=1;//reserved for waiting
    if (!waiting_on_sync)
           state=TRAJ_STATE_ACTIVE; //Fall through and start
 }
 
 if(state==TRAJ_STATE_ACTIVE)
 {
    float t2 = t*t;
    float t3 =t2*t;
    float t4 =t3*t;
    float t5 =t4*t;
    id_curr_seg=seg_active.id;
    q = seg_active.a0 + seg_active.a1*t + seg_active.a2*t2 + seg_active.a3*t3 + seg_active.a4*t4 + seg_active.a5*t5;
    if (t<seg_active.tf) 
      t=min(seg_active.tf,t+STEP_RATE); //Called at 1Khz, increment time for next cycle (Todo: user timer based clock?)
    else //Finished segment
    { 
      if(!seg_next_valid) //Finished trajectory
      {
        state=TRAJ_STATE_IDLE;
        seg_active_valid=false;
        id_curr_seg=0;//reserved for no trajectory
      }
      else //start next segment
      {
        seg_active=seg_next;
        seg_next_valid=false;
        t=0;
      }
    }
 }
 
}

//Called from RPC loop
//Return 1 for success
bool TrajectoryManager::set_next_trajectory_segment(TrajectorySegment * s, MotionLimits * m, Command * c)
{
  if (state==TRAJ_STATE_ACTIVE) //Don't allow starting of new trajectory until current one is done
  {
    seg_in=*s;
    dirty_seg_in=true; //Flag to add on next step cycle (hanlde in irq, not RPC loop
    return seg_active.id;
  }
  if (state==TRAJ_STATE_WAITING_ON_SYNC)
    return 1;
  return 0;
}

//Called from RPC loop
//Return 1 for success
bool TrajectoryManager::start_new_trajectory(TrajectorySegment * s, bool wait_on_sync, MotionLimits * m, Command * c)
{
  if (state==TRAJ_STATE_IDLE) //Don't allow starting of new trajectory until current one is done
  {
    start_new=true;
    seg_in=*s;
    dirty_seg_in=true; //Flag to add on next step cycle (hanlde in irq, not RPC loop
    waiting_on_sync=wait_on_sync;
    return 1;
  }
  return 0;
}

//Determines whether a segment is executable by evaluating along it
//and checking for infeasible positions, velocities, and accelerations
//Return 1 for valid segment
bool TrajectoryManager::is_segment_valid(TrajectorySegment * s, MotionLimits * m, Command * c)
{
  TrajectorySegment a = *s;
  float t1 = 0.0;
  while (t1 < a.tf) {
    float t2 = t1 * t1;
    float t3 = t2 * t1;
    float t4 = t3 * t1;
    float t5 = t4 * t1;

    // evaluate quintic polynomial at t1 for position, velocity, and acceleration
    float pos_t = a.a0 + (a.a1 * t1) + (a.a2 * t2) + (a.a3 * t3) + (a.a4 * t4) + (a.a5 * t5);
    float vel_t = a.a1 + (2.0 * a.a2 * t1) + (3.0 * a.a3 * t2) + (4.0 * a.a4 + t3) + (5.0 * a.a5 + t4);
    float acc_t = (2.0 * a.a2) + (6.0 * a.a3 * t1) + (12 * a.a4 * t2) + (20 * a.a5 * t3);

    // check position within soft motion limits
    if (pos_t < m->pos_min || pos_t > m->pos_max) {
      strncpy(seg_load_error_message, "invalid segment exceeds position limits", 100);
      return 0;
    }

    // check velocity within commanded velocity
    if (abs(vel_t) > c->v_des) {
      strncpy(seg_load_error_message, "invalid segment exceeds velocity limits", 100);
      return 0;
    }

    // check acceleration within commanded acceleration
    if (abs(acc_t) > c->a_des) {
      strncpy(seg_load_error_message, "invalid segment exceeds acceleration limits", 100);
      return 0;
    }

    t1 = min(a.tf, t1 + STEP_RATE);
  }

  strncpy(seg_load_error_message, "", 100);
  return 1;
}

/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/

#ifndef __BEEP_MANAGER_H__
#define  __BEEP_MANAGER_H__

#define BEEP_ID_OFF 0
#define BEEP_ID_SINGLE_SHORT 1
#define BEEP_ID_SINGLE_LONG 2
#define BEEP_ID_DOUBLE_SHORT 3
#define BEEP_ID_DOUBLE_LONG 4

class BeepManager {    
  public: 
    BeepManager();
    void do_beep(int bid);
    void step();
  private:
    int beep1_on_cnt;
    int beep1_off_cnt;
    int beep2_on_cnt;
};

extern BeepManager beep_manager;



#endif

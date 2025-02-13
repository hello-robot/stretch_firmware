/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
  
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/


#include "BeepManager.h"
#include "Common.h"

BeepManager beep_manager;

//Step is called at 100hz, so x100
#define BEEP_4000MS 400
#define BEEP_2000MS 200
#define BEEP_1000MS 100
#define BEEP_500MS 50
#define BEEP_250MS 25

BeepManager::BeepManager()
{
      beep1_on_cnt=0;
      beep1_off_cnt=0;
      beep2_on_cnt=0;
}

void BeepManager::do_beep(int bid)
{
    switch(bid)
    {
      case BEEP_ID_OFF:
        beep1_on_cnt=0;
        beep1_off_cnt=0;
        beep2_on_cnt=0;
        break;
      case BEEP_ID_SINGLE_SHORT:
        beep1_on_cnt=BEEP_250MS;
        beep1_off_cnt=0;
        beep2_on_cnt=0;
        break;
      case BEEP_ID_SINGLE_LONG:
        beep1_on_cnt=BEEP_500MS;
        beep1_off_cnt=0;
        beep2_on_cnt=0;
        break;
      case BEEP_ID_DOUBLE_SHORT:
        beep1_on_cnt=BEEP_250MS;
        beep1_off_cnt=BEEP_500MS;
        beep2_on_cnt=BEEP_250MS;
        break;
      case BEEP_ID_DOUBLE_LONG:
        beep1_on_cnt=BEEP_500MS;
        beep1_off_cnt=BEEP_500MS;
        beep2_on_cnt=BEEP_500MS;
        break;
      default:
        beep1_on_cnt=0;
        beep1_off_cnt=0;
        beep2_on_cnt=0;
        break;
    };
}

  
void BeepManager::step() //Called at 100hz by TC5 loop
{
    if (beep1_on_cnt>0)
    {
      digitalWrite(BUZZER, HIGH);
      beep1_on_cnt=max(0,beep1_on_cnt-1);
    }
    if (beep1_on_cnt==0 && beep1_off_cnt>0)
    {
      beep1_off_cnt=max(0,beep1_off_cnt-1);
      digitalWrite(BUZZER, LOW);
    }
    if (beep1_on_cnt==0 &&  beep1_off_cnt==0 && beep2_on_cnt>0)
    {
      digitalWrite(BUZZER, HIGH);
      beep2_on_cnt=max(0,beep2_on_cnt-1);
    }
    if (beep1_on_cnt==0 && beep1_off_cnt==0 && beep2_on_cnt==0)
    {
      digitalWrite(BUZZER, LOW);
    }
}

#include "ChargerManager.h"
#include "TimeManager.h"
#include "Common.h"


unsigned long t_charger_last=0;


bool ChargerManager::step(float vbat, float sys_current, float charge_current, int board_variant)
{

	unsigned long t = time_manager.get_elapsed_time_ms();
	if (t -  t_charger_last > CHARGER_SAMPLE_RATE)
	{
		t_charger_last = t;

		//Charger Connected Pin LOW Charger Not Connected
		if (digitalRead(CHARGER_CONNECTED) == LOW)
		{
			charging_sts_flag = false;
			charger_plugged_in_flag = false;
		}

		//Charger Connected Pin HIGH Charger Connected
		if (digitalRead(CHARGER_CONNECTED) == HIGH)
		{	

			
			charger_plugged_in_flag = true;
			if (charge_current >= 0.07)
			{
				charging_sts_flag = true;
				
			}
			else if (charge_current < 0.07)
			{
				charging_sts_flag = false;
			}

			
		}

	}
	return charging_sts_flag;
	
}



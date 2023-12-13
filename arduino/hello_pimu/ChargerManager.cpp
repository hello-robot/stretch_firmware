#include "ChargerManager.h"
#include "TimeManager.h"
#include "Common.h"


unsigned long t_charger_last=0;

bool charging_sts_flag = false;
bool hotplug_sts_flag = false;

float v1 = 0.0;

bool hotplug_check(const float v_d)
{
	
	//hotplug condition 
	if (v_d >= 0.07)
	{
		hotplug_sts_flag = false;
		return true;
	}
	if (v_d <= 0.05)
	{
		return false;
	}
	
}

bool ChargerManager::step(float vbat, float sys_current, float charge_current, int board_variant)
{

	unsigned long t = time_manager.get_elapsed_time_ms();

	if (t -  t_charger_last > CHARGER_SAMPLE_RATE)
	{
		t_charger_last = t;

		//Charger Connected Pin LOW Charger Not Connected
		if (digitalRead(CHARGER_CONNECTED) == LOW)
		{
			v1 = vbat;
			hotplug_sts_flag = true;
			charging_sts_flag = false;
		}

		//Charger Connected Pin HIGH Charger Connected
		if (digitalRead(CHARGER_CONNECTED) == HIGH)
		{	
			
			float v2 = vbat;
			float vdiff = v2 - v1;
			charging_sts_flag = true;

			if (charge_current >= 0.15)
			{
				charging_sts_flag = true;
				hotplug_sts_flag = false;
			}

			else if (charge_current < 0.15)
			{
				if (hotplug_sts_flag == true)
				{
					charging_sts_flag = hotplug_check(vdiff);
				}
				if (board_variant >=3 && sys_current < 2)
				{
					charging_sts_flag = false;
				}
			}
		}

	}
	return charging_sts_flag;
	
}



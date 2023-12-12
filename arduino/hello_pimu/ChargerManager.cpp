#include "ChargerManager.h"
#include "TimeManager.h"
#include "Common.h"


unsigned long t_charger_last=0;

float voltage_window[256] = {};
float current_window[256] = {};

float vdiff_window[2];

bool charging_sts_flag = false;
bool hotplug_sts_flag = false;
bool unplug_sts_flag = false;


int chrg_idx = 0;
int vidx = 0;



float v1 = 0.0;
float prev_dc = 0.0;
float prev_sys_current = 0.0;
float prev_vbat = 0.0;

bool hotplug_check(const float v_d)
{
	//hotplug condition 
	if (v_d > 0.1)
	{
		// Serial.println("Hotplugged Voltage went up");
		hotplug_sts_flag = false;
		return true;
	}
	if (-0.02 < v_d <= 0)
	{
		// Serial.println("Hotplugged Voltage stayed the same");
		hotplug_sts_flag = false;
		return false;
	}
	
}

bool ChargerManager::step(float vbat, float sys_current, float charge_current, int board_variant)
{

	unsigned long t = time_manager.get_elapsed_time_ms();
	voltage_window[chrg_idx] = vbat;
	// current_window[chrg_idx] = sys_current;
	chrg_idx++;

	if (t -  t_charger_last > CHARGER_SAMPLE_RATE)
	{
	
		t_charger_last = t;

		//Charger Connected Pin LOW Charger Not Connected
		if (digitalRead(CHARGER_CONNECTED) == LOW)
		{
			v1 = vbat;
			hotplug_sts_flag = true;
			charging_sts_flag = false;
			memset(vdiff_window, 0, sizeof(vdiff_window));
			vidx = 0;
		}

		//Charger Connected Pin HIGH Charger Connected
		if (digitalRead(CHARGER_CONNECTED) == HIGH)
		{
			float v2 = vbat;
			float vdiff = v2 - v1;

			float delta_volt = voltage_window[chrg_idx-1] - voltage_window[1];
			// float delta_curr = current_window[chrg_idx-1] - current_window[1];
			float delta_curr = sys_current - prev_sys_current;

			//When Charging Current is below 150mA other checks need to be do
			if (charge_current < 0.15)
			{
				if (board_variant >= 3)
				{
					if (sys_current >= 3)
					{
						if (hotplug_sts_flag == true)
						{
							if (vidx < 2)
							{
								vdiff_window[vidx] = vdiff;
								vidx++;
							}
							else
							{
								charging_sts_flag = hotplug_check(max(vdiff_window[0], vdiff_window[1]));
							}
						}
						else
						{
							if (charging_sts_flag == true)
							{
								if(delta_volt < -0.12 && (max(delta_curr,prev_dc) < 0.5))
								{
									charging_sts_flag = false;
									unplug_sts_flag = true;
								}
							}

							if (charging_sts_flag == false)
							{
								if(delta_volt > 0.12 && (min(delta_curr,prev_dc) >= -0.2))
								{
									charging_sts_flag = true;
									unplug_sts_flag = false;
								}
							}

							if (unplug_sts_flag == false && vbat > 13.2)
							{
								charging_sts_flag = true;
							}
						}	
					}
					else if (sys_current < 3)
					{
						charging_sts_flag = false;
						hotplug_sts_flag = false;
					}
				}

				if (board_variant < 3)
				{
					charging_sts_flag = true;
				}
			}

			//When Charging Current is high
			else
			{
				charging_sts_flag = true;
				hotplug_sts_flag = false;
			}

			prev_dc = delta_curr;
			prev_sys_current = sys_current;
		}
		

		chrg_idx = 0;
		memset(voltage_window, 0, sizeof(voltage_window));
		memset(current_window, 0, sizeof(current_window));
	}
	return charging_sts_flag;
	
}



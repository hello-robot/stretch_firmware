#include "ChargerManager.h"
#include "TimeManager.h"
#include "Common.h"
#include "Pimu.h"

unsigned long t_charger_last=0;

float voltage_window[256] = {};
float current_window[256] = {};

chargerState charging_sts_flag = CHARGER_FALSE;
bool hotplug_sts_flag = false;
bool unplug_sts_flag = false;


int chrg_idx = 0;
float v1 = 0.0;
float prev_dc = 0.0;

chargerState ChargerManager::step(float vbat, float sys_current, float charge_current)
{

	unsigned long t = time_manager.get_elapsed_time_ms();
	voltage_window[chrg_idx] = vbat;
	current_window[chrg_idx] = sys_current;
	chrg_idx++;

	if (BOARD_VARIANT < 3 && BOARD_VARIANT > 0)
	{
		if (digitalRead(CHARGER_CONNECTED) == LOW)
		{
			charging_sts_flag = CHARGER_FALSE;
		}
		else
		{
			charging_sts_flag = CHARGER_TRUE;
		}
		return charging_sts_flag;
	}

	if (BOARD_VARIANT >= 3)
	{

		if (t -  t_charger_last > CHARGER_SAMPLE_RATE)
		{
		
			t_charger_last = t;

			//Charger Connected Pin LOW Charger Not Connected
			if (digitalRead(CHARGER_CONNECTED) == LOW)
			{
				v1 = vbat;
				hotplug_sts_flag = true;
				charging_sts_flag = CHARGER_FALSE;
			}

			//Charger Connected Pin HIGH Charger Connected
			if (digitalRead(CHARGER_CONNECTED) == HIGH)
			{
				float v2 = vbat;
				float vdiff = v2 - v1;

				float delta_volt = voltage_window[chrg_idx-1] - voltage_window[1];
				float delta_curr = current_window[chrg_idx-1] - current_window[1];

				//When Charging Current is below 150mA other checks need to be do
				if (charge_current < 0.15)
				{
					if (sys_current >= 3)
					{
						//hotplug condition 
						if (hotplug_sts_flag == true && vdiff > 0.1)
						{
							// Serial.println("Hotplugged Voltage went up");
							charging_sts_flag = CHARGER_TRUE;
							hotplug_sts_flag = false;
						}
						if (hotplug_sts_flag == true && -0.02 < vdiff <= 0)
						{
							// Serial.println("Hotplugged Voltage stayed the same");
							charging_sts_flag = CHARGER_NOT_PLUGGED;
							unplug_sts_flag = true;
							hotplug_sts_flag = false;
						}

						if (hotplug_sts_flag == false && unplug_sts_flag == false)
						{
							if(delta_volt < -0.12 and (max(delta_curr,prev_dc) < 0.5))
							{
								charging_sts_flag = CHARGER_NOT_PLUGGED;
								unplug_sts_flag = true;
							}
						}

						if (hotplug_sts_flag == false && unplug_sts_flag == true)
						{
							if(delta_volt > 0.12 and (min(delta_curr,prev_dc) >= -0.2))
							{
								// Serial.println("Voltage Increased while current is low");
								charging_sts_flag = CHARGER_TRUE;
								unplug_sts_flag = false;
							}
						}

						if (unplug_sts_flag == false || vbat > 13.2)
						{
							// Serial.println("Default charging");
							charging_sts_flag = CHARGER_TRUE;

						}


					}

					//When System current is low and charge current is low
					else if (sys_current < 3)
					{
						// Serial.println("System current is low and charge current low");
						charging_sts_flag = CHARGER_NOT_PLUGGED;
						unplug_sts_flag = true;
						hotplug_sts_flag = false;
					}
				}

				else
				{
					// Serial.println("Charging Current is HIGH");
					charging_sts_flag = CHARGER_TRUE;
					unplug_sts_flag = false;
					hotplug_sts_flag = false;
				}
				prev_dc = delta_curr;

			}

			chrg_idx = 0;
			memset(voltage_window, 0, sizeof(voltage_window));
			memset(current_window, 0, sizeof(current_window));
		}
	

		return charging_sts_flag;
	}
}
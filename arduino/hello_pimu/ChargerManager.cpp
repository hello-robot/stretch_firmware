#include "ChargerManager.h"
#include "TimeManager.h"
#include "Common.h"


unsigned long t_charger_last=0;

float v1 = 0.0;
float current_change = 0.0;
float voltage_change = 0.0;

float sys_curr_array[10] = {};
float sys_volt_array[10] = {};
int curr_cnt = 0;


void ChargerManager::hotplug_check(float vd)
{
	//hotplug condition 
	if (vd >= 0.07 && hotplug_sts_flag)
	{	
		hotplug_sts_flag = false;
		charging_sts_flag = true;
	}
	if (vd <= 0.05 && hotplug_sts_flag)
	{
		charging_sts_flag = false;
	}
}

void ChargerManager::unplug_check(float v, float c)
{
	sys_curr_array[curr_cnt] = c;
	sys_volt_array[curr_cnt] = v;
	
	if (curr_cnt == 9)
	{	
		float voltage_change = sys_volt_array[9] - sys_volt_array[0];

		float max_c = sys_curr_array[0];
		float min_c = sys_curr_array[0];
		for (int x = 1; x < 10; ++x)
		{
			if(sys_curr_array[x] > max_c)
			{
				max_c = sys_curr_array[x];
			}
			if(sys_curr_array[x] < min_c)
			{
				min_c = sys_curr_array[x];
			}
		}

		if (voltage_change < -0.15 && (max_c - min_c) < 1 && unplug_sts_flag == false)
		{
			charging_sts_flag = false;
			unplug_sts_flag = true;
		}
		if (voltage_change > 0.15 && (max_c - min_c) < 1 && unplug_sts_flag == true)
		{
			charging_sts_flag = true;
			unplug_sts_flag = false;
		}

		// for (int y=0; y < 10; y++)
		// {
		// 	Serial.print(sys_volt_array[y]);
		// 	Serial.print(", ");
		// }
		// Serial.print(max_c - min_c);
		// Serial.print(", ");
		// Serial.print(hotplug_sts_flag);
		// Serial.print(", ");
		// Serial.print(charging_sts_flag);
		// Serial.println();


		sys_curr_array[0] = sys_curr_array[9];
		sys_volt_array[0] = sys_volt_array[9];
		for (int i = 1; i < 10; i++)
		{
			sys_curr_array[i] = 0;
			sys_volt_array[i] = 0;
		}
		curr_cnt = 0;
	}
	curr_cnt++;

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
			unplug_sts_flag = true;
		}

		//Charger Connected Pin HIGH Charger Connected
		if (digitalRead(CHARGER_CONNECTED) == HIGH)
		{	
			
			float v2 = vbat;
			float vdiff = v2 - v1;

			if (board_variant >= 3)
			{
				if (charge_current >= 0.15)
				{
					charging_sts_flag = true;
					unplug_sts_flag = false;
					hotplug_sts_flag = false;
				}

				if (charge_current < 0.15 && sys_current <= 2)
				{
					charging_sts_flag = false;
					unplug_sts_flag = true;
					hotplug_sts_flag = false;
				}

				if (charge_current < 0.15 && sys_current > 2)
				{
					hotplug_check(vdiff);
					unplug_check(vbat, sys_current);
				}
			}

			if (board_variant < 3)
			{
				hotplug_check(vdiff);
			}
			
		}

	}
	return charging_sts_flag;
	
}



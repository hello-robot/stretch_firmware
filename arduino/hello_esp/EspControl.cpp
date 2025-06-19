
#include "EspControl.h"


UartManager uart_manager; // Using Serial1 for UART communication
PeripheralManager peripheral_manager;

Samd_Status samd_status;
Esp_Status esp_status;
Esp_Trigger esp_trigger;

system_pwr_state current_pwr_state = STATE_ACTIVE;

void send_esp_status();
void read_voltages();
void handle_trigger();

void setup_esp()
{
    peripheral_manager.gpio_init();
    uart_manager.setup_uart();
}

void send_ack(uint8_t sts)
{
	uart_manager.send_status(sts, 0,0);
}

void send_esp_status()
{
	read_voltages();
	esp_status.charger_adapter_fault = digitalRead(PIN_ADAPTER_FAULT);
	esp_status.charger_barrel_fault = digitalRead(PIN_BARREL_FAULT);
	esp_status.cpu_sts = digitalRead(PIN_RPI_STS);
	uart_manager.send_status(UART_ESP_STATUS, &esp_status, sizeof(Esp_Status));
}

void process_pimu_requests()
{
	// Process the received data in the buffer
	uint8_t rx_buf[MAX_UART_PACKET_SIZE];
	uint8_t n = 0;
	if (uart_manager.receive_packet(rx_buf, n, sizeof(rx_buf)))
	{
		switch (rx_buf[0])
		{
			case UART_PWR_SLEEP:
				current_pwr_state = STATE_SLEEP;
				peripheral_manager.peripheral_sleep_state();
				// Handle trigger command
				break;
			case UART_STS_SLEEP_CHRG:
			case UART_STS_SD_CHRG:
				current_pwr_state = STATE_SHUTDOWN_CHRG;
				peripheral_manager.peripheral_sd_state();
				// Handle trigger command
				break;
			case UART_PWR_WAKE:
				current_pwr_state = STATE_ACTIVE;
				digitalWrite(PIN_ROBOT_ACTIVE, HIGH);
				peripheral_manager.peripheral_wakeup_state();
				// Handle trigger command
				break;
			case UART_SAMD_STATUS:
				memcpy(&samd_status, &rx_buf[1], sizeof(Samd_Status));
				break;
			case UART_TRIGGER:
				memcpy(&esp_trigger, &rx_buf[1], sizeof(Esp_Trigger));
				handle_trigger();
				break;
			default:
				break;
		}
	}
	// Serial.print("Battery V: ");
	// Serial.print(samd_status.voltage_battery);
	// Serial.print(" Battery SOC: ");
	// Serial.print(samd_status.battery_soc);
	// Serial.print(" CPU dcdc VOLTAGE: ");
	// Serial.print(samd_status.voltage_20v0);
	// Serial.print(" Charger charging: ");
	// Serial.print(samd_status.charger_charging);
	// Serial.print(" Runstop Event: ");
	// Serial.println(samd_status.state_runstop_event);
	send_esp_status();

}

void handle_trigger()
{
	if(esp_trigger.data & TRIGGER_LIDAR_OFF)
	{
		peripheral_manager.disable_lidar(true);
	}
	if(esp_trigger.data & TRIGGER_LIDAR_ON)
	{
		peripheral_manager.disable_lidar(false);
	}
	if(esp_trigger.data & TRIGGER_20V0_AUX_OFF)
	{
		peripheral_manager.enable_aux_20v0(false);
	}
	if(esp_trigger.data & TRIGGER_20V0_AUX_ON)
	{
		peripheral_manager.enable_aux_20v0(true);
	}
}

void enter_wake()
{
	current_pwr_state = STATE_ACTIVE;
	peripheral_manager.peripheral_wakeup_state();
}

void read_voltages()
{
	esp_status.voltage_12v0 = (analogRead(PIN_12V0_VOLT)*3.4f/4095)*11;
	esp_status.voltage_20v0_aux = (analogRead(PIN_AUX_20V0_VOLT)*3.4f/4095)*11;
}

void toggle_led(int rate_ms)
{
    static uint32_t last_toggle = 0;
    static bool led_state = false;
    uint32_t now = millis();
    if (now - last_toggle > rate_ms) {
        led_state = !led_state;
        digitalWrite(PIN_ESP_STS_LED, led_state);
        last_toggle = now;
    }
}
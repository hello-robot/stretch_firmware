#include "UartManager.h"
#include "CommProtocol.h"
#include "PeripheralManager.h"
#include "EspControl.h"

UartManager uart_manager; // Using Serial1 for UART communication
PeripheralManager peripheral_manager;

VoltageStatus voltage_status;

system_pwr_state current_pwr_state = STATE_ACTIVE;

void setup_esp()
{
    peripheral_manager.gpio_init();
    uart_manager.setup_uart();
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
			case UART_STS_VOLTAGE:
				memcpy(&voltage_status, &rx_buf[1], sizeof(VoltageStatus));
				break;
			case UART_STS_CURRENT:
				// Handle current status
				break;
			case UART_GET_STS:
				break;
			default:
				break;
		}
	}
}

void enter_wake()
{
	current_pwr_state = STATE_ACTIVE;
	peripheral_manager.peripheral_wakeup_state();
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
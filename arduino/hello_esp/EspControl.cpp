#include "UartManager.h"
#include "CommProtocol.h"
#include "PeripheralManager.h"

UartManager uart_manager; // Using Serial1 for UART communication
PeripheralManager peripheral_manager;

VoltageStatus voltage_status;

bool system_pwr_state_active = true;

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
		switch (rx_buf[0]) {
			case UART_STS_VOLTAGE:
				memcpy(&voltage_status, &rx_buf[1], sizeof(VoltageStatus));
				break;
			case UART_STS_CURRENT:
				// Handle current status
				break;
			case UART_TRIGGER:
				// Handle trigger command
				break;
			case UART_PWR_SLEEP:
				system_pwr_state_active = false;
				peripheral_manager.peripheral_sleep_state();
				// Handle trigger command
				break;
			default:
				// Handle unknown command
				break;
		}
	}
}

void enter_wake()
{
	system_pwr_state_active = true;
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
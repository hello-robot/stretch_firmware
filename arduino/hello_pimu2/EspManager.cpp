#include "EspManager.h"
#include <Transport.h>
EspManager::EspManager()
{
	_crc = &crc;
	_cobs = &cobs;
}
void EspManager::setup()
{
	Serial1.begin(1000000);
}

void EspManager::send_packet(const uint8_t *data, uint8_t len)
{
	uint8_t temp_buf[256];
	memcpy(temp_buf, data, len);
	_crc->clearCrc();
	uint16_t value = _crc->Modbus(temp_buf, 0, len);
	temp_buf[len++] = (value >> 8) & 0xff;
	temp_buf[len++] = value & 0xff;
	int nb = _cobs->encode(temp_buf, len, _tx_buffer);
	_tx_buffer[nb++] = COBS_FRAME_DELIMITER;
	Serial1.write(_tx_buffer, nb);
}

void EspManager::rx_step()
{
	unsigned long t_start = micros();
	uint8_t byte_in;

	switch (_esp_rx_state)
	{
		case ESP_RX_WAIT:
		{
			while (Serial1.available())
			{
				byte_in = Serial1.read();

				if (byte_in == COBS_FRAME_DELIMITER)
				{
					
					_esp_rx_state = ESP_RX_VALIDATE;
					break;
				}
				else
				{
					if (_rx_buffer_idx < sizeof(_rx_buffer))
					{
						_rx_buffer[_rx_buffer_idx++] = byte_in;
					}
					else
					{
						_rx_buffer_overflow = true;
						_rx_buffer_idx = 0;
					}
				}
			}
			break;
		}
		case ESP_RX_VALIDATE:
		{
			
			uint8_t decoded[256];
			uint8_t n = _cobs->decode(_rx_buffer, _rx_buffer_idx, decoded);
			
			if (n < 2)
			{
				_esp_rx_state = ESP_RX_WAIT;
				_rx_buffer_idx = 0;
				break;
			}
			_crc->clearCrc();
			uint16_t crc1 = _crc->Modbus(decoded, 0, n - 2);
			uint16_t crc2 = (decoded[n - 2] << 8) | decoded[n - 1];
			n = n - 2;
			_rx_buffer_idx = 0;
			_rx_buffer_overflow = false;
			_esp_rx_state = ESP_RX_WAIT;
			
			if (crc1 == crc2)
			{
				
				switch (decoded[0])
				{
				case UART_ESP_STATUS:
				{
					memcpy(&esp_sts, &decoded[1], sizeof(Esp_Status));
					break;
				}
				case UART_WAKE_ACK:
				{
					wake_ack = true;
					break;
				}
				default:
					break;
				}
			}
		}
		default:
			_esp_rx_state = ESP_RX_WAIT;
			break;
	}
}



void EspManager::send_status(uint8_t sts, const void* data, size_t data_size)
{
	uint8_t buf[MAX_UART_PACKET_SIZE];
	uint8_t idx = 0;
	buf[idx++] = sts; //First byte pwr sts id
	if (data && data_size > 0)
	{
		memcpy(&buf[idx], data, data_size);
		idx += data_size;
	}
	send_packet(buf, idx);
}

void EspManager::esp_fw_update()
{
	digitalWrite(ESP_RESET, HIGH);
	digitalWrite(ESP_BOOT, HIGH);
	digitalWrite(ESP_RESET, LOW);
	delayMicroseconds(500);
	digitalWrite(ESP_BOOT, LOW);
}

void EspManager::esp_reset()
{
	digitalWrite(ESP_RESET, HIGH);
	delayMicroseconds(500);
	digitalWrite(ESP_RESET, LOW);
}
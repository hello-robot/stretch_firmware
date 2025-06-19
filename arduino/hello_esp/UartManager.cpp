#include "UartManager.h"

UartManager::UartManager()
{
	_hardwareSerial = &Serial1;
	_crc = &crc;
	_cobs = &cobs;
}

void UartManager::setup_uart() {
		_hardwareSerial->begin(1000000, SERIAL_8N1, PIN_UART1_RX, PIN_UART1_TX); // RX, TX
		_hardwareSerial->flush(); // Clear any existing data in the buffer
		esp_sleep_enable_uart_wakeup(UART_NUM_1);
		
}

// void UartManager::enable_rx_interrupt() {
//   uart_t* uart = uartGetHwSerial(&_uartSerial);
//   uartEnableRxIntr(uart); // Enable RX interrupt
//   uartAttachRxCallback(uart, onUartRx, NULL); // Attach RX callback
// }

void UartManager::send_packet(const uint8_t *data, uint8_t len)
{
	uint8_t temp_buf[256];
	memcpy(temp_buf, data, len);
	_crc->clearCrc();
	uint16_t value = _crc->Modbus(temp_buf, 0, len);
	temp_buf[len++] = (value >> 8) & 0xff;
	temp_buf[len++] = value & 0xff;
	int nb = _cobs->encode(temp_buf, len, _tx_buffer);
	_tx_buffer[nb++] = COBS_FRAME_DELIMITER;
	_hardwareSerial->write(_tx_buffer, nb);
}

bool UartManager::receive_packet(uint8_t *data, uint8_t &n, int cobbs_frame_size)
{
	unsigned long t_start = micros();
	uint8_t byte_in;
	while ((micros() - t_start) < FRAMING_TIMEOUT) // data may be sparse, keep polling until first byte arrives, then get whole packet
	{
		if (_hardwareSerial->available() > 0)
		{
			byte_in = _hardwareSerial->read();
			t_start = micros(); // Restart the timer otherwise can have race condition as prior start point may be close to expiring

			if (byte_in == COBS_FRAME_DELIMITER)
			{
				n = _cobs->decode(_rx_buffer, _rx_buffer_idx, data);
				_crc->clearCrc();
				uint16_t crc1 = _crc->Modbus(data, 0, n - 2);
				uint16_t crc2 = (data[n - 2] << 8) | data[n - 1];
				n = n - 2;
				_rx_buffer_idx = 0;
				_rx_buffer_overflow = false;
				return (crc1 == crc2);
			}
			else
			{
				if ((_rx_buffer_idx + 1) < cobbs_frame_size)
				{
					_rx_buffer[_rx_buffer_idx++] = byte_in;
				}
				else
				{
					// The buffer will be in an overflowed state if we write
					// so set a buffer overflowed flag.
					_rx_buffer_overflow = true;
					_rx_buffer_idx = 0;
					// ready_rpc_state();
				}
			}
		}
	}
	return false;
}

void UartManager::send_status(uint8_t sts, const void* data, size_t data_size)
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

// void IRAM_ATTR onUartRx(void* arg) {
//   while (_uartSerialPtr && _uartSerialPtr->available()) {
//     uint8_t byte = _uartSerialPtr->read();
//     uint16_t next_head = (_uart_rx_head + 1) % sizeof(uart_rx_buffer);

//     if (next_head != _uart_rx_tail) {
//       uart_rx_buffer[_uart_rx_head] = byte;
//       _uart_rx_head = next_head;
//     }
//   }
// }

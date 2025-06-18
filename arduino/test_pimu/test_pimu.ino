
/*
  -------------------------------------------------------------
  Hello Robot - Hello Pimu
    
  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  
  https://www.gnu.org/licenses/gpl-3.0.html
  
  Copyright (c) 2020 by Hello Robot Inc. All rights reserved.
  --------------------------------------------------------------
*/


#include "Arduino.h"
#include <Transport.h>

#define MAX_UART_PACKET_SIZE 256
#define FRAMING_TIMEOUT 100000

Crc16* _crc;
COBS* _cobs;

enum BmsCommState {
    BMS_START,
    BMS_RX_IDLE,
    BMS_TX_IDLE,
    BMS_PARSE
};


struct BmsStatus {
    float voltage;
    float current;
    uint8_t soh;
    uint8_t soc;
};

bool _rx_buffer_overflow = false;
int  _rx_buffer_idx=0;
uint8_t _rx_buffer[MAX_UART_PACKET_SIZE];
uint8_t _tx_buffer[MAX_UART_PACKET_SIZE];
uint8_t rx_len = 0;

unsigned long last_request_time = 0;
unsigned long response_start_time = 0;

BmsCommState bms_state = BMS_START;
BmsStatus status;


void setup()        // This code runs once at startup
{
	_crc = &crc;
	_cobs = &cobs;
  SerialUSB.begin(2000000);
  while(!SerialUSB);
  Serial2.begin(9600);
  pinMode(PIN_TX_EN, OUTPUT);
  pinMode(PIN_STS_LED, OUTPUT);
  bms_startup();


}

void loop()
{
  // bms_step();
  // delay(1);
  // SerialUSB.print("Voltage: ");
  // SerialUSB.print(status.voltage);
  // SerialUSB.print(" Current: ");
  // SerialUSB.print(status.current);
  // SerialUSB.print(" SOH: ");
  // SerialUSB.print(status.soh);
  // SerialUSB.print(" SOC: ");
  // SerialUSB.println(status.soc);
}

void bms_startup()
{
    digitalWrite(PIN_TX_EN, HIGH);
    send_bms_read_packet(0x06, 0x01);
    while (!(SERCOM4->USART.INTFLAG.bit.TXC));
    digitalWrite(PIN_TX_EN, LOW);
    unsigned long t_start = micros();
    while ((micros() - t_start) < FRAMING_TIMEOUT)
    {
      while (Serial2.available())
      {
          _rx_buffer[rx_len++] = Serial2.read();
          if (rx_len >= 5 && rx_len == _rx_buffer[2] + 5)
          {  
              SerialUSB.println("Buffer size correct");
              break;
          }
          if (rx_len >= sizeof(_rx_buffer)) {
                  SerialUSB.println("Buffer of");
                  // Safety: avoid buffer overflow
                  break;
              }
      }
    }
    if (validate_crc(_rx_buffer, rx_len))
    {
        status.soc = _rx_buffer[4];
        SerialUSB.println(status.soc);
    }
    else{
      SerialUSB.println("Not validated");
    }

}



void bms_step()
{
  const uint32_t now = micros();
  switch (bms_state)
  {
    case BMS_START:
      if (now - last_request_time >= 1000000)
      {
        digitalWrite(PIN_TX_EN, HIGH);
        send_bms_read_packet(0x00, 0x0A);
        response_start_time = now;
        rx_len = 0;
        bms_state = BMS_TX_IDLE;
      }
      break;
    
    case BMS_TX_IDLE:
      if (SERCOM4->USART.INTFLAG.bit.TXC)
      {
        digitalWrite(PIN_TX_EN, LOW);
        bms_state = BMS_RX_IDLE;
      }
      break;

    case BMS_RX_IDLE:
      // if (Serial2.available())
      // {
      //   _rx_buffer[rx_len++] = Serial2.read();
      //   if (rx_len >= 5 && rx_len == _rx_buffer[2] + 5)
      //   {
      //     bms_state = BMS_PARSE;
      //   }
      // }
      while (Serial2.available())
      {
          _rx_buffer[rx_len++] = Serial2.read();
          // SerialUSB.print("rx_len: ");
          // SerialUSB.println(rx_len);

          if (rx_len >= 5 && rx_len == _rx_buffer[2] + 5)
          {
              bms_state = BMS_PARSE;
              break;
          }

          if (rx_len >= sizeof(_rx_buffer)) {
              // Safety: avoid buffer overflow
              SerialUSB.println("RX buffer overflow!");
              bms_state = BMS_START;
              last_request_time = now;
              break;
          }
      }
      if (now - response_start_time > FRAMING_TIMEOUT)
      {
          SerialUSB.println("BMS Timeout");
          bms_state = BMS_START;
          last_request_time = now;
      }
      break;
    
    case BMS_PARSE:
      if (validate_crc(_rx_buffer, rx_len))
      {
        status = parse_bms_response(_rx_buffer);
      }
      bms_state = BMS_START;
      last_request_time = now;
      break;
  }
}

bool validate_crc(uint8_t* buf, uint8_t len)
{
    if (len < 3) return false;
    uint16_t crc_calc = _crc->Modbus(buf, 0, len - 2);
    uint16_t crc_recv = (buf[len - 1] << 8) | buf[len - 2];
    return crc_calc == crc_recv;
}

BmsStatus parse_bms_response(uint8_t* buf)
{
    BmsStatus s;
    s.voltage = ((buf[3] << 8) | buf[4]) * 0.01f;
    s.current = -((int16_t)((buf[5] << 8) | buf[6])) * 0.1f;
    s.soh = buf[15];
    s.soc = buf[16];
    return s;
}


void send_bms_read_packet(uint16_t reg_add, uint16_t reg_count)
{
    
    uint8_t modbus_tx_buffer[8];
    _crc->clearCrc();
    modbus_tx_buffer[0] = 0x01; //BMS address
    modbus_tx_buffer[1] = 0x03; //Read byte
    modbus_tx_buffer[2] = (reg_add >> 8) & 0xFF; //Register address high byte
    modbus_tx_buffer[3] = reg_add & 0xFF; //Register address low byte
    modbus_tx_buffer[4] = (reg_count >> 8) & 0xFF; //Register count hi byte
    modbus_tx_buffer[5] = reg_count & 0xFF; //Register count low byte

    uint16_t crc_t = _crc->Modbus(modbus_tx_buffer, 0, 6);// start at byte 0 and go to byte 6

    modbus_tx_buffer[6] = crc_t & 0xFF;   // CRC low byte
    modbus_tx_buffer[7] = (crc_t >> 8);  // CRC high byte
    Serial2.write(modbus_tx_buffer, sizeof(modbus_tx_buffer));
}

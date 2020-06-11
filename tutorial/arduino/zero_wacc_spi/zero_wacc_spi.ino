/*
  -------------------------------------------------------------
  Hello Robot - Hello Zero SPI Slave

  All materials released under the GNU General Public License v3.0 (GNU GPLv3).
  https://www.gnu.org/licenses/gpl-3.0.html      

  Copyright (c) 2020 by Hello Robot Inc. 
  --------------------------------------------------------------
*/


#include <Arduino.h>


struct __attribute__ ((packed)) Calc_Command{
  float var1;
  float var2;
  uint8_t op;
};

struct __attribute__ ((packed)) Calc_Status{
  float result;
};

Calc_Command cmd;
Calc_Status stat;

float my_calc(uint8_t op, float var1, float var2)
{
  if (op == 0)
    return var1 + var2;
  if (op == 1)
    return var1 * var2;
  if (op == 2)
    return var1 / var2;
  return 0;
}

void setup (void)
{
 pinMode(MISO, OUTPUT);
 SPI_Slave_Setup();
}  


uint8_t buf_in[9];
uint8_t buf_in_idx=0;
uint8_t buf_out[4];
uint8_t buf_out_idx=0;


void OnTransmissionStart()
{
  buf_in_idx = 0;
  buf_out_idx=0;
}

uint8_t spi_byte_received(uint8_t c)
{
  uint8_t out;
  if ((char)c=='X') //Start byte
  {
    buf_in_idx=0;
    buf_out_idx=0;
  }
  else
    buf_in[buf_in_idx++]=c;
     

  if(buf_out_idx<sizeof(Calc_Status))
      out=buf_out[buf_out_idx++];
   else
      out=0;

  if(buf_in_idx==sizeof(Calc_Command))
  {
    memcpy((uint8_t *) (&cmd), buf_in, sizeof(Calc_Command));
    stat.result=my_calc(cmd.op,cmd.var1,cmd.var2);
    memcpy(buf_out, (uint8_t *) (&stat), sizeof(Calc_Status));
    buf_in_idx=0;
    buf_out_idx=0;
  }
  return out;
}


void loop (void)
{

}  



///////////////////////// SPI SLAVE ///////////////////
/*
 * The following code configures an Arduino Zero to operate
 * As an SPI slave. 
 * This code is modified from: https://forum.arduino.cc/index.php?topic=488659.0
 */

#include "wiring_private.h"

enum spi_transfer_mode
{
  SERCOM_SPI_TRANSFER_MODE_0 = 0,
  SERCOM_SPI_TRANSFER_MODE_1 = SERCOM_SPI_CTRLA_CPHA,
  SERCOM_SPI_TRANSFER_MODE_2 = SERCOM_SPI_CTRLA_CPOL,
  SERCOM_SPI_TRANSFER_MODE_3 = SERCOM_SPI_CTRLA_CPHA | SERCOM_SPI_CTRLA_CPOL,
};

enum spi_frame_format
{
  SERCOM_SPI_FRAME_FORMAT_SPI_FRAME      = SERCOM_SPI_CTRLA_FORM(0),
  SERCOM_SPI_FRAME_FORMAT_SPI_FRAME_ADDR = SERCOM_SPI_CTRLA_FORM(2),
};

enum spi_character_size
{
  SERCOM_SPI_CHARACTER_SIZE_8BIT = SERCOM_SPI_CTRLB_CHSIZE(0),
  SERCOM_SPI_CHARACTER_SIZE_9BIT = SERCOM_SPI_CTRLB_CHSIZE(1),
};

enum spi_data_order
{
  SERCOM_SPI_DATA_ORDER_LSB = SERCOM_SPI_CTRLA_DORD,
  SERCOM_SPI_DATA_ORDER_MSB   = 0,
};

void SPI_Slave_Setup()
{
  // Set pin modes
  pinPeripheral(MISO, PIO_SERCOM_ALT);
  pinPeripheral(MOSI, PIO_SERCOM_ALT);
  pinPeripheral(SCK,  PIO_SERCOM_ALT);
  pinPeripheral(A2,   PIO_SERCOM_ALT);
  
  sercom4.disableSPI();
  sercom4.resetSPI();

  // Setting up NVIC
  NVIC_EnableIRQ(SERCOM4_IRQn);
  NVIC_SetPriority(SERCOM4_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
  
  // Setting the CTRLA register
  //  - MISO: PAD[0], PA12, D22
  //  - MOSI: PAD[2], PB10, D23
  //  - SCK:  PAD[3], PB11, D24
  //  - SS:   PAD[1], PB09, A2
  SERCOM4->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_SLAVE        |   // Set as slave
              SERCOM_SPI_CTRLA_DIPO(SERCOM_RX_PAD_2) |   // Set input register
              SERCOM_SPI_CTRLA_DOPO(SPI_PAD_0_SCK_3) |   // Set output register
              SERCOM_SPI_TRANSFER_MODE_0             |   // Use SPI Mode 0
              SERCOM_SPI_FRAME_FORMAT_SPI_FRAME      |   // Disable Address message format
              SERCOM_SPI_DATA_ORDER_MSB              ;   // Most significant bit first
  
  // Setting the CTRLB register
  SERCOM4->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(SPI_CHAR_SIZE_8_BITS) |   // 8 bit message size
              SERCOM_SPI_CTRLB_SSDE              |   // Slave Select Low Detection Enable
              SERCOM_SPI_CTRLB_RXEN                         |   // Active the SPI receiver
              SERCOM_SPI_CHARACTER_SIZE_8BIT                ;   // 8bit message size

  // Set up SPI interrupts
  SERCOM4->SPI.INTENSET.reg = SERCOM_SPI_INTFLAG_SSL   |   //Enable Slave Select low interrupt
                  SERCOM_SPI_INTFLAG_RXC   |   //Receive complete interrupt
                  SERCOM_SPI_INTFLAG_TXC   |
                  SERCOM_SPI_INTFLAG_ERROR |
                  SERCOM_SPI_INTFLAG_DRE   ;
  
  // Enable SPI
  sercom4.enableSPI();
}

void SERCOM4_Handler(void)
{
  /*
  *  1. ERROR
  *   Occurs when the SPI receiver has one or more errors.
  */
  if (SERCOM4->SPI.INTFLAG.bit.ERROR)
  {
    SERCOM4->SPI.INTFLAG.bit.ERROR = 1;
  }
  
  
  /*
  *  2. SSL: Slave Select Low
  *   Occurs when SS goes low
  */
  if (SERCOM4->SPI.INTFLAG.bit.SSL)
  {
    OnTransmissionStart();
    SERCOM4->SPI.INTFLAG.bit.SSL = 1;
  }
  

  /*
  *  3. TXC: Transmission Complete
  *   Occurs when SS goes high. The transmission is complete.
  */
  if (SERCOM4->SPI.INTFLAG.bit.TXC)
  {
    SERCOM4->SPI.INTFLAG.bit.TXC = 1;
  }
  
  
  /*
  *  4. RXC: Receive Complete
  *   Occurs after a character has been full stored in the data buffer. It can now be retrieved from DATA.
  *   Always reply with a byte
  */

  if (SERCOM4->SPI.INTFLAG.bit.RXC)
  {
    uint8_t readData = (uint8_t)SERCOM4->SPI.DATA.reg;
    SERCOM4->SPI.INTFLAG.bit.RXC = 1;
    SERCOM4->SPI.DATA.reg =spi_byte_received(readData);
  }
  
}

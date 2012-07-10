/***************************************************************
 *
 * OpenBeacon.org - 3D acceleration sensor support
 *
 * Copyright 2010 Milosch Meriac <meriac@openbeacon.de>
 *
 ***************************************************************

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/
#include <openbeacon.h>
#include "3d_acceleration.h"
#include "spi.h"
#include "pmu.h"

#define ACC_XOFFL 0x10
#define ACC_YOFFL 0x12
#define ACC_ZOFFL 0x14

extern uint8_t accel;

static void
acc_reg_write (uint8_t addr, uint8_t data)
{
  uint8_t tx[2];

  /* assemble SPI write request */
  tx[0] = 0x80 | addr << 1;
  tx[1] = data;

  /* transmit packet */
  spi_txrx (SPI_CS_ACC3D, tx, sizeof (tx), NULL, 0);
}

static uint8_t
acc_reg_read (uint8_t addr)
{
  uint8_t tx[2], rx[2];

  /* assemble SPI read request */
  tx[0] = addr << 1;
  tx[1] = 0;
  /* transmit packet */
  spi_txrx (SPI_CS_ACC3D, tx, sizeof (tx), rx, sizeof (rx));

  return rx[1];
}

int16_t
acc_reg_read10( uint8_t address )
{
  int16_t value;
  value = acc_reg_read(address) << 6; // low byte
  value |= acc_reg_read(address + 1) << 14; // high byte
  return value >> 6;
}

void
acc_xyz_read (int8_t *x, int8_t *y, int8_t *z)
{
  /* dummy read - FIXME */
  acc_reg_read (0);

  /*  get acceleration values */
  *x = acc_reg_read (6);
  *y = acc_reg_read (7);
  *z = acc_reg_read (8);
}

void
acc_xyz_read10 (int16_t *x, int16_t *y, int16_t *z)
{
  /*  get acceleration values */
  *x = acc_reg_read10 (0);
  *y = acc_reg_read10 (2);
  *z = acc_reg_read10 (4);
}

void
acc_calibrate_offset( int16_t x, int16_t y, int16_t z )
{
    int16_t offset_x = ( 0 - x ) << 1;
    int16_t offset_y = ( 0 - y ) << 1;
    /* 1g offset */
    //int16_t offset_z = ( 64 - z ) << 1;
    /* 0g offset */
    int16_t offset_z = ( 0 - z ) << 1;
    acc_reg_write( ACC_XOFFL, offset_x & 0x00ff );
    acc_reg_write( ACC_XOFFL + 1, ( offset_x & 0xff00 ) >> 8 );
    acc_reg_write( ACC_YOFFL, offset_y & 0x00ff );
    acc_reg_write( ACC_YOFFL + 1, ( offset_y & 0xff00 ) >> 8 );
    acc_reg_write( ACC_ZOFFL, offset_z & 0x00ff );
    acc_reg_write( ACC_ZOFFL + 1, ( offset_z & 0xff00 ) >> 8 );
}

void
acc_status (void)
{
  int8_t x, y, z;

  acc_xyz_read (&x, &y, &z);

  debug_printf (" * 3D_ACC: X=%04i Y=%04i Z=%04i\n", x, y, z);
}

uint8_t acc_source(void) {
  return acc_reg_read(0x0A);
}

void acc_clear(void) {
  acc_reg_write(0x17, 0x03);
  acc_reg_write(0x17, 0x00);
}

uint8_t
acc_IRQ (void)
{
  return !GPIOGetValue (ACC_IRQ_CPU_PORT, ACC_IRQ_CPU_PIN);
}

void
WAKEUP_IRQHandlerPIO1_11 (void)
{
  /* Clear pending IRQ */
  LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO1_11;
  //NVIC_ClearPendingIRQ(WAKEUP_PIO1_11_IRQn);

  accel = 1;

  pmu_cancel_timer();
}

void
acc_power (uint8_t enabled)
{
  ///* switch to input if enabled */
//  if(enabled)
//    GPIOSetDir (1, 11, 0);

  /* dummy read - maybe fixed ? */
  acc_reg_read (0);
  /* set 3D acceleration sensor active, enable level detection */

  //acc_reg_write(0x16, enabled ? (1 << 6 | 0x02) : 0x00);
  //acc_reg_write(0x16, enabled ? 0x02 : 0x00);



//  acc_reg_write(0x18, 0x03 << 3);
//  acc_reg_write(0x19, 0x00);
//
//  /* set threshold level */
//  acc_reg_write(0x1A, 0x15);

  // $16: Mode Control Register
  // 0x0: Standby Mode
  // 0x1: Measurement Mode
  // 0x2: Level Detection Mode
  // 0x3: Pulse Detection
  int mode = 0x2;

  // 0x0: 8g
  // 0x8: 4g
  // 0x4: 2g
  int glv = 0x0;

  acc_reg_read(0x16);
  acc_reg_write(0x16, enabled ? (1 << 6 | glv | mode) : 0x00);


  /*
  // Optimal Settings for Motion using Level Detection
  // 1. THOPT=0 Absolute Condition
  // 2. ZDA=1 Disable Z, YDA=0 Enable Y, XDA=0 Enable X
  acc_reg_write(0x18, (acc_reg_read(0x18) & 0x87) | 0x20);
  // 3. Positive OR Logic Clear LDPL
  acc_reg_write(0x19, acc_reg_read(0x19) & 0xfe);
  // Set Threshold to 2 g
  //acc_reg_write(0x1a, 0x20);
  acc_reg_write(0x1a, 0x7);
  */

  // Custom Settings for Motion using Level Detection
  // 1. THOPT=0 Absolute Condition
  // 2. ZDA=0 Enable Z, YDA=0 Enable Y, XDA=0 Enable X
  acc_reg_write(0x18, (acc_reg_read(0x18) & 0x87) | 0x20);
  // 3. Positive AND Logic Set LDPL
  acc_reg_write(0x19, acc_reg_read(0x19) | 0x01);
  // Set Threshold to 2 g
  //acc_reg_write(0x1a, 0x20);
  acc_reg_write(0x1a, 0x7);


  /*
  // Optimal Settings for Single Pulse Detection
  // 1. Positive OR Logic PDPL=0
  acc_reg_write(0x19, acc_reg_read(0x19) & 0xfd);
  // 2. X,Y,Z enabled
  acc_reg_write(0x18, acc_reg_read(0x18) & 0x87);
  // 3. PDTH (Pulse Threshold) set to 4 g
  acc_reg_write(0x1b, 0x40);
  // 4. PD (Pulse Duration) set to 8 ms
  acc_reg_write(0x1c, 0x80);
  */

//  // Latency time
//  acc_reg_write(0x1d, 0x01);
//  // Time Window for 2nd Pulse Value
//  acc_reg_write(0x1e, 0xff);

  acc_clear();

  /* switch to output after shutting down */
//  if(!enabled)
//  {
//    GPIOSetDir (1, 11, 1);
//    GPIOSetValue (1, 11, 0);
//  }
}

void
acc_init (uint8_t enabled)
{
  /* PIO, PIO0_4 in standard IO functionality */
  LPC_IOCON->PIO0_4 = 1 << 8;

  LPC_IOCON->PIO1_11 = 0x80;
  GPIOSetDir (ACC_IRQ_CPU_PORT, ACC_IRQ_CPU_PIN, 0);
  /* setup IRQ generation */
  //LPC_SYSCON->STARTAPRP0 = (LPC_SYSCON->STARTAPRP0 & ~STARTxPRP0_PIO1_11);
  LPC_SYSCON->STARTAPRP0 = LPC_SYSCON->STARTAPRP0 | STARTxPRP0_PIO1_11;
  LPC_SYSCON->STARTERP0 = LPC_SYSCON->STARTAPRP0 | STARTxPRP0_PIO1_11;
  LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO1_11;
  NVIC_EnableIRQ (WAKEUP_PIO1_11_IRQn);

  /* setup SPI chipselect pin */
  spi_init_pin (SPI_CS_ACC3D);

  /* propagate power settings */
  acc_power (enabled);
}

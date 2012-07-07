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

void
acc_xyz_read (int8_t *x, int8_t *y, int8_t *z)
{
  /* dummy read - FIXME */
  acc_reg_read (0);

  /*  get acceleration values */
  *x = (int8_t) acc_reg_read (6);
  *y = (int8_t) acc_reg_read (7);
  *z = (int8_t) acc_reg_read (8);
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

  accel = 1;

  pmu_cancel_timer();
}

void
acc_power (uint8_t enabled)
{
  ///* switch to input if enabled */
  if(enabled)
    GPIOSetDir (1, 11, 0);

  /* dummy read - maybe fixed ? */
  acc_reg_read (0);
  /* set 3D acceleration sensor active, enable level detection */

  acc_reg_write(0x16, enabled ? 0x02 : 0x00);

  acc_reg_write(0x18, 0x03 << 3);
  acc_reg_write(0x19, 0x00);

  /* set threshold level */
  acc_reg_write(0x1A, 0x1F);

  acc_clear();

  /* switch to output after shutting down */
  if(!enabled)
  {
    GPIOSetDir (1, 11, 1);
    GPIOSetValue (1, 11, 0);
  }
}

void
acc_init (uint8_t enabled)
{
  /* PIO, PIO0_4 in standard IO functionality */
  LPC_IOCON->PIO0_4 = 1 << 8;

  LPC_IOCON->PIO1_11 = 0;
  GPIOSetDir (ACC_IRQ_CPU_PORT, ACC_IRQ_CPU_PIN, 0);
  /* setup IRQ generation */
  NVIC_EnableIRQ (WAKEUP_PIO1_11_IRQn);
  LPC_SYSCON->STARTAPRP0 = (LPC_SYSCON->STARTAPRP0 & ~STARTxPRP0_PIO1_11);
  LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO1_11;
  LPC_SYSCON->STARTERP0 |= STARTxPRP0_PIO1_11;

  /* setup SPI chipselect pin */
  spi_init_pin (SPI_CS_ACC3D);

  /* propagate power settings */
  acc_power (enabled);
}

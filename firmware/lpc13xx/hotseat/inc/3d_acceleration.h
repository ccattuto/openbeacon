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

#ifndef __3D_ACCELERATION_H__
#define __3D_ACCELERATION_H__

#define ACC_IRQ_CPU_PORT	1
#define ACC_IRQ_CPU_PIN		11

extern void acc_init (uint8_t enabled);
extern void acc_power (uint8_t enabled);
extern uint8_t acc_source (void);
extern void acc_clear(void);
extern void acc_status (void);
extern void acc_xyz_read (int8_t *x, int8_t *y, int8_t *z);
extern void acc_calibrate_offset( int16_t x, int16_t y, int16_t z );
extern void acc_xyz_read10 (int16_t *x, int16_t *y, int16_t *z);

extern uint8_t acc_IRQ(void);

#endif/*__3D_ACCELERATION_H__*/

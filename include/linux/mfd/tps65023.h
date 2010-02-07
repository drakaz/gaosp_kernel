/* Driver for TI PMIC TPS65023.
 *
 * Copyright (c) 2009 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 *
 */

#ifndef __LINUX_I2C_TPS65023_H
#define __LINUX_I2C_TPS65023_H

#ifndef CONFIG_TPS65023
/* Set the output voltage for the DCDC1 convertor */
static int tps65023_set_dcdc1_level(int mvolts)  { return -ENODEV; }

/* Read the output voltage from the DCDC1 convertor */
static int tps65023_get_dcdc1_level(int *mvolts) { return -ENODEV; }
#else
/* Set the output voltage for the DCDC1 convertor */
extern int tps65023_set_dcdc1_level(int mvolts);

/* Read the output voltage from the DCDC1 convertor */
extern int tps65023_get_dcdc1_level(int *mvolts);
#endif

#endif

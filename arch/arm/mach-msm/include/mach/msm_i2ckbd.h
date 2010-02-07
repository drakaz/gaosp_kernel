/*
 *  Copyright (c) 2008-2009 QUALCOMM USA, INC.
 *  
 *  All source code in this file is licensed under the following license
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, you can find it at http://www.fsf.org
 */

#ifndef _MSM_I2CKBD_H_
#define _MSM_I2CKBD_H_

struct msm_i2ckbd_platform_data {
	uint8_t hwrepeat;
	uint8_t scanset1;
	int  gpioreset;
	int  gpioirq;
	int  (*gpio_setup) (void);
	void (*gpio_shutdown)(void);
};

#endif

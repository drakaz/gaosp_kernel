/*
 * Copyright (c) 2008-2009 QUALCOMM USA, INC.
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
 */

#ifndef MT9D112_H
#define MT9D112_H

#include <mach/board.h>
#include <mach/camera.h>

extern struct mt9d112_reg_t mt9d112_regs;

enum mt9d112_width_t {
	WORD_LEN,
	BYTE_LEN
};

struct mt9d112_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
	enum mt9d112_width_t width;
	unsigned short mdelay_time;
};


struct mt9d112_reg_t {

	struct register_address_value_pair_t const *prev_snap_reg_settings;
	uint16_t prev_snap_reg_settings_size;
	struct register_address_value_pair_t const
	  *noise_reduction_reg_settings;
	uint16_t noise_reduction_reg_settings_size;
	struct mt9d112_i2c_reg_conf const *plltbl;
	uint16_t plltbl_size;
	struct mt9d112_i2c_reg_conf const *stbl;
	uint16_t stbl_size;
	struct mt9d112_i2c_reg_conf const *rftbl;
	uint16_t rftbl_size;
};


#endif /* MT9D112_H */

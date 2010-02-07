/*
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

#ifndef ARDHELPER_H
#define ARDHELPER_H

#include "msm8k_ardi.h"
#include "msm8k_cad.h"

s32 codec_disable(enum codec_enum_type codec_type, u32 dev_type, u32 dev_id);
s32 codec_enable(enum codec_enum_type codec_type, u32 dev_type, u32 dev_id);
u32 get_device_id(u32 cad_device_requested);
enum codec_enum_type get_codec_type(u32 device_in_use);
enum ard_ret_enum_type test_clocks(void);

#endif

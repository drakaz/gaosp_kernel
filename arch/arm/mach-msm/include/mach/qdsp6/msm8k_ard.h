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

#ifndef ARD_H
#define ARD_H

#include "msm8k_cad_module.h"

enum sample_rate_type {
	SAMPLE_RATE_NONE,
	SAMPLE_RATE_8000,
	SAMPLE_RATE_11025,
	SAMPLE_RATE_12000,
	SAMPLE_RATE_16000,
	SAMPLE_RATE_22050,
	SAMPLE_RATE_24000,
	SAMPLE_RATE_32000,
	SAMPLE_RATE_44100,
	SAMPLE_RATE_48000,
	SAMPLE_RATE_MAX
};


void print_data(u32 session_id);

s32 cad_ard_init(struct cad_func_tbl_type **func_ptr_tbl);

s32 ard_open(s32 session_id,
	struct cad_open_struct_type *open_param);

s32 ard_close(s32 session_id);

s32 ard_ioctl(s32 session_id, u32 cmd_code, void *cmd_buf, u32 cmd_len);

s32 ard_read(s32 session_id, struct cad_buf_struct_type *buf);

s32 ard_write(s32 session_id, struct cad_buf_struct_type *buf);


#endif

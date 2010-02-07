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

#ifndef CAD_H
#define CAD_H

#include <linux/kernel.h>

#define CAD_RES_SUCCESS       0
#define CAD_RES_FAILURE      -1
#define CAD_RES_UNSUPPORTED  -2

#define CAD_FORMAT_PCM      0x00
#define CAD_FORMAT_ADPCM    0x01
#define CAD_FORMAT_MP3      0x02
#define CAD_FORMAT_RA       0x03
#define CAD_FORMAT_WMA      0x04
#define CAD_FORMAT_AAC      0x05
#define CAD_FORMAT_MIDI     0x07
#define CAD_FORMAT_YADPCM   0x08
#define CAD_FORMAT_QCELP    0x09
#define CAD_FORMAT_AMRWB    0x0A
#define CAD_FORMAT_AMRNB    0x0B
#define CAD_FORMAT_EVRC     0x0C
#define CAD_FORMAT_DTMF     0x0D

#define CAD_OPEN_OP_READ   0x01
#define CAD_OPEN_OP_WRITE  0x02

#define CAD_OPEN_OP_DEVICE_CTRL  0x04


struct cad_open_struct_type {
	u32  op_code;
	u32  format;
};


struct cad_buf_struct_type {
	void    *buffer;
	u32     phys_addr;
	u32     max_size;
	u32     actual_size;
	s64	time_stamp;
};


extern u8 *g_audio_mem;
extern u32 g_audio_base;

s32 cad_open(struct cad_open_struct_type *open_param);

s32 cad_close(s32 driver_handle);

s32 cad_write(s32 driver_handle, struct cad_buf_struct_type *buf);

s32 cad_read(s32 driver_handle, struct cad_buf_struct_type *buf);

s32 cad_ioctl(s32 driver_handle, u32 cmd_code, void *cmd_buf,
	u32 cmd_buf_len);

int audio_switch_device(int new_device);

int audio_set_device_volume(int vol);

int audio_set_device_mute(int mute_info);

#endif

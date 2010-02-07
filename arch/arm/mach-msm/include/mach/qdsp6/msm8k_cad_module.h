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

#ifndef CADMODULE_H
#define CADMODULE_H

#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_cad_event.h>

#define CAD_MAX_SESSION         10


typedef s32 (*cad_open_func_ptr_type)(s32 session_id,
	struct cad_open_struct_type *open_param);

typedef s32 (*cad_close_func_ptr_type)(s32 session_id);

typedef s32 (*cad_write_func_ptr_type)(s32 session_id,
	struct cad_buf_struct_type *buf);

typedef s32 (*cad_read_func_ptr_type)(s32 session_id,
	struct cad_buf_struct_type *buf);

typedef s32 (*cad_ioctl_func_ptr_type)(s32 session_id, u32 cmd_code,
	void *cmd_buf, u32 cmd_len);

struct cad_func_tbl_type {
	cad_open_func_ptr_type          open;
	cad_close_func_ptr_type         close;
	cad_write_func_ptr_type         write;
	cad_read_func_ptr_type          read;
	cad_ioctl_func_ptr_type         ioctl;
};


s32 cad_resource_init(struct cad_func_tbl_type **func_tbl);

s32 cad_resource_dinit(void);

s32 cad_audio_dec_init(struct cad_func_tbl_type **func_tbl);

s32 cad_audio_dec_dinit(void);

s32 cad_audio_enc_init(struct cad_func_tbl_type **func_tbl);

s32 cad_audio_enc_dinit(void);

s32 cad_voice_dec_init(struct cad_func_tbl_type **func_tbl);

s32 cad_voice_dec_dinit(void);

s32 cad_voice_enc_init(struct cad_func_tbl_type **func_tbl);

s32 cad_voice_enc_dinit(void);

s32 cad_ard_init(struct cad_func_tbl_type **func_tbl);

s32 cad_ard_dinit(void);

s32 cad_dtmf_init(struct cad_func_tbl_type **func_tbl);

s32 cad_dtmf_dinit(void);

#endif

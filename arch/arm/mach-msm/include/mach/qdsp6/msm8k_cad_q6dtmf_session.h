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

#ifndef _QDSP6DTMFSESSION_H_
#define _QDSP6DTMFSESSION_H_

#include <mach/qdsp6/msm8k_cad_q6dtmf_drvi.h>

s32 cad_dtmf_session_init(struct q6dtmf_session *self);
s32 cad_dtmf_session_dinit(struct q6dtmf_session *self);
s32 cad_dtmf_session_open(struct q6dtmf_session *self, s32 session_id,
				struct cad_open_struct_type *open_param);

s32 cad_dtmf_session_close(struct q6dtmf_session *self);
s32 cad_dtmf_session_ioctl(struct q6dtmf_session *self, s32 cmd_code,
				void *cmd_buf, s32 cmd_len);

#endif

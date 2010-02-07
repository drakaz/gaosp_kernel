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

#ifndef _QDSP6AUDIOENCSESSION_H_
#define _QDSP6AUDIOENCSESSION_H_

#include <mach/qdsp6/msm8k_cad_q6enc_drvi.h>

s32 cad_q6enc_session_init(struct q6_enc_session_data *self);
s32 cad_q6enc_session_dinit(struct q6_enc_session_data *self);
s32 cad_q6enc_session_open(struct q6_enc_session_data *self,
			s32 session_id,
			struct cad_open_struct_type *open_param);

s32 cad_q6enc_session_read(struct q6_enc_session_data *self,
			struct cad_buf_struct_type *buf);

s32 cad_q6enc_session_close(struct q6_enc_session_data *self);
s32 cad_q6enc_session_ioctl(struct q6_enc_session_data *self,
				u32 cmd,
				void *cmd_buf);

#endif

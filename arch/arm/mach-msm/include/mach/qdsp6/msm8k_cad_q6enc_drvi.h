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

#ifndef _QDSP6AUDIOENCDRIVERI_H_
#define _QDSP6AUDIOENCDRIVERI_H_

#include <mach/qdsp6/msm8k_cad_module.h>

#define		Q6_ENC_MAX_SESSION_COUNT	4
#define		Q6_ENC_BUF_PER_SESSION		4
#define		Q6_ENC_BUF_MAX_SIZE		(1024 * 100)

enum q6_enc_session_state {
	Q6_ENC_STATE_RESET = 0,
	Q6_ENC_STATE_INIT,
	Q6_ENC_STATE_PROCESS,
	Q6_ENC_STATE_CLOSING
};

struct q6_enc_session_buf_node {
	u8					*buf;
	u32					phys_addr;
	u32					buf_len;
	struct q6_enc_session_buf_node		*next;
};

struct q6_enc_session_data {
	struct mutex			session_mutex;
	u32				session_id;
	u32				buf_size;
	struct semaphore		buf_done_evt;
	s32				signal_buf_done;
	struct semaphore		all_buf_done_evt;
	s32				signal_all_buf_done;
	struct q6_enc_session_buf_node	*free_nodes;
	struct q6_enc_session_buf_node	*used_nodes;
	struct q6_enc_session_buf_node	*full_nodes_head;
	struct q6_enc_session_buf_node	*full_nodes_tail;
	enum q6_enc_session_state	session_state;
	struct q6_enc_session_data	*next;
	struct cad_event_struct_type	cb_data;
	u8				*shared_buffer;
};

struct q6_audio_enc_data {
	struct q6_enc_session_data	*q6_enc_free_sessions;
	struct q6_enc_session_data	*q6_enc_used_sessions;
};

#endif

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

#ifndef _MSM8K_CAD_Q6DEC_DRVI_H_
#define _MSM8K_CAD_Q6DEC_DRVI_H_

#include <linux/mutex.h>
#include <linux/semaphore.h>

#include <mach/qdsp6/msm8k_cad_module.h>


#define Q6_DEC_MAX_STREAM_COUNT			4
#define Q6_DEC_BUFFER_NUM_PER_STREAM		4
#define Q6_DEC_BUFFER_SIZE_MAX			(1024*100)

enum q6dec_session_state {
	Q6_DEC_RESET		= 0x0,
	Q6_DEC_INIT		= 0x1,
	Q6_DEC_READY		= 0x2,
	Q6_DEC_FLUSHING		= 0x4,
	Q6_DEC_CLOSING		= 0x8,
	Q6_DEC_STATE_MAX	= 0x7FFFFFFF
};

struct q6dec_sesson_buffer_node;
struct q6dec_sesson_buffer_node {
	u8					*buf;
	u32					phys_addr;
	struct q6dec_sesson_buffer_node		*next;
};

struct q6dec_session_data;
struct q6dec_session_data {
	u8					*shared_buf;
	struct mutex				session_mutex;
	u32					session_id;
	u32					buffer_size;
	struct semaphore			buf_done_sem;
	struct semaphore			all_buf_done_sem;
	struct q6dec_sesson_buffer_node		*free_buf_list;
	struct q6dec_sesson_buffer_node		*used_buf_list;
	enum q6dec_session_state		session_state;
	struct q6dec_session_data		*next;
	/* debug variables */
	u32					use_counter;
	u32					ret_counter;
	/* flag to tell if we need flush before close */
	u32					need_flush;
	/* flag for buffer done event */
	u32					need_buffer_done;
};

struct q6dec_data {
	u32				used_stream_count;
	struct q6dec_session_data	*free_session_list;
	struct q6dec_session_data	*used_session_list;
};

#endif

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

#ifndef _QDSP6_RPC_TYPE_H
#define _QDSP6_RPC_TYPE_H

#include <linux/kernel.h>

#define CAD_RPC_PROCESSOR_MASK  0xFF000000

/* These define the sync buffer and async buffer number	*/
#define CAD_RPC_ASYNC_BUFFER_NUM         5
#define CAD_RPC_SYNC_BUFFER_NUM          1

enum cad_rpc_buffer_type {
	CAD_RPC_SYNC_BUFFER = 0,
	CAD_RPC_ASYNC_BUFFER
};


enum cad_rpc_process_type {
	CAD_RPC_ARM9  = 0,
	CAD_RPC_ARM11 = 1,
	CAD_RPC_PROCESSPR_MAX
};


struct cad_rpc_config_info {
	void    *cb_evt;
	u32	local_trigger_evt;
	u32     session_id;
	u32     processor_id;
};


struct cad_rpc_buffer_info {
	u32     buffer_type;
	u32     processor_id;
	u32     flushed_buffer_index;
};

#endif  /* _QDSP6_RPC_TYPE_H */

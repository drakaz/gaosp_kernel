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

#ifndef _MSM8K_CAD_RPC_H_
#define _MSM8K_CAD_RPC_H_

#include <linux/kernel.h>

#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_cad_itypes.h>


typedef void (*RPC_CB_FCN)(struct cadi_evt_struct_type *returnEvent,
				void *client_data);

s32 cad_rpc_init(u32 processor_id);
s32 cad_rpc_deinit(void);
s32 cad_rpc_reg_callback(u32 session_id, RPC_CB_FCN cbFCN, void *client_data);
s32 cad_rpc_dereg_callback(u32 session_id, RPC_CB_FCN cbFCN);

s32 cad_rpc_open(u32 session_id,
		u32 block_flag,        /* 0=none block, 1=block */
		struct cadi_open_struct_type *open_buf,
		struct cadi_evt_struct_type *ret_status);

s32 cad_rpc_read(u32 session_id,
		u32 block_flag,        /* 0=none block, 1=block */
		struct cad_buf_struct_type *read_buf,
		struct cadi_evt_struct_type *ret_status);

s32 cad_rpc_write(u32 session_id,
		u32 block_flag,       /* 0=none block, 1=block */
		struct cad_buf_struct_type *write_buf,
		struct cadi_evt_struct_type *ret_status);

s32 cad_rpc_ioctl(u32 session_id,
		u32 block_flag,       /* 0=none block, 1=block */
		u32 cmd_code,
		u8  *cmd_buf,
		u32 cmd_buf_len,
		struct cadi_evt_struct_type *ret_status);

s32 cad_rpc_close(u32 session_id,
		u32 block_flag,       /* 0=none block, 1=block */
		struct cadi_evt_struct_type *ret_status);

#endif /* _MSM8K_CAD_RPC_H_ */

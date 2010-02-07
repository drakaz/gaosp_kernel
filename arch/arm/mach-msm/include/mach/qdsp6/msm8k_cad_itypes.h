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

#ifndef CADITYPES_H
#define CADITYPES_H

#include <mach/qdsp6/msm8k_cad_event.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>


#define CADI_IOCTL_CMD_OPEN             0x0105c3f0
#define CADI_IOCTL_CMD_CLOSE            0x01075ab1
#define CADI_IOCTL_CMD_STOP             0x01075c54

#define CADI_IOCTL_CMD_DSP_PREP_DEV_CHG		0x01081d29
#define CADI_IOCTL_CMD_DSP_STANDBY		0x01081d2a
#define CADI_IOCTL_CMD_DSP_START		0x01081d2b


struct cadi_dev_chg_struct_type {
	u8	cad_dev_class;
	u8	cad_dev_type;
	u32	cad_old_device;
	u32	cad_new_device;
};


struct cadi_open_struct_type {
	struct cad_open_struct_type		cad_open;
	struct cad_stream_config_struct_type	cad_config;
	struct cad_stream_info_struct_type	cad_stream;
	struct cad_stream_device_struct_type	cad_device;
};


struct cadi_evt_data_struct_type {
	union {
		s32                                     svalue;
		u32                                     uvalue;
		struct cad_evt_avsync_struct_type       av_sync_data;
		struct cad_buf_struct_type              buf_data;
	};
};

struct cadi_evt_hdr_struct_type {
	s32             cad_handle;
	u32             cmd_event_id;
	s32             status;
};

struct cadi_evt_struct_type {
	struct cadi_evt_hdr_struct_type         cad_event_header;
	u32                                     data_len;
	struct cadi_evt_data_struct_type        cad_event_data;
};


typedef void (*cadi_evt_cb_func_type)(struct cadi_evt_struct_type *evt_packet,
	void *client_data);

struct cadi_evt_cb_struct_type {
	cadi_evt_cb_func_type   callback;
	void                    *client_data;
};

#endif

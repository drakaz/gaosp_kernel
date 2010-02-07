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

#ifndef CADEVENT_H
#define CADEVENT_H


#define CAD_EVT_STATUS_BUF_DONE				0x01076062

#define CAD_EVT_STATUS_BUF_UNDERRUN			0x01076061

#define CAD_EVT_STATUS_BUF_OVERFLOW			0x01076060

#define CAD_EVT_AV_SYNC					0x0107605f


struct cad_evt_avsync_struct_type {
	u64		num_of_samples;
	u64		num_of_bytes;
	u64		sample_rate;
};

#endif

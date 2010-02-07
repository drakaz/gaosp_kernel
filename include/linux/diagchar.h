/* include/linux/diagchar.h */

/* Copyright (c) 2008 QUALCOMM USA, INC.
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
*/

#ifndef DIAGCHAR_SHARED
#define DIAGCHAR_SHARED

#define MSG_MASKS_TYPE 1
#define LOG_MASKS_TYPE 2
#define EVENT_MASKS_TYPE 4
#define PKT_TYPE 8

/* different values that go in for diag_data_type */
#define DATA_TYPE_EVENT         0
#define DATA_TYPE_F3            1
#define DATA_TYPE_LOG           2
#define DATA_TYPE_RESPONSE      3


struct bindpkt_params {
	uint16_t cmd_code;
	uint16_t subsys_id;
	uint16_t cmd_code_lo;
	uint16_t cmd_code_hi;
	uint16_t proc_id;
	uint32_t event_id;
	uint32_t log_code;
	uint32_t client_id;
};

#define MAX_SYNC_OBJ_NAME_SIZE 32

struct bindpkt_params_per_process {
	/* Name of the synchronization object associated with this process */
	char sync_obj_name[MAX_SYNC_OBJ_NAME_SIZE];
	uint32_t count;	/* Number of entries in this bind */
	struct bindpkt_params *params; /* first bind params */
};

#endif

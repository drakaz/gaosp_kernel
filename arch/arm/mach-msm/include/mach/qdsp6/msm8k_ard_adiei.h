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

#ifndef ARDIADIEI_H
#define ARDIADIEI_H

#include "msm8k_cad.h"

#define MAX_ADIE_PATH_TYPES   2

enum adie_state_enum_type {
	ADIE_STATE_RESET,
	ADIE_STATE_DIGITAL_ACTIVE,
	ADIE_STATE_DIGITAL_ANALOG_ACTIVE,
};

enum adie_state_ret_enum_type {
	ADIE_STATE_RC_SUCCESS,
	ADIE_STATE_RC_CONTINUE,
	ADIE_STATE_RC_FAILURE
};

enum adie_ret_enum_type {
	ADIE_FALSE = 0,
	ADIE_TRUE
};

struct adie_path_type_struct_type {
	u32		state;
	u8		enable_request;
	u8		enabled;
};

struct adie_state_struct_type {
	void					*adie_handle;
	u8					adie_opened;
	struct adie_path_type_struct_type
					adie_path_type[MAX_ADIE_PATH_TYPES];
};

u32 adie_state_control(u32 dev_type, u32 dev_id);
enum adie_state_ret_enum_type adie_state_reset(u32 dev_type, u32 dev_id);
enum adie_state_ret_enum_type adie_state_digital_active(u32 dev_type,
		u32 dev_id);
enum adie_state_ret_enum_type adie_state_digital_analog_active(u32 dev_type,
		u32 dev_id);

#endif

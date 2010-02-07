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

#ifndef _QDSP6DTMFDRIVERI_H_
#define _QDSP6DTMFDRIVERI_H_

#include <mach/qdsp6/msm8k_cad_module.h>

#define CAD_DTMF_SESSION_MAX 4

struct q6dtmf_session;

struct q6dtmf_session {
	u32				session_id;
	struct q6dtmf_session		*next;
};


struct q6dtmf_driver {
	struct q6dtmf_session		*free_session_list;
	struct q6dtmf_session		*used_session_list;
};

#endif

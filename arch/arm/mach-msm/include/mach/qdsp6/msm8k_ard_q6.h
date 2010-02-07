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

#ifndef ARDQ6_H
#define ARDQ6_H

#include "msm8k_ardi.h"
#include "msm8k_cad.h"

s32 qdsp6_open(s32 session_id);
s32 qdsp6_start(s32 session_id);
s32 qdsp6_close(s32 session_id);
s32 qdsp6_devchg_notify(s32 session_id, u32 dev_id);
s32 qdsp6_standby(s32 session_id);

#endif

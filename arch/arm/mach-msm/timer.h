/* arch/arm/mach-msm/timer.h
 *
 * Copyright (c) 2008 QUALCOMM USA, INC.
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

#ifndef _ARCH_ARM_MACH_MSM_TIMER_H_
#define _ARCH_ARM_MACH_MSM_TIMER_H_

extern struct sys_timer msm_timer;

int64_t msm_timer_enter_idle(void);
void msm_timer_exit_idle(int low_power);
int64_t msm_timer_get_smem_clock_time(int64_t *period);

#endif

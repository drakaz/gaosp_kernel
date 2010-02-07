/* arch/arm/mach-msm/gpio.h
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

#ifndef _ARCH_ARM_MACH_MSM_GPIO_H_
#define _ARCH_ARM_MACH_MSM_GPIO_H_

void msm_gpio_enter_sleep(int from_idle);
void msm_gpio_exit_sleep(void);

#endif

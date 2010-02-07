/* arch/arm/mach-msm/irq.h
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

#ifndef _ARCH_ARM_MACH_MSM_IRQ_H_
#define _ARCH_ARM_MACH_MSM_IRQ_H_

int msm_irq_idle_sleep_allowed(void);
int msm_irq_pending(void);
void msm_irq_enter_sleep1(bool arm9_wake, int from_idle, uint32_t *irq_mask);
int msm_irq_enter_sleep2(bool arm9_wake, int from_idle);
void msm_irq_exit_sleep1
	(uint32_t irq_mask, uint32_t wakeup_reason, uint32_t pending_irqs);
void msm_irq_exit_sleep2
	(uint32_t irq_mask, uint32_t wakeup_reason, uint32_t pending);
void msm_irq_exit_sleep3
	(uint32_t irq_mask, uint32_t wakeup_reason, uint32_t pending_irqs);

#endif

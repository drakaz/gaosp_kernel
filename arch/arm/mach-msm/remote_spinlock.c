/*
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
 */

#include <linux/err.h>
#include <linux/kernel.h>

#include <asm/system.h>

/* This is ugly but it has to be done. If linux/spinlock_types is included,
 * then for a UP kernel, the spinlock will get stubbed out. Since this is a
 * remote spin lock, stubbing out is not the right thing to do.
 */
#define __LINUX_SPINLOCK_TYPES_H
#include <asm/spinlock_types.h>
#undef __LINUX_SPINLOCK_TYPES_H

#include <asm/spinlock.h>

#include <mach/remote_spinlock.h>
#include "smd_private.h"

#define SMEM_SPINLOCK_COUNT 8
#define SMEM_SPINLOCK_ARRAY_SIZE (SMEM_SPINLOCK_COUNT * sizeof(uint32_t))

int _remote_spin_lock_init(remote_spin_lock_id_t id, _remote_spinlock_t *lock)
{
	_remote_spinlock_t spinlock_start;

	/* The raw_spinlock_t structure should be the same as
	 * raw_remote_spinlock_t to be able to reuse the __raw_spin_lock()
	 * and __raw_spin_unlock() functions. If this condition is not met,
	 * then please write new code to replace calls to __raw_spin_lock()
	 * and __raw_spin_unlock(). */
	BUILD_BUG_ON(sizeof(raw_remote_spinlock_t) != sizeof(raw_spinlock_t));

	if (id >= SMEM_SPINLOCK_COUNT)
		return -EINVAL;

	spinlock_start = smem_alloc(SMEM_SPINLOCK_ARRAY,
				    SMEM_SPINLOCK_ARRAY_SIZE);
	if (spinlock_start == NULL)
		return -ENXIO;

	*lock = spinlock_start + id;

	return 0;
}

void _remote_spin_lock(_remote_spinlock_t *lock)
{
	__raw_spin_lock((raw_spinlock_t *) (*lock));
}

void _remote_spin_unlock(_remote_spinlock_t *lock)
{
	__raw_spin_unlock((raw_spinlock_t *) (*lock));
}

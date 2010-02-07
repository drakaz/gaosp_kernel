/* drivers/char/diag/diagmem.c */

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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/mutex.h>
#include <asm/atomic.h>
#include "diagchar.h"

void *diagmem_alloc(struct diagchar_dev *driver, int size)
{
	void *buf;

	mutex_lock(&driver->diagmem_mutex);
	if (driver->count < driver->poolsize) {
		buf = mempool_alloc(driver->diagpool, GFP_ATOMIC);
		atomic_add(1, (atomic_t *)&driver->count);
	} else {
		buf = NULL;
	}
	mutex_unlock(&driver->diagmem_mutex);
	return buf;
}

void diagmem_free(struct diagchar_dev *driver, void *buf)
{
	if (driver->diagpool != NULL) {
		mempool_free(buf, driver->diagpool);
		atomic_add(-1, (atomic_t *)&driver->count);
	} else
		printk(KERN_ALERT "\n Attempt to free up DIAG driver mempool"
				  " memory which is already free");

}

void diagmem_init(struct diagchar_dev *driver)
{
    mutex_init(&driver->diagmem_mutex);
	driver->count = 0;
	driver->diagpool = mempool_create_kmalloc_pool(driver->poolsize,
						       driver->itemsize);
	if (!driver->diagpool)
		printk(KERN_INFO "Cannot allocate diag mempool\n");

}

void diagmem_exit(struct diagchar_dev *driver)
{
    mempool_destroy(driver->diagpool);
}

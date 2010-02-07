/* drivers/char/diag/diagmem.h */

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

#ifndef DIAGMEM_H
#define DIAGMEM_H
#include "diagchar.h"

void *diagmem_alloc(struct diagchar_dev *driver, int size);
void diagmem_free(struct diagchar_dev *driver, void *buf);
void diagmem_init(struct diagchar_dev *driver);
void diagmem_exit(struct diagchar_dev *driver);

#endif

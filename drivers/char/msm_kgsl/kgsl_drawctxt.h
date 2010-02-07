/*
* (C) Copyright Advanced Micro Devices, Inc. 2002, 2007
* Copyright (c) 2008-2009 QUALCOMM USA, INC.
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
#ifndef __GSL_DRAWCTXT_H
#define __GSL_DRAWCTXT_H

#include <linux/types.h>
#include <linux/msm_kgsl.h>

#include "kgsl_sharedmem.h"

struct kgsl_device;

struct kgsl_drawctxt {
	uint32_t         flags;
	struct kgsl_pagetable *pagetable;
	struct kgsl_memdesc       gpustate;
	struct kgsl_memdesc       gmemshadow;
	unsigned int        reg_save[3];
	unsigned int        reg_restore[3];
	unsigned int        gmem_save[3];
	unsigned int        gmem_restore[3];
	unsigned int        shader_save[3];
	unsigned int        shader_fixup[3];
	unsigned int        shader_restore[3];
};


int kgsl_drawctxt_create(struct kgsl_device *, struct kgsl_pagetable *,
			  unsigned int flags,
			  unsigned int *drawctxt_id);

int kgsl_drawctxt_destroy(struct kgsl_device *device, unsigned int drawctxt_id);

int kgsl_drawctxt_init(struct kgsl_device *device);

int kgsl_drawctxt_close(struct kgsl_device *device);

void kgsl_drawctxt_switch(struct kgsl_device *device,
				struct kgsl_drawctxt *drawctxt,
				unsigned int flags);

#endif  /* __GSL_DRAWCTXT_H */

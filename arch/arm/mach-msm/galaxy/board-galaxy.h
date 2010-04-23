/* linux/arch/arm/mach-msm/behold2/board-behold2.h
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#ifndef __ARCH_ARM_MACH_MSM_BOARD_BEHOLD2_H
#define __ARCH_ARM_MACH_MSM_BOARD_BEHOLD2_H

#include <mach/board.h>

#define MSM_SMI_BASE					0x00000000
#define MSM_SMI_SIZE					0x00800000

#define MSM_EBI_BASE					0x10000000
#define MSM_EBI_SIZE					0x06D00000		// Total 109M for ARM11

#define MSM_PMEM_GPU0_BASE			0x00100000
#define MSM_PMEM_GPU0_SIZE			0x00700000

#define SMI64_MSM_PMEM_MDP_BASE		0x02000000
#define SMI64_MSM_PMEM_MDP_SIZE		0x00800000		// 8M

#define SMI64_MSM_PMEM_ADSP_BASE		0x02800000
#define SMI64_MSM_PMEM_ADSP_SIZE		0x00D00000		// 13M

#define SMI64_MSM_PMEM_CAMERA_BASE	0x03500000
#define SMI64_MSM_PMEM_CAMERA_SIZE	0x00B00000		// 11M

#define SMI64_MSM_FB_BASE				0x00800000
#define SMI64_MSM_FB_SIZE				0x00100000		// 1M

#define SMI64_MSM_LINUX_BASE			MSM_EBI_BASE
#define SMI64_MSM_LINUX_SIZE			0x06500000		// 109M - ( 8M for GPU1 in SDRAM ) = 101M

#define MSM_PMEM_GPU1_BASE			(MSM_EBI_BASE + SMI64_MSM_LINUX_SIZE)
#define MSM_PMEM_GPU1_SIZE			0x800000

#define SMI32_MSM_LINUX_BASE			MSM_EBI_BASE
#define SMI32_MSM_LINUX_SIZE			0x04500000		// 109M - ( 40M for pmem in SDRAM) = 69M for Linux

#define SMI32_MSM_PMEM_MDP_BASE		(MSM_EBI_BASE + SMI32_MSM_LINUX_SIZE)
#define SMI32_MSM_PMEM_MDP_SIZE		0x00800000		// 8M

#define SMI32_MSM_PMEM_ADSP_BASE		(SMI32_MSM_PMEM_MDP_BASE + SMI32_MSM_PMEM_MDP_SIZE)
#define SMI32_MSM_PMEM_ADSP_SIZE		0x00D00000		// 13M

#define SMI32_MSM_PMEM_CAMERA_BASE	(SMI32_MSM_PMEM_ADSP_BASE + SMI32_MSM_PMEM_ADSP_SIZE)
#define SMI32_MSM_PMEM_CAMERA_SIZE	0x00B00000		// 11M

#define SMI32_MSM_FB_BASE				0x00800000
#define SMI32_MSM_FB_SIZE				0x00100000		// 1M

#define SMI32_MSM_PMEM_GPU1_BASE		(SMI32_MSM_PMEM_CAMERA_BASE + SMI32_MSM_PMEM_CAMERA_SIZE) 
#define SMI32_MSM_PMEM_GPU1_SIZE		0x800000

#define MSM_PMEM_KERNEL_EBI1_SIZE	0x200000

#define DECLARE_MSM_IOMAP
#include <mach/msm_iomap.h>

int galaxy_get_smi_size(void);

#endif /* GUARD */


/*
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

#ifndef _ARCH_ARM_MACH_MSM_SOCINFO_H_
#define _ARCH_ARM_MACH_MSM_SOCINFO_H_

/*
 * SOC version type with major number in the upper 16 bits and minor
 * number in the lower 16 bits.  For example:
 *   1.0 -> 0x00010000
 *   2.3 -> 0x00020003
 */
#define SOCINFO_VERSION_MAJOR(ver) ((ver & 0xffff0000) >> 16)
#define SOCINFO_VERSION_MINOR(ver) (ver & 0x0000ffff)

enum msm_cpu {
	MSM_CPU_UNKNOWN = 0,
	MSM_CPU_7X01,
	MSM_CPU_7X25,
	MSM_CPU_7X27,
	MSM_CPU_8X50,
};

enum msm_cpu socinfo_get_msm_cpu(void);
uint32_t socinfo_get_id(void);
uint32_t socinfo_get_version(void);
char *socinfo_get_build_id(void);
int __init socinfo_init(void) __must_check;

static inline int cpu_is_msm7x01(void)
{
	enum msm_cpu cpu = socinfo_get_msm_cpu();

	BUG_ON(cpu == MSM_CPU_UNKNOWN);
	return cpu == MSM_CPU_7X01;
}

static inline int cpu_is_msm7x25(void)
{
	enum msm_cpu cpu = socinfo_get_msm_cpu();

	BUG_ON(cpu == MSM_CPU_UNKNOWN);
	return cpu == MSM_CPU_7X25;
}

static inline int cpu_is_msm7x27(void)
{
	enum msm_cpu cpu = socinfo_get_msm_cpu();

	BUG_ON(cpu == MSM_CPU_UNKNOWN);
	return cpu == MSM_CPU_7X27;
}

static inline int cpu_is_qsd8x50(void)
{
	enum msm_cpu cpu = socinfo_get_msm_cpu();

	BUG_ON(cpu == MSM_CPU_UNKNOWN);
	return cpu == MSM_CPU_8X50;
}

#endif

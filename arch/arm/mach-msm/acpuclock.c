/* arch/arm/mach-msm/acpuclock.c
 *
 * MSM architecture clock driver
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, Code Aurora Forum. All rights reserved.
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/sort.h>
#include <linux/remote_spinlock.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>
#include <linux/device.h>

#include "proc_comm.h"
#include "smd_private.h"
#include "clock.h"
#include "acpuclock.h"
#include "socinfo.h"

#define PERF_SWITCH_DEBUG 0
#define PERF_SWITCH_STEP_DEBUG 0

#define VOLT_CLASS "cpu"
#define VOLT_DEVICE "voltage"

#define VOLTAGE_FREQ_ATTR(_name)                                                 \
{                                                                               \
        .attr = { .name = #_name, .mode = 0700, .owner = THIS_MODULE },      \
        .show = voltage_show_vdd,                                      \
        .store = voltage_store_vdd,                                                          \
}

struct class *voltage_class;
struct device *voltage_dev;
static struct device_attribute *voltage_attrs;

static ssize_t voltage_show_vdd(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf);

static ssize_t voltage_store_vdd(struct device *dev, 
                                        struct device_attribute *attr,
                                        const char *buf,
                                        size_t size);
static void __init init_sysfs_interface(void);

#define A11S_CLK_CNTL_ADDR (MSM_CSR_BASE + 0x100)
#define A11S_CLK_SEL_ADDR (MSM_CSR_BASE + 0x104)
#define A11S_VDD_SVS_PLEVEL_ADDR (MSM_CSR_BASE + 0x124)
#define PLLn_MODE(n)	(MSM_CLK_CTL_BASE + 0x300 + 28 * (n))
#define PLLn_L_VAL(n)	(MSM_CLK_CTL_BASE + 0x304 + 28 * (n))

#define dprintk(msg...) \
	cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, "cpufreq-msm", msg)

enum {
	ACPU_PLL_TCXO	= -1,
	ACPU_PLL_0	= 0,
	ACPU_PLL_1,
	ACPU_PLL_2,
	ACPU_PLL_3,
	ACPU_PLL_END,
};

struct clock_state
{
	struct clkctl_acpu_speed	*current_speed;
	struct mutex			lock;
	uint32_t			acpu_switch_time_us;
	uint32_t			max_speed_delta_khz;
	uint32_t			vdd_switch_time_us;
	unsigned long			power_collapse_khz;
	unsigned long			wait_for_irq_khz;
	unsigned long			max_axi_khz;
};

#define PLL_BASE	7

struct shared_pll_control {
	uint32_t	version;
	struct {
		/* Denotes if the PLL is ON. Technically, this can be read
		 * directly from the PLL registers, but this feild is here,
		 * so let's use it.
		 */
		uint32_t	on;
		/* One bit for each processor core. The application processor
		 * is allocated bit position 1. All other bits should be
		 * considered as votes from other processors.
		 */
		uint32_t	votes;
	} pll[PLL_BASE + ACPU_PLL_END];
};

struct clkctl_acpu_speed {
	unsigned int	use_for_scaling;
	unsigned int	a11clk_khz;
	int		pll;
	unsigned int	a11clk_src_sel;
	unsigned int	a11clk_src_div;
	unsigned int	ahbclk_khz;
	unsigned int	ahbclk_div;
	int		vdd;
    unsigned int 	axiclk_khz;
	struct device_attribute attribute;
	unsigned long	lpj; /* loops_per_jiffy */
/* Pointers in acpu_freq_tbl[] for max up/down steppings. */
	struct clkctl_acpu_speed *down[3];
	struct clkctl_acpu_speed *up[3];
	// For overclocking via PLL2 L val
	short    pll2_lval;
};

static remote_spinlock_t pll_lock;
static struct shared_pll_control *pll_control;
static struct clock_state drv_state = { 0 };
static struct clkctl_acpu_speed *acpu_freq_tbl;

static void __init acpuclk_init(void);

/*
 * ACPU freq tables used for different PLLs frequency combinations. The
 * correct table is selected during init.
 *
 * Table stepping up/down entries are calculated during boot to choose the
 * largest frequency jump that's less than max_speed_delta_khz on each PLL.
 */

/* 7x01/7x25 normal with GSM capable modem */
static struct clkctl_acpu_speed pll0_245_pll1_768_pll2_1056[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 30720, VOLTAGE_FREQ_ATTR(19200) },
	{ 1, 122880, ACPU_PLL_0, 4, 1,  61440, 1, 3,  61440, VOLTAGE_FREQ_ATTR(122880) },
	{ 0, 128000, ACPU_PLL_1, 1, 5,  64000, 1, 3,  61440, VOLTAGE_FREQ_ATTR(128000) },
	{ 0, 176000, ACPU_PLL_2, 2, 5,  88000, 1, 3,  61440, VOLTAGE_FREQ_ATTR(176000) },
	{ 1, 245760, ACPU_PLL_0, 4, 0,  81920, 2, 4,  61440, VOLTAGE_FREQ_ATTR(245760) },
	{ 1, 256000, ACPU_PLL_1, 1, 2, 128000, 1, 5, 128000, VOLTAGE_FREQ_ATTR(256000) },
	{ 0, 352000, ACPU_PLL_2, 2, 2,  88000, 3, 5, 128000, VOLTAGE_FREQ_ATTR(352000) },
	{ 1, 384000, ACPU_PLL_1, 1, 1, 128000, 2, 6, 128000, VOLTAGE_FREQ_ATTR(384000) },
	{ 1, 528000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(528000) },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, {{0, 0, 0}, 0, 0}, 0, {0, 0, 0}, {0, 0, 0} }
};

/* 7x01/7x25 normal with CDMA-only modem */
static struct clkctl_acpu_speed pll0_196_pll1_768_pll2_1056[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 24576, VOLTAGE_FREQ_ATTR(19200) },
	{ 1,  98304, ACPU_PLL_0, 4, 1,  49152, 1, 3,  24576, VOLTAGE_FREQ_ATTR(98304) },
	{ 0, 128000, ACPU_PLL_1, 1, 5,  64000, 1, 3,  24576, VOLTAGE_FREQ_ATTR(128000) },
	{ 0, 176000, ACPU_PLL_2, 2, 5,  88000, 1, 3,  24576, VOLTAGE_FREQ_ATTR(176000) },
	{ 1, 196608, ACPU_PLL_0, 4, 0,  65536, 2, 4,  24576, VOLTAGE_FREQ_ATTR(196608) },
	{ 1, 256000, ACPU_PLL_1, 1, 2, 128000, 1, 5, 128000, VOLTAGE_FREQ_ATTR(256000) },
	{ 0, 352000, ACPU_PLL_2, 2, 2,  88000, 3, 5, 128000, VOLTAGE_FREQ_ATTR(352000) },
	{ 1, 384000, ACPU_PLL_1, 1, 1, 128000, 2, 6, 128000, VOLTAGE_FREQ_ATTR(384000) },
	{ 1, 528000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(528000) },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, {{0, 0, 0}, 0, 0}, 0, {0, 0, 0}, {0, 0, 0} }
};

/* 7x01/7x25 turbo with GSM capable modem */
static struct clkctl_acpu_speed pll0_245_pll1_960_pll2_1056[] = {
	{ 1, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 30720, VOLTAGE_FREQ_ATTR(19200) },
	{ 1, 120000, ACPU_PLL_1, 1, 7,  60000, 1, 3,  61440, VOLTAGE_FREQ_ATTR(120000) },
	{ 1, 122880, ACPU_PLL_0, 4, 1,  61440, 1, 3,  61440, VOLTAGE_FREQ_ATTR(122880) },
	{ 1, 176000, ACPU_PLL_2, 2, 5,  88000, 1, 3,  61440, VOLTAGE_FREQ_ATTR(176000) },
	{ 1, 245760, ACPU_PLL_0, 4, 0,  81920, 2, 4,  61440, VOLTAGE_FREQ_ATTR(245760) },
	{ 1, 320000, ACPU_PLL_1, 1, 2, 107000, 2, 5, 120000, VOLTAGE_FREQ_ATTR(320000) },
	{ 1, 352000, ACPU_PLL_2, 2, 2,  88000, 3, 5, 120000, VOLTAGE_FREQ_ATTR(352000) },
	{ 1, 480000, ACPU_PLL_1, 1, 1, 120000, 3, 6, 120000, VOLTAGE_FREQ_ATTR(480000) },
	{ 1, 528000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 122880, VOLTAGE_FREQ_ATTR(528000) },
	{ 1, 576000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(576000) },
	{ 1, 624000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(624000) },
	{ 1, 672000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(672000) },
	{ 1, 720000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(720000) },
	{ 1, 748800, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(748800) },
	{ 1, 758400, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(758400) },
	{ 1, 768000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(768000) },
	{ 1, 777600, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(777600) },
	{ 1, 787200, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(787200) },
	{ 1, 796800, ACPU_PLL_2, 2, 1, 132000, 3, 7, 128000, VOLTAGE_FREQ_ATTR(796800) },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, {{0, 0, 0}, 0, 0}, 0, {0, 0, 0}, {0, 0, 0} }
};

/* 7x01/7x25 turbo with CDMA-only modem */
static struct clkctl_acpu_speed pll0_196_pll1_960_pll2_1056[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 24576, VOLTAGE_FREQ_ATTR(19200) },
	{ 1,  98304, ACPU_PLL_0, 4, 1,  49152, 1, 3,  24576, VOLTAGE_FREQ_ATTR(98304) },
	{ 0, 120000, ACPU_PLL_1, 1, 7,  60000, 1, 3,  24576, VOLTAGE_FREQ_ATTR(120000) },
	{ 0, 176000, ACPU_PLL_2, 2, 5,  88000, 1, 3,  24576, VOLTAGE_FREQ_ATTR(176000) },
	{ 1, 196608, ACPU_PLL_0, 4, 0,  65536, 2, 4,  24576, VOLTAGE_FREQ_ATTR(196608) },
	{ 1, 320000, ACPU_PLL_1, 1, 2, 107000, 2, 5, 120000, VOLTAGE_FREQ_ATTR(320000) },
	{ 0, 352000, ACPU_PLL_2, 2, 2,  88000, 3, 5, 120000, VOLTAGE_FREQ_ATTR(352000) },
	{ 1, 480000, ACPU_PLL_1, 1, 1, 120000, 3, 6, 120000, VOLTAGE_FREQ_ATTR(480000) },
	{ 1, 528000, ACPU_PLL_2, 2, 1, 132000, 3, 7, 120000, VOLTAGE_FREQ_ATTR(528000) },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, {{0, 0, 0}, 0, 0}, 0, {0, 0, 0}, {0, 0, 0} }
};

/* 7x27 normal with GSM capable modem */
static struct clkctl_acpu_speed pll0_245_pll1_960_pll2_1200[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 30720, VOLTAGE_FREQ_ATTR(19200) },
	{ 0, 120000, ACPU_PLL_1, 1, 7,  60000, 1, 3,  61440, VOLTAGE_FREQ_ATTR(120000) },
	{ 1, 122880, ACPU_PLL_0, 4, 1,  61440, 1, 3,  61440, VOLTAGE_FREQ_ATTR(122880) },
	{ 0, 200000, ACPU_PLL_2, 2, 5,  66667, 2, 4,  61440, VOLTAGE_FREQ_ATTR(200000) },
	{ 1, 245760, ACPU_PLL_0, 4, 0, 122880, 1, 4,  61440, VOLTAGE_FREQ_ATTR(245760) },
	{ 1, 320000, ACPU_PLL_1, 1, 2, 160000, 1, 5, 122880, VOLTAGE_FREQ_ATTR(320000) },
	{ 0, 400000, ACPU_PLL_2, 2, 2, 133333, 2, 5, 122880, VOLTAGE_FREQ_ATTR(400000) },
	{ 1, 480000, ACPU_PLL_1, 1, 1, 160000, 2, 6, 122880, VOLTAGE_FREQ_ATTR(480000) },
	{ 1, 600000, ACPU_PLL_2, 2, 1, 200000, 2, 7, 122880, VOLTAGE_FREQ_ATTR(600000) },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, {{0, 0, 0}, 0, 0}, 0, {0, 0, 0}, {0, 0, 0} }
};

/* 7x27 normal with CDMA-only modem */
static struct clkctl_acpu_speed pll0_196_pll1_960_pll2_1200[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 24576, VOLTAGE_FREQ_ATTR(19200) },
	{ 1,  98304, ACPU_PLL_0, 4, 1,  98304, 0, 3,  49152, VOLTAGE_FREQ_ATTR(98304) },
	{ 0, 120000, ACPU_PLL_1, 1, 7,  60000, 1, 3,  49152, VOLTAGE_FREQ_ATTR(120000) },
	{ 1, 196608, ACPU_PLL_0, 4, 0,  65536, 2, 4,  98304, VOLTAGE_FREQ_ATTR(196608) },
	{ 0, 200000, ACPU_PLL_2, 2, 5,  66667, 2, 4,  98304, VOLTAGE_FREQ_ATTR(200000) },
	{ 1, 320000, ACPU_PLL_1, 1, 2, 160000, 1, 5, 120000, VOLTAGE_FREQ_ATTR(320000) },
	{ 0, 400000, ACPU_PLL_2, 2, 2, 133333, 2, 5, 120000, VOLTAGE_FREQ_ATTR(400000) },
	{ 1, 480000, ACPU_PLL_1, 1, 1, 160000, 2, 6, 120000, VOLTAGE_FREQ_ATTR(480000) },
	{ 1, 600000, ACPU_PLL_2, 2, 1, 200000, 2, 7, 120000, VOLTAGE_FREQ_ATTR(600000) },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, {{0, 0, 0}, 0, 0}, 0, {0, 0, 0}, {0, 0, 0} }
};

/* 7x27 normal with GSM capable modem - PLL0 and PLL1 swapped */
static struct clkctl_acpu_speed pll0_960_pll1_245_pll2_1200[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 30720, VOLTAGE_FREQ_ATTR(19200) },
	{ 0, 120000, ACPU_PLL_0, 4, 7,  60000, 1, 3,  61440, VOLTAGE_FREQ_ATTR(120000) },
	{ 1, 122880, ACPU_PLL_1, 1, 1,  61440, 1, 3,  61440, VOLTAGE_FREQ_ATTR(122880) },
	{ 0, 200000, ACPU_PLL_2, 2, 5,  66667, 2, 4,  61440, VOLTAGE_FREQ_ATTR(200000) },
	{ 1, 245760, ACPU_PLL_1, 1, 0, 122880, 1, 4,  61440, VOLTAGE_FREQ_ATTR(245760) },
	{ 1, 320000, ACPU_PLL_0, 4, 2, 160000, 1, 5, 122880, VOLTAGE_FREQ_ATTR(320000) },
	{ 0, 400000, ACPU_PLL_2, 2, 2, 133333, 2, 5, 122880, VOLTAGE_FREQ_ATTR(400000) },
	{ 1, 480000, ACPU_PLL_0, 4, 1, 160000, 2, 6, 122880, VOLTAGE_FREQ_ATTR(480000) },
	{ 1, 600000, ACPU_PLL_2, 2, 1, 200000, 2, 7, 122880, VOLTAGE_FREQ_ATTR(600000) },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, {{0, 0, 0}, 0, 0}, 0, {0, 0, 0}, {0, 0, 0} }
};

/* 7x27 normal with CDMA-only modem - PLL0 and PLL1 swapped */
static struct clkctl_acpu_speed pll0_960_pll1_196_pll2_1200[] = {
	{ 0, 19200, ACPU_PLL_TCXO, 0, 0, 19200, 0, 0, 24576, VOLTAGE_FREQ_ATTR(19200) },
	{ 1,  98304, ACPU_PLL_1, 1, 1,  98304, 0, 3,  49152, VOLTAGE_FREQ_ATTR(98304) },
	{ 0, 120000, ACPU_PLL_0, 4, 7,  60000, 1, 3,  49152, VOLTAGE_FREQ_ATTR(120000) },
	{ 1, 196608, ACPU_PLL_1, 1, 0,  65536, 2, 4,  98304, VOLTAGE_FREQ_ATTR(196608) },
	{ 0, 200000, ACPU_PLL_2, 2, 5,  66667, 2, 4,  98304, VOLTAGE_FREQ_ATTR(200000) },
	{ 1, 320000, ACPU_PLL_0, 4, 2, 160000, 1, 5, 120000, VOLTAGE_FREQ_ATTR(320000) },
	{ 0, 400000, ACPU_PLL_2, 2, 2, 133333, 2, 5, 120000, VOLTAGE_FREQ_ATTR(400000) },
	{ 1, 480000, ACPU_PLL_0, 4, 1, 160000, 2, 6, 120000, VOLTAGE_FREQ_ATTR(480000) },
	{ 1, 600000, ACPU_PLL_2, 2, 1, 200000, 2, 7, 120000, VOLTAGE_FREQ_ATTR(600000) },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, {{0, 0, 0}, 0, 0}, 0, {0, 0, 0}, {0, 0, 0} }
};

#define PLL_196_MHZ	10
#define PLL_245_MHZ	12
#define PLL_491_MHZ	25
#define PLL_768_MHZ	40
#define PLL_960_MHZ	50
#define PLL_1056_MHZ	55
#define PLL_1200_MHZ	62

#define PLL_CONFIG(m0, m1, m2) { \
	PLL_##m0##_MHZ, PLL_##m1##_MHZ, PLL_##m2##_MHZ, \
	pll0_##m0##_pll1_##m1##_pll2_##m2 \
}

struct pll_freq_tbl_map {
	unsigned int	pll0_l;
	unsigned int	pll1_l;
	unsigned int	pll2_l;
	struct clkctl_acpu_speed *tbl;
};

static struct pll_freq_tbl_map acpu_freq_tbl_list[] = {
	PLL_CONFIG(196, 768, 1056),
	PLL_CONFIG(245, 768, 1056),
	PLL_CONFIG(196, 960, 1056),
	PLL_CONFIG(245, 960, 1056),
	PLL_CONFIG(196, 960, 1200),
	PLL_CONFIG(245, 960, 1200),
	PLL_CONFIG(960, 196, 1200),
	PLL_CONFIG(960, 245, 1200),
	{ 0, 0, 0, 0 }
};

static struct cpufreq_frequency_table freq_table[20];

static void __init cpufreq_table_init(void)
{
	unsigned int i;
	unsigned int freq_cnt = 0;

	/* Construct the freq_table table from acpu_freq_tbl since the
	 * freq_table values need to match frequencies specified in
	 * acpu_freq_tbl and acpu_freq_tbl needs to be fixed up during init.
	 */
	for (i = 0; acpu_freq_tbl[i].a11clk_khz != 0
			&& freq_cnt < ARRAY_SIZE(freq_table)-1; i++) {
		if (acpu_freq_tbl[i].use_for_scaling) {
			freq_table[freq_cnt].index = freq_cnt;
			freq_table[freq_cnt].frequency
				= acpu_freq_tbl[i].a11clk_khz;
			freq_cnt++;
		}
	}

	/* freq_table not big enough to store all usable freqs. */
	BUG_ON(acpu_freq_tbl[i].a11clk_khz != 0);

	freq_table[freq_cnt].index = freq_cnt;
	freq_table[freq_cnt].frequency = CPUFREQ_TABLE_END;

	pr_info("%d scaling frequencies supported.\n", freq_cnt);
}

unsigned long clk_get_max_axi_khz(void)
{
	return drv_state.max_axi_khz;
}
EXPORT_SYMBOL(clk_get_max_axi_khz);

static int pc_pll_request(unsigned id, unsigned on)
{
	int res = 0;
	on = !!on;

#if PERF_SWITCH_DEBUG
	if (on)
		printk(KERN_DEBUG "Enabling PLL %d\n", id);
	else
		printk(KERN_DEBUG "Disabling PLL %d\n", id);
#endif

	if (id >= ACPU_PLL_END)
		return -EINVAL;

	if (pll_control) {
		remote_spin_lock(&pll_lock);
		if (on) {
			pll_control->pll[PLL_BASE + id].votes |= 2;
			if (!pll_control->pll[PLL_BASE + id].on) {
				writel(6, PLLn_MODE(id));
				udelay(50);
				writel(7, PLLn_MODE(id));
				pll_control->pll[PLL_BASE + id].on = 1;
			}
		} else {
			pll_control->pll[PLL_BASE + id].votes &= ~2;
			if (pll_control->pll[PLL_BASE + id].on
			    && !pll_control->pll[PLL_BASE + id].votes) {
				writel(0, PLLn_MODE(id));
				pll_control->pll[PLL_BASE + id].on = 0;
			}
		}
		remote_spin_unlock(&pll_lock);
	} else {
	res = msm_proc_comm(PCOM_CLKCTL_RPC_PLL_REQUEST, &id, &on);
	if (res < 0)
		return res;
		else if ((int) id < 0)
			return -EINVAL;
	}

#if PERF_SWITCH_DEBUG
	if (on)
		printk(KERN_DEBUG "PLL %d enabled\n", id);
	else
		printk(KERN_DEBUG "PLL %d disabled\n", id);
#endif
	return res;
}


/*----------------------------------------------------------------------------
 * ARM11 'owned' clock control
 *---------------------------------------------------------------------------*/

unsigned long acpuclk_power_collapse(void) {
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(drv_state.power_collapse_khz, SETRATE_PC);
	return ret * 1000;
}

unsigned long acpuclk_wait_for_irq(void) {
	int ret = acpuclk_get_rate();
	acpuclk_set_rate(drv_state.wait_for_irq_khz, SETRATE_SWFI);
	return ret * 1000;
}

static int acpuclk_set_vdd_level(int vdd)
{
	uint32_t current_vdd;

	current_vdd = readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x07;

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "acpuclock: Switching VDD from %u -> %d\n",
	       current_vdd, vdd);
#endif
	writel((1 << 7) | (vdd << 3), A11S_VDD_SVS_PLEVEL_ADDR);
	udelay(drv_state.vdd_switch_time_us);
	if ((readl(A11S_VDD_SVS_PLEVEL_ADDR) & 0x7) != vdd) {
#if PERF_SWITCH_DEBUG
		printk(KERN_ERR "acpuclock: VDD set failed\n");
#endif
		return -EIO;
	}

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "acpuclock: VDD switched\n");
#endif
	return 0;
}

/* Set proper dividers for the given clock speed. */
static void acpuclk_set_div(const struct clkctl_acpu_speed *hunt_s) {
	uint32_t reg_clkctl, reg_clksel, clk_div, src_sel, a11_div;

	reg_clksel = readl(A11S_CLK_SEL_ADDR);

	/* AHB_CLK_DIV */
	clk_div = (reg_clksel >> 1) & 0x03;
	/* CLK_SEL_SRC1NO */
	src_sel = reg_clksel & 1;


	a11_div=hunt_s->a11clk_src_div;
	if (hunt_s->a11clk_khz > 528000) {
		a11_div = 0;
		 writel(hunt_s->a11clk_khz/19200, MSM_CLK_CTL_BASE+0x33c);
		udelay(50);
	}


	/*
	 * If the new clock divider is higher than the previous, then
	 * program the divider before switching the clock
	 */
	if (hunt_s->ahbclk_div > clk_div) {
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}

	/* Program clock source and divider */
		reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
	reg_clkctl &= ~(0xFF << (8 * src_sel));
	reg_clkctl |= hunt_s->a11clk_src_sel << (4 + 8 * src_sel);
	reg_clkctl |= a11_div << (0 + 8 * src_sel);
		writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

		/* Program clock source selection */
	reg_clksel ^= 1;
		writel(reg_clksel, A11S_CLK_SEL_ADDR);

	/*
	 * If the new clock divider is lower than the previous, then
	 * program the divider after switching the clock
	 */
	if (hunt_s->ahbclk_div < clk_div) {
		reg_clksel &= ~(0x3 << 1);
		reg_clksel |= (hunt_s->ahbclk_div << 1);
		writel(reg_clksel, A11S_CLK_SEL_ADDR);
	}
}

int acpuclk_set_rate(unsigned long rate, enum setrate_reason reason)
{
	uint32_t reg_clkctl;
	struct clkctl_acpu_speed *cur_s, *tgt_s, *strt_s;
	int res, rc = 0;
	unsigned int plls_enabled = 0, pll;

	if (reason == SETRATE_CPUFREQ)
		mutex_lock(&drv_state.lock);

	strt_s = cur_s = drv_state.current_speed;

	WARN_ONCE(cur_s == NULL, "acpuclk_set_rate: not initialized\n");
	if (cur_s == NULL) {
		rc = -ENOENT;
		goto out;
	}

	if (rate == (cur_s->a11clk_khz * 1000))
		goto out;

	for (tgt_s = acpu_freq_tbl; tgt_s->a11clk_khz != 0; tgt_s++) {
		if (tgt_s->a11clk_khz == (rate / 1000))
			break;
	}

	if (tgt_s->a11clk_khz == 0) {
		rc = -EINVAL;
		goto out;
	}

	/* Choose the highest speed at or below 'rate' with same PLL. */
	if (reason != SETRATE_CPUFREQ
	    && tgt_s->a11clk_khz < cur_s->a11clk_khz) {
		while (tgt_s->pll != ACPU_PLL_TCXO && tgt_s->pll != cur_s->pll)
			tgt_s--;
	}

	if (strt_s->pll != ACPU_PLL_TCXO)
		plls_enabled |= 1 << strt_s->pll;

	if (reason == SETRATE_CPUFREQ) {
		if (strt_s->pll != tgt_s->pll && tgt_s->pll != ACPU_PLL_TCXO) {
			rc = pc_pll_request(tgt_s->pll, 1);
			if (rc < 0) {
				pr_err("PLL%d enable failed (%d)\n",
					tgt_s->pll, rc);
				goto out;
			}
			plls_enabled |= 1 << tgt_s->pll;
		}
	}
	/* Need to do this when coming out of power collapse since some modem
	 * firmwares reset the VDD when the application processor enters power
	 * collapse. */
	if (reason == SETRATE_CPUFREQ || reason == SETRATE_PC) {
		/* Increase VDD if needed. */
		if (tgt_s->vdd > cur_s->vdd) {
			rc = acpuclk_set_vdd_level(tgt_s->vdd);
			if (rc < 0) {
				pr_err("Unable to switch ACPU vdd (%d)\n", rc);
				goto out;
			}
		}
	}

	/* Set wait states for CPU inbetween frequency changes */
	reg_clkctl = readl(A11S_CLK_CNTL_ADDR);
	reg_clkctl |= (100 << 16); /* set WT_ST_CNT */
	writel(reg_clkctl, A11S_CLK_CNTL_ADDR);

#if PERF_SWITCH_DEBUG
	printk(KERN_INFO "acpuclock: Switching from ACPU rate %u -> %u\n",
	       strt_s->a11clk_khz * 1000, tgt_s->a11clk_khz * 1000);
#endif

	while (cur_s != tgt_s) {
		/*
		 * Always jump to target freq if within 256mhz, regulardless of
		 * PLL. If differnece is greater, use the predefinied
		 * steppings in the table.
		 */
		int d = abs((int)(cur_s->a11clk_khz - tgt_s->a11clk_khz));
		if (d > drv_state.max_speed_delta_khz) {

			if (tgt_s->a11clk_khz > cur_s->a11clk_khz) {
				/* Step up: jump to target PLL as early as
				 * possible so indexing using TCXO (up[-1])
				 * never occurs. */
				if (likely(cur_s->up[tgt_s->pll]))
					cur_s = cur_s->up[tgt_s->pll];
				else
					cur_s = cur_s->up[cur_s->pll];
			} else {
				/* Step down: stay on current PLL as long as
				 * possible so indexing using TCXO (down[-1])
				 * never occurs. */
				if (likely(cur_s->down[cur_s->pll]))
					cur_s = cur_s->down[cur_s->pll];
				else
					cur_s = cur_s->down[tgt_s->pll];
			}

			if (cur_s == NULL) { /* This should not happen. */
				pr_err("No stepping frequencies found. "
					"strt_s:%u tgt_s:%u\n",
					strt_s->a11clk_khz, tgt_s->a11clk_khz);
				rc = -EINVAL;
				goto out;
			}

		} else {
			cur_s = tgt_s;
		}
#if PERF_SWITCH_STEP_DEBUG
		printk(KERN_DEBUG "%s: STEP khz = %u, pll = %d\n",
			__FUNCTION__, cur_s->a11clk_khz, cur_s->pll);
#endif
		if (cur_s->pll != ACPU_PLL_TCXO
		    && !(plls_enabled & (1 << cur_s->pll))) {
			rc = pc_pll_request(cur_s->pll, 1);
			if (rc < 0) {
				pr_err("PLL%d enable failed (%d)\n",
					cur_s->pll, rc);
				goto out;
			}
			plls_enabled |= 1 << cur_s->pll;
		}

		acpuclk_set_div(cur_s);
		drv_state.current_speed = cur_s;
		/* Re-adjust lpj for the new clock speed. */
		loops_per_jiffy = cur_s->lpj;
		udelay(drv_state.acpu_switch_time_us);
	}

	/* Nothing else to do for SWFI. */
	if (reason == SETRATE_SWFI)
		goto out;

	/* Change the AXI bus frequency if we can. */
	if (strt_s->axiclk_khz != tgt_s->axiclk_khz) {
		res = ebi1_clk_set_min_rate(CLKVOTE_ACPUCLK,
						tgt_s->axiclk_khz * 1000);
		if (res < 0)
			pr_warning("Setting AXI min rate failed (%d)\n", res);
	}

	/* Nothing else to do for power collapse if not 7x27. */
	if (reason == SETRATE_PC && !cpu_is_msm7x27())
		goto out;

	/* Disable PLLs we are not using anymore. */
	if (tgt_s->pll != ACPU_PLL_TCXO)
	plls_enabled &= ~(1 << tgt_s->pll);
	for (pll = ACPU_PLL_0; pll <= ACPU_PLL_2; pll++)
		if (plls_enabled & (1 << pll)) {
			res = pc_pll_request(pll, 0);
			if (res < 0)
				pr_warning("PLL%d disable failed (%d)\n",
						pll, res);
		}

	/* Nothing else to do for power collapse. */
	if (reason == SETRATE_PC)
		goto out;

	/* Drop VDD level if we can. */
	if (tgt_s->vdd < strt_s->vdd) {
		res = acpuclk_set_vdd_level(tgt_s->vdd);
		if (res < 0)
			printk(KERN_ERR "acpuclock: Unable to drop ACPU vdd\n");
	}

#if PERF_SWITCH_DEBUG
	printk(KERN_DEBUG "%s: ACPU speed change complete\n", __FUNCTION__);
#endif
out:
	if (reason == SETRATE_CPUFREQ)
		mutex_unlock(&drv_state.lock);
	return rc;
}

static void __init acpuclk_init(void)
{
	struct clkctl_acpu_speed *speed;
	uint32_t div, sel;
	int res;

	/*
	 * Determine the rate of ACPU clock
	 */

	if (!(readl(A11S_CLK_SEL_ADDR) & 0x01)) { /* CLK_SEL_SRC1N0 */
		/* CLK_SRC0_SEL */
		sel = (readl(A11S_CLK_CNTL_ADDR) >> 12) & 0x7;
		/* CLK_SRC0_DIV */
		div = (readl(A11S_CLK_CNTL_ADDR) >> 8) & 0x0f;
	} else {
		/* CLK_SRC1_SEL */
		sel = (readl(A11S_CLK_CNTL_ADDR) >> 4) & 0x07;
		/* CLK_SRC1_DIV */
		div = readl(A11S_CLK_CNTL_ADDR) & 0x0f;
	}

	for (speed = acpu_freq_tbl; speed->a11clk_khz != 0; speed++) {
		if (speed->a11clk_src_sel == sel
		 && (speed->a11clk_src_div == div))
			break;
	}
	if (speed->a11clk_khz == 0) {
		printk(KERN_WARNING "Warning - ACPU clock reports invalid speed\n");
		return;
	}

	drv_state.current_speed = speed;

	res = ebi1_clk_set_min_rate(CLKVOTE_ACPUCLK, speed->axiclk_khz * 1000);
	if (res < 0)
		pr_err("Setting AXI min rate failed!\n");

	printk(KERN_INFO "ACPU running at %d KHz\n", speed->a11clk_khz);

}

unsigned long acpuclk_get_rate(void)
{
	WARN_ONCE(drv_state.current_speed == NULL,
		  "acpuclk_get_rate: not initialized\n");
	if (drv_state.current_speed)
		return drv_state.current_speed->a11clk_khz;
	else
		return 0;
}

uint32_t acpuclk_get_switch_time(void)
{
	return drv_state.acpu_switch_time_us;
}

/*----------------------------------------------------------------------------
 * Clock driver initialization
 *---------------------------------------------------------------------------*/

#define DIV2REG(n)		((n)-1)
#define REG2DIV(n)		((n)+1)
#define SLOWER_BY(div, factor)	div = DIV2REG(REG2DIV(div) * factor)

static void __init acpu_freq_tbl_fixup(void)
{
	unsigned long pll0_l, pll1_l, pll2_l;
	int axi_160mhz = 0, axi_200mhz = 0;
	struct pll_freq_tbl_map *lst;
	struct clkctl_acpu_speed *t;
	unsigned int pll0_needs_fixup = 0;

	/* Wait for the PLLs to be initialized and then read their frequency.
	 */
	do {
		pll0_l = readl(PLLn_L_VAL(0)) & 0x3f;
		cpu_relax();
		udelay(50);
	} while (pll0_l == 0);
	do {
		pll1_l = readl(PLLn_L_VAL(1)) & 0x3f;
		cpu_relax();
		udelay(50);
	} while (pll1_l == 0);
	do {
		pll2_l = readl(PLLn_L_VAL(2)) & 0x3f;
		cpu_relax();
		udelay(50);
	} while (pll2_l == 0);

	printk(KERN_INFO "L val: PLL0: %d, PLL1: %d, PLL2: %d\n",
				(int)pll0_l, (int)pll1_l, (int)pll2_l);

	/* Some configurations run PLL0 twice as fast. Instead of having
	 * separate tables for this case, we simply fix up the ACPU clock
	 * source divider since it's a simple fix up.
	 */
	if (pll0_l == PLL_491_MHZ) {
		pll0_l = PLL_245_MHZ;
		pll0_needs_fixup = 1;
	}

	/* Select the right table to use. */
	for (lst = acpu_freq_tbl_list; lst->tbl != 0; lst++) {
		if (lst->pll0_l == pll0_l && lst->pll1_l == pll1_l
		    && lst->pll2_l == pll2_l) {
			acpu_freq_tbl = lst->tbl;
			break;
		}
	}

	if (acpu_freq_tbl == NULL) {
		pr_crit("Unknown PLL configuration!\n");
		BUG();
	}

	/* Fix up PLL0 source divider if necessary. Also, fix up the AXI to
	 * the max that's supported by the board (RAM used in board).
	 */
	axi_160mhz = (pll0_l == PLL_960_MHZ || pll1_l == PLL_960_MHZ);
	axi_200mhz = (pll2_l == PLL_1200_MHZ);
	for (t = &acpu_freq_tbl[0]; t->a11clk_khz != 0; t++) {

		if (pll0_needs_fixup && t->pll == ACPU_PLL_0)
			SLOWER_BY(t->a11clk_src_div, 2);
		if (axi_160mhz && drv_state.max_axi_khz >= 160000
		    && t->ahbclk_khz > 128000)
			t->axiclk_khz = 160000;
		if (axi_200mhz && drv_state.max_axi_khz >= 200000
		    && t->ahbclk_khz > 160000)
			t->axiclk_khz = 200000;
	}

	t--;
	drv_state.max_axi_khz = t->axiclk_khz;

	/* The default 7x27 ACPU clock plan supports running the AXI bus at
	 * 200 MHz. So we don't classify it as Turbo mode.
	 */
	if (cpu_is_msm7x27())
		return;

	if (!axi_160mhz)
		pr_info("Turbo mode not supported.\n");
	else if (t->axiclk_khz == 160000)
		pr_info("Turbo mode supported and enabled.\n");
	else
		pr_info("Turbo mode supported but not enabled.\n");
}

/* Initalize the lpj field in the acpu_freq_tbl. */
static void __init lpj_init(void)
{
	int i;
	const struct clkctl_acpu_speed *base_clk = drv_state.current_speed;
	for (i = 0; acpu_freq_tbl[i].a11clk_khz; i++) {
		acpu_freq_tbl[i].lpj = cpufreq_scale(loops_per_jiffy,
						base_clk->a11clk_khz,
						acpu_freq_tbl[i].a11clk_khz);
	}
}

static void __init precompute_stepping(void)
{
	int i, step_idx;

#define cur_freq acpu_freq_tbl[i].a11clk_khz
#define step_freq acpu_freq_tbl[step_idx].a11clk_khz
#define cur_pll acpu_freq_tbl[i].pll
#define step_pll acpu_freq_tbl[step_idx].pll

	for (i = 0; acpu_freq_tbl[i].a11clk_khz; i++) {

		/* Calculate max "up" step for each destination PLL */
		step_idx = i + 1;
		while (step_freq && (step_freq - cur_freq)
					<= drv_state.max_speed_delta_khz) {
			acpu_freq_tbl[i].up[step_pll] =
						&acpu_freq_tbl[step_idx];
			step_idx++;
		}
		if (step_idx == (i + 1) && step_freq) {
			pr_crit("Delta between freqs %u KHz and %u KHz is"
				" too high!\n", cur_freq, step_freq);
			BUG();
		}

		/* Calculate max "down" step for each destination PLL */
		step_idx = i - 1;
		while (step_idx >= 0 && (cur_freq - step_freq)
					<= drv_state.max_speed_delta_khz) {
			acpu_freq_tbl[i].down[step_pll] =
						&acpu_freq_tbl[step_idx];
			step_idx--;
		}
		if (step_idx == (i - 1) && i > 0) {
			pr_crit("Delta between freqs %u KHz and %u KHz is"
				" too high!\n", cur_freq, step_freq);
			BUG();
		}
	}
}

static void __init print_acpu_freq_tbl(void)
{
	struct clkctl_acpu_speed *t;
	short down_idx[3];
	short up_idx[3];
	int i, j;

#define FREQ_IDX(freq_ptr) (freq_ptr - acpu_freq_tbl)
	pr_info("Id CPU-KHz PLL DIV AHB-KHz ADIV AXI-KHz "
		"D0 D1 D2 U0 U1 U2\n");

	t = &acpu_freq_tbl[0];
	for (i = 0; t->a11clk_khz != 0; i++) {

		for (j = 0; j < 3; j++) {
			down_idx[j] = t->down[j] ? FREQ_IDX(t->down[j]) : -1;
			up_idx[j] = t->up[j] ? FREQ_IDX(t->up[j]) : -1;
		}

		pr_info("%2d %7d %3d %3d %7d %4d %7d "
			"%2d %2d %2d %2d %2d %2d\n",
			i, t->a11clk_khz, t->pll, t->a11clk_src_div + 1,
			t->ahbclk_khz, t->ahbclk_div + 1, t->axiclk_khz,
			down_idx[0], down_idx[1], down_idx[2],
			up_idx[0], up_idx[1], up_idx[2]);

		t++;
	}
}

static void msm7x25_acpu_pll_hw_bug_fix(void)
{
	unsigned int n;

	/* The 7625 has a hardware bug and in order to select PLL2 we
	 * must program PLL3.  Use the same table, and just fix up the
	 * numbers on this target. */
	for (n = 0; acpu_freq_tbl[n].a11clk_khz != 0; n++)
		if (acpu_freq_tbl[n].pll == ACPU_PLL_2)
			acpu_freq_tbl[n].a11clk_src_sel = 3;
}

static void shared_pll_control_init(void)
{
#define PLL_REMOTE_SPINLOCK_ID "S:7"
	unsigned smem_size;
	remote_spin_lock_init(&pll_lock, PLL_REMOTE_SPINLOCK_ID);
	pll_control = smem_get_entry(SMEM_CLKREGIM_SOURCES, &smem_size);

	if (!pll_control)
		pr_warning("Can't find shared PLL control data structure!\n");
	/* There might be more PLLs than what the application processor knows
	 * about. But the index used for each PLL is guaranteed to remain the
	 * same. */
	else if (smem_size < sizeof(struct shared_pll_control))
		pr_warning("Shared PLL control data structure too small!\n");
	else if (pll_control->version != 0xCCEE0001)
		pr_warning("Shared PLL control version mismatch!\n");
	else {
		pr_info("Shared PLL control available.\n");
		return;
	}

	pll_control = NULL;
	pr_warning("Falling back to proc_comm PLL control.\n");
}

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *clkdata)
{
	pr_info("acpu_clock_init()\n");

	mutex_init(&drv_state.lock);
	drv_state.acpu_switch_time_us = clkdata->acpu_switch_time_us;
	drv_state.max_speed_delta_khz = clkdata->max_speed_delta_khz;
	drv_state.vdd_switch_time_us = clkdata->vdd_switch_time_us;
	drv_state.power_collapse_khz = clkdata->power_collapse_khz;
	drv_state.wait_for_irq_khz = clkdata->wait_for_irq_khz;
	drv_state.max_axi_khz = clkdata->max_axi_khz;
	acpu_freq_tbl_fixup();
	precompute_stepping();
	acpuclk_init();
	lpj_init();
	print_acpu_freq_tbl();
	if (cpu_is_msm7x25())
		msm7x25_acpu_pll_hw_bug_fix();
	if (cpu_is_msm7x27())
		shared_pll_control_init();
	cpufreq_table_init();
	cpufreq_frequency_table_get_attr(freq_table, smp_processor_id());

	init_sysfs_interface();
}

static int voltage_create_attrs(struct device * dev)
{
	int i, rc;
    
	for (i = 0; acpu_freq_tbl[i].a11clk_khz; i++) {
		rc = device_create_file(dev, &acpu_freq_tbl[i].attribute);
		if (rc)
			goto attrs_failed;
	}

	goto succeed;
    
attrs_failed:
	while (i--)
		device_remove_file(dev, &voltage_attrs[i]);

succeed:        
	return rc;
}


static ssize_t voltage_show_vdd(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
	ssize_t ret = -EINVAL;
    unsigned int ifreq = 0;
	char *after;
	unsigned long freq = simple_strtoul(attr->attr.name, &after, 10);

    while (acpu_freq_tbl[ifreq].a11clk_khz != 0 &&
            acpu_freq_tbl[ifreq].a11clk_khz != freq)
        ifreq++;

    // frequency not found in table
    if (acpu_freq_tbl[ifreq].a11clk_khz == 0) {
        pr_err("%s : frequency not found in acpu_freq_tbl (%ld)\n", __FUNCTION__, freq);
        return ret;
    }

#ifdef PERF_SWITCH_DEBUG
    printk("%s for %ld : returning %d\n", __FUNCTION__,
            freq, acpu_freq_tbl[ifreq].vdd); 
#endif
   
	return sprintf(buf, "%d\n", acpu_freq_tbl[ifreq].vdd);
}


static ssize_t voltage_store_vdd(struct device *dev, 
                                        struct device_attribute *attr,
                                        const char *buf,
                                        size_t size)
{
	ssize_t ret = -EINVAL;
    unsigned int ifreq = 0;
    int res;
	char *after;
	unsigned long freq = simple_strtoul(attr->attr.name, &after, 10);
    unsigned int vdd = (unsigned int) simple_strtoul(buf, &after, 10);

    if (vdd < 1 || vdd > 7) {
        pr_err("%s : Out of range vdd value (%d)\n", __FUNCTION__, vdd);
        return ret;
    }

    while (acpu_freq_tbl[ifreq].a11clk_khz != 0 &&
            acpu_freq_tbl[ifreq].a11clk_khz != freq)
        ifreq++;

    // frequency not found in table
    if (acpu_freq_tbl[ifreq].a11clk_khz == 0) {
        pr_err("%s : frequency not found in acpu_freq_tbl (%ld)\n", __FUNCTION__, freq);
        return ret;
    }

#ifdef PERF_SWITCH_DEBUG
    printk("%s : changing vdd for frequency %ld, %d -> %d\n", __FUNCTION__,
            freq, acpu_freq_tbl[ifreq].vdd, vdd); 
#endif

    if (acpu_freq_tbl[ifreq].vdd == vdd)
        return size;

    acpu_freq_tbl[ifreq].vdd = vdd;

    // if we're changing voltage of actual speed
    // we force a change of frequency
#ifdef PERF_SWITCH_DEBUG
    printk("%s : actual clock speed %d\n", __FUNCTION__,
            drv_state.current_speed->a11clk_khz); 
#endif
    if (drv_state.current_speed->a11clk_khz == freq) {
#ifdef PERF_SWITCH_DEBUG
    printk("%s : freq match actual clock speed, updating vdd\n", __FUNCTION__); 
#endif
		res = acpuclk_set_vdd_level(vdd);
		if (res < 0)
			printk(KERN_ERR "acpuclock: Unable to drop ACPU vdd\n");
    }
    return size;
}

static void __init init_sysfs_interface(void) {
	/* init sysfs */
	voltage_class = class_create(THIS_MODULE, VOLT_CLASS);
	if (IS_ERR(voltage_class)) {
		pr_err("Failed to create class(%s)!\n", VOLT_CLASS);
		return;
	}
	voltage_dev = device_create(voltage_class, NULL, 0, NULL, VOLT_DEVICE);
	if (IS_ERR(voltage_dev)) {
		pr_err("Failed to create device(%s)!\n", VOLT_DEVICE);
		return;
	}

	voltage_create_attrs(voltage_dev);
}


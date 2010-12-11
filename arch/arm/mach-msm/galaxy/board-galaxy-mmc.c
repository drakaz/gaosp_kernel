/* linux/arch/arm/mach-msm/board-galaxy-mmc.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>

#include <linux/gpio.h>
#include <linux/io.h>
#include <asm/mach-types.h>

#include <mach/vreg.h>
#include <mach/hardware.h>

#include <asm/mach/mmc.h>
#include <mach/gpio.h>

#include "../devices.h"
#include "../gpio_chip.h"
#include "board-galaxy.h"
#include "../proc_comm.h"

#define DEBUG_SDSLOT_VDD 0

extern int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags);

static unsigned int vreg_enabled;

static unsigned sdcc_cfg_data[][6] = {
	/* SDC1 configs */
	{
	GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
	},
	/* SDC2 configs */
	{
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	},
	{
	/* SDC3 configs */
	GPIO_CFG(88, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(89, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(90, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(91, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(92, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(93, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	},
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static DEFINE_MUTEX(bcm4325_pwr_lock);
#define BCM4325_BT 0
#define BCM4325_WLAN 1

static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc, *vreg_movinand;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int i, rc;

	if (dev_id ==1)
	{
		if (!(test_bit(dev_id, &gpio_sts)^enable)) {
    			//printk("sdcc_setup_gpio test_bit error....\n");
			return;
		}

		if (enable)
			set_bit(dev_id, &gpio_sts);
		else
			clear_bit(dev_id, &gpio_sts);
	}

	for (i = 0; i < ARRAY_SIZE(sdcc_cfg_data[dev_id - 1]); i++) {
		rc = gpio_tlmm_config(sdcc_cfg_data[dev_id - 1][i],
			enable ? GPIO_ENABLE : GPIO_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, sdcc_cfg_data[dev_id - 1][i], rc);
		}
	}
}

static int msm_wifi_setup_power(int dev_id, int on)
{
	printk("\n wifi_setup_power: %d \n", on);
	return 0;
}

static void sdcc_gpio_init(void)
{
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	int rc = 0;
	if (gpio_request(T_FLASH_DETECT, "sdc1_status_irq"))
		pr_err("failed to request gpio sdc1_status_irq\n");
	rc = gpio_tlmm_config(GPIO_CFG(T_FLASH_DETECT, 0, GPIO_INPUT, GPIO_PULL_UP,
				GPIO_2MA), GPIO_ENABLE);
	if (rc)
		printk(KERN_ERR "%s: Failed to configure GPIO %d\n",
				__func__, rc);
#endif
	/* SDC1 GPIOs */
	if (gpio_request(51, "sdc1_data_3"))
		pr_err("failed to request gpio sdc1_data_3\n");
	if (gpio_request(52, "sdc1_data_2"))
		pr_err("failed to request gpio sdc1_data_2\n");
	if (gpio_request(53, "sdc1_data_1"))
		pr_err("failed to request gpio sdc1_data_1\n");
	if (gpio_request(54, "sdc1_data_0"))
		pr_err("failed to request gpio sdc1_data_0\n");
	if (gpio_request(55, "sdc1_cmd"))
		pr_err("failed to request gpio sdc1_cmd\n");
	if (gpio_request(56, "sdc1_clk"))
		pr_err("failed to request gpio sdc1_clk\n");

	/* SDC2 GPIOs */
	if (gpio_request(62, "sdc2_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(63, "sdc2_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(64, "sdc2_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(65, "sdc2_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(66, "sdc2_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(67, "sdc2_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");

	/* SDC3 GPIOs */
	if (gpio_request(88, "sdc3_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(89, "sdc3_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(90, "sdc3_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(91, "sdc3_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(92, "sdc3_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(93, "sdc3_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");
}

//static uint32_t msm_sdcc_setup_power(int dev_id, unsigned int vdd)
static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

//	printk("[MMC] %s @@@@@@\n", __func__);	
	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			vreg_disable(vreg_mmc);
		}
		return 0;
	}

	if (!vreg_sts) {
		rc = vreg_set_level(vreg_mmc, 2850);
		if (!rc)
			rc = vreg_enable(vreg_mmc);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}

static uint32_t galaxy_movinand_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;


	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {

		rc = vreg_disable(vreg_movinand);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		return 0;
	}

	rc = vreg_set_level(vreg_movinand, 3050);
	if (!rc)
		rc = vreg_enable(vreg_movinand);
	else
		printk(KERN_ERR "%s: return val: %d \n",
				__func__, rc);
	return 0;
}

/* dgahn.bcm_mutex */
int bcm4325_set_core_power(unsigned bcm_core, unsigned pow_on)
{
	unsigned gpio_rst, gpio_rst_another;
	
	switch(bcm_core) {
	case BCM4325_BT:
		gpio_rst = BCM4325_BT_RESET;
		gpio_rst_another = BCM4325_WLAN_RESET;
		break;
		
	case BCM4325_WLAN:
		gpio_rst = BCM4325_WLAN_RESET;
		gpio_rst_another = BCM4325_BT_RESET;
		break;
		
	default:
		printk(KERN_ERR "bcm4325_power: Unknown bcm4325 core!\n");
		return -1;
	}

	mutex_lock(&bcm4325_pwr_lock);

#if !defined(CONFIG_MACH_GALAXY_REV03)
	/* if another core is OFF */
	if( gpio_get_value(gpio_rst_another) == 0 )
	{
		/* enable WLAN_BT_REG_ON */
		gpio_direction_output(GPIO_WLAN_BT_REG_ON, pow_on); 
		printk("bcm4325_power: Set WLAN_BT_REG_ON %s because %s is OFF now.\n", 
			gpio_get_value(GPIO_WLAN_BT_REG_ON)?"High":"Low", bcm_core?"BT":"WLAN");
		msleep(150);
	}
#endif
	/* enable specified core */
	gpio_direction_output(gpio_rst, pow_on);
	printk("bcm4325_power: Set %s %s\n", 
		bcm_core?"WLAN_RESET":"BT_RESET", gpio_get_value(gpio_rst)?"High":"Low");

	mutex_unlock(&bcm4325_pwr_lock);

	return 0;
}

EXPORT_SYMBOL(bcm4325_set_core_power);

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int galaxy_sdcc_slot_status(struct device *dev)
{
	int rc;
	rc = gpio_get_value( T_FLASH_DETECT );

	rc = rc?0:1 ;
	return rc;
}
#endif

static struct mmc_platform_data galaxy_sdcc_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status         = galaxy_sdcc_slot_status,
	.status_irq	= MSM_GPIO_TO_INT(T_FLASH_DETECT),
	.irq_flags      = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
};

static unsigned int msm_movinand_status(struct device *dev)
{
	return 1; // Movinand is not removable.
}

static struct mmc_platform_data movinand_sdcc_data = {
	.ocr_mask	= MMC_VDD_30_31,
	.translate_vdd	= galaxy_movinand_setup_power,
};

static uint32_t wifi_on_gpio_table[] = {			// needed to set GPIO. but...
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT3 */
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT2 */
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT1 */
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT0 */
	GPIO_CFG(GPIO_WLAN_HOST_WAKE, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),  /* WLAN_HOST_WAKE	*/
#if !defined(CONFIG_MACH_GALAXY_REV05)
	GPIO_CFG(20, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN_HOST_WAKE	*/
#endif
};

static uint32_t wifi_off_gpio_table[] = {
	GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	GPIO_CFG(64, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	GPIO_CFG(65, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	GPIO_CFG(66, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT1 */
	GPIO_CFG(67, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT0 */
	GPIO_CFG(GPIO_WLAN_HOST_WAKE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),  /* WLAN_HOST_WAKE */
#if !defined(CONFIG_MACH_GALAXY_REV05)
	GPIO_CFG(20, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
#endif
};

static int wifi_cd = 0;           /* WIFI virtual 'card detect' status */
//static struct vreg *vreg_wifi_osc;      /* WIFI 32khz oscilator */

// dh0421.hwang for wifi
static unsigned int wifi_status(struct device *dev)
{
	return wifi_cd;			// wifi 'card detect' status return
}

static void (*wifi_status_cb)(int card_present, void *dev_id);

static void *wifi_status_cb_devid;

static struct sdio_embedded_func wifi_func[] = {
	{SDIO_CLASS_WLAN, 512},
	{SDIO_CLASS_WLAN, 512},
};

static struct embedded_sdio_data wifi_emb_data = {
	.cis    = {
		.vendor         = 0x02d0,
		.device         = 0x4325,
		.blksize        = 512,
		/*.max_dtr      = 24000000,  Max of chip - no worky on Trout */
		.max_dtr        =   48000000,
	},
	.cccr   = {
		.multi_block    = 1,
		.low_speed      = 1,
		.wide_bus       = 1,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.funcs  = &wifi_func,
	.num_funcs = 2,
};

static int wifi_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
//	printk("%s:%d is called.....\n", __func__, __LINE__);
	if(wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static struct mmc_platform_data wifi_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.status		= wifi_status,
	.register_status_notify	= wifi_status_register,
	.embedded_sdio	= &wifi_emb_data,
	.translate_vdd	= msm_wifi_setup_power,
};

int galaxy_wifi_set_carddetect(int val)
{
	wifi_cd = val;
	if(wifi_status_cb)
	{
		wifi_status_cb(val, wifi_status_cb_devid);
	}
	else
	{
		printk("%s: Nobody to notify\n", __func__);
	}
  return 0;
}
EXPORT_SYMBOL(galaxy_wifi_set_carddetect);

static int wifi_power_state;


int galaxy_wifi_card_power(int on)
{
	printk("%s: %d\n", __func__, on);
    
    if(on)
    {
		config_gpio_table(wifi_on_gpio_table, ARRAY_SIZE(wifi_on_gpio_table));
		bcm4325_set_core_power(BCM4325_WLAN, 1);  /* dgahn.bcm_mutex */
    }
    else
    {
		config_gpio_table(wifi_off_gpio_table, ARRAY_SIZE(wifi_off_gpio_table));
		bcm4325_set_core_power(BCM4325_WLAN, 0);  /* dgahn.bcm_mutex */
    }
    
    return 0;
}
EXPORT_SYMBOL(galaxy_wifi_card_power);

int wifi_power(int on)
{

	printk("%s: %d\n", __func__, on);

	/* Power on the BCM4325 */
    galaxy_wifi_card_power(on);

    /* Do the mmc_rescan */
    galaxy_wifi_set_carddetect(on);

	return 0;
}
EXPORT_SYMBOL(wifi_power);

int galaxy_wifi_reset(int on) {
  // nothing
  return 0;
}

void __init galaxy_init_mmc(void)
{
	vreg_mmc = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed ($ld)\n", __func__, PTR_ERR(vreg_mmc));
		return;
	}

	vreg_movinand = vreg_get(NULL, "wlan");
	if (IS_ERR(vreg_movinand)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n", __func__, PTR_ERR(vreg_movinand));
		return;
	}

	/* wake up the system when inserting or removing SD card */
	set_irq_wake(MSM_GPIO_TO_INT(T_FLASH_DETECT), 1);

	sdcc_gpio_init();

	msm_add_sdcc(3, &movinand_sdcc_data, 0, 0);
	msm_add_sdcc(1, &galaxy_sdcc_data, 0, 0);
	msm_add_sdcc(2, &wifi_data, 0, 0);
}

#if defined(CONFIG_DEBUG_FS)
static int mmc_dbg_wifi_reset_get(void *data, u64 *val)
{
	return 0;
}

static int mmc_dbg_wifi_reset_set(void *data, u64 *val)
{
	return 0;
}

static int mmc_dbg_wifi_cd_get(void *data, u64 *val)	// galaxy_wifi_set_carddetect((int) val);
{
	*val = wifi_cd;
	return 0;
}

static int mmc_dbg_wifi_cd_set(void *data, u64 *val)
{
	galaxy_wifi_set_carddetect((int) val);
	return 0;
}

static int mmc_dbg_wifi_pwr_get(void *data, u64 *val)
{
	*val = wifi_power_state;
	return 0;
}

static int mmc_dbg_wifi_pwr_set(void *data, u64 *val)
{
	wifi_power((int)val);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi_reset_fops,
		mmc_dbg_wifi_reset_get,
		mmc_dbg_wifi_reset_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi_cd_fops,
		mmc_dbg_wifi_cd_get,
		mmc_dbg_wifi_cd_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi_pwr_fops,
		mmc_dbg_wifi_pwr_get,
		mmc_dbg_wifi_pwr_set, "%llu\n");

static int __init galaxy_mmc_dbg_init(void)
{
	struct dentry *dent;

	if (!machine_is_galaxy())
		return 0;

	dent = debugfs_create_dir("galaxy_mmc_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("wifi_reset", 0644, dent, NULL,
			    &mmc_dbg_wifi_reset_fops);
	debugfs_create_file("wifi_cd", 0644, dent, NULL,
			    &mmc_dbg_wifi_cd_fops);
	debugfs_create_file("wifi_pwr", 0644, dent, NULL,
			    &mmc_dbg_wifi_pwr_fops);

	return 0;
}

device_initcall(galaxy_mmc_dbg_init);

#endif

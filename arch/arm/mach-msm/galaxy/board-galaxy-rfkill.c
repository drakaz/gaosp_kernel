/*
 * Copyright (C) 2008 Google, Inc.
 * Author: Nick Pelly <npelly@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* Control bluetooth power for trout platform */
/* BT on/off */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>

#include <asm/gpio.h>
#include <mach/hardware.h>

#include "../devices.h"

//#define CONF_SETTING_GPIO

/* dgahn.bcm_mutex */
extern int bcm4325_set_core_power(unsigned bcm_core, unsigned pow_on);
#define BCM4325_BT 0

#if 1
int btsleep_start(void);
void btsleep_stop(void);
#endif

#ifdef CONF_SETTING_GPIO
/* Setting GPIOs for BT when pwr on/off as the request from BRCM */
static unsigned bt_config_power_on[] = {
	GPIO_CFG(BCM4325_BT_WAKE,   0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* WAKE */
	GPIO_CFG(GPIO_BT_UART_RTS,  2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* RFR_N */
	GPIO_CFG(GPIO_BT_UART_CTS,  2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* CTS_N */
	GPIO_CFG(GPIO_BT_UART_RXD,  2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* RX */
	GPIO_CFG(GPIO_BT_UART_TXD,  3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* TX */
	GPIO_CFG(GPIO_BT_PCM_DOUT,  1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),/* PCM_DOUT */
	GPIO_CFG(GPIO_BT_PCM_DIN,   1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(GPIO_BT_PCM_SYNC,  2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),/* PCM_SYNC */
	GPIO_CFG(GPIO_BT_PCM_CLK,   2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_CLK */
	GPIO_CFG(GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* HOST_WAKE */
};
static unsigned bt_config_power_off[] = {
	GPIO_CFG(BCM4325_BT_WAKE, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* WAKE */
	GPIO_CFG(GPIO_BT_UART_RTS, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* RFR */
	GPIO_CFG(GPIO_BT_UART_CTS, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* CTS */
	GPIO_CFG(GPIO_BT_UART_RXD, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* Rx */
	GPIO_CFG(GPIO_BT_UART_TXD, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* Tx */
	GPIO_CFG(GPIO_BT_PCM_DOUT, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(GPIO_BT_PCM_DIN, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(GPIO_BT_PCM_SYNC, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(GPIO_BT_PCM_CLK, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* PCM_CLK */
	GPIO_CFG(GPIO_BT_HOST_WAKE, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* HOST_WAKE */
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

static void config_gpios_bt_on(void)
{
	config_gpio_table(bt_config_power_on, ARRAY_SIZE(bt_config_power_on));
}

static void config_gpios_bt_off(void)
{
	config_gpio_table(bt_config_power_off, ARRAY_SIZE(bt_config_power_off));
}
#endif  // CONF_SETTING_GPIO_

void rfkill_switch_all(enum rfkill_type type, enum rfkill_state state);

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4325_bt_reset";		// how to? -> It needs to be edited

/* /sys/class/rfkill/rfkill0/state */
static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	switch (state) 
	{
	/* write 1 to state node */
	case RFKILL_STATE_UNBLOCKED:  // write 1 to power-up
#ifdef CONF_SETTING_GPIO
		config_gpios_bt_on();
		printk("%s(%d): GPIOs've been set as BT is ON\n", __FUNCTION__ ,__LINE__);
#endif

		bcm4325_set_core_power(BCM4325_BT, 1);  /* dgahn.bcm_mutex */
		btsleep_start();
		break;

	/* write 0 to state node */
	case RFKILL_STATE_SOFT_BLOCKED:  // write 0 to power-off
		bcm4325_set_core_power(BCM4325_BT, 0);  /* dgahn.bcm_mutex */

#ifdef CONF_SETTING_GPIO
		config_gpios_bt_off();
		printk("%s(%d): GPIOs've been set as BT is OFF\n", __FUNCTION__ ,__LINE__);
#endif

		btsleep_stop();
		break;
        
	default:
		printk(KERN_ERR "bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}

static int galaxy_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	enum rfkill_state default_state = RFKILL_STATE_SOFT_BLOCKED;  /* off */

	/* default to bluetooth off */
	rfkill_set_default(RFKILL_TYPE_BLUETOOTH, default_state);
	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);
	if (!bt_rfk)
		return -ENOMEM;

	bt_rfk->name = bt_name;
	bt_rfk->state = default_state;
	/* userspace cannot take exclusive control */
	bt_rfk->user_claim_unsupported = 1;
	bt_rfk->user_claim = 0;
	bt_rfk->data = NULL;  // user data
	bt_rfk->toggle_radio = bluetooth_set_power;

	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_free(bt_rfk);
	return rc;
}

static int galaxy_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_free(bt_rfk);

	return 0;
}

static struct platform_driver galaxy_rfkill_driver = {
	.probe = galaxy_rfkill_probe,
	.remove = galaxy_rfkill_remove,
	.driver = {
		.name = "galaxy_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init galaxy_rfkill_init(void)
{
	return platform_driver_register(&galaxy_rfkill_driver);
}

static void __exit galaxy_rfkill_exit(void)
{
	platform_driver_unregister(&galaxy_rfkill_driver);
}

module_init(galaxy_rfkill_init);
module_exit(galaxy_rfkill_exit);
MODULE_DESCRIPTION("galaxy rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");

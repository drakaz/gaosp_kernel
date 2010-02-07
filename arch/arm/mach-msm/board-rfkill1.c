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

/* Control Bluetooth BT_WAKE GPIO pin */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/wakelock.h>

#include <asm/gpio.h>
#include <mach/hardware.h>

#include "devices.h"

#define CONF_BT_LPM

#ifdef CONF_BT_LPM
#define CONF_BT_LPM_TX
#define CONF_BT_LPM_WAKELOCK
//#define CONF_BT_LPM_TX_TIMER

#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include <mach/msm_serial_hs.h>

//#define dbgk(fmt, arg...)  printk( "%s(%d): " fmt " ###\n" , __func__, __LINE__, ## arg)
#define dbgk(fmt, arg...)  if(bsi->debug) printk(fmt "\n" , ## arg)

#define PROC_DIR	"bluetooth/sleep"
struct proc_dir_entry *bluetooth_dir, *sleep_dir;

/* external functions in high-speed uart driver */
int msm_hs_uart_power_ex(int on);
struct uart_port * msm_hs_get_uport_ex(unsigned int line);

void btsleep_sleep_wakeup(void);
static void btsleep_sleep_work(struct work_struct *work);

/* work queue */
DECLARE_DELAYED_WORK(sleep_workqueue, btsleep_sleep_work);
/* Macros for handling work_queue function */
#define btsleep_rx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define btsleep_tx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define btsleep_rx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define btsleep_tx_idle()     schedule_delayed_work(&sleep_workqueue, 0)

static unsigned long flags;
/* state variable names and bit positions */
#define BT_PROTO	0x01
#define BT_TXDATA	0x02
#define BT_ASLEEP	0x04

static spinlock_t rw_lock;  /* Lock for state transitons */

struct btsleep_info {
	unsigned host_wake;
	unsigned ext_wake;
	unsigned host_wake_irq;
	struct uart_port *uport;
	unsigned debug;
};

static struct btsleep_info *bsi;
static atomic_t open_count = ATOMIC_INIT(1); /* module_usage */
static struct tasklet_struct hostwake_task;  /* tasklet to respond to change in host_wake line */
#ifdef CONF_BT_LPM_TX_TIMER
static struct timer_list tx_timer;  /* transmission timer */
#define TX_TIMER_INTERVAL	1  /* 1 second */
#endif
#endif

void rfkill_switch_all(enum rfkill_type type, enum rfkill_state state);

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4325_bt_wake";		// how to? -> It needs to be edited

#ifdef CONF_BT_LPM_WAKELOCK
static struct wake_lock rfkill1_wake_lock;
#endif

#ifdef CONF_BT_LPM
static int btsleep_hsuart_power(int on)
{
	dbgk( "msm_hs_uart_power_ex: hs_uart clk %s " , on?"ON":"OFF");
	return msm_hs_uart_power_ex(on);
}

static inline int btsleep_can_sleep(void)
{
	int ret;

	/* check if BT_WAKE GPIO and BT_HOST_WAKE GPIO are both deasserted */
	ret = !gpio_get_value(bsi->ext_wake) && !gpio_get_value(bsi->host_wake);
	return ret;
}

void btsleep_sleep_wakeup(void)
{
	if (test_bit(BT_ASLEEP, &flags)) {
#ifdef CONF_BT_LPM_WAKELOCK
		wake_lock(&rfkill1_wake_lock);  /* hold wakelock before activating UART clk */
#endif
#ifdef CONF_BT_LPM_TX_TIMER
		dbgk("waking up... (*)restart timer");
		/* Start the timer */
		tx_timer.expires = jiffies + (TX_TIMER_INTERVAL * HZ);
		add_timer(&tx_timer);
#else
		dbgk("(!) Waking up... ");
#endif
		//gpio_set_value(bsi->ext_wake, 0); /* assert BT_WAKE */
		
		clear_bit(BT_ASLEEP, &flags);
		/*Activating UART */
		btsleep_hsuart_power(1);
	}
	dbgk("already wake-up");
}

static void btsleep_sleep_work(struct work_struct *work)
{
	if (btsleep_can_sleep()) {
		/* already asleep, this is an error case */
		if (test_bit(BT_ASLEEP, &flags)) {
			dbgk("sleep_cond OK->already asleep");
			return;
		}

		if (msm_hs_tx_empty(bsi->uport)) {
			dbgk("sleep_cond OK->tx_empty-> (!) Going to Sleep...");
			set_bit(BT_ASLEEP, &flags);
			/*Deactivating UART */
			btsleep_hsuart_power(0);
#ifdef CONF_BT_LPM_WAKELOCK
			wake_unlock(&rfkill1_wake_lock);  /* release wakelock after deactivating UART clk */
#endif
		} 
#ifdef CONF_BT_LPM_TX_TIMER
		else {
			dbgk("sleep_cond OK->tx_not_empty->(*)restart timer...");
			tx_timer.expires = jiffies + (TX_TIMER_INTERVAL * HZ);
			add_timer(&tx_timer);
			return;
		}
#endif
	} 
	else {
		if (bsi->debug) printk("sleep_cond NOK->");
		btsleep_sleep_wakeup();
	}
}

static void btsleep_hostwake_task(unsigned long data)
{
	int val;

	spin_lock(&rw_lock);

	val = gpio_get_value(bsi->host_wake);
	if (val) {
		btsleep_rx_busy();
	}
	else {
		btsleep_rx_idle();
	}

	spin_unlock(&rw_lock);

	dbgk("[Det] HOST_WAKE(%s", val?"H) -> rx_busy()":"L) -> rx_idle()");
}

static irqreturn_t btsleep_hostwake_isr(int irq, void *dev_id)
{
	set_irq_type(bsi->host_wake_irq, gpio_get_value(GPIO_BT_HOST_WAKE) ?
									IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	/* schedule a tasklet to handle the change in the host wake line */
	tasklet_schedule(&hostwake_task);

	return IRQ_HANDLED;
}

#ifdef CONF_BT_LPM_TX_TIMER
static void btsleep_tx_timer_expire(unsigned long data)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	dbgk("(*) Tx timer expired...");

	/* were we silent during the last timeout? */
	if (gpio_get_value(bsi->ext_wake) ) {
		dbgk("Tx has been idle");
		btsleep_tx_idle();
	} 
	else {
		dbgk("Tx data during last period. (*)restart timer");
		tx_timer.expires = jiffies + (TX_TIMER_INTERVAL*HZ);
		add_timer(&tx_timer);
	}

	spin_unlock_irqrestore(&rw_lock, irq_flags);
}
#endif

/**
 * Starts the Sleep-Mode Protocol on the Host.
 */
int btsleep_start(void)
{
	int retval;
	unsigned long irq_flags;

	printk("btsleep_start: called.\n");
	spin_lock_irqsave(&rw_lock, irq_flags);

	if (test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return 0;
	}

	spin_unlock_irqrestore(&rw_lock, irq_flags);

	if (!atomic_dec_and_test(&open_count)) {
		atomic_inc(&open_count);
		return -EBUSY;
	}

#ifdef CONF_BT_LPM_TX_TIMER
	printk("btsleep_start: Initially (*)start timer\n");
	/* start the timer */
	tx_timer.expires = jiffies + (TX_TIMER_INTERVAL*HZ);
	add_timer(&tx_timer);
#endif

	/* GPIO setting for BT_HOST_WAKE */
	retval = gpio_request(GPIO_BT_HOST_WAKE , "bt_host_wake");
	if (retval  < 0) {
		printk(KERN_ERR "<!> gpio_request failed!!!\n");
		goto fail_gpio_request;
	}

	retval = gpio_direction_input(GPIO_BT_HOST_WAKE);
	if (retval < 0) {
		printk(KERN_ERR "<!> gpio_direction_input failed!!!\n");
		goto fail_gpio_direction_input;
	}

	/* IRQ setting for BT_HOST_WAKE */
	//set_irq_flags(bsi->host_wake_irq, IRQF_VALID);  /* TO_CHK: is this needed? */
	retval = request_irq(bsi->host_wake_irq, btsleep_hostwake_isr,
				IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW, "bt_host_wake", NULL);
	if (retval  < 0) {
		printk(KERN_ERR "Couldn't acquire BT_HOST_WAKE IRQ\n");
		goto fail_request_irq;
	}

	dbgk("btsleep_start: BT_HOST_WAKE is %s\n", gpio_get_value(bsi->host_wake)?"High":"Low");

	retval = set_irq_wake(bsi->host_wake_irq, 1);
	//retval = enable_irq_wake(bsi->host_wake_irq);
	if (retval < 0) {
		printk(KERN_ERR "Couldn't enable BT_HOST_WAKE as wakeup interrupt\n");
		goto fail_set_irq_wake;
	}

	set_bit(BT_PROTO, &flags);

	return 0;

fail_set_irq_wake:
	free_irq(bsi->host_wake_irq, NULL);
fail_request_irq:
fail_gpio_direction_input:
	gpio_free(GPIO_BT_HOST_WAKE);
fail_gpio_request:
#ifdef CONF_BT_LPM_TX_TIMER
	del_timer(&tx_timer);
#endif
	atomic_inc(&open_count);

	return retval;
}
EXPORT_SYMBOL(btsleep_start);

/**
 * Stop the Sleep-Mode Protocol on the Host.
 */
void btsleep_stop(void)
{
	unsigned long irq_flags;

	printk("btsleep_stop: called.\n");
	spin_lock_irqsave(&rw_lock, irq_flags);

	if (!test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return;
	}

#ifdef CONF_BT_LPM_TX_TIMER
	del_timer(&tx_timer);
#endif
	clear_bit(BT_PROTO, &flags);

	if (test_bit(BT_ASLEEP, &flags)) {
		clear_bit(BT_ASLEEP, &flags);
		btsleep_hsuart_power(1);
	}

	atomic_inc(&open_count);

	spin_unlock_irqrestore(&rw_lock, irq_flags);

	//if (disable_irq_wake(bsi->host_wake_irq))
	if(set_irq_wake(bsi->host_wake_irq, 0))
		printk(KERN_ERR "Couldn't disable hostwake IRQ wakeup mode\n");
	free_irq(bsi->host_wake_irq, NULL);
	gpio_free(GPIO_BT_HOST_WAKE);

#ifdef CONF_BT_LPM_WAKELOCK
	wake_unlock(&rfkill1_wake_lock);
#endif
}
EXPORT_SYMBOL(btsleep_stop);

/**
 * Read the BT_WAKE GPIO pin value via the proc interface.
 * When this function returns, page will contain a 1 if the pin is high, 0 otherwise.
 */
static int btsleep_read_proc_btwake(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "btwake:%u\n", gpio_get_value(bsi->ext_wake));
}

/**
 * Write the BT_WAKE GPIO pin value via the proc interface.
 */
static int btsleep_write_proc_btwake(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {
		gpio_set_value(bsi->ext_wake, 0);
		printk("GPIO_%d is set to %d\n", bsi->ext_wake, 0);
	} else if (buf[0] == '1') {
		gpio_set_value(bsi->ext_wake, 1);
		printk("GPIO_%d is set to %d\n", bsi->ext_wake, 0);
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}

static int btsleep_read_proc_debug(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "debug: %u\n", bsi->debug);
}

static int btsleep_write_proc_debug(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {
			bsi->debug = 0;
			printk("Set debug: %d\n", bsi->debug);
			btsleep_hsuart_power(bsi->debug);
	}
	else if (buf[0] == '1') {
			bsi->debug = 1;
			printk("Set debug: %d\n", bsi->debug);
			btsleep_hsuart_power(bsi->debug);
	} 
	else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}

/**
 * Read the BT_HOST_WAKE GPIO pin value via the proc interface.
 * When this function returns, page will contain a 1 if the pin is high, 0 otherwise.
 */
static int btsleep_read_proc_hostwake(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "hostwake: %u \n", gpio_get_value(bsi->host_wake));
}


/**
 * Read the low-power status of the Host via the proc interface.
 * When this function returns, page contains a 1 if the Host is asleep, 0 otherwise.
 */
static int btsleep_read_proc_asleep(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	unsigned int asleep;

	asleep = test_bit(BT_ASLEEP, &flags) ? 1 : 0;
	*eof = 1;
	return sprintf(page, "asleep: %u\n", asleep);
}

/**
 * Read the low-power protocol being used by the Host via the proc interface.
 * When this function returns, page will contain a 1 if the Host is using the Sleep Mode Protocol, 0 otherwise.
 */
static int btsleep_read_proc_proto(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	unsigned int proto;

	proto = test_bit(BT_PROTO, &flags) ? 1 : 0;
	*eof = 1;
	return sprintf(page, "proto: %u\n", proto);
}

/**
 * Modify the low-power protocol used by the Host via the proc interface.
 */
static int btsleep_write_proc_proto(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char proto;

	if (count < 1)
		return -EINVAL;

	if (copy_from_user(&proto, buffer, 1))
		return -EFAULT;

	if (proto == '0')
		btsleep_stop();
	else
		btsleep_start();

	/* claim that we wrote everything */
	return count;
}

static int __init btsleep_init(void)
{
	int retval;
	struct proc_dir_entry *ent;

	printk("MSM BT Low Power Mode Driver\n");

	/* Creating /proc/bluetooth directory */
	bluetooth_dir = proc_mkdir("bluetooth", NULL);
	if (bluetooth_dir == NULL) {
		printk(KERN_ERR "Unable to create /proc/bluetooth directory\n");
		return -ENOMEM;
	}

	/* Creating /proc/bluetooth/sleep directory */
	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (sleep_dir == NULL) {
		printk(KERN_ERR "Unable to create /proc/%s directory\n", PROC_DIR);
		return -ENOMEM;
	}

	/* Creating RW "/proc/bluetooth/sleep/btwake" entry */
	ent = create_proc_entry("btwake", 0, sleep_dir);
	if (ent == NULL) {
		printk(KERN_ERR "Unable to create /proc/%s/btwake entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
	ent->read_proc = btsleep_read_proc_btwake;
	ent->write_proc = btsleep_write_proc_btwake;

	/* Creating RW "/proc/bluetooth/sleep/debug" entry */
	ent = create_proc_entry("debug", 0, sleep_dir);
	if (ent == NULL) {
		printk(KERN_ERR "Unable to create /proc/%s/debug entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
	ent->read_proc = btsleep_read_proc_debug;
	ent->write_proc = btsleep_write_proc_debug;

	/* Creating RO "/proc/bluetooth/sleep/hostwake" entries */
	if (create_proc_read_entry("hostwake", 0, sleep_dir, 
							btsleep_read_proc_hostwake, NULL) == NULL) 
	{
		printk(KERN_ERR "Unable to create /proc/%s/hostwake entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* Creating RW "/proc/bluetooth/sleep/proto" entries */
	ent = create_proc_entry("proto", 0, sleep_dir);
	if (ent == NULL) {
		printk(KERN_ERR "Unable to create /proc/%s/proto entry\n", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
	ent->read_proc = btsleep_read_proc_proto;
	ent->write_proc = btsleep_write_proc_proto;

	/* Creating RO "/proc/bluetooth/sleep/asleep" entries */
	if (create_proc_read_entry("asleep", 0,	sleep_dir, 
							btsleep_read_proc_asleep, NULL) == NULL) 
	{
		printk(KERN_ERR "Unable to create /proc/%s/asleep entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* clear all status bits */
	flags = 0; 

	/* Initialize spinlock. */
	spin_lock_init(&rw_lock);

#ifdef CONF_BT_LPM_TX_TIMER
	/* Initialize timer */
	init_timer(&tx_timer);
	tx_timer.function = btsleep_tx_timer_expire;
	tx_timer.data = 0;
#endif

	/* initialize host wake tasklet */
	tasklet_init(&hostwake_task, btsleep_hostwake_task, 0);

#ifdef CONF_BT_LPM_WAKELOCK
	wake_lock_init(&rfkill1_wake_lock, WAKE_LOCK_SUSPEND, "board-rfkill1");
#endif

	return 0;

fail:
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("debug", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
	
	return retval;
}
#endif

/* /sys/class/rfkill/rfkill1/state */
static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
		
	/* write 1 to state node */		
	case RFKILL_STATE_UNBLOCKED:  // write 1 to sleep
		gpio_direction_output(BCM4325_BT_WAKE, 0);
		printk("rfk1: [UNBLOCKED] write 1 to sleep (BT_WAKE: %s)\n",
				(gpio_get_value(BCM4325_BT_WAKE))?"High":"Low" );
#ifdef CONF_BT_LPM_TX
		if (test_bit(BT_PROTO, &flags)) {
			unsigned long irq_flags;
			spin_lock_irqsave(&rw_lock, irq_flags);

			//dbgk("Tx has been idle");
			dbgk("[Set] BT_WAKE(L) -> tx_idle()");
			btsleep_tx_idle();

			spin_unlock_irqrestore(&rw_lock, irq_flags);
		}
#endif
		break;

	/* write 0 to state node */
	case RFKILL_STATE_SOFT_BLOCKED:  // write 0 to wake
		gpio_direction_output(BCM4325_BT_WAKE, 1);
		printk("rfk1: [SOFT_BLOCKED] write 0 to wake (BT_WAKE: %s)\n",
				(gpio_get_value(BCM4325_BT_WAKE))?"High":"Low");
#ifdef CONF_BT_LPM_TX
		if (test_bit(BT_PROTO, &flags)) {
			if (bsi->debug) printk("[Set] BT_WAKE(H) -> ");
			btsleep_sleep_wakeup();
		}
#endif
		break;

	default:
		printk(KERN_ERR "bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}

static int __init orion_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;

#ifdef CONF_BT_LPM
	rc = btsleep_init();
	if (rc) {
		printk(KERN_ERR "btsleep_init() failed!\n");
		return rc;
	}

	bsi = kzalloc(sizeof(struct btsleep_info), GFP_KERNEL);
	if (!bsi) {
		printk(KERN_ERR "kzalloc failed!\n");
		return -ENOMEM;
	}

	bsi->host_wake = GPIO_BT_HOST_WAKE;
	bsi->ext_wake = BCM4325_BT_WAKE;
	bsi->host_wake_irq = gpio_to_irq(GPIO_BT_HOST_WAKE);
	if (bsi->host_wake_irq < 0) {
		printk(KERN_ERR "### gpio_to_irq for BT_HOST_WAKE failed!!!\n");
		return bsi->host_wake_irq;
	}
	bsi->uport= msm_hs_get_uport_ex(0);  // ttyHS0
	bsi->debug = 0;

	//dbgk("bsi->uport = 0x%X", bsi->uport);
#endif

	/* default to bluetooth off */
	rfkill_switch_all(RFKILL_TYPE_BLUETOOTH, RFKILL_STATE_SOFT_BLOCKED);
	bluetooth_set_power(NULL, RFKILL_STATE_SOFT_BLOCKED);

	bt_rfk = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);
	if (!bt_rfk)
		return -ENOMEM;

	bt_rfk->name = bt_name;
	bt_rfk->state = RFKILL_STATE_SOFT_BLOCKED;
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

static struct platform_driver orion_rfkill1_driver = {
	.probe = orion_rfkill_probe,
	.driver = {
		.name = "orion_rfkill1",
		.owner = THIS_MODULE,
	},
};

static int __init orion_rfkill_init(void)
{
	return platform_driver_register(&orion_rfkill1_driver);
}

module_init(orion_rfkill_init);
MODULE_DESCRIPTION("orion BT LPM");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");

/*
 * Copyright (C) 2007-2008 SAMSUNG Corporation.
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

#include <linux/i2c.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/input.h>
//#include <mach/rpc_pm.h>
#include <mach/vreg.h>
#include <asm/io.h>
//#include <asm/mach-types.h>

//#include <linux/fsa9480.h> /* define ioctls */
#include <linux/uaccess.h>

#define ALLOW_USPACE_RW		1

static struct i2c_client *pclient;

static int opened;
static int pclk_set;

#define FSA9480_DEVICE_ID_ADD 0x01
#define FSA9480_DEVICE_TYPE1_ADD 0x0A
#define FSA9480_DEVICE_TYPE2_ADD 0x0B



DECLARE_MUTEX(fsa_sem);

struct fsa9480_data {
	struct work_struct work;
};

static DECLARE_WAIT_QUEUE_HEAD(g_data_ready_wait_queue);


#define I2C_WRITE(reg,data) if (!fsa9480_i2c_write(reg, data) < 0) return -EIO
#define I2C_READ(reg,data) if (fsa9480_i2c_read(reg,data) < 0 ) return -EIO


int fsa9480_i2c_tx_data(char* txData, int length)
{
	int rc; 

	struct i2c_msg msg[] = {
		{
			.addr = pclient->addr,
			.flags = 0,
			.len = length,
			.buf = txData,		
		},
	};
    
	rc = i2c_transfer(pclient->adapter, msg, 1);
	if (rc < 0) {
//		printk(KERN_ERR "fsa9480: fsa9480_i2c_tx_data error %d\n", rc);
		return rc;
	}
	return 0;
}

	
static int fsa9480_i2c_write(unsigned char u_addr, unsigned char u_data)
{
	int rc;
	unsigned char buf[2];

	buf[0] = u_addr;
	buf[1] = u_data;
    
	rc = fsa9480_i2c_tx_data(buf, 2);
	if(rc < 0)
		printk(KERN_ERR "fsa9480: txdata error %d add:0x%02x data:0x%02x\n", rc, u_addr, u_data);
	return rc;	
}


static int fsa9480_i2c_rx_data(char* rxData, int length)
{
	int rc;

	struct i2c_msg msgs[] = {
		{
			.addr = pclient->addr,
			.flags = 0,      
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = pclient->addr,
			.flags = I2C_M_RD,//|I2C_M_NO_RD_ACK,
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(pclient->adapter, msgs, 2);
      
	if (rc < 0) {
//		printk(KERN_ERR "fsa9480: fsa9480_i2c_rx_data error %d\n", rc);
		return rc;
	}
      
	return 0;
}

static int fsa9480_i2c_read(unsigned char u_addr, unsigned char *pu_data)
{
	int rc;
	unsigned char buf;

	buf = u_addr;
	rc = fsa9480_i2c_rx_data(&buf, 1);
	if (!rc)
		*pu_data = buf;
	else 
		printk(KERN_ERR "fsa9480: rxdata error %d add:0x%02x\n", rc, u_addr);
	return rc;	
}


static void fsa9480_chip_init(void)
{
	printk(KERN_INFO "fsa9480: init\n");
	if (!pclient) 
		return;

	msleep(2);
	printk(KERN_INFO "fsa9480: fsa9480 sensor init sequence done\n");
}

static int fsa9480_open(struct inode *ip, struct file *fp)
{
	int rc = -EBUSY;
	down(&fsa_sem);
	printk(KERN_INFO "fsa9480: open\n");
	if (!opened) {
		printk(KERN_INFO "fsa9480: prevent collapse on idle\n");

//		opened = 1;
		rc = 0;
	}
	up(&fsa_sem);
	return rc;
}

static int fsa9480_release(struct inode *ip, struct file *fp)
{
	int rc = -EBADF;
	printk(KERN_INFO "fsa9480: release\n");
	down(&fsa_sem);
	if (opened) {
		printk(KERN_INFO "fsa9480: release clocks\n");
             // PWR_DOWN Ã³¸® fsa9480_i2c_power_down();
		printk(KERN_INFO "fsa9480: allow collapse on idle\n");

		rc = pclk_set = opened = 0;
	}
	up(&fsa_sem);
	return rc;
}

#if ALLOW_USPACE_RW
#define COPY_FROM_USER(size) ({                                         \
        if (copy_from_user(rwbuf, argp, size)) rc = -EFAULT;            \
        !rc; })
#endif

static long fsa9480_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
#if ALLOW_USPACE_RW
	void __user *argp = (void __user *)arg;
#endif
	int rc = 0;

	down(&fsa_sem); 

	switch(cmd) {
    default:
        printk(KERN_INFO "fsa9480: unknown ioctl %d\n", cmd);
        break;
	}

	up(&fsa_sem);

	return rc;
}

unsigned char fsa9480_i2c_read_reg(unsigned char addr, unsigned char *reg_value)
{
        unsigned char data;

	I2C_READ(addr, &data);

//	printk("[FSA9480] I2C read 0x%x Reg Data : 0x%x\n",addr, data);
	*reg_value = data;
	return 0;
}

unsigned char fsa9480_i2c_write_reg(unsigned char addr, unsigned char reg_value)
{
	I2C_WRITE(addr, reg_value);
	return 0;
}

int fsa9480_i2c_sensor_init(void)
{
		printk("FSA9480 : %s is done.\n", __func__);
	return 0;
}


#undef I2C_WRITE
#undef I2C_READ

static int fsa9480_init_client(struct i2c_client *client)
{
	/* Initialize the fsa9480 Chip */
	init_waitqueue_head(&g_data_ready_wait_queue);
	return 0;
}

static struct file_operations fsa9480_fops = {
        .owner 	= THIS_MODULE,
        .open 	= fsa9480_open,
        .release = fsa9480_release,
        .unlocked_ioctl = fsa9480_ioctl,
};

static struct miscdevice fsa9480_device = {
        .minor 	= MISC_DYNAMIC_MINOR,
        .name 	= "fsa9480",
        .fops 	= &fsa9480_fops,
};

extern fsa_init_done;
int fsa_suspended = 0;

#define GPIO_MICROUSB_DET		49
#define MICROUSB_DET			MSM_GPIO_TO_INT(GPIO_MICROUSB_DET)

void AutoSetting(void);
extern unsigned char ftm_sleep;
extern int cable_status_update(int status);

static irqreturn_t usb_switch_interrupt_handler(int irq, void *data)
{
	int ret;
	disable_irq(irq);

	if (ftm_sleep == 1)
		AutoSetting();

	enable_irq(irq);
	return IRQ_HANDLED;
}


static int fsa9480_probe(struct i2c_client *client)
{
	struct fsa9480_data *mt;
	int err = 0;
	unsigned char cont_reg;
	printk(KERN_INFO "fsa9480: probe\n");
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		goto exit_check_functionality_failed;		
	
	if(!(mt = kzalloc( sizeof(struct fsa9480_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, mt);
	fsa9480_init_client(client);
	pclient = client;
	fsa9480_chip_init();
	
	/* Register a misc device */
	err = misc_register(&fsa9480_device);
	if(err) {
		printk(KERN_ERR "fsa9480_probe: misc_register failed \n");
		goto exit_misc_device_register_failed;
	}
	fsa9480_i2c_write_reg(0x02, 0x1E); // FSA9480 initilaization
	fsa9480_i2c_read_reg(0x02, &cont_reg); // FSA9480 initilaization check
	printk("fsa9480 : Initial control reg 0x02 : 0x%x\n", cont_reg);
	fsa_init_done = 1;

#if 1
	int retval;
	//ret = gpio_configure(GPIO_MICROUSB_DET, GPIOF_INPUT | IRQF_TRIGGER_LOW);
	
	retval = gpio_request(49 , "micro usb switch");
	if (retval  < 0) {
		printk(KERN_ERR "<!> gpio_request failed!!!\n");
	}

	retval = gpio_direction_input(49);
	if (retval < 0) {
		printk(KERN_ERR "<!> gpio_direction_input failed!!!\n");
	}
	
	if(request_irq(MICROUSB_DET, usb_switch_interrupt_handler, IRQF_TRIGGER_FALLING, "MICROUSB", NULL)) {
		free_irq(MICROUSB_DET, NULL);
		printk("[error] usb_switch_interrupt_handler can't register the handler! and passing....\n");
	}

	retval = set_irq_wake(MICROUSB_DET, 1);
#endif

	return 0;
	
exit_misc_device_register_failed:
exit_alloc_data_failed:
exit_check_functionality_failed:
	
	return err;
}

	
static int fsa9480_remove(struct i2c_client *client)
{
	struct fsa9480_data *mt = i2c_get_clientdata(client);
	free_irq(client->irq, mt);
#ifdef CONFIG_ANDROID_POWER
	android_unregister_early_suspend(&mt->early_suspend);
#endif
	i2c_detach_client(client);
	pclient = NULL;
	misc_deregister(&fsa9480_device);
	kfree(mt);
	return 0;
}

static int fsa9480_suspend(struct i2c_client *client)
{
	fsa_suspended = 1;
	return 0;
}

static int fsa9480_resume(struct i2c_client *client)
{
	fsa_suspended = 0;
	return 0;
}

static const struct i2c_device_id fsa9480_id[] = {
	{ "fsa9480", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, fsa9480_id);

static struct i2c_driver fsa9480_driver = {
	.id_table	= fsa9480_id,
	.probe = fsa9480_probe,
	.remove = fsa9480_remove,
	.suspend = fsa9480_suspend,
	.resume = fsa9480_resume,
	.driver = {		
		.name   = "fsa9480",
	},
};

static int __init fsa9480_init(void)
{
	printk(KERN_INFO "fsa9480_init\n");

	return i2c_add_driver(&fsa9480_driver);
}

static void __exit fsa9480_exit(void)
{
	i2c_del_driver(&fsa9480_driver);
}

/*================================================
	When send sleep cmd via UART, UART3_RXD, TXD port make disable
================================================*/

#define REGISTER_MANUALSW1	0x13                       // Read/Write
#define REGISTER_MANUALSW2	0x14                       // Read/Write

// Hidden Register
#define HIDDEN_REGISTER_MANUAL_OVERRDES1	0x1B  // Read/Write

void ManualSetting(unsigned char valManualSw1, unsigned char valManualSw2)
{
	unsigned char cont_reg, man_sw1, man_sw2;

	fsa9480_i2c_write_reg(REGISTER_MANUALSW1, valManualSw1);
	mdelay(20);
	fsa9480_i2c_write_reg(REGISTER_MANUALSW2, valManualSw2);
	mdelay(20);

	//when detached the cable, Control register automatically be restored.
	fsa9480_i2c_read_reg(0x02, &cont_reg);
	mdelay(20);
	fsa9480_i2c_write_reg(0x02, 0x1A);

	// Read current setting value , manual sw1, manual sw2, control register.
	fsa9480_i2c_read_reg(REGISTER_MANUALSW1, &man_sw1);
	mdelay(20);
	fsa9480_i2c_read_reg(REGISTER_MANUALSW2, &man_sw2);
	mdelay(20);
	fsa9480_i2c_read_reg(0x02, &cont_reg);
}

void AutoSetting(void)
{
	unsigned char cont_reg=0xff;
	unsigned char hidden_reg;
	
	fsa9480_i2c_write_reg(0x02, 0x1E);
	fsa9480_i2c_read_reg(0x03, &hidden_reg);
}

void Make_RXD_LOW(void)
{
	unsigned char hidden_reg;
	
	fsa9480_i2c_write_reg(HIDDEN_REGISTER_MANUAL_OVERRDES1, 0x0a); 
	mdelay(20);
	fsa9480_i2c_read_reg(HIDDEN_REGISTER_MANUAL_OVERRDES1, &hidden_reg);
	ManualSetting(0x00, 0x00);
}
EXPORT_SYMBOL(ManualSetting);
EXPORT_SYMBOL(AutoSetting);
EXPORT_SYMBOL(Make_RXD_LOW);


EXPORT_SYMBOL(fsa9480_i2c_read_reg);

module_init(fsa9480_init);
module_exit(fsa9480_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("FSA9480 Driver");
MODULE_LICENSE("GPL");


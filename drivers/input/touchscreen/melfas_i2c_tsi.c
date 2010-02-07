#if defined(CONFIG_SAMSUNG_CAPELA)
/* drivers/input/keyboard/melfas_i2c_mth_st909.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/miscdevice.h>
#include <mach/vreg.h>
#include "melfas_i2c_tsi.h"

#define INPUT_TOUCH_DEBUG

#ifdef INPUT_TOUCH_DEBUG 
#define DPRINTK(X...) do { printk("%s(): ", __FUNCTION__); printk(X); } while(0)
#else
#define DPRINTK(x...)        /* !!!! */
#endif

static struct vreg *vreg_touch;	
static struct workqueue_struct *melfas_wq;

struct melfas_ts_pdev {
	unsigned short force[3];
	unsigned short *forces[2];
	struct i2c_client_address_data addr_data;
	int irq;
};

struct melfas_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
};

static int ts_irq_num;
extern int bridge_on;

static void melfas_tsp_init(void)
{
	int rc;
	printk(KERN_INFO "melfas_tsp_init because i2c_transfer failed\n");
	
	// TOUCH OFF
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SCL_F, 0);
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SDA_F, 0);
	gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_INT, 0);

	gpio_direction_output(TOUCH_EN, 0);  // TOUCH EN

	rc = vreg_disable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not disable gp3\n");
	msleep(100);

	// TOUCH ON
	rc = vreg_enable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not enable gp3\n");
	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN
	
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SCL_F, 1);
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SDA_F, 1);
	gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_INT, 1);
	
	msleep(200);
}

static void melfas_ts_work_func(struct work_struct *work)
{
	int ret;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf1[7];
	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x10;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf1);
	msg[1].buf = buf1;

//    printk("melfas_ts_work_func: ");  

	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		printk(KERN_ERR "melfas_ts_work_func: i2c_transfer failed\n");
	} else {
	   if(bridge_on)  // lcd on status
	   {
			int x = buf1[2] | (uint16_t)(buf1[1] & 0x03) << 8; 
			int y = buf1[4] | (uint16_t)(buf1[3] & 0x0f) << 8; 
			int z = buf1[5];
			int finger = buf1[0] & 0x01;
			int width = buf1[6];

//	        printk("x%4d, y%4d, z%4d, finger%4d, width%4d\n",x, y, z, buf1[0],width); 

			if((buf1[0] == 0)||(buf1[0] == 1)) // Only Finger Touch, Palm Touch Disable
			{
				//if(!(buf1[0]))
				//			 printk("x%4d, y%4d, z%4d, finger%4d, width%4d\n",x, y, z, finger,width); 

				//printk("x%4d, y%4d, z%4d, finger%4d, width%4d\n",x, y, z, finger,width);
				if ( x < 320 && x > 0 && y < 480 && y > 0 ) {
					input_report_abs(ts->input_dev, ABS_X, x);
					input_report_abs(ts->input_dev, ABS_Y, y);
					input_report_abs(ts->input_dev, ABS_PRESSURE, z);
					input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
					input_report_key(ts->input_dev, BTN_TOUCH, finger);
					input_sync(ts->input_dev);
				}
#if defined(CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_DEVICE_CHECK)
//				if ( x < 50 && y < 50)
//					printk("x%4d, y%4d, z%4d, finger%4d, width%4d\n",x, y, z, finger,width);
					
				if (finger == 0)
					printk("T%3d%3d\n", x, y);
#endif
				input_report_abs(ts->input_dev, ABS_PRESSURE, z);
				input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
				input_report_key(ts->input_dev, BTN_TOUCH, finger);
				input_sync(ts->input_dev);
			}

//			printk("[KTH] buf1[0]=%x \n", buf1[0]);
			if (buf1[0] == 0xed || x == 941 || y == 2989 )
			{
				melfas_tsp_init();
			}
			
			if(!(gpio_get_value(TOUCH_EN)))
			{
				printk(KERN_ERR "melfas_ts_work_func: TOUCH_EN is Low\n");
				gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN
				msleep(200);
			}
	    }
	}


	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart melfas_ts_timer_func(struct hrtimer *timer)
{
	struct melfas_ts_data *ts = container_of(timer, struct melfas_ts_data, timer);
	/* printk("melfas_ts_timer_func\n"); */

	queue_work(melfas_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	struct melfas_ts_data *ts = dev_id;

	/* printk("melfas_ts_irq_handler\n"); */
	disable_irq(ts->client->irq);
	queue_work(melfas_wq, &ts->work);
	return IRQ_HANDLED;
}


static int __init melfas_ts_probe(struct i2c_client *client)
{
	struct melfas_ts_data *ts;

	int ret = 0;
	uint16_t max_x, max_y;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, melfas_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	
	ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "melfas_ts_probe: Firmware Version %x\n", ret);

	ret = i2c_smbus_read_byte_data(ts->client, 0x21);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "melfas_ts_probe: Module Revision %x\n", ret);

	ret = i2c_smbus_read_word_data(ts->client, 0x08);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	ts->max[0] = max_x = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);
	ret = i2c_smbus_read_word_data(ts->client, 0x0a);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	ts->max[1] = max_y = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "melfas_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "melfas-tsi-touchscreen";

	ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
	}
	ts->input_dev->id.version  = ret; 

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

    printk("melfas_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	input_set_abs_params(ts->input_dev, ABS_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

    ts_irq_num = client->irq;

	if (client->irq) {
		ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);

		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = melfas_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	printk(KERN_INFO "melfas_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret,rc;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq); 

	gpio_direction_output(TOUCH_EN, 0);  // TOUCH EN
	msleep(20);
    
	rc = vreg_disable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not disable gp3\n");
	msleep(200);

    printk(KERN_INFO "Melfas Touchscreen : suspend.\n");
	
	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	int rc;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	rc = vreg_enable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not enable gp3\n");
	msleep(20);

	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN
	msleep(200);
	
	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

    printk(KERN_INFO "Melfas Touchscreen : resume.\n");
	
	return 0;
}


static const struct i2c_device_id melfas_ts_id[] = {
	{ "melfas-tsi-ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, melfas_ts_id);

static struct i2c_driver melfas_ts_driver = {
	.id_table	= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= melfas_ts_remove,
	.suspend	= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
	.driver = {
		.name	= "melfas-tsi-ts",
	},
};

static int __devinit melfas_ts_init(void)
{
	melfas_wq = create_singlethread_workqueue("melfas_wq");
	if (!melfas_wq)
		return -ENOMEM;

	vreg_touch = vreg_get(0, "gp3");
	if (IS_ERR(vreg_touch))
		return PTR_ERR(vreg_touch);

	gpio_direction_output(TOUCH_EN, 0);  // TOUCH EN

	msleep(100);

	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN

	msleep(200);

	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
	if (melfas_wq)
		destroy_workqueue(melfas_wq);
}
module_init(melfas_ts_init);
module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");
#endif /* CONFIG_SAMSUNG_CAPELA */

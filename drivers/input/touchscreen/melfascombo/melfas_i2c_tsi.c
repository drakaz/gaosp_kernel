
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
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#include <mach/vreg.h>
#include "melfas_i2c_tsi.h"
#include "melfas_download.h"
#include "melfas_downloadmcs5000.h"


#define BOTH_SUPPORT_MCS5000_MCS6000
#define ADDED_DEBUG_TOUCH

#define INPUT_TOUCH_DEBUG

#ifdef INPUT_TOUCH_DEBUG 
#define DPRINTK(X...) do { printk("%s(): ", __FUNCTION__); printk(X); } while(0)
#else
#define DPRINTK(x...)        /* !!!! */
#endif

#define MAX_ABS_X  383
#define MAX_ABS_Y  511


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
#ifdef CONFIG_ANDROID_POWER
	struct android_early_suspend early_suspend;
#endif
};

#ifdef CONFIG_ANDROID_POWER
static void melfas_ts_early_suspend(struct android_early_suspend *h);
static void melfas_ts_late_resume(struct android_early_suspend *h);
#endif


//--------------------------------------------
//
//   Write ISP Mode entering signal
//
//--------------------------------------------
static void mcsdl_write_download_mode_signal(void)
{
    int    i;

    UINT8 enter_code[14] = { 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1 };

    //---------------------------
    // ISP mode signal 0
    //---------------------------

      printk("[MELFAS]melfas_ts_write_download_mode_signal\n");
      printk("[MELFAS]melfas_ts_write_download_mode_signal\n");
      printk("[MELFAS]melfas_ts_write_download_mode_signal\n");
      printk("[MELFAS]melfas_ts_write_download_mode_signal\n");
      printk("[MELFAS]melfas_ts_write_download_mode_signal\n");
    
      //---------------------------
      // ISP mode signal 0
      //---------------------------
    
      for(i=0; i<14; i++){
    
          if( enter_code[i] ) {
                     
              gpio_set_value(TOUCH_INT, 1);
    
          }else{
                 
              gpio_set_value(TOUCH_INT, 0);
          }
    
          gpio_set_value(TOUCH_I2C_SCL_F, 1);
      
          udelay(15);
          gpio_set_value(TOUCH_I2C_SCL_F, 0);
    
          gpio_set_value(TOUCH_INT, 0);
    
          msleep(10);
     }
    
      gpio_set_value(TOUCH_I2C_SCL_F, 1);
    
      msleep(1);
    
     
      gpio_set_value(TOUCH_INT, 1);
}


//--------------------------------------------




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
//	int i;
	int ret, count;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf1[7];
	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);
#if 0
	int x, y, z, rc;
	int finger, width;
#endif
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

#if 1
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		printk(KERN_ERR "melfas_ts_work_func: i2c_transfer failed\n");
		melfas_tsp_init();
	} else {

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

				if (x)
					input_report_abs(ts->input_dev, ABS_X, x);
				if (y)
					input_report_abs(ts->input_dev, ABS_Y, y);
				//			printk("[KTH] X:%d Y:%d \n", x, y);
				input_report_abs(ts->input_dev, ABS_PRESSURE, z);
				input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
				input_report_key(ts->input_dev, BTN_TOUCH, finger);
				input_sync(ts->input_dev);
			}

//			printk("[KTH] buf1[0]=%x \n", buf1[0]);
			if (buf1[0] == 0xed || x == 941 || y == 2989 )
			{
				printk("[KTH] touch_ESD test\n");

				melfas_tsp_init();

			}
			if(!(gpio_get_value(TOUCH_EN)))
			{
				printk(KERN_ERR "melfas_ts_work_func: TOUCH_EN is Low\n");
				gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN
				msleep(200);
			}
	}

#if 0 // KTHUR
	char version_id[4];
	ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
	}
	sprintf(version_id, "%x", ret);
	printk(KERN_INFO "melfas_ts_probe: Firmware Version %x %s %x\n", ret, version_id, version_id);
	ts->input_dev input_id->vendor  = ret; //version_id ;
	printk(KERN_INFO "melfas_ts_probe: Firmware Version %x\n", ret);
#else
#if 0
	char version_id[4];
	ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
	}
	sprintf(version_id, "%x", ret);
	ts->input_dev->id.version  = ret; //version_id;
#endif
#endif

#else
	count = 0;
	do {
		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret < 0) 
			printk(KERN_ERR "melfas_ts_work_func: i2c_transfer failed\n");

		count++;
		if (count > 5)
		{
			printk(KERN_ERR"melfas_ts_work_func: i2c FAIL %d times TOUCH DEVICE RESET", count);
		
			gpio_set_value(TOUCH_EN, 0);  // TOUCH EN
			gpio_set_value(TOUCH_INT, 0);  // TOUCH INT 
			
			msleep(200);
			
			gpio_set_value(TOUCH_EN, 1);  // TOUCH EN
			gpio_set_value(TOUCH_INT, 1);  // TOUCH INT 

		}

	} while (ret < 0);

	x = buf1[2] | (uint16_t)(buf1[1] & 0x03) << 8; 
	y = buf1[4] | (uint16_t)(buf1[3] & 0x0f) << 8; 
	z = buf1[5];
	finger = buf1[0] & 0x01;
	width = buf1[6];

	//			 printk("x%4d, y%4d, z%4d, finger%4d, width%4d\n",x, y, z, finger,width); 

	if (x)
		input_report_abs(ts->input_dev, ABS_X, x);
	if (y)
		input_report_abs(ts->input_dev, ABS_Y, y);
	printk("[KTH] X:%d Y:%d \n", x, y);
	input_report_abs(ts->input_dev, ABS_PRESSURE, z);
	input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
	input_report_key(ts->input_dev, BTN_TOUCH, finger);
	input_sync(ts->input_dev);
#endif
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
#if 1
//#ifdef CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE
       udelay(15);
	
	disable_irq(ts->client->irq);					// Disable Baseband touch interrupt ISR.
      printk(KERN_ERR "melfas_ts_probe\n");
      
      printk(KERN_ERR "melfas_ts_download_firmware\n");
      printk(KERN_ERR "melfas_ts_download_firmware\n");
      printk(KERN_ERR "melfas_ts_download_firmware\n");
      printk(KERN_ERR "melfas_ts_download_firmware\n");

      ret = mcsdl_download_binary_data(); //For MCS6000

    printk( "[MELFAS]mcsdl_download_binary_data after\n");
    printk( "[MELFAS]BOTH_SUPPORT_MCS5000_MCS6000 after\n");


//#ifdef BOTH_SUPPORT_MCS5000_MCS6000
//      if(ret==MCSDL_RET_ENTER_DOWNLOAD_MODE_FAILED)
      {

        
            printk( "[MELFAS]melfas_ts_download_firmware5000 starting\n");
            ret = melfas_ts_download_firmware5000( ); //For MCS5000

      }
//#endif /*BOTH_SUPPORT_MCS5000_MCS6000*/


    enable_irq(ts->client->irq);// Enable Baseband touch interrupt ISR.

#ifdef  ADDED_DEBUG_TOUCH
    //ENABLE INT PIN
    gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA),GPIO_ENABLE);
    gpio_set_value(TOUCH_INT, 1);
#endif 

	msleep(200);
//#endif
#else



    disable_irq(ts->client->irq);                   // Disable Baseband touch interrupt ISR.
    printk(KERN_ERR "melfas_ts_probe\n");
    printk(KERN_ERR "melfas_ts_download_firmware\n");
    printk(KERN_ERR "melfas_ts_download_firmware\n");
    printk(KERN_ERR "melfas_ts_download_firmware\n");
    printk(KERN_ERR "melfas_ts_download_firmware\n");
    ret = mcsdl_download_binary_data();
    enable_irq(ts->client->irq);// Enable Baseband touch interrupt ISR.


   




#endif
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
#if 1 // KTHUR
	char version_id[4];
	ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
	}
	ts->input_dev->id.version  = ret; //version_id;
#endif
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

	if (client->irq) {
#if 1  // Because of Lock up
		ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);
#else
		ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_LOW, client->name, ts);
#endif
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

#ifdef CONFIG_ANDROID_POWER
	ts->early_suspend.level = ANDROID_EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	android_register_early_suspend(&ts->early_suspend);
	if (android_power_is_driver_suspended())
		melfas_ts_early_suspend(&ts->early_suspend);
#endif

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
#ifdef CONFIG_ANDROID_POWER
	android_unregister_early_suspend(&ts->early_suspend);
#endif
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
#if 1 // save wake up time 
	gpio_direction_output(TOUCH_EN, 0);  // TOUCH EN
	msleep(20);
    
	rc = vreg_disable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not disable gp3\n");
	msleep(200);
#endif
    printk(KERN_INFO "Melfas Touchscreen : suspend.\n");
	
	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	int rc;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

#if 1 // save wake up time
	rc = vreg_enable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not enable gp3\n");
	msleep(20);

	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN
	msleep(200);
#endif
	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

    printk(KERN_INFO "Melfas Touchscreen : resume.\n");
	
	return 0;
}

#ifdef CONFIG_ANDROID_POWER
static void melfas_ts_early_suspend(struct android_early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct android_early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ "melfas-tsi-ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, melfas_ts_id);

static struct i2c_driver melfas_ts_driver = {
	.id_table	= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= melfas_ts_remove,
#ifndef CONFIG_ANDROID_POWER
	.suspend	= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
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

#if 1
	gpio_direction_output(TOUCH_EN, 0);  // TOUCH EN

	msleep(100);

	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN

	msleep(200);

#else
	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN
	msleep(300);
#endif

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


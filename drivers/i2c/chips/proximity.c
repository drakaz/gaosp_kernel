/* reference : drivers/input/misc/prox_sharp.c
 *
 * Sharp proximity driver
 *
 * Copyright (C) 2008 LGE, Inc.
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
#include <asm/uaccess.h>//add to test //diyu@lge.com
#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>
#include <asm/io.h>
#include <linux/delay.h>//LGE_CHANGE [diyu@lge.com]  //diyu@lge.com
#include <asm/gpio.h>//LGE_CHANGE [diyu@lge.com]  //diyu@lge.com
#include <mach/vreg.h> //LGE_CHANGE [diyu@lge.com] To set vreg
#include <linux/wakelock.h>

#define OBJECT_DETECTED		 1
#define	OBJECT_NOT_DETECTED	0

#define MISC_DEV_NAME		"proximity" //"proxi-sensor"
#define USE_IRQ				1
#define GPIO_PROX_IRQ 		57 /*PROX_OUT*/

#define PROXIMITY_DEBUG 0
#if PROXIMITY_DEBUG
#define PDBG(fmt, args...) printk(fmt, ##args)
#else
#define PDBG(fmt, args...) do {} while (0)
#endif /* PROXIMITY_DEBUG */

static void *_dev = NULL;

struct psensor_dev {
	//struct sensors_dev *sensor;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct list_head plist;
	volatile int status;
	volatile int ref_count;
	int use_irq;
	//wait_queue_head_t waitq;
	struct work_struct work;
	int irq;
};

static struct workqueue_struct *proximity_wq;
static atomic_t s_prox_sensor = ATOMIC_INIT(0);
//static int prox_sensor_status = -1;
static struct psensor_dev sharp_psensor;
static int irq_set = 0; 
static int proximity_near =-1; 


int is_proxi_open(void)
{
	PDBG("\n%s \n", __FUNCTION__);
	return proximity_near;
}

EXPORT_SYMBOL(is_proxi_open);


static int gp2ap002_open(struct inode* inode, struct file* filp)
{
	sharp_psensor.ref_count++;

	return 0;
}

static int gp2ap002_release(struct inode* inode, struct file* filp)
{
	if (sharp_psensor.ref_count == 0) return 0;

	sharp_psensor.ref_count--;

	return 0;
}
#if 0  /*Not used*/
static ssize_t gp2ap002_read(struct file *filp, char *buf, size_t count, loff_t *ofs)
{
	int size = sizeof(struct input_event);
	struct input_event data;

	do_gettimeofday(&data.time);
	data.type = EV_ABS;
	data.code = ABS_MISC;
	data.value = sharp_psensor.status;

	if (copy_to_user(buf, &data, size)) 
		return -EFAULT;
	else
		return size;
}
#endif
static int gp2ap002_ioctl(struct inode *inode, struct file *filp, 
		unsigned int cmd, unsigned long arg)
{
	struct psensor_dev *dev = (struct psensor_dev *)_dev;
	PDBG("\n\nCheck point : %s-1\n", __FUNCTION__);
	
	if (NULL == dev)
		return -ENODEV;

	return 0;
}

/*
static unsigned int gp2ap002_poll(struct file *filp, struct poll_table_struct *wait)
{
	poll_wait(filp, &sharp_psensor.waitq, wait);

	return POLLIN;
}
*/
/*
static struct file_operations gp2ap002_fops = {
	.owner 	= THIS_MODULE,
	.open  	= gp2ap002_open,
	.read	= gp2ap002_read,
	//.poll	= gp2ap002_poll,
	.release= gp2ap002_release,
};

*/


/* use miscdevice for ioctls */
static struct file_operations gp2ap002_fops = {
	.owner   = THIS_MODULE,
	.open    = gp2ap002_open,
	.release = gp2ap002_release,
	.ioctl   = gp2ap002_ioctl,
};

static struct miscdevice gp2ap002_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = MISC_DEV_NAME,
	.fops  = &gp2ap002_fops,
};

/* 
 * Sharp Proximity Sensor Interface 
 * */
#define PROX_REG	0
#define GAIN_REG	1
#define HYS_REG		2
	#define HYS_MODE_A		0xC2
	#define HYS_MODE_B1_VO0	0x40
	#define HYS_MODE_B1_VO1	0x20
	#define	HYS_MODE_B2_VO0	0x20
	#define HYS_MODE_B2_VO1	0x00
#define CYCLE_REG	3
#define	OPMOD_REG	4
	#define OPMODE_POWER_ON			(1 << 0)
	#define	OPMODE_INTR_MODE		(1 << 1)
	#define OPMODE_OPERTATING_MODE  0x01
		#define OPMODE_NORMAL_MODE  0x01
		#define OPMODE_INTERRUPT_MODE  0x03
	#define OPMODE_SHUTDONW_MODE  0x00
	
	
#define	CON_REG		6
	#define	CON_VOUT_ENABLE		0x00
	#define CON_VOUT_DISABLE	0x18

#define PROX_I2C_INT_NO_CLEAR	0
//#define PROX_I2C_INT_NO_CLEAR	0

#define	PROX_I2C_INT_CLEAR		0x80

//int prox_i2c_write( u8 addr, u8 value, u8 intr_clr)

int prox_i2c_write(struct i2c_client *client, u8 addr, u8 value, u8 intr_clr)
{
	sharp_psensor.client = client;
	return i2c_smbus_write_byte_data(client, addr | intr_clr, value);
	//	return i2c_smbus_write_byte_data(sharp_psensor.client, addr | intr_clr, value);
	
}

int prox_i2c_read(struct i2c_client *client, u8 addr, u8 intr_clr)
{
	int ret;

	client->flags |= I2C_M_RD; //I2C_M_RDW 
	ret = i2c_smbus_read_byte_data(client, addr | intr_clr);
	client->flags &=~I2C_M_RD; //I2C_M_RDW 

	return ret;
}

static int prox_vreg_set(int onoff)
{
	struct vreg *vreg_proximity;
	int rc = -1;

	//START: proximity sensor  2.8v setting
	vreg_proximity = vreg_get(0, "gp6");
	if (onoff)
	{
		vreg_enable(vreg_proximity);
		rc = vreg_set_level(vreg_proximity, 2800);
	}
	else
	{
		rc = vreg_disable(vreg_proximity);
	}
	if (rc != 0) {
		printk("diyu vreg_proximity failed(err:%d)\n", rc);
		return -1;
	}
//END: proximity sensor  2.8v setting
	return rc;
}

static int sharp_psensor_init(struct i2c_client *client)
{
	//struct i2c_client *client = sharp_psensor.client;
	int ret;
	
	PDBG("diyu %s\n","sharp_psensor_init");

#if 1
		ret = prox_i2c_write(client, GAIN_REG, 0x08, PROX_I2C_INT_NO_CLEAR);
		ret = prox_i2c_write(client, HYS_REG, HYS_MODE_A,  PROX_I2C_INT_NO_CLEAR);
		ret = prox_i2c_write(client, CYCLE_REG, 0x04, PROX_I2C_INT_NO_CLEAR);
		ret = prox_i2c_write(client, OPMOD_REG, OPMODE_NORMAL_MODE, PROX_I2C_INT_NO_CLEAR);
//		ret = prox_i2c_write(client, OPMOD_REG, OPMODE_INTERRUPT_MODE, PROX_I2C_INT_NO_CLEAR);
#else
//	prox_i2c_write(CON_REG, CON_VOUT_DISABLE, PROX_I2C_INT_NO_CLEAR);
	prox_i2c_write(GAIN_REG, 0x08, PROX_I2C_INT_NO_CLEAR);
	//prox_i2c_write(HYS_REG, HYS_MODE_A,  PROX_I2C_INT_NO_CLEAR);
	prox_i2c_write(HYS_REG, 0x40,  PROX_I2C_INT_NO_CLEAR);
	prox_i2c_write(CYCLE_REG, 0x04, PROX_I2C_INT_NO_CLEAR);
	//prox_i2c_write(HYS_REG, HYS_MODE_B1_VO0,  PROX_I2C_INT_NO_CLEAR);
	prox_i2c_write(OPMOD_REG, 0x03, PROX_I2C_INT_NO_CLEAR);
#endif
	//sharp_psensor_on();
	prox_i2c_write(client, CON_REG, CON_VOUT_ENABLE, PROX_I2C_INT_NO_CLEAR);

 	return ret;
}

static void gp2ap002_work_func(struct work_struct *work)
{
	struct psensor_dev *dev = container_of(work, struct psensor_dev, work);

	int do_report = 1, report_value;
	PDBG("%s()\n",__FUNCTION__);

	if(gpio_get_value(GPIO_PROX_IRQ) == 1){
		report_value = 1;
		if (do_report) {
			proximity_near = 1; /*NEAR ?*/
			input_report_abs(dev->input_dev, ABS_DISTANCE, report_value );
			input_sync(dev->input_dev);
		}
	}else{
		report_value = 0;
		if (do_report) {
			proximity_near = 0; /*FAR AWAY ?*/
			input_report_abs(dev->input_dev, ABS_DISTANCE, report_value );
			input_sync(dev->input_dev);
		}
	}
}


void sharp_psensor_on(void)
{
	prox_i2c_write(sharp_psensor.client, OPMOD_REG, OPMODE_POWER_ON, PROX_I2C_INT_NO_CLEAR);
	sharp_psensor_init(sharp_psensor.client);
	printk("diyu %s\n","sharp_psensor_on" );
}

void sharp_psensor_off(void)
{
	prox_i2c_write(sharp_psensor.client, OPMOD_REG, OPMODE_INTR_MODE, PROX_I2C_INT_NO_CLEAR);	
	prox_i2c_write(sharp_psensor.client, OPMOD_REG, 0, PROX_I2C_INT_NO_CLEAR);	
	//prox_i2c_write(sharp_psensor.client, OPMOD_REG, 0, PROX_I2C_INT_NO_CLEAR);	
	
	printk("diyu %s\n","sharp_psensor_off" );
}

int sharp_psensor_read(void)
{
	return prox_i2c_read(sharp_psensor.client, PROX_REG, PROX_I2C_INT_CLEAR);
	
	printk("diyu %s\n","sharp_psensor_read" );

}

/*
 *	system filesystem 
 **/
#if 0 /*Not use*//*CONFIG_SYSFS*/
static ssize_t sharp_gp2ap002_switch(struct device *dev, const char *buf, size_t count)
{
	char sen_switch[8];

	sscanf(buf, "%4s", sen_switch);
	printk(KERN_INFO"%s\n", sen_switch);

	if (strncmp(sen_switch, "on", 2) == 0) {
		PDBG(KERN_INFO"sharp_psensor_on\n");
		sharp_psensor_on();
	}
	else if (strncmp(sen_switch, "off", 3) == 0) {
		PDBG(KERN_INFO"sharp_psensor_off\n");
		sharp_psensor_off();
	}

	return count;
}

static ssize_t sharp_gp2ap002_interval(struct device *dev, const char *buf, size_t count)
{
	char sen_switch[8];

	sscanf(buf, "%4s", sen_switch);
	printk(KERN_INFO"%s\n", sen_switch);

	if (strncmp(sen_switch, "read", 1) == 0) 
		PDBG("[0x00] = %x\n", sharp_psensor_read());
	else if (strncmp(sen_switch, "high", 1) == 0)
		prox_i2c_write(sharp_psensor.client, CON_REG, CON_VOUT_DISABLE, PROX_I2C_INT_NO_CLEAR);
	else if (strncmp(sen_switch, "low", 1) == 0)
		prox_i2c_write(sharp_psensor.client, CON_REG, CON_VOUT_ENABLE, PROX_I2C_INT_NO_CLEAR);



	return count;
}

static DEVICE_ATTR(switch, (S_IRUSR|S_IWUSR), NULL, sharp_gp2ap002_switch);
static DEVICE_ATTR(interval, (S_IRUSR|S_IWUSR), NULL, sharp_gp2ap002_interval);

static struct attribute* psensor_attr[] = {
	&dev_attr_switch,
	&dev_attr_interval,
	NULL
};

static struct attribute_group psensor_attr_group = {
	.attrs = psensor_attr
};
#endif

static ssize_t sharp_gp2ap002_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int prox_gpio_state;
	prox_gpio_state =  proximity_near;
//	prox_gpio_state = (float)gpio_get_value(GPIO_PROX_IRQ);
	
	return sprintf(buf, "%d\n", prox_gpio_state );
}

static ssize_t sharp_gp2ap002_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", s_prox_sensor); //irq_set);
}

static ssize_t sharp_gp2ap002_enable_store(
		struct device *dev, struct device_attribute *attr, 
		const char *buf, size_t size)
{
	struct psensor_dev *pdev = _dev;
	int value;
	int ret=0;
	sscanf(buf, "%d", &value);

	atomic_set(&s_prox_sensor, value);

	/*  value == 0  : stop hall ic_irq_wake	 
	 *    value ==1  : start hall ic_irq_wake
	       value ==   :  */

	printk(KERN_ERR "\n[PROX] %s(old: %d, new: %d)\n", __FUNCTION__, irq_set, value);
    if(value == irq_set)
      return size; /* nothing to do */

	if(value ==1 ) {
		irq_set = 1;
		prox_vreg_set(1);
		udelay(100);
		ret = sharp_psensor_init(pdev->client);
		PDBG("set_irq_wake on \n");
	}else if(value == 0){
		irq_set = 0;	
		prox_vreg_set(0);
		PDBG("set_irq_wake off \n"); 
		
	}

	if(irq_set == 1){
		enable_irq(gpio_to_irq(GPIO_PROX_IRQ));
		set_irq_wake(gpio_to_irq(GPIO_PROX_IRQ), irq_set);
	}else if(irq_set == 0){
		disable_irq(gpio_to_irq(GPIO_PROX_IRQ));
		set_irq_wake(gpio_to_irq(GPIO_PROX_IRQ), irq_set);
	}
		
	
	
	return size;
}

static DEVICE_ATTR(show, S_IRUGO | S_IWUSR, sharp_gp2ap002_show, NULL);

static DEVICE_ATTR(enable, S_IRUGO | S_IWUGO, sharp_gp2ap002_enable_show, sharp_gp2ap002_enable_store);

/*
 * interrupt service routine
 */
static int gp2ap002_interrupt(int irq, void *dev_id)
{
	struct psensor_dev *pdev = dev_id;
	
	if(irq_set == 1 ){
		PDBG("diyu/yong 11p = %d\n", sharp_psensor.status);
		//disable_irq(pdev->client->irq);
		sharp_psensor.status = !sharp_psensor.status; 
		PDBG("diyu  proximity-interrupt  = %d\n", sharp_psensor.status);
		queue_work(proximity_wq, &pdev->work);
		//enable_irq(pdev->client->irq);
	}else{
		printk("diyu/yong p = %d disable \n", sharp_psensor.status);
		//disable_irq(pdev->client->irq);
	}

	return IRQ_HANDLED;
}

static int gp2ap002_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret=0;
	struct psensor_dev *pdev = _dev;
	
	/* future capability*/
	PDBG("diyu/yong [%s] \n", __FUNCTION__);
	ret = cancel_work_sync(&pdev->work);
	if (ret != 0) {
		printk("diyu vreg_proximity failed\n");
		return -1;
	}
	return 0;
}

static int gp2ap002_resume(struct i2c_client *client)
{
	//int ret=0;
	//struct psensor_dev *pdev = _dev;
	
	/* future capability*/
	PDBG("diyu/yong [%s] \n", __FUNCTION__);

	return 0;
}

static int gp2ap002_probe(struct i2c_client *client,
			const struct i2c_device_id *dev_id)
{
	struct psensor_dev *pdev; /*= &sharp_psensor;*/
	struct input_dev *input_dev;
	int ret;
				
	PDBG("diyu/yong [dist test] gp2ap002_probe addr = 0x%x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
			dev_err(&client->dev, "need I2C_FUNC_I2C\n");
			ret = -ENODEV;
			//goto err_check_functionality;
	}
	
	pdev = kzalloc(sizeof(struct psensor_dev), GFP_KERNEL);
	if (NULL == pdev) {
		ret = -ENOMEM;
		//goto err_alloc_data;
	}
	_dev = pdev; /* for miscdevice */

	pdev->client = client;
	sharp_psensor.client = client;
	pdev->use_irq = USE_IRQ;
//	pdev->sensor = &gp2ap002_dev;
	INIT_LIST_HEAD(&pdev->plist);
	pdev->status = OBJECT_NOT_DETECTED;
	pdev->ref_count = 0;
	i2c_set_clientdata(client, pdev);
	atomic_set(&s_prox_sensor, -1);
//	s_prox_sensor = 0;
	INIT_WORK(&pdev->work, gp2ap002_work_func);

#if 0
	ret = sharp_psensor_init(client);
	//ret = sharp_psensor_init();
	if(ret) 
		printk("Error proximity initializaton\n");
#endif	
	
	ret = misc_register(&gp2ap002_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register miscdevice\n");
		//goto err_miscdevice;
	}
	
	pdev->input_dev = input_allocate_device();
	if (NULL == pdev->input_dev) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		//goto err_input_allocate_device;
	}

	input_dev = pdev->input_dev;
	input_dev->name = "proximity";
	input_dev->phys = "proximity/input0";
	//input_dev->id.bustype = BUS_HOST;
	//input_dev->id.vendor = 0x0001;
	//input_dev->id.product = 0x0002;
	//input_dev->id.version = 0x0100;
	
	set_bit(EV_SYN, input_dev->evbit); //for sync
	set_bit(EV_ABS, input_dev->evbit);
//	set_bit(EV_DISTANCE, input_dev->evbit); // value of distance in proximity sensor
	input_set_abs_params(input_dev, ABS_DISTANCE, 0/*MIN*/, 1/*MIN*/, 0, 0);

	ret = input_register_device(input_dev);

	if (ret) {
		printk(KERN_ERR"sharp psensor unable to register device. %d\n", ret);
		//goto err_input_register_device;
	}

	PDBG ("diyu  client->irq : %d \n",client->irq ); 
	PDBG ("diyu  pdev->client->irq : %d \n",pdev->client->irq ); 
	//pdev->irq = gpio_direction_input(client->irq);
	PDBG ("diyu  client->irq : %d \n",pdev->irq ); 
	
	pdev->irq = client->irq;

	ret = request_irq(/*pdev->client->irq*/gpio_to_irq(GPIO_PROX_IRQ), gp2ap002_interrupt,  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, 
		"proximity", pdev);
	if (ret) {
		printk(KERN_ERR"sharp psensor request irq error !!\n");
		free_irq(pdev->irq, NULL);
		return ret;
	}

#if 1  /* initialize for function "sharp_gp2ap002_enable_store"*/
	irq_set = 0 ;
	disable_irq(gpio_to_irq(GPIO_PROX_IRQ));

#endif 
	ret = sharp_psensor_init(client);
	udelay(100);
	prox_vreg_set(0);

#if 1
	ret = device_create_file(&client->dev, &dev_attr_enable);
	if (ret) {
		printk( "android-proximity: device_create_fail : Fail\n");
		device_remove_file(&client->dev, &dev_attr_enable);
		//goto err_request_irq;
	}

	ret = device_create_file(&client->dev, &dev_attr_show);
	if (ret) {
		printk( "android-proximity: device_create_fail : Fail\n");
		device_remove_file(&client->dev, &dev_attr_show);
		//goto err_request_irq;
	}
#endif

#ifdef CONFIG_SYSFS
/*
	ret = sysfs_create_group(&client->dev.kobj, &psensor_attr_group);
	if (ret) {
		printk(KERN_INFO"psensor create sysfs file  failed!\n");
		return ret;
	}
*/
#endif

	return 0;
}

static int gp2ap002_remove(struct i2c_client *client)
{
	struct psensor_dev *pdev = &sharp_psensor;

	free_irq(pdev->irq, NULL);
#ifdef CONFIG_SYSFS
/*
	sysfs_remove_group(&client->dev.kobj, &psensor_attr_group);
*/
#endif
	input_unregister_device(pdev->input_dev);
	input_free_device(pdev->input_dev);
	misc_deregister(&gp2ap002_dev);
	kfree(pdev);
	
	return 0;
}

static struct i2c_device_id gp2ap002_idtable[] = {
        { "proximity_i2c", 1 },
};

static struct i2c_driver i2c_gp2ap002_driver = {
	.driver = {
			.name = "proximity_i2c"
		},
	.probe 	= gp2ap002_probe,
	.remove = __devexit_p(gp2ap002_remove),
	.id_table 	= gp2ap002_idtable,
#ifdef CONFIG_PM
	.suspend	= gp2ap002_suspend,
	.resume 	= gp2ap002_resume,
#endif
};

static int gp2ap002_init(void)
{
	int ret;
	PDBG("diyu/yong gp2ap002_init\n");

	proximity_wq = create_singlethread_workqueue("proximity_wq");
	if (!proximity_wq)
		return -ENOMEM;
	

	ret = i2c_add_driver(&i2c_gp2ap002_driver);
	if (ret) {
		printk(KERN_ERR"SHARP GP2AP002 : Proximity Sensor Driver Registeration Failed!!\n");
		return ret;
	}

	return 0;
}

static void gp2ap002_exit(void)
{
	i2c_del_driver(&i2c_gp2ap002_driver);
	if (proximity_wq)
		destroy_workqueue(proximity_wq);
}

module_init(gp2ap002_init);
module_exit(gp2ap002_exit);

MODULE_DESCRIPTION("Sharp Proximity Sensor Driver(gp2ap002)");
MODULE_AUTHOR("Dae il, yu <diyu@lge.com>");
MODULE_LICENSE("GPL");

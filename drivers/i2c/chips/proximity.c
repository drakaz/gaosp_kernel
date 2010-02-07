#include <linux/i2c.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <mach/vreg.h>
#include <asm/io.h>

#include <linux/uaccess.h>

#include <linux/wakelock.h>

#include <linux/proximity.h>

#define DEBUG 0

static struct i2c_client *proximity_pclient;

static int opened;
static int APP_STATE;

DECLARE_MUTEX(proximity);

static struct workqueue_struct *proximity_wq;

struct proximity_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int use_irq;
};

static struct vreg *vreg_proximity;

static int global_value;
static struct proximity_data *global_mt;

static DECLARE_WAIT_QUEUE_HEAD(g_data_ready_wait_queue);

static int proximity_i2c_write(unsigned char u_addr, unsigned char u_data);
static int proximity_i2c_read(unsigned char u_addr, unsigned char *pu_data);
static int proximity_remove(struct i2c_client *client);

static void proximity_mt_work_func(struct work_struct *work);
static irqreturn_t proximity_interrupt_handler(int irq, void *dev_id);

char proximity_outmod_reg;

struct class *proximity_class;
EXPORT_SYMBOL(proximity_class);

struct device *switch_dev;
EXPORT_SYMBOL(switch_dev);

static struct wake_lock proximity_wakeup;

int proximity_i2c_tx_data(char* txData, int length)
{
	int rc;
#if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

	struct i2c_msg msg[] = {
		{
			.addr = proximity_pclient->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	rc = i2c_transfer(proximity_pclient->adapter, msg, 1);
	if(rc < 0){
		printk(KERN_ERR "proximity: proximity_i2c_tx_data error %d\n", rc);

		return rc;
		}
	return 0;
}

static int proximity_i2c_write(unsigned char u_addr, unsigned char u_data)
{
	int rc;
	unsigned char buf[2];
#if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

	buf[0] = u_addr;
	buf[1] = u_data;
    
	rc = proximity_i2c_tx_data(buf, 2);
	if(rc < 0)
		printk(KERN_ERR "proximity: txdata error %d add:0x%02x data:0x%02x\n",
			rc, u_addr, u_data);
	return rc;	
}

static int proximity_i2c_rx_data(char* rxData, int length)
{
    int rc;
 #if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

    struct i2c_msg msgs[] = {
        {
            .addr = proximity_pclient->addr,
            .flags = 1, //TEST      
            .len = 2,
            .buf = rxData,
        },
    };
 
    rc = i2c_transfer(proximity_pclient->adapter, msgs, 1);
 
    if (rc < 0) {
        printk(KERN_ERR "proximity: proximity_i2c_rx_data error %d\n", rc);
        return rc;
    }
return 0;
}

static int proximity_i2c_read(unsigned char u_addr, unsigned char *pu_data)
{
    int rc;
    unsigned char buf[3];
 #if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

    memset(buf, 0, sizeof(buf));
 
    buf[0] = u_addr;
 
 
    rc = proximity_i2c_rx_data(&buf, 1);
 
	if (!rc)
        *pu_data = buf[0] << 8 | buf[1];
    else printk(KERN_ERR "proximity: i2c read failed\n");
    return rc;


}

static void proximity_chip_init(void)
{
#if 1
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

	if (!proximity_pclient) 
		return;

    /* proximity init sequence */
      
	/* delay 2 ms */
	msleep(2);
}

static int proximity_open(struct inode *ip, struct file *fp)
{
	int rc = -EBUSY;
#if 1
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif
	if (!opened) {
		rc = 0;
	}
	return rc;
}

static int proximity_on( void ) // To go back to operation
{
	uint16_t test_value= 0;
	short value;

#if 1
	printk(KERN_INFO "[PROXIMITY] %s, APP_STATE = %d", __FUNCTION__,APP_STATE);
#endif

	if( !APP_STATE )
		return printk("\n" );
	
	proximity_i2c_write(0x06, 0x18); // reg Symbol : CON -> force VOUT to go H
	proximity_i2c_write(0x02, 0x40); // reg Symbol : HYS -> prepare VO reset to 0
	proximity_i2c_write(0x04, 0x03); // reg Symbol : OPMOD -> release from shutdown

    // permit host's interrupt input
	enable_irq_wake(global_mt->client->irq);
	
	proximity_i2c_write(0x06, 0x00); // reg Symbol : CON -> force VOUT to go H
	
	printk(" : APP_STATE = %d\n", APP_STATE );

	proximity_i2c_read(0x00, &test_value );
	value = ( 0x01 & test_value ) ;
	global_value = value;
#if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s value = %d\n", __FUNCTION__,value);
#endif
	
	return 0;
}

static int proximity_off( void ) // To go shutdown : not allowed interrupts
{
#if 1
	printk(KERN_INFO "[PROXIMITY] %s", __FUNCTION__);
#endif

	// host's interrupt is made forbidden
	
	disable_irq_wake(global_mt->client->irq);
	
	proximity_i2c_write(0x04, 0x02); // reg Symbol : HYS -> Going back to operation

	printk(" : APP_STATE = %d\n", APP_STATE );
	return 0;
}
int proximity_get_value(void)
{
	return global_value;
}
static long proximity_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	void __user *argp = (void __user *)arg;

	switch(cmd) {
		
		case SHARP_GP2AP_OPEN: 
        {
			printk(KERN_INFO "[PROXIMITY] %s : case OPEN\n", __FUNCTION__);
			APP_STATE = APP_OPEN;
			proximity_on();	
        }
        break;    

    	case SHARP_GP2AP_CLOSE: 
        {
			printk(KERN_INFO "[PROXIMITY] %s : case CLOSE\n", __FUNCTION__);
			APP_STATE = APP_CLOSE;	
			proximity_off();
        }
        break;    
        
    	default:
        printk(KERN_INFO "[PROXIMITY] unknown ioctl %d\n", cmd);
        break;
	}
	return rc;
}

int proximity_i2c_sensor_init(void){
	
	int i;
	int rc=0;
	uint16_t auto_value = 0;
#if 1
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

	vreg_set_level(vreg_proximity, 3000); // set to 3.0V voltage 
	vreg_enable(vreg_proximity); // voltage 

//=========== to set interrupt mode ============

    proximity_i2c_write(0x01, 0x08); // reg Symbol : GAIN
    proximity_i2c_write(0x02, 0x40); // reg Symbol : HYS 
    proximity_i2c_write(0x03, 0x14); // reg symbol : CYCLE 32ms detection cycle
    proximity_i2c_write(0x04, 0x03); // reg symbol : OPMOD

	return 0;
}


static int proximity_init_client(struct i2c_client *client)
{
	/* Initialize the proximity Chip */
	init_waitqueue_head(&g_data_ready_wait_queue);
#if 1
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

	return 0;
}

static struct file_operations proximity_fops = {
        .owner 	= THIS_MODULE,
        .open 	= proximity_open,
        .unlocked_ioctl = proximity_ioctl,
};

static struct miscdevice proximity_device = {
        .minor 	= MISC_DYNAMIC_MINOR,
        .name 	= "proximity",
        .fops 	= &proximity_fops,
};

static int proximity_probe(struct i2c_client *client)
{
	struct proximity_data *mt;
	int err = 0;
#if 1
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

	uint16_t test_value = 0;
	
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		goto exit_check_functionality_failed;		
	
	if(!(mt = kzalloc( sizeof(struct proximity_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}


	INIT_WORK(&mt->work, proximity_mt_work_func );
	mt->client = client;
    global_mt = mt;

	i2c_set_clientdata(client, mt);

	proximity_init_client(client);
	proximity_pclient = client;
	proximity_chip_init();
	
	/* Register a misc device */
	err = misc_register(&proximity_device);
	if(err) {
		printk(KERN_ERR "proximity_probe: misc_register failed \n");
		goto exit_misc_device_register_failed;
	}

 	mt->input_dev = input_allocate_device();
	if (mt->input_dev == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR "proximity_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	mt->input_dev->name = "proximity_i2c";


	err = input_register_device(mt->input_dev);
	if (err) {
		printk(KERN_ERR "proximity_probe: Unable to register %s input device\n", mt->input_dev->name);
		goto err_input_register_device_failed;
	}

	proximity_i2c_sensor_init();

/* proximity/lightsensor i2c TEST */
	if ( proximity_i2c_read(0x00, &test_value ) < 0 ) {
		printk("proximity_i2c fail!! proximity/lightsensor wont be probed.\n");
		goto exit_check_functionality_failed;		
	}

	printk("[PROXIMITY] TEST_value = %x\n", test_value);
	
	/* [PROXIMITY] Edited for registration of irq - START */
	err = gpio_configure( 57, GPIOF_INPUT | IRQF_TRIGGER_FALLING );
	if(err)
		printk(KERN_ERR "gpio_request err\n");

//	printk("[PROXIMITY] try to register interrupt\n");
	if(request_irq( client->irq, proximity_interrupt_handler, IRQF_TRIGGER_FALLING, "proximity_i2c", mt )) 
	{
		free_irq(client->irq, NULL);
		printk("[error] proximity_interrupt_handler can't register the handler! and passing....\n");
	}
	enable_irq_wake( client->irq );

	/* [PROXIMITY] Edited for registration of irq - END */
	
	proximity_off( );
	APP_STATE = APP_CLOSE;

	wake_lock_init(&proximity_wakeup, WAKE_LOCK_SUSPEND, "proximity_wakeups");

	printk(KERN_INFO "[PROXIMITY] proximity_probe END!\n");

	return 0;
		
exit_misc_device_register_failed:
exit_alloc_data_failed:
exit_check_functionality_failed:
err_input_register_device_failed:
err_input_dev_alloc_failed:

	return err;
}

static int proximity_remove(struct i2c_client *client)
{
	struct proximity_data *mt = i2c_get_clientdata(client);
#if 1
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

	free_irq(client->irq, mt);
	i2c_detach_client(client);
	proximity_pclient = NULL;
	misc_deregister(&proximity_device);
	kfree(mt);
	return 0;
}

static void proximity_mt_work_func(struct work_struct *work)
{
	struct proximity_data *mt = container_of(work, struct proximity_data, work);
	
	uint16_t test_value= 0;
	short value;
	
	wake_lock_timeout(&proximity_wakeup, 2000);

//	Procedure 5
	proximity_i2c_read(0x00, &test_value );
	value = ( 0x01 & test_value ) ;

	global_value = value;
	printk(KERN_INFO "[PROXIMITY] %s : value = %d\n", __FUNCTION__,value);

//	Procedure 6
	if( value == 0 ) // VO=0 : no detection
	{
		proximity_i2c_write(0x02, 0x40); // reg Symbol : HYS for Mode. B1 
	}
	else if(value == 1) // VO=1 : detection
	{
		proximity_i2c_write(0x02, 0x20); // reg Symbol : HYS for Mode. B1 
	}
	else
		printk("VO is error.\n");
	msleep(500);

//	Procedure 7
	proximity_i2c_write(0x06, 0x18); // reg Symbol : CON, disable VOUT terminal
//	Procedure 8
	enable_irq(mt->client->irq);
//	Procedure 9
	proximity_i2c_write(0x06, 0x0); // reg Symbol : CON, enable VOUT terminal
}

static irqreturn_t proximity_interrupt_handler(int irq, void *dev_id)
{
	struct proximity_data *mt = dev_id;
#if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

//	Procedure 4
	disable_irq(mt->client->irq);
	
	queue_work(proximity_wq, &mt->work);	

	return IRQ_HANDLED;
} 

static const struct i2c_device_id proximity_id[] = {
	{ "proximity_i2c", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, proximity_id);


static struct i2c_driver proximity_driver = {
	.id_table	= proximity_id,
	.probe = proximity_probe,
	.remove = proximity_remove,
	.driver = {		
		.name   = "proximity_i2c",
	},
};

static ssize_t proximity_file_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif

	return sprintf(buf, "%d\n", global_value);
}

static ssize_t proximity_file_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
#if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s : operate nothing\n", __FUNCTION__);
#endif


	return ret;
}
static ssize_t proximity_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int value;
#if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s : operate nothing\n", __FUNCTION__);
#endif

	return sprintf(buf, "%d\n", value);
}

static ssize_t proximity_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
#if DEBUG
	printk(KERN_INFO "[PROXIMITY] %s\n", __FUNCTION__);
#endif
		
		if (value)
		{
			printk("[%s] CMD 1\n", __func__);
			APP_STATE = APP_OPEN;
			proximity_on();
	    }		
		else	
		{
			printk("[%s] CMD 0\n", __func__);
			APP_STATE = APP_CLOSE;
			proximity_off();
		}
	

	return size;
}

static DEVICE_ATTR(proximity_file_data, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, proximity_file_data_show, proximity_file_data_store);
static DEVICE_ATTR(proximity_file_cmd, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, proximity_file_cmd_show, proximity_file_cmd_store);

static int __init proximity_init(void)
{
	printk(KERN_INFO "===== [PROXIMITY] proximity sensor driver =====\n");
	printk("[PROXIMITY] __init proximity_init \n");
	
	proximity_wq= create_singlethread_workqueue("proximity_wq");
	if (!proximity_wq)
		return -ENOMEM; 
	
	vreg_proximity = vreg_get(0, "gp6");
	i2c_add_driver(&proximity_driver);
	if (IS_ERR(vreg_proximity))
	{	
		printk("===== [PROXIMITY] proximity IS_ERR TEST =====\n");
		return PTR_ERR(vreg_proximity);
	}

	proximity_class = class_create(THIS_MODULE, "proximity");
	if (IS_ERR(proximity_class))
		pr_err("Failed to create class(sec)!\n");

	switch_dev = device_create_drvdata(proximity_class, NULL, 0, NULL, "switch");
	if (IS_ERR(switch_dev))
		pr_err("Failed to create device(switch)!\n");

	if (device_create_file(switch_dev, &dev_attr_proximity_file_data) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_proximity_file_data.attr.name);
	if (device_create_file(switch_dev, &dev_attr_proximity_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_proximity_file_cmd.attr.name);

	return 0;
}

static void __exit proximity_exit(void)
{
	i2c_del_driver(&proximity_driver);
}

module_init(proximity_init);
module_exit(proximity_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Proximity Sensor Driver");
MODULE_LICENSE("GPL");

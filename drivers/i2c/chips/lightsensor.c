#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/uaccess.h>
#include <../../../arch/arm/mach-msm/proc_comm.h>
#include <linux/time.h>
#include <linux/timer.h>

#define suspend_test 1   // 1 is suspend, 0 is early_suspend

#ifndef suspend_test 
#include <linux/earlysuspend.h>
#endif

#define DEBUG 0
#define TIME_INT (2*HZ)  // 2 sec

static struct workqueue_struct *lightsensor_wq;

DECLARE_MUTEX(lightsensor);

struct lightsensor_data {
	struct input_dev *input_dev;
	struct work_struct  work;
	struct timer_list timer;
#ifndef suspend_test
	struct early_suspend early_suspend;
#endif
};

static int lightsensor_remove(struct platform_device *pdev);
static int lightsensor_probe(struct platform_device *pdev);
static void lightsensor_set_lcd_initial_brightness( void );

struct class *lightsensor_class;
EXPORT_SYMBOL(lightsensor_class);

struct device *switch_cmd_dev;
EXPORT_SYMBOL(switch_cmd_dev);

static int CMD_STATE=0;
static int low_battery_flag=0;
static int previous_level=0;
static int lightsensor_light_level=0;

extern void capela_set_lightsensor_level ( int level);
extern int bridge_on;
struct lightsensor_data *global_mt;
void lightsensor_timeover(unsigned long arg );

void lightsensor_low_battery_flag_set( int flag );
EXPORT_SYMBOL(lightsensor_low_battery_flag_set);

int get_lightsensor_level( void );
EXPORT_SYMBOL(get_lightsensor_level);

void lightsensor_registertimer(struct timer_list* ptimer, unsigned long timeover )
{
	init_timer( ptimer );
	ptimer->expires = get_jiffies_64() + timeover;
	ptimer->data = NULL;
	ptimer->function = lightsensor_timeover;
	add_timer( ptimer );
}

void lightsensor_timeover(unsigned long arg )
{
	queue_work(lightsensor_wq , &global_mt->work );
 	lightsensor_registertimer( &global_mt->timer, TIME_INT );	
}

int get_lightsensor_level( void )
{
	if ( !CMD_STATE )
		return -1;
			
	uint32_t raw_data = 0;
	int test_value=0;
	int present_level=0;

    	if ( low_battery_flag == 1 ){
		printk("[%s] flag = %d, stop\n", __func__ , low_battery_flag );
		return -1;
    	}

	msm_proc_comm(PCOM_CUSTOMER_CMD2, &raw_data, 0);
	test_value = raw_data;
	
	if( test_value < 17 )	
		present_level = 1;	   
	else if( test_value < 130 )
		present_level = 13;
	else 
		present_level = 22;

	previous_level=present_level;
	printk("[LIGHTSENSOR] get_lightsensor_level = %d\n", present_level );	
	
	return present_level;	
}

void lightsensor_low_battery_flag_set( int flag )
{
	static int cnt = 1;
	low_battery_flag = flag;
	printk("[%s] %d\n", __func__, low_battery_flag );

	if ( !cnt && CMD_STATE ) // if CMD_STATE is 1, call below fn.
		lightsensor_set_lcd_initial_brightness();
	cnt = cnt && 0;

	return;
}

static void lightsensor_work_func(struct work_struct *work)
{
	uint32_t raw_data = 0;
	int test_value=0;
	int present_level=0;
	static int previous_value=0;
	static int present_value=0;
	static int cnt=0;

#if DEBUG
	printk(KERN_INFO "[LIGHTSENSOR] %s, bridge_on=%d\n", __FUNCTION__, bridge_on);
#endif

    if ( low_battery_flag == 1 || bridge_on == 0 )
		return;

	msm_proc_comm(PCOM_CUSTOMER_CMD2, &raw_data , 0);
	test_value = raw_data;
	
	lightsensor_light_level = test_value;

	if( test_value < 17 )	
		present_value = 1;	   
	else if( test_value < 130 )
		present_value = 13;
	else 
		present_value = 22;

	if ( present_value == previous_value )
	{
		cnt++;
		
		if(cnt >= 3 )
		{
			present_level = present_value;
			if( !( present_level == previous_level ) )
				{
				capela_set_lightsensor_level ( present_level );
				previous_level = present_level;	
				printk("[LIGHTSENSOR] set level = %d\n", present_level );
			}
			cnt=0;
		}
		previous_value = present_value;
		return ;
	}
	else
	{
		cnt =0;
		previous_value = present_value;
	}
	return ;	
}

static void lightsensor_set_lcd_initial_brightness( void )
{
	uint32_t raw_data = 0;
	int test_value=0;
	int present_level=0;

    if ( low_battery_flag == 1 ){
		printk("[%s] flag = %d, stop\n", __func__ , low_battery_flag );
		return;
    }

	msm_proc_comm(PCOM_CUSTOMER_CMD2, &raw_data , 0);
	test_value = raw_data;
	
	if( test_value < 17 )	
		present_level = 1;	   
	else if( test_value < 130 )
		present_level = 13;
	else 
		present_level = 22;
	
	capela_set_lightsensor_level ( present_level );
	previous_level=present_level;
	printk("[LIGHTSENSOR] set level = %d\n", present_level );
}

#ifndef suspend_test 
static void lightsensor_eartly_suspend(struct early_suspend *h)
{
	pr_info("##### %s: CMD_STATE: %d\n", __func__, CMD_STATE );

	if ( CMD_STATE )
}

static void lightsensor_early_resume(struct early_suspend *h)
{
	pr_info("##### %s: CMD_STATE: %d\n", __func__, CMD_STATE );

	if ( CMD_STATE ) 
	
}
#endif

static int lightsensor_probe(struct platform_device *pdev)
{
	pr_info("%s: Called!\n", __func__);

	return 0;
}

static int lightsensor_remove(struct platform_device *pdev)
{
#if 1
	printk(KERN_INFO "[LIGHTSENSOR] %s\n", __FUNCTION__);
#endif

	return 0;
}

#if suspend_test
static int lightsensor_suspend(struct platform_device *pdev, pm_message_t mesg)
{
#if DEBUG
	printk("%s success CMD=%d\n", __func__, CMD_STATE );
#endif
	if ( CMD_STATE )

	return 0;
}

static int lightsensor_resume(struct platform_device *pdev)
{
#if DEBUG
	printk("%s success CMD=%d\n", __func__, CMD_STATE );
#endif
	if ( CMD_STATE ) 
 		return 0;
}
#endif

static ssize_t lightsensor_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int value=0;
#if DEBUG
	printk(KERN_INFO "[LIGHTSENSOR] %s : Error, operate nothing\n", __FUNCTION__);
#endif

	return sprintf(buf, "%d\n", value);
}

static ssize_t lightsensor_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{ 
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
#if DEBUG
	printk(KERN_INFO "[LIGHTSENSOR] %s\n", __FUNCTION__);
#endif
		
		if ( value == 1 )
		{
			printk("[%s] CMD 1\n", __func__);
			lightsensor_set_lcd_initial_brightness(); 
			if( CMD_STATE ==1 ) // timer is already running.
			{
				printk(KERN_INFO "[LIGHTSENSOR] %s : Error CMD, timer is already running!!\n", __FUNCTION__);	
				return size;
			}
 			lightsensor_registertimer( &global_mt->timer, TIME_INT );	
			CMD_STATE = 1; // running
	        }		
		else if( value == 0 )	
		{
			printk("[%s] CMD 0\n", __func__);
			if( CMD_STATE ==0 ) // timer is already stopped.
			{
				printk(KERN_INFO "[LIGHTSENSOR] %s : Error CMD, timer is already stopped!!\n", __FUNCTION__);	
				return size;
			}
			del_timer( &global_mt->timer );
			CMD_STATE = 0; // stop
		}
		else
			printk("[%s] Error CMD!! cmd=%d\n", __func__,value);

	return size;
}

static ssize_t lightsensor_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if DEBUG
		printk(KERN_INFO "[LIGHTSENSOR] %s, level=%d\n", __FUNCTION__,lightsensor_light_level );
#endif

	return sprintf(buf, "%d\n", lightsensor_light_level );
}

static ssize_t lightsensor_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
#if DEBUG
		printk(KERN_INFO "[LIGHTSENSOR] %s : Error, operate nothing\n", __FUNCTION__);
#endif
	
	return size;
}

static DEVICE_ATTR(lightsensor_file_cmd, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, lightsensor_file_cmd_show, lightsensor_file_cmd_store);
static DEVICE_ATTR(lightsensor_level, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, lightsensor_level_show, lightsensor_level_store);

static struct platform_driver lightsensor_driver = {
	.probe = lightsensor_probe,
	.remove = lightsensor_remove,
	.driver = {		
		.name   = "lightsensor",
	},
};

static struct platform_device lightsensor_device = {
	.name   = "lightsensor",
	.id = 0,
};

static int __init lightsensor_init(void)
{
	int err=0;
	struct lightsensor_data *mt;

	printk(KERN_INFO "===== [LIGHTSENSOR] lightsensor sensor driver =====\n");
	printk("[LIGHTSENSOR] _init lightsensor_init \n");

	lightsensor_wq= create_singlethread_workqueue("lightsensor_wq");
	if (!lightsensor_wq)
		return -ENOMEM; 
	
	lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(lightsensor_class))
		pr_err("Failed to create class(sec)!\n");

	switch_cmd_dev = device_create_drvdata(lightsensor_class, NULL, 0, NULL, "switch_cmd");
	if (IS_ERR(switch_cmd_dev))
		pr_err("Failed to create device(switch)!\n");

	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_cmd.attr.name);

	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_level) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_cmd.attr.name);



	if(!(mt = kzalloc( sizeof(struct lightsensor_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	global_mt = mt;
	platform_driver_register(&lightsensor_driver);
	
	platform_device_register(&lightsensor_device);

	INIT_WORK(&mt->work, lightsensor_work_func);
	queue_work(lightsensor_wq, &mt->work);

#ifndef suspend_test //def CONFIG_HAS_EARLYSUSPEND
	global_mt->early_suspend.suspend = lightsensor_eartly_suspend;
	global_mt->early_suspend.resume = lightsensor_early_resume;
	global_mt->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 3;  // CHK_THIS
	register_early_suspend(&global_mt->early_suspend);
#endif

	return 0;
	
exit_alloc_data_failed:
	return 0;
}

static void __exit lightsensor_exit(void)
{
	platform_driver_unregister(&lightsensor_driver);
}

#if suspend_test
EXPORT_SYMBOL(lightsensor_suspend);
EXPORT_SYMBOL(lightsensor_resume);
#endif

module_init(lightsensor_init);
module_exit(lightsensor_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Proximity Sensor Driver");
MODULE_LICENSE("GPL");

#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/uaccess.h>
#include <../../../arch/arm/mach-msm/proc_comm.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <linux/miscdevice.h>
#include <linux/i2c/lightsensor.h>
#include <linux/i2c/proximity.h>

#define suspend_test 1   // 1 is suspend, 0 is early_suspend
#ifndef suspend_test //def CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define LIGHTSENSOR_DEBUG 0
#if LIGHTSENSOR_DEBUG
#define PDBG(fmt, args...) printk(fmt, ##args)
#else
#define PDBG(fmt, args...) do {} while (0)
#endif /* PROXIMITY_DEBUG */

#define TIME_INT (2*HZ)  // 2 sec
static struct workqueue_struct *lightsensor_wq;
DECLARE_MUTEX(lightsensor);

struct lightsensor_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct timer_list timer;
#ifndef suspend_test //def CONFIG_HAS_EARLYSUSPEND
struct early_suspend early_suspend;
#endif
};

static int lightsensor_remove(struct platform_device *pdev);
static int lightsensor_probe(struct platform_device *pdev);
static void lightsensor_set_lcd_initial_brightness(void);

struct class *lightsensor_class;
EXPORT_SYMBOL(lightsensor_class);

struct device *switch_cmd_dev;
EXPORT_SYMBOL(switch_cmd_dev);

static int CMD_STATE = 0;
static int low_battery_flag = 0;
static int previous_level = 0;
static int lightsensor_light_level = 0;

extern void capela_set_lightsensor_level(int level);
extern int bridge_on;
struct lightsensor_data *global_mt;
void lightsensor_timeover(unsigned long arg);

void lightsensor_low_battery_flag_set(int flag);
EXPORT_SYMBOL(lightsensor_low_battery_flag_set);

void lightsensor_registertimer(struct timer_list* ptimer,
        unsigned long timeover)
{
	init_timer(ptimer);
	ptimer->expires = get_jiffies_64() + timeover;
	ptimer->data = 0;
	ptimer->function = lightsensor_timeover;
	add_timer(ptimer);
}

void lightsensor_timeover(unsigned long arg)
{
	queue_work(lightsensor_wq, &global_mt->work);
	lightsensor_registertimer(&global_mt->timer, TIME_INT);
}

int lightsensor_get_level(void)
{
	uint32_t raw_data = 0;
	int current_value = 0;

	msm_proc_comm(PCOM_CUSTOMER_CMD2, &raw_data, 0);
	current_value = raw_data;
	lightsensor_light_level = current_value;

	PDBG("[LIGHTSENSOR] %s = %d\n", __FUNCTION__, current_value);

	return current_value;
}

void lightsensor_low_battery_flag_set(int flag)
{
	static int cnt = 1;
	low_battery_flag = flag;
	printk("[LIGHTSENSOR] %s %d\n", __func__, low_battery_flag);

	//TODO: correctly handle low battery flag
	if (!cnt && CMD_STATE) // if CMD_STATE is 1, call below fn.
		lightsensor_set_lcd_initial_brightness();
	cnt = cnt && 0;

	return;
}

void lightsensor_report_data(void)
{
	int value;
	struct lightsensor_data *li = global_mt;

	value = lightsensor_get_level();

	PDBG("[LIGHTSENSOR] %s value=%d\n", __FUNCTION__, value);

	input_report_abs(li->input_dev, ABS_MISC, value);
	input_sync(li->input_dev);
}

static void lightsensor_work_func(struct work_struct *work)
{
	PDBG(KERN_INFO "[LIGHTSENSOR] %s, bridge_on=%d\n", __FUNCTION__, bridge_on);

	if (low_battery_flag == 1 || bridge_on == 0)
		return;

	lightsensor_report_data();

	return;
}

static int enable_lightsensor_work(bool enable)
{
	PDBG("[LIGHTSENSOR] %s enable : %d\n", __FUNCTION__, enable);
	if (enable) {
		// timer is already running.
		if (CMD_STATE == 1) {
			printk(
			        KERN_INFO "[LIGHTSENSOR] %s : Error CMD, timer is already running!!\n",
			        __FUNCTION__);
			return 1;
		}

		// lightsensor rely on gp2a002
		sharp_gp2ap002_enable(1, LIGHT);

		input_report_abs(global_mt->input_dev, ABS_MISC, 0);
		input_sync(global_mt->input_dev);

		lightsensor_registertimer(&global_mt->timer, TIME_INT);
		// running
		CMD_STATE = 1;
	} else {
		// timer is already stopped.
		if (CMD_STATE == 0) {
			printk(
			        KERN_INFO "[LIGHTSENSOR] %s : Error CMD, timer is already stopped!!\n",
			        __FUNCTION__);
			return 1;
		}
		del_timer(&global_mt->timer);
		sharp_gp2ap002_enable(0, LIGHT);
		CMD_STATE = 0; // stop
	}
	return 0;
}

static void lightsensor_set_lcd_initial_brightness(void)
{
	uint32_t raw_data = 0;
	int test_value = 0;
	int present_level = 0;

	if (low_battery_flag == 1) {
		printk("[LIGHTSENSOR] %s flag = %d, stop\n", __func__,
		        low_battery_flag);
		return;
	}

	msm_proc_comm(PCOM_CUSTOMER_CMD2, &raw_data, 0);
	test_value = raw_data;

	if (test_value < 17)
		present_level = 1;
	else if (test_value < 130)
		present_level = 13;
	else
		present_level = 22;

	capela_set_lightsensor_level(present_level);
	previous_level = present_level;
	printk("[LIGHTSENSOR] set level = %d\n", present_level);
}

#ifndef suspend_test //def CONFIG_HAS_EARLYSUSPEND
static void lightsensor_eartly_suspend(struct early_suspend *h)
{
	pr_info("##### %s: CMD_STATE: %d\n", __func__, CMD_STATE );

	if ( CMD_STATE )

#if 0
	hrtimer_cancel(&global_mt->timer);
#endif
}

static void lightsensor_early_resume(struct early_suspend *h)
{
	pr_info("##### %s: CMD_STATE: %d\n", __func__, CMD_STATE );

	if ( CMD_STATE )

#if 0
	hrtimer_start(&global_mt->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif
}
#endif

static int lightsensor_probe(struct platform_device *pdev)
{
	PDBG("[LIGHTSENSOR] %s: Called!\n", __func__);

	return 0;
}

static int lightsensor_remove(struct platform_device *pdev)
{
	PDBG(KERN_INFO "[LIGHTSENSOR] %s\n", __FUNCTION__);

	return 0;
}

#if suspend_test
static int lightsensor_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	PDBG("[LIGHTSENSOR] %s success CMD=%d\n", __func__, CMD_STATE);

#if 0
	if (CMD_STATE)
		hrtimer_cancel(&global_mt->timer);
#endif
	return 0;
}

static int lightsensor_resume(struct platform_device *pdev)
{
	PDBG("[LIGHTSENSOR] %s success CMD=%d\n", __func__, CMD_STATE);
#if 0
	if (CMD_STATE)
		hrtimer_start(&global_mt->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif
	return 0;
}
#endif

static ssize_t lightsensor_file_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	int value = lightsensor_get_level();
	PDBG(KERN_INFO "[LIGHTSENSOR] %s value = %d\n", __FUNCTION__, value);

	return sprintf(buf, "%d\n", value);
}

static ssize_t lightsensor_file_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);
	PDBG(KERN_INFO "[LIGHTSENSOR] %s CMD %ld\n", __FUNCTION__, value);

	if (value != 0 && value != 1)
		printk("[LIGHTSENSOR] %s Error CMD!! cmd=%ld\n", __func__,
		        value);

	enable_lightsensor_work(value == 1);

	return size;
}

static ssize_t lightsensor_level_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	PDBG(KERN_INFO "[LIGHTSENSOR] %s, level=%d\n", __FUNCTION__, lightsensor_light_level);

	return sprintf(buf, "%d\n", lightsensor_light_level);
}

static ssize_t lightsensor_level_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	pr_err(KERN_INFO "[LIGHTSENSOR] %s : Error, operate nothing\n", __FUNCTION__);

	return size;
}

static DEVICE_ATTR(lightsensor_file_cmd, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, lightsensor_file_cmd_show, lightsensor_file_cmd_store);
static DEVICE_ATTR(lightsensor_level, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, lightsensor_level_show, lightsensor_level_store);

static struct platform_driver lightsensor_driver = {
        .probe = lightsensor_probe,
        .remove = lightsensor_remove,
#if suspend_test
        //	.suspend = lightsensor_suspend,
        //	.resume = lightsensor_resume,
#endif
        .driver = { .name = "lightsensor", }, };

static struct platform_device lightsensor_device = {
        .name = "lightsensor",
        .id = 0, };

DEFINE_MUTEX(ls_i2c_api_lock);
static int lightsensor_opened;

static int lightsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	PDBG("[LIGHTSENSOR] %s\n", __func__);
	mutex_lock(&ls_i2c_api_lock);
	if (lightsensor_opened) {
		pr_err("[LIGHTSENSOR] %s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lightsensor_opened = 1;
	mutex_unlock(&ls_i2c_api_lock);
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	PDBG("[LIGHTSENSOR] %s\n", __func__);
	mutex_lock(&ls_i2c_api_lock);
	lightsensor_opened = 0;
	mutex_unlock(&ls_i2c_api_lock);
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
        unsigned long arg)
{
	int rc, val;
	mutex_lock(&ls_i2c_api_lock);
	PDBG("[LIGHTSENSOR] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd)
	{
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		pr_info("[LIGHTSENSOR] %s set value = %d\n", __func__, val);
		rc = enable_lightsensor_work(val != 0);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = CMD_STATE;
		pr_info("[LIGHTSENSOR] %s get enabled status: %d\n", __func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[LIGHTSENSOR] %s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	mutex_unlock(&ls_i2c_api_lock);
	return rc;
}

static struct file_operations lightsensor_fops = {
        .owner = THIS_MODULE,
        .open = lightsensor_open,
        .release = lightsensor_release,
        .unlocked_ioctl = lightsensor_ioctl };

static struct miscdevice lightsensor_misc = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "lightsensor",
        .fops = &lightsensor_fops };

static int __init lightsensor_init(void)
{
	int err = 0;
	struct lightsensor_data *mt;

	printk(KERN_INFO "===== [LIGHTSENSOR] lightsensor sensor driver =====\n");
	printk("[LIGHTSENSOR] _init lightsensor_init \n");

	lightsensor_wq = create_singlethread_workqueue("lightsensor_wq");
	if (!lightsensor_wq)
		return -ENOMEM;

	lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(lightsensor_class))
		pr_err("[LIGHTSENSOR] Failed to create class(sec)!\n");

	switch_cmd_dev = device_create(lightsensor_class, NULL, 0, NULL,
	        "switch_cmd");
	if (IS_ERR(switch_cmd_dev))
		pr_err("[LIGHTSENSOR] Failed to create device(switch)!\n");

	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd)
	        < 0)
		pr_err("[LIGHTSENSOR] Failed to create device file(%s)!\n", dev_attr_lightsensor_file_cmd.attr.name);

	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_level) < 0)
		pr_err("[LIGHTSENSOR] Failed to create device file(%s)!\n", dev_attr_lightsensor_file_cmd.attr.name);

	if (!(mt = kzalloc(sizeof(struct lightsensor_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	global_mt = mt;

	global_mt->input_dev = input_allocate_device();
	if (!global_mt->input_dev) {
		pr_err("[LIGHTSENSOR] %s: could not allocate input device\n", __func__);
		return -ENOMEM;
	}
	global_mt->input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, global_mt->input_dev->evbit);
	input_set_abs_params(global_mt->input_dev, ABS_MISC, 0, 9, 0, 0);

	err = input_register_device(global_mt->input_dev);
	if (err < 0) {
		pr_err("[LIGHTSENSOR] %s: can not register input device\n",
			__func__);
		return err;
	}

	err = misc_register(&lightsensor_misc);
	if (err < 0) {
		pr_err("[LIGHTSENSOR] %s: can not register misc device\n",
			__func__);
		return err;
	}

	platform_driver_register(&lightsensor_driver);

	platform_device_register(&lightsensor_device);

	INIT_WORK(&mt->work, lightsensor_work_func);
	queue_work(lightsensor_wq, &mt->work);

#ifndef suspend_test //def CONFIG_HAS_EARLYSUSPEND
	global_mt->early_suspend.suspend = lightsensor_eartly_suspend;
	global_mt->early_suspend.resume = lightsensor_early_resume;
	global_mt->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 3; // CHK_THIS
	register_early_suspend(&global_mt->early_suspend);
#endif

	return 0;

	exit_alloc_data_failed: return 0;
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

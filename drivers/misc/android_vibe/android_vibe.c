/*
** =========================================================================
** File:
**     android_vibe.c
**
** Description: 
**     VibeTonz Kernel Module main entry-point.
**
** Portions Copyright (c) 2008 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** VibeTonzSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>

#include <linux/android_vibe.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <mach/vreg.h>

#include <linux/delay.h>
#include <linux/timed_gpio.h>
#include <linux/timed_output.h>
#include <linux/hrtimer.h>

/* Device name and version information */
#define VERSION_STR " v2.0.92.3\n"                  /* PLEASE DO NOT CHANGE - this is auto-generated */
#define VERSION_STR_LEN 16                          /* account extra space for future extra digits in version number */
static char g_szDeviceName[   VIBE_MAX_DEVICE_NAME_LENGTH 
                            + VERSION_STR_LEN];     /* initialized in init_module */
static size_t g_cchDeviceName;                      /* initialized in init_module */

/* Flag indicating whether the driver is in use */
static char g_bIsPlaying = false;

struct clk *android_vib_clk; /* gp_clk */

static struct hrtimer android_timer; // for 3.10 cupcake
static int is_vib_on = 0;

/* For QA purposes */
#ifdef QA_TEST
#define FORCE_LOG_BUFFER_SIZE   128
#define TIME_INCREMENT          5
static int g_nTime = 0;
static int g_nForceLogIndex = 0;
static VibeInt8 g_nForceLog[FORCE_LOG_BUFFER_SIZE];
#endif

#define GP_CLK_M_DEFAULT			21
#define GP_CLK_N_DEFAULT			18000
#define GP_CLK_D_DEFAULT			9000	/* 50% duty cycle */ 
#define IMM_PWM_MULTIPLIER		    17778	/* Must be integer */
/* Variable defined to allow for tuning of the handset. */
//#define VIBETONZ_TUNING /* For temporary section for Tuning Work */

/*
** Global variables for LRA PWM M,N and D values.
*/
VibeInt32 g_nLRA_GP_CLK_M = GP_CLK_M_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_N = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_D = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_PWM_MUL = IMM_PWM_MULTIPLIER;

#if ((LINUX_VERSION_CODE & 0xFFFF00) < KERNEL_VERSION(2,6,0))
#error Unsupported Kernel version
#endif

//#define IMPLEMENT_AS_CHAR_DRIVER

#ifdef IMPLEMENT_AS_CHAR_DRIVER
static int g_nMajor = 0;
#endif

/* added by gtuo.park */
struct timed_gpio_data {
	struct device *dev;
	struct hrtimer timer;
	spinlock_t lock;
	unsigned    gpio;
	int         max_timeout;
	u8      active_low;
};
struct timed_gpio_data *gpio_data;
extern struct class *timed_output_class;

/* Needs to be included after the global variables because it uses them */
//#include <VibeOSKernelLinuxTime.c>

/* File IO */
//static int ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int suspend(struct platform_device *pdev, pm_message_t state);
static int resume(struct platform_device *pdev);

#define TIMER_INCR                      1       /* run timer at 1 jiffie */
#define WATCHDOG_TIMEOUT                10      /* 10 timer cycles = 50ms */


DECLARE_MUTEX(g_hMutex);

/* Variable for setting PWM in Force Out Set */
VibeInt32 g_nForce_32 = 0;


/* High resolution timer funstions */
static enum hrtimer_restart vibetonz_timer_func(struct hrtimer *timer)
{
	unsigned int remain;

	if(hrtimer_active(&android_timer)) {
		ktime_t r = hrtimer_get_remaining(&android_timer);
		remain=r.tv.sec * 1000000 + r.tv.nsec;
		remain = remain / 1000;
//		printk("vibrator time remain:%dsec/%dnsec remain:%d\n",r.tv.sec, r.tv.nsec,remain);
		if(r.tv.sec < 0) {
			remain = 0;
		}
//		if((r.tv.sec > 0) || (r.tv.nsec > 0))
		if(!remain) {
			clk_disable(android_vib_clk);
			gpio_direction_output(101,0);
			is_vib_on = 0;
		} 
	} else {
		printk("hrtimer end!\n");
		clk_disable(android_vib_clk);
		gpio_direction_output(101,0);
		is_vib_on = 0;
	}
//	clk_disable(android_vib_clk);
//	gpio_direction_output(101, 0);
	return HRTIMER_NORESTART;
}


/*
** This function is used to set and re-set the GP_CLK M and N counters
** to output the desired target frequency.
**
*/

static  VibeStatus vibe_set_pwm_freq(VibeInt8 nForce)
{
   /* Put the MND counter in reset mode for programming */
   HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 0);
   HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK, 0 << HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT);	/* P: 0 => Freq/1, 1 => Freq/2, 4 => Freq/4 */
   HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_SRC_SEL_BMSK, 0 << HWIO_GP_NS_REG_SRC_SEL_SHFT);	/* S : 0 => TXCO(19.2MHz), 1 => Sleep XTAL(32kHz) */
   HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_MODE_BMSK, 2 << HWIO_GP_NS_REG_MNCNTR_MODE_SHFT);	/* Dual-edge mode */
   HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_M_VAL_BMSK, g_nLRA_GP_CLK_M << HWIO_GP_MD_REG_M_VAL_SHFT);
   g_nForce_32 = ((nForce * g_nLRA_GP_CLK_PWM_MUL) >> 8) + g_nLRA_GP_CLK_D;
   printk("%s, g_nForce_32 : %d\n",__FUNCTION__,g_nForce_32);
   HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK, ( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT);
   HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_N_VAL_BMSK, ~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M) << HWIO_GP_NS_REG_GP_N_VAL_SHFT);
   HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);			           /* Enable M/N counter */
   printk("%x, %x, %x\n",( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT,~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M) << HWIO_GP_NS_REG_GP_N_VAL_SHFT,1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);

	return VIBE_S_SUCCESS;
}

static int android_vib_power(int on)
{
	struct vreg *vreg_vibetoz;

	vreg_vibetoz = vreg_get(0, "gp2");

    if(on)
    {
	  vreg_set_level(vreg_vibetoz, 3000);
	  vreg_enable(vreg_vibetoz);
    }
	else
	{
	  vreg_disable(vreg_vibetoz);
	}
	return 0;
}



#if 0
static int ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    printk(KERN_INFO "android_vibe: ioctl.cmd= %d\n",cmd);

    switch (cmd)
    {
		case VIBRATION_ON:
			gpio_direction_output(101,1);
			printk("android_vibe : turn on MOTOR\n");
			break;
		case VIBRATION_OFF:
			gpio_direction_output(101,0);
			printk("android_vibe : turn off MOTOR\n");
			break;
	}

	return 0;
}
#endif

static int vibrator_value;
static int vibrator_pwm;
static int current_pwm;
static int set_vibetonz(int timeout)
{
//	printk("[VIB] : %d\n",timeout);

	hrtimer_cancel(&android_timer);

	if(!timeout) {
		if(is_vib_on) {
			clk_disable(android_vib_clk);
			gpio_direction_output(101, 0);
			is_vib_on = 0;
		}
	} else {
		if(timeout < 50) {
			vibrator_pwm = 140;
			if(current_pwm != vibrator_pwm) {
				vibe_set_pwm_freq(vibrator_pwm);
				current_pwm = vibrator_pwm;
			}
		} else {
			vibrator_pwm = 210;
			if(current_pwm != vibrator_pwm) {
				if(timeout == 10000) {
					vibrator_pwm = 250;
				} else {
					vibrator_pwm = 210;
				}
				vibe_set_pwm_freq(vibrator_pwm);
				current_pwm = vibrator_pwm;
			}
		}
		gpio_direction_output(101,1);
		clk_enable(android_vib_clk);
		hrtimer_start(&android_timer,ktime_set(timeout / 1000, (timeout % 1000) * 1000000),
					HRTIMER_MODE_REL);
		is_vib_on = 1;
	}
	vibrator_value = timeout;

	return 0;
}



static ssize_t show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", vibrator_value);
}

static ssize_t store_enable(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct timed_gpio_data *gpio_data = dev_get_drvdata(dev);
	int value;
	unsigned long flags;

	sscanf(buf, "%d", &value);
//	printk("Android Vibetonz, value : %d\n",value);
	spin_lock_irqsave(&gpio_data->lock, flags);
	/* TODO locking... */
	set_vibetonz(value);
	/* TODO unlocking... */
	spin_unlock_irqrestore(&gpio_data->lock, flags);

	return size;
}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", vibrator_pwm);
}

static ssize_t store_pwm(
	   struct device *dev, struct device_attribute *attr,
	   const char *buf, size_t size) {

	int value;
	sscanf(buf, "%d", &value);
	printk("PWM Changed to %d\n",value);
	vibrator_pwm = value;
	if(vibrator_pwm < 0) {
		printk("wrong pwm value\n");
		return 0;
	} else if(vibrator_pwm > 255) {
		printk("wrong pwm value\n");
		return 0;
	}

	vibe_set_pwm_freq(vibrator_pwm);

	return vibrator_pwm;

}
static DEVICE_ATTR(enable, 0777, show_enable,store_enable);
static DEVICE_ATTR(pwm, 0777, show_pwm, store_pwm);


static int suspend(struct platform_device *pdev, pm_message_t state) 
{
    if (g_bIsPlaying)
    {
        printk(KERN_INFO "android_vibe: can't suspend, still playing effects.\n");
        return -EBUSY;
    }
    else
    {
        printk(KERN_INFO "android_vibe: suspend.\n");
        return 0;
    }
}

static int resume(struct platform_device *pdev) 
{	
    printk(KERN_INFO "android_vibe: resume.\n");

	return 0;   /* can resume */
}

static int __init vibetonz_init(void)
{
    int nRet;   /* initialized below */

	struct device_driver driver;

	driver.name = "vibrator";
	driver.owner = THIS_MODULE;

	
    printk(KERN_INFO "android_vibe: vibetonz_init.\n");
	
	gpio_data = kzalloc(sizeof(struct timed_gpio_data), GFP_KERNEL);
	if (!gpio_data)
		return -ENOMEM;

	hrtimer_init(&android_timer, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
	android_timer.function = vibetonz_timer_func;

	spin_lock_init(&gpio_data->lock);

	if(timed_output_class == NULL) {
		timed_output_class = class_create(THIS_MODULE, "timed_output");
	}

	gpio_data->dev = device_create(timed_output_class, NULL, 0, "%s", "vibrator");

	nRet = device_create_file(gpio_data->dev, &dev_attr_enable);
	if (nRet)
		return nRet;
	nRet = device_create_file(gpio_data->dev, &dev_attr_pwm);
	if (nRet)
		return nRet;

    /* Append version information and get buffer length */
    strcat(g_szDeviceName, VERSION_STR);
    g_cchDeviceName = strlen(g_szDeviceName);


	/* initialize android viberation mode */
	android_vib_clk = clk_get(NULL,"gp_clk");
	android_vib_power(1);

	/* clock enable and freq set to 210 */
	/* PWM Max is 255 */
	vibrator_pwm = 210;
	current_pwm = vibrator_pwm;
	vibe_set_pwm_freq(vibrator_pwm);

	printk("android viberation initialize OK(base pwm : 180)\n");
	return 0;


    return 0;
}

static void __exit vibetonz_exit(void)
{
    printk(KERN_INFO "android_vibe: cleanup_module.\n");

	hrtimer_cancel(&android_timer);
	/* vibrator clock disable */
	if(is_vib_on) {
		clk_disable(android_vib_clk);
		gpio_direction_output(101,0);
	}


	android_vib_power(0);
	device_remove_file(gpio_data->dev, &dev_attr_enable);
	device_remove_file(gpio_data->dev, &dev_attr_pwm);
	device_destroy(timed_output_class, 0);
	kfree(gpio_data);

}

module_init(vibetonz_init);
module_exit(vibetonz_exit);

/* Module info */
MODULE_AUTHOR("Immersion Corporation");
MODULE_DESCRIPTION("VibeTonz Kernel Module");
MODULE_LICENSE("GPL v2");


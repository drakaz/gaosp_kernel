#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/blkpg.h>
#include <linux/hdreg.h>
#include <linux/genhd.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>
#include <linux/reboot.h>

#include "proc_comm.h"
    
#include <mach/hardware.h>
#include "smd_private.h"
#include <linux/delay.h>


#define DL_SET  (MSM_SHARED_RAM_BASE + 0x20D0)
#define DLOAD_MAGIC1 0xFCDE8462
#define DLOAD_MAGIC2 0x0F1E2000

#define DL_CMD	"DOWNLOAD"

extern uint32_t restart_reason;

static struct class *sec_class;
static struct device *param_dev;

static ssize_t download_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
}

extern int mark_jtag_download_mode(void);


ssize_t download_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	static int count = 0;
	
	if(!strncmp(buf, DL_CMD, strlen(DL_CMD))) { 
		writel(DLOAD_MAGIC1, DL_SET); 
                writel(DLOAD_MAGIC2, DL_SET + 0x04); 
		restart_reason = 0xAAAAAAAA;

		smsm_reset_modem(SMSM_SYSTEM_DOWNLOAD);
		msleep(500);
		msm_proc_comm_reset_modem_now();

		printk("%s : %s downloader mode setting\n", __FUNCTION__, buf);
	}else {
		printk("cmd = %s\n", buf);
	}
	
}

DEVICE_ATTR(download_mode, S_IRUGO | S_IWUSR, download_mode_show, download_mode_store);

static int __init param_init(void)
{
	printk("%s\n", __FUNCTION__);
	if(!sec_class) {
		sec_class = class_create(THIS_MODULE, "sec");
		if (IS_ERR(sec_class))
			return PTR_ERR(sec_class);
	}

	param_dev = device_create_drvdata(sec_class, NULL, 0, NULL, "param");
	if(IS_ERR(param_dev)) {
		class_destroy(sec_class);
		return PTR_ERR(param_dev);
	}
	device_create_file(param_dev, &dev_attr_download_mode);
}

static void __exit param_exit(void)
{
	printk("%s\n", __FUNCTION__);
	device_remove_file(param_dev, &dev_attr_download_mode);
	class_destroy(sec_class);
}

module_init(param_init);
module_exit(param_exit);

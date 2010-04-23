#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/module.h>


#define SHARP_GP2AP_IOC_MAGIC	'C'
#define SHARP_GP2AP_OPEN	_IO(SHARP_GP2AP_IOC_MAGIC,1)
#define SHARP_GP2AP_CLOSE	_IO(SHARP_GP2AP_IOC_MAGIC,2)

 int proximity_get_value(void);
// application state define
#define APP_OPEN 1
#define APP_CLOSE 0


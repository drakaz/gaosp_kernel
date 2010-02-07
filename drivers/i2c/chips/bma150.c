/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2008 Bosch Sensortec GmbH
 * All Rights Reserved
 */

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

#include "smb380.h"
#include "smb380.c"

#define BMA150_MAJOR	100
#define BMA150_MINOR	3

#define BMA150_IOC_MAGIC 'B'

#define BMA150_SOFT_RESET			_IO(BMA150_IOC_MAGIC,0)
#define BMA150_GET_OFFSET			_IOWR(BMA150_IOC_MAGIC,1, short)
#define BMA150_SET_OFFSET			_IOWR(BMA150_IOC_MAGIC,2, short)
#define BMA150_SELFTEST				_IOWR(BMA150_IOC_MAGIC,3, unsigned char)
#define BMA150_SET_RANGE			_IOWR(BMA150_IOC_MAGIC,4, unsigned char)
#define BMA150_GET_RANGE			_IOWR(BMA150_IOC_MAGIC,5, unsigned char)
#define BMA150_SET_MODE				_IOWR(BMA150_IOC_MAGIC,6, unsigned char)
#define BMA150_GET_MODE				_IOWR(BMA150_IOC_MAGIC,7, unsigned char)
#define BMA150_SET_BANDWIDTH			_IOWR(BMA150_IOC_MAGIC,8, unsigned char)
#define BMA150_GET_BANDWIDTH			_IOWR(BMA150_IOC_MAGIC,9, unsigned char)
#define BMA150_SET_WAKE_UP_PAUSE		_IOWR(BMA150_IOC_MAGIC,10,unsigned char)
#define BMA150_GET_WAKE_UP_PAUSE		_IOWR(BMA150_IOC_MAGIC,11,unsigned char)
#define BMA150_SET_LOW_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,12,unsigned char)
#define BMA150_GET_LOW_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,13,unsigned char)
#define BMA150_SET_LOW_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,14,unsigned char)
#define BMA150_GET_LOW_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,15,unsigned char)
#define BMA150_SET_HIGH_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,16,unsigned char)
#define BMA150_GET_HIGH_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,17,unsigned char)
#define BMA150_SET_LOW_G_DURATION		_IOWR(BMA150_IOC_MAGIC,18,unsigned char)
#define BMA150_GET_LOW_G_DURATION		_IOWR(BMA150_IOC_MAGIC,19,unsigned char)
#define BMA150_SET_HIGH_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,20,unsigned char)
#define BMA150_GET_HIGH_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,21,unsigned char)
#define BMA150_SET_HIGH_G_DURATION		_IOWR(BMA150_IOC_MAGIC,22,unsigned char)
#define BMA150_GET_HIGH_G_DURATION		_IOWR(BMA150_IOC_MAGIC,23,unsigned char)
#define BMA150_SET_ANY_MOTION_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,24,unsigned char)
#define BMA150_GET_ANY_MOTION_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,25,unsigned char)
#define BMA150_SET_ANY_MOTION_COUNT		_IOWR(BMA150_IOC_MAGIC,26,unsigned char)
#define BMA150_GET_ANY_MOTION_COUNT		_IOWR(BMA150_IOC_MAGIC,27,unsigned char)
#define BMA150_SET_INTERRUPT_MASK		_IOWR(BMA150_IOC_MAGIC,28,unsigned char)
#define BMA150_GET_INTERRUPT_MASK		_IOWR(BMA150_IOC_MAGIC,29,unsigned char)
#define BMA150_RESET_INTERRUPT			_IO(BMA150_IOC_MAGIC,30)
#define BMA150_READ_ACCEL_X			_IOWR(BMA150_IOC_MAGIC,31,short)
#define BMA150_READ_ACCEL_Y			_IOWR(BMA150_IOC_MAGIC,32,short)
#define BMA150_READ_ACCEL_Z			_IOWR(BMA150_IOC_MAGIC,33,short)
#define BMA150_GET_INTERRUPT_STATUS		_IOWR(BMA150_IOC_MAGIC,34,unsigned char)
#define BMA150_SET_LOW_G_INT			_IOWR(BMA150_IOC_MAGIC,35,unsigned char)
#define BMA150_SET_HIGH_G_INT			_IOWR(BMA150_IOC_MAGIC,36,unsigned char)
#define BMA150_SET_ANY_MOTION_INT		_IOWR(BMA150_IOC_MAGIC,37,unsigned char)
#define BMA150_SET_ALERT_INT			_IOWR(BMA150_IOC_MAGIC,38,unsigned char)
#define BMA150_SET_ADVANCED_INT			_IOWR(BMA150_IOC_MAGIC,39,unsigned char)
#define BMA150_LATCH_INT			_IOWR(BMA150_IOC_MAGIC,40,unsigned char)
#define BMA150_SET_NEW_DATA_INT			_IOWR(BMA150_IOC_MAGIC,41,unsigned char)
#define BMA150_GET_LOW_G_HYST			_IOWR(BMA150_IOC_MAGIC,42,unsigned char)
#define BMA150_SET_LOW_G_HYST			_IOWR(BMA150_IOC_MAGIC,43,unsigned char)
#define BMA150_GET_HIGH_G_HYST			_IOWR(BMA150_IOC_MAGIC,44,unsigned char)
#define BMA150_SET_HIGH_G_HYST			_IOWR(BMA150_IOC_MAGIC,45,unsigned char)
#define BMA150_READ_ACCEL_XYZ			_IOWR(BMA150_IOC_MAGIC,46,short)
#define BMA150_READ_TEMPERATURE			_IOWR(BMA150_IOC_MAGIC,47,short)

#define BMA150_IOC_MAXNR			48


#define DEBUG 0

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

static struct i2c_client *bma150_client = NULL;

struct bma150_data{
	struct i2c_client client;
};



static smb380_t smb380;

static struct class *bma_dev_class;

static int bma150_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);

static char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);

static const struct i2c_device_id bma150_id[] = {
        { "bma150", 0 },
        { }
};

/*	i2c write routine for bma150	*/
static inline char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	int dummy;	
#if DEBUG
	printk(KERN_INFO"[BMA150] %s\n", __FUNCTION__);
#endif
	if( bma150_client == NULL )	/*	No global client pointer?	*/
		return -1;
	dummy = i2c_smbus_write_byte_data(bma150_client, reg_addr, data[0]);
	return dummy;	
}

/*	i2c read routine for bma150	*/
static inline char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len) 
{
	int dummy=0;
	int i=0;
#if DEBUG
	printk(KERN_INFO"[BMA150] %s\n", __FUNCTION__);
#endif
	if( bma150_client == NULL )	/*	No global client pointer?	*/
		return -1;
	while(i<len)
	{        
		dummy = i2c_smbus_read_word_data(bma150_client, reg_addr);
		if (dummy>=0)
		{         
			data[i] = dummy & 0x00ff;
			i++;
			if (i<len)
			{            
				data[i] = (dummy>>8)&0x00ff;
				i++;
			}
			reg_addr+=2;
		} 
		else 
			return dummy;
		dummy = len;
	}
	return dummy;
}

/*	read command for BMA150 device file	*/
static ssize_t bma150_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	#if DEBUG	
	smb380acc_t acc;	
	#endif
	if( bma150_client == NULL )
		return -1;
	#if DEBUG
	smb380_read_accel_xyz(&acc);
	printk("[BMA150] X axis: %d\n" , acc.x);
	printk("[BMA150] Y axis: %d\n" , acc.y); 
	printk("[BMA150] Z axis: %d\n" , acc.z);  
	#endif
	return 0;
}

/*	write command for BMA150 device file	*/
static ssize_t bma150_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	if( bma150_client == NULL )
		return -1;
	#if DEBUG
	printk("[BMA150] BMA150 should be accessed with ioctl command\n");
	#endif
	return 0;
}

/*	open command for BMA150 device file	*/
static int bma150_open(struct inode *inode, struct file *file)
{
	if( bma150_client == NULL)
	{
		#if DEBUG
		printk("I2C driver not install\n"); 
		#endif
		return -1;
	}
	smb380.bus_write = bma150_i2c_write;
	smb380.bus_read = bma150_i2c_read;
	smb380_init(&smb380);

	if (smb380.chip_id>0)
	{
		#if DEBUG
		printk("[BMA150] ChipId: 0x%x\n" , smb380.chip_id); 
		printk("[BMA150] ALVer: 0x%x MLVer: 0x%x\n", smb380.al_version, smb380.ml_version);
		#endif
	}
	else
	{
		#if DEBUG
		printk("BMA150: open error\n"); 
		#endif
		return -1;
	}
	#if DEBUG
	printk("BMA150 has been opened\n");
	#endif
	return 0;
}

/*	release command for BMA150 device file	*/
static int bma150_close(struct inode *inode, struct file *file)
{
	#if DEBUG	
	printk("[BMA150] has been closed\n");	
	#endif
    smb380_set_mode(SMB380_MODE_SLEEP); // to mode_sleep
	return 0;
}


/*	ioctl command for BMA150 device file	*/
static int bma150_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];

	/* check cmd */
	if(_IOC_TYPE(cmd) != BMA150_IOC_MAGIC)	
	{
		#if DEBUG		
		printk("cmd magic type error\n");
		#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > BMA150_IOC_MAXNR)
	{
		#if DEBUG
		printk("cmd number error\n");
		#endif
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
		#if DEBUG
		printk("cmd access_ok error\n");
		#endif
		return -EFAULT;
	}
	/* check bam150_client */
	if( bma150_client == NULL)
	{
		#if DEBUG
		printk("I2C driver not install\n"); 
		#endif
		return -EFAULT;
	}
	
	/* cmd mapping */

	switch(cmd)
	{
	case BMA150_SOFT_RESET:
		err = smb380_soft_reset();
		return err;

	case BMA150_GET_OFFSET:
		if(copy_from_user((unsigned short*)data,(unsigned short*)arg,4)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_get_offset(*((unsigned short*)data),(unsigned short*)(data+2));
		if(copy_to_user((unsigned short*)arg,(unsigned short*)data,4)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_OFFSET:
		if(copy_from_user((unsigned short*)data,(unsigned short*)arg,4)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_offset(*((unsigned short*)data),*(unsigned short*)(data+2));
		return err;

	case BMA150_SELFTEST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_selftest(*data);
		return err;

	case BMA150_SET_RANGE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_range(*data);
		return err;

	case BMA150_GET_RANGE:
		err = smb380_get_range(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_mode(*data);
		return err;

	case BMA150_GET_MODE:
		err = smb380_get_mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_BANDWIDTH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_bandwidth(*data);
		return err;

	case BMA150_GET_BANDWIDTH:
		err = smb380_get_bandwidth(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_WAKE_UP_PAUSE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_wake_up_pause(*data);
		return err;

	case BMA150_GET_WAKE_UP_PAUSE:
		err = smb380_get_wake_up_pause(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_threshold(*data);
		return err;

	case BMA150_GET_LOW_G_THRESHOLD:
		err = smb380_get_low_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_countdown(*data);
		return err;

	case BMA150_GET_LOW_G_COUNTDOWN:
		err = smb380_get_low_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_COUNTDOWN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_countdown(*data);
		return err;

	case BMA150_GET_HIGH_G_COUNTDOWN:
		err = smb380_get_high_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG			
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_duration(*data);
		return err;

	case BMA150_GET_LOW_G_DURATION:
		err = smb380_get_low_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_threshold(*data);
		return err;

	case BMA150_GET_HIGH_G_THRESHOLD:
		err = smb380_get_high_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_DURATION:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_duration(*data);
		return err;

	case BMA150_GET_HIGH_G_DURATION:
		err = smb380_get_high_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_ANY_MOTION_THRESHOLD:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_threshold(*data);
		return err;

	case BMA150_GET_ANY_MOTION_THRESHOLD:
		err = smb380_get_any_motion_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_ANY_MOTION_COUNT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_count(*data);
		return err;

	case BMA150_GET_ANY_MOTION_COUNT:
		err = smb380_get_any_motion_count(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_INTERRUPT_MASK:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_interrupt_mask(*data);
		return err;

	case BMA150_GET_INTERRUPT_MASK:
		err = smb380_get_interrupt_mask(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_RESET_INTERRUPT:
		err = smb380_reset_interrupt();
		return err;

	case BMA150_READ_ACCEL_X:
		err = smb380_read_accel_x((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_READ_ACCEL_Y:
		err = smb380_read_accel_y((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_READ_ACCEL_Z:
		err = smb380_read_accel_z((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_GET_INTERRUPT_STATUS:
		err = smb380_get_interrupt_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_int(*data);
		return err;

	case BMA150_SET_HIGH_G_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_int(*data);
		return err;

	case BMA150_SET_ANY_MOTION_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_int(*data);
		return err;

	case BMA150_SET_ALERT_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_alert_int(*data);
		return err;

	case BMA150_SET_ADVANCED_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("[BMA150] copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_advanced_int(*data);
		return err;

	case BMA150_LATCH_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_latch_int(*data);
		return err;

	case BMA150_SET_NEW_DATA_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_new_data_int(*data);
		return err;

	case BMA150_GET_LOW_G_HYST:
		err = smb380_get_low_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_LOW_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_hysteresis(*data);
		return err;

	case BMA150_GET_HIGH_G_HYST:
		err = smb380_get_high_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_SET_HIGH_G_HYST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_hysteresis(*data);
		return err;

	case BMA150_READ_ACCEL_XYZ:
		err = smb380_read_accel_xyz((smb380acc_t*)data);
		if(copy_to_user((smb380acc_t*)arg,(smb380acc_t*)data,6)!=0)
		{
			#if DEBUG
			printk("copy_to error\n");
			#endif
			return -EFAULT;
		}
		return err;

	case BMA150_READ_TEMPERATURE:
		err = smb380_read_temperature(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		return err;
	default:
		return 0;
	}
}


static const struct file_operations bma150_fops = {
	.owner = THIS_MODULE,
	.read = bma150_read,
	.write = bma150_write,
	.open = bma150_open,
	.release = bma150_close,
	.ioctl = bma150_ioctl,
};

int bma150_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
	struct bma150_data *data;
	int err = 0;
	int tempvalue;
	
	#if DEBUG
	printk(KERN_INFO "[BMA150] %s\n", __FUNCTION__);
	#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet. */
	if (!(data = kmalloc(sizeof(struct bma150_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct bma150_data));
	i2c_set_clientdata(client, data);
	bma150_client = client;

	if (i2c_smbus_read_byte(client) < 0)
	{
		printk("i2c_smbus_read_byte error!!\n");
		goto exit_kfree;
	}
	else
	{
		printk("[BMA150] Bosch Sensortec Device detected!\n");
	}

	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, 0x00);
	if((tempvalue&0x00FF) == 0x0002)
	{
		printk("[BMA150] BMA150/SMB380 registered I2C driver!\n");
	}
	else
	{
		printk("BMA150/SMB380 not registered 0x%x!\n", tempvalue);
		i2c_detach_client(bma150_client);
		bma150_client = NULL;
		goto exit_kfree;
	}
	
	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int bma150_detect(struct i2c_client *client, int kind,
			 struct i2c_board_info *info)
{
	strlcpy(info->type, "bma150", I2C_NAME_SIZE);
	return 0;
}

static int bma150_remove(struct i2c_client *client)
{
	int err;
	struct bma150_data *bma = i2c_get_clientdata(client);
#if DEBUG
	printk(KERN_INFO "[BMA150] driver: remove\n");
#endif
	if ((err = i2c_detach_client(client)))
	{
		dev_err(&client->dev,"Client deregistration failed, client can not be detached!\n");
	}

	kfree(bma);
	bma150_client = NULL;

	return 0;
}


int bma150_resume(void)
{
#if DEBUG
	printk("[BMA150] %s requested\n",__func__);
#endif 
    smb380_set_mode(SMB380_MODE_NORMAL);
 
    return 0;
}
 
int bma150_suspend(void)
{
#if DEBUG
	printk("[BMA150] %s requested\n",__func__);
#endif 
    smb380_set_mode(SMB380_MODE_SLEEP);
 
    return 0;
}

static struct i2c_driver bma150_driver = {
    .class = I2C_CLASS_HWMON,
	.probe = bma150_probe,
	.remove = bma150_remove,
	.id_table = bma150_id,
	.driver = {
                .owner = THIS_MODULE,
		.name	= "bma150",
	},
	.detect	= bma150_detect,
    .address_data = &addr_data,
	.suspend = bma150_suspend,
	.resume  = bma150_resume,

};

static int __init bma150_init(void)
{
	printk(KERN_INFO "===== [BMA150] acceleration sensor driver =====\n");
 	printk("[BMA150] %s\n", __func__ );
	
	int res;
	struct device *dev;
	/* register a char dev	*/
	res = register_chrdev(BMA150_MAJOR, "BMA150", &bma150_fops);
	if (res)
		goto out;
	/* create BMA-dev device class */
	bma_dev_class = class_create(THIS_MODULE, "BMA-dev");
	if (IS_ERR(bma_dev_class)) {
		res = PTR_ERR(bma_dev_class);
		goto out_unreg_chrdev;
	}
	/* add i2c driver for bma150 */
	res = i2c_add_driver(&bma150_driver);

	if (res)
		goto out_unreg_class;
	/* create device node for bma150 */
	dev = device_create(bma_dev_class, NULL,
				     MKDEV(BMA150_MAJOR, 0),
				     NULL,
				     "bma150");
	if (IS_ERR(dev)) {
		res = PTR_ERR(dev);
		goto error_destroy;
	}

	smb380.bus_write = bma150_i2c_write;
	smb380.bus_read = bma150_i2c_read;
	
	smb380_init(&smb380);
    smb380_set_mode(SMB380_MODE_SLEEP); // to mode_sleep
	printk(KERN_INFO "[BMA150] %s end! BMA150 device create ok\n", __func__);

	return 0;

error_destroy:
	i2c_del_driver(&bma150_driver);
out_unreg_class:
	class_destroy(bma_dev_class);
out_unreg_chrdev:
	unregister_chrdev(BMA150_MAJOR, "BMA150");
out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
	return res;
}

static void __exit bma150_exit(void)
{
	i2c_del_driver(&bma150_driver);
	class_destroy(bma_dev_class);
	unregister_chrdev(BMA150_MAJOR,"BMA150");
	printk(KERN_ERR "BMA150 exit\n");
}


MODULE_AUTHOR("Bin Du <bin.du@cn.bosch.com>");
MODULE_DESCRIPTION("BMA150 driver");
MODULE_LICENSE("GPL");

module_init(bma150_init);
module_exit(bma150_exit);

EXPORT_SYMBOL(bma150_suspend);
EXPORT_SYMBOL(bma150_resume);


/* drivers/char/diag/diagchar_core.c */

/* Copyright (c) 2008 QUALCOMM USA, INC. 
 *  
 * All source code in this file is licensed under the following license
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org 
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/diagchar.h>
#include <linux/sched.h>
#include <mach/usbdiag.h>
#include <asm/current.h>
#include "diagchar_hdlc.h"
#include "diagfwd.h"
#include "diagmem.h"
#include "diagchar.h"


MODULE_DESCRIPTION("Diag Char Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");

struct diagchar_dev *driver;

/* The following variables can be specified by module options */
static unsigned int itemsize = 2512; /*Size of item in the mempool*/
static unsigned int poolsize = 10;  /*Number of items in the mempool*/
/* This is the maximum number of user-space clients supported */
static unsigned int max_clients = 5;

module_param(itemsize, uint, 0);
module_param(poolsize, uint, 0);
module_param(max_clients, uint, 0);

static int diagchar_open(struct inode *inode, struct file *file)
{
	int i = 0;

	if (driver) {
		mutex_lock(&driver->diagchar_mutex);

		for (i = 0; i < driver->num_clients; i++)
			if (driver->client_map[i] == 0)
				break;

		if (i < driver->num_clients)
			driver->client_map[i] = current->tgid;
		else
			return -ENOMEM;

		driver->data_ready[i] |= MSG_MASKS_TYPE;
		driver->data_ready[i] |= EVENT_MASKS_TYPE;
		driver->data_ready[i] |= LOG_MASKS_TYPE;

		if (driver->ref_count == 0)
			diagmem_init(driver);
		driver->ref_count++;
		mutex_unlock(&driver->diagchar_mutex);
		return 0;
	}
	return -ENOMEM;
}

static int diagchar_close(struct inode *inode, struct file *file)
{
	int i = 0;

	/* Delete the pkt response table entry for the exiting process */
	for (i = 0; i < REG_TABLE_SIZE; i++)
			if (driver->table[i].process_id == current->tgid)
					driver->table[i].process_id = 0;

			if (driver) {
				mutex_lock(&driver->diagchar_mutex);
				driver->ref_count--;
				if (driver->ref_count == 0)
					diagmem_exit(driver);

				for (i = 0; i < driver->num_clients; i++)
					if (driver->client_map[i] ==
					     current->tgid) {
						driver->client_map[i] = 0;
						break;
					}
		mutex_unlock(&driver->diagchar_mutex);
		return 0;
	}
	return -ENOMEM;
}

static int diagchar_ioctl(struct inode *inode, struct file *filp,
			   unsigned int iocmd, unsigned long ioarg)
{
	int i, count_entries = 0;
	struct bindpkt_params_per_process *pkt_params =
			 (struct bindpkt_params_per_process *) ioarg;

	for (i = 0; i < REG_TABLE_SIZE; i++) {
		if (driver->table[i].process_id == 0) {
			driver->table[i].cmd_code =
				 pkt_params->params->cmd_code;
			driver->table[i].subsys_id =
				 pkt_params->params->subsys_id;
			driver->table[i].cmd_code_lo =
				 pkt_params->params->cmd_code_hi;
			driver->table[i].cmd_code_hi =
				 pkt_params->params->cmd_code_lo;
			driver->table[i].process_id = current->tgid;
			count_entries++;
			if (pkt_params->count > count_entries)
					pkt_params->params++;
			else
					break;
		}
	}

	return -EINVAL;
}

static int diagchar_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	int index = -1, i = 0, ret = 0;
	int data_type;
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i] == current->tgid)
			index = i;

	if (index == -1)
		return -EINVAL;

	wait_event_interruptible(driver->wait_q,
				  driver->data_ready[index]);
	mutex_lock(&driver->diagchar_mutex);

	if (driver->data_ready[index] & MSG_MASKS_TYPE) {
		/*Copy the type of data being passed*/
		data_type = driver->data_ready[index] & MSG_MASKS_TYPE;
		if (copy_to_user(buf, (void *)&data_type, 4)) {
			ret = -EFAULT;
			goto exit;
		}
		ret += 4;

		if (copy_to_user(buf+4, (void *)driver->msg_masks,
				  MSG_MASK_SIZE)) {
			ret =  -EFAULT;
			goto exit;
		}
		ret += MSG_MASK_SIZE;
		driver->data_ready[index] ^= MSG_MASKS_TYPE;
		goto exit;
	}

	if (driver->data_ready[index] & EVENT_MASKS_TYPE) {
		/*Copy the type of data being passed*/
		data_type = driver->data_ready[index] & EVENT_MASKS_TYPE;
		if (copy_to_user(buf, (void *)&data_type, 4)) {
			ret = -EFAULT;
			goto exit;
		}
		ret += 4;
		if (copy_to_user(buf+4, (void *)driver->event_masks,
				  EVENT_MASK_SIZE)) {
			ret = -EFAULT;
			goto exit;
		}
		ret += EVENT_MASK_SIZE;
		driver->data_ready[index] ^= EVENT_MASKS_TYPE;
		goto exit;
	}

	if (driver->data_ready[index] & LOG_MASKS_TYPE) {
		/*Copy the type of data being passed*/
		data_type = driver->data_ready[index] & LOG_MASKS_TYPE;
		if (copy_to_user(buf, (void *)&data_type, 4)) {
			ret = -EFAULT;
			goto exit;
		}
		ret += 4;

		if (copy_to_user(buf+4, (void *)driver->log_masks,
				 LOG_MASK_SIZE)) {
			ret = -EFAULT;
			goto exit;
		}
		ret += LOG_MASK_SIZE;
		driver->data_ready[index] ^= LOG_MASKS_TYPE;
		goto exit;
	}

	if (driver->data_ready[index] & PKT_TYPE) {
		/*Copy the type of data being passed*/
		data_type = driver->data_ready[index] & PKT_TYPE;
		if (copy_to_user(buf, (void *)&data_type, 4)) {
			ret = -EFAULT;
			goto exit;
		}
		ret += 4;

		if (copy_to_user(buf+4, (void *)driver->pkt_buf,
				 driver->pkt_length)) {
			ret = -EFAULT;
			goto exit;
		}
		ret += driver->pkt_length;
		driver->data_ready[index] ^= PKT_TYPE;
		goto exit;
	}

exit:
	mutex_unlock(&driver->diagchar_mutex);
	return ret;
}

static int diagchar_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	int err;
	int used, ret = 0;
#ifdef DIAG_DEBUG
	int length = 0, i;
#endif
	struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
	struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };
	void *buf_copy;
	void *buf_hdlc;
	int payload_size;

	if (!driver->usb_connected) {
		/*Drop the diag payload */
		return -EIO;
	}

	/*First 4 bytes indicate the type of payload - ignore these */
	payload_size = count - 4;

	buf_copy = diagmem_alloc(driver, payload_size);
	if (!buf_copy) {
		driver->dropped_count++;
		return -ENOMEM;
	}

	err = copy_from_user(buf_copy, buf + 4, payload_size);
	if (err) {
		printk(KERN_INFO "diagchar : copy_from_user failed \n");
		ret = -EFAULT;
		goto fail_free_copy;
	}
#ifdef DIAG_DEBUG
	printk(KERN_DEBUG "data is --> \n");
	for (i = 0; i < payload_size; i++)
		printk(KERN_DEBUG "\t %x \t", *(((unsigned char *)buf_copy)+i));
#endif
	send.state = DIAG_STATE_START;
	send.pkt = buf_copy;
	send.last = (void *)(buf_copy + payload_size - 1);
	send.terminate = 1;

	/*Allocate a buffer for CRC + HDLC framed output to USB */
	buf_hdlc = diagmem_alloc(driver, payload_size + 8);
	if (!buf_hdlc) {
		driver->dropped_count++;
		ret = -ENOMEM;
		goto fail_free_copy;
	}

	enc.dest = buf_hdlc;
	enc.dest_last = (void *)(buf_hdlc + payload_size + 7);

	diag_hdlc_encode(&send, &enc);

	used = (uint32_t) enc.dest - (uint32_t) buf_hdlc;

	diagmem_free(driver, buf_copy);
#ifdef DIAG_DEBUG
	printk(KERN_DEBUG "hdlc encoded data is --> \n");
	for (i = 0; i < payload_size + 8; i++) {
		printk(KERN_DEBUG "\t %x \t", *(((unsigned char *)buf_hdlc)+i));
		if (*(((unsigned char *)buf_hdlc)+i) != 0x7e)
			length++;
	}
#endif
	err = diag_write(buf_hdlc, used);
	if (err) {
		/*Free the buffer right away if write failed */
		ret = -EIO;
		goto fail_free_hdlc;
	}

	return 0;

fail_free_hdlc:
	diagmem_free(driver, buf_hdlc);
	return ret;

fail_free_copy:
	diagmem_free(driver, buf_copy);
	return ret;
}

static const struct file_operations diagcharfops = {
	.owner = THIS_MODULE,
	.read = diagchar_read,
	.write = diagchar_write,
	.ioctl = diagchar_ioctl,
	.open = diagchar_open,
	.release = diagchar_close
};

static int diagchar_setup_cdev(dev_t devno)
{

	int err;

	cdev_init(driver->cdev, &diagcharfops);

	driver->cdev->owner = THIS_MODULE;
	driver->cdev->ops = &diagcharfops;

	err = cdev_add(driver->cdev, devno, 1);

	if (err) {
		printk(KERN_INFO "diagchar cdev registration failed !\n\n");
		return -1;
	}

	driver->diagchar_class = class_create(THIS_MODULE, "diag");

	if (IS_ERR(driver->diagchar_class)) {
		printk(KERN_ERR "Error creating diagchar class.\n");
		return -1;
	}

	device_create(driver->diagchar_class, NULL, devno,
				  (void *)driver, "diag");

	return 0;

}

static int diagchar_cleanup(void)
{
	if (driver) {
		if (driver->cdev) {
			/* TODO - Check if device exists before deleting */
			device_destroy(driver->diagchar_class,
				       MKDEV(driver->major,
					     driver->minor_start));
			cdev_del(driver->cdev);
		}
		if (!IS_ERR(driver->diagchar_class))
			class_destroy(driver->diagchar_class);
		kfree(driver);
	}
	return 0;
}

static int __init diagchar_init(void)
{
	dev_t dev;
	int error;

	printk(KERN_INFO "diagfwd initializing ..\n");
	driver = kzalloc(sizeof(struct diagchar_dev) + 5, GFP_KERNEL);

	if (driver) {
		driver->itemsize = itemsize;
		driver->poolsize = poolsize;
		driver->num_clients = max_clients;
		mutex_init(&driver->diagchar_mutex);
		init_waitqueue_head(&driver->wait_q);
		diagfwd_init();

		printk(KERN_INFO "diagchar initializing ..\n");
		driver->num = 1;
		driver->name = ((void *)driver) + sizeof(struct diagchar_dev);
		strlcpy(driver->name, "diag", 4);

		/* Get major number from kernel and initialize */
		error = alloc_chrdev_region(&dev, driver->minor_start,
					    driver->num, driver->name);
		if (!error) {
			driver->major = MAJOR(dev);
			driver->minor_start = MINOR(dev);
		} else {
			printk(KERN_INFO "Major number not allocated \n");
			goto fail;
		}
		driver->cdev = cdev_alloc();
		error = diagchar_setup_cdev(dev);
		if (error)
			goto fail;
	} else {
		printk(KERN_INFO "kzalloc failed\n");
		goto fail;
	}

	printk(KERN_INFO "diagchar initialized\n");
	return 0;

fail:
	diagchar_cleanup();
	diagfwd_exit();
	return -1;

}

static void __exit diagchar_exit(void)
{
	printk(KERN_INFO "diagchar exiting ..\n");
	if (driver->ref_count)
		diagmem_exit(driver);
	diagfwd_exit();
	diagchar_cleanup();
	printk(KERN_INFO "done diagchar exit\n");
}

module_init(diagchar_init);
module_exit(diagchar_exit);

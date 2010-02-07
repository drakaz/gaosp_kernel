/*
 *
 * Copyright (c) 2009 QUALCOMM USA, INC.
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
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/msm_audio.h>

#include <asm/ioctls.h>
#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>
#include <mach/qdsp6/msm8k_ard.h>
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_cad_write_amr_format.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_voice: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

#define MSM8K_VOICE_PROC_NAME "msm8k_voice"

#define AUDIO_MAGIC 'a'

struct voice {
	u32 cad_r_handle;
	u32 cad_w_handle;
};

struct voice g_voice;

static int msm8k_voice_open(struct inode *inode, struct file *f)
{
	struct voice *voice = &g_voice;
	struct cad_open_struct_type  cos;
	struct cad_stream_info_struct_type cad_stream_info;
	struct cad_stream_device_struct_type cad_stream_dev;
	struct cad_write_amr_format_struct_type cad_write_amr_fmt;
	u32 stream_device[1];
	int rc;
	D("%s\n", __func__);

	memset(&cad_stream_info, 0, sizeof(struct cad_stream_info_struct_type));
	memset(&cad_write_amr_fmt, 0,
			sizeof(struct cad_write_amr_format_struct_type));

	f->private_data = voice;

	cos.op_code = CAD_OPEN_OP_WRITE;
	voice->cad_w_handle = cad_open(&cos);

	if (voice->cad_w_handle == 0)
		return CAD_RES_FAILURE;

	cad_stream_info.app_type = CAD_STREAM_APP_VOICE;
	cad_stream_info.priority = 0;
	cad_stream_info.buf_mem_type = CAD_STREAM_BUF_MEM_HEAP;
	cad_stream_info.ses_buf_max_size = 1024;
	rc = cad_ioctl(voice->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_INFO,
		&cad_stream_info,
		sizeof(struct cad_stream_info_struct_type));
	if (rc) {
		pr_err("cad_ioctl() CAD_IOCTL_CMD_SET_STREAM_INFO failed\n");
		return CAD_RES_FAILURE;
	}

	stream_device[0] = CAD_HW_DEVICE_ID_DEFAULT_RX;
	cad_stream_dev.device = (u32 *)&stream_device[0];
	cad_stream_dev.device_len = 1;
	rc = cad_ioctl(voice->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_DEVICE,
		&cad_stream_dev,
		sizeof(struct cad_stream_device_struct_type));
	if (rc) {
		pr_err("cad_ioctl() CAD_IOCTL_CMD_SET_STREAM_DEVICE failed\n");
		return CAD_RES_FAILURE;
	}

	rc = cad_ioctl(voice->cad_w_handle, CAD_IOCTL_CMD_STREAM_START,
		NULL, 0);
	if (rc) {
		pr_err("cad_ioctl() CAD_IOCTL_CMD_STREAM_START failed\n");
		return CAD_RES_FAILURE;
	}

	cos.op_code = CAD_OPEN_OP_READ;
	voice->cad_r_handle = cad_open(&cos);

	if (voice->cad_r_handle == 0)
		return CAD_RES_FAILURE;

	cad_stream_info.app_type = CAD_STREAM_APP_VOICE;
	cad_stream_info.priority = 0;
	cad_stream_info.buf_mem_type = CAD_STREAM_BUF_MEM_HEAP;
	cad_stream_info.ses_buf_max_size = 1024;
	rc = cad_ioctl(voice->cad_r_handle, CAD_IOCTL_CMD_SET_STREAM_INFO,
		&cad_stream_info,
		sizeof(struct cad_stream_info_struct_type));
	if (rc) {
		pr_err("cad_ioctl() CAD_IOCTL_CMD_SET_STREAM_INFO failed\n");
		return CAD_RES_FAILURE;
	}

	stream_device[0] = CAD_HW_DEVICE_ID_DEFAULT_TX;
	cad_stream_dev.device = (u32 *)&stream_device[0];
	cad_stream_dev.device_len = 1;
	rc = cad_ioctl(voice->cad_r_handle, CAD_IOCTL_CMD_SET_STREAM_DEVICE,
		&cad_stream_dev,
		sizeof(struct cad_stream_device_struct_type));
	if (rc) {
		pr_err("cad_ioctl() CAD_IOCTL_CMD_SET_STREAM_DEVICE failed\n");
		return CAD_RES_FAILURE;
	}

	rc = cad_ioctl(voice->cad_r_handle, CAD_IOCTL_CMD_STREAM_START,
		NULL, 0);
	if (rc) {
		pr_err("cad_ioctl() CAD_IOCTL_CMD_STREAM_START failed\n");
		return CAD_RES_FAILURE;
	}

	return rc;
}

static int msm8k_voice_release(struct inode *inode, struct file *f)
{
	int rc = CAD_RES_SUCCESS;
	struct voice *voice = f->private_data;
	D("%s\n", __func__);

	cad_close(voice->cad_w_handle);
	cad_close(voice->cad_r_handle);

	return rc;
}

static ssize_t msm8k_voice_read(struct file *f, char __user *buf, size_t cnt,
		loff_t *pos)
{
	D("%s\n", __func__);
	return -EINVAL;
}

static ssize_t msm8k_voice_write(struct file *f, const char __user *buf,
		size_t cnt, loff_t *pos)
{
	struct cad_buf_struct_type cbs;
	struct voice *voice = f->private_data;

	D("%s\n", __func__);

	memset(&cbs, 0, sizeof(struct cad_buf_struct_type));
	cbs.buffer = (void *)buf;
	cbs.phys_addr = 0;
	cbs.max_size = cnt;

	cad_write(voice->cad_w_handle, &cbs);

	return cnt;
}

static int msm8k_voice_ioctl(struct inode *inode, struct file *f,
		unsigned int cmd, unsigned long arg)
{
	int rc;
	struct voice *voice = f->private_data;
	struct cad_stream_info_struct_type cad_stream_info;
	D("%s\n", __func__);

	memset(&cad_stream_info, 0, sizeof(struct cad_stream_info_struct_type));

	switch (cmd) {
	case AUDIO_START:
	case AUDIO_STOP:
	case AUDIO_FLUSH:
		rc = CAD_RES_SUCCESS;
		break;
	case AUDIO_GET_CONFIG:
		rc = CAD_RES_SUCCESS;
		break;
	case AUDIO_SET_CONFIG:
		rc = cad_ioctl(voice->cad_w_handle,
			CAD_IOCTL_CMD_SET_STREAM_INFO,
			&cad_stream_info,
			sizeof(struct cad_stream_info_struct_type));
		if (rc)
			pr_err("cad_ioctl() CAD_IOCTL_CMD_SET_STREAM_INFO"
					" failed\n");

		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

#ifdef CONFIG_PROC_FS
int msm8k_voice_read_proc(char *pbuf, char **start, off_t offset,
			int count, int *eof, void *data)
{
	int len = 0;
	len += snprintf(pbuf, 16, "voice\n");

	*eof = 1;
	return len;
}
#endif

static const struct file_operations msm8k_voice_fops = {
	.owner = THIS_MODULE,
	.open = msm8k_voice_open,
	.release = msm8k_voice_release,
	.read = msm8k_voice_read,
	.write = msm8k_voice_write,
	.ioctl = msm8k_voice_ioctl,
	.llseek = no_llseek,
};


struct miscdevice msm8k_voice_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_voice",
	.fops	= &msm8k_voice_fops,
};

static int __init msm8k_voice_init(void)
{
	int rc;
	D("%s\n", __func__);

	rc = misc_register(&msm8k_voice_misc);

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(MSM8K_VOICE_PROC_NAME,
			0, NULL, msm8k_voice_read_proc, NULL);
#endif

	return rc;
}

static void __exit msm8k_voice_exit(void)
{
	D("%s\n", __func__);
#ifdef CONFIG_PROC_FS
	remove_proc_entry(MSM8K_VOICE_PROC_NAME, NULL);
#endif
}


module_init(msm8k_voice_init);
module_exit(msm8k_voice_exit);

MODULE_DESCRIPTION("MSM Voice driver");
MODULE_LICENSE("GPL v2");


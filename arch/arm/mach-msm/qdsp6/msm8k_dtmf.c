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
#include <mach/qdsp6/msm8k_cad_write_pcm_format.h>
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_cad_volume.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_dtmf: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

#define MSM8K_DTMF_PROC_NAME "msm8k_dtmf"

#define AUDIO_MAGIC 'a'

struct dtmf {
	u32 cad_w_handle;
};

struct dtmf g_dtmf;

static int msm8k_dtmf_open(struct inode *inode, struct file *f)
{
	struct dtmf *dtmf = &g_dtmf;
	struct cad_open_struct_type  cos;
	D("%s\n", __func__);

	cos.format = CAD_FORMAT_DTMF;

	f->private_data = dtmf;

	cos.op_code = CAD_OPEN_OP_WRITE;
	dtmf->cad_w_handle = cad_open(&cos);

	if (dtmf->cad_w_handle == 0)
		return CAD_RES_FAILURE;
	else
		return CAD_RES_SUCCESS;
}

static int msm8k_dtmf_release(struct inode *inode, struct file *f)
{
	int rc = CAD_RES_SUCCESS;
	struct dtmf *dtmf = f->private_data;
	D("%s\n", __func__);

	cad_close(dtmf->cad_w_handle);

	return rc;
}

static ssize_t msm8k_dtmf_read(struct file *f, char __user *buf, size_t cnt,
		loff_t *pos)
{
	D("%s\n", __func__);
	return -EINVAL;
}

static ssize_t msm8k_dtmf_write(struct file *f, const char __user *buf,
		size_t cnt, loff_t *pos)
{
	D("%s\n", __func__);
	return -EINVAL;
}

static int msm8k_dtmf_ioctl(struct inode *inode, struct file *f,
		unsigned int cmd, unsigned long arg)
{
	int rc = CAD_RES_SUCCESS;
	struct dtmf *p = f->private_data;
	void *cad_arg = (void *)arg;

	struct cad_stream_device_struct_type	cad_stream_dev;
	struct cad_stream_info_struct_type	cad_stream_info;

	D("%s\n", __func__);

	memset(&cad_stream_dev, 0,
			sizeof(struct cad_stream_device_struct_type));
	memset(&cad_stream_info, 0, sizeof(struct cad_stream_info_struct_type));

	switch (cmd) {
	case AUDIO_PLAY_DTMF:
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_GEN_DTMF,
			cad_arg, sizeof(cad_arg));
		if (rc) {
			pr_err("cad_ioctl() CMD_GEN_DTMF failed\n");
			break;
		}
		break;
	case AUDIO_START:
		cad_stream_dev.device = cad_arg;
		cad_stream_dev.device_len = 1;

		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_DEVICE,
			&cad_stream_dev, sizeof(cad_stream_dev));
		if (rc) {
			pr_err("cad_ioctl() SET_STREAM_DEVICE failed\n");
			break;
		}

		cad_stream_info.app_type = CAD_STREAM_APP_DTMF;

		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_INFO,
			&cad_stream_info, sizeof(cad_stream_info));
		if (rc) {
			pr_err("cad_ioctl() SET_STREAM_INFO failed\n");
			break;
		}


		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_START,
			NULL, 0);
		if (rc) {
			pr_err("cad_ioctl() CMD_STREAM_START failed\n");
			break;
		}

		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

#ifdef CONFIG_PROC_FS
int msm8k_dtmf_read_proc(char *pbuf, char **start, off_t offset,
			int count, int *eof, void *data)
{
	int len = 0;
	len += snprintf(pbuf, 16, "dtmf\n");

	*eof = 1;
	return len;
}
#endif

static const struct file_operations msm8k_dtmf_fops = {
	.owner = THIS_MODULE,
	.open = msm8k_dtmf_open,
	.release = msm8k_dtmf_release,
	.read = msm8k_dtmf_read,
	.write = msm8k_dtmf_write,
	.ioctl = msm8k_dtmf_ioctl,
	.llseek = no_llseek,
};


struct miscdevice msm8k_dtmf_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_dtmf",
	.fops	= &msm8k_dtmf_fops,
};

static int __init msm8k_dtmf_init(void)
{
	int rc;
	D("%s\n", __func__);

	rc = misc_register(&msm8k_dtmf_misc);


#ifdef CONFIG_PROC_FS
	create_proc_read_entry(MSM8K_DTMF_PROC_NAME,
			0, NULL, msm8k_dtmf_read_proc, NULL);
#endif

	return rc;
}

static void __exit msm8k_dtmf_exit(void)
{
	D("%s\n", __func__);
#ifdef CONFIG_PROC_FS
	remove_proc_entry(MSM8K_DTMF_PROC_NAME, NULL);
#endif
}


module_init(msm8k_dtmf_init);
module_exit(msm8k_dtmf_exit);

MODULE_DESCRIPTION("MSM DTMF driver");
MODULE_LICENSE("GPL v2");


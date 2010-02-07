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
#include <mach/qdsp6/msm8k_cad_write_amr_format.h>
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_cad_volume.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_amr: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

#define MSM8K_AMR_PROC_NAME "msm8k_amr"

#define AUDIO_MAGIC 'a'

struct amr {
	u32 cad_w_handle;
	struct msm_audio_config cfg;
	u32 volume;
};

struct amr g_amr;

static int msm8k_amr_open(struct inode *inode, struct file *f)
{
	struct amr *amr = &g_amr;
	struct cad_open_struct_type  cos;
	D("%s\n", __func__);

	cos.format = CAD_FORMAT_AMRNB;

	f->private_data = amr;

	cos.op_code = CAD_OPEN_OP_WRITE;
	amr->cad_w_handle = cad_open(&cos);

	if (amr->cad_w_handle == 0)
		return CAD_RES_FAILURE;
	else
		return CAD_RES_SUCCESS;
}

static int msm8k_amr_release(struct inode *inode, struct file *f)
{
	int rc = CAD_RES_SUCCESS;
	struct amr *amr = f->private_data;
	D("%s\n", __func__);

	cad_close(amr->cad_w_handle);

	return rc;
}

static ssize_t msm8k_amr_read(struct file *f, char __user *buf, size_t cnt,
		loff_t *pos)
{
	D("%s\n", __func__);
	return -EINVAL;
}

static ssize_t msm8k_amr_write(struct file *f, const char __user *buf,
		size_t cnt, loff_t *pos)
{
	struct cad_buf_struct_type cbs;
	struct amr *amr = f->private_data;

	D("%s\n", __func__);

	memset(&cbs, 0, sizeof(struct cad_buf_struct_type));
	cbs.buffer = (void *)buf;
	cbs.phys_addr = 0;
	cbs.max_size = cnt;
	cbs.actual_size = cnt;

	cad_write(amr->cad_w_handle, &cbs);

	return cnt;
}

static int msm8k_amr_ioctl(struct inode *inode, struct file *f,
		unsigned int cmd, unsigned long arg)
{
	int rc = CAD_RES_SUCCESS;
	struct amr *p = f->private_data;
	void *cad_arg = (void *)arg;
	u32 stream_device[1];
	struct cad_device_struct_type cad_dev;
	struct cad_stream_device_struct_type cad_stream_dev;
	struct cad_stream_info_struct_type cad_stream_info;
	struct cad_write_amr_format_struct_type cad_write_amr_fmt;
	struct cad_flt_cfg_strm_vol cad_strm_volume;
	struct cad_stream_filter_struct_type cad_stream_filter;

	D("%s\n", __func__);

	memset(&cad_dev, 0, sizeof(struct cad_device_struct_type));
	memset(&cad_stream_dev, 0,
			sizeof(struct cad_stream_device_struct_type));
	memset(&cad_stream_info, 0, sizeof(struct cad_stream_info_struct_type));
	memset(&cad_write_amr_fmt, 0,
			sizeof(struct cad_write_amr_format_struct_type));
	memset(&cad_stream_filter, 0,
			sizeof(struct cad_stream_filter_struct_type));

	switch (cmd) {
	case AUDIO_START:

		cad_stream_info.app_type = CAD_STREAM_APP_PLAYBACK;
		cad_stream_info.priority = 0;
		cad_stream_info.buf_mem_type = CAD_STREAM_BUF_MEM_HEAP;
		cad_stream_info.ses_buf_max_size = 1024 * 10;
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_INFO,
			&cad_stream_info,
			sizeof(struct cad_stream_info_struct_type));
		if (rc) {
			pr_err("cad_ioctl() SET_STREAM_INFO failed\n");
			break;
		}

		stream_device[0] = CAD_HW_DEVICE_ID_DEFAULT_RX;
		cad_stream_dev.device = (u32 *)&stream_device[0];
		cad_stream_dev.device_len = 1;
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_DEVICE,
			&cad_stream_dev,
			sizeof(struct cad_stream_device_struct_type));
		if (rc) {
			pr_err("cad_ioctl() SET_STREAM_DEVICE failed\n");
			break;
		}

		cad_write_amr_fmt.ver_id = CAD_WRITE_AMR_VERSION_10;
		cad_write_amr_fmt.amr.sample_rate = 48000;
		cad_write_amr_fmt.amr.stereo_config = 1;

		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_SET_STREAM_CONFIG,
			&cad_write_amr_fmt,
			sizeof(struct cad_write_amr_format_struct_type));
		if (rc) {
			pr_err("cad_ioctl() SET_STREAM_CONFIG failed\n");
			break;
		}

		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_START,
			NULL, 0);
		if (rc) {
			pr_err("cad_ioctl() STREAM_START failed\n");
			break;
		}
		break;
	case AUDIO_STOP:
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_PAUSE,
			cad_arg, sizeof(u32));
		break;
	case AUDIO_FLUSH:
		rc = cad_ioctl(p->cad_w_handle, CAD_IOCTL_CMD_STREAM_FLUSH,
			cad_arg, sizeof(u32));
		break;
	case AUDIO_GET_CONFIG:
		if (copy_to_user((void *)arg, &p->cfg,
				sizeof(struct msm_audio_config)))
			return -EFAULT;
		break;
	case AUDIO_SET_CONFIG:
		rc = copy_from_user(&p->cfg, (void *)arg,
				sizeof(struct msm_audio_config));
		break;
	case AUDIO_SET_VOLUME:
		rc = copy_from_user(&p->volume, (void *)arg, sizeof(u32));

		memset(&cad_strm_volume, 0,
				sizeof(struct cad_flt_cfg_strm_vol));
		cad_strm_volume.volume = p->volume;
		cad_stream_filter.filter_type = CAD_FILTER_CONFIG_STREAM_VOLUME;
		cad_stream_filter.format_block = &cad_strm_volume;
		cad_stream_filter.format_block_len =
			sizeof(struct cad_flt_cfg_strm_vol);

		rc = cad_ioctl(p->cad_w_handle,
			CAD_IOCTL_CMD_SET_STREAM_FILTER_CONFIG,
			&cad_stream_filter,
			sizeof(struct cad_stream_filter_struct_type));
		if (rc) {
			pr_err("cad_ioctl() set volume failed\n");
			break;
		}
		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

#ifdef CONFIG_PROC_FS
int msm8k_amr_read_proc(char *pbuf, char **start, off_t offset,
			int count, int *eof, void *data)
{
	int len = 0;
	len += snprintf(pbuf, 16, "amr\n");

	*eof = 1;
	return len;
}
#endif

static const struct file_operations msm8k_amr_fops = {
	.owner = THIS_MODULE,
	.open = msm8k_amr_open,
	.release = msm8k_amr_release,
	.read = msm8k_amr_read,
	.write = msm8k_amr_write,
	.ioctl = msm8k_amr_ioctl,
	.llseek = no_llseek,
};


struct miscdevice msm8k_amr_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_amr",
	.fops	= &msm8k_amr_fops,
};

static int __init msm8k_amr_init(void)
{
	int rc;
	D("%s\n", __func__);

	rc = misc_register(&msm8k_amr_misc);

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(MSM8K_AMR_PROC_NAME,
			0, NULL, msm8k_amr_read_proc, NULL);
#endif

	return rc;
}

static void __exit msm8k_amr_exit(void)
{
	D("%s\n", __func__);
#ifdef CONFIG_PROC_FS
	remove_proc_entry(MSM8K_AMR_PROC_NAME, NULL);
#endif
}


module_init(msm8k_amr_init);
module_exit(msm8k_amr_exit);

MODULE_DESCRIPTION("MSM AMR driver");
MODULE_LICENSE("GPL v2");


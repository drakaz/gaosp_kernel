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
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_cad_volume.h>


#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_audio_dev_ctrl: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

#define MSM8K_AUDIO_PROC_NAME "msm8k_audio_dev_ctrl"

#define AUDIO_MAGIC 'a'

struct msm8k_audio_dev_ctrl {
	u32 cad_ctrl_handle;
	u32 current_volume;
	int current_rx_device;
	int current_tx_device;
};

struct msm8k_audio_dev_ctrl g_ctrl;

static int msm8k_audio_dev_ctrl_open(struct inode *inode, struct file *f)
{
	struct msm8k_audio_dev_ctrl *ctrl = &g_ctrl;
	D("%s\n", __func__);

	f->private_data = ctrl;

	return CAD_RES_SUCCESS;
}

static int msm8k_audio_dev_ctrl_release(struct inode *inode, struct file *f)
{
	int rc = CAD_RES_SUCCESS;

	D("%s\n", __func__);

	return rc;
}

static ssize_t msm8k_audio_dev_ctrl_read(struct file *f, char __user *buf,
	size_t cnt, loff_t *pos)
{
	D("%s\n", __func__);
	return -EINVAL;
}

static ssize_t msm8k_audio_dev_ctrl_write(struct file *f,
	const char __user *buf, size_t cnt, loff_t *pos)
{
	D("%s\n", __func__);
	return -EINVAL;
}

int audio_switch_device(int new_device)
{
	int rc;
	struct msm8k_audio_dev_ctrl *ctrl = &g_ctrl;
	struct cad_device_struct_type cad_dev;

	D("%s\n", __func__);

	memset(&cad_dev, 0, sizeof(struct cad_device_struct_type));

	switch (new_device) {
	case HANDSET_MIC:
		cad_dev.device = CAD_HW_DEVICE_ID_HANDSET_MIC;
		cad_dev.reserved = CAD_TX_DEVICE;
		break;
	case HANDSET_SPKR:
		cad_dev.device = CAD_HW_DEVICE_ID_HANDSET_SPKR;
		cad_dev.reserved = CAD_RX_DEVICE;
		break;
	case HEADSET_MIC:
		cad_dev.device = CAD_HW_DEVICE_ID_HEADSET_MIC;
		cad_dev.reserved = CAD_TX_DEVICE;
		break;
	case HEADSET_SPKR_MONO:
		cad_dev.device = CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO;
		cad_dev.reserved = CAD_RX_DEVICE;
		break;
	case HEADSET_SPKR_STEREO:
		cad_dev.device = CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO;
		cad_dev.reserved = CAD_RX_DEVICE;
		break;
	case SPKR_PHONE_MIC:
		cad_dev.device = CAD_HW_DEVICE_ID_SPKR_PHONE_MIC;
		cad_dev.reserved = CAD_TX_DEVICE;
		break;
	case SPKR_PHONE_MONO:
		cad_dev.device = CAD_HW_DEVICE_ID_SPKR_PHONE_MONO;
		cad_dev.reserved = CAD_RX_DEVICE;
		break;
	case SPKR_PHONE_STEREO:
		cad_dev.device = CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO;
		cad_dev.reserved = CAD_RX_DEVICE;
		break;
	case BT_SCO_MIC:
		cad_dev.device = CAD_HW_DEVICE_ID_BT_SCO_MIC;
		cad_dev.reserved = CAD_TX_DEVICE;
		break;
	case BT_SCO_SPKR:
		cad_dev.device = CAD_HW_DEVICE_ID_BT_SCO_SPKR;
		cad_dev.reserved = CAD_RX_DEVICE;
		break;
	case BT_A2DP_SPKR:
		cad_dev.device = CAD_HW_DEVICE_ID_BT_A2DP_SPKR;
		cad_dev.reserved = CAD_RX_DEVICE;
		break;
	case TTY_HEADSET_MIC:
		cad_dev.device = CAD_HW_DEVICE_ID_TTY_HEADSET_MIC;
		cad_dev.reserved = CAD_TX_DEVICE;
		break;
	case TTY_HEADSET_SPKR:
		cad_dev.device = CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR;
		cad_dev.reserved = CAD_RX_DEVICE;
		break;
	case I2S_RX:
		cad_dev.device = CAD_HW_DEVICE_ID_I2S_RX;
		cad_dev.reserved = CAD_RX_DEVICE;
		break;
	case I2S_TX:
		cad_dev.device = CAD_HW_DEVICE_ID_I2S_TX;
		cad_dev.reserved = CAD_TX_DEVICE;
		break;
	default:
		return -ENODEV;
	}

	if (cad_dev.reserved == CAD_RX_DEVICE)
		ctrl->current_rx_device = cad_dev.device;
	else
		ctrl->current_tx_device = cad_dev.device;

	rc = cad_ioctl(ctrl->cad_ctrl_handle,
			CAD_IOCTL_CMD_DEVICE_SET_GLOBAL_DEFAULT,
			&cad_dev,
			sizeof(struct cad_device_struct_type));
	if (rc)
		pr_err("cad_ioctl() SET_GLOBAL_DEFAULT failed\n");

	return rc;
}
EXPORT_SYMBOL(audio_switch_device);


int audio_set_device_volume(int vol)
{
	int rc;
	struct cad_flt_cfg_dev_vol cad_dev_volume;
	struct cad_stream_filter_struct_type strm_flt;
	struct msm8k_audio_dev_ctrl *ctrl = &g_ctrl;

	D("%s\n", __func__);

	if ((vol < 0) || (vol > 100)) {
		D("invalid volume value\n");
		return -EINVAL;
	}

	memset(&strm_flt, 0,
		sizeof(struct cad_stream_filter_struct_type));
	memset(&cad_dev_volume, 0,
			sizeof(struct cad_flt_cfg_dev_vol));
	ctrl->current_volume = vol;
	cad_dev_volume.volume = ctrl->current_volume;
	cad_dev_volume.path = 0;
	cad_dev_volume.device_id = ctrl->current_rx_device;
	strm_flt.filter_type =
				CAD_FILTER_CONFIG_DEVICE_VOLUME;
	strm_flt.format_block = &cad_dev_volume;
	strm_flt.format_block_len =
				sizeof(struct cad_flt_cfg_dev_vol);

	rc = cad_ioctl(ctrl->cad_ctrl_handle,
		CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG,
		&strm_flt,
		sizeof(struct cad_stream_filter_struct_type));
	if (rc)
		pr_err("cad_ioctl() set volume failed\n");

	return rc;
}
EXPORT_SYMBOL(audio_set_device_volume);


int audio_set_device_mute(int mute_info)
{
	int rc;
	struct cad_stream_filter_struct_type strm_flt;
	struct cad_flt_cfg_dev_mute dev_mute_buf;
	struct msm8k_audio_dev_ctrl *ctrl = &g_ctrl;
	int mute;
	int path;

	D("%s\n", __func__);

	/* Lower word is mute/unmute, upper is path. */
	mute = mute_info & 0x0000FFFF;
	path = (mute_info & 0xFFFF0000) >> 16;

	if ((path != 0) && (path != 1)) {
		pr_err("%s: invalid path\n", __func__);
		return -1;
	}

	memset(&strm_flt, 0,
		sizeof(struct cad_stream_filter_struct_type));
	memset(&dev_mute_buf, 0,
		sizeof(struct cad_flt_cfg_dev_mute));

	dev_mute_buf.ver_id = CAD_FILTER_CONFIG_DEVICE_VOLUME_VERID;

	if (path == 0)
		dev_mute_buf.device_id = ctrl->current_rx_device;
	else
		dev_mute_buf.device_id = ctrl->current_tx_device;

	dev_mute_buf.path = path;
	dev_mute_buf.mute = mute;

	strm_flt.filter_type = CAD_FILTER_CONFIG_DEVICE_MUTE;
	strm_flt.format_block_len =
		sizeof(struct cad_flt_cfg_dev_mute);
	strm_flt.format_block = &dev_mute_buf;
	rc = cad_ioctl(ctrl->cad_ctrl_handle,
		CAD_IOCTL_CMD_SET_DEVICE_FILTER_CONFIG,
		&strm_flt,
		sizeof(struct cad_stream_filter_struct_type));
	if (rc)
		pr_err("cad_ioctl() set mute failed\n");

	return rc;
}
EXPORT_SYMBOL(audio_set_device_mute);


static int msm8k_audio_dev_ctrl_ioctl(struct inode *inode, struct file *f,
		unsigned int cmd, unsigned long arg)
{
	int rc;
	u32 uparam;

	D("%s\n", __func__);

	switch (cmd) {
	case AUDIO_SWITCH_DEVICE:
		if (copy_from_user(&uparam, (void *)arg,
				sizeof(uparam)))
			return CAD_RES_FAILURE;

		rc = audio_switch_device(uparam);
		break;
	case AUDIO_SET_VOLUME:
		if (copy_from_user(&uparam, (void *)arg,
				sizeof(uparam)))
			return CAD_RES_FAILURE;

		rc = audio_set_device_volume(uparam);

		break;
	case AUDIO_SET_MUTE:
		rc = copy_from_user(&uparam, (void *)arg, sizeof(u32));
		if (rc) {
			pr_err("AUDIO_SET_MUTE copy from user failed\n");
			break;
		}

		rc = audio_set_device_mute(uparam);

		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

#ifdef CONFIG_PROC_FS
int msm8k_audio_dev_ctrl_read_proc(char *pbuf, char **start, off_t offset,
			int count, int *eof, void *data)
{
	int len = 0;
	len += snprintf(pbuf, 16, "audio\n");

	*eof = 1;
	return len;
}
#endif

static const struct file_operations msm8k_audio_dev_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = msm8k_audio_dev_ctrl_open,
	.release = msm8k_audio_dev_ctrl_release,
	.read = msm8k_audio_dev_ctrl_read,
	.write = msm8k_audio_dev_ctrl_write,
	.ioctl = msm8k_audio_dev_ctrl_ioctl,
	.llseek = no_llseek,
};


struct miscdevice msm8k_audio_dev_ctrl_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_audio_dev_ctrl",
	.fops	= &msm8k_audio_dev_ctrl_fops,
};

static int __init msm8k_audio_dev_ctrl_init(void)
{
	struct cad_open_struct_type  cos;
	int rc;
	struct msm8k_audio_dev_ctrl *ctrl = &g_ctrl;

	D("%s\n", __func__);

	rc = misc_register(&msm8k_audio_dev_ctrl_misc);
	if (rc) {
		pr_err("failed to register audio control device\n");
		return CAD_RES_FAILURE;
	}

	cos.format = 0;
	cos.op_code = CAD_OPEN_OP_DEVICE_CTRL;
	ctrl->cad_ctrl_handle = cad_open(&cos);
	ctrl->current_rx_device = CAD_HW_DEVICE_ID_HANDSET_SPKR;
	ctrl->current_tx_device = CAD_HW_DEVICE_ID_HANDSET_MIC;

	if (ctrl->cad_ctrl_handle < 0) {
		pr_err("Dev CTRL handle < 0\n");
		return CAD_RES_FAILURE;
	}

	rc = cad_ioctl(ctrl->cad_ctrl_handle,
			CAD_IOCTL_CMD_STREAM_START,
			NULL,
			0);
	if (rc) {
		pr_err("cad_ioctl() STREAM_START failed\n");
		return CAD_RES_FAILURE;
	}

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(MSM8K_AUDIO_PROC_NAME,
			0, NULL, msm8k_audio_dev_ctrl_read_proc, NULL);
#endif

	return rc;
}

static void __exit msm8k_audio_dev_ctrl_exit(void)
{
	struct msm8k_audio_dev_ctrl *ctrl = &g_ctrl;
	D("%s\n", __func__);

	cad_close(ctrl->cad_ctrl_handle);

#ifdef CONFIG_PROC_FS
	remove_proc_entry(MSM8K_AUDIO_PROC_NAME, NULL);
#endif
}


module_init(msm8k_audio_dev_ctrl_init);
module_exit(msm8k_audio_dev_ctrl_exit);

MODULE_DESCRIPTION("MSM Audio Device Control driver");
MODULE_LICENSE("GPL v2");


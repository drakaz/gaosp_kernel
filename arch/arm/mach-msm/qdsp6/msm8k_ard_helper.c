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
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <mach/qdsp6/msm8k_ard_helper.h>
#include <mach/qdsp6/msm8k_ard.h>
#include <mach/qdsp6/msm8k_ardi.h>
#include <mach/qdsp6/msm8k_cad.h>
#include <mach/qdsp6/msm8k_cad_devices.h>
#include <mach/qdsp6/msm8k_ard_adie.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "ARD: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

/*
	Function to get device ID. Currently there are 4 device IDs that can be
	active at any given time, since there are 4 indpendent DMA paths. These
	are:

	0 - Internal Codec RX
	1 - Internal Codec TX
	2 - External Codec RX
	3 - External Codec TX
*/

u32 get_device_id(u32 cad_device_requested)
{
	u32 dev_id = 0;

	if ((cad_device_requested == CAD_HW_DEVICE_ID_SPKR_PHONE_MONO) ||
		(cad_device_requested == CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO) ||
		(cad_device_requested == CAD_HW_DEVICE_ID_HANDSET_SPKR) ||
		(cad_device_requested == CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO) ||
		(cad_device_requested == CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR) ||
		(cad_device_requested == CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO))

		dev_id = 0;
	else if ((cad_device_requested == CAD_HW_DEVICE_ID_SPKR_PHONE_MIC) ||
		(cad_device_requested == CAD_HW_DEVICE_ID_HEADSET_MIC) ||
		(cad_device_requested == CAD_HW_DEVICE_ID_HANDSET_MIC))

		dev_id = 1;
	else if ((cad_device_requested == CAD_HW_DEVICE_ID_BT_SCO_SPKR) ||
		(cad_device_requested == CAD_HW_DEVICE_ID_BT_A2DP_SPKR))

		dev_id = 2;
	else if (cad_device_requested == CAD_HW_DEVICE_ID_BT_SCO_MIC)
		dev_id = 3;
	else if (cad_device_requested == CAD_HW_DEVICE_ID_I2S_RX)
		dev_id = 6;
	else if (cad_device_requested == CAD_HW_DEVICE_ID_I2S_TX)
		dev_id = 7;
	else
		pr_err("ARD No Support for other devices device = %d\n",
			cad_device_requested);

	return dev_id;
}


s32 codec_disable(enum codec_enum_type codec_type, u32 dev_type, u32 dev_id)
{
	s32 rc = CAD_RES_SUCCESS;

	switch (codec_type) {
	case CODEC_INT:
		rc = adie_disable(dev_type, dev_id);
		if (rc != CAD_RES_FAILURE) {
			rc = adie_close(dev_type);
			if (rc != CAD_RES_SUCCESS) {
				pr_err("ARD Error Closing ADIE, device = %d\n",
						dev_id);
				rc = CAD_RES_FAILURE;
			}
		} else
			pr_err("ARD Error Disabling ADIE, device"
				" = %d\n", dev_id);
		break;
	case CODEC_AUX_PCM:
		pr_err("ARD TBD - DISABLING EXT CODEC, device = %d\n", dev_id);
		break;
	case CODEC_I2S:
		pr_err("ARD - DISABLING GPIOs for I2S, device = %d\n", dev_id);
		break;
	default:
		break;
	}
	return rc;
}


s32 codec_enable(enum codec_enum_type codec_type, u32 dev_type, u32 dev_id)
{
	s32 rc = CAD_RES_SUCCESS;

	switch (codec_type) {
	case CODEC_INT:
		rc = adie_open(dev_type);
		if (rc != CAD_RES_FAILURE) {
			rc = adie_enable(dev_type, dev_id);
			if (rc != CAD_RES_SUCCESS) {
				pr_err("ARD Error enabling ADIE, device = %d\n",
					dev_id);
				rc = CAD_RES_FAILURE;
			}
		} else
			pr_err("ARD Error Opening ADIE, device = %d\n", dev_id);
		break;
	case CODEC_AUX_PCM:
		break;
	case CODEC_I2S:
		pr_err("ARD - ENABLING I2S GPIOs, device = %d\n", dev_id);
		break;
	default:
		break;
	}

	return rc;
}


enum codec_enum_type get_codec_type(u32 device_in_use)
{
	enum codec_enum_type rc;

	switch (device_in_use) {
	case CAD_HW_DEVICE_ID_HANDSET_SPKR:
	case CAD_HW_DEVICE_ID_HANDSET_MIC:
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO:
	case CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO:
	case CAD_HW_DEVICE_ID_HEADSET_MIC:
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MIC:
	case CAD_HW_DEVICE_ID_SPKR_PHONE_MONO:
	case CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO:
	case CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR:
		rc = CODEC_INT;
		break;
	case CAD_HW_DEVICE_ID_BT_SCO_MIC:
	case CAD_HW_DEVICE_ID_BT_SCO_SPKR:
		rc = CODEC_AUX_PCM;
		break;
	case CAD_HW_DEVICE_ID_I2S_TX:
	case CAD_HW_DEVICE_ID_I2S_RX:
		rc = CODEC_I2S;
		break;
	default:
		rc = CODEC_INT;
		pr_err("unsupported device %d\n", device_in_use);
		break;
	}

	return rc;
}

enum ard_ret_enum_type test_clocks(void)
{
	return ARD_FALSE;
}


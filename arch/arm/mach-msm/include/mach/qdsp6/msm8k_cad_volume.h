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

#include <mach/qdsp6/msm8k_cad_module.h>

#define CAD_STREAM_MAX_GAIN                     1200
#define CAD_STREAM_MIN_GAIN                     -4000

#define CAD_DEVICE_HANDSET_MAX_GAIN             1100
#define CAD_DEVICE_HANDSET_MIN_GAIN             -700
#define CAD_DEVICE_HEADSET_MAX_GAIN             400
#define CAD_DEVICE_HEADSET_MIN_GAIN             -1400
#define CAD_DEVICE_SPEAKER_MAX_GAIN             500
#define CAD_DEVICE_SPEAKER_MIN_GAIN             -700
#define CAD_DEVICE_BT_SCO_MAX_GAIN              400
#define CAD_DEVICE_BT_SCO_MIN_GAIN              -1400
#define CAD_DEVICE_BT_A2DP_MAX_GAIN             400
#define CAD_DEVICE_BT_A2DP_MIN_GAIN             -1400
#define CAD_DEVICE_TTY_MAX_GAIN                 400
#define CAD_DEVICE_TTY_MIN_GAIN                 -1400

#define CAD_FILTER_CONFIG_DEVICE_VOLUME_VERID   0x1000
#define CAD_FILTER_CONFIG_STREAM_VOLUME_VERID   0x1000

#define CAD_FILTER_CONFIG_DEVICE_MUTE_VERID     0x1000
#define CAD_FILTER_CONFIG_STREAM_MUTE_VERID     0x1000

#define CAD_FILTER_CONFIG_DEVICE_VOLUME         0x01087472
#define CAD_FILTER_CONFIG_DEVICE_MUTE           0x01087473

#define CAD_FILTER_CONFIG_STREAM_VOLUME         0x010874f2
#define CAD_FILTER_CONFIG_STREAM_MUTE           0x010874f3

struct cad_flt_cfg_dev_vol {
	u32 ver_id;
	u32 device_id;
	u32 path;    /* 0 == Rx, 1 == Tx and 2 == both  */
	s32 volume;  /* Per Device: % map to the gain range of each device. */
};

/*
	Per Stream: 0 is max gain, -4000 is min gain.
	Absolute mB value.
	Support for full range volume control only limited by HW capability.
*/
struct cad_flt_cfg_strm_vol {
	u32 ver_id;
	s32 volume;
};

struct cad_flt_cfg_dev_mute {
	u32 ver_id;
	u32 device_id;
	u32 path;   /* 0 == Rx, 1 == Tx and 2 == both */
	u32 mute;   /* 0 == UnMute,  1 == Mute        */
};

struct cad_flt_cfg_strm_mute {
	u32 ver_id;
	u32 mute;    /* 0 == UnMute,  1 == Mute */
};

#define QDSP6VOLUME_MAX_DEVICE_COUNT         0x0F

struct cad_device_volume_cache {
	s32 current_volume;
	s32 default_volume;
	s32 max_gain;
	s32 min_gain;
	u32 mute;                 /* 1: Mute   0: UnMute  */
	u8  valid_current_volume; /* 1: Valid  0: Invalid */
	u8  setup_session;        /* 1: True   0: False   */
};

/*
	where struct cad_device_volume_cache is the type of an array,
	and the array index is device ID.
*/

s32 qdsp6_volume_mapping(u32 deviceId, s32 percentage);


#define QDSP_IOCTL_CMD_STREAM_DTMF_START            0x01087553

struct q6_dtmf_start {
	u32 tone1_hz;      /* First tone in Hz */
	u32 tone2_hz;      /* Second tone in Hz */
	s32 duration_usec; /* Duration in microseconds */
	s32 gain_mb;       /* Gain in millibels */
};

int cad_volume_dinit(void);
int cad_volume_init(struct cad_func_tbl_type **func_tbl);


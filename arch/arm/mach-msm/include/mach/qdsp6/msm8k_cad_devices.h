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

#ifndef CADDEVICES_H
#define CADDEVICES_H


#define CAD_HW_DEVICE_ID_HANDSET_MIC		0x01
#define CAD_HW_DEVICE_ID_HANDSET_SPKR		0x02
#define CAD_HW_DEVICE_ID_HEADSET_MIC		0x03
#define CAD_HW_DEVICE_ID_HEADSET_SPKR_MONO	0x04
#define CAD_HW_DEVICE_ID_HEADSET_SPKR_STEREO	0x05
#define CAD_HW_DEVICE_ID_SPKR_PHONE_MIC		0x06
#define CAD_HW_DEVICE_ID_SPKR_PHONE_MONO	0x07
#define CAD_HW_DEVICE_ID_SPKR_PHONE_STEREO	0x08
#define CAD_HW_DEVICE_ID_BT_SCO_MIC		0x09
#define CAD_HW_DEVICE_ID_BT_SCO_SPKR		0x0A
#define CAD_HW_DEVICE_ID_BT_A2DP_SPKR		0x0B
#define CAD_HW_DEVICE_ID_TTY_HEADSET_MIC	0x0C
#define CAD_HW_DEVICE_ID_TTY_HEADSET_SPKR	0x0D

#define CAD_HW_DEVICE_ID_DEFAULT_TX		0x0E
/*Starts with CAD_HW_DEVICE_ID_HANDSET_MIC*/
#define CAD_HW_DEVICE_ID_DEFAULT_RX		0x0F
/*Starts with CAD_HW_DEVICE_ID_HANDSET_SPKR*/

/* Logical Device to indicate A2DP routing */
#define CAD_HW_DEVICE_ID_BT_A2DP_TX             0x10

#define CAD_HW_DEVICE_ID_I2S_RX                 0x20
#define CAD_HW_DEVICE_ID_I2S_TX                 0x21


#define CAD_RX_DEVICE  0x00
#define CAD_TX_DEVICE  0x01

#endif

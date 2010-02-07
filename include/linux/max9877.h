#if defined(CONFIG_SAMSUNG_CAPELA)

/*
 *  Copyright (c) 2007, 2008 SAMSUNG, Inc
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without modification,
 *   are permitted provided that the following conditions are met: 
 *
 *   Redistributions of source code must retain the above copyright notice, this list 
 *   of conditions and the following disclaimer. 
 *   Redistributions in binary form must reproduce the above copyright notice, this 
 *   list of conditions and the following disclaimer in the documentation and/or other
 *   materials provided with the distribution. 
 *   Neither the name of the HTC,Inc nor the names of its contributors may be used 
 *   to endorse or promote products derived from this software without specific prior
 *   written permission. 
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
 *   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 *   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE, DATA, OR 
 *   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
 *   OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT 
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 *   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
 *   DAMAGE
 */

#ifndef AMP_MAX9877_H
#define AMP_MAX9877_H
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <asm/sizes.h>

/*************************************************************
*	IOCTL define
*************************************************************/

#define MAX9877_I2C_IOCTL_MAGIC 'm'

#define MAX9877_I2C_IOCTL_W \
	        _IOW(MAX9877_I2C_IOCTL_MAGIC, 0, unsigned)

#define MAX9877_I2C_IOCTL_R \
	        _IOR(MAX9877_I2C_IOCTL_MAGIC, 1, unsigned)

#define MAX9877_I2C_IOCTL_SWITCH_DEVICE \
	        _IO(MAX9877_I2C_IOCTL_MAGIC, 2)

#define MAX9877_HPH_VOL_SET \
	        _IOW(MAX9877_I2C_IOCTL_MAGIC, 3, unsigned)

#define MAX9877_SPK_VOL_SET \
	        _IOW(MAX9877_I2C_IOCTL_MAGIC, 4, unsigned)

#define MAX9877_SPEAKER_ON \
	        _IO(MAX9877_I2C_IOCTL_MAGIC, 5)

#define MAX9877_SPEAKER_OFF \
	        _IO(MAX9877_I2C_IOCTL_MAGIC, 6)
    
#define MAX9877_HEADSET_ON \
               _IOW(MAX9877_I2C_IOCTL_MAGIC, 7, unsigned)

#define MAX9877_HEADSET_OFF \
               _IO(MAX9877_I2C_IOCTL_MAGIC, 8)

#define MAX9877_SPK_EAR_ON \
	        _IO(MAX9877_I2C_IOCTL_MAGIC, 9)

#define MAX9877_RCV_ON \
	        _IO(MAX9877_I2C_IOCTL_MAGIC, 10)

#define MAX9877_RCV_OFF \
	        _IO(MAX9877_I2C_IOCTL_MAGIC, 11)

    
#define MAX9877_I2C_IOCTL_INIT \
	_IO(MAX9877_I2C_IOCTL_MAGIC, 12) 

#define MAX9877_AMP_SUSPEND \
	_IO(MAX9877_I2C_IOCTL_MAGIC, 13)

#define MAX9877_AMP_RESUME \
	_IO(MAX9877_I2C_IOCTL_MAGIC, 14)

#define MAX9877_AMP_RECORDING_START \
	_IO(MAX9877_I2C_IOCTL_MAGIC, 15)

#define MAX9877_AMP_RECORDING_STOP \
	_IO(MAX9877_I2C_IOCTL_MAGIC, 16)

#if 0
struct max9877_init {
	int preview; /* in: 1 for preview, 0 for capture */
	uint16_t chipid; /* out: chip id */
};
#endif
#endif
#endif /* CONFIG_SAMSUNG_CAPELA */

int max9877_suspend(void);
int max9877_resume(void);

int max9877_gpio_recording_start(int state);

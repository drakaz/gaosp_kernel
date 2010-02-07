/* drivers/char/diag/diagchar.h */

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

#ifndef DIAGCHAR_H
#define DIAGCHAR_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <mach/msm_smd.h>
#include <asm/atomic.h>

/* Size of the USB buffers used for read and write*/
#define USB_MAX_BUF 4096

/* Size of the buffer used for deframing a packet
  reveived from the PC tool*/
#define HDLC_MAX 4096

/* Number of maximum USB requests that the USB layer should handle at
   one time. */
#define MAX_DIAG_USB_REQUESTS 12
#define MSG_MASK_SIZE 8000
#define LOG_MASK_SIZE 1000
#define EVENT_MASK_SIZE 1000
#define REG_TABLE_SIZE 25
#define PKT_SIZE 1000

struct diag_master_table {
	uint16_t cmd_code;
	uint16_t subsys_id;
	uint16_t cmd_code_lo;
	uint16_t cmd_code_hi;
	int process_id;
};


struct diagchar_dev {

	/* State for the char driver */
	unsigned int major;
	unsigned int minor_start;
	int num;
	struct cdev *cdev;
	char *name;
	int dropped_count;
	struct class *diagchar_class;
	int ref_count;
	struct mutex diagchar_mutex;
	wait_queue_head_t wait_q;
	int *client_map;
	int *data_ready;
	int num_clients;
	unsigned int itemsize;
	unsigned int poolsize;

	/* State for the mempool for the char driver */
	mempool_t *diagpool;
	struct mutex diagmem_mutex;
	int count;

	/* State for diag forwarding */
	unsigned char *usb_buf_in;
	unsigned char *usb_buf_in_qdsp;
	unsigned char *usb_buf_out;
	smd_channel_t *ch;
	smd_channel_t *chqdsp;
	int in_busy;
	int in_busy_qdsp;
	int read_len;
	unsigned char *hdlc_buf;
	unsigned hdlc_count;
	unsigned hdlc_escape;
	int usb_connected;
	struct workqueue_struct *diag_wq;
	struct work_struct diag_read_work;
	uint8_t *msg_masks;
	uint8_t *log_masks;
	uint8_t *event_masks;
	struct diag_master_table *table;
	uint8_t *pkt_buf;
	int pkt_length;
};

extern struct diagchar_dev *driver;
#endif

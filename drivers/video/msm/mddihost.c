/* drivers/video/msm/src/drv/mddi/mddihost.c
 *
 * Copyright (c) 2008 QUALCOMM USA, INC.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#include <linux/clk.h>
#include <mach/clk.h>

struct semaphore mddi_host_mutex;

struct clk *mddi_io_clk;
static boolean mddi_host_powered = FALSE;
static boolean mddi_host_initialized = FALSE;
extern uint32 *mddi_reg_read_value_ptr;

mddi_lcd_func_type mddi_lcd;

extern mddi_client_capability_type mddi_client_capability_pkt;

#ifdef FEATURE_MDDI_HITACHI
extern void mddi_hitachi_window_adjust(uint16 x1,
				       uint16 x2, uint16 y1, uint16 y2);
#endif

extern void mddi_toshiba_lcd_init(void);

#ifdef FEATURE_MDDI_S6D0142
extern void mddi_s6d0142_lcd_init(void);
extern void mddi_s6d0142_window_adjust(uint16 x1,
				       uint16 x2,
				       uint16 y1,
				       uint16 y2,
				       mddi_llist_done_cb_type done_cb);
#endif

void mddi_init(void)
{
	if (mddi_host_initialized)
		return;

	mddi_host_initialized = TRUE;

	init_MUTEX(&mddi_host_mutex);

	if (!mddi_host_powered) {
		down(&mddi_host_mutex);
		mddi_host_init(MDDI_HOST_PRIM);
		mddi_host_powered = TRUE;
		up(&mddi_host_mutex);
		mdelay(10);
	}
}

void mddi_host_register_read
    (uint32 reg_addr,
     uint32 *reg_value_ptr, boolean wait, mddi_host_type host) {
	mddi_linked_list_type *curr_llist_ptr;
	mddi_register_access_packet_type *regacc_pkt_ptr;
	uint16 curr_llist_idx;

	if (in_interrupt()) {
		MDDI_MSG_CRIT("Called from ISR context\n");
	}

	if (!mddi_host_powered) {
		MDDI_MSG_ERR("MDDI powered down!\n");
		mddi_init();
	}

	down(&mddi_host_mutex);

	mddi_reg_read_value_ptr = reg_value_ptr;
	curr_llist_idx = mddi_get_reg_read_llist_item(host, TRUE);
	if (curr_llist_idx == UNASSIGNED_INDEX) {
		up(&mddi_host_mutex);

		/* need to change this to some sort of wait */
		MDDI_MSG_ERR("Attempting to queue up more than 1 reg read\n");
		return;
	}

	curr_llist_ptr = &llist_extern[host][curr_llist_idx];
	curr_llist_ptr->link_controller_flags = 0x11;
	curr_llist_ptr->packet_header_count = 14;
	curr_llist_ptr->packet_data_count = 0;

	curr_llist_ptr->next_packet_pointer = NULL;
	curr_llist_ptr->packet_data_pointer = NULL;
	curr_llist_ptr->reserved = 0;

	regacc_pkt_ptr = &curr_llist_ptr->packet_header.register_pkt;

	regacc_pkt_ptr->packet_length = curr_llist_ptr->packet_header_count;
	regacc_pkt_ptr->packet_type = 146;	/* register access packet */
	regacc_pkt_ptr->bClient_ID = 0;
	regacc_pkt_ptr->read_write_info = 0x8001;
	regacc_pkt_ptr->register_address = reg_addr;

	/* now adjust pointers */
	mddi_queue_forward_packets(curr_llist_idx, curr_llist_idx, wait,
				   NULL, host);
	/* need to check if we can write the pointer or not */

	up(&mddi_host_mutex);

	if (wait) {
		mddi_linked_list_notify_type *llist_notify_ptr;
		llist_notify_ptr = &llist_extern_notify[host][curr_llist_idx];
		wait_for_completion_interruptible(&
						  (llist_notify_ptr->
						   done_comp));
	}

	MDDI_MSG_DEBUG("Reg Read value=0x%x\n", *reg_value_ptr);

}				/* mddi_host_register_read */

void mddi_host_register_write
    (uint32 reg_addr,
     uint32 reg_val,
     boolean wait, mddi_llist_done_cb_type done_cb, mddi_host_type host) {
	mddi_linked_list_type *curr_llist_ptr;
	mddi_linked_list_type *curr_llist_dma_ptr;
	mddi_register_access_packet_type *regacc_pkt_ptr;
	uint16 curr_llist_idx;

	if (in_interrupt()) {
		MDDI_MSG_CRIT("Called from ISR context\n");
	}

	if (!mddi_host_powered) {
		MDDI_MSG_ERR("MDDI powered down!\n");
		mddi_init();
	}

	down(&mddi_host_mutex);

	curr_llist_idx = mddi_get_next_free_llist_item(host, TRUE);
	curr_llist_ptr = &llist_extern[host][curr_llist_idx];
	curr_llist_dma_ptr = &llist_dma_extern[host][curr_llist_idx];

	curr_llist_ptr->link_controller_flags = 1;
	curr_llist_ptr->packet_header_count = 14;
	curr_llist_ptr->packet_data_count = 4;

	curr_llist_ptr->next_packet_pointer = NULL;
	curr_llist_ptr->reserved = 0;

	regacc_pkt_ptr = &curr_llist_ptr->packet_header.register_pkt;

	regacc_pkt_ptr->packet_length = curr_llist_ptr->packet_header_count + 4;
	regacc_pkt_ptr->packet_type = 146;	/* register access packet */
	regacc_pkt_ptr->bClient_ID = 0;
	regacc_pkt_ptr->read_write_info = 0x0001;
	regacc_pkt_ptr->register_address = reg_addr;
	regacc_pkt_ptr->register_data_list = reg_val;

	MDDI_MSG_DEBUG("Reg Access write reg=0x%x, value=0x%x\n",
		       regacc_pkt_ptr->register_address,
		       regacc_pkt_ptr->register_data_list);

	regacc_pkt_ptr = &curr_llist_dma_ptr->packet_header.register_pkt;
	curr_llist_ptr->packet_data_pointer =
	    (void *)(&regacc_pkt_ptr->register_data_list);

	/* now adjust pointers */
	mddi_queue_forward_packets(curr_llist_idx, curr_llist_idx, wait,
				   done_cb, host);

	up(&mddi_host_mutex);

	if (wait) {
		mddi_linked_list_notify_type *llist_notify_ptr;
		llist_notify_ptr = &llist_extern_notify[host][curr_llist_idx];
		wait_for_completion_interruptible(&
						  (llist_notify_ptr->
						   done_comp));
	}

}				/* mddi_host_register_write */

#if defined(CONFIG_SAMSUNG_CAPELA)
void mddi_host_register_multi_write
    (uint32 reg_addr,
     uint32 * reg_val_ptr,
     uint32 data_len,
     boolean wait, mddi_llist_done_cb_type done_cb, mddi_host_type host) {
	mddi_linked_list_type *curr_llist_ptr;
	mddi_linked_list_type *curr_llist_dma_ptr;
	mddi_register_multi_access_packet_type *regacc_pkt_ptr;

	uint16 curr_llist_idx;
	int n;

	if (in_interrupt()) {
		MDDI_MSG_CRIT("Called from ISR context\n");
	}

	if (!mddi_host_powered) {
		MDDI_MSG_ERR("MDDI powered down!\n");
		mddi_init();
	}

	down(&mddi_host_mutex);

	curr_llist_idx = mddi_get_next_free_llist_item(host, TRUE);
	curr_llist_ptr = &llist_extern[host][curr_llist_idx];
	curr_llist_dma_ptr = &llist_dma_extern[host][curr_llist_idx];

	curr_llist_ptr->link_controller_flags = 1;
	curr_llist_ptr->packet_header_count = 14;
	curr_llist_ptr->packet_data_count = ( 4 * data_len);

	curr_llist_ptr->next_packet_pointer = NULL;
	curr_llist_ptr->reserved = 0;

	regacc_pkt_ptr = &curr_llist_ptr->packet_header.register_pkt;

	regacc_pkt_ptr->packet_length = curr_llist_ptr->packet_header_count + ( 4 * data_len);
	regacc_pkt_ptr->packet_type = 146;	/* register access packet */
	regacc_pkt_ptr->bClient_ID = 0;
	regacc_pkt_ptr->read_write_info = 0x0001;
	regacc_pkt_ptr->register_address = reg_addr;

	for(n = 0; n < data_len ; n++)
	{
		regacc_pkt_ptr->register_data_list [n] = *reg_val_ptr;
		*reg_val_ptr++;

		MDDI_MSG_DEBUG("Reg Access write reg=0x%x, value=0x%x\n",
			       regacc_pkt_ptr->register_address,
			       regacc_pkt_ptr->register_data_list[n]);
	}

	regacc_pkt_ptr = &curr_llist_dma_ptr->packet_header.register_pkt;
	curr_llist_ptr->packet_data_pointer =
	    (void *)(&regacc_pkt_ptr->register_data_list);

	/* now adjust pointers */
	mddi_queue_forward_packets(curr_llist_idx, curr_llist_idx, wait,
				   done_cb, host);

	up(&mddi_host_mutex);

	if (wait) {
		mddi_linked_list_notify_type *llist_notify_ptr;
		llist_notify_ptr = &llist_extern_notify[host][curr_llist_idx];
		wait_for_completion_interruptible(&
						  (llist_notify_ptr->
						   done_comp));
	}

}				/* mddi_host_register_multi_write */
#endif

boolean mddi_host_register_read_int
    (uint32 reg_addr, uint32 *reg_value_ptr, mddi_host_type host) {
	mddi_linked_list_type *curr_llist_ptr;
	mddi_register_access_packet_type *regacc_pkt_ptr;
	uint16 curr_llist_idx;

	if (!in_interrupt()) {
		MDDI_MSG_CRIT("Called from TASK context\n");
	}

	if (!mddi_host_powered) {
		MDDI_MSG_ERR("MDDI powered down!\n");
		return FALSE;
	}

	if (down_trylock(&mddi_host_mutex) != 0) {
		return FALSE;
	}

	mddi_reg_read_value_ptr = reg_value_ptr;
	curr_llist_idx = mddi_get_reg_read_llist_item(host, FALSE);
	if (curr_llist_idx == UNASSIGNED_INDEX) {
		up(&mddi_host_mutex);
		return FALSE;
	}

	curr_llist_ptr = &llist_extern[host][curr_llist_idx];
	curr_llist_ptr->link_controller_flags = 0x11;
	curr_llist_ptr->packet_header_count = 14;
	curr_llist_ptr->packet_data_count = 0;

	curr_llist_ptr->next_packet_pointer = NULL;
	curr_llist_ptr->packet_data_pointer = NULL;
	curr_llist_ptr->reserved = 0;

	regacc_pkt_ptr = &curr_llist_ptr->packet_header.register_pkt;

	regacc_pkt_ptr->packet_length = curr_llist_ptr->packet_header_count;
	regacc_pkt_ptr->packet_type = 146;	/* register access packet */
	regacc_pkt_ptr->bClient_ID = 0;
	regacc_pkt_ptr->read_write_info = 0x8001;
	regacc_pkt_ptr->register_address = reg_addr;

	/* now adjust pointers */
	mddi_queue_forward_packets(curr_llist_idx, curr_llist_idx, FALSE,
				   NULL, host);
	/* need to check if we can write the pointer or not */

	up(&mddi_host_mutex);

	return TRUE;

}				/* mddi_host_register_read */

boolean mddi_host_register_write_int
    (uint32 reg_addr,
     uint32 reg_val, mddi_llist_done_cb_type done_cb, mddi_host_type host) {
	mddi_linked_list_type *curr_llist_ptr;
	mddi_linked_list_type *curr_llist_dma_ptr;
	mddi_register_access_packet_type *regacc_pkt_ptr;
	uint16 curr_llist_idx;

	if (!in_interrupt()) {
		MDDI_MSG_CRIT("Called from TASK context\n");
	}

	if (!mddi_host_powered) {
		MDDI_MSG_ERR("MDDI powered down!\n");
		return FALSE;
	}

	if (down_trylock(&mddi_host_mutex) != 0) {
		return FALSE;
	}

	curr_llist_idx = mddi_get_next_free_llist_item(host, FALSE);
	if (curr_llist_idx == UNASSIGNED_INDEX) {
		up(&mddi_host_mutex);
		return FALSE;
	}

	curr_llist_ptr = &llist_extern[host][curr_llist_idx];
	curr_llist_dma_ptr = &llist_dma_extern[host][curr_llist_idx];

	curr_llist_ptr->link_controller_flags = 1;
	curr_llist_ptr->packet_header_count = 14;
	curr_llist_ptr->packet_data_count = 4;

	curr_llist_ptr->next_packet_pointer = NULL;
	curr_llist_ptr->reserved = 0;

	regacc_pkt_ptr = &curr_llist_ptr->packet_header.register_pkt;

	regacc_pkt_ptr->packet_length = curr_llist_ptr->packet_header_count + 4;
	regacc_pkt_ptr->packet_type = 146;	/* register access packet */
	regacc_pkt_ptr->bClient_ID = 0;
	regacc_pkt_ptr->read_write_info = 0x0001;
	regacc_pkt_ptr->register_address = reg_addr;
	regacc_pkt_ptr->register_data_list = reg_val;

	regacc_pkt_ptr = &curr_llist_dma_ptr->packet_header.register_pkt;
	curr_llist_ptr->packet_data_pointer =
	    (void *)(&(regacc_pkt_ptr->register_data_list));

	/* now adjust pointers */
	mddi_queue_forward_packets(curr_llist_idx, curr_llist_idx, FALSE,
				   done_cb, host);
	up(&mddi_host_mutex);

	return TRUE;

}				/* mddi_host_register_write */

void mddi_wait(uint16 time_ms)
{
	mdelay(time_ms);
}

void mddi_client_lcd_vsync_detected(boolean detected)
{
	if (mddi_lcd.vsync_detected) {
		(*mddi_lcd.vsync_detected) (detected);
	}
}

/* extended version of function includes done callback */
void mddi_window_adjust_ext(struct msm_fb_data_type *mfd,
			    uint16 x1,
			    uint16 x2,
			    uint16 y1,
			    uint16 y2, mddi_llist_done_cb_type done_cb)
{
//UNIVERSAL_SURF
#if defined(CONFIG_SAMSUNG_CAPELA)
	uint32_t sr1, sr2, er1, er2;
	uint32_t sc1, sc2, ec1, ec2;

	sr1 = (y1 & 0x00FF ) << 8;
	sr2 = (y1 & 0xFF00 ) >> 8;

	er1 = ((y1+y2) & 0x00FF ) << 8;
	er2 = ((y1+y2) & 0xFF00 ) >> 8;

	sc1 = (x1 & 0x00FF ) << 8;
	sc2 = (x1 & 0xFF00 ) >> 8;

	ec1 = ((x1+x2) & 0x00FF ) << 8;
	ec2 = ((x1+x2) & 0xFF00 ) >> 8;

	mddi_queue_register_write(0x2A, ((ec1|ec2) << 16 |(sc1|sc2)), TRUE , 0);
	mddi_queue_register_write(0x2B, ((er1|er2) << 16 | (sr1|sr2)), TRUE, 0);
#else
#ifdef FEATURE_MDDI_HITACHI
	if (mfd->panel.id == HITACHI) {
		mddi_hitachi_window_adjust(x1, x2, y1, y2);
	}
#elif defined(FEATURE_MDDI_S6D0142)
	if (mfd->panel.id == MDDI_LCD_S6D0142) {
		mddi_s6d0142_window_adjust(x1, x2, y1, y2, done_cb);
	}
#else
	/* Do nothing then... except avoid lint/compiler warnings */
	(void)x1;
	(void)x2;
	(void)y1;
	(void)y2;
	(void)done_cb;
#endif
#endif /* CONFIG_SAMSUNG_CAPELA */
}

void mddi_window_adjust(struct msm_fb_data_type *mfd,
			uint16 x1, uint16 x2, uint16 y1, uint16 y2)
{
	mddi_window_adjust_ext(mfd, x1, x2, y1, y2, NULL);
}

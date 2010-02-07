/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/delay.h>  

#include "power.h"

// for LCD
int lcd_suspend;
static int lcd_is_on; 
extern int bridge_on; 
EXPORT_SYMBOL(lcd_suspend);

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;
        
	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");
	
	// for LCD
	lcd_suspend = 1;

	list_for_each_entry(pos, &early_suspend_handlers, link) {
		if (pos->suspend != NULL)
			pos->suspend(pos);
	}
	mutex_unlock(&early_suspend_lock);

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: sync\n");

	sys_sync();
abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		wake_unlock(&main_wake_lock);
	spin_unlock_irqrestore(&state_lock, irqflags);
}


static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;
	int level;
 
	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
		
	// for LCD
	lcd_suspend = 0;  
	
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link) {
	    level = pos->level;	    
	    
            if (lcd_is_on || bridge_on) {  
                if (pos->level >= 148 ) {
                    if (debug_mask & DEBUG_SUSPEND)
                        printk("skip	lcd_is_on(%d), bridge_on(%d)\n", lcd_is_on, bridge_on);           	
                }                     
                else
                {
	            if (pos->resume != NULL)
		        pos->resume(pos);
	        }	
            }
            else 
            {                      
	        if (pos->resume != NULL)
		    pos->resume(pos);
	    }	
	}
	lcd_is_on = 1;	
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
}

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;
        
	struct early_suspend *pos;  
        int level;                 
	int abort = 0;             

        if (debug_mask & DEBUG_SUSPEND)
            printk("\n\n---- Enter request_suspend_state() 	in earlysuspend.c\n");

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		state |= SUSPEND_REQUESTED;
		queue_work(suspend_work_queue, &early_suspend_work);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
		state &= ~SUSPEND_REQUESTED;
		wake_lock(&main_wake_lock);
		lcd_is_on = 0;
		queue_work(suspend_work_queue, &late_resume_work);  
	}
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);

	mutex_lock(&early_suspend_lock);
        if (old_sleep && new_state == PM_SUSPEND_ON) {
	    spin_lock_irqsave(&state_lock, irqflags);
	    if (lcd_is_on) {   
		if (debug_mask & DEBUG_SUSPEND)
		    printk("LCD is already ON!! case 1 ------------- request_suspend_state() : state : %d  in earlysuspend.c\n", state);
		abort = 1;		
	    }
	    else if (bridge_on)  
	    {
	    	if (debug_mask & DEBUG_SUSPEND)
	    	    printk("LCD is already ON!! case 2 ------------- Don't try to turn on the LCD device.\n");
		abort = 1;	    	
	    }
	    else {
		printk("Turn on the LCD, request_suspend_state() in earlysuspend.c : state is (%d) \n", state);		
	    }
	    spin_unlock_irqrestore(&state_lock, irqflags);

	    if (abort) {
		goto abort;
	    }
    
	    // just turn on the LCD. 
	    lcd_suspend = 0;
	    list_for_each_entry_reverse(pos, &early_suspend_handlers, link) {
                level = pos->level;
                if (level >= 148) {
  	            if (pos->resume != NULL) {
	                pos->resume(pos);
	            }
	        }	        
            }
            lcd_is_on = 1;
	}
abort:
    mutex_unlock(&early_suspend_lock);

}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}

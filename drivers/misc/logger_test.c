/*
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
 */
#include <linux/module.h>
#include <linux/logger.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#define MODULE_NAME "logger_test"

/* 1/2 second for timer delay */
#define TIMER_DELAY_JIFFIES (msecs_to_jiffies(500))
/* 1/2 second for timer delay */
#define THREAD_SLEEP_TIME_JIFFIES (msecs_to_jiffies(500))
#define LOGGER_TEST_PRIORITY (LOG_PRIORITY_SILENT)

static struct task_struct *kthread;
static struct timer_list my_timer;

static void timer_func(unsigned long ptr)
{
	static int counter;
	int ret = logger_write(LOG_RADIO_IDX,
		LOGGER_TEST_PRIORITY,
		"MYTAG-INTERRUPT",
		"From timer interrupt: %d\n",
		counter);
	if (ret)
		printk(KERN_ERR "interrupt %d logger write ret: %d\n",
			counter, ret);

	counter++;
	my_timer.expires = jiffies + TIMER_DELAY_JIFFIES;
	add_timer(&my_timer);
}

static int thread(void *n)
{
	int counter = 0;
	do {
		int ret;

		schedule_timeout_interruptible(THREAD_SLEEP_TIME_JIFFIES);
		ret = logger_write(LOG_RADIO_IDX,
			LOGGER_TEST_PRIORITY,
			"MYTAG-THREAD",
			"From Thread: %d\n",
			counter);
		if (ret)
			printk(KERN_ERR "thread %d logger write ret: %d\n",
				counter, ret);
		counter++;
	} while (!kthread_should_stop());
	return 0;
}

static int __init logger_test_init(void)
{
	int junk = 25;
	int ret;

	/* start kernel thread */
	kthread = kthread_run((void *)thread, NULL, MODULE_NAME"_thread");
	if (IS_ERR(kthread)) {
		printk(KERN_INFO MODULE_NAME
			": unable to start kernel thread\n");
		return -ENOMEM;
	}

	init_timer(&my_timer);
	my_timer.function = timer_func;
	my_timer.data = 0;
	my_timer.expires = jiffies + TIMER_DELAY_JIFFIES;
	add_timer(&my_timer);

	ret = logger_write(LOG_RADIO_IDX,
		LOG_PRIORITY_DEBUG,
		"MYTAG",
		"This should not be present in the log, %d\n",
		junk++);
	if (ret)
		printk(KERN_ERR "logger write 1 returned %d\n", ret);


	ret = logger_write(LOG_RADIO_IDX,
		LOGGER_TEST_PRIORITY,
		"MYTAG",
		"This >should< be present in the log, %d\n",
		junk++);
	if (ret)
		printk(KERN_ERR "logger write 2 returned %d\n", ret);

	ret = logger_write(LOG_RADIO_IDX,
		LOGGER_TEST_PRIORITY,
		"MYTAG1",
		"This >should< be present in the log with a new tag, %d\n",
		junk++);
	if (ret)
		printk(KERN_ERR "logger write 3 returned %d\n", ret);

	return 0;
}

static void __exit logger_test_exit(void)
{
	del_timer_sync(&my_timer);
	kthread_stop(kthread);
}


device_initcall(logger_test_init);
module_exit(logger_test_exit);

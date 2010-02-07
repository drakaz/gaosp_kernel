/* linux/arch/arm/mach-msm/board-keypad.c
 *
 * Copyright (C) 2008 Samsung Electronics.
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>

#include <asm/gpio.h>
#include <mach/hardware.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>

#include <asm/io.h>
#include <asm/delay.h>

#include <linux/gpio_event.h>

#if defined(CONFIG_SAMSUNG_CAPELA)
static unsigned int row_gpios[] = {35, 34, 33, 32};		// KEYSCAN
static unsigned int col_gpios[] = {42, 41, 40, 39};		// KEYSENSE

#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(col_gpios) + (col))

unsigned short keymap[ARRAY_SIZE(col_gpios) * ARRAY_SIZE(row_gpios)] = {

	[KEYMAP_INDEX(0, 0)] = 232, // KEY_ENTER,		// CAPELA : OK
	[KEYMAP_INDEX(0, 1)] = KEY_LEFT,		// CAPELA : LEFT
	[KEYMAP_INDEX(0, 2)] = KEY_MENU,		// CAPELA : CLR		-> KEYMAP fault : MENU
	[KEYMAP_INDEX(0, 3)] = KEY_END,			// CAPELA : KEY INTERRUPT

	[KEYMAP_INDEX(1, 0)] = KEY_UP,			// CAPELA : UP
	[KEYMAP_INDEX(1, 1)] = KEY_RIGHT,		// CAPELA : RIGHT
	[KEYMAP_INDEX(1, 2)] = KEY_HOME,		// CAPELA : HOME
	[KEYMAP_INDEX(1, 3)] = KEY_RESERVED, //KEY_D,	// CAPELA : X

	[KEYMAP_INDEX(2, 0)] = KEY_VOLUMEUP,	// CAPELA : VOLUMEUP
	[KEYMAP_INDEX(2, 1)] = KEY_VOLUMEDOWN,	// CAPELA : VOLUME DOWN
	[KEYMAP_INDEX(2, 2)] = KEY_CAMERA_AF,	// CAPELA : CAM AF(247)
	[KEYMAP_INDEX(2, 3)] = KEY_CAMERA,		// CAPELA : CAM CAPTURE

	[KEYMAP_INDEX(3, 0)] = KEY_DOWN,		// CAPELA : DOWN
	[KEYMAP_INDEX(3, 1)] = KEY_BACK,		// CAPELA : MENU	-> KEYMAP fault : BACK
	[KEYMAP_INDEX(3, 2)] = KEY_SEND,		// CAPELA : SEND
	[KEYMAP_INDEX(3, 3)] = KEY_SCREENLOCK,	// CAPELA : 152 hold added ORION 0.5 (09. 04. 17)
};
#endif

static const unsigned short virtual_keys[] = {
	KEY_END,
	KEY_POWER
};

static int board_gpio_event_matrix_func(struct input_dev *input_dev,
					  struct gpio_event_info *info,
					  void **data, int func);


static struct gpio_event_matrix_info matrix_info = {
	.info.func	= board_gpio_event_matrix_func,
	.keymap		= keymap,
	.output_gpios	= row_gpios,
	.input_gpios	= col_gpios,
	.noutputs	= ARRAY_SIZE(row_gpios),
	.ninputs	= ARRAY_SIZE(col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE | GPIOKPF_PRINT_UNMAPPED_KEYS
};

struct gpio_event_info *keypad_info[] = {
	&matrix_info.info,
};

static struct gpio_event_platform_data keypad_data = {
	.name		= "I7500_keypad",
	.info		= keypad_info,
	.info_count	= ARRAY_SIZE(keypad_info)
};

static struct platform_device keypad_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &keypad_data,
	},
};

static struct input_dev *dev;

static int board_gpio_event_matrix_func(struct input_dev *input_dev,
					  struct gpio_event_info *info,
					  void **data, int func)
{
	int err;
	int i;

	err = gpio_event_matrix_func(input_dev, info, data, func);

	if (func == GPIO_EVENT_FUNC_INIT && !err) {
		dev = input_dev;
		for (i = 0; i < ARRAY_SIZE(virtual_keys); i++)
			set_bit(virtual_keys[i] & KEY_MAX,
				input_dev->keybit);
	} else if (func == GPIO_EVENT_FUNC_UNINIT) {
		dev = NULL;
	}

	return err;
}


struct input_dev *msm_keypad_get_input_dev(void)
{
	return dev;
}
void init_keypad(void)
{
	platform_device_register(&keypad_device);
}

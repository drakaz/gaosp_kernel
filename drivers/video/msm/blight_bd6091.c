/* arch/arm/mach-msm/blight_bd6091.c
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
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/io.h>
#include <asm/gpio.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>

//#define DEBUG
#ifdef _DEBUG
#define dprintk(s, args...) printk("[BackLight] %s:%d - " s, __func__, __LINE__,  ##args)
#else
#define dprintk(s, args...)
#endif

#define MDDI_CLIENT_CORE_BASE  0x108000
#define LCD_CONTROL_BLOCK_BASE 0x110000
#define PWM_BLOCK_BASE         0x140000
#define DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define SYSCLKENA   (MDDI_CLIENT_CORE_BASE|0x2C)
#define START       (LCD_CONTROL_BLOCK_BASE|0x08)
#define PWM0OFF     (PWM_BLOCK_BASE|0x1C)

#define REPUS_DEFAULT_BACKLIGHT_BRIGHTNESS 255

#define SENSOR_SCL 2
#define SENSOR_SDA 3
#define BD6091_ADDR 0xEC // 0x76

static int repus_backlight_off;
static int repus_backlight_brightness = REPUS_DEFAULT_BACKLIGHT_BRIGHTNESS;
static uint8_t repus_backlight_last_level = 33;
static DEFINE_MUTEX(repus_backlight_lock);
extern void panel_backlight_set_with_i2c_gpio(unsigned *, unsigned);

static void repus_set_backlight_level(uint8_t level)
{
	unsigned percent = ((int)level * 100) / 255;
	unsigned long flags;
	int i = 0;
	
	unsigned bl_current[3] =   {BD6091_ADDR,0x03,0x4F};
	unsigned led_level = 0;

	dprintk("level = %d\n", level);
	if (repus_backlight_last_level == level)
		return;

	if (level) 
	{
		local_irq_save(flags);

		led_level = (127 * percent) / 100;
		dprintk("led_level = %d\n", led_level);
		bl_current[2] = led_level;
		panel_backlight_set_with_i2c_gpio(bl_current,3);

		local_irq_restore(flags);
	}
	repus_backlight_last_level = level;
}

static void repus_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	dprintk("\n");
	mutex_lock(&repus_backlight_lock);
	repus_backlight_brightness = value;
	if(!repus_backlight_off)
		repus_set_backlight_level(repus_backlight_brightness);
	mutex_unlock(&repus_backlight_lock);
}

static struct led_classdev repus_backlight_led = {
	.name			= "lcd-backlight",
	.brightness = REPUS_DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set = repus_brightness_set,
};

static int __init repus_backlight_probe(struct platform_device *pdev)
{
	return led_classdev_register(&pdev->dev, &repus_backlight_led);
}

static int repus_backlight_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&repus_backlight_led);
	return 0;
}

static struct platform_driver repus_backlight_driver = 
{
	.probe		= repus_backlight_probe,
	.remove		= repus_backlight_remove,
	.driver		= 
	{
		.name		= "repus-backlight",
		.owner		= THIS_MODULE,
	},
};

static int __init repus_backlight_init(void)
{
	unsigned bl_reset[3] =   {BD6091_ADDR,0x00,0x01};
	
	// for ALC (Auto Luminous Control)
	unsigned bl_on[3] =   {BD6091_ADDR,0x01,0x11};
	//unsigned bl_on[3] =   {BD6091_ADDR,0x01,0x117};
	unsigned bl_current[3] =   {BD6091_ADDR,0x03,0x4F};
	unsigned bl_off[3] =  {BD6091_ADDR,0x01,0x10};
	unsigned bl_transition[3] =   {BD6091_ADDR,0x08,0x00};
	                      
	gpio_direction_output(26, 0);
	panel_backlight_set_with_i2c_gpio(bl_transition,3);
	panel_backlight_set_with_i2c_gpio(bl_on,3);
	
	// for ALC (Auto Luminous Control)
	//panel_backlight_set_with_i2c_gpio(bl_current,3);
	
	return platform_driver_register(&repus_backlight_driver);
}

static void __exit repus_backlight_exit(void)
{
	platform_driver_unregister(&repus_backlight_driver);
}

static struct platform_device repus_backlight = {
	.name		= "repus-backlight",
};

module_init(repus_backlight_init);
module_exit(repus_backlight_exit);


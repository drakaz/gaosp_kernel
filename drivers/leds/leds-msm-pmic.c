/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009 QUALCOMM USA, INC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <mach/pmic.h>

#if defined(CONFIG_SAMSUNG_BEHOLD2)
#include <mach/vreg.h>

static struct vreg *vreg_keypad_bl;	

#define _DEBUG 1

#ifdef _DEBUG
#define dprintk(s, args...) printk("[keypad_bl] %s:%d - " s, __func__, __LINE__,  ##args)
#else
#define dprintk(s, args...)
#endif  /* _DEBUG */

#endif

#define MAX_KEYPAD_BL_LEVEL	16

static void msm_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret;
#if defined(CONFIG_SAMSUNG_BEHOLD2)
       // dprintk("msm_keypad_bl_led_set:%d\n", vreg_keypad_bl);

       if( value == LED_HALF || value == LED_FULL )
       {
       		dprintk("[LED] msm_keypad_bl_led_set()	---- (Enable) ---- in leds-msm-pmic.c  (value:%d)\n\n", value);
		ret = vreg_enable(vreg_keypad_bl);
		if (ret)
			dprintk("vreg_enable failed!\n");	   
       }
	else
	{
      		dprintk("[LED] msm_keypad_bl_led_set()	---- (Disable) ---- in leds-msm-pmic.c 	(value:%d) \n\n", value);
		ret = vreg_disable(vreg_keypad_bl);
		if (ret)
			dprintk("vreg_disable failed!\n");	   
	}
#else
	ret = set_led_intensity(LED_KEYPAD, value / MAX_KEYPAD_BL_LEVEL);
	if (ret)
		dev_err(led_cdev->dev, "can't set keypad backlight\n");
#endif

}

static struct led_classdev msm_kp_bl_led = {
//	.name			= "keyboard-backlight",
	.name			= "button-backlight",
	.brightness_set		= msm_keypad_bl_led_set,
	.brightness		= LED_OFF,
};

static int msm_pmic_led_probe(struct platform_device *pdev)
{
#if defined(CONFIG_SAMSUNG_BEHOLD2)
       dprintk("msm_pmic_led_probe\n");

       vreg_set_level(vreg_keypad_bl, 3000); // set voltage to 3.0V
#endif
	return led_classdev_register(&pdev->dev, &msm_kp_bl_led);
}

static int __devexit msm_pmic_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&msm_kp_bl_led);

	return 0;
}

#ifdef CONFIG_PM
static int msm_pmic_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&msm_kp_bl_led);

	return 0;
}

static int msm_pmic_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&msm_kp_bl_led);

	return 0;
}
#else
#define msm_pmic_led_suspend NULL
#define msm_pmic_led_resume NULL
#endif

static struct platform_driver msm_pmic_led_driver = {
	.probe		= msm_pmic_led_probe,
	.remove		= __devexit_p(msm_pmic_led_remove),
	.suspend	= msm_pmic_led_suspend,
	.resume		= msm_pmic_led_resume,
	.driver		= {
		.name	= "pmic-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_pmic_led_init(void)
{
#if defined(CONFIG_SAMSUNG_BEHOLD2)
       int ret = 0;

       dprintk("msm_pmic_led_init\n");
       vreg_keypad_bl = vreg_get(0, "gp1");       
#endif

	return platform_driver_register(&msm_pmic_led_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&msm_pmic_led_driver);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("MSM PMIC LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-leds");

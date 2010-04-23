/* linux/arch/arm/mach-msm/board-halibut.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * Copyright (c) 2008-2009 QUALCOMM Incorporated.
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
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <linux/mmc/sdio_ids.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#include <asm/system.h>


#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_hsusb.h>
#include <mach/vreg.h>
#include <mach/msm_rpcrouter.h>
#include <mach/memory.h>
#include <mach/camera.h>

#include <linux/i2c.h> // KHG_AL04
#include <linux/i2c-gpio.h> // KHG_AL04

#include "../socinfo.h"
#include "../devices.h"
#include "../pm.h"
#include "../proc_comm.h"
#include "board-galaxy.h"

#include <mach/galaxy.h>

//#define CONFIG_MSM7K_SMI64 //by Anubis

extern int galaxy_init_mmc(void);

/* added by gtuo.park for KERNEL_PANIC_DUMP  */
#define MSM_KERNEL_PANIC_DUMP_SIZE 0x8000 /* 32kbytes */
void *MSM_KERNEL_PANIC_DUMP_ADDR;
/* end */

int new_board_revison_chk=1;
static void bcm4325_init(void); //added for bcm4325
void init_keypad(void);

static struct msm_pmem_setting pmem_setting_32 = {
	.pmem_start = SMI32_MSM_PMEM_MDP_BASE,
	.pmem_size = SMI32_MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = SMI32_MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = SMI32_MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = SMI32_MSM_PMEM_GPU1_BASE,
	.pmem_gpu1_size = SMI32_MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = SMI32_MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = SMI32_MSM_PMEM_CAMERA_SIZE,
	.pmem_kernel_ebi1_start = 0, 
	.pmem_kernel_ebi1_size = 0,
	.ram_console_start = 0,
	.ram_console_size = 0,
};

static struct msm_pmem_setting pmem_setting_64 = {
	.pmem_start = SMI64_MSM_PMEM_MDP_BASE,
	.pmem_size = SMI64_MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = SMI64_MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = SMI64_MSM_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size = MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = 0,	// will be allocated
	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,
	.pmem_camera_start = SMI64_MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = SMI64_MSM_PMEM_CAMERA_SIZE,
	.pmem_kernel_ebi1_start = 0, // will be allocated 
	.pmem_kernel_ebi1_size = MSM_PMEM_KERNEL_EBI1_SIZE,
	.ram_console_start = 0,
	.ram_console_size = 0,
};

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(HEADSET, 2),
	SND(STEREO_HEADSET, 3),
	SND(HEADSET_AND_SPEAKER, 3),
	SND(SPEAKER, 6),
	SND(SPEAKER_MIDI, 26),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(CURRENT, 28),
};
#undef SND

static struct msm_snd_endpoints galaxy_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device galaxy_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &galaxy_snd_endpoints
	},
};


static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(45),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};

static char *msm_fb_vreg[] = {
};              
        
#define MSM_FB_VREG_OP(name, op)                                        \
        do {vreg = vreg_get(0, name);                                   \
        if (vreg_##op(vreg))                                            \
                printk(KERN_ERR "%s: %s vreg operation failed \n",      \
                (vreg_##op == vreg_enable) ? "vreg_enable" : "vreg_disable",\
                name); } while (0)
        
static void msm_fb_mddi_power_save(int on)
{               
        struct vreg *vreg;
        int i;

        for (i = 0; i < ARRAY_SIZE(msm_fb_vreg); i++) {
                if (on) 
                        MSM_FB_VREG_OP(msm_fb_vreg[i], enable);
                else
                        MSM_FB_VREG_OP(msm_fb_vreg[i], disable);
        }
}               
        
static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};

static struct mddi_platform_data mddi_pdata = {
        .mddi_power_save = msm_fb_mddi_power_save,
};

static struct resource msm_fb_resources[] = {
	{
		.start = SMI64_MSM_FB_BASE,
		.end = SMI64_MSM_FB_BASE + SMI64_MSM_FB_SIZE - 1,
		.flags  = IORESOURCE_DMA,
	}
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
};

static struct i2c_gpio_platform_data sensor_i2c_gpio_data = {
	.sda_pin	= SENSOR_SDA,
	.scl_pin	= SENSOR_SCL,
};

static struct i2c_gpio_platform_data amp_i2c_gpio_data = {
	.sda_pin	= AUDIO_AMP_SDA,
	.scl_pin	= AUDIO_AMP_SCL,
//	.udelay = 5,
};

static struct i2c_board_info            cam_pm_lp8720_i2c_devices[] = {
    {
        I2C_BOARD_INFO("cam_pm_lp8720_i2c", 0x7D),
    },
};

static struct i2c_gpio_platform_data    cam_pm_lp8720_i2c_gpio_data = {
    .sda_pin    = CAM_PM_LP8720_SDA,
    .scl_pin    = CAM_PM_LP8720_SCL,
};

static struct platform_device           cam_pm_lp8720_i2c_gpio_device = {
    .name       = "i2c-gpio",
    .id         =  4,
    .dev        = {
                  .platform_data = &cam_pm_lp8720_i2c_gpio_data,
                  },
};

//////////// PGH START : CAMERA SENSOR ////////////////////////////////////
static struct i2c_gpio_platform_data cam_i2c_gpio_data = {
	.sda_pin	= CAM_5M_SDA,
	.scl_pin	= CAM_5M_SCL,
//	.udelay = 5,
};

static struct platform_device cam_i2c_gpio_device = {
	.name		= "i2c-gpio",
	.id		= 3,
	.dev		= {
		.platform_data	= &cam_i2c_gpio_data,
	},
};

static struct i2c_board_info cam_i2c_devices[] = {
	{
#if defined(CONFIG_SENSOR_M4MO)
		I2C_BOARD_INFO("m4mo", 0x3F>>1),
#endif//PGH
	},
};
//////////// PGH END : CAMERA SENSOR //////////////////////////////////////

static struct platform_device galaxy_rfkill = {
	.name = "galaxy_rfkill",
	.id = 0,
};

static struct platform_device bluetooth_lpm_device = {
	.name = "bluetooth_lpm",
	.id = 1,
};


static struct platform_device sensor_i2c_gpio_device = {	 /*chris@bk21,smb380 */
	.name		= "i2c-gpio",
	.id		= 1,
	.dev		= {
		.platform_data	= &sensor_i2c_gpio_data,
	},
};

static struct platform_device amp_i2c_gpio_device = {
	.name		= "i2c-gpio",
	.id		= 2,
	.dev		= {
		.platform_data	= &amp_i2c_gpio_data,
	},
};

static struct i2c_gpio_platform_data touch_i2c_gpio_data = {
	.sda_pin	= INPUT_TOUCH_SDA,
	.scl_pin	= INPUT_TOUCH_SCL,
	.udelay = 5,
};

static struct platform_device touch_i2c_gpio_device = {
	.name		= "i2c-gpio",
	.id		= 5,
	.dev		= {
		.platform_data	= &touch_i2c_gpio_data,
	},
};

extern struct sys_timer msm_timer;

static void __init galaxy_init_irq(void)
{
	msm_init_irq();
}

void msm_serial_debug_init(unsigned int base, int irq,
				struct device *clk_device, int signal_irq);

static struct i2c_board_info touch_i2c_devices [] = {
#if defined(CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI)
       {
		I2C_BOARD_INFO("melfas-tsi-ts", 0x20),
		.irq = MSM_GPIO_TO_INT( 19 ),
       },
#elif defined(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_SEC)
       {
		I2C_BOARD_INFO("synaptics-rmi-ts", 0x20),
		.irq = MSM_GPIO_TO_INT( 19 ),
       },
#endif
};
static struct i2c_board_info sensor_i2c_devices[] = {
    {
            I2C_BOARD_INFO("bma150", 0x38),
    },
    {
            I2C_BOARD_INFO("akm8973", 0x3C>>1 ),
    },

    {   
            I2C_BOARD_INFO("proximity_i2c" ,0x88>>1),
            .irq = MSM_GPIO_TO_INT( PROXIMITY_SENSOR_INT ),
    },    
};

static struct i2c_board_info gpio_i2c_devices[] = {
    {
            I2C_BOARD_INFO("max9877",0x9A>>1),
    },
    {
            I2C_BOARD_INFO("fsa9480",0x4A>>1),
    },
};


static uint32_t new_board_gpio_table[] = {
	GPIO_CFG(38,  0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* SEND_END */
	GPIO_CFG(20,  0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* T_FLASH_DET */
	GPIO_CFG(85,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* WLAN_BT_REG_ON */
	GPIO_CFG(109, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* BT_RESET */
};


static uint32_t old_board_gpio_table[] = {
	GPIO_CFG(38,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* BT_RESET */
	GPIO_CFG(20,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* WLAN_BT_REG_ON */
	GPIO_CFG(85,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* CAM_STBY */
	GPIO_CFG(109, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* SEND_END */
};

static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */
//	GPIO_CFG(23, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	// PGH CAM_FLASH_EN
//	GPIO_CFG(31, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	// PGH CAM_FLASH_SET
	GPIO_CFG(36, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(37, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(60, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(61, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),

};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */

#if 1//PGH EDITED FOR CLOCK Stabilization 2009-05-23
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* MCLK */
#else//ORG
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
#endif//PGH

//	GPIO_CFG(23, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),	// PGH CAM_FLASH_EN
//	GPIO_CFG(31, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),	// PGH CAM_FLASH_SET

	GPIO_CFG(36, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
	GPIO_CFG(37, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),

	GPIO_CFG(60, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
	GPIO_CFG(61, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),

};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

#define MSM_PROBE_INIT(name) name##_probe_init
static struct msm_camera_sensor_info msm_camera_sensor[2] = {
	{
#if defined(CONFIG_SENSOR_AIT848)
		.sensor_reset = 17,
		.sensor_pwd	  = 85,
		.vcm_pwd      = 0,
		.sensor_name  = "ait848",
		.flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_MSM_CAMERA
		.sensor_probe = MSM_PROBE_INIT(ait848),
#endif
#elif defined(CONFIG_SENSOR_M4MO)
		.sensor_reset = 17,
		.sensor_pwd	  = 85,
		.vcm_pwd      = 0,
		.sensor_name  = "m4mo",
		.flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_MSM_CAMERA
		.sensor_probe = MSM_PROBE_INIT(m4mo),
#endif

#else //ORG
		.sensor_reset   = 89,
		.sensor_pwd	  = 85,
		.vcm_pwd      = 0,
		.sensor_name  = "mt9d112",
		.flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_MSM_CAMERA
//		.sensor_probe = MSM_PROBE_INIT(mt9d112),
#endif
#endif//PGH
	},
	{
		.sensor_reset   = 89,
		.sensor_pwd	  = 85,
		.vcm_pwd      = 0,
		.sensor_name  = "mt9t013",
		.flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_MSM_CAMERA
//		.sensor_probe = MSM_PROBE_INIT(mt9t013),
#endif
	},
};
#undef MSM_PROBE_INIT

static void galaxy_phy_reset(void)
{
	return;
}

static void config_new_board_gpios(void)
{
	config_gpio_table(new_board_gpio_table,
		ARRAY_SIZE(new_board_gpio_table));
}

static void config_old_board_gpios(void)
{
	config_gpio_table(old_board_gpio_table,
		ARRAY_SIZE(old_board_gpio_table));
}

static void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}


static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.snum = ARRAY_SIZE(msm_camera_sensor),
	.sinfo = &msm_camera_sensor[0],
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static void __init msm_camera_add_device(void)
{
	msm_camera_register_device(NULL, 0, &msm_camera_device_data);
	config_camera_off_gpios();
}

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
        msm_fb_register_device("pmdh", &mddi_pdata);
}

#if 0//PGH
static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
};
#endif

static void __init msm_device_i2c_init(void)
{
//PGH	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static unsigned bt_config_uart[] = {
	GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),	/* RFR */
	GPIO_CFG(44, 2, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),	/* CTS */
	GPIO_CFG(45, 2, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),	/* Rx */
	GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),	/* Tx */
};

static void bcm4325_init(void)	// added for bcm4325
{
	int rc;
	int pin;

	for (pin = 0; pin < ARRAY_SIZE(bt_config_uart); pin++) {
		rc = gpio_tlmm_config(bt_config_uart[pin], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_uart[pin], rc);
			return -EIO;
		}
	}

	gpio_direction_output(BCM4325_BT_WAKE, 0);       // BT_WAKE_N
#if !defined(CONFIG_MACH_GALAXY_REV05)				// REV02, REV03 has WLAN_WAKE
	gpio_direction_output(BCM4325_WLAN_WAKE, 0);       // BT_WAKE_N
#endif
	
	msleep(100);
	
	gpio_direction_output(BCM4325_WLAN_RESET, 0);       // WLAN
	gpio_direction_output(BCM4325_BT_RESET, 0);       // BT

#if !defined(CONFIG_MACH_GALAXY_REV03)  // for REG_ON pin - REV02, REV05 has REG_ON
	gpio_direction_output(GPIO_WLAN_BT_REG_ON, 0);       // REG_ON
#endif

//	msleep(500);	// dh0421

#if !defined(CONFIG_MACH_GALAXY_REV03)  // for REG_ON pin - REV02, REV05 has REG_ON
//	gpio_direction_output(GPIO_WLAN_BT_REG_ON, 1);       // REG_ON
//	Power-up Sequence for BT off and WIFI off is
//	VBAT & VDDIO is up
//	REG_ON, WL_RESET and BT_RESET is low
	
//	msleep(150);
	
	gpio_direction_output(BCM4325_WLAN_RESET, 0);       // WLAN
//	gpio_direction_output(BCM4325_BT_RESET, 1);       // BT -> on
	gpio_direction_output(BCM4325_BT_RESET, 0);       // BT -> off
#else
	gpio_direction_output(BCM4325_WLAN_RESET, 0);       // WLAN
	msleep(150);
	gpio_direction_output(BCM4325_BT_RESET, 1);       // BT
#endif

}

static struct platform_device *devices[] __initdata = {
	&msm_device_uart3,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&galaxy_snd,
	&msm_fb_device,
	&sensor_i2c_gpio_device,
	&amp_i2c_gpio_device,
	&galaxy_rfkill,
	&bluetooth_lpm_device,
	&cam_i2c_gpio_device,
	&touch_i2c_gpio_device,
	&cam_pm_lp8720_i2c_gpio_device,  //PGH CAM_PM_LP8720 
};

int galaxy_get_smi_size(void)
{
#if defined(CONFIG_MSM7K_SMI64)
	return 64;
#else
	return 32;
#endif
}

/* All 7x01 2.0 based boards are expected to have RAM chips capable of 160 MHz. */
static struct msm_acpu_clock_platform_data galaxy_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
	.wait_for_irq_khz = 128000000,
	.max_axi_khz=128000, 
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
};

static void __init galaxy_init(void)
{
	if (socinfo_init() < 0)
		BUG();

#if defined (CONFIG_MSM7K_SMI64)
	msm_add_mem_devices(&pmem_setting_64);
#else
	if (galaxy_get_smi_size() == 32 )
		msm_add_mem_devices(&pmem_setting_32);
	else
		msm_add_mem_devices(&pmem_setting_64);
#endif

	init_keypad();

#if 0	// HW rev0.6 -> fix : new_board_revison_chk is 1
    if( gpio_get_value(GPIO_CHK_BOARD_REV) == 1 )
    {
        printk("[%s:%d] OLD BOARD(51Ohm_100nF)\n",__func__,__LINE__);
//		config_new_board_gpios();

        new_board_revison_chk = 1;
    }
    else
    {
        printk("[%s:%d] NEW BOARD(100Ohm_220nF)\n",__func__,__LINE__);
//		config_old_board_gpios();

        new_board_revison_chk = 0;
    }
#else
	new_board_revison_chk = 1;
	config_new_board_gpios();
#endif

	galaxy_init_mmc();	// SD Card Power Issue

	/* All 7x01 2.0 based boards are expected to have RAM chips capable
	 * of 160 MHz. */
	if (cpu_is_msm7x01()
	    && SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2)
		galaxy_clock_data.max_axi_khz = 160000;

	msm_acpu_clock_init(&galaxy_clock_data);

	msm_device_uart_dm1.dev.platform_data = NULL;

	platform_add_devices(devices, ARRAY_SIZE(devices));
        msm_device_i2c_init();

	i2c_register_board_info(3, cam_i2c_devices,ARRAY_SIZE(cam_i2c_devices));		// CAMERA
	i2c_register_board_info(1, sensor_i2c_devices,ARRAY_SIZE(sensor_i2c_devices));
	i2c_register_board_info(2, gpio_i2c_devices,ARRAY_SIZE(gpio_i2c_devices));
	i2c_register_board_info(4, cam_pm_lp8720_i2c_devices, ARRAY_SIZE(cam_pm_lp8720_i2c_devices));
	i2c_register_board_info(5, touch_i2c_devices,ARRAY_SIZE(touch_i2c_devices));				// TOUCH

	msm_add_usb_devices(galaxy_phy_reset);

	msm_fb_add_devices();
    	msm_camera_add_device();    
	bcm4325_init();	// added for bcm4325

	msm_pm_set_platform_data(msm_pm_data);

}

extern struct android_pmem_platform_data android_pmem_gpu1_pdata;
extern struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata;
static void __init msm_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = MSM_KERNEL_PANIC_DUMP_SIZE;
	addr = alloc_bootmem(size);
	MSM_KERNEL_PANIC_DUMP_ADDR = addr;

	size = MSM_PMEM_KERNEL_EBI1_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	android_pmem_kernel_ebi1_pdata.start = __pa(addr);
	android_pmem_kernel_ebi1_pdata.size = size;
	pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
		" ebi1 pmem arena\n", size, addr, __pa(addr));

	size = MSM_PMEM_GPU1_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	android_pmem_gpu1_pdata.start = __pa(addr);
	android_pmem_gpu1_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for gpu1 pmem\n", size, addr, __pa(addr));
}

static void __init galaxy_map_io(void)
{
	msm_map_common_io();
	msm_allocate_memory_regions();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

MACHINE_START(GALAXY, "Samsung GT-I7500 Board")
/* UART for LL DEBUG */
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif

	.boot_params	= 0x10000100,
	.map_io		= galaxy_map_io,
	.init_irq		= galaxy_init_irq,
	.init_machine	= galaxy_init,
	.timer		= &msm_timer,
MACHINE_END

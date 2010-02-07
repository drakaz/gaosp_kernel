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
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_hsusb.h>
#include <mach/vreg.h>
#include <mach/msm_rpcrouter.h>
#include <mach/memory.h>
#include <mach/camera.h>

#ifdef CONFIG_USB_FUNCTION
#include <linux/usb/mass_storage_function.h>
#endif
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#include <mach/rpc_hsusb.h>
#endif

#include <linux/i2c.h> // KHG_AL04
#include <linux/i2c-gpio.h> // KHG_AL04

#include "devices.h"
#include "socinfo.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#include "proc_comm.h"

static DEFINE_MUTEX(bcm4325_pwr_lock);
#define BCM4325_BT 0
#define BCM4325_WLAN 1

#ifdef CONFIG_MSM_STACKED_MEMORY
#define MSM_SMI_BASE		0x100000
#define MSM_SMI_SIZE		0x800000

#define MSM_PMEM_GPU0_BASE	MSM_SMI_BASE
#define MSM_PMEM_GPU0_SIZE	0x800000
#endif
/* added by gtuo.park for KERNEL_PANIC_DUMP  */
#define MSM_KERNEL_PANIC_DUMP_SIZE 0x8000 /* 32kbytes */
void *MSM_KERNEL_PANIC_DUMP_ADDR;
/* end */

#define MSM_EBI_BASE		0x10000000
#define MSM_EBI_SIZE		0x06D00000				// Total 109M for ARM11

#ifdef CONFIG_MSM7K_SMI64
#define SMI64_MSM_PMEM_MDP_BASE	0x02000000
#define SMI64_MSM_PMEM_MDP_SIZE	0x00800000		// 8M

#define SMI64_MSM_PMEM_ADSP_BASE    0x02800000
#define SMI64_MSM_PMEM_ADSP_SIZE 	0x00D00000		// 13M-->7M from IF4 => 8M for MPEG4 Play 480*360 ==> 13M for safety

#define SMI64_MSM_PMEM_CAMERA_BASE	0x03500000
#define SMI64_MSM_PMEM_CAMERA_SIZE	0x00B00000		// 11M
#endif

#define MSM_PMEM_MDP_SIZE	0x800000
#define MSM_PMEM_CAMERA_SIZE	0xa00000
#define MSM_PMEM_ADSP_SIZE	0xd00000
#define MSM_PMEM_GPU1_SIZE	0x800000
#define MSM_FB_SIZE		0x100000					// 1M is enough for Orion Project

#define SENSOR_SCL	2
#define SENSOR_SDA	3

#if defined(CONFIG_SAMSUNG_CAPELA)
#define AUDIO_AMP_SCL 82
#define AUDIO_AMP_SDA 83

#define INPUT_TOUCH_SCL	29
#define INPUT_TOUCH_SDA	30

#define T_FLASH_DETECT	20
#endif

int new_board_revison_chk=1;

static void bcm4325_init(void); //added for bcm4325
//Proximity Sensor
#define PROXIMITY_SENSOR_INT 57

#define SENSOR_RESET 108


#define		CAM_5M_SCL	60
#define		CAM_5M_SDA	61

void init_keypad(void);

#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
#if 0
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
	.release        = 0xffff,
#else
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "Samsung",
	.product        = "SAMSUNG Android Mass storage",
	.release        = 0x0100,
#endif
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.product_id	= 0xF000,
	.adb_product_id	= 0x9015,
	.version	= 0x0100,
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
	.nluns = 1,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#ifdef CONFIG_USB_FUNCTION
static void hsusb_gpio_init(void)
{
	if (gpio_request(111, "ulpi_data_0"))
		pr_err("failed to request gpio ulpi_data_0\n");
	if (gpio_request(112, "ulpi_data_1"))
		pr_err("failed to request gpio ulpi_data_1\n");
	if (gpio_request(113, "ulpi_data_2"))
		pr_err("failed to request gpio ulpi_data_2\n");
	if (gpio_request(114, "ulpi_data_3"))
		pr_err("failed to request gpio ulpi_data_3\n");
	if (gpio_request(115, "ulpi_data_4"))
		pr_err("failed to request gpio ulpi_data_4\n");
	if (gpio_request(116, "ulpi_data_5"))
		pr_err("failed to request gpio ulpi_data_5\n");
	if (gpio_request(117, "ulpi_data_6"))
		pr_err("failed to request gpio ulpi_data_6\n");
	if (gpio_request(118, "ulpi_data_7"))
		pr_err("failed to request gpio ulpi_data_7\n");
	if (gpio_request(119, "ulpi_dir"))
		pr_err("failed to request gpio ulpi_dir\n");
	if (gpio_request(120, "ulpi_next"))
		pr_err("failed to request gpio ulpi_next\n");
	if (gpio_request(121, "ulpi_stop"))
		pr_err("failed to request gpio ulpi_stop\n");
}

static unsigned usb_gpio_lpm_config[] = {
	GPIO_CFG(111, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 0 */
	GPIO_CFG(112, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 1 */
	GPIO_CFG(113, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 2 */
	GPIO_CFG(114, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 3 */
	GPIO_CFG(115, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 4 */
	GPIO_CFG(116, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 5 */
	GPIO_CFG(117, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 6 */
	GPIO_CFG(118, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DATA 7 */
	GPIO_CFG(119, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* DIR */
	GPIO_CFG(120, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),	/* NEXT */
	GPIO_CFG(121, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* STOP */
};

static unsigned usb_gpio_lpm_unconfig[] = {
	GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 0 */
	GPIO_CFG(112, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 1 */
	GPIO_CFG(113, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 2 */
	GPIO_CFG(114, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 3 */
	GPIO_CFG(115, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 4 */
	GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 5 */
	GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 6 */
	GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DATA 7 */
	GPIO_CFG(119, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DIR */
	GPIO_CFG(120, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* NEXT */
	GPIO_CFG(121, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), /* STOP */
};

static int usb_config_gpio(int config)
{
	int pin, rc;

	if (config) {
		for (pin = 0; pin < ARRAY_SIZE(usb_gpio_lpm_config); pin++) {
			rc = gpio_tlmm_config(usb_gpio_lpm_config[pin],
					      GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, usb_gpio_lpm_config[pin], rc);
				return -EIO;
			}
		}
	} else {
		for (pin = 0; pin < ARRAY_SIZE(usb_gpio_lpm_unconfig); pin++) {
			rc = gpio_tlmm_config(usb_gpio_lpm_unconfig[pin],
					      GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, usb_gpio_lpm_config[pin], rc);
				return -EIO;
			}
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"modem", 0},
	{"diag", 1},
	{"mass_storage", 2},
	{"adb", 3},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
        {
                .product_id     = 0x6601, //by gtuo.park
                .functions      = 0x7, /* 0111 modem, diag, ums */
        },      
        
        {
                .product_id     = 0x6640, // by gtuo.park
                .functions      = 0xF,  /* 1111 modem, diag, ums, adb */
        },      
 
        {
                .product_id     = 0x6603, // by gtuo.park
                .functions      = 0x4,  /* ums only*/
        },

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9015,
		.functions          = 0x12, /* 10010 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0xD00D,
		.functions	    = 0x1F, /* 011111 */
	},

	{
		.product_id         = 0x901A,
		.functions          = 0x0F, /* 01111 */
	},

};
#endif

#ifdef CONFIG_USB_ANDROID
static void hsusb_phy_reset(void)
{
	msm_hsusb_phy_reset();
}
#endif

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_ANDROID
	.phy_reset	= hsusb_phy_reset,
#endif
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= USB_PHY_EXTERNAL,
	.vendor_id			= 0x04E8, //by gtuo.park SAMSUNG vendor ID
	.product_name       = "Samsung Android USB Device",
	.serial_number      = "GT-I7500",
	.manufacturer_name  = "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.ulpi_data_1_pin = 112,
	.ulpi_data_3_pin = 114,
	.config_gpio 	= usb_config_gpio,
#endif
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

static struct msm_snd_endpoints halibut_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device halibut_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &halibut_snd_endpoints
	},
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
#ifdef CONFIG_MSM7K_SMI64
	.start = SMI64_MSM_PMEM_MDP_BASE,
	.size = SMI64_MSM_PMEM_MDP_SIZE,
#endif
	.no_allocator = 1,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name = "pmem_camera",
#ifdef CONFIG_MSM7K_SMI64
	.start = SMI64_MSM_PMEM_CAMERA_BASE,
	.size = SMI64_MSM_PMEM_CAMERA_SIZE,
#endif
	.no_allocator = 1,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
#ifdef CONFIG_MSM7K_SMI64
	.start = SMI64_MSM_PMEM_ADSP_BASE,
	.size = SMI64_MSM_PMEM_ADSP_SIZE,
#endif
	.no_allocator = 0,
	.cached = 0,
};

#ifdef CONFIG_MSM_STACKED_MEMORY
static struct android_pmem_platform_data android_pmem_gpu0_pdata = {
	.name = "pmem_gpu0",
	.start = MSM_PMEM_GPU0_BASE,
	.size = MSM_PMEM_GPU0_SIZE,
	.no_allocator = 1,
	.cached = 0,
	.buffered = 1,
};
#endif

static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.no_allocator = 1,
	.cached = 0,
	.buffered = 1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_camera_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

#ifdef CONFIG_MSM_STACKED_MEMORY
static struct platform_device android_pmem_gpu0_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_gpu0_pdata },
};
#endif

static struct platform_device android_pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_gpu1_pdata },
};

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(45),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};

static char *msm_fb_vreg[] = {
//	"gp5"           
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
        
#define PM_VID_EN_CONFIG_PROC          24
#define PM_VID_EN_API_PROG             0x30000061
#define PM_VID_EN_API_VERS             0x00010001 
        
static struct msm_rpc_endpoint *pm_vid_en_ep;
        
static int msm_fb_pm_vid_en(int on)
{
        int rc = 0;
        struct msm_fb_pm_vid_en_req {
                struct rpc_request_hdr hdr;
                uint32_t on;
        } req;
        
        pm_vid_en_ep = msm_rpc_connect(PM_VID_EN_API_PROG,
                                        PM_VID_EN_API_VERS, 0);
        if (IS_ERR(pm_vid_en_ep)) {
                printk(KERN_ERR "%s: msm_rpc_connect failed! rc = %ld\n",
                        __func__, PTR_ERR(pm_vid_en_ep));
                return -EINVAL;
        }

        req.on = cpu_to_be32(on);
        rc = msm_rpc_call(pm_vid_en_ep,
                        PM_VID_EN_CONFIG_PROC,
                        &req, sizeof(req),
                        5 * HZ);
        if (rc)
                printk(KERN_ERR
                        "%s: msm_rpc_call failed! rc = %d\n", __func__, rc);
                
        msm_rpc_close(pm_vid_en_ep);
        return rc;
} 

static struct mddi_platform_data mddi_pdata = {
        .mddi_power_save = msm_fb_mddi_power_save,
};

static struct resource msm_fb_resources[] = {
	{
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


#if defined(CONFIG_SAMSUNG_CAPELA) 

#define     CAM_PM_LP8720_SCL           36
#define     CAM_PM_LP8720_SDA           37


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
#endif//


#if defined(CONFIG_SAMSUNG_CAPELA) 
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
#endif






#if defined(CONFIG_SAMSUNG_CAPELA)
static struct i2c_board_info cam_i2c_devices[] = {
    {
#if defined(CONFIG_SENSOR_AIT848)
            I2C_BOARD_INFO("ait848",0x03>>1),
#elif defined(CONFIG_SENSOR_M4MO)
			I2C_BOARD_INFO("m4mo", 0x3F>>1),
#else
            I2C_BOARD_INFO("mt9d112",0x78>>1),
#endif
    },
	{
            I2C_BOARD_INFO("mt9t013",0x6C>>1),
	},
};
#endif


static struct platform_device orion_rfkill = {
	.name = "orion_rfkill",
	.id = 0,
};

static struct platform_device orion_rfkill1 = {
	.name = "orion_rfkill1",
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

#if defined(CONFIG_SAMSUNG_CAPELA)
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
#endif

static struct platform_device *devices[] __initdata = {
	&msm_device_uart3,
	&msm_device_uart_dm1,
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_tssc,
	&android_pmem_camera_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
#ifdef CONFIG_MSM_STACKED_MEMORY
	&android_pmem_gpu0_device,
#endif
	&android_pmem_gpu1_device,
	&msm_device_hsusb_otg,
	&msm_device_hsusb_host,
#if defined(CONFIG_USB_FUNCTION) || defined(CONFIG_USB_ANDROID)
	&msm_device_hsusb_peripheral,
#endif
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#endif

	&halibut_snd,
	//&msm_bluesleep_device,
	&msm_fb_device,
	&sensor_i2c_gpio_device,
	&amp_i2c_gpio_device,
	&orion_rfkill,
	&orion_rfkill1,


	&cam_i2c_gpio_device,
    &cam_pm_lp8720_i2c_gpio_device,  

#if defined(CONFIG_SAMSUNG_CAPELA)
	&touch_i2c_gpio_device,
#endif
};

extern struct sys_timer msm_timer;

static void __init init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data halibut_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
	.wait_for_irq_khz = 128000000,
	.max_axi_khz=128000, 
};

void msm_serial_debug_init(unsigned int base, int irq,
				struct device *clk_device, int signal_irq);
#if 0
static struct akm8976_platform_data compass_platform_data={ //chris_1222
	.reset =SENSOR_RESET,
//	.clk_on = 0,    //for compile, dont necessary
	.intr =SENSOR_INT_1,
};
#endif
static struct i2c_board_info touch_i2c_devices [] = {
#if defined(CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI)
       {
		I2C_BOARD_INFO("melfas-tsi-ts", 0x20),
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
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* MCLK */
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
		.sensor_reset = 17,
		.sensor_pwd	  = 85,
		.vcm_pwd      = 0,
		.sensor_name  = "m4mo",
		.flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_MSM_CAMERA
		.sensor_probe = MSM_PROBE_INIT(m4mo),
#endif
	},
	{
		.sensor_reset   = 89,
		.sensor_pwd	  = 85,
		.vcm_pwd      = 0,
		.sensor_name  = "mt9t013",
		.flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_MSM_CAMERA
#endif
	},
};
#undef MSM_PROBE_INIT

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

static void sdcc_gpio_init(void)
{
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	int rc = 0;
	if (gpio_request(T_FLASH_DETECT, "sdc1_status_irq"))
		pr_err("failed to request gpio sdc1_status_irq\n");
	rc = gpio_tlmm_config(GPIO_CFG(T_FLASH_DETECT, 0, GPIO_INPUT, GPIO_PULL_UP,
				GPIO_2MA), GPIO_ENABLE);
	if (rc)
		printk(KERN_ERR "%s: Failed to configure GPIO %d\n",
				__func__, rc);
#endif
	/* SDC1 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (gpio_request(51, "sdc1_data_3"))
		pr_err("failed to request gpio sdc1_data_3\n");
	if (gpio_request(52, "sdc1_data_2"))
		pr_err("failed to request gpio sdc1_data_2\n");
	if (gpio_request(53, "sdc1_data_1"))
		pr_err("failed to request gpio sdc1_data_1\n");
	if (gpio_request(54, "sdc1_data_0"))
		pr_err("failed to request gpio sdc1_data_0\n");
	if (gpio_request(55, "sdc1_cmd"))
		pr_err("failed to request gpio sdc1_cmd\n");
	if (gpio_request(56, "sdc1_clk"))
		pr_err("failed to request gpio sdc1_clk\n");
#endif

	if (machine_is_msm7201a_ffa())
		return;

	/* SDC2 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (gpio_request(62, "sdc2_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(63, "sdc2_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(64, "sdc2_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(65, "sdc2_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(66, "sdc2_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(67, "sdc2_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");
#endif

	/* SDC3 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	if (gpio_request(88, "sdc3_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(89, "sdc3_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(90, "sdc3_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(91, "sdc3_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(92, "sdc3_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(93, "sdc3_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");
#endif

#if !defined(CONFIG_SAMSUNG_CAPELA)
	/* SDC4 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	if (gpio_request(19, "sdc4_data_3"))
		pr_err("failed to request gpio sdc4_data_3\n");
	if (gpio_request(20, "sdc4_data_2"))
		pr_err("failed to request gpio sdc4_data_2\n");
	if (gpio_request(21, "sdc4_data_1"))
		pr_err("failed to request gpio sdc4_data_1\n");
	if (gpio_request(107, "sdc4_cmd"))
		pr_err("failed to request gpio sdc4_cmd\n");
	if (gpio_request(108, "sdc4_data_0"))
		pr_err("failed to request gpio sdc4_data_0\n");
	if (gpio_request(109, "sdc4_clk"))
		pr_err("failed to request gpio sdc4_clk\n");
#endif
#endif
}

static unsigned int vreg_enabled;
static unsigned sdcc_cfg_data[][6] = {
	/* SDC1 configs */
	{
	GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
	},
	/* SDC2 configs */
	{
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	},
	{
	/* SDC3 configs */
	GPIO_CFG(88, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(89, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(90, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(91, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(92, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(93, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	},
#if !defined(CONFIG_SAMSUNG_CAPELA)
	/* SDC4 configs */
	{
	GPIO_CFG(19, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(20, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(21, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(107, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(108, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA),
	GPIO_CFG(109, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),
	}
#endif
};

static unsigned long vreg_sts, gpio_sts;
static struct mpp *mpp_mmc;
static struct vreg *vreg_mmc, *vreg_movinand;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int i, rc;
if (dev_id ==1)
{
	if (!(test_bit(dev_id, &gpio_sts)^enable)) {
    		//printk("sdcc_setup_gpio test_bit error....\n");
		return;
	}

	if (enable)
		set_bit(dev_id, &gpio_sts);
	else
		clear_bit(dev_id, &gpio_sts);
}

	for (i = 0; i < ARRAY_SIZE(sdcc_cfg_data[dev_id - 1]); i++) {
		rc = gpio_tlmm_config(sdcc_cfg_data[dev_id - 1][i],
			enable ? GPIO_ENABLE : GPIO_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, sdcc_cfg_data[dev_id - 1][i], rc);
		}
	}
}

static int msm_wifi_setup_power(int dev_id, int on)
{
	printk("\n wifi_setup_power: %d \n", on);
	return 0;
}

//static uint32_t msm_sdcc_setup_power(int dev_id, unsigned int vdd)
static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

//	printk("[MMC] %s @@@@@@\n", __func__);	
	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			if (machine_is_msm7201a_ffa())
				rc = mpp_config_digital_out(mpp_mmc,
				     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
				     MPP_DLOGIC_OUT_CTRL_LOW));
			else
				rc = vreg_disable(vreg_mmc);
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
		if (machine_is_msm7201a_ffa())
			rc = mpp_config_digital_out(mpp_mmc,
			     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			     MPP_DLOGIC_OUT_CTRL_HIGH));
		else {
			rc = vreg_set_level(vreg_mmc, 2850);
			if (!rc)
				rc = vreg_enable(vreg_mmc);
		}
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int halibut_sdcc_slot_status(struct device *dev)
{
	int rc;
	rc = gpio_get_value( T_FLASH_DETECT );

	rc = rc?0:1 ;
	return rc;
}
#endif

#if 0
static int msm_movinand_setup_power(int dev_id, int on)
{
	// KTH
	printk("%s \n", __func__);
	int rc = 0;

	msm_sdcc_setup_gpio(dev_id, on);
	
#if 0
	if ((on && vreg_enabled) || (!on && !vreg_enabled))
		return 0;
#endif

	//unit of mV : 3V
    rc=vreg_set_level(vreg_movinand, 3000);
	if(rc){
		printk(KERN_ERR, "failed to set level in movinand\n");
	}

	rc = on ? vreg_enable(vreg_movinand) : vreg_disable(vreg_movinand);
	if (rc) {
		printk(KERN_ERR "%s: Failed to configure vreg (%d)\n",
				__func__, rc);
		return rc;
	}
	vreg_enabled = on;
	return 0;
}
#else
static uint32_t orion_movinand_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;


	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {

		rc = vreg_disable(vreg_movinand);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		return 0;
	}

	rc = vreg_set_level(vreg_movinand, 3050);
	if (!rc)
		rc = vreg_enable(vreg_movinand);
	else
		printk(KERN_ERR "%s: return val: %d \n",
				__func__, rc);
	return 0;
}
#endif

#if 0
static unsigned int sdcc_slot_status(struct device *dev)
{
	return 1; // Temp...
}
#endif

static struct mmc_platform_data halibut_sdcc_data_rev04 = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status         = halibut_sdcc_slot_status,
	.status_irq	= MSM_GPIO_TO_INT(T_FLASH_DETECT),
	.irq_flags      = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
};

static unsigned int msm_movinand_status(struct device *dev)
{
	return 1; // Movinand is not removable.
}

static struct mmc_platform_data movinand_sdcc_data = {
	.ocr_mask	= MMC_VDD_30_31,
	.translate_vdd	= orion_movinand_setup_power,
};


int bcm4325_set_core_power(unsigned bcm_core, unsigned pow_on)
{
	unsigned gpio_rst, gpio_rst_another;
	
	switch(bcm_core) {
	case BCM4325_BT:
		gpio_rst = BCM4325_BT_RESET;
		gpio_rst_another = BCM4325_WLAN_RESET;
		break;
		
	case BCM4325_WLAN:
		gpio_rst = BCM4325_WLAN_RESET;
		gpio_rst_another = BCM4325_BT_RESET;
		break;
		
	default:
		printk(KERN_ERR "bcm4325_power: Unknown bcm4325 core!\n");
		return -1;
	}

	mutex_lock(&bcm4325_pwr_lock);

#if !defined(CONFIG_MACH_CAPELA_REV03)
	/* if another core is OFF */
	if( gpio_get_value(gpio_rst_another) == 0 )
	{
		/* enable WLAN_BT_REG_ON */
		gpio_direction_output(GPIO_WLAN_BT_REG_ON, pow_on); 
		printk("bcm4325_power: Set WLAN_BT_REG_ON %s because %s is OFF now.\n", 
			gpio_get_value(GPIO_WLAN_BT_REG_ON)?"High":"Low", bcm_core?"BT":"WLAN");
		msleep(150);
	}
#endif
	/* enable specified core */
	gpio_direction_output(gpio_rst, pow_on);
	printk("bcm4325_power: Set %s %s\n", 
		bcm_core?"WLAN_RESET":"BT_RESET", gpio_get_value(gpio_rst)?"High":"Low");

	mutex_unlock(&bcm4325_pwr_lock);

	return 0;
}

EXPORT_SYMBOL(bcm4325_set_core_power);


/*-------------- wifi ----------------*/

#if 1

static uint32_t wifi_on_gpio_table[] = {			// needed to set GPIO. but...
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT3 */
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT2 */
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT1 */
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT0 */
	GPIO_CFG(GPIO_WLAN_HOST_WAKE, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),  /* WLAN_HOST_WAKE	*/
#if !defined(CONFIG_MACH_CAPELA_REV05)
	GPIO_CFG(20, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN_HOST_WAKE	*/
#endif
};

static uint32_t wifi_off_gpio_table[] = {
	GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	GPIO_CFG(64, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	GPIO_CFG(65, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	GPIO_CFG(66, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT1 */
	GPIO_CFG(67, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT0 */
	GPIO_CFG(GPIO_WLAN_HOST_WAKE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),  /* WLAN_HOST_WAKE */
#if !defined(CONFIG_MACH_CAPELA_REV05)
	GPIO_CFG(20, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
#endif
};

static int wifi_cd = 0;           /* WIFI virtual 'card detect' status */
//static struct vreg *vreg_wifi_osc;      /* WIFI 32khz oscilator */

static unsigned int wifi_status(struct device *dev)
{
	return wifi_cd;			// wifi 'card detect' status return
}

static void (*wifi_status_cb)(int card_present, void *dev_id);

static void *wifi_status_cb_devid;

static struct sdio_embedded_func wifi_func = {
	.f_class        = SDIO_CLASS_WLAN,
	.f_maxblksize   = 512,
};

static struct embedded_sdio_data wifi_emb_data = {
	.cis    = {
		.vendor         = 0x02d0,
		.device         = 0x4325,
		.blksize        = 512,
		/*.max_dtr      = 24000000,  Max of chip - no worky on Trout */
		.max_dtr        =   48000000,
	},
	.cccr   = {
		.multi_block    = 1,
		.low_speed      = 1,
		.wide_bus       = 1,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.funcs  = &wifi_func,
	.num_funcs = 2,
};

static int wifi_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
//	printk("%s:%d is called.....\n", __func__, __LINE__);
	if(wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static struct mmc_platform_data wifi_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.status		= wifi_status,
	.register_status_notify	= wifi_status_register,
	.embedded_sdio	= &wifi_emb_data,
	.translate_vdd	= msm_wifi_setup_power,
};

#if 1

void wifi_set_carddetect(int val)
{
	wifi_cd = val;
	if(wifi_status_cb)
	{
		wifi_status_cb(val, wifi_status_cb_devid);
	}
	else
	{
		printk("%s: Nobody to notify\n", __func__);
	}
}
EXPORT_SYMBOL(wifi_set_carddetect);

static int wifi_power_state;


int wifi_card_power(int on)
{
	printk("%s: %d\n", __func__, on);

	if(on)
	{
		config_gpio_table(wifi_on_gpio_table, ARRAY_SIZE(wifi_on_gpio_table));
		bcm4325_set_core_power(BCM4325_WLAN, 1);
	}
	else
	{
		config_gpio_table(wifi_off_gpio_table, ARRAY_SIZE(wifi_off_gpio_table));
		bcm4325_set_core_power(BCM4325_WLAN, 0);
	}

	return 0;
}
EXPORT_SYMBOL(wifi_card_power);

int wifi_power(int on)
{

	printk("%s: %d\n", __func__, on);

	/* Power on the BCM4325 */
    wifi_card_power(on);

    /* Do the mmc_rescan */
    wifi_set_carddetect(on);

	return 0;
}
EXPORT_SYMBOL(wifi_power);


static int mmc_dbg_wifi_reset_get(void *data, u64 *val)
{
	return 0;
}

static int mmc_dbg_wifi_reset_set(void *data, u64 *val)
{
	return 0;
}

static int mmc_dbg_wifi_cd_get(void *data, u64 *val)	// wifi_set_carddetect((int) val);
{
	*val = wifi_cd;
	return 0;
}

static int mmc_dbg_wifi_cd_set(void *data, u64 *val)
{
	wifi_set_carddetect((int) val);
	return 0;
}

static int mmc_dbg_wifi_pwr_get(void *data, u64 *val)
{
	*val = wifi_power_state;
	return 0;
}

static int mmc_dbg_wifi_pwr_set(void *data, u64 *val)
{
	wifi_power((int)val);
	return 0;
}

#if 1
DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi_reset_fops,
		mmc_dbg_wifi_reset_get,
		mmc_dbg_wifi_reset_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi_cd_fops,
		mmc_dbg_wifi_cd_get,
		mmc_dbg_wifi_cd_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_wifi_pwr_fops,
		mmc_dbg_wifi_pwr_get,
		mmc_dbg_wifi_pwr_set, "%llu\n");
#endif
#endif

#endif

static void __init init_mmc(void)
{
#if 1
	vreg_mmc = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed ($ld)\n", __func__, PTR_ERR(vreg_mmc));
		return;
	}

	vreg_movinand = vreg_get(NULL, "wlan");
	if (IS_ERR(vreg_movinand)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n", __func__, PTR_ERR(vreg_movinand));
		return;
	}
/*
 * 2009. 06. 25 added by kt.hur
 * 		wake up when inserting or removing SD Card
 */
	set_irq_wake(MSM_GPIO_TO_INT(T_FLASH_DETECT), 1);

#endif
	sdcc_gpio_init();
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	msm_add_sdcc(3, &movinand_sdcc_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	msm_add_sdcc(1, &halibut_sdcc_data_rev04);
#endif
}

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};
static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
        msm_fb_register_device("pmdh", &mddi_pdata);
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
};

static void __init msm_device_i2c_init(void)
{
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
};

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
#if !defined(CONFIG_MACH_CAPELA_REV05)				// REV02, REV03 has WLAN_WAKE
	gpio_direction_output(BCM4325_WLAN_WAKE, 0);       // BT_WAKE_N
#endif
	
	msleep(100);
	
	gpio_direction_output(BCM4325_WLAN_RESET, 0);       // WLAN
	gpio_direction_output(BCM4325_BT_RESET, 0);       // BT

#if !defined(CONFIG_MACH_CAPELA_REV03)  // for REG_ON pin - REV02, REV05 has REG_ON
	gpio_direction_output(GPIO_WLAN_BT_REG_ON, 0);       // REG_ON
#endif

#if !defined(CONFIG_MACH_CAPELA_REV03)  // for REG_ON pin - REV02, REV05 has REG_ON
//	gpio_direction_output(GPIO_WLAN_BT_REG_ON, 1);       // REG_ON
//	Power-up Sequence for BT off and WIFI off is
//	VBAT & VDDIO is up
//	REG_ON, WL_RESET and BT_RESET is low
	
	gpio_direction_output(BCM4325_WLAN_RESET, 0);       // WLAN
	gpio_direction_output(BCM4325_BT_RESET, 0);       // BT -> off
#else
	gpio_direction_output(BCM4325_WLAN_RESET, 0);       // WLAN
	msleep(150);
	gpio_direction_output(BCM4325_BT_RESET, 1);       // BT
#endif

}

static void __init init(void)
{
    int rc = 0;
	socinfo_init();
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
	init_mmc();	// SD Card Power Issue

	/* All 7x01 2.0 based boards are expected to have RAM chips capable
	 * of 160 MHz. */
	if (cpu_is_msm7x01()
	    && SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2)
		halibut_clock_data.max_axi_khz = 160000;

	msm_acpu_clock_init(&halibut_clock_data);
	msm_device_uart_dm1.dev.platform_data = NULL;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
	msm_device_hsusb_host.dev.platform_data = &msm_hsusb_pdata;
	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_device_i2c_init();

#if defined(CONFIG_SAMSUNG_CAPELA)

	i2c_register_board_info(3, cam_i2c_devices,ARRAY_SIZE(cam_i2c_devices));
	i2c_register_board_info(1, sensor_i2c_devices,ARRAY_SIZE(sensor_i2c_devices));
	i2c_register_board_info(2, gpio_i2c_devices,ARRAY_SIZE(gpio_i2c_devices));
	i2c_register_board_info(4, cam_pm_lp8720_i2c_devices, ARRAY_SIZE(cam_pm_lp8720_i2c_devices));
	i2c_register_board_info(5, touch_i2c_devices,ARRAY_SIZE(touch_i2c_devices));				// TOUCH
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	msm_add_sdcc(2, &wifi_data);
#endif
//	if (rc)
//		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

#ifdef CONFIG_USB_FUNCTION
	hsusb_gpio_init();
#endif
	msm_fb_add_devices();
	msm_camera_add_device();
	bcm4325_init();	// added for bcm4325
#ifdef CONFIG_USB_ANDROID
	msm_hsusb_rpc_connect();
	msm_hsusb_set_vbus_state(1) ;
#endif
	msm_pm_set_platform_data(msm_pm_data);
}

static void __init msm_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = MSM_KERNEL_PANIC_DUMP_SIZE;
	addr = alloc_bootmem(size);
	MSM_KERNEL_PANIC_DUMP_ADDR = addr;


#ifndef CONFIG_MSM7K_SMI64
	size = MSM_PMEM_MDP_SIZE;
	addr = alloc_bootmem(size);
	android_pmem_pdata.start = __pa(addr);
	android_pmem_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for pmem\n", size, addr, __pa(addr));

	size = MSM_PMEM_CAMERA_SIZE;
	addr = alloc_bootmem(size);
	android_pmem_camera_pdata.start = __pa(addr);
	android_pmem_camera_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for camera pmem\n", size, addr, __pa(addr));

	size = MSM_PMEM_ADSP_SIZE;
	addr = alloc_bootmem(size);
	android_pmem_adsp_pdata.start = __pa(addr);
	android_pmem_adsp_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for adsp pmem\n", size, addr, __pa(addr));
#endif

	size = MSM_PMEM_GPU1_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	android_pmem_gpu1_pdata.start = __pa(addr);
	android_pmem_gpu1_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for gpu1 pmem\n", size, addr, __pa(addr));

	size = MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));

}

static void __init map_io(void)
{
	msm_shared_ram_phys = 0x01F00000;

	msm_map_common_io();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
	msm_allocate_memory_regions();
}

#if 1
MACHINE_START(UNIV, "Samsung GT-I7500 Board")

/* UART for LL DEBUG */
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif

	.boot_params	= 0x10000100,
	.map_io		= map_io,
	.init_irq	= init_irq,
	.init_machine	= init,
	.timer		= &msm_timer,
MACHINE_END
#endif

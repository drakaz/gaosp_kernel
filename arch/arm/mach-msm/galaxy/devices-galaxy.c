/* linux/arch/arm/mach-msm/behold2/devices-behold2.c
 *
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
 */

// drakaz : 3D patch

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include "../gpio_chip.h"
#include "../devices.h"
#include <mach/board.h>
#include <mach/galaxy.h>
#include <mach/msm_hsusb.h>

#ifdef CONFIG_USB_FUNCTION
#include <linux/usb/mass_storage_function.h>
#endif
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_GADGET_MSM_72K)
#include <linux/usb/android.h>
#endif
#ifdef CONFIG_USB_GADGET_MSM_72K
#include <mach/rpc_hsusb.h>
#endif

#include <asm/mach/flash.h>
#include <asm/setup.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>
#include <mach/gpio.h>
#include <asm/mach/mmc.h>

static char *df_serialno = "000000000000";

#define HSUSB_API_INIT_PHY_PROC	2
#define HSUSB_API_PROG		0x30000064
#define HSUSB_API_VERS MSM_RPC_VERS(1,1)


#if defined(CONFIG_USB_FUNCTION) || defined(CONFIG_USB_GADGET_MSM_72K)
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

static void internal_phy_reset(void)
{
	struct msm_rpc_endpoint *usb_ep;
	int rc;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
	} req;

	printk(KERN_INFO "msm_hsusb_phy_reset\n");

	usb_ep = msm_rpc_connect(HSUSB_API_PROG, HSUSB_API_VERS, 0);
	if (IS_ERR(usb_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(usb_ep));
		goto close;
	}
	rc = msm_rpc_call(usb_ep, HSUSB_API_INIT_PHY_PROC,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(usb_ep);
}

#if defined(CONFIG_USB_FUNCTION) || defined(CONFIG_USB_GADGET_MSM_72K)
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.nluns          = 0x02,
	.buf_size       = 16384,
#endif
	.vendor         = "Samsung",
	.product        = "SAMSUNG Android Mass storage",
	.release        = 0x0100,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_GADGET_MSM_72K)
static void hsusb_phy_reset(void)
{
	msm_hsusb_phy_reset();
}
#endif

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_GADGET_MSM_72K)
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x04E8,
	.product_id	= 0x6603,
	.adb_product_id	= 0x6640,
	.version	= 0x0100,
	.product_name	= "GT-I7500       ",
	.manufacturer_name = "Samsung ",
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
static struct usb_function_map usb_functions_map[] = {
	{"ethernet", 0},
	{"modem", 1},
	{"diag", 2},
	{"mass_storage", 3},
	{"adb", 4},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id          = 0x6601, //by gtuo.park
                .functions      = 0x0E, /* 1110 modem, diag, ums */
	},

	{
                .product_id     = 0x6640, // by gtuo.park
                .functions      = 0x1E,  /* 11110 modem, diag, ums, adb */
	},

	{
		.product_id			= 0x6603, // by gtuo.park
                .functions      = 0x8,  /* ums only*/
	},

	{
		.product_id         = 0x6605,
		.functions	    = 0x01, /* 1 ethernet */
	},

	{
		.product_id         = 0x6606,
		.functions	    = 0x11, /* 100001 adb, ethernet */
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

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_GADGET_MSM_72K)
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

void __init msm_add_usb_devices(void (*phy_reset) (void))
{
	hsusb_gpio_init();

	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;	
	msm_device_hsusb_host.dev.platform_data = &msm_hsusb_pdata;
	platform_device_register(&msm_device_hsusb_peripheral);
	platform_device_register(&msm_device_hsusb_host);
	platform_device_register(&msm_device_hsusb_otg);
	platform_device_register(&mass_storage_device);
	
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_GADGET_MSM_72K)	
	platform_device_register(&android_usb_device);
#endif
}

struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 1,
};

struct android_pmem_platform_data android_pmem_gpu1_pdata = {
       .name = "pmem_gpu1",
       .allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
       .cached = 0,
};


static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name = "pmem_camera",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_camera_pdata },
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 5,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

static struct resource ram_console_resource[] = {
	{
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resource),
	.resource       = ram_console_resource,
};

static struct resource resources_hw3d[] = {
	{
		.start	= 0xA0000000,
		.end	= 0xA00fffff,
		.flags	= IORESOURCE_MEM,
		.name	= "regs",
	},
	{
		.flags	= IORESOURCE_MEM,
		.name	= "smi",
	},
	{
		.flags	= IORESOURCE_MEM,
		.name	= "ebi",
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
		.name	= "gfx",
	},
};

static struct platform_device hw3d_device = {
	.name		= "msm_hw3d",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_hw3d),
	.resource	= resources_hw3d,
};


void __init msm_add_mem_devices(struct msm_pmem_setting *setting)
{
	if (setting->pmem_size) {
		android_pmem_pdata.start = setting->pmem_start;
		android_pmem_pdata.size = setting->pmem_size;
		platform_device_register(&android_pmem_device);
	}

	if (setting->pmem_adsp_size) {
		android_pmem_adsp_pdata.start = setting->pmem_adsp_start;
		android_pmem_adsp_pdata.size = setting->pmem_adsp_size;
		platform_device_register(&android_pmem_adsp_device);
	}

/*
	if (setting->pmem_gpu0_size) {
		android_pmem_gpu0_pdata.start = setting->pmem_gpu0_start;
		android_pmem_gpu0_pdata.size = setting->pmem_gpu0_size;
		platform_device_register(&android_pmem_gpu0_device);
	}

	if (setting->pmem_gpu1_size) {
		#if 0 // dynamically alloc when machine init with alloc_bootmem
		android_pmem_gpu1_pdata.start = setting->pmem_gpu1_start;
		android_pmem_gpu1_pdata.size = setting->pmem_gpu1_size;
		#endif
		platform_device_register(&android_pmem_gpu1_device);
	}
*/

	if (setting->pmem_gpu0_size && setting->pmem_gpu1_size) {
		struct resource *res;

		res = platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,
						   "smi");
		res->start = setting->pmem_gpu0_start;
		res->end = res->start + setting->pmem_gpu0_size - 1;

		res = platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,
						   "ebi");
		res->start = android_pmem_gpu1_pdata.start; 
//		res->start = setting->pmem_gpu1_start; // using dynamically allocated addr
		res->end = res->start + setting->pmem_gpu1_size - 1;
		platform_device_register(&hw3d_device);
	}


	if (setting->pmem_camera_size) {
		android_pmem_camera_pdata.start = setting->pmem_camera_start;
		android_pmem_camera_pdata.size = setting->pmem_camera_size;
		platform_device_register(&android_pmem_camera_device);
	}
 
	if (setting->pmem_kernel_ebi1_size) {
		android_pmem_kernel_ebi1_pdata.start = setting->pmem_kernel_ebi1_start;
		android_pmem_kernel_ebi1_pdata.size = setting->pmem_kernel_ebi1_size;
		platform_device_register(&android_pmem_kernel_ebi1_device);
	}

	if (setting->ram_console_size) {
		ram_console_resource[0].start = setting->ram_console_start;
		ram_console_resource[0].end = setting->ram_console_start
			+ setting->ram_console_size - 1;
		platform_device_register(&ram_console_device);
	}
}


/* drivers/video/msm/src/panel/mddi/toshiba.c
 *
 * Copyright (C) 2008 QUALCOMM Incorporated.
 */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#include <linux/device.h>
#include <mach/vreg.h>
#include <mach/gpio.h>


// For Samsung BigFoot S6D05A0 LCD device //
#define SWRESET          0x01
#define RDDIDIF          0x04
#define RDDST            0x09
#define RDDPM            0x0A
#define RDDMADCTL        0x0B
#define RDDCOLMOD        0x0C
#define RDDSM            0x0E
#define RDDSSDR          0x0F
#define SLPIN            0x10
#define SLPOUT           0x11
#define PTLON            0x12
#define NORON            0x13
#define DISPOFF          0x28
#define DISPON           0x29
#define CASET            0x2A
#define PASET            0x2B
#define PRAMWR           0x2C
#define RAMRD            0x2C
#define PTLAR            0x30
#define TEOFF            0x34
#define TEON             0x35
#define MADCTL           0x36
#define IDMOFF           0x38
#define IDMON            0x39
#define COLMOD           0x3A
#define WRDISBV          0x51
#define RDDISBV          0x52
#define WRCTRLD          0x53
#define RDCTRLD          0x54
#define WRCABC           0x55
#define RDCABC           0x56
#define WRCABCMB         0x5E
#define RDCABCMB         0x5F
#define MIECTL1          0xCA
#define BCMODE           0xCB
#define RDID1            0xDA
#define RDID2            0xDB
#define RDID3            0xDC
#define DSTB             0xB0
#define MIECTL2          0xCC
#define MIECTL3          0xCD
#define MTPCTL           0xD0
#define WRVCMOC          0xD1
#define WRVMLOC          0xD2
#define WRGVDOC          0xD3
#define WRID             0xD4
#define RDOFFSETC        0xD5
#define MDDICTL          0xE0
#define MDILINK          0xE1
#define DCON             0xEF
#define WR_PWD           0xF0
#define DISCTL           0xF2
#define PWRCTL           0xF3
#define VCMCTL           0xF4
#define SRCCTL           0xF5
#define IFCTL            0xF6
#define RPGAMCTL         0xF7
#define RNGAMCTL         0xF8
#define GPGAMCTL         0xF9
#define GNGAMCTL         0xFA
#define BPGAMCTL         0xFB
#define BNGAMCTL         0xFC
#define GATECTL          0xFD

struct mddi_table {
	uint32_t reg;
	unsigned value[4];
	uint32_t val_len;
};

static struct mddi_table mddi_epson_panel_power_on_table[] = {
	{ PWRCTL	,	{0x002A0000,0x41413300,0x00000000}				,	3	},
	{ SLPOUT	,	{0x00000000}									,	1	},
	{ 1			,	{ 0 }											,	20	},	//msleep 20 
	{ DISCTL		,	{0x110F1616,0x10111111,0x00161600}				,	3	},
	{ PWRCTL	,	{0x002A0100,0x41413300,0x00000000}				,	3	},
	{ VCMCTL	,	{0x21213939,0x00000000}							,	2	},
	{ GATECTL	,	{0x00003B44}									,	1	},
	{ PWRCTL	,	{0x002A0300,0x41413300,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	20	},	//msleep 20 
	{ PWRCTL	,	{0x002A0700,0x41413300,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	20	},	//msleep 20 
	{ PWRCTL	,	{0x002A0F00,0x41413302,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	20	},	//msleep 20 
	{ PWRCTL	,	{0x002A1F00,0x41413302,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	30	},	//msleep 30 
	{ PWRCTL	,	{0x002A3F00,0x41413308,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	40	},	//msleep 40 
	{ PWRCTL	,	{0x002A7F00,0x41413308,0x00000000}				,	3	},
	{ TEON		,	{0x00000000}									,	1	},
	{ MADCTL	,	{0x00000048}									,	1	},
	{ COLMOD	,	{0x00000066}									,	1	},
	{ RPGAMCTL	,	{0x07010D00,0x22222318,0x18171A1B,0x00222206}		,	4	},
	{ RNGAMCTL	,	{0x07010D00,0x22222318,0x18171A1B,0x00222206}		,	4	},
	{ CASET		,	{ 0x3F010000 }									,	1	},
	{ PASET		,	{ 0xDF010000 }									,	1	},
	{ DCON		,	{0x00000006 }									,	1	},
	{ 1			,	{ 0 }											,	45	},	//msleep 45 
	{ DCON		,	{0x00000007}									,	1	},
};

static struct mddi_table mddi_epson_panel_sleep_in_table[] = {
	{ DCON		,	{0x00000006}									,	1	},
	{ 1			,	{ 0 }											,	90	},	//msleep 90 
	{ DCON		,	{0x00000000}									,	1	},
	{ 1			,	{ 0 }											,	45	},	//msleep 45 
	{ PWRCTL	,	{0x002A0000,0x41413300,0x00000000}				,	3	},
	{ SLPIN		,	{0x00000000}									,	1	},
	{ 1			,	{ 0 }											,	250	},	//msleep 250 
};
static struct mddi_table mddi_epson_panel_sleep_out_table[] = {
	{ PWRCTL	,	{0x002A0000,0x41413300,0x00000000}				,	3	},
	{ SLPOUT	,	{0x00000000}									,	1	},
	{ 1			,	{ 0 }											,	20	},	//msleep 20 
	{ PWRCTL	,	{0x002A0100,0x41413300,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	20	},	//msleep 20 
	{ PWRCTL	,	{0x002A0300,0x41413300,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	20	},	//msleep 20 
	{ PWRCTL	,	{0x002A0700,0x41413300,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	20	},	//msleep 20 
	{ PWRCTL	,	{0x002A0F00,0x41413302,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	20	},	//msleep 20 
	{ PWRCTL	,	{0x002A1F00,0x41413302,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	30	},	//msleep 30 
	{ PWRCTL	,	{0x002A3F00,0x41413308,0x00000000}				,	3	},
	{ 1			,	{ 0 }											,	40	},	//msleep 40 
	{ PWRCTL	,	{0x002A7F00,0x41413308,0x00000000}				,	3	},
	{ DCON		,	{0x00000006 }									,	1	},
	{ 1			,	{ 0 }											,	45	},	//msleep 45 
	{ DCON		,	{0x00000007}									,	1	},
};	

#define HVGA_WIDTH          320
#define HVGA_HEIGHT         480

static uint32 mddi_samsung_curr_vpos;
static boolean mddi_samsung_monitor_refresh_value = FALSE;
static boolean mddi_samsung_report_refresh_measurements = FALSE;

boolean mddi_samsung_61Hz_refresh = TRUE;

/* Modifications to timing to increase refresh rate to > 60Hz.
 *   20MHz dot clock.
 *   646 total rows.
 *   506 total columns.
 *   refresh rate = 61.19Hz
 */
static uint32 mddi_samsung_rows_per_second = 39526;
static uint32 mddi_samsung_usecs_per_refresh = 16344;
static uint32 mddi_samsung_rows_per_refresh = 646;
extern boolean mddi_vsync_detect_enabled;

static msm_fb_vsync_handler_type mddi_samsung_vsync_handler = NULL;
static void *mddi_samsung_vsync_handler_arg;
static uint16 mddi_samsung_vsync_attempts;

static struct msm_panel_common_pdata *mddi_samsung_pdata;

static int mddi_samsung_lcd_on(struct platform_device *pdev);
static int mddi_samsung_lcd_off(struct platform_device *pdev);

#define SENSOR_SCL 2
#define SENSOR_SDA 3
#define BD6091_ADDR (0x76 << 1)

void panel_backlight_set_with_i2c_gpio(unsigned *data, unsigned len)
{
  unsigned i,j;

  //if there is no pull up
  gpio_direction_output(SENSOR_SDA, 1);
  udelay(3);
  gpio_direction_output(SENSOR_SCL, 1);
  udelay(10);


  //start condition
  gpio_direction_output(SENSOR_SDA, 0);
  udelay(3);
  gpio_direction_output(SENSOR_SCL, 0);
  udelay(2);


  for(i = 0; i < len; i++)
  {
    for(j = 0; j < 8; j++)
    {
      if((data[i]<<j) & 0x80)
      {
        gpio_direction_output(SENSOR_SDA, 1);;   // SDA
      }
      else
      {
        gpio_direction_output(SENSOR_SDA, 0);;
      }
      udelay(2);
      gpio_direction_output(SENSOR_SCL, 1);    // SCL
      udelay(8);
      gpio_direction_output(SENSOR_SCL, 0);;
      udelay(10);
    }


    //gpio_direction_input(SENSOR_SDA);  // SDA
    gpio_direction_output(SENSOR_SDA, 0);  // SDA
    udelay(2);
    gpio_direction_output(SENSOR_SCL, 1);   // 9¹øÂ° SCL - ACK
    udelay(4);
    gpio_direction_output(SENSOR_SCL, 0);
    udelay(10);
  }

  // stop condition
  gpio_direction_output(SENSOR_SCL, 1);  // SCL
  udelay(5);
  gpio_direction_output(SENSOR_SDA, 1);  // SDA
  udelay(5);
  
  /* delay to allow sensor to complete the operation */
  for (i=0; i<100; i++);
}

#define write_client_reg(__X,__Y,__Z) {\
  mddi_queue_register_write(__X,__Y,TRUE,0);\
}
#define write_client_reg_multi(__X, __Y, __Z) {\
	mddi_queue_register_multi_write(__X, __Y, __Z, TRUE, 0);\
}

static void process_mddi_table(struct mddi_table *table, size_t count)
{
	int i;
	for(i = 0; i < count; i++) {
		uint32_t reg = table[i].reg;
		unsigned *value = table[i].value;
		uint32_t val_len = table[i].val_len;

		if (reg == 0)
			udelay(val_len);
		else if (reg == 1)
			msleep(val_len);
		else
			write_client_reg_multi( reg,  (unsigned *)value, val_len);
	}
}

static void samsung_s6d05a0_init(void)
{
	process_mddi_table(mddi_epson_panel_power_on_table, ARRAY_SIZE(mddi_epson_panel_power_on_table));
}

static void samsung_s6d05a0_sleep_in(void)
{
//	printk("[HSI] %s: %d\n", __func__, __LINE__);
	process_mddi_table(mddi_epson_panel_sleep_in_table, ARRAY_SIZE(mddi_epson_panel_sleep_in_table));
}

static void samsung_s6d05a0_sleep_out(void)
{
	process_mddi_table(mddi_epson_panel_sleep_out_table, ARRAY_SIZE(mddi_epson_panel_sleep_out_table));
}

static void mddi_samsung_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	uint32 level;
	unsigned bl_on[3] =   {BD6091_ADDR,0x01,0x11};
	unsigned bl_current[3] =   {BD6091_ADDR,0x03,0x4F};
	unsigned bl_off[3] =  {BD6091_ADDR,0x01,0x10};

	level = mfd->bl_level;

	printk("[HSI BL] %s: %d\n", __func__, __LINE__);
	if ( level == 0 ) 
	{
		printk("[HSI] %s: %d : level =0 ==. OFF\n", __func__, __LINE__);
		panel_backlight_set_with_i2c_gpio(bl_off,3);
	} 
	else 
	{
	    panel_backlight_set_with_i2c_gpio(bl_on,3);
	}
}

static void mddi_samsung_vsync_set_handler(msm_fb_vsync_handler_type handler,	/* ISR to be executed */
					   void *arg)
{
	boolean error = FALSE;
	unsigned long flags;

	/* Disable interrupts */
	spin_lock_irqsave(&mddi_host_spin_lock, flags);
	// INTLOCK();

	if (mddi_samsung_vsync_handler != NULL) {
		error = TRUE;
	} else {
		/* Register the handler for this particular GROUP interrupt source */
		mddi_samsung_vsync_handler = handler;
		mddi_samsung_vsync_handler_arg = arg;
	}

	/* Restore interrupts */
	spin_unlock_irqrestore(&mddi_host_spin_lock, flags);
	// MDDI_INTFREE();
	if (error) {
		MDDI_MSG_ERR("MDDI: Previous Vsync handler never called\n");
	} else {
		mddi_samsung_vsync_attempts = 1;
		mddi_vsync_detect_enabled = TRUE;
	}
}				/* mddi_toshiba_vsync_set_handler */

static void mddi_samsung_lcd_vsync_detected(boolean detected)
{
	// static timetick_type start_time = 0;
	static struct timeval start_time;
	static boolean first_time = TRUE;
	// uint32 mdp_cnt_val = 0;
	// timetick_type elapsed_us;
	struct timeval now;
	uint32 elapsed_us;
	uint32 num_vsyncs;

	if ((detected) || (mddi_samsung_vsync_attempts > 5)) {
		if ((detected) && (mddi_samsung_monitor_refresh_value)) {
			// if (start_time != 0)
			if (!first_time) {
				// elapsed_us = timetick_get_elapsed(start_time, T_USEC);
				jiffies_to_timeval(jiffies, &now);
				elapsed_us =
				    (now.tv_sec - start_time.tv_sec) * 1000000 +
				    now.tv_usec - start_time.tv_usec;
				/* LCD is configured for a refresh every * usecs, so to
				 * determine the number of vsyncs that have occurred
				 * since the last measurement add half that to the
				 * time difference and divide by the refresh rate. */
				num_vsyncs = (elapsed_us +
					      (mddi_samsung_usecs_per_refresh >>
					       1)) /
				    mddi_samsung_usecs_per_refresh;
				/* LCD is configured for * hsyncs (rows) per refresh cycle.
				 * Calculate new rows_per_second value based upon these
				 * new measurements. MDP can update with this new value. */
				mddi_samsung_rows_per_second =
				    (mddi_samsung_rows_per_refresh * 1000 *
				     num_vsyncs) / (elapsed_us / 1000);
			}
			// start_time = timetick_get();
			first_time = FALSE;
			jiffies_to_timeval(jiffies, &start_time);
			if (mddi_samsung_report_refresh_measurements) {
				// mdp_cnt_val = MDP_LINE_COUNT;
			}
		}
		/* if detected = TRUE, client initiated wakeup was detected */
		if (mddi_samsung_vsync_handler != NULL) {
			(*mddi_samsung_vsync_handler)
			    (mddi_samsung_vsync_handler_arg);
			mddi_samsung_vsync_handler = NULL;
		}
		mddi_vsync_detect_enabled = FALSE;
		mddi_samsung_vsync_attempts = 0;
		/* need to disable the interrupt wakeup */

		if (!detected) {
			/* give up after 5 failed attempts but show error */
			MDDI_MSG_NOTICE("Vsync detection failed!\n");
		} else if ((mddi_samsung_monitor_refresh_value) &&
			   (mddi_samsung_report_refresh_measurements)) {
			MDDI_MSG_NOTICE("  Last Line Counter=%d!\n",
					mddi_samsung_curr_vpos);
			// MDDI_MSG_NOTICE("  MDP Line Counter=%d!\n",mdp_cnt_val);
			MDDI_MSG_NOTICE("  Lines Per Second=%d!\n",
					mddi_samsung_rows_per_second);
		}

	} else {
		/* if detected = FALSE, we woke up from hibernation, but did not
		 * detect client initiated wakeup.
		 */
		mddi_samsung_vsync_attempts++;
	}
}

static void mddi_samsung_init(void)
{
	samsung_s6d05a0_init();
	mddi_host_write_pix_attr_reg(0x00C3);
}

static void mddi_samsung_lcd_sleep_in(void)
{
	samsung_s6d05a0_sleep_in();
}

static void mddi_samsung_lcd_powerdown(void)
{
#if 0
	switch (toshiba_state) {
	case TOSHIBA_STATE_PRIM_SEC_READY:
		mddi_toshiba_prim_init();
		mddi_toshiba_lcd_powerdown();
		return;
	case TOSHIBA_STATE_PRIM_SEC_STANDBY:
		break;
	case TOSHIBA_STATE_PRIM_NORMAL_MODE:
		toshiba_prim_lcd_off();
		break;
	case TOSHIBA_STATE_SEC_NORMAL_MODE:
		toshiba_sec_cont_update_stop();
		toshiba_sec_sleep_in();
		toshiba_sec_sleep_out();
		toshiba_sec_lcd_off();
		break;
	default:
		MDDI_MSG_ERR("mddi_toshiba_lcd_powerdown from state %d\n",
			     toshiba_state);
	}
#else
	mddi_samsung_lcd_sleep_in();
#endif
}

static int panel_detect ;

static void mddi_samsung_panel_detect(void)
{
        panel_detect = 1;
}

static int mddi_samsung_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mddi_samsung_panel_detect();
	mddi_samsung_init();

	mfd->bl_level = 100;
	mddi_samsung_lcd_set_backlight(mfd);
	samsung_s6d05a0_sleep_out();

	return 0;
}

static int mddi_samsung_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	printk("[HSI] %s: %d\n", __func__, __LINE__);

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mfd->bl_level = 0;
	mddi_samsung_lcd_set_backlight(mfd);

	mddi_samsung_panel_detect();
	mddi_samsung_lcd_powerdown();
	return 0;
}

static int __init mddi_samsung_lcd_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mddi_samsung_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mddi_samsung_lcd_probe,
	.driver = {
		.name   = "mddi_samsung_hvga",
	},
};

static struct msm_fb_panel_data samsung_panel_data0 = {
	.on 		= mddi_samsung_lcd_on,
	.off 		= mddi_samsung_lcd_off,
	.set_backlight 	= mddi_samsung_lcd_set_backlight,
	.set_vsync_notifier = mddi_samsung_vsync_set_handler,
};

static struct platform_device this_device_0 = {
	.name   = "mddi_samsung_hvga",
	.id	= 1,
	.dev	= {
		.platform_data = &samsung_panel_data0,
	}
};

static int __init mddi_toshiba_lcd_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	ret = platform_driver_register(&this_driver);
	if (!ret) {
		pinfo = &samsung_panel_data0.panel_info;
		pinfo->xres = HVGA_WIDTH;
		pinfo->yres = HVGA_HEIGHT;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->wait_cycle = 0;
		pinfo->bpp = 18;
		pinfo->lcd.vsync_enable = TRUE;
		pinfo->lcd.refx100 =
		    (mddi_samsung_rows_per_second * 100) /
		    mddi_samsung_rows_per_refresh;
		pinfo->lcd.v_back_porch = 6;
		pinfo->lcd.v_front_porch = 0;
		pinfo->lcd.v_pulse_width = 0;
		pinfo->lcd.hw_vsync_mode = FALSE;
		pinfo->lcd.vsync_notifier_period = (1 * HZ);
		pinfo->bl_max = 100;
		pinfo->bl_min = 10;
		pinfo->clk_rate = 122880000;
		pinfo->fb_num = 2;

		ret = platform_device_register(&this_device_0);
		if (ret)
			platform_driver_unregister(&this_driver);
	}

	if (!ret)
		mddi_lcd.vsync_detected = mddi_samsung_lcd_vsync_detected;

	return ret;
}

module_init(mddi_toshiba_lcd_init);

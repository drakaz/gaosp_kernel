/* arch/arm/mach-msm/board-battery.c
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <asm/gpio.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/irq.h>
static struct wake_lock vbus_wake_lock;
extern void lightsensor_low_battery_flag_set (int );
extern void lcd_low_battery_flag_set_for_lightsensor (int );


#define TRACE_BATT 1

//#define BATT_TABLE_CALL_BASIS // battery table applied by call connected level base

#if TRACE_BATT
#define BATT(x...) printk(KERN_INFO "[BATT] " x)
#else
#define BATT(x...) do {} while (0)
#endif

#define BATT_LOG(en, x...) if(en) printk(KERN_INFO "[BATT] " x)
#define BATT_DBG(en, x...) if(en>=2) printk(KERN_INFO "[BATT] " x)

/* rpc related */
#define APP_BATT_PDEV_NAME         "rs3000008d:00010000"
#define APP_BATT_PROG                   0x3000008d
#define APP_BATT_VER                    0x00010001

#define SEC_PROCEDURE_BATTERY_NULL      0
#define SEC_PROCEDURE_GET_BATT_LEVEL    1
#define SEC_PROCEDURE_GET_BATT_INFO     2
#define SEC_PROCEDURE_GET_CABLE_STATUS  3
#define SEC_PROCEDURE_SET_BATT_DELTA    4
#define SEC_PROCEDURE_SET_CHARGER_ON    5
#define SEC_PROCEDURE_SET_CHARGING_SRC    6
#define SEC_PROCEDURE_SET_USING_DEV    7

#define SCALING_VAL			100

/* Enable this will shut down if no battery */
#define ENABLE_BATTERY_DETECTION        0

#define CHARGER_CONTROL

#ifdef CHARGER_CONTROL
#define CHG_OFF_LOW_TEMP 80
#define CHG_RESTART_LOW_TEMP 87
#define CHG_OFF_HIGH_TEMP 230
#define CHG_RESTART_HIGH_TEMP 224
#ifdef BATT_TABLE_CALL_BASIS
#define CHG_OFF_LEVEL 1420
#define CHG_RESTART_LEVEL 1370
#define PWR_OFF_LEVEL 500
#else
#define CHG_OFF_LEVEL 1460
#define CHG_RESTART_LEVEL 1390
#define PWR_OFF_LEVEL 540
#endif // BATT_TABLE_CALL_BASIS
#endif

typedef enum {
        DISABLE = 0,
        ENABLE_SLOW_CHG,
        ENABLE_FAST_CHG
} batt_ctl_t;

/* This order is the same as power_supplies[]
 * And it's also the same as cable_status_update()
 */
typedef enum {
        CHARGER_BATTERY = 0,
        CHARGER_USB,
        CHARGER_AC
} charger_type_t;

struct battery_level_reply {
        u32 batt_level;           
};

struct battery_info_reply {
        u32 batt_id;            /* Battery ID from ADC */
        u32 batt_vol;           /* Battery voltage from ADC */
        u32 batt_temp;          /* Battery Temperature (C) from formula and ADC */
        u32 batt_current;       /* Battery current from ADC */
        u32 level;              /* formula */
        u32 charging_source;    /* 0: no cable, 1:usb, 2:AC */
        u32 charging_enabled;   /* 0: Disable, 1: Enable */ // 2: too warm 3: too cold
        u32 full_bat;           /* Full capacity of battery (mAh) */
	u32 batt_therm;           /* therm_adc */
};

struct battery_comp_check{
	u32 vibrator;
	u32 keypad_backlight;
	u32 lcd_backlight;
	u32 lcd_dimming;
	u32 camera_down;
	u32 camcoder_down;
	u32 amp_down;
	u32 video_down;
	u32 bt_down;
	u32 camera_flash_down;
	u32 talk_wcdma;
	u32 talk_gsm;
	u32 data_wcdma;
};

struct battery_info {
       int present;
       unsigned long update_time;

       /* lock to protect the battery info */
       struct mutex lock;

       /* lock held while calling the arm9 to query the battery info */
       struct mutex rpc_lock;
       struct battery_info_reply rep;
	   struct battery_comp_check comp_check;
	u32 batt_delta;
	u32 batt_adc_comp;
	u32 batt_voltage_comp;
	u32 charger_time_out;
	u32 log_en;
};

struct battery_level_info {
       struct mutex lock;

       struct mutex rpc_lock;
       struct battery_level_reply rep;

	struct work_struct  work;
	struct timer_list timer;
};

static struct msm_rpc_endpoint *endpoint;

static struct battery_info batt_info;

static struct battery_level_info batt_level_info;

static unsigned int cache_time = 1000;

static int battery_initial = 0;

static enum power_supply_property battery_properties[] = {
        POWER_SUPPLY_PROP_STATUS,
        POWER_SUPPLY_PROP_HEALTH,
        POWER_SUPPLY_PROP_PRESENT,
        POWER_SUPPLY_PROP_TECHNOLOGY,
        POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property power_properties[] = {
        POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
        "battery",
};

/* SEC dedicated attributes */
static struct workqueue_struct *batt_level_wq;

static void batt_level_work_func(struct work_struct *work);

static ssize_t battery_show_property(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf);

static ssize_t battery_store_property(struct device *dev, 
					struct device_attribute *attr, const char *buf, size_t size);

static int power_get_property(struct power_supply *psy, 
                                    enum power_supply_property psp,
                                    union power_supply_propval *val);

static int battery_get_property(struct power_supply *psy, 
                                    enum power_supply_property psp,
                                    union power_supply_propval *val);

static struct power_supply power_supplies[] = {
        {
                .name = "battery",
                .type = POWER_SUPPLY_TYPE_BATTERY,
                .properties = battery_properties,
                .num_properties = ARRAY_SIZE(battery_properties),
                .get_property = battery_get_property,
        },
        {
                .name = "usb",
                .type = POWER_SUPPLY_TYPE_USB,
                .supplied_to = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties = power_properties,
                .num_properties = ARRAY_SIZE(power_properties),
                .get_property = power_get_property,
        },
        {
                .name = "ac",
                .type = POWER_SUPPLY_TYPE_MAINS,
                .supplied_to = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties = power_properties,
                .num_properties = ARRAY_SIZE(power_properties),
                .get_property = power_get_property,
        },
};


int fsa_init_done = 0;

static int current_vbatt_adc = 0;
static int pre_vbatt_adc = 0;
static int pre_scaled_val = 0;

unsigned int vbatt_adc_table[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned int therm_adc_table[3] = {0, 0, 0};

unsigned int on_call = 0;
static int ignore_prev_batt = 0;

extern unsigned char fsa9480_i2c_read_reg(unsigned char addr, unsigned char *reg_value);
#define FSA_INT1 	0x03
#define FSA_INT2 	0x04
#define FSA_DTYPE1 	0x0A

static int out_count = 0, charger_off = 0;

struct batt_level_table{
	int vbatt_adc;
	int level;
	int volt;
};

#ifdef BATT_TABLE_CALL_BASIS
struct batt_level_table orion_batt_level_table[] = {
	{	1420	,	100	,	4140	},
	{	1419	,	99	,	4139	},
	{	1398	,	98	,	4122	},
	{	1388	,	97	,	4113	},
	{	1377	,	96	,	4104	},
	{	1366	,	95	,	4095	},
	{	1356	,	94	,	4086	},
	{	1345	,	93	,	4077	},
	{	1334	,	92	,	4068	},
	{	1324	,	91	,	4059	},
	{	1313	,	90	,	4050	},
	{	1302	,	89	,	4041	},
	{	1292	,	88	,	4032	},
	{	1281	,	87	,	4023	},
	{	1270	,	86	,	4014	},
	{	1260	,	85	,	4005	},
	{	1249	,	84	,	3996	},
	{	1238	,	83	,	3987	},
	{	1228	,	82	,	3978	},
	{	1217	,	81	,	3969	},
	{	1206	,	80	,	3960	},
	{	1196	,	79	,	3951	},
	{	1185	,	78	,	3942	},
	{	1174	,	77	,	3933	},
	{	1164	,	76	,	3924	},
	{	1153	,	75	,	3915	},
	{	1142	,	74	,	3906	},
	{	1132	,	73	,	3897	},
	{	1121	,	72	,	3888	},
	{	1110	,	71	,	3879	},
	{	1100	,	70	,	3870	},
	{	1094	,	69	,	3865	},
	{	1089	,	68	,	3860	},
	{	1083	,	67	,	3855	},
	{	1078	,	66	,	3850	},
	{	1072	,	65	,	3845	},
	{	1067	,	64	,	3840	},
	{	1061	,	63	,	3835	},
	{	1056	,	62	,	3830	},
	{	1050	,	61	,	3825	},
	{	1045	,	60	,	3820	},
	{	1039	,	59	,	3815	},
	{	1034	,	58	,	3810	},
	{	1028	,	57	,	3805	},
	{	1023	,	56	,	3800	},
	{	1017	,	55	,	3795	},
	{	1012	,	54	,	3790	},
	{	1006	,	53	,	3785	},
	{	1001	,	52	,	3780	},
	{	995	,	51	,	3775	},
	{	990	,	50	,	3770	},
	{	987	,	49	,	3768	},
	{	984	,	48	,	3765	},
	{	981	,	47	,	3763	},
	{	978	,	46	,	3760	},
	{	975	,	45	,	3758	},
	{	972	,	44	,	3755	},
	{	969	,	43	,	3753	},
	{	966	,	42	,	3750	},
	{	963	,	41	,	3748	},
	{	960	,	40	,	3745	},
	{	957	,	39	,	3743	},
	{	954	,	38	,	3740	},
	{	951	,	37	,	3738	},
	{	948	,	36	,	3735	},
	{	945	,	35	,	3733	},
	{	942	,	34	,	3730	},
	{	939	,	33	,	3728	},
	{	936	,	32	,	3725	},
	{	933	,	31	,	3723	},
	{	930	,	30	,	3720	},
	{	926	,	29	,	3717	},
	{	922	,	28	,	3714	},
	{	918	,	27	,	3710	},
	{	914	,	26	,	3707	},
	{	910	,	25	,	3704	},
	{	906	,	24	,	3700	},
	{	902	,	23	,	3697	},
	{	898	,	22	,	3694	},
	{	894	,	21	,	3690	},
	{	890	,	20	,	3687	},
	{	886	,	19	,	3684	},
	{	882	,	18	,	3680	},
	{	878	,	17	,	3677	},
	{	874	,	16	,	3674	},
	{	870	,	15	,	3670	},
	{	864	,	14	,	3665	},
	{	858	,	13	,	3660	},
	{	852	,	12	,	3655	},
	{	846	,	11	,	3650	},
	{	840	,	10	,	3645	},
	{	834	,	9	,	3640	},
	{	828	,	8	,	3635	},
	{	822	,	7	,	3630	},
	{	816	,	6	,	3625	},
	{	810	,	5	,	3620	},
	{	783	,	4	,	3597	},
	{	756	,	3	,	3573	},
	{	730	,	2	,	3550	},
	{	615	,	1	,	3475	},
	{	500	,	0	,	3400	},
		{0, 0, 0},

};
#else
struct batt_level_table orion_batt_level_table[] = {
	{	1460	,	100	,	4175	},
	{	1459	,	99	,	4169	},
	{	1442	,	98	,	4155	},
	{	1425	,	97	,	4141	},
	{	1407	,	96	,	4126	}, 
	{	1390	,	95	,	4111	},
	{	1380	,	94	,	4103	},
	{	1370	,	93	,	4095	},
	{	1361	,	92	,	4087	},
	{	1351	,	91	,	4079	}, 
	{	1342	,	90	,	4071	},
	{	1332	,	89	,	4062	},
	{	1322	,	88	,	4054	},
	{	1313	,	87	,	4046	},
	{	1303	,	86	,	4038	}, 
	{	1294	,	85	,	4030	},
	{	1284	,	84	,	4022	},
	{	1274	,	83	,	4014	},
	{	1265	,	82	,	4006	},
	{	1255	,	81	,	3998	}, 
	{	1246	,	80	,	3990	},
	{	1236	,	79	,	3981	},
	{	1226	,	78	,	3973	},
	{	1217	,	77	,	3965	},
	{	1207	,	76	,	3957	}, 
	{	1198	,	75	,	3949	},
	{	1188	,	74	,	3941	},
	{	1178	,	73	,	3933	},
	{	1169	,	72	,	3925	},
	{	1159	,	71	,	3917	}, 
	{	1150	,	70	,	3909	},
	{	1144	,	69	,	3904	},
	{	1138	,	68	,	3899	},
	{	1132	,	67	,	3893	},
	{	1126	,	66	,	3888	}, 
	{	1120	,	65	,	3882	},
	{	1114	,	64	,	3877	},
	{	1108	,	63	,	3871	},
	{	1102	,	62	,	3866	},
	{	1096	,	61	,	3860	}, 
	{	1090	,	60	,	3855	},
	{	1084	,	59	,	3850	},
	{	1078	,	58	,	3844	},
	{	1072	,	57	,	3839	},
	{	1066	,	56	,	3833	}, 
	{	1060	,	55	,	3828	},
	{	1054	,	54	,	3822	},
	{	1048	,	53	,	3817	},
	{	1042	,	52	,	3811	},
	{	1036	,	51	,	3806	}, 
	{	1030	,	50	,	3800	},
	{	1024	,	48	,	3795	},
	{	1018	,	46	,	3790	},
	{	1012	,	44	,	3785	},
	{	1006	,	42	,	3780	}, 
	{	1000	,	40	,	3775	},
	{	994	,	38	,	3770	},
	{	988	,	36	,	3765	},
	{	982	,	34	,	3760	},
	{	976	,	32	,	3755	}, 
	{	970	,	30	,	3750	}, 
	{	964	,	29	,	3745	},
	{	958	,	28	,	3740	},
	{	952	,	26	,	3735	},
	{	946	,	25	,	3730	}, 
	{	940	,	23	,	3725	},
	{	934	,	22	,	3720	},
	{	928	,	20	,	3715	},
	{	922	,	19	,	3710	},
	{	916	,	17	,	3705	}, 
	{	910	,	16	,	3700	}, 
	{	904	,	15	,	3695	},
	{	898	,	14	,	3690	},
	{	892	,	13	,	3685	},
	{	886	,	12	,	3680	}, 
	{	880	,	11	,	3675	},
	{	874	,	10	,	3670	},
	{	868	,	9	,	3665	},
	{	862	,	8	,	3660	},
	{	856	,	7	,	3655	}, 
	{	850	,	6	,	3650	}, 
	{	844	,	5	,	3645	},
	{	839	,	5	,	3641	},
	{	834	,	5	,	3636	},
	{	828	,	5	,	3631	}, 
	{	823	,	4	,	3627	},
	{	818	,	4	,	3622	},
	{	812	,	4	,	3617	},
	{	807	,	4	,	3613	},
	{	802	,	3	,	3608	}, 
	{	796	,	3	,	3603	},
	{	791	,	3	,	3599	},
	{	786	,	3	,	3594	},
	{	780	,	2	,	3589	},
	{	775	,	2	,	3585	}, 
	{	770	,	2	,	3580	},
	{	724	,	2	,	3550	},
	{	678	,	1	,	3520	},
	{	632	,	1	,	3490	},
	{	586	,	1	,	3460	}, 
	{	540	,	0	,	3440	},
	{	0	,	0	,	0	},
};
#endif // BATT_TABLE_CALL_BASIS

unsigned int battery_get_level(int vbatt_adc)
{
	int i; 
	unsigned int batt_level = 0;;

	BATT_LOG(batt_info.log_en, "TA_nCHG %d\n", gpio_get_value(107));

	if(vbatt_adc <= PWR_OFF_LEVEL)
		return 0;
	else if(vbatt_adc >= CHG_OFF_LEVEL)
	{

		if(!gpio_get_value(107))
			return 99;
		else
		    return 100;
	}

	
	i=1;
	for(;;)
	{
		if((vbatt_adc >= orion_batt_level_table[i].vbatt_adc) && (vbatt_adc < orion_batt_level_table[i-1].vbatt_adc))
		{
			batt_level = orion_batt_level_table[i-1].level;
			break;
		}
		if(orion_batt_level_table[i].vbatt_adc==0 && orion_batt_level_table[i].level==0)
			break;
		i++;
	}
	return batt_level;
}

unsigned int battery_get_voltage(int vbatt_adc)
{
	int i; 
	int batt_voltage = 0;;
#ifdef BATT_TABLE_CALL_BASIS
	if(vbatt_adc <= PWR_OFF_LEVEL)
		return 3400;
	else if(vbatt_adc >= CHG_OFF_LEVEL)
	{
			return 4140;
	}
#else
	if(vbatt_adc <= PWR_OFF_LEVEL)
		return 3440;
	else if(vbatt_adc >= CHG_OFF_LEVEL)
	{
			return 4175;
	}
#endif // BATT_TABLE_CALL_BASIS
	
	i=1;
	for(;;)
	{
		if((vbatt_adc >= orion_batt_level_table[i].vbatt_adc) && (vbatt_adc < orion_batt_level_table[i-1].vbatt_adc))
		{
			batt_voltage = orion_batt_level_table[i].volt;
			break;
		}
		if(orion_batt_level_table[i].vbatt_adc==0 && orion_batt_level_table[i].volt==0)
			break;
		i++;
	}
	return batt_voltage;
}

extern bridge_on;
int using_dev = 0;
int battery_compensation_val(int vbatt_adc)
{
	int compensation_val = 0, comp_dev = 0;

#ifdef BATT_TABLE_CALL_BASIS
	if(!batt_info.rep.charging_source || charger_off) // battery compensation when no charger attached
	{
		if(bridge_on){
			compensation_val += 50;
			comp_dev |= 0x0004;
		}
		if(batt_info.comp_check.camera_down){
			compensation_val += 100;
			comp_dev |= 0x0010;
		}
		if(batt_info.comp_check.camcoder_down){
			compensation_val += 20;
			comp_dev |= 0x0020;
		}
		if(batt_info.comp_check.camera_flash_down){
			compensation_val += 30;
			comp_dev |= 0x0200;
		}
		if(batt_info.comp_check.amp_down){
			compensation_val += 0;
			comp_dev |= 0x0040;
		}
		if(batt_info.comp_check.video_down){
			compensation_val += 10;
			comp_dev |= 0x0080;
		}
		if(batt_info.comp_check.talk_wcdma){
			compensation_val += 0;
			comp_dev |= 0x0400;
		}
		if(batt_info.comp_check.talk_gsm){
			compensation_val += 0;
			comp_dev |= 0x0800;
		}
		if(batt_info.comp_check.data_wcdma){
			compensation_val += 70;
			comp_dev |= 0x1000;
		}
	}

	if(batt_info.rep.charging_source && (!charger_off))
	{
		compensation_val -= 100;
	}
	else if (!comp_dev) // reverse compensation for charging or no dev used
		compensation_val -= 90;
	
		using_dev = comp_dev;
		
		if(vbatt_adc + compensation_val >= CHG_OFF_LEVEL)
			compensation_val = CHG_OFF_LEVEL-(vbatt_adc);
		BATT_LOG(batt_info.log_en, "on using devs %x comepensation value %d\n", comp_dev, compensation_val);

#else
	if(!batt_info.rep.charging_source || charger_off) // battery compensation when no charger attached
	{
		if(batt_info.comp_check.vibrator){
			compensation_val += 30;
			comp_dev |= 0x0001;
		}
		if(batt_info.comp_check.keypad_backlight){
			compensation_val += 0;
			comp_dev |= 0x0002;
		}
		if(bridge_on){
			if(current_vbatt_adc >= 770)
			{
			compensation_val += 100;
			}
			else // dimming
			{
				compensation_val += 50;
			}
			comp_dev |= 0x0004;
		}
		if(batt_info.comp_check.lcd_dimming){
			compensation_val += 20;
			comp_dev |= 0x0008;
		}
		if(batt_info.comp_check.camera_down){
			compensation_val += 100;
			comp_dev |= 0x0010;
		}
		if(batt_info.comp_check.camcoder_down){
			compensation_val += 20;
			comp_dev |= 0x0020;
		}
		if(batt_info.comp_check.amp_down){
			if(bridge_on)
			compensation_val += 10;
			else
				compensation_val += 70;
			comp_dev |= 0x0040;
		}
		if(batt_info.comp_check.video_down){
			compensation_val += 40;
			comp_dev |= 0x0080;
		}
		if(batt_info.comp_check.bt_down){
			compensation_val += 0;
			comp_dev |= 0x0100;
		}
		if(batt_info.comp_check.camera_flash_down){
			compensation_val += 30;
			comp_dev |= 0x0200;
		}
		if(batt_info.comp_check.talk_wcdma){
			compensation_val += 40;
			if(!bridge_on)
				compensation_val += 50;
			comp_dev |= 0x0400;
		}
		if(batt_info.comp_check.talk_gsm){
			compensation_val += 20;
			if(!bridge_on)
				compensation_val += 50;
			comp_dev |= 0x0800;
		}
		if(batt_info.comp_check.data_wcdma){
			compensation_val += 50;
			comp_dev |= 0x1000;
		}
		if (!comp_dev && !batt_info.rep.charging_source) // no device alive but the arm11 in resume, so compensation for that
		compensation_val += 50;
		using_dev = comp_dev;
		
		if(vbatt_adc + compensation_val >= CHG_OFF_LEVEL)
			compensation_val = CHG_OFF_LEVEL-(vbatt_adc);

		BATT_LOG(batt_info.log_en, "on using devs %x comepensation value %d\n", comp_dev, compensation_val);
	}
	else if(batt_info.rep.charging_source==1 && (!charger_off)) // when USB attached level measured higer
	{
		if(vbatt_adc <= 910)
			compensation_val -= 250;
		else if(vbatt_adc <= 1132)
			compensation_val -= 187;
		else if(vbatt_adc <= 1442)
			compensation_val -= 125;
	}
	else if(batt_info.rep.charging_source==2 && (!charger_off)) // when TA attached level measured higer
	{
		if(vbatt_adc <= 910)
			compensation_val -= 275;
		else if(vbatt_adc <= 1132)
			compensation_val -= 212;
		else if(vbatt_adc <= 1442)
			compensation_val -= 150;
	}
#endif // BATT_TABLE_CALL_BASIS
	return compensation_val;
}

int vbatt_adc_change_threshold(int curr_vbatt_adc)
{
	int threshold = 0;
#ifdef BATT_TABLE_CALL_BASIS
	if(curr_vbatt_adc >= 1100)
		threshold = 30;
	else if(curr_vbatt_adc >= 990)
		threshold = 15;
	else if(curr_vbatt_adc >= 930)
		threshold = 9;
	else if(curr_vbatt_adc >= 870)
		threshold = 12;
	else if(curr_vbatt_adc >= 810)
		threshold = 18;
	else if(curr_vbatt_adc >= 730)
		threshold = 78;
	else
		threshold = 240;
#else
	#if 0
	if(curr_vbatt_adc >= 1140)
		threshold = 30;
	else if(curr_vbatt_adc >= 1030)
		threshold = 15;
	else if(curr_vbatt_adc >= 970)
		threshold = 9;
	else if(curr_vbatt_adc >= 910)
		threshold = 12;
	else if(curr_vbatt_adc >= 850)
		threshold = 18;
	else if(curr_vbatt_adc >= 770)
		threshold = 78;
	else
		threshold = 240;
	#else
	if(curr_vbatt_adc >= 770)
		threshold = 30;
	else
		threshold = 200;
	#endif
#endif

	return threshold;
}

/* -------------------------------------------------------------------------- */

#if defined(CONFIG_DEBUG_FS)
int battery_set_charging(batt_ctl_t ctl);
static int batt_debug_set(void *data, u64 val)
{
        return battery_set_charging((batt_ctl_t) val);
}

static int batt_debug_get(void *data, u64 *val)
{
        return -ENOSYS;
}

DEFINE_SIMPLE_ATTRIBUTE(batt_debug_fops, batt_debug_get, batt_debug_set, "%llu\n");
static int __init batt_debug_init(void)
{
        struct dentry *dent;

        dent = debugfs_create_dir("battery", 0);
        if (IS_ERR(dent))
                return PTR_ERR(dent);

        debugfs_create_file("charger_state", 0644, dent, NULL, &batt_debug_fops);

        return 0;
}

device_initcall(batt_debug_init);
#endif

/* 
 *      battery_charging_ctrl - battery charing control.
 *      @ctl:                   battery control command
 *
 */
static int battery_charging_ctrl(batt_ctl_t ctl)
{
        int result = 0;

        switch (ctl) {
        case DISABLE:
                BATT_LOG(batt_info.log_en,"charger OFF\n");
                /* 0 for enable; 1 disable */
                break;
        case ENABLE_SLOW_CHG:
                BATT_LOG(batt_info.log_en,"charger ON (SLOW)\n");
                break;
        case ENABLE_FAST_CHG:
                BATT_LOG(batt_info.log_en,"charger ON (FAST)\n");
                break;
        default:
                printk(KERN_ERR "Not supported battery ctr called.!\n");
                result = -EINVAL;
                break;
        }
        
        return result;
}

int battery_set_charging(batt_ctl_t ctl)
{
        int rc;
        
        if ((rc = battery_charging_ctrl(ctl)) < 0)
                goto result;
        
        if (!battery_initial) {
                batt_info.rep.charging_enabled = ctl & 0x3;
        } else {
                mutex_lock(&batt_info.lock);
                batt_info.rep.charging_enabled = ctl & 0x3;
                mutex_unlock(&batt_info.lock);
        }
result: 
        return rc;
}

static int set_charging_status(int enable)
{
	int result;
	BATT_LOG(batt_info.log_en,"set_charging_status : %d\n", enable);
	if(!enable)
	{
		batt_info.charger_time_out = 1;
		charger_off = 1;
		batt_info.rep.charging_enabled = 4; // battery full
	}
	else
	{
		batt_info.charger_time_out = 0;
		charger_off = 0;
		batt_info.rep.charging_enabled = 1; 
	}

	result = batt_info.rep.charging_source;

	return result;
}

static int rpc_set_charger_on(int charger_off)
{
        struct set_charger_on_req {
                struct rpc_request_hdr hdr;
                uint32_t charger_on;
        } req;

        req.charger_on = cpu_to_be32(!charger_off);
        return msm_rpc_call(endpoint, SEC_PROCEDURE_SET_CHARGER_ON,
                            &req, sizeof(req), 5 * HZ);
}

static int rpc_set_charging_src(int charging_src)
{
        struct set_charging_src_req {
                struct rpc_request_hdr hdr;
                uint32_t charging_src;
        } req;

        req.charging_src = cpu_to_be32(charging_src);
        return msm_rpc_call(endpoint, SEC_PROCEDURE_SET_CHARGING_SRC,
                            &req, sizeof(req), 5 * HZ);
}

static int rpc_set_using_dev(int using_dev)
{
        struct set_charging_src_req {
                struct rpc_request_hdr hdr;
                uint32_t using_dev;
        } req;

        req.using_dev = cpu_to_be32(using_dev);
        return msm_rpc_call(endpoint, SEC_PROCEDURE_SET_USING_DEV,
                            &req, sizeof(req), 5 * HZ);
}

int battery_status_update(u32 curr_level)
{
        int notify;
		int low_battery_flag=0;
		static int previous_low_battery_flag=-1;
        if (!battery_initial)
                return 0;

        mutex_lock(&batt_info.lock);
        notify = (batt_info.rep.level != curr_level);
        batt_info.rep.level = curr_level;
        mutex_unlock(&batt_info.lock);
#if 1
	// when battery charged fully or charger attached over 5 hours, batt level must be 100
	if(batt_info.charger_time_out || batt_info.rep.charging_enabled == 4) 
	{
		batt_info.rep.level = 100;
	}
#endif
		if( curr_level <= 10 ){
			low_battery_flag=( !batt_info.rep.charging_source );
			}
		
		if( previous_low_battery_flag != low_battery_flag )
		{
			lightsensor_low_battery_flag_set( low_battery_flag );
			lcd_low_battery_flag_set_for_lightsensor( low_battery_flag );
			previous_low_battery_flag = low_battery_flag;
		}

// KTH
// TOUCH DEVICE DIE SOMETIMES. SO TOUCH_EN(16) ADDED
		gpio_direction_output( 16, 1);

#if 0 // for always update
        if (notify)
#endif
                power_supply_changed(&power_supplies[CHARGER_BATTERY]);
        return 0;
}

int current_cable; // ANUBIS_TEST
extern int fsa_suspended;

int cable_status_update(int status)
{
        int rc = 0;
        unsigned source;
	unsigned char int1, int2, dtype;

        if (!battery_initial)
                return 0;
        
	if( fsa_init_done&& !fsa_suspended){
		mdelay(200); // old charger needs more delay to detect cable type
		fsa9480_i2c_read_reg(FSA_INT1, &int1);
		mdelay(10);
		fsa9480_i2c_read_reg(FSA_DTYPE1, &dtype);
		BATT_LOG(batt_info.log_en,"FSA9480 i2c read 0x03 int1 0x%x, 0x0A dtype 0x%x\n", int1, dtype);
		if(dtype==0x00){
			status = CHARGER_BATTERY;

			batt_info.charger_time_out = 0;
			charger_off = 0;
			batt_info.rep.charging_enabled = 1; 
			}
		else if (dtype==0x04){
			status = CHARGER_USB;
			}
		else if (dtype==0x40){
			status = CHARGER_AC;
			}
		else if (dtype==0x10){ // old charger (ver IV)
			status = CHARGER_AC;
			}
		else 
			status = CHARGER_BATTERY;
		current_cable = status;
	}

        mutex_lock(&batt_info.lock);
        switch(current_cable) {
        case CHARGER_BATTERY:
                BATT_LOG(batt_info.log_en,"cable NOT PRESENT\n");
                batt_info.rep.charging_source = CHARGER_BATTERY;
                break;
        case CHARGER_USB:
                BATT_LOG(batt_info.log_en,"cable USB\n");
                batt_info.rep.charging_source = CHARGER_USB;
                break;
        case CHARGER_AC:
                BATT_LOG(batt_info.log_en,"cable AC\n");
                batt_info.rep.charging_source = CHARGER_AC;
                break;
        default:
                printk(KERN_ERR "%s: Not supported cable status received!\n",
                                __FUNCTION__);
                rc = -EINVAL;
        }
        source = batt_info.rep.charging_source;
        mutex_unlock(&batt_info.lock);

//	msm_hsusb_set_vbus_state(source == CHARGER_USB);
//	msm_hsusb_set_vbus_state(1); // temp code by gtuo.park 2009.03.02

        if (source == CHARGER_USB && !fsa_suspended) { 
		wake_lock(&vbus_wake_lock);
        } else {
                /* give userspace some time to see the uevent and update
                 * LED state or whatnot...
                 */
		wake_lock_timeout(&vbus_wake_lock, HZ / 2);
        }

        /* if the power source changes, all power supplies may change state */
        power_supply_changed(&power_supplies[CHARGER_BATTERY]);
        power_supply_changed(&power_supplies[CHARGER_USB]);
        power_supply_changed(&power_supplies[CHARGER_AC]);

        return rc;
}

#ifndef max
#define max(a,b) (((a)>(b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b) (((a)<(b)) ? (a) : (b))
#endif

unsigned int avg_vbatt_adc(int last_vbatt_adc)
{
	unsigned int result = 0, min_adc, max_adc;
	int i, j;

	for(i=0; i<12; i++)
	{
		if(i==11)
			vbatt_adc_table[i]= (unsigned int) last_vbatt_adc;
		else
		{
			vbatt_adc_table[i] = vbatt_adc_table[i+1];
			if(!vbatt_adc_table[i])
				vbatt_adc_table[i] = (unsigned int) last_vbatt_adc;
		}
		
		result += vbatt_adc_table[i];

		if(!i)
		{
			min_adc = vbatt_adc_table[i];
			max_adc = vbatt_adc_table[i];
		}
		else
		{
			min_adc = min(min_adc, vbatt_adc_table[i]);
			max_adc = max(max_adc, vbatt_adc_table[i]);
		}
		BATT_DBG(batt_info.log_en,"avg_vbatt_adc : vbatt_adc_table[%d] %d, min %d, max %d\n", i, vbatt_adc_table[i], min_adc, max_adc);
	}

		result -= (min_adc + max_adc);
		result = result/10;

	return result;
}

unsigned int avg_therm_adc(int last_therm_adc)
{
	unsigned int result = 0, min_adc, max_adc;
	int i;

	for(i=0; i<3; i++)
	{
		if(i==2)
			therm_adc_table[i]= (unsigned int) last_therm_adc;
		else
		{
			therm_adc_table[i] = therm_adc_table[i+1];
			if(!therm_adc_table[i])
				therm_adc_table[i] = (unsigned int) last_therm_adc;
		}
		
		result += therm_adc_table[i];

#if 0
		if(!i)
		{
			min_adc = therm_adc_table[i];
			max_adc = therm_adc_table[i];
		}
		else
		{
			min_adc = min(min_adc, therm_adc_table[i]);
			max_adc = max(max_adc, therm_adc_table[i]);
		}
		BATT_DBG(batt_info.log_en,"avg_therm_adc : therm_adc_table[%d] %d, min %d, max %d\n", i, therm_adc_table[i], min_adc, max_adc);
#else
		BATT_DBG(batt_info.log_en,"avg_therm_adc : therm_adc_table[%d] %d\n", i, therm_adc_table[i]);
#endif
	}

//	result -= (min_adc + max_adc);
	result = result/3;

	return result;
}

static int get_batt_level(struct battery_level_reply *buffer)
{
        struct rpc_request_hdr req;
        
        struct get_batt_level_rep {
                struct rpc_reply_hdr hdr;
                struct battery_level_reply batt_level_rep;
        } rep;
		
	int rc;
	int scaled_val;
	int diff_adc;
	static int initial_skip = 0;
	static int level_change_cnt = 0;
	int level_change_threshold;
	static int init_vbatt_adc = 0;

	if (buffer == NULL) 
		return -EINVAL;

	rc = msm_rpc_call_reply(endpoint, SEC_PROCEDURE_GET_BATT_LEVEL,
                                &req, sizeof(req),
                                &rep, sizeof(rep),
                                5 * HZ);
	if ( rc < 0 ) 
		return rc;

	mutex_lock(&batt_level_info.lock);
	buffer->batt_level = be32_to_cpu(rep.batt_level_rep.batt_level);
	mutex_unlock(&batt_level_info.lock);


	if(!initial_skip)
	{
		initial_skip += 1;
		return 0;
	}
	else if((initial_skip == 1) && gpio_get_value(107))
	{
		initial_skip += 1;
		buffer->batt_level += 50; // initial compensation
	}
			
	BATT_LOG(batt_info.log_en,"Modem original send vbatt_adc %d\n",buffer->batt_level);

	batt_info.batt_delta = battery_compensation_val(buffer->batt_level);
	buffer->batt_level += batt_info.batt_delta;

	if(ignore_prev_batt)
	{
		ignore_prev_batt = 0;
		current_vbatt_adc = (int) avg_vbatt_adc(buffer->batt_level);

		scaled_val = battery_get_level(current_vbatt_adc);
		if(pre_scaled_val > scaled_val)
		{
			if(!gpio_get_value(107)) //charging
			{
				scaled_val = pre_scaled_val;
			}
		}
		else
		{
			if(gpio_get_value(107)) //Not charging
			{
				scaled_val = pre_scaled_val;
			}
		}
		pre_scaled_val = scaled_val;

		battery_status_update(pre_scaled_val);
		BATT_LOG(batt_info.log_en, "get_batt_level : current_adc %d, scaled_val %d\n", current_vbatt_adc, pre_scaled_val);
		return 0;
	}
		
	if(current_vbatt_adc>400 && current_vbatt_adc<2000)
	{
		pre_vbatt_adc = current_vbatt_adc;
#if 1 
		diff_adc = pre_vbatt_adc - buffer->batt_level;
		level_change_threshold = vbatt_adc_change_threshold(pre_vbatt_adc);

		if((using_dev != 0x400) && (using_dev != 0x800) && (using_dev != 0x0)) // when call event or ARM11 sleep, do not apply threshold
		{
			if(diff_adc > level_change_threshold)
				buffer->batt_level = pre_vbatt_adc -level_change_threshold;
			else if(diff_adc < -level_change_threshold)
				buffer->batt_level = pre_vbatt_adc +level_change_threshold;
		}
#endif
	}

	BATT_LOG(batt_info.log_en,"Compensated and saturated vbatt_adc %d\n",buffer->batt_level);
		
	current_vbatt_adc = (int) avg_vbatt_adc(buffer->batt_level);

	diff_adc = pre_vbatt_adc - current_vbatt_adc;

	if(pre_vbatt_adc > 400 && pre_vbatt_adc < 2000)
	{
		if(!batt_info.rep.charging_source || gpio_get_value(107)) // not charging
		{
			if(current_vbatt_adc>=pre_vbatt_adc) // cannot increase batt level when discharging
				current_vbatt_adc=pre_vbatt_adc;
		}
		else
		{
			if(current_vbatt_adc<=pre_vbatt_adc) // cannot decrease batt level when charging
				current_vbatt_adc=pre_vbatt_adc;
		}
	}
			
	  scaled_val = battery_get_level(current_vbatt_adc);
#if 1  // battery level changed only when new level differs from previous level over 3 times
	if(pre_scaled_val >0 && pre_scaled_val < 101)
	{
		if((using_dev != 0x400) && (using_dev != 0x800) && (using_dev != 0x0)) // do not apply step changing, when call or ARM11 sleep
		{
			if(pre_scaled_val > scaled_val)
			{
				if(!batt_info.rep.charging_source || gpio_get_value(107)) // not charging
				{
				level_change_cnt += 1;
				if(level_change_cnt >=3)
				{
					level_change_cnt = 0;
					scaled_val = pre_scaled_val-1;
				}
				else
				{
					scaled_val = pre_scaled_val;
				}
			}
				else
					scaled_val = pre_scaled_val;
			}
			else if(pre_scaled_val < scaled_val)
			{
				if(!gpio_get_value(107)) // charging
				{
				level_change_cnt += 1;
				if(level_change_cnt >=3)
				{
					level_change_cnt = 0;
					scaled_val = pre_scaled_val+1;
			  	}
				else
				{
					scaled_val = pre_scaled_val;
				}
			}
				else
					scaled_val = pre_scaled_val;
			}
			else if(pre_scaled_val == scaled_val)
			{
				level_change_cnt = 0;
				scaled_val = pre_scaled_val;
			}
	  	}
	}
#endif
	pre_scaled_val = scaled_val;
	battery_status_update(scaled_val);
		  
	batt_info.batt_adc_comp = buffer->batt_level;
	batt_info.batt_voltage_comp = battery_get_voltage(batt_info.batt_adc_comp);
       BATT_LOG(batt_info.log_en,"handle_battery_call : dem_battery_update: vbatt_adc=%d, level=%d, level_change_cnt=%d\n",current_vbatt_adc, scaled_val, level_change_cnt);
	BATT_LOG(batt_info.log_en,"Testmode therm_adc=%d, vbatt_adc=%d, level=%d mV\n",batt_info.rep.batt_therm, batt_info.batt_adc_comp, batt_info.batt_voltage_comp);
	cable_status_update(batt_info.rep.charging_source);

	return 0;
}

static int get_batt_info(struct battery_info_reply *buffer)
{
		int received_level;

        struct rpc_request_hdr req;
        
        struct get_batt_info_rep {
                struct rpc_reply_hdr hdr;
                struct battery_info_reply info;
        } rep;
        
        int rc;
	int rc2, received_adc, rc3;


        if (buffer == NULL) 
                return -EINVAL;

        rc = msm_rpc_call_reply(endpoint, SEC_PROCEDURE_GET_BATT_INFO,
                                &req, sizeof(req),
                                &rep, sizeof(rep),
                                5 * HZ);
        if ( rc < 0 ) 
                return rc;
        
        mutex_lock(&batt_info.lock);
        buffer->batt_id                 = be32_to_cpu(rep.info.batt_id);
        buffer->batt_vol                = be32_to_cpu(rep.info.batt_vol);
        buffer->batt_temp               = be32_to_cpu(rep.info.batt_temp);
        buffer->batt_current            = be32_to_cpu(rep.info.batt_current);

        buffer->full_bat                = be32_to_cpu(rep.info.full_bat);
//	buffer->level			= battery_get_level(current_vbatt_adc);
	buffer->batt_therm			= avg_therm_adc(be32_to_cpu(rep.info.batt_therm)); 
	
        mutex_unlock(&batt_info.lock);

#ifdef CHARGER_CONTROL // charger on off by vbatt_adc and therm_adc
	if(batt_info.rep.charging_source == CHARGER_BATTERY)
	{
		charger_off = 0;
		out_count = 0;
		rc2 = rpc_set_charger_on(charger_off); // charger on
		if ( rc2 < 0 ) 
			BATT("rpc_set_charger_on fail\n");
	}

	if((buffer->batt_therm>=CHG_OFF_HIGH_TEMP)||(buffer->batt_therm<=CHG_OFF_LOW_TEMP)||(current_vbatt_adc>=CHG_OFF_LEVEL && gpio_get_value(107))) // charger off
	{
		if(charger_off)
			out_count = 0;
		else
			out_count += 1;

		if(out_count >= 3)
		{
			out_count = 0;
			charger_off = 1;
			rc2 = rpc_set_charger_on(charger_off);
			if ( rc2 < 0 ) 
				BATT("rpc_set_charger_on fail\n");
			if(buffer->batt_therm<=CHG_OFF_LOW_TEMP)
			{
				BATT("charger off : phone temp too low\n");
				buffer->charging_enabled = 2; // temperature too low
			}
			else if (buffer->batt_therm>=CHG_OFF_HIGH_TEMP)
			{
				BATT("charger off : phone temp too high\n");
				buffer->charging_enabled = 3; // temperature too high
			}
			else
			{
				BATT("charger off : battery full\n");
				buffer->charging_enabled = 4; // battery full
			}

			BATT_LOG(batt_info.log_en, "charger off : charging_enabled %d\n", buffer->charging_enabled);
		}
	}
	else if(((buffer->batt_therm<=CHG_RESTART_HIGH_TEMP)&&(buffer->batt_therm>=CHG_RESTART_LOW_TEMP))&&(current_vbatt_adc<=CHG_RESTART_LEVEL)) // charger on
	{
#if 0
		if(charger_off)
			out_count += 1;
		else
			out_count = 0;

		if(out_count >= 1)
#else
		if(charger_off)
			BATT("charging restart\n");
		
		if(1)
#endif
		{
			out_count = 0;
			charger_off = 0;	
			rc2 = rpc_set_charger_on(charger_off);
			if ( rc2 < 0 ) 
				BATT("rpc_set_charger_on fail\n");

			if(batt_info.rep.charging_enabled == 2 || batt_info.rep.charging_enabled == 3)
			{
				buffer->charging_enabled = 1; // charger enable
			}

		}
	}

#endif
	rc3 = rpc_set_charging_src(batt_info.rep.charging_source);
	if ( rc3 < 0 ) 
		BATT("rpc_set_charging_src fail\n");

	rc3 = rpc_set_using_dev(using_dev);
	if ( rc3 < 0 ) 
		BATT("rpc_set_using_dev fail\n");

	BATT_LOG(batt_info.log_en,"get_batt_info : vbatt_adc=%d, batt_level=%d, therm=%d, enabled=%d, out_count=%d\n",current_vbatt_adc, buffer->level, buffer->batt_therm, buffer->charging_enabled, out_count);
        return 0;
}


/* -------------------------------------------------------------------------- */
static int power_get_property(struct power_supply *psy, 
                                    enum power_supply_property psp,
                                    union power_supply_propval *val)
{
        charger_type_t charger;
        
        mutex_lock(&batt_info.lock);
        charger = batt_info.rep.charging_source;
        mutex_unlock(&batt_info.lock);

        switch (psp) {
        case POWER_SUPPLY_PROP_ONLINE:
                if (psy->type == POWER_SUPPLY_TYPE_MAINS)
                        val->intval = (charger ==  CHARGER_AC ? 1 : 0);
                else if (psy->type == POWER_SUPPLY_TYPE_USB)
                        val->intval = (charger ==  CHARGER_USB ? 1 : 0);
                else
                        val->intval = 0;
                break;
        default:
                return -EINVAL;
        }
        
        return 0;
}

static charger_type_t prev_charger=0; 
static int battery_get_charging_status(void)
{
        u32 level;
        charger_type_t charger; 
        int ret;
        
        mutex_lock(&batt_info.lock);
        charger = batt_info.rep.charging_source;

        switch (charger) {
        case CHARGER_BATTERY:
                ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
                break;
        case CHARGER_USB:
        case CHARGER_AC:
                level = batt_info.rep.level;
                if (level == 100)
                        ret = POWER_SUPPLY_STATUS_FULL;
                else
                {
#ifdef CHARGER_CONTROL_
                	if(!gpio_get_value(107))
                        ret = POWER_SUPPLY_STATUS_CHARGING;
			else{
				BATT_LOG(batt_info.log_en, "battery_get_charging_status : charger off\n");
				ret = POWER_SUPPLY_STATUS_DISCHARGING;
			}
#else
                        ret = POWER_SUPPLY_STATUS_CHARGING;
#endif
                }

                break;
        default:
                ret = POWER_SUPPLY_STATUS_UNKNOWN;
        }
        mutex_unlock(&batt_info.lock);

#if 0
	if(batt_info.charger_time_out)
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
#endif
        return ret;
}

static int battery_get_property(struct power_supply *psy, 
                                    enum power_supply_property psp,
                                    union power_supply_propval *val)
{
        switch (psp) {
        case POWER_SUPPLY_PROP_STATUS:
                val->intval = battery_get_charging_status();
                break;
        case POWER_SUPPLY_PROP_HEALTH:
                val->intval = POWER_SUPPLY_HEALTH_GOOD;
                break;
        case POWER_SUPPLY_PROP_PRESENT:
                val->intval = batt_info.present;
                break;
        case POWER_SUPPLY_PROP_TECHNOLOGY:
                val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
                break;
        case POWER_SUPPLY_PROP_CAPACITY:
                mutex_lock(&batt_info.lock);
                val->intval = batt_info.rep.level;
                mutex_unlock(&batt_info.lock);
                break;
        default:                
                return -EINVAL;
        }
        
        return 0;
}

#define SEC_BATTERY_ATTR(_name)                                                 \
{                                                                               \
        .attr = { .name = #_name, .mode = 0777, .owner = THIS_MODULE },      \
        .show = battery_show_property,                                      \
        .store = battery_store_property,                                                          \
}

static struct device_attribute battery_attrs[] = {
        SEC_BATTERY_ATTR(batt_id),
        SEC_BATTERY_ATTR(batt_vol),
        SEC_BATTERY_ATTR(batt_temp),
        SEC_BATTERY_ATTR(batt_current),
        SEC_BATTERY_ATTR(charging_source),
        SEC_BATTERY_ATTR(charging_enabled),
        SEC_BATTERY_ATTR(full_bat),
	SEC_BATTERY_ATTR(batt_therm),
#if 1 // for battery level compensation
	SEC_BATTERY_ATTR(vibrator),
	SEC_BATTERY_ATTR(keypad_backlight),
	SEC_BATTERY_ATTR(lcd_backlight),
	SEC_BATTERY_ATTR(lcd_dimming),
	SEC_BATTERY_ATTR(camera_down),
	SEC_BATTERY_ATTR(camcoder_down),	
	SEC_BATTERY_ATTR(amp_down),
	SEC_BATTERY_ATTR(video_down),
	SEC_BATTERY_ATTR(bt_down),
	SEC_BATTERY_ATTR(camera_flash_down),
	SEC_BATTERY_ATTR(talk_wcdma),
	SEC_BATTERY_ATTR(talk_gsm),
	SEC_BATTERY_ATTR(data_wcdma),
#endif
	SEC_BATTERY_ATTR(batt_delta),
	SEC_BATTERY_ATTR(batt_adc_comp),
	SEC_BATTERY_ATTR(batt_voltage_comp),
	SEC_BATTERY_ATTR(charger_time_out),
	SEC_BATTERY_ATTR(log_en),
};

enum {
        BATT_ID = 0,
        BATT_VOL,
        BATT_TEMP,
        BATT_CURRENT,
        CHARGING_SOURCE,
        CHARGING_ENABLED,
        FULL_BAT,
	BATT_THERM,
#if 1 // for battery level compensation
	VIBRATOR,
	KEYPAD_BACKLIGHT,
	LCD_BACKLIGHT,
	LCD_DIMMING,
	CAMERA_DOWN,
	CAMCODER_DOWN,
	AMP_DOWN,
	VIDEO_DOWN,
	BT_DOWN,
	CAMERA_FLASH_DOWN,
	TALK_WCDMA,
	TALK_GSM,
	DATA_WCDMA,
#endif
	BATT_DELTA,
	BATT_ADC_COMP,
	BATT_VOLTAGE_COMP,
	CHARGER_TIME_OUT,
	LOG_EN,
};

static int rpc_set_delta(unsigned delta)
{
        struct set_batt_delta_req {
                struct rpc_request_hdr hdr;
                uint32_t data;
        } req;

        req.data = cpu_to_be32(delta);
        return msm_rpc_call(endpoint, SEC_PROCEDURE_SET_BATT_DELTA,
                            &req, sizeof(req), 5 * HZ);
}


static ssize_t battery_set_delta(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
        int rc;
        unsigned long delta = 0;
        
        delta = simple_strtoul(buf, NULL, 10);

        if (delta > 100)
                return -EINVAL;

        mutex_lock(&batt_info.rpc_lock);
//        rc = rpc_set_delta(delta);
        mutex_unlock(&batt_info.rpc_lock);
        if (rc < 0)
                return rc;
        return count;
}

static struct device_attribute set_delta_attrs[] = {
        __ATTR(delta, S_IWUSR | S_IWGRP, NULL, battery_set_delta),
};

static int battery_create_attrs(struct device * dev)
{
        int i, j, rc;
        
        for (i = 0; i < ARRAY_SIZE(battery_attrs); i++) {
                rc = device_create_file(dev, &battery_attrs[i]);
                if (rc)
                        goto attrs_failed;
        }

        for (j = 0; j < ARRAY_SIZE(set_delta_attrs); j++) {
                rc = device_create_file(dev, &set_delta_attrs[j]);
                if (rc)
                        goto delta_attrs_failed;
        }
        
        goto succeed;
        
attrs_failed:
        while (i--)
                device_remove_file(dev, &battery_attrs[i]);
delta_attrs_failed:
        while (j--)
                device_remove_file(dev, &set_delta_attrs[i]);
succeed:        
        return rc;
}

static ssize_t battery_show_property(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
        int i = 0;
        const ptrdiff_t off = attr - battery_attrs;
        
        /* rpc lock is used to prevent two threads from calling
         * into the get info rpc at the same time
         */

        mutex_lock(&batt_info.rpc_lock);
        /* check cache time to decide if we need to update */
        if (batt_info.update_time &&
            time_before(jiffies, batt_info.update_time +
                                msecs_to_jiffies(cache_time)))
                goto dont_need_update;
        
        if (get_batt_info(&batt_info.rep) < 0) {
                printk(KERN_ERR "%s: rpc failed!!!\n", __FUNCTION__);
        } else {

                batt_info.update_time = jiffies;
        }
dont_need_update:
        mutex_unlock(&batt_info.rpc_lock);

        mutex_lock(&batt_info.lock);
        switch (off) {
        case BATT_ID:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.rep.batt_id);
                break;
        case BATT_VOL:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.rep.batt_vol);
                break;
        case BATT_TEMP:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.rep.batt_temp);
                break;
        case BATT_CURRENT:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.rep.batt_current);
                break;
        case CHARGING_SOURCE:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.rep.charging_source);
                break;
        case CHARGING_ENABLED:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.rep.charging_enabled);
                break;          
        case FULL_BAT:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.rep.full_bat);
                break;
        case BATT_THERM:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.rep.batt_therm);
                break;
#if 1 // for battery level compensation
        case VIBRATOR:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.vibrator);
                break;
        case KEYPAD_BACKLIGHT:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.keypad_backlight);
                break;
        case LCD_BACKLIGHT:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.lcd_backlight);
                break;
        case LCD_DIMMING:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.lcd_dimming);
                break;
        case CAMERA_DOWN:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.camera_down);
                break;
        case CAMCODER_DOWN:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.camcoder_down);
                break;
        case AMP_DOWN:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.amp_down);
                break;
        case VIDEO_DOWN:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.video_down);
                break;
        case BT_DOWN:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.bt_down);
                break;
        case CAMERA_FLASH_DOWN:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.camera_flash_down);
                break;
        case TALK_WCDMA:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.talk_wcdma);
                break;
        case TALK_GSM:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.talk_gsm);
                break;
        case DATA_WCDMA:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.comp_check.data_wcdma);
                break;
#endif
        case BATT_DELTA:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.batt_delta);
                break;
        case BATT_ADC_COMP:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.batt_adc_comp);
                break;
        case BATT_VOLTAGE_COMP:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.batt_voltage_comp);
                break;
        case CHARGER_TIME_OUT:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.charger_time_out);
                break;
        case LOG_EN:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               batt_info.log_en);
                break;
        default:
                i = -EINVAL;
        }       
        mutex_unlock(&batt_info.lock);
        
        return i;
}

static ssize_t battery_store_property(struct device *dev, 
					struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;
	int i = 0;
	const ptrdiff_t offset = attr - battery_attrs;

	mutex_lock(&batt_info.lock);

       switch (offset) {
#if 1 // for battery level compensation
        case VIBRATOR:
		batt_info.comp_check.vibrator = state;
                break;
        case KEYPAD_BACKLIGHT:
		batt_info.comp_check.keypad_backlight= state;
                break;
        case LCD_BACKLIGHT:
		batt_info.comp_check.lcd_backlight= state;
                break;
        case LCD_DIMMING:
		batt_info.comp_check.lcd_dimming= state;
                break;
        case CAMERA_DOWN:
		batt_info.comp_check.camera_down= state;
                break;
        case CAMCODER_DOWN:
		batt_info.comp_check.camcoder_down= state;
                break;
        case AMP_DOWN:
		batt_info.comp_check.amp_down= state;
                break;
        case VIDEO_DOWN:
		batt_info.comp_check.video_down= state;
                break;
        case BT_DOWN:
		batt_info.comp_check.bt_down= state;
                break;
        case CAMERA_FLASH_DOWN:
		batt_info.comp_check.camera_flash_down= state;
                break;
        case TALK_WCDMA:
		batt_info.comp_check.talk_wcdma= state;
                break;
        case TALK_GSM:
		batt_info.comp_check.talk_gsm= state;
                break;
        case DATA_WCDMA:
		batt_info.comp_check.data_wcdma= state;
                break;
#endif
        case BATT_DELTA:
		batt_info.batt_delta= state;
                break;
        case BATT_ADC_COMP:
		batt_info.batt_adc_comp= state;
                break;
        case BATT_VOLTAGE_COMP:
		batt_info.batt_voltage_comp= state;
                break;
        case CHARGER_TIME_OUT:
		batt_info.charger_time_out= state;
                break;
        case LOG_EN:
		batt_info.log_en= state;
                break;
        default:
                i = -EINVAL;
        }       
        mutex_unlock(&batt_info.lock);

	on_call = batt_info.comp_check.talk_wcdma | batt_info.comp_check.talk_gsm;
	return ret;

}

static void batt_level_work_func(struct work_struct *work)
{
        mutex_lock(&batt_level_info.rpc_lock);
        if (get_batt_level(&batt_level_info.rep) < 0)
                printk(KERN_ERR "%s: get level info failed\n", __FUNCTION__);
        mutex_unlock(&batt_level_info.rpc_lock);
}

static irqreturn_t batt_level_update_isr(int irq, void *dev_id)
{
	queue_work(batt_level_wq , &batt_level_info.work );
	return IRQ_HANDLED;
}  

static int battery_probe(struct platform_device *pdev)
{
        int i;
        int rc = 0, r;

        /* init structure data member */
	printk(KERN_INFO "Battery: probe\n");
        batt_info.update_time       = jiffies;
        batt_info.present           = 1; 		// TODO : How to find out batt presence....?
        
        /* init rpc */
        endpoint = msm_rpc_connect(APP_BATT_PROG, APP_BATT_VER, 0);
        if (IS_ERR(endpoint)) {
                printk(KERN_ERR "%s: init rpc failed! rc = %ld\n",
                       __FUNCTION__, PTR_ERR(endpoint));
                return rc;
        }

        /* init power supplier framework */
        for (i = 0; i < ARRAY_SIZE(power_supplies); i++) {
                rc = power_supply_register(&pdev->dev, &power_supplies[i]);
                if (rc)
                        printk(KERN_ERR "Failed to register power supply (%d)\n", rc);  
        }

        /* create sec detail attributes */
        battery_create_attrs(power_supplies[CHARGER_BATTERY].dev);

        /* After battery driver gets initialized, send rpc request to inquiry
         * the battery status in case of we lost some info
         */
        battery_initial = 1;

        mutex_lock(&batt_info.rpc_lock);
        if (get_batt_info(&batt_info.rep) < 0)
                printk(KERN_ERR "%s: get info failed\n", __FUNCTION__);

        cable_status_update(batt_info.rep.charging_source);
        battery_charging_ctrl(batt_info.rep.charging_enabled ?
                              ENABLE_SLOW_CHG : DISABLE);

        if (rpc_set_delta(1) < 0)
                printk(KERN_ERR "%s: set delta failed\n", __FUNCTION__);

        batt_info.update_time = jiffies;
        mutex_unlock(&batt_info.rpc_lock);

        if (batt_info.rep.charging_enabled == 0)
                battery_charging_ctrl(DISABLE);
        
	r = gpio_tlmm_config(GPIO_CFG(107, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
	if(r<0)
		printk(KERN_ERR "%s: TA_nCHG config failed\n", __FUNCTION__);

        mutex_lock(&batt_level_info.rpc_lock);
        if (get_batt_level(&batt_level_info.rep) < 0)
                printk(KERN_ERR "%s: get level info failed\n", __FUNCTION__);
        mutex_unlock(&batt_level_info.rpc_lock);

	batt_level_wq= create_singlethread_workqueue("batt_level_wq");
	INIT_WORK(&batt_level_info.work, batt_level_work_func);

	r = request_irq(INT_A9_M2A_2, batt_level_update_isr, IRQF_TRIGGER_RISING, "batt_level_update", NULL);

	if (r) {
		free_irq(INT_A9_M2A_2, 0);
		return -1;
	}

	r = enable_irq_wake(INT_A9_M2A_2);
	if (r) {
		return -1;
	}

        return 0;
}

int battery_suspend(void)
{
    int ret, i; 
	for(i=0; i<12; i++)
	{
		vbatt_adc_table[i] = 0;
//		therm_adc_table[i] = 0;
	}
	ignore_prev_batt = 1;
    return 0;
}

int battery_resume(void)
{
    int ret, i; 
/*	for(i=0; i<3; i++)
	{
		vbatt_adc_table[i] = 0;
		therm_adc_table[i] = 0;
	}
*/
    return 0;
}


static struct platform_driver battery_driver = {
        .probe  = battery_probe,
	.suspend	= battery_suspend,
	.resume	= battery_resume,
        .driver = {
                .name   = APP_BATT_PDEV_NAME,
                .owner  = THIS_MODULE,
        },
};

/* batt_mtoa server definitions */
#define BATT_MTOA_PROG                          0x3000008e
#define BATT_MTOA_VERS                          0x00010001
#define RPC_BATT_MTOA_NULL                      0
#define RPC_BATT_MTOA_SET_CHARGING_PROC         1
#define RPC_BATT_MTOA_CABLE_STATUS_UPDATE_PROC  2
#define RPC_BATT_MTOA_LEVEL_UPDATE_PROC         3

struct rpc_batt_mtoa_set_charging_args {
        int enable;
};

struct rpc_batt_mtoa_cable_status_update_args {
        int status;
};

struct rpc_dem_battery_update_args {
        uint32_t level;
};

static int handle_battery_call(struct msm_rpc_server *server,
                               struct rpc_request_hdr *req, unsigned len)
{       
		int scaled_val;
		int diff_adc;
		static int initial_skip = 0;
		static int level_change_cnt = 0;
		int level_change_threshold;
		static int init_vbatt_adc = 0;

        switch (req->procedure) {
        case RPC_BATT_MTOA_NULL:
                return 0;

        case RPC_BATT_MTOA_SET_CHARGING_PROC: {
                struct rpc_batt_mtoa_set_charging_args *args;
                args = (struct rpc_batt_mtoa_set_charging_args *)(req + 1);
                args->enable = be32_to_cpu(args->enable);
//                battery_set_charging(args->enable);
		set_charging_status(args->enable);
		BATT("RPC_BATT_MTOA_SET_CHARGING_PROC %d\n", args->enable);
                return 0;
        }
        case RPC_BATT_MTOA_CABLE_STATUS_UPDATE_PROC: {
                struct rpc_batt_mtoa_cable_status_update_args *args;
                args = (struct rpc_batt_mtoa_cable_status_update_args *)(req + 1);
                args->status = be32_to_cpu(args->status);
		BATT_LOG(batt_info.log_en,"handle_battery_call : cable_status_update: status=%d\n",args->status);

                cable_status_update(args->status);
                return 0;
        }
        case RPC_BATT_MTOA_LEVEL_UPDATE_PROC: {
                struct rpc_dem_battery_update_args *args;
                args = (struct rpc_dem_battery_update_args *)(req + 1);
                args->level = be32_to_cpu(args->level);

		if(!initial_skip)
		{
			initial_skip += 1;
			return 0;
		}
		else if((initial_skip == 1) && gpio_get_value(107))
		{
			initial_skip += 1;
			args->level += 50; // initial compensation
		}
			

		BATT_LOG(batt_info.log_en,"Modem original send vbatt_adc %d\n",args->level);

		batt_info.batt_delta = battery_compensation_val(args->level);
		args->level += batt_info.batt_delta;

		if(ignore_prev_batt)
		{
			ignore_prev_batt = 0;
			current_vbatt_adc = (int) avg_vbatt_adc(args->level);

			scaled_val = battery_get_level(current_vbatt_adc);
			if(pre_scaled_val > scaled_val)
			{
				if(!gpio_get_value(107)) //charging
				{
					scaled_val = pre_scaled_val;
				}
			}
			else
			{
				if(gpio_get_value(107)) //Not charging
				{
					scaled_val = pre_scaled_val;
				}
			}
			pre_scaled_val = scaled_val;

			battery_status_update(pre_scaled_val);
			BATT_LOG(batt_info.log_en, "handle_battery_call : current_adc %d, scaled_val %d\n", current_vbatt_adc, pre_scaled_val);
			return 0;
		}
		
		if(current_vbatt_adc>400 && current_vbatt_adc<2000)
		{
			pre_vbatt_adc = current_vbatt_adc;
#if 1 
			diff_adc = pre_vbatt_adc - args->level;
			level_change_threshold = vbatt_adc_change_threshold(pre_vbatt_adc);

			if((using_dev != 0x400) && (using_dev != 0x800) && (using_dev != 0x0)) // when call event or ARM11 sleep, do not apply threshold
			{
				if(diff_adc > level_change_threshold)
					args->level = pre_vbatt_adc -level_change_threshold;
				else if(diff_adc < -level_change_threshold)
					args->level = pre_vbatt_adc +level_change_threshold;
			}
#endif
		}
		BATT_LOG(batt_info.log_en,"Compensated and saturated vbatt_adc %d\n",args->level);
		
		current_vbatt_adc = (int) avg_vbatt_adc(args->level);

		diff_adc = pre_vbatt_adc - current_vbatt_adc;

		if(pre_vbatt_adc > 400 && pre_vbatt_adc < 2000)
		{
			if(!batt_info.rep.charging_source || gpio_get_value(107)) // not charging
			{
				if(current_vbatt_adc>=pre_vbatt_adc) // cannot increase batt level when discharging
					current_vbatt_adc=pre_vbatt_adc;
			}
			else
			{
				if(current_vbatt_adc<=pre_vbatt_adc) // cannot decrease batt level when charging
					current_vbatt_adc=pre_vbatt_adc;
			}
		}
			
		  scaled_val = battery_get_level(current_vbatt_adc);
#if 1  // battery level changed only when new level differs from previous level over 3 times
		  if(pre_scaled_val >0 && pre_scaled_val < 101)
		{
			  if((using_dev != 0x400) && (using_dev != 0x800) && (using_dev != 0x0)) // do not apply step changing, when call or ARM11 sleep
			  {
				if(pre_scaled_val > scaled_val)
				{
					if(!batt_info.rep.charging_source || gpio_get_value(107)) // not charging
					{
						if(pre_scaled_val>=20) // step changing applied only battery level higer than 3.65V
						{
							level_change_cnt += 1;
							if(level_change_cnt >=3)
							{
								level_change_cnt = 0;
								scaled_val = pre_scaled_val-1;
							}
							else
							{
								scaled_val = pre_scaled_val;
							}
						}
						else
						{
							level_change_cnt = 0;
							scaled_val = pre_scaled_val-1;
						}
					}
					else
						scaled_val = pre_scaled_val;
				}
				else if(pre_scaled_val < scaled_val)
				{
					if(!gpio_get_value(107)) // charging
					{
						if(pre_scaled_val>=20) // step changing applied only battery level higer than 3.65V
						{
							level_change_cnt += 1;
							if(level_change_cnt >=3)
							{
								level_change_cnt = 0;
								scaled_val = pre_scaled_val+1;
						  	}
							else
							{
								scaled_val = pre_scaled_val;
							}
						}
						else
						{
							level_change_cnt = 0;
							scaled_val = pre_scaled_val+1;
						}
					}
					else
						scaled_val = pre_scaled_val;
				}
				else if(pre_scaled_val == scaled_val)
				{
					level_change_cnt = 0;
					scaled_val = pre_scaled_val;
				}
		  	}
		  }
#endif
		pre_scaled_val = scaled_val;
                battery_status_update(scaled_val);
		  
		  batt_info.batt_adc_comp = args->level;
		  batt_info.batt_voltage_comp = battery_get_voltage(batt_info.batt_adc_comp);
                BATT_LOG(batt_info.log_en,"handle_battery_call : dem_battery_update: vbatt_adc=%d, level=%d, level_change_cnt=%d\n",current_vbatt_adc, scaled_val, level_change_cnt);
		BATT_LOG(batt_info.log_en,"Testmode therm_adc=%d, vbatt_adc=%d, level=%d mV\n",batt_info.rep.batt_therm, batt_info.batt_adc_comp, batt_info.batt_voltage_comp);
		cable_status_update(batt_info.rep.charging_source);

                return 0;
        }
        default:
                printk(KERN_ERR "%s: program 0x%08x:%d: unknown procedure %d\n",
                       __FUNCTION__, req->prog, req->vers, req->procedure);
                return -ENODEV;
        }
}

static struct msm_rpc_server battery_server = {
	.prog = BATT_MTOA_PROG,
	.vers = BATT_MTOA_VERS,
	.rpc_call = handle_battery_call,
};

static int __init battery_init(void)
{
	int rc;
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	mutex_init(&batt_info.lock);
	mutex_init(&batt_info.rpc_lock);
	mutex_init(&batt_level_info.lock);
	mutex_init(&batt_level_info.rpc_lock);
	msm_rpc_create_server(&battery_server);
	platform_driver_register(&battery_driver);
	return 0;
}

module_init(battery_init);
MODULE_DESCRIPTION("ORION Battery Driver");
MODULE_LICENSE("GPL");

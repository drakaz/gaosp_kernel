/* drivers/video/msm/panel/mddi/mddi_toshiba_smd.c
**
** Support for toshiba mddi and smd oled client devices which require no
** special initialization code.
**
** Copyright (C) 2008 Samsung Electronics Incorporated
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
*/
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <mach/gpio.h>
// hsil
#include <linux/device.h>
#include <mach/vreg.h>
#include <linux/leds.h>
#include <linux/delay.h>

#define _DEBUG
#ifdef _DEBUG
#define dprintk(s, args...) printk("[OLED] %s:%d - " s, __func__, __LINE__,  ##args)
#else
#define dprintk(s, args...)
#endif  /* _DEBUG */

#define SMD_AUTO_POWER_CMD
#define SMD_VYSNC_50HZ_CMD
//#define SMD_OLD_GAMMA_CMD

#define MDDI_CLIENT_CORE_BASE  0x108000
#define LCD_CONTROL_BLOCK_BASE 0x110000
#define SPI_BLOCK_BASE         0x120000
#define PWM_BLOCK_BASE         0x140000
#define GPIO_BLOCK_BASE        0x150000
#define SYSTEM_BLOCK1_BASE     0x160000
#define SYSTEM_BLOCK2_BASE     0x170000

#define TTBUSSEL    (MDDI_CLIENT_CORE_BASE|0x18)
#define DPSET0      (MDDI_CLIENT_CORE_BASE|0x1C)
#define DPSET1      (MDDI_CLIENT_CORE_BASE|0x20)
#define DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define DPRUN       (MDDI_CLIENT_CORE_BASE|0x28)
#define SYSCKENA    (MDDI_CLIENT_CORE_BASE|0x2C)

#define BITMAP0     (MDDI_CLIENT_CORE_BASE|0x44)
#define BITMAP1     (MDDI_CLIENT_CORE_BASE|0x48)
#define BITMAP2     (MDDI_CLIENT_CORE_BASE|0x4C)
#define BITMAP3     (MDDI_CLIENT_CORE_BASE|0x50)
#define BITMAP4     (MDDI_CLIENT_CORE_BASE|0x54)

#define SRST        (LCD_CONTROL_BLOCK_BASE|0x00)
#define PORT_ENB    (LCD_CONTROL_BLOCK_BASE|0x04)
#define START       (LCD_CONTROL_BLOCK_BASE|0x08)
#define PORT        (LCD_CONTROL_BLOCK_BASE|0x0C)

#define INTFLG      (LCD_CONTROL_BLOCK_BASE|0x18)
#define INTMSK      (LCD_CONTROL_BLOCK_BASE|0x1C)
#define MPLFBUF     (LCD_CONTROL_BLOCK_BASE|0x20)

#define PXL         (LCD_CONTROL_BLOCK_BASE|0x30)
#define HCYCLE      (LCD_CONTROL_BLOCK_BASE|0x34)
#define HSW         (LCD_CONTROL_BLOCK_BASE|0x38)
#define HDE_START   (LCD_CONTROL_BLOCK_BASE|0x3C)
#define HDE_SIZE    (LCD_CONTROL_BLOCK_BASE|0x40)
#define VCYCLE      (LCD_CONTROL_BLOCK_BASE|0x44)
#define VSW         (LCD_CONTROL_BLOCK_BASE|0x48)
#define VDE_START   (LCD_CONTROL_BLOCK_BASE|0x4C)
#define VDE_SIZE    (LCD_CONTROL_BLOCK_BASE|0x50)
#define WAKEUP      (LCD_CONTROL_BLOCK_BASE|0x54)
#define REGENB      (LCD_CONTROL_BLOCK_BASE|0x5C)
#define VSYNIF      (LCD_CONTROL_BLOCK_BASE|0x60)
#define WRSTB       (LCD_CONTROL_BLOCK_BASE|0x64)
#define RDSTB       (LCD_CONTROL_BLOCK_BASE|0x68)
#define ASY_DATA    (LCD_CONTROL_BLOCK_BASE|0x6C)
#define ASY_DATB    (LCD_CONTROL_BLOCK_BASE|0x70)
#define ASY_DATC    (LCD_CONTROL_BLOCK_BASE|0x74)
#define ASY_DATD    (LCD_CONTROL_BLOCK_BASE|0x78)
#define ASY_DATE    (LCD_CONTROL_BLOCK_BASE|0x7C)
#define ASY_DATF    (LCD_CONTROL_BLOCK_BASE|0x80)
#define ASY_DATG    (LCD_CONTROL_BLOCK_BASE|0x84)
#define ASY_DATH    (LCD_CONTROL_BLOCK_BASE|0x88)
#define ASY_CMDSET  (LCD_CONTROL_BLOCK_BASE|0x8C)
#define MONI        (LCD_CONTROL_BLOCK_BASE|0xB0)
#define VPOS        (LCD_CONTROL_BLOCK_BASE|0xC0)

#define SSICTL      (SPI_BLOCK_BASE|0x00)
#define SSITIME     (SPI_BLOCK_BASE|0x04)
#define SSITX       (SPI_BLOCK_BASE|0x08)
#define SSIINTS     (SPI_BLOCK_BASE|0x14)

#define TIMER0LOAD    (PWM_BLOCK_BASE|0x00)
#define TIMER0CTRL    (PWM_BLOCK_BASE|0x08)
#define PWM0OFF       (PWM_BLOCK_BASE|0x1C)
#define TIMER1LOAD    (PWM_BLOCK_BASE|0x20)
#define TIMER1CTRL    (PWM_BLOCK_BASE|0x28)
#define PWM1OFF       (PWM_BLOCK_BASE|0x3C)
#define TIMER2LOAD    (PWM_BLOCK_BASE|0x40)
#define TIMER2CTRL    (PWM_BLOCK_BASE|0x48)
#define PWM2OFF       (PWM_BLOCK_BASE|0x5C)
#define PWMCR         (PWM_BLOCK_BASE|0x68)

#define GPIODATA    (GPIO_BLOCK_BASE|0x00)
#define GPIODIR     (GPIO_BLOCK_BASE|0x04)
#define GPIOIS      (GPIO_BLOCK_BASE|0x08)
#define GPIOIEV     (GPIO_BLOCK_BASE|0x10)
#define GPIOIC      (GPIO_BLOCK_BASE|0x20)
#define GPIOPC      (GPIO_BLOCK_BASE|0x28)

#define WKREQ       (SYSTEM_BLOCK1_BASE|0x00)
#define CLKENB      (SYSTEM_BLOCK1_BASE|0x04)
#define DRAMPWR     (SYSTEM_BLOCK1_BASE|0x08)
#define INTMASK     (SYSTEM_BLOCK1_BASE|0x0C)
#define CNT_DIS     (SYSTEM_BLOCK1_BASE|0x10)
#define GPIOSEL     (SYSTEM_BLOCK2_BASE|0x00)

#define HVGA_XRES	320
#define HVGA_YRES	480

#define CAPELA_DEFAULT_BACKLIGHT_BRIGHTNESS 255

struct mddi_table {
	uint32_t reg;
	unsigned value[4];
	uint32_t val_len;
};

int bridge_on = 1;
EXPORT_SYMBOL(bridge_on);
int TestBridgeOn = 1;

static struct mddi_table toshiba_mddi_init_table[] = 
{
#ifdef SMD_VYSNC_50HZ_CMD  // change -> 57hz -> 50hz (flicker)
	{ DPSET0,   {0x0A990050},   1   }, 
	{ DPSET1,   {0x00000116},   1   }, 
#else
	{ DPSET0,	{0x4CB60075},	1 	},
	{ DPSET1,	{0x00000112},	1	},
#endif
	{ DPSUS,	{0x00000000},	1 	},
	{ DPRUN,	{0x00000001},	1 	},
	{ 1,		{0x00000000},	500	},
	{ SYSCKENA,	{0x00000001},	1 	},
	{ CLKENB,	{0x000021EF},	1	},
	{ GPIODATA,	{0x03FF0000},	1 	},
	{ GPIODIR,	{0x00000008},	1 	},
	{ GPIOSEL,	{0x00000000},	1 	},
	{ GPIOPC,	{0x00080000},	1 	},
	{ WKREQ,	{0x00000000},	1 	},
	{ GPIODATA,	{0x00080000},	1 	},
	{ 1,		{0x00000000},	10	},
	{ GPIODATA,	{0x00080008},	1 	},
	{ 1,		{0x00000000},	10	},
	{ GPIODATA,	{0x00080000},	1 	},
	{ 1,		{0x00000000},	15	},
	{ GPIODATA,	{0x00080008},	1 	},
	{ DRAMPWR,	{0x00000001},	1 	},
	{ CLKENB,	{0x000021EF},	1 	},
	{ 1,		{0x00000000},	15	},
	{ SSICTL,	{0x00000170},	1 	},
	{ SSITIME,	{0x00000101},	1 	},
	{ CNT_DIS,	{0x00000000},	1 	},
	{ SSICTL,	{0x00000173},	1 	},
	{ BITMAP0,	{0x01E00140},	1 	},
	{ PORT_ENB,	{0x00000001},	1 	},
	{ PORT,		{0x00000008},	1 	},
	{ PXL,		{0x0000003A},	1 	},
	{ MPLFBUF,	{0x00000000},	1 	},
	{ HCYCLE,	{0x000000DF},	1 	},
	{ HSW,		{0x00000000},	1 	},
	{ HDE_START,{0x0000001F},	1 	},
	{ HDE_SIZE,	{0x0000009F},	1 	},
	{ VCYCLE,	{0x000001EF},	1 	},
	{ VSW,		{0x00000001},	1 	},
	{ VDE_START,{0x00000007},	1 	},
	{ VDE_SIZE,	{0x000001DF},	1 	},
	{ CNT_DIS,	{0x00000000},	1 	},
	{ START,	{0x00000001},	1 	},
	{ 1,		{0x00000000},	10	},
};
static struct mddi_table smd_oled_init_table[] = 
{
// Power Setting Sequence - 1 (Analog Setting)
	{ SSITX,	{0x00010100},	1 	},
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00012122},	1	}, 
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00012208}, 	1	},	//   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00012300}, 	1	},	//   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00012433},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00012533},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
#ifdef SMD_AUTO_POWER_CMD
	{ SSITX,	{0x00012606},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
#else
	{ SSITX,	{0x00012602},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
#endif
	{ SSITX,	{0x00012742},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00012F02},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
// Power Setting Sequence - 2 (Power Boosting Setting)
#ifdef SMD_AUTO_POWER_CMD
	{ SSITX,	{0x00012001},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	10	},
#else
	{ SSITX,	{0x00012001},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	10	},
	{ SSITX,	{0x00012011},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	20	},
	{ SSITX,	{0x00012031},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	60	},
	{ SSITX,	{0x00012071},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	60	},  //  wait_ms(1);
	{ SSITX,	{0x00012073},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	20	},  //  wait_ms(1);
	{ SSITX,	{0x00012077},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	10	},  //  wait_ms(1);
#endif
// Power Setting Sequence - 3 (AMP On)
	{ SSITX,	{0x00010401},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	10	},  //  wait_ms(1);
// Initial Sequence - 1 (LTPS Setting)
	{ SSITX,	{0x00010644},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00010704},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00010801},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
#ifdef SMD_AUTO_POWER_CMD
	{ SSITX,	{0x00010926},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
#else
	{ SSITX,	{0x00010906},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
#endif
	{ SSITX,	{0x00010A21},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00010C00},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00010D14},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00010E00},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00010F1E},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00011000},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
// Initial Sequence - 2 (RGB I/F Setting)
	{ SSITX, 	{0x00011C08},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00011D05},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
	{ SSITX,	{0x00011F00},	1	},  //   # SPI.SSITX  
	{ 1,		{0x00000000},	1	},
// Gamma Selection Sequence  default 13 level	 
    { SSITX,	{0x00013032},	1 	},
	{ 1,		{0x00000000},	1	},    
	{ SSITX,	{0x00013138},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013236},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x0001334A},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013462},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013562},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x0001361F},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x0001371B},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x0001381C},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013924},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013A20},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013B21},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013C2C},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013D1E},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013E24},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00013F3A},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x0001401C},	1 	},
	{ 1,		{0x00000000},	1	},	
	{ SSITX,	{0x00014128},	1 	},

#if 0 //to_protect_blink
    { SSITX, 	{0x00010405},	1	},  //   # SPI.SSITX  
    { 2,        {0x00000000},   20  },  //  wait_ms(1);
    { SSITX,	{0x00010407},	1	},  //   # SPI.SSITX  
    { 2,        {0x00000000},   15  },  //  wait_ms(1);
    { SSITX,    {0x00010405},	1	},  //   # SPI.SSITX      
    { 2,        {0x00000000},   25  },  //  wait_ms(1);	
    { SSITX,    {0x00010407},	1	},  //   # SPI.SSITX  
    { 2,        {0x00000000},   15  },  //  wait_ms(1);
    { SSITX,    {0x00010405},	1	},  //   # SPI.SSITX      
    { 2,        {0x00000000},   25  },  //  wait_ms(1);	
    { SSITX,    {0x00010407},	1	},  //   # SPI.SSITX 
#else
    { SSITX, 	{0x00010405},	1	},  //   # SPI.SSITX  
    { 1,        {0x00000000},   20  },  //  wait_ms(1);
    { SSITX,	{0x00010407},	1	},  //   # SPI.SSITX  
#endif
};

static struct mddi_table smd_oled_sleep_table[] = 
{
#if 1  // Sleep Sequence
	{ SSITX,	{0x00010403},	1 	},
	{ 1,		{0x00000000},	100 },
	{ SSITX,	{0x00010401},	1 	},
	{ 1,		{0x00000000},	20 	},
	{ SSITX,	{0x00010400},	1 	},
	{ 1,		{0x00000000},	10 },
#ifdef SMD_AUTO_POWER_CMD	
	{ SSITX,	{0x00010500},	1 	},  // power off
#endif
	{ SSITX,	{0x00010302},	1 	},  // Stand by on
	{ 1,		{0x00000000},	10 },
	{ START,	{0x00000000},	1 	},
	{ 1,        {0x00000000},	60	},
#else  // Same Power off sequence Requested By SMD, Because of lcd flicker
   // Power Off Sequence
	{ SSITX,	{0x00010403},	1 	},
	{ 1,		{0x00000000},	100 },
	{ SSITX,	{0x00010401},	1 	},
	{ 1,		{0x00000000},	60 	},
	{ SSITX,	{0x00010400},	1 	},
	{ 1,		{0x00000000},	100 },
#ifdef SMD_AUTO_POWER_CMD	
	{ SSITX,	{0x00010500},	1 	},
	{ 1,        {0x00000000},	100	},
	{ START,	{0x00000000},	1 	},
#else
	{ START,	{0x00000000},	1 	},
	{ 1,        {0x00000000},	100	},
	{ SSITX,	{0x00012000},	1 	},
#endif
#endif
};	

static struct mddi_table mddi_toshiba_sleep_table[] = 
{
	{ PXL,           {0x00000000},   1   },   // LCDC sleep mode
	{ START,         {0x00000000},   1   },   // LCD.START  Sync I/F OFF
	{ REGENB,        {0x00000001},   1   },   // reflect setup by next VSYNC
	{ 1,		     {0x00000000},	 30  },                
	{ CNT_DIS,       {0x00000003},   1   }  , // SYS.CNT_DIS LCD all signal to Low state
	{ CLKENB,        {0x00000000},   1   }, // SYS.CLKENB all module(system) clock disable
	{ DRAMPWR,       {0x00000000},   1   }, // SYS.DRAMPWR
	{ SYSCKENA,      {0x00000000},   1   }, // MDC.SYSCKENA system clock disable
	{ DPSUS,         {0x00000001},   1   },  // MDC.DPSUS -> Suspend
	{ DPRUN,         {0x00000000},   1   },  // MDC.DPRUN -> Reset
	{ 1,		     {0x00000000},	 14   },                
};

static struct mddi_table mddi_toshiba_sleep_first_table[] = 
{
	{ DPSET0,        {0x4CB60075},   1   }, // MDC.DPSET0 
	{ DPSET1,        {0x00000112},   1   }, // MDC.DPSET1 (96MHz PLLCLK)  2008.10.16 DPLL 120MHz -> 96MHz change asked by H/W
	{ DPSUS,         {0x00000000},   1   }, // MDC.DPSUS 
	{ DPRUN,         {0x00000001},   1   }, // MDC.DPRUN reset DPLL
};

static struct mddi_table mddi_toshiba_sleep_second_table[] = 
{
	{ SYSCKENA,      {0x00000001},   1   }, // MDC.SYSCKENA system clock disable
	{ CLKENB,        {0x000000C0},   1   }, // SYS.CLKENB all module(system) clock disable
	{ CNT_DIS,       {0x00000011},   1   }, // SYS.CNT_DIS LCD all signal to Low state	
	{ DRAMPWR,       {0x00000000},   1   }, // SYS.DRAMPWR
	{ GPIODIR,	     {0x00000008},	 1 	 },
	{ GPIOSEL,	     {0x00000000},	 1 	 },
	{ GPIOPC,	     {0x00080000},	 1 	 },
	{ GPIODATA,	     {0x00080000},	 1 	 },
	{ 1,		     {0x00000000},	 10	 },
	{ GPIODATA,	     {0x00080008},	 1 	 },
	{ 1,		     {0x00000000},	 30	 },
	{ SYSCKENA,      {0x00000000},   1   }, // MDC.SYSCKENA system clock disable
	{ DPSET0,        {0x4CB60075},   1   }, // MDC.DPSET0 
	{ DPSET1,        {0x00000112},   1   }, // MDC.DPSET1 (96MHz PLLCLK)  2008.10.16 DPLL 120MHz -> 96MHz change asked by H/W
	{ DPSUS,         {0x00000001},   1   }, // MDC.DPSUS 
	{ DPRUN,         {0x00000000},   1   }, // MDC.DPRUN reset DPLL
};


static struct mddi_table mddi_toshiba_wakeup_first_table[] = 
{	
#ifdef SMD_VYSNC_50HZ_CMD  // change -> 57hz -> 50hz (flicker)
	{ DPSET0,        {0x0A990050},   1   }, // MDC.DPSET0 
	{ DPSET1,        {0x00000116},   1   }, // MDC.DPSET1 (96MHz PLLCLK)  2008.10.16 DPLL 120MHz -> 96MHz change asked by H/W
#else
	{ DPSET0,        {0x4CB60075},   1   }, // MDC.DPSET0 
	{ DPSET1,        {0x00000112},   1   }, // MDC.DPSET1 (96MHz PLLCLK)  2008.10.16 DPLL 120MHz -> 96MHz change asked by H/W
#endif
	{ DPSUS,         {0x00000000},   1   }, // MDC.DPSUS 
	{ DPRUN,         {0x00000001},   1   }, // MDC.DPRUN reset DPLL
}; 

static struct mddi_table mddi_toshiba_wakeup_second_table[] = 
{
	{ SYSCKENA,      {0x00000001},   1   }, // MDC.SYSCKENA 
	{ CLKENB,        {0x000020EF},   1   }, // SYS.CLKENB 
	{ GPIODIR,	     {0x00000008},	 1 	 },
	{ GPIOPC,	     {0x00080008},	 1 	 },
	{ DRAMPWR,       {0x00000001},   1   }, // SYS.DRAMPWR
	{ CLKENB,        {0x000021EF},   1   }, // SYS.CLKENB 
	{ SSICTL,        {0x00000110},   1   }, // SPI operation mode 
	{ SSITIME,       {0x00000101},   1   }, // SPI serial i/f timing
	{ SSICTL,        {0x00000113},   1   }, // Set SPI active mode
#if 1   // LCDC Reset 
	{ SRST,          {0x00000000},   1   }, // LCDC Reset
	{ 1,		     {0x00000000},	 1	 },
	{ SRST,          {0x00000003},   1   },
#endif
	{ BITMAP0,	     {0x01E00140},	 1	 },
	{ CLKENB,        {0x000021EF},   1   }, // SYS.CLKENB 
	{ PORT_ENB,	     {0x00000001},	 1 	 },
	{ PORT,		     {0x00000008},	 1 	 },
	{ PXL,		     {0x0000003A},	 1 	 },
	{ MPLFBUF,	     {0x00000000},	 1 	 },
	{ HCYCLE,	     {0x000000DF},	 1 	 },
	{ HSW,		     {0x00000000},	 1 	 },
	{ HDE_START,     {0x0000001F},	 1 	 },
	{ HDE_SIZE,	     {0x0000009F}, 	 1 	 },
	{ VCYCLE,	     {0x000001EF},	 1 	 },
	{ VSW,		     {0x00000001},	 1 	 },
	{ VDE_START,     {0x00000007},	 1 	 },  
	{ VDE_SIZE,	     {0x000001DF},	 1 	 },
	{ CNT_DIS,        {0x00000000 }, 1 },	
    { START,	    {0x00000001},	1 	},
	{ 1,		     {0x00000000},	 10	 }
};	

static struct mddi_table smd_oled_wakeup_start_table[] = 
{
    { SSITX,	{0x00010300},   1 	},   // Stand by off
#if 0  // dark display problem By SMD
	{ GPIODATA,	{0x00080000},	 1 	 },
	{ 1,		{0x00000000},	 15	 },
	{ GPIODATA,	{0x00080008},	 1 	 },
	{ 1,		{0x00000000},	 5	 },
	{ SSITX,	{0x00012200},	 1 	 },
	{ SSITX,	{0x00012001},	 1 	 },
	{ 1,		{0x00000000},	 10	 },
	{ SSITX,	{0x00012011},	 1 	 },
	{ 1,		{0x00000000},	 20	 },
	{ SSITX,	{0x00012031},	 1 	 },
	{ 1,		{0x00000000},	 60	 },
	{ SSITX,	{0x00010302},	 1 	 },
	{ 1,		{0x00000000},	 10	 },
#endif
#if 1  // Reset Add Requested By SMD, Because of lcd flicker
	{ GPIODATA,	{0x00080000},	 1 	 },
	{ 1,		{0x00000000},	 15	 },
	{ GPIODATA,	{0x00080008},	 1 	 },
	{ 1,		{0x00000000},	 15	 },
#endif

};

static struct mddi_table smd_oled_wakeup_power_table[] = 
{
    // Power Setting Sequence - 1 (Analog Setting)
	{ SSITX,	{0x00010100},	1 	},
	{ 0,        {0x00000000},	2	},
	{ SSITX,	{0x00012133},	1	}, 
	{ 0,        {0x00000000},	2	},
	{ SSITX,	{0x00012208},   1	},	//   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00012300},   1	},	//   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00012433},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00012533},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2	},  //  wait_ms(1);
#ifdef SMD_AUTO_POWER_CMD
	{ SSITX,    {0x00012606},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},	2	},  //  wait_ms(1);
#else
	{ SSITX,    {0x00012602},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},	2	},  //  wait_ms(1);
#endif
	{ SSITX,	{0x00012742},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},	2	},  //  wait_ms(1);
	{ SSITX,    {0x00012F02},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},	2	},  //  wait_ms(1);
    // Power Setting Sequence - 2 (Power Boosting Setting)
#ifdef SMD_AUTO_POWER_CMD
	{ SSITX,    {0x00010501},  1 },  //   # SPI.SSITX  
	{ 1,        {0x00000000},  200 },  //  more wait_ms(200) ;
#else
	{ SSITX,    {0x00012001},	1	},  //   # SPI.SSITX  
	{ 1,        {0x00000000},   10  },  //  wait_ms(1);
	{ SSITX,    {0x00012011},	1	},  //   # SPI.SSITX  
	{ 1,        {0x00000000},   20  },  //  wait_ms(1);
	{ SSITX,    {0x00012031},	1	},  //   # SPI.SSITX  
	{ 1,        {0x00000000},   60  },  //  wait_ms(1);
	{ SSITX,    {0x00012071},	1	},  //   # SPI.SSITX  
	{ 1,        {0x00000000},   60  },  //  wait_ms(1);
	{ SSITX,    {0x00012073},	1	},  //   # SPI.SSITX  
	{ 1,        {0x00000000},   20  },  //  wait_ms(1);
	{ SSITX,    {0x00012077},	1	},  //   # SPI.SSITX  
	{ 1,        {0x00000000},   10  },  //  wait_ms(1);
#endif
	// Power Setting Sequence - 3 (AMP On)
	{ SSITX,    {0x00010401},	1	},  //   # SPI.SSITX  
	{ 1,        {0x00000000},   10  },  //  wait_ms(1);
};

static struct mddi_table smd_oled_wakeup_init_table[] = 
{
	// Initial Sequence - 1 (LTPS Setting)
	{ SSITX,    {0x00010644},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00010704},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00010801},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
#ifdef SMD_VYSNC_50HZ_CMD  // change -> 57hz -> 50hz (flicker)
#ifdef SMD_AUTO_POWER_CMD
	{ SSITX,    {0x00010902},  1 },    //   # SPI.SSITX  
	{ 1,        {0x00000000},  2 },    //   wait_ms(1) ;
#else
	{ SSITX,    {0x00010906},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
#endif
	{ SSITX,    {0x00010A11},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
#else
#ifdef SMD_AUTO_POWER_CMD
	{ SSITX,    {0x00010926},  1 },    //   # SPI.SSITX  
	{ 1,        {0x00000000},  2 },    //   wait_ms(1) ;
#else
	{ SSITX,    {0x00010906},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
#endif
	{ SSITX,    {0x00010A21},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
#endif
	{ SSITX,    {0x00010C00},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00010D14},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00010E00},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00010F1E},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00011000},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	// Initial Sequence - 2 (RGB I/F Setting)
    { SSITX,    {0x00011C08},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
    { SSITX,    {0x00011D05},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00011F00},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
};

static struct mddi_table smd_oled_wakeup_gamma_table[] = 
{
	// Gamma Selection Sequence			 
	{ SSITX,    {0x00013028},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013130},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013230},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013358},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013470},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013576},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x0001361E},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x0001371A},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013819},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013929},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013A23},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013B24},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013C35},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013D25},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013E27},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00013F3F},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00014028},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
	{ SSITX,    {0x00014130},	1	},  //   # SPI.SSITX  
	{ 0,        {0x00000000},   2   },  //  wait_ms(1);
};

static struct mddi_table smd_oled_wakeup_display_on_table[] = 
{
    // Display On Sequence
#if 0 //to_protect_blink
    { SSITX,    {0x00010405},	1	},  //   # SPI.SSITX  
    { 2,        {0x00000000},   20  },  //  wait_ms(1);
    { SSITX,    {0x00010407},	1	},  //   # SPI.SSITX  
    { 2,        {0x00000000},   15  },  //  wait_ms(1);
    { SSITX,    {0x00010405},	1	},  //   # SPI.SSITX      
    { 2,        {0x00000000},   25  },  //  wait_ms(1);	
    { SSITX,    {0x00010407},	1	},  //   # SPI.SSITX  
    { 2,        {0x00000000},   15  },  //  wait_ms(1);
    { SSITX,    {0x00010405},	1	},  //   # SPI.SSITX      
    { 2,        {0x00000000},   25  },  //  wait_ms(1);	
    { SSITX,    {0x00010407},	1	},  //   # SPI.SSITX  
#else
    { SSITX,    {0x00010405},	1	},  //   # SPI.SSITX  
    { 1,        {0x00000000},   20  },  //  wait_ms(1);
    { SSITX,    {0x00010407},	1	},  //   # SPI.SSITX  
#endif
};

static struct mddi_table smd_oled_shutdown_table[] = 
{
   // Power Off Sequence
	{ SSITX,	{0x00010403},	1 	},
	{ 1,		{0x00000000},	100 },
	{ SSITX,	{0x00010401},	1 	},
	{ 1,		{0x00000000},	20 	},
	{ SSITX,	{0x00010400},	1 	},
	{ 1,		{0x00000000},	10 },
#ifdef SMD_AUTO_POWER_CMD	
	{ SSITX,	{0x00010500},	1 	},
	{ SSITX,	{0x00010302},	1 	},	
	{ 1,        {0x00000000},	10	},
	{ START,	{0x00000000},	1 	},
#else
	{ START,	{0x00000000},	1 	},
	{ 1,        {0x00000000},	100	},
	{ SSITX,	{0x00012000},	1 	},
#endif
};

static struct mddi_table mddi_toshiba_shutdown_table[] = 
{
    // Power Off Sequence
	{ PORT,		     {0x00000003},	 1 	 },
	{ REGENB,        {0x00000001},   1   },   // reflect setup by next VSYNC
	{ 1,		     {0x00000000},	 16  },                
	{ PXL,           {0x00000000},   1   },   // LCDC sleep mode
	{ START,         {0x00000000},   1   },   // LCD.START  Sync I/F OFF
	{ REGENB,        {0x00000001},   1   },   // reflect setup by next VSYNC
};


#ifdef SMD_OLD_GAMMA_CMD
// Dimming
static struct mddi_table smd_oled_gamma_40cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013323},	1 	},
	{ SSITX,	{0x0001342A},	1 	},
	{ SSITX,	{0x00013531},	1 	},
	{ SSITX,	{0x00013626},	1 	},
	{ SSITX,	{0x00013721},	1 	},
	{ SSITX,	{0x00013820},	1 	},
	{ SSITX,	{0x0001392B},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B25},	1 	},
	{ SSITX,	{0x00013C31},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E22},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 1
static struct mddi_table smd_oled_gamma_95cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013336},	1 	},
	{ SSITX,	{0x0001343E},	1 	},
	{ SSITX,	{0x00013548},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371F},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x0001392A},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B24},	1 	},
	{ SSITX,	{0x00013C31},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E22},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 2
static struct mddi_table smd_oled_gamma_100cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013339},	1 	},
	{ SSITX,	{0x00013441},	1 	},
	{ SSITX,	{0x0001354B},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371F},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x0001392A},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B24},	1 	},
	{ SSITX,	{0x00013C31},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E22},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 3
static struct mddi_table smd_oled_gamma_110cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x0001333C},	1 	},
	{ SSITX,	{0x00013444},	1 	},
	{ SSITX,	{0x0001354F},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371F},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x0001392A},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B24},	1 	},
	{ SSITX,	{0x00013C31},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E22},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 4	
static struct mddi_table smd_oled_gamma_120cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x0001333E},	1 	},
	{ SSITX,	{0x00013446},	1 	},
	{ SSITX,	{0x00013551},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371F},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x0001392A},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B24},	1 	},
	{ SSITX,	{0x00013C31},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E22},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 5	
static struct mddi_table smd_oled_gamma_130cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013340},	1 	},
	{ SSITX,	{0x00013448},	1 	},
	{ SSITX,	{0x00013553},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371F},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x0001392A},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B25},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E23},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 6	
static struct mddi_table smd_oled_gamma_140cd_table[] = 
{
	{ SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013342},	1 	},
	{ SSITX,	{0x0001344B},	1 	},
	{ SSITX,	{0x00013556},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371F},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x0001392A},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B25},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E23},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 7
static struct mddi_table smd_oled_gamma_150cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013344},	1 	},
	{ SSITX,	{0x0001344D},	1 	},
	{ SSITX,	{0x00013558},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x00013929},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B24},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E23},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 8
static struct mddi_table smd_oled_gamma_160cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013346},	1 	},
	{ SSITX,	{0x00013450},	1 	},
	{ SSITX,	{0x0001355B},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x00013929},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B24},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E23},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 9
static struct mddi_table smd_oled_gamma_170cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013348},	1 	},
	{ SSITX,	{0x00013452},	1 	},
	{ SSITX,	{0x0001355D},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x00013929},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B23},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1E},	1 	},
	{ SSITX,	{0x00013E23},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 10
static struct mddi_table smd_oled_gamma_180cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x0001334A},	1 	},
	{ SSITX,	{0x00013454},	1 	},
	{ SSITX,	{0x00013560},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x00013929},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B23},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1E},	1 	},
	{ SSITX,	{0x00013E23},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 11
static struct mddi_table smd_oled_gamma_190cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x0001334B},	1 	},
	{ SSITX,	{0x00013456},	1 	},
	{ SSITX,	{0x00013562},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013929},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B24},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1F},	1 	},
	{ SSITX,	{0x00013E23},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 12
static struct mddi_table smd_oled_gamma_200cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x0001334D},	1 	},
	{ SSITX,	{0x00013458},	1 	},
	{ SSITX,	{0x00013564},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013929},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B24},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1F},	1 	},
	{ SSITX,	{0x00013E23},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 13
static struct mddi_table smd_oled_gamma_210cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x0001334F},	1 	},
	{ SSITX,	{0x0001345A},	1 	},
	{ SSITX,	{0x00013566},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371C},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013928},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B23},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1F},	1 	},
	{ SSITX,	{0x00013E24},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 14
static struct mddi_table smd_oled_gamma_220cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013350},	1 	},
	{ SSITX,	{0x0001345C},	1 	},
	{ SSITX,	{0x00013568},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371C},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013928},	1 	},
	{ SSITX,	{0x00013A23},	1 	},
	{ SSITX,	{0x00013B23},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D1F},	1 	},
	{ SSITX,	{0x00013E24},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 15
static struct mddi_table smd_oled_gamma_230cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013351},	1 	},
	{ SSITX,	{0x0001345D},	1 	},
	{ SSITX,	{0x0001356A},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371C},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013928},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B23},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D20},	1 	},
	{ SSITX,	{0x00013E24},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 16
static struct mddi_table smd_oled_gamma_240cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013353},	1 	},
	{ SSITX,	{0x00013460},	1 	},
	{ SSITX,	{0x0001356C},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371C},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013928},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B23},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D20},	1 	},
	{ SSITX,	{0x00013E24},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 17
static struct mddi_table smd_oled_gamma_250cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013355},	1 	},
	{ SSITX,	{0x00013462},	1 	},
	{ SSITX,	{0x0001356F},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371B},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013927},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D21},	1 	},
	{ SSITX,	{0x00013E24},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 18
static struct mddi_table smd_oled_gamma_260cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013356},	1 	},
	{ SSITX,	{0x00013464},	1 	},
	{ SSITX,	{0x00013570},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371B},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013927},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C32},	1 	},
	{ SSITX,	{0x00013D21},	1 	},
	{ SSITX,	{0x00013E24},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 19
static struct mddi_table smd_oled_gamma_270cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013357},	1 	},
	{ SSITX,	{0x00013465},	1 	},
	{ SSITX,	{0x00013571},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371B},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x00013927},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C34},	1 	},
	{ SSITX,	{0x00013D22},	1 	},
	{ SSITX,	{0x00013E25},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 20	
static struct mddi_table smd_oled_gamma_280cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013358},	1 	},
	{ SSITX,	{0x00013467},	1 	},
	{ SSITX,	{0x00013572},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371B},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x00013927},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C34},	1 	},
	{ SSITX,	{0x00013D22},	1 	},
	{ SSITX,	{0x00013E25},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 21	
static struct mddi_table smd_oled_gamma_290cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x00013359},	1 	},
	{ SSITX,	{0x0001346A},	1 	},
	{ SSITX,	{0x00013573},	1 	},
	{ SSITX,	{0x0001361E},	1 	},
	{ SSITX,	{0x0001371A},	1 	},
	{ SSITX,	{0x0001381A},	1 	},
	{ SSITX,	{0x00013927},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C33},	1 	},
	{ SSITX,	{0x00013D21},	1 	},
	{ SSITX,	{0x00013E25},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};

// level 22
static struct mddi_table smd_oled_gamma_300cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x0001313A},	1 	},
	{ SSITX,	{0x0001323A},	1 	},
	{ SSITX,	{0x0001335B},	1 	},
	{ SSITX,	{0x0001346B},	1 	},
	{ SSITX,	{0x00013575},	1 	},
	{ SSITX,	{0x0001361E},	1 	},
	{ SSITX,	{0x0001371A},	1 	},
	{ SSITX,	{0x0001381A},	1 	},
	{ SSITX,	{0x00013927},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C33},	1 	},
	{ SSITX,	{0x00013D21},	1 	},
	{ SSITX,	{0x00013E25},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};	
#else
// Dimming -> 50cd
static struct mddi_table smd_oled_gamma_40cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x00013133},	1 	},
	{ SSITX,	{0x00013233},	1 	},
	{ SSITX,	{0x00013323},	1 	},
	{ SSITX,	{0x00013431},	1 	},
	{ SSITX,	{0x00013531},	1 	},
	{ SSITX,	{0x00013622},	1 	},
	{ SSITX,	{0x00013720},	1 	},
	{ SSITX,	{0x00013820},	1 	},
	{ SSITX,	{0x00013927},	1 	},
	{ SSITX,	{0x00013A24},	1 	},
	{ SSITX,	{0x00013B25},	1 	},
	{ SSITX,	{0x00013C2A},	1 	},
	{ SSITX,	{0x00013D1E},	1 	},
	{ SSITX,	{0x00013E21},	1 	},
	{ SSITX,	{0x00013F38},	1 	},
	{ SSITX,	{0x00014020},	1 	},
	{ SSITX,	{0x00014128},	1 	},
};	

// level 1 -> 110cd
static struct mddi_table smd_oled_gamma_95cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x00013336},	1 	},
	{ SSITX,	{0x00013448},	1 	},
	{ SSITX,	{0x00013548},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381F},	1 	},
	{ SSITX,	{0x00013925},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C29},	1 	},
	{ SSITX,	{0x00013D1A},	1 	},
	{ SSITX,	{0x00013E1F},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};

// level 2 -> 115cd
static struct mddi_table smd_oled_gamma_100cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x00013338},	1 	},
	{ SSITX,	{0x00013449},	1 	},
	{ SSITX,	{0x0001354B},	1 	},
	{ SSITX,	{0x0001361F},	1 	},
	{ SSITX,	{0x0001371E},	1 	},
	{ SSITX,	{0x0001381D},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B21},	1 	},
	{ SSITX,	{0x00013C2A},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E21},	1 	},
	{ SSITX,	{0x00013F39},	1 	},
	{ SSITX,	{0x00014021},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 3 -> 120cd 
static struct mddi_table smd_oled_gamma_110cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x00013338},	1 	},
	{ SSITX,	{0x0001344B},	1 	},
	{ SSITX,	{0x0001354B},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381F},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C29},	1 	},
	{ SSITX,	{0x00013D1A},	1 	},
	{ SSITX,	{0x00013E1F},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};

// level 4	-> 125cd
static struct mddi_table smd_oled_gamma_120cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x0001333A},	1 	},
	{ SSITX,	{0x0001344C},	1 	},
	{ SSITX,	{0x0001354D},	1 	},
	{ SSITX,	{0x0001361F},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381D},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C2A},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E21},	1 	},
	{ SSITX,	{0x00013F39},	1 	},
	{ SSITX,	{0x00014021},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 5	
static struct mddi_table smd_oled_gamma_130cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x0001333A},	1 	},
	{ SSITX,	{0x0001344E},	1 	},
	{ SSITX,	{0x0001354E},	1 	},
	{ SSITX,	{0x00013621},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381E},	1 	},
	{ SSITX,	{0x00013925},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C2A},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E20},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};

// level 6	
static struct mddi_table smd_oled_gamma_140cd_table[] = 
{
	{ SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x0001333D},	1 	},
	{ SSITX,	{0x00013450},	1 	},
	{ SSITX,	{0x00013551},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381D},	1 	},
	{ SSITX,	{0x00013925},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C29},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E20},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};	

// level 7
static struct mddi_table smd_oled_gamma_150cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x0001333F},	1 	},
	{ SSITX,	{0x00013453},	1 	},
	{ SSITX,	{0x00013553},	1 	},
	{ SSITX,	{0x0001361F},	1 	},
	{ SSITX,	{0x0001371D},	1 	},
	{ SSITX,	{0x0001381D},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C2A},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E21},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};	

// level 8
static struct mddi_table smd_oled_gamma_160cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x00013341},	1 	},
	{ SSITX,	{0x00013456},	1 	},
	{ SSITX,	{0x00013556},	1 	},
	{ SSITX,	{0x0001361F},	1 	},
	{ SSITX,	{0x0001371C},	1 	},
	{ SSITX,	{0x0001381D},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B21},	1 	},
	{ SSITX,	{0x00013C2A},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E22},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};	

// level 9
static struct mddi_table smd_oled_gamma_170cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x00013343},	1 	},
	{ SSITX,	{0x00013458},	1 	},
	{ SSITX,	{0x00013559},	1 	},
	{ SSITX,	{0x00013620},	1 	},
	{ SSITX,	{0x0001371C},	1 	},
	{ SSITX,	{0x0001381D},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B21},	1 	},
	{ SSITX,	{0x00013C29},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E21},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};

// level 10
static struct mddi_table smd_oled_gamma_180cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013137},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x00013345},	1 	},
	{ SSITX,	{0x0001345B},	1 	},
	{ SSITX,	{0x0001355B},	1 	},
	{ SSITX,	{0x0001361F},	1 	},
	{ SSITX,	{0x0001371B},	1 	},
	{ SSITX,	{0x0001381D},	1 	},
	{ SSITX,	{0x00013923},	1 	},
	{ SSITX,	{0x00013A22},	1 	},
	{ SSITX,	{0x00013B21},	1 	},
	{ SSITX,	{0x00013C2A},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E22},	1 	},
	{ SSITX,	{0x00013F39},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 11
static struct mddi_table smd_oled_gamma_190cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013138},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x00013347},	1 	},
	{ SSITX,	{0x0001345D},	1 	},
	{ SSITX,	{0x0001355D},	1 	},
	{ SSITX,	{0x0001361E},	1 	},
	{ SSITX,	{0x0001371B},	1 	},
	{ SSITX,	{0x0001381D},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C2B},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E21},	1 	},
	{ SSITX,	{0x00013F39},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 12
static struct mddi_table smd_oled_gamma_200cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013138},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x00013349},	1 	},
	{ SSITX,	{0x0001345F},	1 	},
	{ SSITX,	{0x00013560},	1 	},
	{ SSITX,	{0x0001361F},	1 	},
	{ SSITX,	{0x0001371B},	1 	},
	{ SSITX,	{0x0001381D},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A21},	1 	},
	{ SSITX,	{0x00013B22},	1 	},
	{ SSITX,	{0x00013C2A},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E20},	1 	},
	{ SSITX,	{0x00013F39},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 13
static struct mddi_table smd_oled_gamma_210cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x00013138},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x0001334A},	1 	},
	{ SSITX,	{0x00013462},	1 	},
	{ SSITX,	{0x00013562},	1 	},
	{ SSITX,	{0x0001361F},	1 	},
	{ SSITX,	{0x0001371B},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A20},	1 	},
	{ SSITX,	{0x00013B21},	1 	},
	{ SSITX,	{0x00013C2C},	1 	},
	{ SSITX,	{0x00013D1E},	1 	},
	{ SSITX,	{0x00013E24},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014128},	1 	},
};

// level 14
static struct mddi_table smd_oled_gamma_220cd_table[] = 
{
    { SSITX,	{0x00013032},	1 	},
	{ SSITX,	{0x00013138},	1 	},
	{ SSITX,	{0x00013236},	1 	},
	{ SSITX,	{0x0001334C},	1 	},
	{ SSITX,	{0x00013463},	1 	},
	{ SSITX,	{0x00013564},	1 	},
	{ SSITX,	{0x0001361F},	1 	},
	{ SSITX,	{0x0001371C},	1 	},
	{ SSITX,	{0x0001381C},	1 	},
	{ SSITX,	{0x00013923},	1 	},
	{ SSITX,	{0x00013A20},	1 	},
	{ SSITX,	{0x00013B21},	1 	},
	{ SSITX,	{0x00013C2B},	1 	},
	{ SSITX,	{0x00013D1D},	1 	},
	{ SSITX,	{0x00013E22},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014128},	1 	},
};	

// level 15
static struct mddi_table smd_oled_gamma_230cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013139},	1 	},
	{ SSITX,	{0x00013237},	1 	},
	{ SSITX,	{0x0001334E},	1 	},
	{ SSITX,	{0x00013466},	1 	},
	{ SSITX,	{0x00013567},	1 	},
	{ SSITX,	{0x0001361D},	1 	},
	{ SSITX,	{0x0001371A},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A20},	1 	},
	{ SSITX,	{0x00013B21},	1 	},
	{ SSITX,	{0x00013C2A},	1 	},
	{ SSITX,	{0x00013D1C},	1 	},
	{ SSITX,	{0x00013E20},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401C},	1 	},
	{ SSITX,	{0x00014128},	1 	},
};	

// level 16
static struct mddi_table smd_oled_gamma_240cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013139},	1 	},
	{ SSITX,	{0x00013237},	1 	},
	{ SSITX,	{0x00013350},	1 	},
	{ SSITX,	{0x00013468},	1 	},
	{ SSITX,	{0x00013569},	1 	},
	{ SSITX,	{0x0001361E},	1 	},
	{ SSITX,	{0x0001371A},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013923},	1 	},
	{ SSITX,	{0x00013A20},	1 	},
	{ SSITX,	{0x00013B20},	1 	},
	{ SSITX,	{0x00013C27},	1 	},
	{ SSITX,	{0x00013D1A},	1 	},
	{ SSITX,	{0x00013E1E},	1 	},
	{ SSITX,	{0x00013F39},	1 	},
	{ SSITX,	{0x0001401D},	1 	},
	{ SSITX,	{0x00014128},	1 	},
};	

// level 17
static struct mddi_table smd_oled_gamma_250cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013139},	1 	},
	{ SSITX,	{0x00013237},	1 	},
	{ SSITX,	{0x00013352},	1 	},
	{ SSITX,	{0x0001346A},	1 	},
	{ SSITX,	{0x0001356B},	1 	},
	{ SSITX,	{0x0001361C},	1 	},
	{ SSITX,	{0x0001371A},	1 	},
	{ SSITX,	{0x0001381A},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A1F},	1 	},
	{ SSITX,	{0x00013B21},	1 	},
	{ SSITX,	{0x00013C27},	1 	},
	{ SSITX,	{0x00013D1A},	1 	},
	{ SSITX,	{0x00013E1E},	1 	},
	{ SSITX,	{0x00013F34},	1 	},
	{ SSITX,	{0x0001401E},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 18
static struct mddi_table smd_oled_gamma_260cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013139},	1 	},
	{ SSITX,	{0x00013237},	1 	},
	{ SSITX,	{0x00013353},	1 	},
	{ SSITX,	{0x0001346C},	1 	},
	{ SSITX,	{0x0001356C},	1 	},
	{ SSITX,	{0x0001361D},	1 	},
	{ SSITX,	{0x0001371A},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013923},	1 	},
	{ SSITX,	{0x00013A1F},	1 	},
	{ SSITX,	{0x00013B20},	1 	},
	{ SSITX,	{0x00013C27},	1 	},
	{ SSITX,	{0x00013D1A},	1 	},
	{ SSITX,	{0x00013E1E},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401E},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	

// level 19
static struct mddi_table smd_oled_gamma_270cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013139},	1 	},
	{ SSITX,	{0x00013237},	1 	},
	{ SSITX,	{0x00013354},	1 	},
	{ SSITX,	{0x0001346E},	1 	},
	{ SSITX,	{0x0001356E},	1 	},
	{ SSITX,	{0x0001361D},	1 	},
	{ SSITX,	{0x00013719},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013924},	1 	},
	{ SSITX,	{0x00013A1F},	1 	},
	{ SSITX,	{0x00013B20},	1 	},
	{ SSITX,	{0x00013C27},	1 	},
	{ SSITX,	{0x00013D1A},	1 	},
	{ SSITX,	{0x00013E1E},	1 	},
	{ SSITX,	{0x00013F3A},	1 	},
	{ SSITX,	{0x0001401E},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};

// level 20	
static struct mddi_table smd_oled_gamma_280cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013139},	1 	},
	{ SSITX,	{0x00013237},	1 	},
	{ SSITX,	{0x00013355},	1 	},
	{ SSITX,	{0x00013470},	1 	},
	{ SSITX,	{0x00013570},	1 	},
	{ SSITX,	{0x0001361E},	1 	},
	{ SSITX,	{0x00013719},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013923},	1 	},
	{ SSITX,	{0x00013A1F},	1 	},
	{ SSITX,	{0x00013B1F},	1 	},
	{ SSITX,	{0x00013C28},	1 	},
	{ SSITX,	{0x00013D1B},	1 	},
	{ SSITX,	{0x00013E1F},	1 	},
	{ SSITX,	{0x00013F3B},	1 	},
	{ SSITX,	{0x0001401F},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};	

// level 21	
static struct mddi_table smd_oled_gamma_290cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013139},	1 	},
	{ SSITX,	{0x00013237},	1 	},
	{ SSITX,	{0x00013357},	1 	},
	{ SSITX,	{0x00013471},	1 	},
	{ SSITX,	{0x00013572},	1 	},
	{ SSITX,	{0x0001361E},	1 	},
	{ SSITX,	{0x0001371A},	1 	},
	{ SSITX,	{0x0001381B},	1 	},
	{ SSITX,	{0x00013922},	1 	},
	{ SSITX,	{0x00013A1F},	1 	},
	{ SSITX,	{0x00013B1F},	1 	},
	{ SSITX,	{0x00013C28},	1 	},
	{ SSITX,	{0x00013D1B},	1 	},
	{ SSITX,	{0x00013E20},	1 	},
	{ SSITX,	{0x00013F3B},	1 	},
	{ SSITX,	{0x0001401F},	1 	},
	{ SSITX,	{0x0001412A},	1 	},
};

// level 22
static struct mddi_table smd_oled_gamma_300cd_table[] = 
{
    { SSITX,	{0x00013033},	1 	},
	{ SSITX,	{0x00013139},	1 	},
	{ SSITX,	{0x00013237},	1 	},
	{ SSITX,	{0x00013359},	1 	},
	{ SSITX,	{0x00013474},	1 	},
	{ SSITX,	{0x00013574},	1 	},
	{ SSITX,	{0x0001361C},	1 	},
	{ SSITX,	{0x00013719},	1 	},
	{ SSITX,	{0x0001381A},	1 	},
	{ SSITX,	{0x00013923},	1 	},
	{ SSITX,	{0x00013A1F},	1 	},
	{ SSITX,	{0x00013B20},	1 	},
	{ SSITX,	{0x00013C29},	1 	},
	{ SSITX,	{0x00013D1B},	1 	},
	{ SSITX,	{0x00013E20},	1 	},
	{ SSITX,	{0x00013F39},	1 	},
	{ SSITX,	{0x00014021},	1 	},
	{ SSITX,	{0x00014129},	1 	},
};	
#endif

static uint32 mddi_toshiba_curr_vpos;
static boolean mddi_toshiba_monitor_refresh_value = FALSE;
static boolean mddi_toshiba_report_refresh_measurements = FALSE;

boolean mddi_toshiba_61Hz_refresh = TRUE;

#if 1  // calculate refresh rate  <hg2395.kim>
/* Timing variables for tracking vsync */
/* dot_clock = 13.332MHz
 * horizontal count = 448
 * vertical count = 496
 * refresh rate = 13332000/(448*496) = 60Hz
 */

static uint32 mddi_toshiba_rows_per_second = 29758;  /* 13332000/448 */
static uint32 mddi_toshiba_usecs_per_refresh = 16667; /* (448*496) / 13332000 */
static uint32 mddi_toshiba_rows_per_refresh = 496;   
#else
/* Modifications to timing to increase refresh rate to > 60Hz.
 *   20MHz dot clock.
 *   646 total rows.
 *   506 total columns.
 *   refresh rate = 61.19Hz
 */
static uint32 mddi_toshiba_rows_per_second = 39526;
static uint32 mddi_toshiba_usecs_per_refresh = 16344;
static uint32 mddi_toshiba_rows_per_refresh = 646;

#endif
extern boolean mddi_vsync_detect_enabled;
extern int lightsensor_suspend();
extern int lightsensor_resume();
extern int get_lightsensor_level( void );
static int low_battery_flag=0;

void lcd_low_battery_flag_set_for_lightsensor( int flag );
EXPORT_SYMBOL(lcd_low_battery_flag_set_for_lightsensor);

static msm_fb_vsync_handler_type mddi_toshiba_vsync_handler = NULL;
static void *mddi_toshiba_vsync_handler_arg;
static uint16 mddi_toshiba_vsync_attempts;

static int mddi_smd_hvga_lcd_on(struct platform_device *pdev);
static int mddi_smd_hvga_lcd_off(struct platform_device *pdev);
//static panel_info_type mddi_smd_hvga_lcd_get_disp_info(struct msm_fb_data_type *mfd);
//static int mddi_smd_hvga_lcd_init(struct msm_fb_data_type *mfd);
//static panel_info_type panel_info;
static void capela_set_backlight_level(uint8_t level);

// hsil
static int capela_backlight_off;
static int capela_backlight_brightness = CAPELA_DEFAULT_BACKLIGHT_BRIGHTNESS;
static int capela_backlight_last_level = 4;
static int capela_backlight_is_dimming = 0;
static DEFINE_MUTEX(capela_backlight_lock);
static DEFINE_MUTEX(capela_lcd_on_lock);
static DEFINE_MUTEX(capela_lcd_off_lock);

#if 0
#define read_client_reg(__X,__Y,__Z) {\
	mddi_queue_register_read(__X,__Y,TRUE,0);\
}
#else
static uint32 read_client_reg(uint32 addr)
{
	uint32 val;
	mddi_queue_register_read(addr, &val, TRUE, 0);
	return val;
}
#endif

#define write_client_reg(__X,__Y,__Z) {\
	mddi_queue_register_write(__X,__Y,TRUE,0);\
}
   
#define write_client_reg_multi(__X, __Y, __Z) {\
	mddi_queue_register_multi_write(__X, __Y, __Z, TRUE, 0);\
}
	    
static void process_mddi_table(struct mddi_table *table, size_t count)
{
	int i;
	
	for(i = 0; i < count; i++) 
	{
		uint32_t reg = table[i].reg;
		unsigned *value = table[i].value;
		uint32_t val_len = table[i].val_len;
										 
		if (reg == 0)
			udelay(val_len);
		else if (reg == 1)
			msleep(val_len);
#if 1 //to_protect_blink			
		else if (reg == 2)
			msleep(val_len);
#endif
		else
			write_client_reg_multi( reg,  (unsigned *)value, val_len);
	}
}

static void mddi_toshiba_vsync_set_handler(msm_fb_vsync_handler_type handler,	/* ISR to be executed */
					   void *arg)
{
	boolean error = FALSE;
	unsigned long flags;

	/* Disable interrupts */
	spin_lock_irqsave(&mddi_host_spin_lock, flags);
	// INTLOCK();

	if (mddi_toshiba_vsync_handler != NULL) {
		error = TRUE;
	} else {
		/* Register the handler for this particular GROUP interrupt source */
		mddi_toshiba_vsync_handler = handler;
		mddi_toshiba_vsync_handler_arg = arg;
	}

	/* Restore interrupts */
	spin_unlock_irqrestore(&mddi_host_spin_lock, flags);
	// MDDI_INTFREE();
	if (error) {
//		MDDI_MSG_ERR("MDDI: Previous Vsync handler never called\n");
	} else {
		mddi_toshiba_vsync_attempts = 1;
		mddi_vsync_detect_enabled = TRUE;
	}
}				/* mddi_samsung_vsync_set_handler */

static void mddi_smd_hvga_lcd_vsync_detected(boolean detected)
{
	// static timetick_type start_time = 0;
	static struct timeval start_time;
	static boolean first_time = TRUE;
	// uint32 mdp_cnt_val = 0;
	// timetick_type elapsed_us;
	struct timeval now;
	uint32 elapsed_us;
	uint32 num_vsyncs;

	if ((detected) || (mddi_toshiba_vsync_attempts > 5)) {
		if ((detected) && (mddi_toshiba_monitor_refresh_value)) {
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
					      (mddi_toshiba_usecs_per_refresh >>
					       1)) /
				    mddi_toshiba_usecs_per_refresh;
				/* LCD is configured for * hsyncs (rows) per refresh cycle.
				 * Calculate new rows_per_second value based upon these
				 * new measurements. MDP can update with this new value. */
				mddi_toshiba_rows_per_second =
				    (mddi_toshiba_rows_per_refresh * 1000 *
				     num_vsyncs) / (elapsed_us / 1000);
			}
			// start_time = timetick_get();
			first_time = FALSE;
			jiffies_to_timeval(jiffies, &start_time);
			if (mddi_toshiba_report_refresh_measurements) {
				// mdp_cnt_val = MDP_LINE_COUNT;
			}
		}
		/* if detected = TRUE, client initiated wakeup was detected */
		if (mddi_toshiba_vsync_handler != NULL) {
			(*mddi_toshiba_vsync_handler)
			    (mddi_toshiba_vsync_handler_arg);
			mddi_toshiba_vsync_handler = NULL;
		}
		mddi_vsync_detect_enabled = FALSE;
		mddi_toshiba_vsync_attempts = 0;
		/* need to disable the interrupt wakeup */

		if (!detected) {
			/* give up after 5 failed attempts but show error */
			MDDI_MSG_NOTICE("Vsync detection failed!\n");
		} else if ((mddi_toshiba_monitor_refresh_value) &&
			   (mddi_toshiba_report_refresh_measurements)) {
			MDDI_MSG_NOTICE("  Last Line Counter=%d!\n",
					mddi_toshiba_curr_vpos);
			// MDDI_MSG_NOTICE("  MDP Line Counter=%d!\n",mdp_cnt_val);
			MDDI_MSG_NOTICE("  Lines Per Second=%d!\n",
					mddi_toshiba_rows_per_second);
		}

	} else {
		/* if detected = FALSE, we woke up from hibernation, but did not
		 * detect client initiated wakeup.
		 */
		mddi_toshiba_vsync_attempts++;
	}
}

static void mddi_smd_hvga_init(void)
{
	int i;

	dprintk("\n");
	process_mddi_table(toshiba_mddi_init_table, ARRAY_SIZE(toshiba_mddi_init_table));
	process_mddi_table(smd_oled_init_table, ARRAY_SIZE(smd_oled_init_table));
}

// hsil
static void smd_hvga_oled_sleep(void)
{
	process_mddi_table(smd_oled_sleep_table, ARRAY_SIZE(smd_oled_sleep_table));
}

static void toshiba_bridge_sleep(void)
{
	uint32 DPLL_locked=0;
	uint32 LockTimeDpll=0;

#if 1
	process_mddi_table(mddi_toshiba_sleep_table, ARRAY_SIZE(mddi_toshiba_sleep_table));
#else
	process_mddi_table(mddi_toshiba_sleep_table_first, ARRAY_SIZE(mddi_toshiba_sleep_table_first));

	while ( !DPLL_locked  &&  LockTimeDpll <= 460 )
	{
	    mddi_wait(20);  
	    LockTimeDpll += 20;
	//        DPLL_locked=read_client_reg(DPSUS); // &0x10;
	    DPLL_locked &= 0x10;
	}
	mddi_wait(10);

	process_mddi_table(mddi_toshiba_sleep_table_second, ARRAY_SIZE(mddi_toshiba_sleep_table_second));
#endif
}

static void smd_hvga_oled_start_wakeup(void)
{
    printk("[LCD] smd_hvga_oled_start_wakeup\n");
    process_mddi_table(smd_oled_wakeup_start_table, ARRAY_SIZE(smd_oled_wakeup_start_table));
}

static void smd_hvga_oled_power_wakeup(void)
{
	process_mddi_table(smd_oled_wakeup_power_table, ARRAY_SIZE(smd_oled_wakeup_power_table));
}

static void smd_hvga_oled_init_wakeup(void)
{
	process_mddi_table(smd_oled_wakeup_init_table, ARRAY_SIZE(smd_oled_wakeup_init_table));
}

#if 0
static void smd_hvga_oled_gamma_wakeup(void)
{
	process_mddi_table(smd_oled_wakeup_gamma_table, ARRAY_SIZE(smd_oled_wakeup_gamma_table));
}
#else
static void smd_hvga_oled_gamma_wakeup(int level)
{
//	printk("[KHG] smd_hvga_oled_gamma_wakeup,level=%d\n",level);

        switch(level)
		{
			case 0:
				process_mddi_table(smd_oled_gamma_40cd_table, ARRAY_SIZE(smd_oled_gamma_40cd_table));				
				break;
			case 1:
				process_mddi_table(smd_oled_gamma_95cd_table, ARRAY_SIZE(smd_oled_gamma_95cd_table));				

				break;
			case 2:
				process_mddi_table(smd_oled_gamma_100cd_table, ARRAY_SIZE(smd_oled_gamma_100cd_table));				

				break;
			case 3:
				process_mddi_table(smd_oled_gamma_110cd_table, ARRAY_SIZE(smd_oled_gamma_110cd_table));				

				break;
			case 4:
				process_mddi_table(smd_oled_gamma_120cd_table, ARRAY_SIZE(smd_oled_gamma_120cd_table));				

				break;
			case 5:
				process_mddi_table(smd_oled_gamma_130cd_table, ARRAY_SIZE(smd_oled_gamma_130cd_table));				

				break;
			case 6:
				process_mddi_table(smd_oled_gamma_140cd_table, ARRAY_SIZE(smd_oled_gamma_140cd_table));				

				break;
			case 7:
				process_mddi_table(smd_oled_gamma_150cd_table, ARRAY_SIZE(smd_oled_gamma_150cd_table));				

				break;
			case 8:
				process_mddi_table(smd_oled_gamma_160cd_table, ARRAY_SIZE(smd_oled_gamma_160cd_table));				

				break;
			case 9:
				process_mddi_table(smd_oled_gamma_170cd_table, ARRAY_SIZE(smd_oled_gamma_170cd_table));				

				break;
			case 10:
				process_mddi_table(smd_oled_gamma_180cd_table, ARRAY_SIZE(smd_oled_gamma_180cd_table));				
				break;
			case 11:
				process_mddi_table(smd_oled_gamma_190cd_table, ARRAY_SIZE(smd_oled_gamma_190cd_table));				
				break;
			case 12:
				process_mddi_table(smd_oled_gamma_200cd_table, ARRAY_SIZE(smd_oled_gamma_200cd_table));				
				break;
			case 13:
				process_mddi_table(smd_oled_gamma_210cd_table, ARRAY_SIZE(smd_oled_gamma_210cd_table));				
				break;
			case 14:
				process_mddi_table(smd_oled_gamma_220cd_table, ARRAY_SIZE(smd_oled_gamma_220cd_table));				
				break;
			case 15:
				process_mddi_table(smd_oled_gamma_230cd_table, ARRAY_SIZE(smd_oled_gamma_230cd_table));				
				break;
			case 16:
				process_mddi_table(smd_oled_gamma_240cd_table, ARRAY_SIZE(smd_oled_gamma_240cd_table));				
				break;
			case 17:
				process_mddi_table(smd_oled_gamma_250cd_table, ARRAY_SIZE(smd_oled_gamma_250cd_table));				
				break;
			case 18:
				process_mddi_table(smd_oled_gamma_260cd_table, ARRAY_SIZE(smd_oled_gamma_260cd_table));				
				break;
			case 19:
				process_mddi_table(smd_oled_gamma_270cd_table, ARRAY_SIZE(smd_oled_gamma_270cd_table));				
				break;
			case 20:
				process_mddi_table(smd_oled_gamma_280cd_table, ARRAY_SIZE(smd_oled_gamma_280cd_table));				
				break;
			case 21:
				process_mddi_table(smd_oled_gamma_290cd_table, ARRAY_SIZE(smd_oled_gamma_290cd_table));				
				break;
			case 22:
				process_mddi_table(smd_oled_gamma_300cd_table, ARRAY_SIZE(smd_oled_gamma_300cd_table));				
				break;

			default:
				break;
		}
//		capela_backlight_last_level = level;

}
#endif

static void smd_hvga_oled_display_on_wakeup(void)
{
    printk("[LCD] smd_hvga_oled_display_on_wakeup\n");
	process_mddi_table(smd_oled_wakeup_display_on_table, ARRAY_SIZE(smd_oled_wakeup_display_on_table));
}

static void toshiba_bridge_read_check_regs(void)
{
   uint32 sys_reg_dpset0,sys_reg_dpset1,sys_reg_dpsus,sys_reg_dprun;

   printk("Read Toshiba IC Registers......\n");

   mddi_wait(100);
   sys_reg_dpset0 = read_client_reg(DPSET0);
   sys_reg_dpset1 = read_client_reg(DPSET1);
   sys_reg_dpsus = read_client_reg(DPSUS);
   sys_reg_dprun = read_client_reg(DPRUN);
   printk("DPSET0= 0x%x,DPSET1= 0x%x,DPSUS= 0x%x,DPRUN= 0x%x\n",
   sys_reg_dpset0,sys_reg_dpset1,sys_reg_dpsus,sys_reg_dprun); 
   
   if((sys_reg_dpset0 != 0x0BA80062) || (sys_reg_dpset1 != 0x00000114) || 
   	   sys_reg_dpsus !=  0x00000000  || (sys_reg_dprun  != 0x00000001))
      printk("Once more Set Toshiba IC Registers!!!!\n");
      process_mddi_table(mddi_toshiba_wakeup_first_table, ARRAY_SIZE(mddi_toshiba_wakeup_first_table));   	   
}

static void toshiba_bridge_read_regs(void)
{
   uint32 sys_reg_dpset0,sys_reg_dpset1,sys_reg_dpsus,sys_reg_dprun;
   uint32 sys_reg_sysckena,sys_reg_clkenb,sys_reg_gpiopc,sys_reg_gpiodir;
   uint32 sys_reg_dpramwr,sys_reg_ssictl,sys_reg_ssitime,sys_reg_srst;
   uint32 sys_reg_bitmap0,sys_reg_port_enb,sys_reg_port,sys_reg_pxl;
   uint32 sys_reg_malbuf,sys_reg_hcycle,sys_reg_hsw,sys_reg_hdestart;
   uint32 sys_reg_hdesize,sys_reg_vcycle,sys_reg_vsw,sys_reg_vdestart;
   uint32 sys_reg_vdesize,sys_reg_cnt_dis;

   printk("Read Toshiba IC Registers......\n");
   
   sys_reg_dpset0 = read_client_reg(DPSET0);
   sys_reg_dpset1 = read_client_reg(DPSET1);
   sys_reg_dpsus = read_client_reg(DPSUS);
   sys_reg_dprun = read_client_reg(DPRUN);
   printk("DPSET0= 0x%x,DPSET1= 0x%x,DPSUS= 0x%x,DPRUN= 0x%x\n",
   sys_reg_dpset0,sys_reg_dpset1,sys_reg_dpsus,sys_reg_dprun); 
   
   sys_reg_sysckena = read_client_reg(SYSCKENA);
   sys_reg_clkenb = read_client_reg(CLKENB);
   sys_reg_gpiopc = read_client_reg(GPIODIR);
   sys_reg_gpiodir = read_client_reg(GPIOPC);
   printk("SYSCKENA= 0x%x,CLKENB= 0x%x,GPIODIR= 0x%x,GPIOPC= 0x%x\n",
   sys_reg_sysckena,sys_reg_clkenb,sys_reg_gpiopc,sys_reg_gpiodir); 

   sys_reg_dpramwr = read_client_reg(DRAMPWR);
   sys_reg_ssictl = read_client_reg(SSICTL);
   sys_reg_ssitime = read_client_reg(SSITIME);
   sys_reg_srst = read_client_reg(SRST);
   printk("DRAMPWR= 0x%x,SSICTL= 0x%x,SSITIME= 0x%x,SRST= 0x%x\n",
   sys_reg_dpramwr,sys_reg_ssictl,sys_reg_ssitime,sys_reg_srst);       

   sys_reg_bitmap0 = read_client_reg(BITMAP0);
   sys_reg_port_enb = read_client_reg(PORT_ENB);
   sys_reg_port = read_client_reg(PORT);
   sys_reg_pxl = read_client_reg(PXL);
   printk("BITMAP0= 0x%x,PORT_ENB= 0x%x,PORT= 0x%x,PXL= 0x%x\n",
   sys_reg_bitmap0,sys_reg_port_enb,sys_reg_port,sys_reg_pxl);       

   sys_reg_malbuf = read_client_reg(MPLFBUF);
   sys_reg_hcycle = read_client_reg(HCYCLE);
   sys_reg_hsw = read_client_reg(HSW);
   sys_reg_hdestart = read_client_reg(HDE_START);
   printk("MPLFBUF= 0x%x,HCYCLE= 0x%x,HSW= 0x%x,HDE_START= 0x%x\n",
   sys_reg_malbuf,sys_reg_hcycle,sys_reg_hsw,sys_reg_hdestart);       
   
   sys_reg_hdestart = read_client_reg(HDE_SIZE);
   sys_reg_vcycle = read_client_reg(VCYCLE);
   sys_reg_vsw = read_client_reg(VSW);
   sys_reg_vdestart = read_client_reg(VDE_START);
   printk("HDE_SIZE= 0x%x,VCYCLE= 0x%x,VSW= 0x%x,VDE_START= 0x%x\n",
   sys_reg_hdesize,sys_reg_vcycle,sys_reg_vsw,sys_reg_vdestart);       

   sys_reg_vdesize = read_client_reg(VDE_SIZE);
   sys_reg_cnt_dis = read_client_reg(CNT_DIS);
   printk("VDE_SIZE= 0x%x,CNT_DIS= 0x%x\n",sys_reg_vdesize,sys_reg_cnt_dis);       
}   

static void toshiba_bridge_wakeup(void)
{
	uint32 count;   // loop counter
	uint32 DPLL_locked=0;
	uint32 LockTimeDpll=0;


    printk("[LCD] toshiba_bridge_wakeup\n");
	process_mddi_table(mddi_toshiba_wakeup_first_table, ARRAY_SIZE(mddi_toshiba_wakeup_first_table));
//    toshiba_bridge_read_check_regs();
//	printk("[KHG] toshiba_bridge_wakeup_first-2\n");
//	 printk("[KHG] toshiba_bridge_wakeup_DPLL_locked_Before\n");
#if 0
	while ( !DPLL_locked  &&  LockTimeDpll <= 500 )   
	{
		mddi_wait(10);                     
		LockTimeDpll += 10;
		DPLL_locked=read_client_reg(DPSUS);
		DPLL_locked &= 0x10;
	}
//	        printk("[KHG] toshiba_bridge_wakeup_DPLL_locked_After\n");
	mddi_wait(10);
#else
	mddi_wait(200);    
#endif
//    printk("toshiba_bridge_wakeup : DPLL_locked=%d LockTimeDpll=%d\n", DPLL_locked, LockTimeDpll); 

//	printk("[KHG] toshiba_bridge_wakeup_second-1\n");
	process_mddi_table(mddi_toshiba_wakeup_second_table, ARRAY_SIZE(mddi_toshiba_wakeup_second_table));
//    toshiba_bridge_read_regs();
//	printk("[KHG] toshiba_bridge_wakeup_second-2\n");
}
#if 0
static panel_info_type mddi_smd_hvga_lcd_get_disp_info(struct msm_fb_data_type
						      *mfd)
{
	panel_info.xres = HVGA_XRES;
	panel_info.yres = HVGA_YRES;
	panel_info.type = MDDI_PANEL;
	panel_info.pdest = DISPLAY_1;
	panel_info.wait_cycle = 0;
	panel_info.bpp = 18;

	/* vsync config */
	panel_info.lcd.vsync_enable = TRUE;

	panel_info.lcd.refx100 =
	    (mddi_samsung_rows_per_second * 100) /
	    mddi_samsung_rows_per_refresh;
	panel_info.lcd.v_back_porch = 6;
	panel_info.lcd.v_front_porch = 0;
	panel_info.lcd.v_pulse_width = 0;
	panel_info.lcd.hw_vsync_mode = TRUE;
	panel_info.lcd.vsync_notifier_period = (1 * HZ);

	// backlight level
	panel_info.bl_max = 100;
	panel_info.bl_min = 10;

	return panel_info;
}
#endif

static void smd_hvga_oled_shutdown(void)
{
	process_mddi_table(smd_oled_shutdown_table, ARRAY_SIZE(smd_oled_shutdown_table));
}

static void toshiba_bridge_shutdown(void)
{
	process_mddi_table(mddi_toshiba_shutdown_table, ARRAY_SIZE(mddi_toshiba_shutdown_table));
}

static int panel_detect ;

static void mddi_toshiba_panel_detect(void)
{
	panel_detect = 1;
}

//hsil
static struct platform_device capela_backlight = 
{
    .name       = "capela-backlight",
};

#if 0
static int mddi_smd_hvga_lcd_init(struct msm_fb_data_type *mfd)
{
    dprintk("\n");

	mddi_host_type host_idx = MDDI_HOST_PRIM;
	// Already done by appsboot...
	// mddi_host_reg_out(DRIVE_LO, 0x0032);
	// mddi_host_reg_out(DRIVE_LO, 0x0050);

	mddi_samsung_panel_detect();

	if (mfd->panel.id == TOSHIBA_HVGA_SMD) {
// hsil : init sequence is already done in appsboot
//		mddi_smd_hvga_init();
//		printk("[KHG] %s : register platform device\n", __func__);
//		platform_device_register(&repus_backlight);

		mfd->mddi_vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		mddi_lcd.vsync_detected = mddi_samsung_wqvga_lcd_vsync_detected;
	} else {
		mfd->mddi_vdopkt = 0x400;
	}

	return 0;
}
#endif


static int mddi_smd_hvga_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	int tmp=0;

#if 0	
	tmp = get_lightsensor_level( );
printk("[CHJ] get_level test tmp=%d\n", tmp );
	if ( tmp > 0 )
		capela_backlight_last_level = tmp;
#endif
//	printk("[KHG] mddi_smd_hvga_lcd_on_1,bridge_on=%d\n",bridge_on);
	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
	{
  	    printk("[LCD] mddi_smd_hvga_lcd_on,mfd is NULL\n");
		return -ENODEV;
	}
	if (mfd->key != MFD_KEY)
	{
  	    printk("[LCD] mddi_smd_hvga_lcd_on,mfd is Invalide Argument\n");
		return -EINVAL;
	}
	mutex_lock(&capela_lcd_on_lock);

    if(!bridge_on){
		// Bridge Wake Up 
		toshiba_bridge_wakeup();
		// OLED Wake Up 
        smd_hvga_oled_start_wakeup();
        smd_hvga_oled_power_wakeup();
		smd_hvga_oled_init_wakeup();		
		if ( low_battery_flag )
			smd_hvga_oled_gamma_wakeup(0);
		else
			smd_hvga_oled_gamma_wakeup(capela_backlight_last_level);					
        smd_hvga_oled_display_on_wakeup();
		printk("[LCD] mddi_smd_hvga_lcd_on,bridge_on=%d\n",bridge_on);
		bridge_on = 1;
    }
#if 1  // remove display noise
	// refresh temp screen
		mfd->panel_power_on = 1;
		mfd->ibuf.dma_x=0;
		mfd->ibuf.dma_y=0;
		mfd->ibuf.dma_w=320;
		mfd->ibuf.dma_h=480;
		mdp_dma2_update(mfd);
#endif
	
	mutex_unlock(&capela_lcd_on_lock);

//=== to lightsensor - hojung.choi
	//lightsensor_resume();
	capela_backlight_is_dimming =0;

	return 0;
}

static int mddi_smd_hvga_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	printk("[LCD] mddi_smd_hvga_lcd_off\n");

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

//=== to lightsensor - hojung.choi
	//lightsensor_suspend();

	if (!mfd)
	{
  	    printk("[LCD] mddi_smd_hvga_lcd_off,mfd is NULL\n");
		return -ENODEV;
	}
	if (mfd->key != MFD_KEY)
	{
        printk("[LCD] mddi_smd_hvga_lcd_off,mfd is Invalide Argument\n");
		return -EINVAL;
	}
	mutex_lock(&capela_lcd_off_lock);
	bridge_on = 0;
	smd_hvga_oled_sleep();
	toshiba_bridge_sleep();
	mutex_unlock(&capela_lcd_off_lock);

	return 0;
}

static void mddi_smd_hvga_lcd_shutdown(struct platform_device *pdev)

{
	struct msm_fb_data_type *mfd;

	printk("[LCD] mddi_smd_hvga_lcd_shutdown\n");

    smd_hvga_oled_shutdown();
	toshiba_bridge_shutdown();
}

// hsil for oled on/off test
void test_lcd_on(void)
{
	if(!TestBridgeOn){
#if 1
		toshiba_bridge_wakeup();
		smd_hvga_oled_start_wakeup();
		smd_hvga_oled_power_wakeup();
		smd_hvga_oled_init_wakeup();
		smd_hvga_oled_gamma_wakeup(capela_backlight_last_level);
		smd_hvga_oled_display_on_wakeup();
#else
        mddi_smd_hvga_init();
#endif
		TestBridgeOn = 1; 
	}    
}
EXPORT_SYMBOL(test_lcd_on);

void test_lcd_off(void)
{
	TestBridgeOn = 0; 
#if 1	
	smd_hvga_oled_sleep();
//	toshiba_bridge_sleep();
#else
    smd_hvga_oled_shutdown();
	toshiba_bridge_shutdown();
#endif
}
EXPORT_SYMBOL(test_lcd_off);


static int __init toshiba_smd_probe(struct platform_device *pdev)
{
    msm_fb_add_device(pdev);
	return 0;
}
		  
static struct platform_driver this_driver = {
	.probe  = toshiba_smd_probe,
	.driver = {
		.name   = "mddi_toshiba_smd_wvga",
	},
};
							   
static struct msm_fb_panel_data toshiba_smd_panel_data = {
	.on = mddi_smd_hvga_lcd_on,
	.off = mddi_smd_hvga_lcd_off,
	.shutdown = mddi_smd_hvga_lcd_shutdown,
};
									    
static struct platform_device this_device = {
	.name   = "mddi_toshiba_smd_wvga",
	.id = 0,
	.dev    = {
		.platform_data = &toshiba_smd_panel_data,
	}
};

void lcd_low_battery_flag_set_for_lightsensor( int flag )
{
	low_battery_flag = flag;
	printk("[%s] %d\n", __func__, low_battery_flag );

	return;
}

void capela_set_lightsensor_level(uint8_t level)
{
	unsigned long flags;
//	printk("[%s] level=%d\n", __func__,level);		
	
// To test
//			printk("[%s] capela_backlight_is_dimming=%d\n", __func__, capela_backlight_is_dimming);		

			
	  if(!bridge_on){
	  	 printk("lcd_driver bridge_on off! last_level saved by lightsensor\n");
	 	 capela_backlight_last_level = level ;
	  	 return;
	  }
				                            
		local_irq_save(flags);
        printk("capela_lightsensor_set..lightsensor_level=%d\n",level);		
		if(capela_backlight_is_dimming) 
		    goto lightsensor_backlight_is_dimming;



		switch(level)
	{
		case 0:  // Dimming Not Saved
			process_mddi_table(smd_oled_gamma_40cd_table, ARRAY_SIZE(smd_oled_gamma_40cd_table));	
			capela_backlight_is_dimming = 1;
			break;
		case 1:
			process_mddi_table(smd_oled_gamma_95cd_table, ARRAY_SIZE(smd_oled_gamma_95cd_table));				
    	    break;
		case 2:
			process_mddi_table(smd_oled_gamma_100cd_table, ARRAY_SIZE(smd_oled_gamma_100cd_table));				
			break;
		case 3:
			process_mddi_table(smd_oled_gamma_110cd_table, ARRAY_SIZE(smd_oled_gamma_110cd_table));				
 	        break;
		case 4:
			process_mddi_table(smd_oled_gamma_120cd_table, ARRAY_SIZE(smd_oled_gamma_120cd_table));							
	        break;
		case 5:
			process_mddi_table(smd_oled_gamma_130cd_table, ARRAY_SIZE(smd_oled_gamma_130cd_table));				
	        break;
		case 6:
			process_mddi_table(smd_oled_gamma_140cd_table, ARRAY_SIZE(smd_oled_gamma_140cd_table));				
	        break;
		case 7:
			process_mddi_table(smd_oled_gamma_150cd_table, ARRAY_SIZE(smd_oled_gamma_150cd_table));				
	        break;
		case 8:
			process_mddi_table(smd_oled_gamma_160cd_table, ARRAY_SIZE(smd_oled_gamma_160cd_table));				
	        break;
		case 9:
			process_mddi_table(smd_oled_gamma_170cd_table, ARRAY_SIZE(smd_oled_gamma_170cd_table));				
	        break;
		case 10:
			process_mddi_table(smd_oled_gamma_180cd_table, ARRAY_SIZE(smd_oled_gamma_180cd_table));				
	        break;
		case 11:
			process_mddi_table(smd_oled_gamma_190cd_table, ARRAY_SIZE(smd_oled_gamma_190cd_table));				
	        break;
		case 12:
			process_mddi_table(smd_oled_gamma_200cd_table, ARRAY_SIZE(smd_oled_gamma_200cd_table));				
	        break;
		case 13:
			process_mddi_table(smd_oled_gamma_210cd_table, ARRAY_SIZE(smd_oled_gamma_210cd_table));				
	        break;								
		case 14:
			process_mddi_table(smd_oled_gamma_220cd_table, ARRAY_SIZE(smd_oled_gamma_220cd_table));				
	        break;		
		case 15:
			process_mddi_table(smd_oled_gamma_230cd_table, ARRAY_SIZE(smd_oled_gamma_230cd_table));				
	        break;	
		case 16:
			process_mddi_table(smd_oled_gamma_240cd_table, ARRAY_SIZE(smd_oled_gamma_240cd_table));				
	        break;	
		case 17:
			process_mddi_table(smd_oled_gamma_250cd_table, ARRAY_SIZE(smd_oled_gamma_250cd_table));				
	        break;	
		case 18:
			process_mddi_table(smd_oled_gamma_260cd_table, ARRAY_SIZE(smd_oled_gamma_260cd_table));				
	        break;	
		case 19:
			process_mddi_table(smd_oled_gamma_270cd_table, ARRAY_SIZE(smd_oled_gamma_270cd_table));				
	        break;															
		case 20:
			process_mddi_table(smd_oled_gamma_280cd_table, ARRAY_SIZE(smd_oled_gamma_280cd_table));				
	        break;															
		case 21:
			process_mddi_table(smd_oled_gamma_290cd_table, ARRAY_SIZE(smd_oled_gamma_290cd_table));				
	        break;															
		case 22:
			process_mddi_table(smd_oled_gamma_300cd_table, ARRAY_SIZE(smd_oled_gamma_300cd_table));				
	        break;															
		default:
			break;
	}
lightsensor_backlight_is_dimming :
	capela_backlight_last_level = level ;
		local_irq_restore(flags);
}



static void capela_set_backlight_level(uint8_t level)
{
	unsigned int brightness_level;

	capela_backlight_is_dimming =0;
	 
    if(level==20)
		brightness_level = 0;
	else if(level==255)
		brightness_level = 22;
	else
		brightness_level =(level-30)/11 + 1; 

//    printk("[KHG] capela_brightness_set..level= %d, brightness_level=%d\n",level,brightness_level);		
	switch(brightness_level)
	{
		case 0:  // Dimming Not Saved
			process_mddi_table(smd_oled_gamma_40cd_table, ARRAY_SIZE(smd_oled_gamma_40cd_table));	
			capela_backlight_is_dimming = 1;
			break;
		case 1:
			process_mddi_table(smd_oled_gamma_95cd_table, ARRAY_SIZE(smd_oled_gamma_95cd_table));				
    	    capela_backlight_last_level = brightness_level;
			break;
		case 2:
			process_mddi_table(smd_oled_gamma_100cd_table, ARRAY_SIZE(smd_oled_gamma_100cd_table));				
			capela_backlight_last_level = brightness_level;
			break;
		case 3:
			process_mddi_table(smd_oled_gamma_110cd_table, ARRAY_SIZE(smd_oled_gamma_110cd_table));				
 	        capela_backlight_last_level = brightness_level;
			break;
		case 4:
			process_mddi_table(smd_oled_gamma_120cd_table, ARRAY_SIZE(smd_oled_gamma_120cd_table));							
	        capela_backlight_last_level = brightness_level;
			break;
		case 5:
			process_mddi_table(smd_oled_gamma_130cd_table, ARRAY_SIZE(smd_oled_gamma_130cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;
		case 6:
			process_mddi_table(smd_oled_gamma_140cd_table, ARRAY_SIZE(smd_oled_gamma_140cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;
		case 7:
			process_mddi_table(smd_oled_gamma_150cd_table, ARRAY_SIZE(smd_oled_gamma_150cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;
		case 8:
			process_mddi_table(smd_oled_gamma_160cd_table, ARRAY_SIZE(smd_oled_gamma_160cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;
		case 9:
			process_mddi_table(smd_oled_gamma_170cd_table, ARRAY_SIZE(smd_oled_gamma_170cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;
		case 10:
			process_mddi_table(smd_oled_gamma_180cd_table, ARRAY_SIZE(smd_oled_gamma_180cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;
		case 11:
			process_mddi_table(smd_oled_gamma_190cd_table, ARRAY_SIZE(smd_oled_gamma_190cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;
		case 12:
			process_mddi_table(smd_oled_gamma_200cd_table, ARRAY_SIZE(smd_oled_gamma_200cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;
		case 13:
			process_mddi_table(smd_oled_gamma_210cd_table, ARRAY_SIZE(smd_oled_gamma_210cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;								
		case 14:
			process_mddi_table(smd_oled_gamma_220cd_table, ARRAY_SIZE(smd_oled_gamma_220cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;		
		case 15:
			process_mddi_table(smd_oled_gamma_230cd_table, ARRAY_SIZE(smd_oled_gamma_230cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;	
		case 16:
			process_mddi_table(smd_oled_gamma_240cd_table, ARRAY_SIZE(smd_oled_gamma_240cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;	
		case 17:
			process_mddi_table(smd_oled_gamma_250cd_table, ARRAY_SIZE(smd_oled_gamma_250cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;	
		case 18:
			process_mddi_table(smd_oled_gamma_260cd_table, ARRAY_SIZE(smd_oled_gamma_260cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;	
		case 19:
			process_mddi_table(smd_oled_gamma_270cd_table, ARRAY_SIZE(smd_oled_gamma_270cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;															
		case 20:
			process_mddi_table(smd_oled_gamma_280cd_table, ARRAY_SIZE(smd_oled_gamma_280cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;															
		case 21:
			process_mddi_table(smd_oled_gamma_290cd_table, ARRAY_SIZE(smd_oled_gamma_290cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;															
		case 22:
			process_mddi_table(smd_oled_gamma_300cd_table, ARRAY_SIZE(smd_oled_gamma_300cd_table));				
	        capela_backlight_last_level = brightness_level;
			break;															
		default:
			break;
	}
// To test
//			printk("[%s] capela_backlight_is_dimming=%d\n", __func__, capela_backlight_is_dimming);		

}

static void capela_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
//	printk("[KHG] capela_brightness_set,bridge_on=%d\n",bridge_on);
	if(bridge_on)
	{
//		printk("[KHG] capela_brightness_set,val=%d\n",value);
		mutex_lock(&capela_backlight_lock);
		capela_backlight_brightness = value;
		if(capela_backlight_brightness)
			capela_set_backlight_level(capela_backlight_brightness);
		mutex_unlock(&capela_backlight_lock);
	}
}

static struct led_classdev capela_backlight_led = {
    .name           = "lcd-backlight",
	.brightness = CAPELA_DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set = capela_brightness_set,
};

static int __init capela_backlight_probe(struct platform_device *pdev)
{
    return led_classdev_register(&pdev->dev, &capela_backlight_led);
}
	 
static int capela_backlight_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&capela_backlight_led);
	return 0;
}

static struct platform_driver capela_backlight_driver =
{
    .probe      = capela_backlight_probe,
    .remove     = capela_backlight_remove,
    .shutdown   = mddi_smd_hvga_lcd_shutdown,
	.driver     =
	{
		.name       = "capela-backlight",
		.owner      = THIS_MODULE,
	},
};

static int __init mddi_toshiba_lcd_smd_init(void)
{
    int ret;
	struct msm_panel_info *pinfo;
		 
#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	u32 id;
			  
	id = mddi_get_client_id();
	if ((id >> 16) != 0xD263)
	return 0;
#endif

	ret = platform_driver_register(&this_driver);
	if (!ret) {
	pinfo = &toshiba_smd_panel_data.panel_info;
	pinfo->xres = 320;
	pinfo->yres = 480;
	pinfo->type = MDDI_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->lcd.vsync_enable = TRUE;		// hsil SD for frame rate
//	pinfo->lcd.vsync_enable = FALSE;
	pinfo->lcd.refx100 = (mddi_toshiba_rows_per_second * 100) /	mddi_toshiba_rows_per_refresh;
#if 0  // calculate refresh rate  <hg2395.kim>
	pinfo->lcd.v_back_porch = 8;
	pinfo->lcd.v_front_porch = 8;
#else
	pinfo->lcd.v_back_porch = 6;
	pinfo->lcd.v_front_porch = 0;
#endif
	pinfo->lcd.v_pulse_width = 0;
	pinfo->lcd.hw_vsync_mode = TRUE;
	pinfo->lcd.vsync_notifier_period = (1 * HZ);

	pinfo->bl_max = 100;
	pinfo->bl_min = 10;
	pinfo->clk_rate = 122880000;

	//3D performance (SR # 123450)
    pinfo->clk_min =  120000000;
    pinfo->clk_max =  200000000;

	pinfo->fb_num = 2;
	
	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);
	}
													 
	if (!ret)
		mddi_lcd.vsync_detected = mddi_smd_hvga_lcd_vsync_detected;

	// mddi_smd_hvga_init();	// aleady done in appsbootloader
	// for controlling brightness
	platform_device_register(&capela_backlight);
	platform_driver_register(&capela_backlight_driver);

	return ret;
}

module_init(mddi_toshiba_lcd_smd_init);

EXPORT( capela_set_lightsensor_level );

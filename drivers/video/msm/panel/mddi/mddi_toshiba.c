/* drivers/video/msm/src/panel/mddi/toshiba.c
 *
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
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

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

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

#define PRIM_WIDTH          480
#define PRIM_HEIGHT         640

#define SEC_WIDTH           176
#define SEC_HEIGHT          220

typedef enum {
	TOSHIBA_STATE_OFF,
	TOSHIBA_STATE_PRIM_SEC_STANDBY,
	TOSHIBA_STATE_PRIM_SEC_READY,
	TOSHIBA_STATE_PRIM_NORMAL_MODE,
	TOSHIBA_STATE_SEC_NORMAL_MODE
} mddi_toshiba_state_t;

static uint32 mddi_toshiba_curr_vpos;
static boolean mddi_toshiba_monitor_refresh_value = FALSE;
static boolean mddi_toshiba_report_refresh_measurements = FALSE;

boolean mddi_toshiba_61Hz_refresh = TRUE;

/* Modifications to timing to increase refresh rate to > 60Hz.
 *   20MHz dot clock.
 *   646 total rows.
 *   506 total columns.
 *   refresh rate = 61.19Hz
 */
static uint32 mddi_toshiba_rows_per_second = 39526;
static uint32 mddi_toshiba_usecs_per_refresh = 16344;
static uint32 mddi_toshiba_rows_per_refresh = 646;
extern boolean mddi_vsync_detect_enabled;

static msm_fb_vsync_handler_type mddi_toshiba_vsync_handler = NULL;
static void *mddi_toshiba_vsync_handler_arg;
static uint16 mddi_toshiba_vsync_attempts;

static mddi_toshiba_state_t toshiba_state = TOSHIBA_STATE_OFF;

static int mddi_toshiba_lcd_on(struct platform_device *pdev);
static int mddi_toshiba_lcd_off(struct platform_device *pdev);
static panel_info_type mddi_toshiba_lcd_get_disp_info(struct msm_fb_data_type
						      *mfd);
static int mddi_toshiba_lcd_init(struct msm_fb_data_type *mfd);
static panel_info_type panel_info;

static void mddi_toshiba_state_transition(mddi_toshiba_state_t a,
					  mddi_toshiba_state_t b)
{
	if (toshiba_state != a) {
		MDDI_MSG_ERR("toshiba state trans. (%d->%d) found %d\n", a, b,
			     toshiba_state);
	}
	toshiba_state = b;
}

#define write_client_reg(__X,__Y,__Z) {\
  mddi_queue_register_write(__X,__Y,TRUE,0);\
}

static void toshiba_common_initial_setup(void)
{
	write_client_reg(DPSET0, 0x4BEC0066, TRUE);	// Setup DPLL parameters
	write_client_reg(DPSET1, 0x00000113, TRUE);	// MDC.DPSET1
	write_client_reg(DPSUS, 0x00000000, TRUE);	// Set DPLL oscillation enable
	write_client_reg(DPRUN, 0x00000001, TRUE);	// Release reset sig for DPLL
	mddi_wait(14);
	write_client_reg(SYSCKENA, 0x00000001, TRUE);	// Enable system clock output
	write_client_reg(CLKENB, 0x000000EF, TRUE);	// Enable clks w/o DCLK,i2cCLK
	write_client_reg(GPIODATA, 0x03FF0000, TRUE);	// RESET_LCD_N=0, eDRAM_Pwr=0
	write_client_reg(GPIODIR, 0x0000024D, TRUE);	// dir of GPIO (0,2,3,6,9 out)
	write_client_reg(GPIOSEL, 0x00000173, TRUE);	// GPIO multiplexing control
	write_client_reg(GPIOPC, 0x03C300C0, TRUE);	// GPIO2,3 PD cut
	write_client_reg(WKREQ, 0x00000000, TRUE);	// Wake-up request VSYNC align
	write_client_reg(GPIOIS, 0x00000000, TRUE);	// Set interrupt sense of GPIO
	write_client_reg(GPIOIEV, 0x00000001, TRUE);	// Set interrupt event of GPIO
	write_client_reg(GPIOIC, 0x000003FF, TRUE);	// GPIO interrupt clear
	write_client_reg(GPIODATA, 0x00060006, TRUE);	// Release LCDD reset
	write_client_reg(GPIODATA, 0x00080008, TRUE);	// eDRAM VD supply
	write_client_reg(GPIODATA, 0x02000200, TRUE);	// GPI.GPIODATA TEST LED ON
	write_client_reg(DRAMPWR, 0x00000001, TRUE);	// eDRAM power up
	write_client_reg(TIMER0CTRL, 0x00000060, TRUE);	// PWM0 output stop
	write_client_reg(TIMER0LOAD, 0x00001388, TRUE);	// PWM0 10kHz Duty 99 BL OFF
	write_client_reg(TIMER1CTRL, 0x00000060, TRUE);	// PWM1 output stop
	write_client_reg(TIMER1LOAD, 0x00001388, TRUE);	// PWM1 10kHz Duty 99 BL OFF
	write_client_reg(PWM1OFF, 0x00001387, TRUE);
	write_client_reg(TIMER0CTRL, 0x000000E0, TRUE);	// PWM0 output start
	write_client_reg(TIMER1CTRL, 0x000000E0, TRUE);	// PWM1 output start
	write_client_reg(PWMCR, 0x00000003, TRUE);	// PWM output enable
	mddi_wait(1);
	write_client_reg(SSICTL, 0x00000799, TRUE);	// SPI operation mode
	write_client_reg(SSITIME, 0x00000100, TRUE);	// SPI serial i/f timing
	write_client_reg(SSICTL, 0x0000079b, TRUE);	// Set SPI active mode

	write_client_reg(SSITX, 0x00000000, TRUE);	// Release from Deep Standby
	mddi_wait(1);
	write_client_reg(SSITX, 0x00000000, TRUE);	// SPI.SSITX
	mddi_wait(1);
	write_client_reg(SSITX, 0x00000000, TRUE);	// SPI.SSITX
	mddi_wait(1);
	write_client_reg(SSITX, 0x000800BA, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000111, TRUE);	// Display mode setup(1)
	write_client_reg(SSITX, 0x00080036, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000100, TRUE);	// Memory access control
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800BB, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000100, TRUE);	// Display mode setup(2)
	write_client_reg(SSITX, 0x0008003A, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000160, TRUE);	// RGB Interface data format
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800BF, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000100, TRUE);	// Drivnig method
	write_client_reg(SSITX, 0x000800B1, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x0000015D, TRUE);	// Booster operation setup
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800B2, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000133, TRUE);	// Booster mode setup
	write_client_reg(SSITX, 0x000800B3, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000122, TRUE);	// Booster frequencies setup
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800B4, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000102, TRUE);	// OP-amp clock freq div
	write_client_reg(SSITX, 0x000800B5, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x0000011F, TRUE);	// VCS adjust 1C->1F for Rev 2
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800B6, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000128, TRUE);	// VCOM Voltage adjustment
	write_client_reg(SSITX, 0x000800B7, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000103, TRUE);	// Config extern disp signal
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800B9, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000120, TRUE);	// DCCK/DCEV timing setup
	write_client_reg(SSITX, 0x000800BD, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000102, TRUE);	// ASW signal control
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800BE, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000100, TRUE);	// Dummy display (white/black)
	write_client_reg(SSITX, 0x000800C0, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000111, TRUE);	// FR count setup (A)
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800C1, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000111, TRUE);	// FR count setup (B)
	write_client_reg(SSITX, 0x000800C2, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000111, TRUE);	// FR count setup (C)
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800C3, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x0008010A, TRUE);	// line clock count setup (D)
	write_client_reg(SSITX, 0x0000010A, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800C4, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080160, TRUE);	// line clock count setup (E)
	write_client_reg(SSITX, 0x00000160, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800C5, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080160, TRUE);	// line clock count setup (F)
	write_client_reg(SSITX, 0x00000160, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800C6, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080160, TRUE);	// line clock setup (G)
	write_client_reg(SSITX, 0x00000160, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800C7, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080133, TRUE);	// Gamma 1 fine tuning (1)
	write_client_reg(SSITX, 0x00000143, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800C8, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000144, TRUE);	// Gamma 1 fine tuning (2)
	write_client_reg(SSITX, 0x000800C9, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000133, TRUE);	// Gamma 1 inclination adjust
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800CA, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000100, TRUE);	// Gamma 1 blue offset adjust
	mddi_wait(2);		// Wait SPI fifo empty

	// [PCLK Sync. Table1 for VGA]
	write_client_reg(SSITX, 0x000800EC, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080102, TRUE);	// horz cycles (1)
	write_client_reg(SSITX, 0x00000118, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800CF, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000101, TRUE);	// Blanking period (1)
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D0, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080110, TRUE);	// Blanking period (2)
	write_client_reg(SSITX, 0x00000104, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D1, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000101, TRUE);	// CKV timing on/off
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D2, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080100, TRUE);	// CKV1,2 timing
	write_client_reg(SSITX, 0x0000013A, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D3, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080100, TRUE);	// OEV timing
	write_client_reg(SSITX, 0x0000013A, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D4, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080124, TRUE);	// ASW timing control (1)
	write_client_reg(SSITX, 0x0000016E, TRUE);	//
	mddi_wait(1);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D5, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000124, TRUE);	// ASW timing (2)
	mddi_wait(2);		//  Wait SPI fifo empty

	// [PCLK Sync. Table2 for QVGA ]
	write_client_reg(SSITX, 0x000800ED, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080101, TRUE);	// num horz clock cycles (2)
	write_client_reg(SSITX, 0x0000010A, TRUE);	//
	mddi_wait(2);		//  Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D6, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000101, TRUE);	// Blanking period (1)
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D7, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080110, TRUE);	// Blanking period (2)
	write_client_reg(SSITX, 0x0000010A, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D8, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000101, TRUE);	// CKV timing on/off
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800D9, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080100, TRUE);	// CKV1,2 timing
	write_client_reg(SSITX, 0x00000114, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800DE, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080100, TRUE);	// OEV timing
	write_client_reg(SSITX, 0x00000114, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800DF, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080112, TRUE);	// ASW timing (1)
	write_client_reg(SSITX, 0x0000013F, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800E0, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x0000010B, TRUE);	// ASW timing (2)
	write_client_reg(SSITX, 0x000800E2, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000101, TRUE);	// osc freq div ratio : 2
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800E3, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000136, TRUE);	// osc clk cnt setup
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800E4, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080100, TRUE);	// CKV timing using osc
	write_client_reg(SSITX, 0x00000103, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800E5, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080102, TRUE);	// OEV timing using osc
	write_client_reg(SSITX, 0x00000104, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800E6, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000103, TRUE);	// DCEV timing using osc
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800E7, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00080104, TRUE);	// ASW timing using osc(1)
	write_client_reg(SSITX, 0x0000010A, TRUE);	//
	mddi_wait(2);		// Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800E8, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000104, TRUE);	// ASW timing using osc(2)

	write_client_reg(CLKENB, 0x000001EF, TRUE);	// DCLK enable
	write_client_reg(START, 0x00000000, TRUE);	// LCDC mddi_wait( mode
	write_client_reg(WRSTB, 0x0000003F, TRUE);	// write_client_reg( strobe
	write_client_reg(RDSTB, 0x00000432, TRUE);	// Read strobe
	write_client_reg(PORT_ENB, 0x00000002, TRUE);	// Asynchronous port enable
	write_client_reg(VSYNIF, 0x00000000, TRUE);	// VSYNC I/F mode set
	write_client_reg(ASY_DATA, 0x80000000, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000001, TRUE);	// Oscillator start
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// Direct cmd transfer disable
	mddi_wait(10);		//
	write_client_reg(ASY_DATA, 0x80000000, TRUE);	//
	write_client_reg(ASY_DATB, 0x80000000, TRUE);	//
	write_client_reg(ASY_DATC, 0x80000000, TRUE);	//
	write_client_reg(ASY_DATD, 0x80000000, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000009, TRUE);	// LCD.ASY_CMDSET
	write_client_reg(ASY_CMDSET, 0x00000008, TRUE);	// LCD.ASY_CMDSET
	write_client_reg(ASY_DATA, 0x80000007, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00004005, TRUE);	// LCD driver control
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// Direct cmd transfer disable
	mddi_wait(20);		//
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000000, TRUE);	// LTPS I/F control
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// Direct cmd transfer disable

	write_client_reg(VSYNIF, 0x00000001, TRUE);	// VSYNC I/F mode OFF
	write_client_reg(PORT_ENB, 0x00000001, TRUE);	// SYNC I/F  output select

	mddi_toshiba_state_transition(TOSHIBA_STATE_PRIM_SEC_STANDBY,
				      TOSHIBA_STATE_PRIM_SEC_READY);
}

static void toshiba_prim_start(void)
{
	write_client_reg(VSYNIF, 0x00000001, TRUE);	// VSYNC I/F mode OFF
	write_client_reg(PORT_ENB, 0x00000001, TRUE);	// SYNC I/F mode ON

	write_client_reg(BITMAP1, 0x01E000F0, TRUE);	// PITCH size to Frame buffer1
	write_client_reg(BITMAP2, 0x01E000F0, TRUE);	// PITCH size to Frame buffer2
	write_client_reg(BITMAP3, 0x01E000F0, TRUE);	// PITCH size to Frame buffer3
	write_client_reg(BITMAP4, 0x00DC00B0, TRUE);	// PITCH size to Frame buffer4
	write_client_reg(CLKENB, 0x000001EF, TRUE);	// DCLK supply
	write_client_reg(PORT_ENB, 0x00000001, TRUE);	// Synchronous port enable
	write_client_reg(PORT, 0x00000004, TRUE);	// Polarity of DE high active
	write_client_reg(PXL, 0x00000002, TRUE);	// ACTMODE 2 (1st frame black)
	write_client_reg(MPLFBUF, 0x00000000, TRUE);	// Select the reading buffer

	if (mddi_toshiba_61Hz_refresh) {
		write_client_reg(HCYCLE, 0x000000FC, TRUE);	// Setup to VGA size
		mddi_toshiba_rows_per_second = 39526;
		mddi_toshiba_rows_per_refresh = 646;
		mddi_toshiba_usecs_per_refresh = 16344;
	} else {
		write_client_reg(HCYCLE, 0x0000010b, TRUE);	// Setup to VGA size
		mddi_toshiba_rows_per_second = 37313;
		mddi_toshiba_rows_per_refresh = 646;
		mddi_toshiba_usecs_per_refresh = 17313;
	}

	write_client_reg(HSW, 0x00000003, TRUE);	// LCD.HSW
	write_client_reg(HDE_START, 0x00000007, TRUE);	// LCD.HDE_START
	write_client_reg(HDE_SIZE, 0x000000EF, TRUE);	// LCD.HDE_SIZE
	write_client_reg(VCYCLE, 0x00000285, TRUE);	// LCD.VCYCLE
	write_client_reg(VSW, 0x00000001, TRUE);	// LCD.VSW
	write_client_reg(VDE_START, 0x00000003, TRUE);	// LCD.VDE_START
	write_client_reg(VDE_SIZE, 0x0000027F, TRUE);	// LCD.VDE_SIZE

	write_client_reg(START, 0x00000001, TRUE);	// Pixel data transfer start

	mddi_wait(10);
	write_client_reg(SSITX, 0x000800BC, TRUE);	// Command set SPI block
	write_client_reg(SSITX, 0x00000180, TRUE);	// Display data setup
	write_client_reg(SSITX, 0x0008003B, TRUE);	// Command set SPI block
	write_client_reg(SSITX, 0x00000100, TRUE);	// Quad Data config - VGA
	mddi_wait(1);		//  Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800B0, TRUE);	// Command set SPI block
	write_client_reg(SSITX, 0x00000116, TRUE);	// Power supply ON/OFF control
	mddi_wait(1);		//  Wait SPI fifo empty
	write_client_reg(SSITX, 0x000800B8, TRUE);	// Command setSPI block
	write_client_reg(SSITX, 0x000801FF, TRUE);	// Output control
	write_client_reg(SSITX, 0x000001F5, TRUE);
	mddi_wait(1);		//  Wait SPI fifo empty
	write_client_reg(SSITX, 0x00000011, TRUE);	// mddi_wait (Command only)
	write_client_reg(SSITX, 0x00000029, TRUE);	// Display on (Command only)

	// write_client_reg(WKREQ,0x00000002,TRUE);//  wakeREQ -> GPIO
	write_client_reg(WKREQ, 0x00000000, TRUE);	// wakeREQ -> VSYNC
	write_client_reg(WAKEUP, 0x00000000, TRUE);	// WAKEUP on line 0
	// write_client_reg(INTMASK    ,0x00000001,TRUE); // SYS.INTMASK vwakeout
	write_client_reg(INTMSK, 0x00000001, TRUE);	// LCDC.INTMSK mask vsync

	mddi_toshiba_state_transition(TOSHIBA_STATE_PRIM_SEC_READY,
				      TOSHIBA_STATE_PRIM_NORMAL_MODE);
}

static void toshiba_sec_start(void)
{
	write_client_reg(VSYNIF, 0x00000000, TRUE);	// VSYNC I/F mode ON
	write_client_reg(PORT_ENB, 0x00000002, TRUE);	// ASYNC I/F mode ON

	write_client_reg(CLKENB, 0x000011EF, TRUE);

	// Pixel Data is written in eDRAM by Video Stream Packets

	write_client_reg(BITMAP0, 0x028001E0, TRUE);	// PITCH size Frame buffer0
	write_client_reg(BITMAP1, 0x00000000, TRUE);	// PITCH size Frame buffer1
	write_client_reg(BITMAP2, 0x00000000, TRUE);	// PITCH size Frame buffer2
	write_client_reg(BITMAP3, 0x00000000, TRUE);	// PITCH size Frame buffer3
	write_client_reg(BITMAP4, 0x00DC00B0, TRUE);	// PITCH size Frame buffer4
	write_client_reg(PORT, 0x00000000, TRUE);	// LCD.PORT  Polarity set
	write_client_reg(PXL, 0x00000000, TRUE);	// LCD.PXL  ACTMODE set
	write_client_reg(MPLFBUF, 0x00000004, TRUE);	// LCD.MPLFBUF reading buffer
	write_client_reg(HCYCLE, 0x0000006B, TRUE);	// LCD.HCYCLE QCIF+ size
	write_client_reg(HSW, 0x00000003, TRUE);	// LCD.HSW
	write_client_reg(HDE_START, 0x00000007, TRUE);	// LCD.HDE_START
	write_client_reg(HDE_SIZE, 0x00000057, TRUE);	// LCD.HDE_SIZE
	write_client_reg(VCYCLE, 0x000000E6, TRUE);	// LCD.VCYCLE
	write_client_reg(VSW, 0x00000001, TRUE);	// LCD.VSW
	write_client_reg(VDE_START, 0x00000003, TRUE);	// LCD.VDE_START
	write_client_reg(VDE_SIZE, 0x000000DB, TRUE);	// LCD.VDE_SIZE

	write_client_reg(ASY_DATA, 0x80000001, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x0000011B, TRUE);	// LINE number set
	write_client_reg(ASY_DATC, 0x80000002, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00000700, TRUE);	// AC drive and 1H inversion
	write_client_reg(ASY_DATE, 0x80000003, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATF, 0x00000230, TRUE);	// (high speed control)
	write_client_reg(ASY_DATG, 0x80000008, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATH, 0x00000402, TRUE);	// FP,BP set
	write_client_reg(ASY_CMDSET, 0x00000001, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// Direct cmd transfer disable
	write_client_reg(ASY_DATA, 0x80000009, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000000, TRUE);	// PARTIAL set
	write_client_reg(ASY_DATC, 0x8000000B, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00000000, TRUE);	// fFRAME set
	write_client_reg(ASY_DATE, 0x8000000C, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATF, 0x00000000, TRUE);	// External I/F set
	write_client_reg(ASY_DATG, 0x8000000D, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATH, 0x00000409, TRUE);	// Driver output control (1)
	write_client_reg(ASY_CMDSET, 0x00000001, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// Direct cmd transfer disable
	write_client_reg(ASY_DATA, 0x8000000E, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000409, TRUE);	// Driver output control (2)
	write_client_reg(ASY_DATC, 0x80000030, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00000000, TRUE);	// GAMMA set (1)
	write_client_reg(ASY_DATE, 0x80000031, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATF, 0x00000100, TRUE);	// GAMMA set (2)
	write_client_reg(ASY_DATG, 0x80000032, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATH, 0x00000104, TRUE);	// GAMMA set (3)
	write_client_reg(ASY_CMDSET, 0x00000001, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// Direct cmd transfer disable
	write_client_reg(ASY_DATA, 0x80000033, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000400, TRUE);	// GAMMA set (4)
	write_client_reg(ASY_DATC, 0x80000034, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00000306, TRUE);	// GAMMA set (5)
	write_client_reg(ASY_DATE, 0x80000035, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATF, 0x00000706, TRUE);	// GAMMA set (6)
	write_client_reg(ASY_DATG, 0x80000036, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATH, 0x00000707, TRUE);	// GAMMA set (7)
	write_client_reg(ASY_CMDSET, 0x00000001, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// Direct cmd transfer disable
	write_client_reg(ASY_DATA, 0x80000037, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000004, TRUE);	// GAMMA set (8)
	write_client_reg(ASY_DATC, 0x80000038, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00000000, TRUE);	// GAMMA set (9)
	write_client_reg(ASY_DATE, 0x80000039, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATF, 0x00000000, TRUE);	// GAMMA set (10)
	write_client_reg(ASY_DATG, 0x8000003A, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATH, 0x00000001, TRUE);	// GAMMA set (11)
	write_client_reg(ASY_CMDSET, 0x00000001, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// Direct cmd transfer disable
	write_client_reg(ASY_DATA, 0x80000044, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x0000AF00, TRUE);	// Horizontal RAM AREA set
	write_client_reg(ASY_DATC, 0x80000045, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x0000DB00, TRUE);	// Vertical RAM AREA set
	write_client_reg(ASY_DATE, 0x08000042, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATF, 0x0000DB00, TRUE);	// 1st screen position set
	write_client_reg(ASY_DATG, 0x80000021, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATH, 0x00000000, TRUE);	// REM address set
	write_client_reg(ASY_CMDSET, 0x00000001, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// Direct cmd transfer disable
	write_client_reg(PXL, 0x0000000C, TRUE);	// ACTMODE one shot set
	write_client_reg(VSYNIF, 0x00000001, TRUE);	// VSYNC I/F mode set
	write_client_reg(ASY_DATA, 0x80000022, TRUE);	// RAM write_client_reg( index
	write_client_reg(ASY_CMDSET, 0x00000003, TRUE);	// Direct cmd transfer enable
	write_client_reg(START, 0x00000001, TRUE);	// VSYNC I/F mode START
	// PIXEL data write_client_reg( to RAM
	mddi_wait(60);
	// 0x1100C4               Check LCD.LCDACT = 0
	write_client_reg(PXL, 0x00000000, TRUE);
	write_client_reg(VSYNIF, 0x00000000, TRUE);	// Direct accsess mode set
	write_client_reg(START, 0x00000000, TRUE);	// VSYNC I/F mode off
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// Direct cmd transfer disable
	write_client_reg(ASY_DATA, 0x80000050, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000000, TRUE);	// STV position set
	write_client_reg(ASY_DATC, 0x80000051, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00000E00, TRUE);	// CKV pos and H pulse width
	write_client_reg(ASY_DATE, 0x80000052, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATF, 0x00000D01, TRUE);	// OEV pos and H pulse width
	write_client_reg(ASY_DATG, 0x80000053, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATH, 0x00000000, TRUE);	// FR position set
	write_client_reg(ASY_CMDSET, 0x00000001, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// Direct cmd transfer disable
	write_client_reg(ASY_DATA, 0x80000058, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000000, TRUE);	// DCCK clock number set
	write_client_reg(ASY_DATC, 0x8000005A, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00000E01, TRUE);	// DCEV pos and H pulse width
	write_client_reg(ASY_CMDSET, 0x00000009, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000008, TRUE);	// Direct cmd transfer disable

	// Display ON sequence

	write_client_reg(ASY_DATA, 0x80000011, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000812, TRUE);	// POWER control
	write_client_reg(ASY_DATC, 0x80000012, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00000003, TRUE);	// VDH control
	write_client_reg(ASY_DATE, 0x80000013, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATF, 0x00000909, TRUE);	// VCS, VCOM control
	write_client_reg(ASY_DATG, 0x80000010, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATH, 0x00000040, TRUE);	// POWER control
	write_client_reg(ASY_CMDSET, 0x00000001, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// Direct cmd transfer disable
	mddi_wait(40);
	write_client_reg(ASY_DATA, 0x80000010, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000340, TRUE);	// POWER control
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// Direct cmd transfer disable
	mddi_wait(60);
	write_client_reg(ASY_DATA, 0x80000010, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00003340, TRUE);	// POWER control
	write_client_reg(ASY_DATC, 0x80000007, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00004007, TRUE);	// LCD drive control
	write_client_reg(ASY_CMDSET, 0x00000009, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000008, TRUE);	// Direct cmd transfer disable
	mddi_wait(1);
	write_client_reg(ASY_DATA, 0x80000007, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00004017, TRUE);	// LCD drive control
	write_client_reg(ASY_DATC, 0x8000005B, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATD, 0x00000000, TRUE);	// AUTO mode set
	write_client_reg(ASY_DATE, 0x80000059, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATF, 0x00000011, TRUE);	// LTPS I/F control
	write_client_reg(ASY_CMDSET, 0x0000000D, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x0000000C, TRUE);	// Direct cmd transfer disable
	mddi_wait(20);
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000019, TRUE);	// LTPS I/F control
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// Direct cmd transfer disable
	mddi_wait(20);
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x00000079, TRUE);	// LTPS I/F control
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// Direct cmd transfer disable
	mddi_wait(20);
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// Index setting of SUB LCDD
	write_client_reg(ASY_DATB, 0x000003FD, TRUE);	// LTPS I/F control
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// Direct cmd transfer enable
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// Direct cmd transfer disable
	mddi_wait(20);

	mddi_toshiba_state_transition(TOSHIBA_STATE_PRIM_SEC_READY,
				      TOSHIBA_STATE_SEC_NORMAL_MODE);
}

static void toshiba_prim_lcd_off(void)
{
	// Main panel power off (Deep standby in)
	write_client_reg(SSITX, 0x000800BC, TRUE);	// SPI.SSITX command SPI block
	write_client_reg(SSITX, 0x00000100, TRUE);	// RGB I/F Disable
	write_client_reg(SSITX, 0x00000028, TRUE);	// Display off (Command Only)
	mddi_wait(1);		//           # Wait SPI fifo enpty
	write_client_reg(SSITX, 0x000800B8, TRUE);	// Command setting SPI block
	write_client_reg(SSITX, 0x00000180, TRUE);	// Output control
	write_client_reg(SSITX, 0x00000102, TRUE);	// Output control
	write_client_reg(SSITX, 0x00000010, TRUE);	// Sleep-in (Command Only)

	write_client_reg(PORT, 0x00000003, TRUE);	// LCD.PORT  DE output OFF
	write_client_reg(REGENB, 0x00000001, TRUE);	// reflect setup by next VSYNC
	mddi_wait(1);		//6
	write_client_reg(PXL, 0x00000000, TRUE);	// LCDC sleep mode
	write_client_reg(START, 0x00000000, TRUE);	// LCD.START  Sync I/F OFF
	write_client_reg(REGENB, 0x00000001, TRUE);	// reflect setup by next VSYNC
	mddi_wait(3);		//2
	write_client_reg(SSITX, 0x000800B0, TRUE);	// SPI.SSITX Command SPI block
	write_client_reg(SSITX, 0x00000100, TRUE);	// Deep standby in
	mddi_toshiba_state_transition(TOSHIBA_STATE_PRIM_NORMAL_MODE,
				      TOSHIBA_STATE_PRIM_SEC_STANDBY);
}

static void toshiba_sec_lcd_off(void)
{
	// Sub panel power off (Deep stanby in)

	write_client_reg(VSYNIF, 0x00000000, TRUE);	// VSYNC I/F mode ON
	write_client_reg(PORT_ENB, 0x00000002, TRUE);	// ASYNC I/F set

	write_client_reg(ASY_DATA, 0x80000007, TRUE);	// LCD.ASY_DATx  LCD drive
	write_client_reg(ASY_DATB, 0x00004016, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// LCD.ASY_CMDSET
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// LCD.ASY_CMDSET
	mddi_wait(2);		//0
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// LCD.ASY_DATx  LTPS I/F
	write_client_reg(ASY_DATB, 0x00000019, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// LCD.ASY_CMDSET
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// LCD.ASY_CMDSET
	mddi_wait(2);		//0
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// LCD.ASY_DATx  LTPS I/F
	write_client_reg(ASY_DATB, 0x0000000B, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// LCD.ASY_CMDSET
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// LCD.ASY_CMDSET
	mddi_wait(2);		//0
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// LCD.ASY_DATx  LTPS I/F
	write_client_reg(ASY_DATB, 0x00000002, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// LCD.ASY_CMDSET
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// LCD.ASY_CMDSET
	mddi_wait(4);		//0
	write_client_reg(ASY_DATA, 0x80000010, TRUE);	// LCD.ASY_DATx  POWER control
	write_client_reg(ASY_DATB, 0x00000300, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// LCD.ASY_CMDSET
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// LCD.ASY_CMDSET
	mddi_wait(4);		//0
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// LCD.ASY_DATx  LTPS I/F
	write_client_reg(ASY_DATB, 0x00000000, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// LCD.ASY_CMDSET
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// LCD.ASY_CMDSET
	mddi_wait(2);		//0
	write_client_reg(ASY_DATA, 0x80000007, TRUE);	// LCD.ASY_DATx  LCD drive
	write_client_reg(ASY_DATB, 0x00004004, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	// LCD.ASY_CMDSET
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	// LCD.ASY_CMDSET
	mddi_wait(2);		//0
	write_client_reg(PORT, 0x00000000, TRUE);	// LCD.PORT  DE output off
	write_client_reg(PXL, 0x00000000, TRUE);	// LCD.PXL   LCDC sleep mode
	write_client_reg(START, 0x00000000, TRUE);	// LCD.START LCDC START OFF

	write_client_reg(VSYNIF, 0x00000001, TRUE);	// VSYNC I/F mode OFF
	write_client_reg(PORT_ENB, 0x00000001, TRUE);	// SYNC I/F mode ON

	write_client_reg(REGENB, 0x00000001, TRUE);	// LCD.REGENBL REGENBL VSYNC

	mddi_toshiba_state_transition(TOSHIBA_STATE_SEC_NORMAL_MODE,
				      TOSHIBA_STATE_PRIM_SEC_STANDBY);
}

static void toshiba_sec_cont_update_start(void)
{

	write_client_reg(VSYNIF, 0x00000000, TRUE);	// LCD.VSYNCIF VSYNC mode OFF
	write_client_reg(PORT_ENB, 0x00000002, TRUE);	// LCD.PORT_ENB  ASYNC select

	write_client_reg(INTMASK, 0x00000001, TRUE);	// SYS.INTMASK
	//            # 01 : VWAKEOUT

	write_client_reg(TTBUSSEL, 0x0000000B, TRUE);	// MDC.TESTBUS
	write_client_reg(MONI, 0x00000008, TRUE);	// LCDC monitor sel

	write_client_reg(CLKENB, 0x000000EF, TRUE);	// SYS.CLKENB  DCLK OFF
	write_client_reg(CLKENB, 0x000010EF, TRUE);	// DCLK deviser set
	write_client_reg(CLKENB, 0x000011EF, TRUE);	// DCLK ON
	write_client_reg(BITMAP4, 0x00DC00B0, TRUE);	// BITMAP4 SUB PITCH SIZE set

	write_client_reg(HCYCLE, 0x0000006B, TRUE);	// LCD.HCYCLE  QCIFP size set
	write_client_reg(HSW, 0x00000003, TRUE);	// LCD.HSW
	write_client_reg(HDE_START, 0x00000002, TRUE);	// LCD.HDE_START
	write_client_reg(HDE_SIZE, 0x00000057, TRUE);	// LCD.HDE_SIZE
	write_client_reg(VCYCLE, 0x000000E6, TRUE);	// LCD.VCYCLE  0xE6=230
	//write  VCYCLE            ,  0x000001FF  # LCD.VCYCLE  0xFF=255
	write_client_reg(VSW, 0x00000001, TRUE);	// LCD.VSW
	write_client_reg(VDE_START, 0x00000003, TRUE);	// LCD.VDE_START
	write_client_reg(VDE_SIZE, 0x000000DB, TRUE);	// LCD.VDE_SIZE
	write_client_reg(WRSTB, 0x00000015, TRUE);	// LCD.WRSTB 1,1,1
	//write  WRSTB             ,  0x00000005  # LCD.WRSTB 0,1,1

	write_client_reg(MPLFBUF, 0x00000004, TRUE);	// LCD.MPLFBUF Sub select

	write_client_reg(ASY_DATA, 0x80000021, TRUE);	// LCD.ASY_DATx Write addr set
	write_client_reg(ASY_DATB, 0x00000000, TRUE);	//
	write_client_reg(ASY_DATC, 0x80000022, TRUE);	// LCD.ASY_DATx RAM write start
	write_client_reg(ASY_CMDSET, 0x00000007, TRUE);	// LCD.ASY_CMDSET

	write_client_reg(PXL, 0x00000089, TRUE);	// REFVSYNC=On,ACTMODE=2,RGB565
	write_client_reg(VSYNIF, 0x00000001, TRUE);	// LCD.VSYNCIF VSYNC mode ON

	mddi_wait(2);		//
}

static void toshiba_sec_cont_update_stop(void)
{
	write_client_reg(PXL, 0x00000000, TRUE);	// LCD.PXL
	write_client_reg(VSYNIF, 0x00000000, TRUE);	// LCD.VSYNCIF
	write_client_reg(START, 0x00000000, TRUE);	// LCD.START
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	// LCD.ASY_CMDSET
	mddi_wait(3);		//
	write_client_reg(SRST, 0x00000002, TRUE);	// LCD.SRST
	mddi_wait(3);		//
	write_client_reg(SRST, 0x00000003, TRUE);	// LCD.SRST
}

static void toshiba_sec_backlight_on(void)
{

	// LED control 10 kHz
	write_client_reg(TIMER0CTRL, 0x00000060, TRUE);	// PWM 0 control
	write_client_reg(TIMER0LOAD, 0x00001388, TRUE);	// PWM 0 Load
	write_client_reg(PWM0OFF, 0x00000001, TRUE);	// PWM 0 OFF

	write_client_reg(TIMER1CTRL, 0x00000060, TRUE);	// PWM 1 control
	write_client_reg(TIMER1LOAD, 0x00001388, TRUE);	// PWM 1 Load
	write_client_reg(PWM1OFF, 0x00001387, TRUE);	// PWM 1 OFF

	write_client_reg(TIMER0CTRL, 0x000000E0, TRUE);	//
	write_client_reg(TIMER1CTRL, 0x000000E0, TRUE);	//
	write_client_reg(PWMCR, 0x00000003, TRUE);	// LED output start
}

static void toshiba_sec_sleep_in(void)
{
	// Sub panel Display OFF ssequence

	write_client_reg(VSYNIF, 0x00000000, TRUE);	// LCD.VSYNCIF VSYNC mode OFF
	write_client_reg(PORT_ENB, 0x00000002, TRUE);	// LCD.PORT_ENB ASYNC select

	write_client_reg(ASY_DATA, 0x80000007, TRUE);	// LCD drive control
	write_client_reg(ASY_DATB, 0x00004016, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(2);		//0

	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// LTPS I/F control
	write_client_reg(ASY_DATB, 0x00000019, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(2);		//0

	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// LTPS I/F control
	write_client_reg(ASY_DATB, 0x0000000B, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(2);		//0

	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// LTPS I/F control
	write_client_reg(ASY_DATB, 0x00000002, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(4);		//0

	write_client_reg(ASY_DATA, 0x80000010, TRUE);	// POWER control
	write_client_reg(ASY_DATB, 0x00000300, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(4);		//0

	write_client_reg(ASY_DATA, 0x80000059, TRUE);	// LTPS I/F control
	write_client_reg(ASY_DATB, 0x00000000, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(2);		//0

	write_client_reg(ASY_DATA, 0x80000007, TRUE);	// LCD drive control
	write_client_reg(ASY_DATB, 0x00004004, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(2);		//0

	write_client_reg(PORT, 0x00000000, TRUE);	// DE output off
	write_client_reg(PXL, 0x00000000, TRUE);	// LCDC sleep mode
	write_client_reg(START, 0x00000000, TRUE);	// Syncchronous I/F OFF
	write_client_reg(REGENB, 0x00000001, TRUE);	// REGENBL VSYNC.

	// Sleep in sequence

	write_client_reg(ASY_DATA, 0x80000010, TRUE);	//
	write_client_reg(ASY_DATB, 0x00000302, TRUE);	//  # Sub LCDD Sleep in
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
}

static void toshiba_sec_sleep_out(void)
{

	write_client_reg(VSYNIF, 0x00000000, TRUE);	// LCD.VSYNCIF VSYNC mode OFF
	write_client_reg(PORT_ENB, 0x00000002, TRUE);	// LCD.PORT_ENB ASYNC select

	write_client_reg(ASY_DATA, 0x80000010, TRUE);	//
	write_client_reg(ASY_DATB, 0x00000300, TRUE);	//  # Sub LCDD sleep out
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//

	// Display ON sequence

	write_client_reg(ASY_DATA, 0x80000011, TRUE);	//
	write_client_reg(ASY_DATB, 0x00000812, TRUE);	//
	write_client_reg(ASY_DATC, 0x80000012, TRUE);	//
	write_client_reg(ASY_DATD, 0x00000003, TRUE);	//
	write_client_reg(ASY_DATE, 0x80000013, TRUE);	//
	write_client_reg(ASY_DATF, 0x00000909, TRUE);	//
	write_client_reg(ASY_DATG, 0x80000010, TRUE);	//
	write_client_reg(ASY_DATH, 0x00000040, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000001, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000000, TRUE);	//
	mddi_wait(4);		//0
	write_client_reg(ASY_DATA, 0x80000010, TRUE);	//
	write_client_reg(ASY_DATB, 0x00000340, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(6);		//0
	write_client_reg(ASY_DATA, 0x80000010, TRUE);	//
	write_client_reg(ASY_DATB, 0x00003340, TRUE);	//
	write_client_reg(ASY_DATC, 0x80000007, TRUE);	//
	write_client_reg(ASY_DATD, 0x00004007, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000009, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000008, TRUE);	//
	mddi_wait(1);		//
	write_client_reg(ASY_DATA, 0x80000007, TRUE);	//
	write_client_reg(ASY_DATB, 0x00004017, TRUE);	//
	write_client_reg(ASY_DATC, 0x8000005B, TRUE);	//
	write_client_reg(ASY_DATD, 0x00000000, TRUE);	//
	write_client_reg(ASY_DATE, 0x80000059, TRUE);	//
	write_client_reg(ASY_DATF, 0x00000011, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x0000000D, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x0000000C, TRUE);	//
	mddi_wait(2);		//0
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	//
	write_client_reg(ASY_DATB, 0x00000019, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(2);		//0
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	//
	write_client_reg(ASY_DATB, 0x00000079, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(2);		//0
	write_client_reg(ASY_DATA, 0x80000059, TRUE);	//
	write_client_reg(ASY_DATB, 0x000003FD, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000005, TRUE);	//
	write_client_reg(ASY_CMDSET, 0x00000004, TRUE);	//
	mddi_wait(2);		//0
}

static void mddi_toshiba_lcd_set_backlight(struct msm_fb_data_type *mfd)
{
	uint32 level;
	uint32 out_val;

	if (mfd->panel.id != TOSHIBA_VGA_PRIM)
		return;

	level = mfd->bl_level;

	if (machine_is_msm7201a_ffa()) {
		switch (level) {
		case 0:
			out_val = 0x00001387;
			break;

		case 1:
			out_val = 3700;
			break;

		case 2:
			out_val = 3200;
			break;

		case 3:
			out_val = 2700;
			break;

		case 4:
			out_val = 2200;
			break;

		default:
			out_val = 0x0000;
			break;
		}
	} else {
		switch (level) {
		case 0:
			out_val = 0x0000;
			break;

		case 1:
			out_val = 1250;
			break;

		case 2:
			out_val = 2500;
			break;

		case 3:
			out_val = 3750;
			break;

		case 4:
			out_val = 4999;
			break;

		default:
			out_val = 0x00001387;
			break;
		}
	}

	write_client_reg(PWM0OFF, out_val, TRUE);
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
		MDDI_MSG_ERR("MDDI: Previous Vsync handler never called\n");
	} else {
		/* Enable the vsync wakeup */
		mddi_queue_register_write(INTMSK, 0x0000, FALSE, 0);

		mddi_toshiba_vsync_attempts = 1;
		mddi_vsync_detect_enabled = TRUE;
	}
}				/* mddi_toshiba_vsync_set_handler */

static void mddi_toshiba_lcd_vsync_detected(boolean detected)
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
				(void)mddi_queue_register_read_int(VPOS,
								   &mddi_toshiba_curr_vpos);
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
		if (!mddi_queue_register_write_int(INTMSK, 0x0001)) {
			MDDI_MSG_ERR("Vsync interrupt disable failed!\n");
		}
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
		/* clear the interrupt */
		if (!mddi_queue_register_write_int(INTFLG, 0x0001)) {
			MDDI_MSG_ERR("Vsync interrupt clear failed!\n");
		}
	} else {
		/* if detected = FALSE, we woke up from hibernation, but did not
		 * detect client initiated wakeup.
		 */
		mddi_toshiba_vsync_attempts++;
	}
}

static void mddi_toshiba_prim_init(void)
{
	switch (toshiba_state) {
	case TOSHIBA_STATE_PRIM_SEC_READY:
		break;
	case TOSHIBA_STATE_PRIM_SEC_STANDBY:
	case TOSHIBA_STATE_OFF:
		toshiba_common_initial_setup();
		break;
	case TOSHIBA_STATE_SEC_NORMAL_MODE:
		toshiba_sec_cont_update_stop();
		toshiba_sec_sleep_in();
		toshiba_sec_sleep_out();
		toshiba_sec_lcd_off();
		toshiba_common_initial_setup();
		break;
	default:
		MDDI_MSG_ERR("mddi_toshiba_prim_init from state %d\n",
			     toshiba_state);
	}

	toshiba_prim_start();
	mddi_host_write_pix_attr_reg(0x00C3);
}

static void mddi_toshiba_sec_init(void)
{

	switch (toshiba_state) {
	case TOSHIBA_STATE_PRIM_SEC_READY:
		break;
	case TOSHIBA_STATE_PRIM_SEC_STANDBY:
		toshiba_common_initial_setup();
		break;
	case TOSHIBA_STATE_PRIM_NORMAL_MODE:
		toshiba_prim_lcd_off();
		toshiba_common_initial_setup();
		break;
	default:
		MDDI_MSG_ERR("mddi_toshiba_sec_init from state %d\n",
			     toshiba_state);
	}

	toshiba_sec_start();
	toshiba_sec_backlight_on();
	toshiba_sec_cont_update_start();
	mddi_host_write_pix_attr_reg(0x0400);
}

static void mddi_toshiba_lcd_powerdown(void)
{
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
}

static panel_info_type mddi_toshiba_lcd_get_disp_info(struct msm_fb_data_type
						      *mfd)
{
	if (mfd->panel.id == TOSHIBA_VGA_PRIM) {
		panel_info.xres = PRIM_WIDTH;
		panel_info.yres = PRIM_HEIGHT;
		panel_info.type = MDDI_PANEL;
		panel_info.pdest = DISPLAY_1;
		panel_info.wait_cycle = 0;
		panel_info.bpp = 18;

		// vsync config
		panel_info.lcd.vsync_enable = TRUE;
		panel_info.lcd.refx100 =
		    (mddi_toshiba_rows_per_second * 100) /
		    mddi_toshiba_rows_per_refresh;
		panel_info.lcd.v_back_porch = 6;
		panel_info.lcd.v_front_porch = 0;
		panel_info.lcd.v_pulse_width = 0;
		panel_info.lcd.hw_vsync_mode = FALSE;
		panel_info.lcd.vsync_notifier_period = (1 * HZ);

		// backlight level
		panel_info.bl_max = 4;
		panel_info.bl_min = 1;
	} else {
		panel_info.xres = SEC_WIDTH;
		panel_info.yres = SEC_HEIGHT;
		panel_info.type = MDDI_PANEL;
		panel_info.pdest = DISPLAY_2;
		panel_info.wait_cycle = 0;
		panel_info.bpp = 18;

		// vsync config
		panel_info.lcd.vsync_enable = FALSE;
	}
	return panel_info;
}

static int mddi_toshiba_lcd_init(struct msm_fb_data_type *mfd)
{
	mddi_host_type host_idx = MDDI_HOST_PRIM;

	/* Toshiba display requires larger drive_lo value */
	mddi_host_reg_out(DRIVE_LO, 0x0050);

	if (mfd->panel.id == TOSHIBA_VGA_PRIM) {
		mfd->mddi_vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		mddi_lcd.vsync_detected = mddi_toshiba_lcd_vsync_detected;
	} else {
		mfd->mddi_vdopkt = 0x400;
	}

	return 0;
}

static int mddi_toshiba_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (mfd->panel.id == TOSHIBA_VGA_PRIM) {
		mddi_toshiba_prim_init();
	} else {
		mddi_toshiba_sec_init();
	}

	return 0;
}

static int mddi_toshiba_lcd_off(struct platform_device *pdev)
{
	mddi_toshiba_lcd_powerdown();
	return 0;
}

struct msm_fb_panel_data toshiba_panel_data = {
	.init = mddi_toshiba_lcd_init,
	.get_info = mddi_toshiba_lcd_get_disp_info,
	.on = mddi_toshiba_lcd_on,
	.off = mddi_toshiba_lcd_off,
	.set_backlight = mddi_toshiba_lcd_set_backlight,
	.set_vsync_notifier = mddi_toshiba_vsync_set_handler,
};

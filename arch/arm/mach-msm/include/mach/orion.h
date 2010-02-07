/* arch/arm/mach-msm/include/mach/hardware.h
 *
 * Copyright (C) 2007 Google, Inc.
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

#ifndef ASM_MACH_ORION_H
#define ASM_MACH_ORION_H

/** GPIO definitions for Orion board ***/
extern int new_board_revison_chk;

#if defined(CONFIG_MACH_CAPELA_REV05)
//#define GPIO_WLAN_BT_REG_ON	20  // dgahn
#define GPIO_WLAN_BT_REG_ON (new_board_revison_chk ? 85 : 20)
#else
#define BCM4325_WLAN_WAKE	20
#endif

#define GPIO_WLAN_HOST_WAKE	28
//#define BCM4325_BT_RESET	38
#define BCM4325_BT_RESET (new_board_revison_chk ? 109 : 38)

#define GPIO_BT_UART_RTS	43
#define GPIO_BT_UART_CTS	44
#define GPIO_BT_UART_RXD	45
#define GPIO_BT_UART_TXD	46

#define GPIO_BT_PCM_DOUT	68
#define GPIO_BT_PCM_DIN		69
#define GPIO_BT_PCM_SYNC	70
#define GPIO_BT_PCM_CLK		71

#define BCM4325_BT_WAKE		77
#define BCM4325_WLAN_RESET	81
#define GPIO_BT_HOST_WAKE	94

#define GPIO_CHK_BOARD_REV 99

#if !defined(CONFIG_MACH_CAPELA_REV05) && !defined(CONFIG_MACH_CAPELA_REV03)
#define GPIO_WLAN_BT_REG_ON	104
//#define GPIO_WLAN_BT_REG_ON (new_board_revison_chk ? 85 : 20)
#endif

#define GPIO_SEND_END ( new_board_revison_chk?38:109 )


#endif  // ASM_MACH_ORION_H

/**
 * Copyright (c) 2007 Samsung Electronics, Inc.
 * All rights reserved.
 *
 * This software is a confidential and proprietary information
 * of Samsung Electronics, Inc. ("Confidential Information").  You
 * shall not disclose such Confidential Information and shall use
 * it only in accordance with the terms of the license agreement
 * you entered into with Samsung Electronics. 
 */

/**
 * This file contains the declaration of contact debug
 *
 * @file		pgh_debug.h
 * @version    0.1
 */


#ifndef	__PGH_DEBUG_H__
#define __PGH_DEBUG_H__


#ifdef PGH_ENABLE_DEBUG
#define PGH_DEBUG(fmt, arg...)	\
		do{\
		printk("\n\033[1;37;44m[PGH_DEBUG]%s:%d: " fmt "\033[0m", __FUNCTION__, __LINE__, ##arg);}\
		while(0)
#else
#define PGH_DEBUG(fmt, arg...)	
#endif


#endif // __PGH_DEBUG_H_


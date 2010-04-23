  /**
 * XOcean 

 * Copyright   2007 Samsung Electronics, Inc. 

 * All rights reserved. 

 * 

 * This program is free software: you can redistribute it and/or modify 

 * it under the terms of the GNU General Public License as published by 

 * the Free Software Foundation, version 2 of the License. 

 *  

 * This program is distributed in the hope that it will be useful, 

 * but WITHOUT ANY WARRANTY; without even the implied warranty of 

 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 

 * GNU General Public License for more details. 
 */

#ifndef	__PGH_DEBUG_H__
#define __PGH_DEBUG_H__

//#define PGH_ENABLE_DEBUG 	//ON/OFF


#ifdef PGH_ENABLE_DEBUG
#define PGH_DEBUG(fmt, arg...)	\
		do{\
		printk("\n\033[1;37;44m[PGH_DEBUG]%s:%d: " fmt "\033[0m", __FUNCTION__, __LINE__, ##arg);}\
		while(0)
#else
#define PGH_DEBUG(fmt, arg...)	
#endif


#endif // __PGH_DEBUG_H_

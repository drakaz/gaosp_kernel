/*
 * Copyright (c) 2009 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
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
#ifndef __MACH_CLK_H
#define __MACH_CLK_H

struct clk;

/* Rate is minimum clock rate in Hz */
int clk_set_min_rate(struct clk *clk, unsigned long rate);

/* Rate is maximum clock rate in Hz */
int clk_set_max_rate(struct clk *clk, unsigned long rate);

#endif

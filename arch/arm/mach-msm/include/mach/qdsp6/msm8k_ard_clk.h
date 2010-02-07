/*
 *
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
 *
 */

#ifndef ARDCLK_H
#define ARDCLK_H

#include "msm8k_ardi.h"

void ard_clk_enable(u32 dev_id);
void ard_clk_disable(u32 dev_id);

void ard_clk_set_icodec_rx_clk(void);
void ard_clk_set_icodec_tx_clk(void);

void ard_clk_enable_internal_codec_clk_rx(void);
void ard_clk_enable_internal_codec_clk_tx(void);
void ard_clk_disable_internal_codec_clk_rx(void);
void ard_clk_disable_internal_codec_clk_tx(void);

void ard_clk_enable_external_codec_clk(void);
void ard_clk_disable_external_codec_clk(void);
void ard_clk_set_ecodec_clk(void);

void ard_clk_enable_sdac_rx_clk(void);
void ard_clk_enable_sdac_tx_clk(void);
void ard_clk_set_sdac_rx_clk(void);
void ard_clk_set_sdac_tx_clk(void);
void ard_clk_disable_sdac_clk(void);

#endif



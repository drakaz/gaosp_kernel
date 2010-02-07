/*
 * Copyright (c) 2008 QUALCOMM USA, INC.
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
 * Q6 Video encoder remote device API.
 */
#include <linux/kernel.h>
#include <mach/dal.h>

enum {
	VENC_DALRPC_INITIALIZE = DALDEVICE_FIRST_DEVICE_API_IDX,
	VENC_DALRPC_SET_CB_CHANNEL,
	VENC_DALRPC_ENCODE,
	VENC_DALRPC_INTRA_REFRESH,
	VENC_DALRPC_RC_CONFIG,
	VENC_DALRPC_ENCODE_CONFIG,
	VENC_DALRPC_STOP,
};

static int q6venc_initialize(void *handle, const void *ibuf, uint32_t ilen)
{
	return dalrpc_fcn_5(VENC_DALRPC_INITIALIZE, handle, ibuf, ilen);
}

static int q6venc_set_cb_channel(void *handle, const void *ibuf, uint32_t ilen)
{
	return dalrpc_fcn_5(VENC_DALRPC_SET_CB_CHANNEL, handle, ibuf, ilen);
}

static int q6venc_encode(void *handle, const void *ibuf, uint32_t ilen)
{
	return dalrpc_fcn_5(VENC_DALRPC_ENCODE, handle, ibuf, ilen);
}

static int q6venc_intra_refresh(void *handle, const void *ibuf, uint32_t ilen)
{
	return dalrpc_fcn_5(VENC_DALRPC_INTRA_REFRESH, handle, ibuf, ilen);
}

static int q6venc_rc_config(void *handle, const void *ibuf, uint32_t ilen)
{
	return dalrpc_fcn_5(VENC_DALRPC_RC_CONFIG, handle, ibuf, ilen);
}

static int q6venc_encode_config(void *handle, const void *ibuf, uint32_t ilen)
{
	return dalrpc_fcn_5(VENC_DALRPC_ENCODE_CONFIG, handle, ibuf, ilen);
}

static int q6venc_stop(void *handle)
{
	return dalrpc_fcn_0(VENC_DALRPC_STOP, handle, 1);
}

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


#include <mach/qdsp6/msm8k_cad_q6dtmf_session.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_volume.h>
#include <mach/qdsp6/msm8k_cad_ioctl.h>

#if 0
#define D(fmt, args...) printk(KERN_INFO "msm8k_cad: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

s32 cad_dtmf_session_init(struct q6dtmf_session *self)
{
	return CAD_RES_SUCCESS;
}


s32 cad_dtmf_session_dinit(struct q6dtmf_session *self)
{
	return CAD_RES_SUCCESS;
}


s32 cad_dtmf_session_open(struct q6dtmf_session *self, s32 session_id,
				struct cad_open_struct_type *open_param)
{
	self->session_id = session_id;
	D("CAD:DTMF ===> Session open successful\n");
	return CAD_RES_SUCCESS;
}


s32 cad_dtmf_session_close(struct q6dtmf_session *self)
{
	self->session_id = 0;
	D("CAD:DTMF ===> Session close successful\n");
	return CAD_RES_SUCCESS;
}


s32 cad_dtmf_session_ioctl(struct q6dtmf_session *self, s32 cmd_code,
				void *cmd_buf, s32 cmd_len)
{
	struct cadi_evt_struct_type	return_status;
	struct q6_dtmf_start		q6data;
	struct cad_cmd_gen_dtmf		*data;
	s32				result = CAD_RES_SUCCESS;

	switch (cmd_code) {
	case CAD_IOCTL_CMD_GEN_DTMF:
		data = (struct cad_cmd_gen_dtmf *)cmd_buf;
		q6data.tone1_hz = data->dtmf_hi;
		q6data.tone2_hz = data->dtmf_low;
		q6data.duration_usec = data->duration * 1000;
		q6data.gain_mb = data->rx_gain;
		D("CAD:DTMF ===> send %d, %d, %d, %d\n", q6data.tone1_hz,
			q6data.tone2_hz, q6data.duration_usec, q6data.gain_mb);

		/* send the dtmf start with the configuration */
		result = cad_rpc_ioctl(self->session_id, 1,
			QDSP_IOCTL_CMD_STREAM_DTMF_START,
			(void *)&q6data, sizeof(q6data), &return_status);
		break;
	}

	return result;
}

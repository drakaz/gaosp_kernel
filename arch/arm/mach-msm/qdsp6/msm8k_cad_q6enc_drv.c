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

#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/semaphore.h>

#include <mach/qdsp6/msm8k_cad_q6enc_drvi.h>
#include <mach/qdsp6/msm8k_cad_module.h>
#include <mach/qdsp6/msm8k_cad_rpc.h>
#include <mach/qdsp6/msm8k_cad_q6enc_session.h>


static struct q6_audio_enc_data  q6_enc_data;


static s32 cad_q6enc_open(s32 session_id,
		struct cad_open_struct_type *open_param)
{
	struct q6_enc_session_data	*session = NULL;

	if (!open_param || (session_id >= CAD_MAX_SESSION) || session_id <= 0)
		return CAD_RES_FAILURE;

	if ((open_param->op_code == CAD_OPEN_OP_READ) &&
		q6_enc_data.q6_enc_free_sessions) {

		session = q6_enc_data.q6_enc_free_sessions;

		if (cad_q6enc_session_open(session, session_id, open_param) !=
			CAD_RES_SUCCESS) {

			cad_q6enc_session_close(session);
			return CAD_RES_FAILURE;
		}
		q6_enc_data.q6_enc_free_sessions =
			q6_enc_data.q6_enc_free_sessions->next;

		session->next = q6_enc_data.q6_enc_used_sessions;
		q6_enc_data.q6_enc_used_sessions = session;
	}
	return CAD_RES_SUCCESS;
}


static s32 cad_q6enc_close(s32 session_id)
{
	struct q6_enc_session_data *prev_session = NULL;
	struct q6_enc_session_data *session = q6_enc_data.q6_enc_used_sessions;

	if ((session_id >= CAD_MAX_SESSION) || (session_id <= 0))
		return CAD_RES_FAILURE;

	/* get the right session */
	while (session) {
		if (session_id == session->session_id) {
			if (prev_session == NULL) {
				/* first node */
				q6_enc_data.q6_enc_used_sessions =
					session->next;
				break;
			}
			/* not first node */
			prev_session->next = session->next;
			break;
		}
		prev_session = session;
		session = session->next;
	}

	if (session != NULL) {
		cad_q6enc_session_close(session);
		session->next = q6_enc_data.q6_enc_free_sessions;
		q6_enc_data.q6_enc_free_sessions = session;
	}

	return CAD_RES_SUCCESS;
}

static s32 cad_q6enc_write(s32 session_id, struct cad_buf_struct_type *buf)
{
	return CAD_RES_SUCCESS;
}

static s32 cad_q6enc_read(s32 session_id, struct cad_buf_struct_type *buf)
{
	struct q6_enc_session_data *session = q6_enc_data.q6_enc_used_sessions;

	if (!buf)
		return CAD_RES_FAILURE;

	while (session != NULL) {
		if (session_id == session->session_id) {
			if (cad_q6enc_session_read(session, buf)
					!= CAD_RES_SUCCESS)
				return CAD_RES_FAILURE;
			break;
		}
		session = session->next;
	}
	return CAD_RES_SUCCESS;
}

static s32 cad_q6enc_ioctl(s32 session_id, u32 cmd_code, void *cmd_buf,
			u32 cmd_len)
{
	struct q6_enc_session_data *session = q6_enc_data.q6_enc_used_sessions;

	while (session != NULL) {
		if (session_id == session->session_id) {
			if (cad_q6enc_session_ioctl(session, cmd_code, cmd_buf)
					!= CAD_RES_SUCCESS)
				return CAD_RES_FAILURE;
			break;
		}
		session = session->next;
	}
	return CAD_RES_SUCCESS;
}


s32 cad_audio_enc_dinit(void)
{
	struct q6_enc_session_data *session = NULL;

	while (q6_enc_data.q6_enc_free_sessions) {
		session = q6_enc_data.q6_enc_free_sessions->next;
		cad_q6enc_session_dinit(q6_enc_data.q6_enc_free_sessions);
		kfree(q6_enc_data.q6_enc_free_sessions);
		q6_enc_data.q6_enc_free_sessions = session;
	}

	while (q6_enc_data.q6_enc_used_sessions) {
		session = q6_enc_data.q6_enc_used_sessions->next;
		cad_q6enc_session_dinit(q6_enc_data.q6_enc_used_sessions);
		kfree(q6_enc_data.q6_enc_used_sessions);
		q6_enc_data.q6_enc_used_sessions = session;
	}

	memset(&q6_enc_data, 0, sizeof(q6_enc_data));
	return CAD_RES_SUCCESS;
}


s32 cad_audio_enc_init(struct cad_func_tbl_type **func_ptr_tbl)
{
	u32 i;
	struct q6_enc_session_data *node = NULL;

	static struct cad_func_tbl_type  vtable = {
		cad_q6enc_open,
		cad_q6enc_close,
		cad_q6enc_write,
		cad_q6enc_read,
		cad_q6enc_ioctl
	};

	memset(&q6_enc_data, 0, sizeof(q6_enc_data));

	/* create session list */
	for (i = 0; i < Q6_ENC_MAX_SESSION_COUNT; i++) {

		node = kmalloc(sizeof(struct q6_enc_session_data), GFP_KERNEL);
		if (node == NULL) {
			cad_audio_enc_dinit();
			return CAD_RES_FAILURE;
		}

		memset(node, 0, sizeof(struct q6_enc_session_data));
		if (cad_q6enc_session_init(node) != CAD_RES_SUCCESS) {
			cad_audio_enc_dinit();
			return CAD_RES_FAILURE;
		}
		node->next = q6_enc_data.q6_enc_free_sessions;
		q6_enc_data.q6_enc_free_sessions = node;
	}
	*func_ptr_tbl = &vtable;
	return CAD_RES_SUCCESS;
}

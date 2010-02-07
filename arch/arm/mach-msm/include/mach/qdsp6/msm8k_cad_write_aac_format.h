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

#ifndef CAD_WRITE_AAC_FORMAT_H
#define CAD_WRITE_AAC_FORMAT_H

#define CAD_WRITE_AAC_VERSION_10	0x10

/*
* 0. 96000, 1. 88200, 2. 64000, 3. 48000, 4. 44100, 5. 32000
* 6. 24000, 7. 22050, 8. 16000, 9. 12000, 10. 11025, 11. 8000
*/
#define CAD_SAMPLE_RATE_96000		0
#define CAD_SAMPLE_RATE_88200		1
#define CAD_SAMPLE_RATE_64000		2
#define CAD_SAMPLE_RATE_48000		3
#define CAD_SAMPLE_RATE_44100		4
#define CAD_SAMPLE_RATE_32000		5
#define CAD_SAMPLE_RATE_24000		6
#define CAD_SAMPLE_RATE_22050		7
#define CAD_SAMPLE_RATE_16000		8
#define CAD_SAMPLE_RATE_12000		9
#define CAD_SAMPLE_RATE_11025		10
#define CAD_SAMPLE_RATE_08000		11

/*
*  1: Mono
*  2: Stereo (interleaved)
*/
#define CAD_CHANNEL_CFG_MONO		1
#define CAD_CHANNEL_CFG_STEREO		2

/*
*  0xffff, ADTS
*  0     , RAW (ADIF) format
*  1     , PSUEDO-RAW format
*  2     , LOAS format
*/
#define CAD_BLK_FMT_ADTS		0xFFFF
#define CAD_BLK_FMT_RAW			0
#define CAD_BLK_FMT_PSEUDO_RAW		1
#define CAD_BLK_FMT_LOAS		2

/*
*  2, bitstream is in AAC LC format
*  4, bitstream is in AAC LTP format
* 17, bitstream is in ER AAC LC format
* 22, bitsream is in  AAC BSAC format
* other audio object types are not supported
*/
#define CAD_AUDIO_OBJ_TYPE_AAC_LC	2
#define CAD_AUDIO_OBJ_TYPE_AAC_LTP	4
#define CAD_AUDIO_OBJ_TYPE_ER_AAC_LC	17
#define CAD_AUDIO_OBJ_TYPE_AAC_BSAC	22

/*
*  ={0,1,2,3}, indicating the configuration of the error
*  protection scheme. This information shall be retrieved from
* the mp4 header. This information is required by the DSP only
* when AOT==17. Currently, only epConfig=0 is supported.
*/
#define CAD_ERR_PROT_SCHEME_0		0
#define CAD_ERR_PROT_SCHEME_1		1
#define CAD_ERR_PROT_SCHEME_2		2
#define CAD_ERR_PROT_SCHEME_3		3


struct cad_write_aac_struct_type {
	u16	sample_rate;
	u16	channel_config;
	u16	block_formats;
	u16	audio_object_type;
	u16	ep_config;
	/*
	* ={0,1}, indicating whether the VCB11 (an error resilience tool) is
	* used. This information shall be retrieved from the mp4 header.
	* Note that this field must be zero if (AOT!=17)
	*/
	u16	aac_section_data_resilience_flag;
	/*
	*  ={0,1}, indicating whether the RVLC (an error resilience tool)
		is used.
	* This information shall be retrieved from the mp4 header. Note that
	* this field must be zero if (AOT!=17)
	*/
	u16	aac_scalefactor_data_resilience_flag;
	/*
	* ={0,1}, indicating whether the HCR (an error resilience tool) is used.
	* This information shall be retrieved from the mp4 header. Note that
	* this field must be zero if (AOT!=17)
	*/
	u16	aac_spectral_data_resilience_flag;
	/*
	* = 1 to turn on SBR if present in the bitstream
	* = 0 to turn off SBR
	*/
	u16	sbr_on_flag;
	/*
	* = 1 to turn on PS if present in the bitstream.
	* = 0 to turn off PS
	*/
	u16	sbr_ps_on_flag;
	u32	bit_rate;
};


struct cad_write_aac_format_struct_type {
	u16 ver_id;
	struct cad_write_aac_struct_type aac;
};

#endif

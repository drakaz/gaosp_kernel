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

#ifndef CAD_WRITE_MIDI_FORMAT_H
#define CAD_WRITE_MIDI_FORMAT_H

#define CAD_WRITE_MIDI_VERSION_10 0x10


struct cad_write_midi_struct_type {
	/*
	* 0. 96000, 1. 88200, 2. 64000, 3. 48000, 4. 44100, 5. 32000
	* 6. 24000, 7. 22050, 8. 16000, 9. 12000, 10. 11025, 11. 8000
	*/
	u16  sample_rate;

	/*
	*  0x1 = indicates synthesize mono output
	*  0x2 = indicates synthesize stereo output
	*/
	u16  midi_stereo;

	/*
	*       hybridMode	HP	HQ	Comments
	*       0x0000		72	0	All-HP (Default)
	*       0x0001		56	4
	*       0x0002		46	8
	*       0x0003		38	12
	*       0x0004		30	16
	*       0x0005		20	20
	*       0x0006		12	24
	*       0x0007		0	33	All-HQ
	*/
	u16  hybrid_mode;

	/*
	*  DLS program definition base. If this is not a DLS File then all
	*  DLS entries will be 0
	*/
	u16  dls_prog_def_base_lsw;
	u16  dls_prog_def_base_msw;

	/*
	* DLS drum definition base
	*/

	u16  dls_drum_def_base_lsw;
	u16  dls_drum_def_base_msw;

	/*
	*  DLS Prog Max Key Base
	*/
	u16  dls_prog_max_key_base_lsw;
	u16  dls_prog_max_key_base_msw;

	/*
	* DLS Waveform Info Base
	*/
	u16  dls_wf_info_base_lsw;
	u16  dls_wf_info_base_msw;

	/*
	* DLS HQ Waveform Info Base
	*/
	u16  dls_hq_wf_info_base_lsw;
	u16  dls_hq_wf_info_base_msw;

	/*
	* DLS HQ Program Pointer
	*/
	u16  dls_hq_prog_lsw;
	u16  dls_hq_prog_msw;

	/*
	* DLS HQ Drum Pointer
	*/
	u16  dls_hq_drum_lsw;
	u16  dls_hq_drum_msw;

	 /*
	* Possible values are 0 or 1 (1 indicates SMAF and
	* 0 indicates GM)
	*/
	u16  volume_lookup_index;

};


struct cad_write_midi_format_struct_type {
   u16	ver_id;
   cad_write_midi_struct_type  midi;
};
#endif




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

#ifndef CAD_WRITE_MP3_FORMAT_H
#define CAD_WRITE_MP3_FORMAT_H

#define CAD_WRITE_MP3_VERSION_10	0x10

struct cad_write_mp3_struct_type {
	/*
	*  0. 96000, 1. 88200, 2. 64000, 3. 48000, 4. 44100, 5. 32000
	*  6. 24000, 7. 22050, 8. 16000, 9. 12000, 10. 11025, 11. 8000
	*/
	u16 sample_rate;
};

struct cad_write_mp3_format_struct_type {
	u16 ver_id;
	struct cad_write_mp3_struct_type mp3;
};

#endif

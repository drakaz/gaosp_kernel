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

enum spkr_left_right_type {
    LEFT_SPKR,
    RIGHT_SPKR,
    SPKR_OUT_OF_RANGE          /* Not valid */
};

/* Valid gain values for the PMIC Speaker */
enum spkr_gain_type {
    SPKR_GAIN_MINUS16DB,      /* -16 db */
    SPKR_GAIN_MINUS12DB,      /* -12 db */
    SPKR_GAIN_MINUS08DB,      /* -08 db */
    SPKR_GAIN_MINUS04DB,      /* -04 db */
    SPKR_GAIN_00DB,           /*  00 db */
    SPKR_GAIN_PLUS04DB,       /* +04 db */
    SPKR_GAIN_PLUS08DB,       /* +08 db */
    SPKR_GAIN_PLUS12DB,       /* +12 db */
    SPKR_GAIN_OUT_OF_RANGE    /* Not valid */
};

/* Turn the speaker on or off and enables or disables mute.*/
enum spkr_cmd_type {
    SPKR_DISABLE,  /* Enable Speaker                                 */
    SPKR_ENABLE,   /* Disable Speaker                                */
    SPKR_MUTE_OFF, /* turn speaker mute off, SOUND ON                */
    SPKR_MUTE_ON,  /* turn speaker mute on, SOUND OFF                */
    SPKR_OFF,      /* turn speaker OFF (speaker disable and mute on) */
    SPKR_ON,        /* turn speaker ON (speaker enable and mute off)  */
    SPKR_SET_FREQ_CMD,    /* set speaker frequency */
    SPKR_GET_FREQ_CMD,    /* get speaker frequency */
    SPKR_SET_GAIN_CMD,    /* set speaker gain */
    SPKR_GET_GAIN_CMD,    /* get speaker gain */
    SPKR_SET_DELAY_CMD,   /* set speaker delay */
    SPKR_GET_DELAY_CMD,   /* get speaker delay */
    SPKR_SET_PDM_MODE,
    SPKR_SET_PWM_MODE,
    SPKR_CMD_OUT_OF_RANGE /* Not valid */
};

int spkr_en_right_chan(unsigned char enable);
int spkr_is_right_chan_en(unsigned char *enabled);
int spkr_en_left_chan(unsigned char enable);
int spkr_is_left_chan_en(unsigned char *enabled);
int spkr_is_en(enum spkr_left_right_type left_right, unsigned char *enabled);
int spkr_get_gain(enum spkr_left_right_type left_right,
		  enum spkr_gain_type *gain);
int set_speaker_gain(enum spkr_gain_type speaker_gain);
int speaker_cmd(enum spkr_cmd_type cmd);

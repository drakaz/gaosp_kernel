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

enum spkr_left_right {
    LEFT_SPKR,
    RIGHT_SPKR,
    SPKR_OUT_OF_RANGE          /* Not valid */
};

enum spkr_gain {
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
enum spkr_cmd {
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

struct spkr_config_mode {
    uint32_t is_right_chan_en;
    uint32_t is_left_chan_en;
    uint32_t is_right_left_chan_added;
    uint32_t is_stereo_en;
    uint32_t is_usb_with_hpf_20hz;
    uint32_t is_mux_bypassed;
    uint32_t is_hpf_en;
    uint32_t is_sink_curr_from_ref_volt_cir_en;
};

enum mic_volt {
    MIC_VOLT_2_00V,            /*  2.00 V  */
    MIC_VOLT_1_93V,            /*  1.93 V  */
    MIC_VOLT_1_80V,            /*  1.80 V  */
    MIC_VOLT_1_73V,            /*  1.73 V  */
    MIC_VOLT_OUT_OF_RANGE      /* Not valid */
};

enum ledtype {
	LED_LCD,
	LED_KEYPAD,
	LED_TYPE_OUT_OF_RANGE
};

int spkr_en_right_chan(const unsigned char enable);
int spkr_is_right_chan_en(unsigned char *enabled);
int spkr_en_left_chan(const unsigned char enable);
int spkr_is_left_chan_en(unsigned char *enabled);
int spkr_is_en(const enum spkr_left_right left_right, unsigned char *enabled);
int spkr_get_gain(const enum spkr_left_right left_right, enum spkr_gain *gain);
int set_speaker_gain(const enum spkr_gain speaker_gain);
int speaker_cmd(const enum spkr_cmd cmd);
int set_spkr_configuration(const struct spkr_config_mode *t);
int get_spkr_configuration(struct spkr_config_mode *t);

int mic_en(const unsigned char enable);
int mic_is_en(unsigned char *enabled);
int mic_set_volt(const enum mic_volt type);

/* Cannot use 'current' as the parameter name because 'current' is defined as
 * a macro to get a pointer to the current task.
 */
int flash_led_set_current(const uint16_t milliamps);

int set_led_intensity(const enum ledtype type, int val);

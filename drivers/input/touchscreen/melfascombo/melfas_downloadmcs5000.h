//--------------------------------------------------------
//
//    MELFAS Firmware download base code for MCS5000
//    Version : v01
//    Date    : 2009.01.20
//
//--------------------------------------------------------

//#ifndef __MELFAS_FIRMWARE_DOWNLOADMCS5000_H__
//#define __MELFAS_FIRMWARE_DOWNLOADMCS5000_H__


//============================================================
//
//    Porting factors for Baseband
//
//============================================================



#define CONFIG_MELFAS_TOUCH_VERSION5  //MCS5000 은 F/W Version 05 부터 시작 
				                                          //mcs6000 은 F/W Version 03 까지
#define FEATURE_MELFAS_DISABLE_DOWNLOAD_IF_MODULE_VERSION_NOT_MATCH
#define FEATURE_MELFAS_ENABLE_DBG_PRINT
//=====================================================================
//
//   MELFAS Firmware download
//
//=====================================================================

#define MELFAS_TRANSFER_LENGTH_5000        64	    // Program & Read flash block size

//-----------------------------------------------
//	MELFAS Version information address
//-----------------------------------------------
#define MCSDL_ADDR_MODULE_REVISION_5000    0x97
#define MCSDL_ADDR_FIRMWARE_VERSION_5000   0x9B

#ifdef MELFAS_ENABLE_DOWNLOAD_ENABLE_COMMAND
//-----------------------------------------------
//	Command address of requesting enable reset. 
//-----------------------------------------------
#define MCSDL_ADDR_ENABLE_MODULE_RESET_5000          0xED

//-----------------------------------------------
//	Command of requesting enable reset. 
//-----------------------------------------------
#define MCSDL_CMD_ENABLE_MODULE_RESET_5000           0xED
#endif

//----------------------------------------------------
//   Return values of download function
//----------------------------------------------------
#define MCSDL_RET_SUCCESS_5000						0x00
#define MCSDL_RET_ENTER_DOWNLOAD_MODE_FAILED_5000	0x01
#define MCSDL_RET_ERASE_FLASH_FAILED_5000			0x02
#define MCSDL_RET_ERASE_VERIFY_FAILED_5000			0x03
#define MCSDL_RET_READ_FLASH_FAILED_5000				0x04
#define MCSDL_RET_READ_EEPROM_FAILED_5000			0x05
#define MCSDL_RET_READ_INFORMAION_FAILED_5000		0x06
#define MCSDL_RET_PROGRAM_FLASH_FAILED_5000			0x07
#define MCSDL_RET_PROGRAM_EEPROM_FAILED_5000			0x08
#define MCSDL_RET_PROGRAM_INFORMAION_FAILED_5000		0x09
#define MCSDL_RET_PROGRAM_VERIFY_FAILED_5000			0x0A

#define MCSDL_RET_WRONG_MODE_ERROR_5000				0xF0
#define MCSDL_RET_WRONG_SLAVE_SELECTION_ERROR_5000	0xF1
#define MCSDL_RET_WRONG_PARAMETER_5000				0xF2
#define MCSDL_RET_COMMUNICATION_FAILED_5000			0xF3
#define MCSDL_RET_READING_HEXFILE_FAILED_5000		0xF4
#define MCSDL_RET_FILE_ACCESS_FAILED_5000			0xF5
#define MCSDL_RET_MELLOC_FAILED_5000					0xF6
#define MCSDL_RET_WRONG_MODULE_REVISION_5000			0xF7
#define MCSDL_RET_SAME_FIRMWARE_VERSION_5000		    0xF8  // KGH_CA
//-----------------------------------------------
//	MELFAS Firmware source type
//-----------------------------------------------
//#define MELFAS_DOWNLOAD_TYPE_BINARY 0x01

//------------------------------
// MDS ISP mode entering
//------------------------------
#define MCSDL_MDS_ENTERING_ISP_MODE_CODE2_5000		0x00

#define MCSDL_MDS_ENTERING_ISP_MODE_ACK_1_5000		0x55
#define MCSDL_MDS_ENTERING_ISP_MODE_ACK_2_5000		0x80

//------------------------------
// ISP commands - MDS & I2C
//------------------------------
#define MCSDL_ISP_CMD_ERASE_5000	                	0x02
#define MCSDL_ISP_CMD_PROGRAM_FLASH_5000	        	0x03
#define MCSDL_ISP_CMD_READ_FLASH_5000	        	0x04
#define MCSDL_ISP_CMD_PROGRAM_INFORMATION_5000		0x05
#define MCSDL_ISP_CMD_READ_INFORMATION_5000	    	0x06
#define MCSDL_ISP_CMD_RESET_5000	                	0x07

//------------------------------
// MCS5000's responses
//------------------------------
#define MCSDL_ISP_ACK_ERASE_DONE_2ND_MDS_5000		0x81
#define MCSDL_ISP_ACK_ERASE_DONE_5000	        	0x82
#define MCSDL_I2C_ACK_PROGRAM_INFORMATION_5000		0x85
#define MCSDL_MDS_ACK_PROGRAM_FLASH_5000	        	0x83
#define MCSDL_MDS_ACK_READ_FLASH_5000	        	0x84
#define MCSDL_MDS_ACK_PROGRAM_INFORMATION_5000		0x88
#define MCSDL_MDS_ACK_PROGRAM_LOCKED_5000	    	0xFE
#define MCSDL_MDS_ACK_READ_LOCKED_5000	        	0xFE
#define MCSDL_MDS_ACK_FAIL_5000	                	0xFE

//------------------------------
//	I2C ISP
//------------------------------

#define MCSDL_I2C_SLAVE_ADDR_ORG_5000                0x20                            
#define MCSDL_I2C_SLAVE_ADDR_DN_5000	            	0x7F	                        /* Down load Address - 7bit*/
#define MCSDL_I2C_SLAVE_ADDR_DN_SHIFTED_5000	        (MCSDL_I2C_SLAVE_ADDR_DN_5000<<1)    // Adress after sifting.

#define MCSDL_I2C_SLAVE_READY_STATUS_5000	    	0x55

#define MCSDL_ISP_PROGRAM_TIMING_VALUE_5000	    	0x78
#define MCSDL_ISP_PROGRAM_TIMING_VALUE_0_5000		0xC0
#define MCSDL_ISP_PROGRAM_TIMING_VALUE_1_5000		0xD4
#define MCSDL_ISP_PROGRAM_TIMING_VALUE_2_5000		0x01

//----------------------------------------------------
//	Functions
//----------------------------------------------------

int  melfas_ts_download_firmware5000(void);

// MELFAS HEX Studio v0.5 [2008.12.11]
//#endif        //#ifndef __MELFAS_FIRMWARE_DOWNLOADMCS5000_H__
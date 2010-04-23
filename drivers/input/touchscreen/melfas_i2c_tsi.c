/* drivers/input/keyboard/melfas_i2c_mth_st909.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/miscdevice.h>
#include <mach/vreg.h>
#include "melfas_i2c_tsi.h"

#define INPUT_TOUCH_DEBUG

#ifdef INPUT_TOUCH_DEBUG 
#define DPRINTK(X...) do { printk("%s(): ", __FUNCTION__); printk(X); } while(0)
#else
#define DPRINTK(x...)        /* !!!! */
#endif

#define MAX_ABS_X  383
#define MAX_ABS_Y  511

static struct vreg *vreg_touch;	
static struct workqueue_struct *melfas_wq;

struct melfas_ts_pdev {
	unsigned short force[3];
	unsigned short *forces[2];
	struct i2c_client_address_data addr_data;
	int irq;
};

struct melfas_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static int melfas_ts_download_firmware(struct i2c_client *client);

static int ts_irq_num;
extern int bridge_on;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE
static int melfas_ts_enter_download_mode(void);
static int melfas_ts_i2c_erase_flash(void);
static int melfas_ts_i2c_read_flash(uint8_t *pBuffer, uint16_t nAddr_start, uint8_t cLength);
static int mcsdl_i2c_program_info(void);
static int melfas_ts_i2c_program_flash( uint8_t *pData, uint16_t nAddr_start, uint8_t cLength );

static void melfas_ts_hw_i2c_mode(int on) // 0: OFF, 1: ON
{
    if(on)
    {
      gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
      gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
	}
	else
	{
      gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);
	  gpio_set_value(TOUCH_I2C_SCL_F, 0);
      gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);
	  gpio_set_value(TOUCH_I2C_SDA_F, 0);
	}
}

static void melfas_ts_int_mode(int on)  // 0: OFF, 1: ON
{
    if(on)
    {
	  gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);		
    }
	else
	{
      gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);		
	  gpio_set_value(TOUCH_INT, 0);
    }
}

//============================================================
//
//	Porting section 6.	I2C function calling
//
//	Connect baseband i2c function
//
//	Warning 1. !!!!  Burst mode is not supported. Transfer 1 byte Only.
//
//    	Every i2c packet has to
//            " START > Slave address > One byte > STOP " at download mode.
//
//	Warning 2. !!!!  Check return value of i2c function.
//
//    	_i2c_read_(), _i2c_write_() must return
//        	TRUE (1) if success,
//        	FALSE(0) if failed.
//
//    	If baseband i2c function returns different value, convert return value.
//        	ex> baseband_return = baseband_i2c_read( slave_addr, pData, cLength );
//            	return ( baseband_return == BASEBAND_RETURN_VALUE_SUCCESS );
//
//
//	Warning 3. !!!!  Check Slave address
//
//    	Slave address is '0x7F' at download mode. ( Diffrent with Normal touch working mode )
//        '0x7F' is original address,
//        	If shift << 1 bit, It becomes '0xFE'
//
//============================================================
void i2c_write_byte(uint8_t cData)
{
    int i;
    
    for(i=7; i>=0; i--)
    {
        if( (cData>>i) & 0x01){
			gpio_set_value(TOUCH_I2C_SDA_F, 1);
        }else{
			gpio_set_value(TOUCH_I2C_SDA_F, 0);
        }

        udelay(1);

        gpio_set_value(TOUCH_I2C_SCL_F, 1); 
		udelay(1);
        gpio_set_value(TOUCH_I2C_SCL_F, 0); 
		udelay(1);
    }
}

void i2c_read_byte(uint8_t *pData)
{
    int i;
    
    *pData  = 0;
	for(i=7; i>=0; i--){

    	gpio_set_value(TOUCH_I2C_SCL_F, 1);

    	if( gpio_get_value(TOUCH_I2C_SDA_F) ){
            *pData |= 0x1<<i;
        }

    	gpio_set_value(TOUCH_I2C_SCL_F, 0);
    }
}

static int _i2c_read_( uint8_t slave_addr, uint8_t *pData)
{
	int bRet = 0;
    
#ifdef USE_BASEBAND_I2C_FUNCTION

	bRet = baseband_i2c_read_a_byte(slave_addr, pData);

#else //USE_BASEBAND_I2C_FUNCTION

    // START
	gpio_set_value(TOUCH_I2C_SDA_F, 0);
	gpio_set_value(TOUCH_I2C_SCL_F, 0);     
	
    //Write slave addr with read bit.
    i2c_write_byte( (slave_addr<<1)|1 );

    // CHKECK ACK
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SCL_F, 1);  
	bRet = gpio_get_value(TOUCH_I2C_SDA_F);	
	gpio_set_value(TOUCH_I2C_SCL_F, 0); 

	if( bRet )
    	return 0;

	udelay(15);

    i2c_read_byte( pData );

    // SEND NAK
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SDA_F, 1);
	gpio_set_value(TOUCH_I2C_SCL_F, 1);     
	gpio_set_value(TOUCH_I2C_SCL_F, 0);     

	udelay(15);
    
    // STOP
    gpio_set_value(TOUCH_I2C_SDA_F, 0);
	gpio_set_value(TOUCH_I2C_SCL_F, 1);     
    gpio_set_value(TOUCH_I2C_SDA_F, 1);

	return 1;
#endif //USE_BASEBAND_I2C_FUNCTION
}

static int _i2c_write_(uint8_t slave_addr, uint8_t data)
{
	int bRet = 0;

#ifdef USE_BASEBAND_I2C_FUNCTION

    bRet = baseband_i2c_write_a_byte( slave_addr, data);

#else //USE_BASEBAND_I2C_FUNCTION

    // START
	gpio_set_value(TOUCH_I2C_SDA_F, 0);
	gpio_set_value(TOUCH_I2C_SCL_F, 0);     

    //Write Slave Addr with write bit
    i2c_write_byte( (slave_addr<<1)|0 );

    // CHKECK ACK
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SCL_F, 1);  
	bRet = gpio_get_value(TOUCH_I2C_SDA_F);	
	gpio_set_value(TOUCH_I2C_SCL_F, 0); 
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);	

	if( bRet )
    	return 0;

	i2c_write_byte(data);

    // CHKECK ACK
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SCL_F, 1);  
	bRet = gpio_get_value(TOUCH_I2C_SDA_F);	
	gpio_set_value(TOUCH_I2C_SCL_F, 0); 
    gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),GPIO_ENABLE);	

    // STOP
	gpio_set_value(TOUCH_I2C_SDA_F, 0);  
	gpio_set_value(TOUCH_I2C_SCL_F, 1);
	gpio_set_value(TOUCH_I2C_SDA_F, 1);  

	return 1;
    
#endif //USE_BASEBAND_I2C_FUNCTION
}

#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
//============================================================
//
//	Debugging print functions.
//
//	Change MCSDL_PRINT() to Baseband printing function
//
//============================================================
static void melfas_ts_print_result(int nRet)
{
    if( nRet == MCSDL_RET_SAME_FIRMWARE_VERSION ){
		printk("[MELFAS] Firmware Version is Same, Not download.\n");
    }	
	else if( nRet == MCSDL_RET_SUCCESS ){

		printk("[MELFAS] Firmware downloading SUCCESS.\n");

	}else{

		printk("[MELFAS] Firmware downloading FAILED  :  ");

		switch( nRet ){

			case MCSDL_RET_SUCCESS                  		:   printk("[MELFAS] MCSDL_RET_SUCCESS\n" );                 	    break;
			case MCSDL_RET_ENTER_DOWNLOAD_MODE_FAILED   	:   printk("[MELFAS] MCSDL_RET_ENTER_ISP_MODE_FAILED\n" );      	break;
			case MCSDL_RET_ERASE_FLASH_FAILED           	:   printk("[MELFAS] MCSDL_RET_ERASE_FLASH_FAILED\n" );         	break;
			case MCSDL_RET_READ_FLASH_FAILED				:   printk("[MELFAS] MCSDL_RET_READ_FLASH_FAILED\n" );         	    break;
			case MCSDL_RET_READ_EEPROM_FAILED           	:   printk("[MELFAS] MCSDL_RET_READ_EEPROM_FAILED\n" );         	break;
			case MCSDL_RET_READ_INFORMAION_FAILED        	:   printk("[MELFAS] MCSDL_RET_READ_INFORMAION_FAILED\n" );     	break;
			case MCSDL_RET_PROGRAM_FLASH_FAILED				:   printk("[MELFAS] MCSDL_RET_PROGRAM_FLASH_FAILED\n" );        	break;
			case MCSDL_RET_PROGRAM_EEPROM_FAILED        	:   printk("[MELFAS] MCSDL_RET_PROGRAM_EEPROM_FAILED\n" );      	break;
			case MCSDL_RET_PROGRAM_INFORMAION_FAILED    	:   printk("[MELFAS] MCSDL_RET_PROGRAM_INFORMAION_FAILED\n" );      break;
			case MCSDL_RET_PROGRAM_VERIFY_FAILED			:   printk("[MELFAS] MCSDL_RET_PROGRAM_VERIFY_FAILED\n" );      	break;

			case MCSDL_RET_WRONG_MODE_ERROR             	:   printk("[MELFAS] MCSDL_RET_WRONG_MODE_ERROR\n" );         	    break;
			case MCSDL_RET_WRONG_SLAVE_SELECTION_ERROR		:   printk("[MELFAS] MCSDL_RET_WRONG_SLAVE_SELECTION_ERROR\n" );    break;
			case MCSDL_RET_COMMUNICATION_FAILED				:   printk("[MELFAS] MCSDL_RET_COMMUNICATION_FAILED\n" );      	    break;
			case MCSDL_RET_READING_HEXFILE_FAILED       	:   printk("[MELFAS] MCSDL_RET_READING_HEXFILE_FAILED\n" );         break;
			case MCSDL_RET_WRONG_PARAMETER       			:   printk("[MELFAS] MCSDL_RET_WRONG_PARAMETER\n" );      		    break;
			case MCSDL_RET_FILE_ACCESS_FAILED       		:   printk("[MELFAS] MCSDL_RET_FILE_ACCESS_FAILED\n" );      	    break;
			case MCSDL_RET_MELLOC_FAILED     		  		:   printk("[MELFAS] MCSDL_RET_MELLOC_FAILED\n" );      			break;
			case MCSDL_RET_WRONG_MODULE_REVISION     		:   printk("[MELFAS] MCSDL_RET_WRONG_MODULE_REVISION\n" );      	break;

			default                             			:	printk("[MELFAS] UNKNOWN ERROR. [0x%02X].\n", nRet );        	break;
		}

		printk("\n");
	}

}
#endif

#if 1 // KHG_0619
int mcsdl_i2c_mark_finished( void )
{
    int i, nRet, bRet;
    uint8_t i2c_buffer[4];


    nRet = MCSDL_RET_PROGRAM_FLASH_FAILED;
    //-----------------------------
    // Send Program Command
    //-----------------------------
    i2c_buffer[0] = MCSDL_ISP_CMD_PROGRAM_FLASH;
    i2c_buffer[1] = (uint8_t)((MCSDL_I2C_ADDRESS_FINISH_MARK >> 8 ) & 0xFF);
    i2c_buffer[2] = (uint8_t)((MCSDL_I2C_ADDRESS_FINISH_MARK      ) & 0xFF);
    i2c_buffer[3] = 1;
 
    for(i=0; i<4; i++){

        bRet = _i2c_write_(MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer[i]);

        udelay(15);
		
        if( bRet == 0 )
            goto MCSDL_I2C_MARK_FINISHED_END;

    }

    //-----------------------------
    // Program Finish-Mark
    //-----------------------------

    bRet = _i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, MCSDL_I2C_DATA_FINISH_MARK);

    udelay(50);

    if( bRet == 0 )
        goto MCSDL_I2C_MARK_FINISHED_END;

    //-----------------------------
    // Get result
    //-----------------------------

    bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &i2c_buffer[0]);

    if( bRet == 0 || i2c_buffer[0] != MCSDL_MDS_ACK_PROGRAM_FLASH )
        goto MCSDL_I2C_MARK_FINISHED_END;

    nRet = MCSDL_RET_SUCCESS;

MCSDL_I2C_MARK_FINISHED_END :

   return nRet;
}


int mcsdl_i2c_check_finished( void )
{
    int i, nRet, bRet;
    uint8_t i2c_buffer[4];

    nRet = MCSDL_RET_READ_INFORMAION_FAILED;
    //-----------------------------
    // Send Read Command
    //-----------------------------
    i2c_buffer[0] = MCSDL_ISP_CMD_READ_FLASH;
    i2c_buffer[1] = (uint8_t)((MCSDL_I2C_ADDRESS_FINISH_MARK >> 8 ) & 0xFF);
    i2c_buffer[2] = (uint8_t)((MCSDL_I2C_ADDRESS_FINISH_MARK      ) & 0xFF);
    i2c_buffer[3] = 1;
 
    for(i=0; i<4; i++){

        bRet = _i2c_write_(MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer[i]);

        udelay(15);
		
        if( bRet == 0 )
            goto MCSDL_I2C_CHECK_FINISHED_END;

    }

    //-----------------------------
    // Read Finish-Mark
    //-----------------------------

    bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &i2c_buffer[0]);

    udelay(50);

    if( bRet == 0 )
        goto MCSDL_I2C_CHECK_FINISHED_END;

    if(i2c_buffer[0] != MCSDL_I2C_DATA_FINISH_MARK){
        goto MCSDL_I2C_CHECK_FINISHED_END;
    }

    nRet = MCSDL_RET_SUCCESS;

MCSDL_I2C_CHECK_FINISHED_END :

   return nRet;
}
#endif
//--------------------------------------------
//
//   Write ISP Mode entering signal
//
//--------------------------------------------
static void melfas_ts_write_download_mode_signal(void)
{
	int    i;

	uint8_t enter_code[14] = { 0, 1, 0, 1, 0, 1, 0, 1,   1, 0, 0, 1, 0, 1 };

	//---------------------------
	// ISP mode signal 0
	//---------------------------

	for(i=0; i<14; i++){

		if( enter_code[i] )	{
			gpio_set_value(TOUCH_INT, 1);

		}else{
			gpio_set_value(TOUCH_INT, 0);
		}

		gpio_set_value(TOUCH_I2C_SCL_F, 1);
	
		udelay(15);
		gpio_set_value(TOUCH_I2C_SCL_F, 0);

		gpio_set_value(TOUCH_INT, 0);

		udelay(100);
   }

	gpio_set_value(TOUCH_I2C_SCL_F, 1);

	udelay(100);

	gpio_set_value(TOUCH_INT, 1);
}

//------------------------------------------------------------------
//
//   Enter Download mode ( MDS ISP or I2C ISP )
//
//------------------------------------------------------------------
static int melfas_ts_enter_download_mode(void)
{

	int	nRet = MCSDL_RET_ENTER_DOWNLOAD_MODE_FAILED;
	int bRet;

	uint8_t cData=0;

	//--------------------------------------------
	// Tkey module reset
	//--------------------------------------------
	vreg_disable(vreg_touch);

    gpio_set_value(TOUCH_EN, 0);

    melfas_ts_hw_i2c_mode(0);

    melfas_ts_int_mode(0);

	msleep(45);

	msleep(45);

	vreg_enable(vreg_touch);

    gpio_set_value(TOUCH_EN, 1);
	

    gpio_set_value(TOUCH_I2C_SDA_F, 1);
	
    
	msleep(25);

	//-------------------------------
	// Write 1st signal
	//-------------------------------
	melfas_ts_write_download_mode_signal();

	msleep(25);

//	melfas_ts_hw_i2c_mode(1);

	//-------------------------------
	// Check response
	//-------------------------------

	bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &cData);

#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("[MELFAS] melfas_ts_enter_download_mode() returns - ret : 0x%x \n", bRet);
#endif
    if( bRet != 1 || cData != MCSDL_I2C_SLAVE_READY_STATUS )
		goto MCSDL_ENTER_DOWNLOAD_MODE_FINISH;

	nRet = MCSDL_RET_SUCCESS;

	//-----------------------------------
	// Entering MDS ISP mode finished.
	//-----------------------------------

MCSDL_ENTER_DOWNLOAD_MODE_FINISH:

   return nRet;
}

//------------------------------------------------------------------
//
//	Download function
//
//------------------------------------------------------------------

int melfas_ts_download(const uint8_t *pData, const uint16_t nLength )
{
	int		i;
	int		nRet;

	uint16_t  nCurrent=0;
	uint8_t   cLength;

	uint8_t	buffer[MELFAS_TRANSFER_LENGTH];
	uint8_t	buffer2[MELFAS_TRANSFER_LENGTH];

	uint8_t	*pBuffer;

	#ifdef FEATURE_MELFAS_DISABLE_DOWNLOAD_IF_MODULE_VERSION_NOT_MATCH
	uint8_t	melfas_module_revision;
	uint8_t	melfas_module_revision_of_new_firmware;
	uint8_t	melfas_module_hw_revision;
	#endif

#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("[MELFAS] Starting download...\n");
#endif    
	//--------------------------------------------------------------
	//
	// Enter Download mode
	//
	//--------------------------------------------------------------
	nRet = melfas_ts_enter_download_mode();

	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;

	msleep(1);					// Delay '1 msec'

	//--------------------------------------------------------------
	//
	// Check H/W Revision
	//
	// Don't download firmware, if Module H/W revision does not match.
	//
	//--------------------------------------------------------------
	#ifdef FEATURE_MELFAS_DISABLE_DOWNLOAD_IF_MODULE_VERSION_NOT_MATCH


		pBuffer  = (uint8_t *)pData;

		melfas_module_revision_of_new_firmware =
				((( pBuffer[MCSDL_ADDR_FIRMWARE_VERSION+1] - '0' ) & 0x0F) << 4)
			+ 	( pBuffer[MCSDL_ADDR_FIRMWARE_VERSION+2] - '0' );


		nRet = melfas_ts_i2c_read_flash( buffer, MCSDL_ADDR_FIRMWARE_VERSION, 8 );

		if( nRet != MCSDL_RET_SUCCESS )
			goto MCSDL_DOWNLOAD_FINISH;

		melfas_module_revision = ((( buffer[1]-'0' )&0x0F) << 4 ) + ( buffer[2] - '0' );

#if 0
		nRet = melfas_ts_i2c_read_flash( buffer, MCSDL_ADDR_MODULE_REVISION, 8 );
        if( nRet != MCSDL_RET_SUCCESS )
			goto MCSDL_DOWNLOAD_FINISH;

		melfas_module_hw_revision = ((( buffer2[1]-'0' )&0x0F) << 4 ) + ( buffer2[2] - '0' );
#endif
		
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
#if 1
		printk("[MELFAS] melfas_revision_of_new_firmware = 0x%x, melfas_firmware_revision = 0x%x, buffer[1]=%x,buffer[2]=%x,\n",
			melfas_module_revision_of_new_firmware,melfas_module_revision,melfas_module_hw_revision,buffer[1],buffer[2]);
#else
		printk("[MELFAS] melfas_revision_of_new_firmware = 0x%x, melfas_firmware_revision = 0x%x\n",
			melfas_module_revision_of_new_firmware,melfas_module_revision);
#endif			
#endif			
        if( mcsdl_i2c_check_finished() == MCSDL_RET_SUCCESS)   // KGH_0619
        {
#if 0			
            if( melfas_module_hw_revision != MELFAS_LATEST_HW_MODULE_REVSION)
            {
  				nRet = MCSDL_RET_NOT_SUPPORT_HW_VERSION;
			goto MCSDL_DOWNLOAD_FINISH;

		}
#endif
			if( melfas_module_revision == melfas_module_revision_of_new_firmware)
			{
			nRet = MCSDL_RET_SAME_FIRMWARE_VERSION;
			goto MCSDL_DOWNLOAD_FINISH;
		}
		msleep(1);					// Delay '1 msec'
        }

	#endif



	//--------------------------------------------------------------
	//
	// Erase Flash
	//
	//--------------------------------------------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("[MELFAS] Erasing...\n");
#endif

	nRet = melfas_ts_i2c_erase_flash();

	if( nRet != MCSDL_RET_SUCCESS ){
		goto MCSDL_DOWNLOAD_FINISH;
	}

	msleep(1);					// Delay '1 msec'

#if 0
	//---------------------------
	//
	// Verify erase
	//
	//---------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT	
	printk("[MELFAS] Verify Erasing...\n");
#endif

	nRet = melfas_ts_i2c_read_flash( buffer, 0x00, 16 );

	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;

	for(i=0; i<16; i++){

		if( buffer[i] != 0xFF ){

			nRet = MCSDL_RET_ERASE_VERIFY_FAILED;
			goto MCSDL_DOWNLOAD_FINISH;
		}
	}

	msleep(1);					// Delay '1 msec'
#endif

	//-------------------------------
	//
	// Program flash information
	//
	//-------------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("[MELFAS] Program information...\n");
#endif

	nRet = mcsdl_i2c_program_info();

	if( nRet != MCSDL_RET_SUCCESS )
		goto MCSDL_DOWNLOAD_FINISH;


	msleep(1);					// Delay '1 msec'


   //-------------------------------
   // Program flash
   //-------------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("[MELFAS] Program flash...  ");
#endif

	pBuffer  = (uint8_t *)pData;
	nCurrent = 0;
	cLength  = MELFAS_TRANSFER_LENGTH;

	while( nCurrent < nLength ){
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
		printk("#");
#endif
		if( ( nLength - nCurrent ) < MELFAS_TRANSFER_LENGTH ){
			cLength = (uint8_t)(nLength - nCurrent);
		}

		nRet = melfas_ts_i2c_program_flash( pBuffer, nCurrent, cLength );

        if( nRet != MCSDL_RET_SUCCESS ){

			printk("[MELFAS] Program flash failed position : 0x%x / nRet : 0x%x ", nCurrent, nRet);
            goto MCSDL_DOWNLOAD_FINISH;
		}

		pBuffer  += cLength;
		nCurrent += (uint16_t)cLength;

		msleep(1);					// Delay '1 msec'

	}
#ifdef CONFIG_MACH_BEHOLD2 // KGH_0619
	nRet = mcsdl_i2c_mark_finished();

	if(nRet != MCSDL_RET_SUCCESS){
	         goto MCSDL_DOWNLOAD_FINISH;
	}
#endif	
#if 0
	//-------------------------------
	//
	// Verify flash
	//
	//-------------------------------
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("\n");
	printk("[MELFAS] Verify flash...   ");
#endif

	pBuffer  = (uint8_t *) pData;

	nCurrent = 0;

	cLength  = MELFAS_TRANSFER_LENGTH;

	while( nCurrent < nLength ){
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
		printk("#");
#endif
		if( ( nLength - nCurrent ) < MELFAS_TRANSFER_LENGTH ){
			cLength = (uint8_t)(nLength - nCurrent);
		}

		//--------------------
		// Read flash
		//--------------------
		nRet = melfas_ts_i2c_read_flash( buffer, nCurrent, cLength );

		//--------------------
		// Comparing
		//--------------------
		for(i=0; i<(int)cLength; i++){

			if( buffer[i] != pBuffer[i] ){

				printk("0x%04X : 0x%02X - 0x%02X\n", nCurrent, pBuffer[i], buffer[i] );
				nRet = MCSDL_RET_PROGRAM_VERIFY_FAILED;
				goto MCSDL_DOWNLOAD_FINISH;

			}
		}

		pBuffer  += cLength;
		nCurrent += (uint16_t)cLength;

		msleep(1);					// Delay '1 msec'
	}
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
	printk("\n");
#endif
#endif
	nRet = MCSDL_RET_SUCCESS;


MCSDL_DOWNLOAD_FINISH :

	msleep(1);					// Delay '1 msec'

	//---------------------------
	//	Reset command
	//---------------------------
	buffer[0] = MCSDL_ISP_CMD_RESET;

	_i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, buffer[0]);

    msleep(300);
#if defined(CONFIG_SAMSUNG_BIGFOOT)    
    melfas_ts_hw_i2c_mode(1);
#else
    melfas_ts_hw_i2c_mode(0);
#endif
    melfas_ts_int_mode(1);
#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT
    printk("[MELFAS] Reset!!! \n");
#endif
	return nRet;
}

//--------------------------------------------
//
//   Erase flash
//
//--------------------------------------------
static int melfas_ts_i2c_erase_flash(void)
{
	int   nRet = MCSDL_RET_ERASE_FLASH_FAILED;

	uint8_t i;
	int   bRet;

	uint8_t i2c_buffer[4] = {	MCSDL_ISP_CMD_ERASE,
	                        MCSDL_ISP_PROGRAM_TIMING_VALUE_2,
	                        MCSDL_ISP_PROGRAM_TIMING_VALUE_1,
	                        MCSDL_ISP_PROGRAM_TIMING_VALUE_0   };

   //-----------------------------
   // Send Erase code
   //-----------------------------

   for(i=0; i<4; i++){

		bRet = _i2c_write_(MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer[i]);

		if( !bRet )
			goto MCSDL_I2C_ERASE_FLASH_FINISH;

		udelay(15);
   }

   //-----------------------------
   // Read Result
   //-----------------------------

	msleep(45);                  // Delay 45ms


	bRet = _i2c_read_(MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer);

	if( bRet && i2c_buffer[0] == MCSDL_ISP_ACK_ERASE_DONE ){

		nRet = MCSDL_RET_SUCCESS;

	}


MCSDL_I2C_ERASE_FLASH_FINISH :

   return nRet;

}

//--------------------------------------------
//
//   Read flash
//
//--------------------------------------------
static int melfas_ts_i2c_read_flash(uint8_t *pBuffer, uint16_t nAddr_start, uint8_t cLength)
{
	int nRet = MCSDL_RET_READ_FLASH_FAILED;

	int     i;
	int   bRet;
	uint8_t   cmd[4];

	//-----------------------------------------------------------------------------
	// Send Read Flash command   [ Read code - address high - address low - size ]
	//-----------------------------------------------------------------------------

	cmd[0] = MCSDL_ISP_CMD_READ_FLASH;
	cmd[1] = (uint8_t)((nAddr_start >> 8 ) & 0xFF);
	cmd[2] = (uint8_t)((nAddr_start      ) & 0xFF);
	cmd[3] = cLength;

	for(i=0; i<4; i++){

		bRet = _i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, cmd[i]);

		udelay(15);

		if( bRet == 0 )
			goto MCSDL_I2C_READ_FLASH_FINISH;

   }

	//----------------------------------
	// Read Data  [ pCmd[3] == Size ]
	//----------------------------------
	for(i=0; i<(int)cmd[3]; i++){

		udelay(100);                  // Delay about 100us

		bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, pBuffer++);

		if( bRet == 0 )
			goto MCSDL_I2C_READ_FLASH_FINISH;
	}

	nRet = MCSDL_RET_SUCCESS;


MCSDL_I2C_READ_FLASH_FINISH :

	return nRet;
}

//--------------------------------------------
//
//   Program information
//
//--------------------------------------------
static int mcsdl_i2c_program_info(void)
{

	int nRet = MCSDL_RET_PROGRAM_INFORMAION_FAILED;

	int i;
	int j;
	int bRet;

	uint8_t i2c_buffer[5] = { MCSDL_ISP_CMD_PROGRAM_INFORMATION,
		                    MCSDL_ISP_PROGRAM_TIMING_VALUE,
		                    0x00,                           // High addr
		                    0x00,                           // Low  addr
		                    0x00 };                         // Data

	uint8_t info_data[] = { 0x78, 0x00, 0xC0, 0xD4, 0x01 };

	//------------------------------------------------------
	//   Send information signal for programming flash
	//------------------------------------------------------
	for(i=0; i<5; i++){

		i2c_buffer[3] = 0x08 + i;            // Low addr
		i2c_buffer[4] = info_data[i];         // Program data

		for(j=0; j<5; j++){

			bRet = _i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, i2c_buffer[j]);

			if( bRet == 0 )
				goto MCSDL_I2C_PROGRAM_INFO_FINISH;

			udelay(15);
		}

		udelay(500);                     // delay about  500us

		bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &i2c_buffer[4]);

		if( bRet == 0 || i2c_buffer[4] != MCSDL_I2C_ACK_PROGRAM_INFORMATION )
			goto MCSDL_I2C_PROGRAM_INFO_FINISH;

		udelay(100);                     // delay about  100us

   }

   nRet = MCSDL_RET_SUCCESS;

MCSDL_I2C_PROGRAM_INFO_FINISH :

   return nRet;

}

//--------------------------------------------
//
//   Program Flash
//
//--------------------------------------------

static int melfas_ts_i2c_program_flash( uint8_t *pData, uint16_t nAddr_start, uint8_t cLength )
{
	int nRet = MCSDL_RET_PROGRAM_FLASH_FAILED;

	int     i;
	int   bRet;
	uint8_t    cData;

	uint8_t cmd[4];

	//-----------------------------
	// Send Read code
	//-----------------------------

	cmd[0] = MCSDL_ISP_CMD_PROGRAM_FLASH;
	cmd[1] = (uint8_t)((nAddr_start >> 8 ) & 0xFF);
	cmd[2] = (uint8_t)((nAddr_start      ) & 0xFF);
	cmd[3] = cLength;

	for(i=0; i<4; i++){

		bRet = _i2c_write_(MCSDL_I2C_SLAVE_ADDR_DN, cmd[i]);

		udelay(15);

		if( bRet == 0 )
			goto MCSDL_I2C_PROGRAM_FLASH_FINISH;

	}

	//-----------------------------
	// Program Data
	//-----------------------------

	udelay(500);                  // Delay about 500us

	for(i=0; i<(int)(cmd[3]); i++){


		bRet = _i2c_write_( MCSDL_I2C_SLAVE_ADDR_DN, pData[i]);

		udelay(500);                  // Delay about 500us

		if( bRet == 0 )
			goto MCSDL_I2C_PROGRAM_FLASH_FINISH;
	}

	//-----------------------------
	// Get result
	//-----------------------------

	bRet = _i2c_read_( MCSDL_I2C_SLAVE_ADDR_DN, &cData);

	if( bRet == 0 || cData != MCSDL_MDS_ACK_PROGRAM_FLASH )
		goto MCSDL_I2C_PROGRAM_FLASH_FINISH;

	nRet = MCSDL_RET_SUCCESS;

MCSDL_I2C_PROGRAM_FLASH_FINISH :

   return nRet;
}
#if 1
static int melfas_ts_download_firmware(struct i2c_client *client)
{
#else
static int melfas_ts_download_firmware()
{
#endif
	int ret;

	udelay(15);

	disable_irq(ts_irq_num);					// Disable Baseband touch interrupt ISR.

	//------------------------
	// Run Download
	//------------------------
	ret = melfas_ts_download(MELFAS_binary, MELFAS_binary_nLength);

	enable_irq(ts_irq_num);					// Roll-back Baseband touch interrupt ISR.

	#ifdef FEATURE_MELFAS_ENABLE_DBG_PRINT

		//------------------------
		// Show result
		//------------------------

		melfas_ts_print_result( ret );

	#endif


	return ( ret == MCSDL_RET_SUCCESS );
}
#endif

static void melfas_tsp_init(void)
{
	int rc;
	printk(KERN_INFO "melfas_tsp_esd_init because of esd recovery\n");

	// TOUCH OFF
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SCL_F, 0);
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SDA_F, 0);
	gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_INT, 0);

	gpio_direction_output(TOUCH_EN, 0);  // TOUCH EN

	rc = vreg_disable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not disable gp3\n");
	msleep(1000);

	// TOUCH ON
	rc = vreg_enable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not enable gp3\n");
	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN

	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SCL_F, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SCL_F, 1);
	gpio_tlmm_config(GPIO_CFG(TOUCH_I2C_SDA_F, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_I2C_SDA_F, 1);
	gpio_tlmm_config(GPIO_CFG(TOUCH_INT, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA),GPIO_ENABLE);
	gpio_set_value(TOUCH_INT, 1);

	msleep(300);
}

static void melfas_ts_work_func(struct work_struct *work)
{
//	int i;
	int ret, count;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf1[7];
	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);
#if 0
	int x, y, z, rc;
	int finger, width;
#endif
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x10;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf1);
	msg[1].buf = buf1;

//    printk("melfas_ts_work_func: ");  

#if 1
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
		printk(KERN_ERR "melfas_ts_work_func: i2c_transfer failed\n");
		melfas_tsp_init();
	} else {
	   if(bridge_on)  // lcd on status
	   {
			int x = buf1[2] | (uint16_t)(buf1[1] & 0x03) << 8; 
			int y = buf1[4] | (uint16_t)(buf1[3] & 0x0f) << 8; 
			int z = buf1[5];
			int finger = buf1[0] & 0x01;
			int width = buf1[6];

//	        printk("x%4d, y%4d, z%4d, finger%4d, width%4d\n",x, y, z, buf1[0],width); 

			if((buf1[0] == 0)||(buf1[0] == 1)) // Only Finger Touch, Palm Touch Disable
			{
				//if(!(buf1[0]))
				//			 printk("x%4d, y%4d, z%4d, finger%4d, width%4d\n",x, y, z, finger,width); 
				if ( x < 320 && x > 0 && y < 480 && y > 0 ) {
					input_report_abs(ts->input_dev, ABS_X, x);
					input_report_abs(ts->input_dev, ABS_Y, y);
					input_report_abs(ts->input_dev, ABS_PRESSURE, z);
					input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
					input_report_key(ts->input_dev, BTN_TOUCH, finger);
					input_sync(ts->input_dev);
				}
#if defined(CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_DEVICE_CHECK)
//				if ( x < 50 && y < 50)
//					printk("x%4d, y%4d, z%4d, finger%4d, width%4d\n",x, y, z, finger,width);
					
				if (finger == 0)
					printk("T%3d%3d\n", x, y);
#endif
				input_report_abs(ts->input_dev, ABS_PRESSURE, z);
				input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
				input_report_key(ts->input_dev, BTN_TOUCH, finger);
				input_sync(ts->input_dev);
			}

//			printk("[KTH] buf1[0]=%x \n", buf1[0]);
			if (buf1[0] == 0xed || x == 941 || y == 2989 )
			{
				printk("[KTH] touch_ESD test\n");

				melfas_tsp_init();

			}
			if(!(gpio_get_value(TOUCH_EN)))
			{
				printk(KERN_ERR "melfas_ts_work_func: TOUCH_EN is Low\n");
				gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN
				msleep(200);
			}
	    }
	}

#if 0 // KTHUR
	char version_id[4];
	ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
	}
	sprintf(version_id, "%x", ret);
	printk(KERN_INFO "melfas_ts_probe: Firmware Version %x %s %x\n", ret, version_id, version_id);
	ts->input_dev input_id->vendor  = ret; //version_id ;
	printk(KERN_INFO "melfas_ts_probe: Firmware Version %x\n", ret);
#else
#if 0
	char version_id[4];
	ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
	}
	sprintf(version_id, "%x", ret);
	ts->input_dev->id.version  = ret; //version_id;
#endif
#endif

#else
	count = 0;
	do {
		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret < 0) 
			printk(KERN_ERR "melfas_ts_work_func: i2c_transfer failed\n");

		count++;
		if (count > 5)
		{
			printk(KERN_ERR"melfas_ts_work_func: i2c FAIL %d times TOUCH DEVICE RESET", count);
		
			gpio_set_value(TOUCH_EN, 0);  // TOUCH EN
			gpio_set_value(TOUCH_INT, 0);  // TOUCH INT 
			
			msleep(200);
			
			gpio_set_value(TOUCH_EN, 1);  // TOUCH EN
			gpio_set_value(TOUCH_INT, 1);  // TOUCH INT 

		}

	} while (ret < 0);

	x = buf1[2] | (uint16_t)(buf1[1] & 0x03) << 8; 
	y = buf1[4] | (uint16_t)(buf1[3] & 0x0f) << 8; 
	z = buf1[5];
	finger = buf1[0] & 0x01;
	width = buf1[6];

	//			 printk("x%4d, y%4d, z%4d, finger%4d, width%4d\n",x, y, z, finger,width); 

	if (x)
		input_report_abs(ts->input_dev, ABS_X, x);
	if (y)
		input_report_abs(ts->input_dev, ABS_Y, y);
	printk("[KTH] X:%d Y:%d \n", x, y);
	input_report_abs(ts->input_dev, ABS_PRESSURE, z);
	input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, width);
	input_report_key(ts->input_dev, BTN_TOUCH, finger);
	input_sync(ts->input_dev);
#endif
	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart melfas_ts_timer_func(struct hrtimer *timer)
{
	struct melfas_ts_data *ts = container_of(timer, struct melfas_ts_data, timer);
	/* printk("melfas_ts_timer_func\n"); */

	queue_work(melfas_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	struct melfas_ts_data *ts = dev_id;

	/* printk("melfas_ts_irq_handler\n"); */
	disable_irq(ts->client->irq);
	queue_work(melfas_wq, &ts->work);
	return IRQ_HANDLED;
}

static int melfas_fw_open(struct inode *inode, struct file *file)
{
#if 1
	printk(KERN_INFO "[melfas_fw_open] %s\n", __FUNCTION__);
#endif
	return nonseekable_open(inode, file);
}

static int melfas_fw_release(struct inode *inode, struct file *file)
{
#if 1
	printk(KERN_INFO "[melfas_fw_release] %s\n", __FUNCTION__);
#endif
	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE

#define MELFAS_TSP_IOCTL_MAGIC 78

#define TSP_FW_UPDATE _IO(MELFAS_TSP_IOCTL_MAGIC, 1)

static int melfas_fw_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
       int ret;
	   
		switch (cmd) {
			case TSP_FW_UPDATE:
#if DEBUG
				printk("[melfas_fw_ioctl] TSP_FW_UPDATE %x\n", cmd);
#endif
				//ret = melfas_ts_download_firmware();
				if (ret < 0)
					return ret;
				break;
			default:
#if DEBUG
				printk("Unknown cmd %x\n", cmd);
#endif
				return -ENOTTY;
		}

		return 0;
}
static struct file_operations melfas_fw_fops = {
	.owner = THIS_MODULE,
	.open = melfas_fw_open,
	.release = melfas_fw_release,
	.ioctl = melfas_fw_ioctl,
};

static struct miscdevice melfas_fw_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "melfas_fw_download",
	.fops = &melfas_fw_fops,
};
#endif

static int __init melfas_ts_probe(struct i2c_client *client)
{
	struct melfas_ts_data *ts;

	int ret = 0;
	uint16_t max_x, max_y;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, melfas_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	
#ifdef CONFIG_MACH_BEHOLD2
#ifdef CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE
    ret = melfas_ts_download_firmware(ts->client);
	msleep(50);
#endif
#endif

	ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "melfas_ts_probe: Firmware Version %x\n", ret);

	ret = i2c_smbus_read_byte_data(ts->client, 0x21);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}
	printk(KERN_INFO "melfas_ts_probe: Module Revision %x\n", ret);

	ret = i2c_smbus_read_word_data(ts->client, 0x08);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	ts->max[0] = max_x = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);
	ret = i2c_smbus_read_word_data(ts->client, 0x0a);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_word_data failed\n");
		goto err_detect_failed;
	}
	ts->max[1] = max_y = (ret >> 8 & 0xff) | ((ret & 0x03) << 8);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "melfas_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "melfas-tsi-touchscreen";
#if 1 // KTHUR
	char version_id[4];
	ret = i2c_smbus_read_byte_data(ts->client, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
	}
	ts->input_dev->id.version  = ret; //version_id;
#endif
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

    printk("melfas_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	input_set_abs_params(ts->input_dev, ABS_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
#ifdef CONFIG_TOUCHSCREEN_MELFAS_I2C_TSI_FW_UPDATE
    ret = misc_register(&melfas_fw_device);
	if (ret) {
		printk(KERN_ERR "melfas_fw_device: melfas_fw_device register failed\n");
		goto err_misc_register_device_failed;
	}
#endif

    ts_irq_num = client->irq;

	if (client->irq) {
#if 1  // Because of Lock up
		ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);
#else
		ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_LOW, client->name, ts);
#endif
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = melfas_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#if 0 //def CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 4;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "melfas_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;
err_misc_register_device_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret,rc;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq); 
#if 1 // save wake up time 
	gpio_direction_output(TOUCH_EN, 0);  // TOUCH EN
	msleep(20);
    
	rc = vreg_disable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not disable gp3\n");
	msleep(200);
#endif
    printk(KERN_INFO "Melfas Touchscreen : suspend.\n");
	
	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	int rc;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

#if 1 // save wake up time
	rc = vreg_enable(vreg_touch);
	if (rc )
		printk(KERN_ERR "can not enable gp3\n");
	msleep(20);

	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN
	msleep(200);
#endif
	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

    printk(KERN_INFO "Melfas Touchscreen : resume.\n");
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ "melfas-tsi-ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, melfas_ts_id);

static struct i2c_driver melfas_ts_driver = {
	.id_table	= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= melfas_ts_remove,
#if 1 //ndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
	.driver = {
		.name	= "melfas-tsi-ts",
	},
};

static int __devinit melfas_ts_init(void)
{
	melfas_wq = create_singlethread_workqueue("melfas_wq");
	if (!melfas_wq)
		return -ENOMEM;

	vreg_touch = vreg_get(0, "gp3");
	if (IS_ERR(vreg_touch))
		return PTR_ERR(vreg_touch);

#if 1
	gpio_direction_output(TOUCH_EN, 0);  // TOUCH EN

	msleep(100);

	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN

	msleep(200);

#else
	gpio_direction_output(TOUCH_EN, 1);  // TOUCH EN
	msleep(300);
#endif

	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
	if (melfas_wq)
		destroy_workqueue(melfas_wq);
}
module_init(melfas_ts_init);
module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");


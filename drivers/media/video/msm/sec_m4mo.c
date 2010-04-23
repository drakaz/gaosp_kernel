/*
  SEC M4MO
 */
/***************************************************************
CAMERA DRIVER FOR 5M CAM(FUJITSU M4Mo) by PGH
ver 0.1 : only preview (base on universal)
****************************************************************/

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>//PGH


#include "sec_m4mo.h"

#include <asm/gpio.h> //PGH

#if 1//Mclk_timing for M4Mo spec.
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/board.h>
#endif



//unuse standby pin #define CAM_STB						85
#define CAM_RST						17
#define CAM_ON						76//REV02
//#define CAM_ON						22//REV01
#define CAM_FLASH_EN				23
#define CAM_FLASH_SET				31



#define LOW							0
#define HIGH						1
#define M4MO_IMG_I2C_ADDR			0x3F



static int preview_flag = 0;
static int old_hw = 0;

#if defined(CONFIG_SAMSUNG_TARGET)

static struct i2c_client *cam_pm_lp8720_pclient; 

struct cam_pm_lp8720_data {
	struct work_struct work;
#if 0//PGH
	#ifdef CONFIG_ANDROID_POWER
		struct android_early_suspend early_suspend;
	#endif
#endif//PGH
};

static DECLARE_WAIT_QUEUE_HEAD(cam_pm_lp8720_wait_queue);
DECLARE_MUTEX(cam_pm_lp8720_sem);

#endif//PGH FOR CAM PM LP8720 ////////////////////////////////////


struct m4mo_work_t {
	struct work_struct work;
};

static struct  m4mo_work_t *m4mo_sensorw;
static struct  i2c_client *m4mo_client;

struct m4mo_ctrl_t {
	int8_t  opened;
	struct  msm_camera_sensor_info 	*sensordata;
};



static struct m4mo_ctrl_t *m4mo_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(m4mo_wait_queue);
DECLARE_MUTEX(m4mo_sem);


#if 0 //not used
static int m4mo_reset(struct msm_camera_sensor_info *dev)
{
	int rc = 0;

	rc = gpio_request(dev->sensor_reset, "m4mo");

	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 0);
		mdelay(20);
		rc = gpio_direction_output(dev->sensor_reset, 1);
	}

	gpio_free(dev->sensor_reset);
	return rc;
}
#endif

static int32_t m4mo_i2c_txdata(unsigned short saddr, char *txdata, int length)
{
	struct i2c_msg msg[] = {
	{
		.addr = saddr,
		.flags = 0,
		.len = length,
		.buf = txdata,
	},
	};

	if (i2c_transfer(m4mo_client->adapter, msg, 1) < 0) {
		printk("m4mo_i2c_txdata faild\n");
		return -EIO;
	}

	return 0;
}

static int32_t m4mo_i2c_write_8bit(unsigned char category, unsigned char byte, unsigned char value)
{
	int32_t rc = -EFAULT;
	char buf[5] = {5, 2, category, byte, value & 0xFF};
	int i;

	PGH_DEBUG("START");

	for(i=0; i<10; i++)
	{
		rc = m4mo_i2c_txdata(0x1F, buf, 5); //1F=3E>>1=M4MO's ADDRESS,  5=>buf's length

		if(rc == 0)
		{
			mdelay(2);
			return rc;
		}

		else
		{
			mdelay(i*2);
			printk("i2c_write failed, category = 0x%x, byte = 0x%x value = 0x!\n", category, byte, value);
		}
	}


	return rc;
}



#if 1//PGH 
int32_t m4mo_i2c_write_8bit_external(unsigned char category, unsigned char byte, unsigned char value)
{
	int32_t rc = -EFAULT;
	char buf[5] = {5, 2, category, byte, value & 0xFF};
	int i;

	PGH_DEBUG("START");

	for(i=0; i<10; i++)
	{
	rc = m4mo_i2c_txdata(0x1F, buf, 5); //1F=3E>>1=M4MO's ADDRESS,  5=>buf's length

		if(rc == 0)
		{
			mdelay(2);
			return rc;
		}

		else
		{
			mdelay(i*2);
			printk("i2c_write failed, category = 0x%x, byte = 0x%x value = 0x!\n", category, byte, value);
		}
	}

	return rc;

}
#endif//PGH 

static int32_t m4mo_i2c_write_16bit(unsigned char category, unsigned char byte, short value)
{
	int32_t rc = -EFAULT;
	char buf[6] = {6, 2, category, byte, 0,0};
	int i;

	PGH_DEBUG("START");
	
	buf[4] = (value & 0xFF00) >>8;
	buf[5] = (value & 0x00FF);
	

	for(i=0; i<10; i++)
	{
	rc = m4mo_i2c_txdata(0x1F, buf, 6); //1F=3E>>1=M4MO's ADDRESS,  6=>buf's length

		if(rc == 0)
		{
			mdelay(2);
			return rc;
		}
		else
		{
			mdelay(i*2);
			printk("i2c_write failed, category = 0x%x, byte = 0x%x value = 0x!\n", category, byte, value);
		}
	}


	mdelay(20);

	return rc;
}


static int32_t m4mo_i2c_write_32bit(unsigned char category, unsigned char byte, int value)
{
	int32_t rc = -EFAULT;
	char buf[8] = {8, 2, category, byte, 0,0,0,0};
	int i;

	PGH_DEBUG("START");
	
	buf[4] = (value & 0xFF000000) >>24;
	buf[5] = (value & 0x00FF0000) >>16;
	buf[6] = (value & 0x0000FF00) >> 8;
	buf[7] = (value & 0x000000FF);

	for(i=0; i<10; i++)
	{
	rc = m4mo_i2c_txdata(0x1F, buf, 8);

		if(rc == 0)
		{
			mdelay(2);
			return rc;
		}
		else
		{
			mdelay(i*2);
			printk("i2c_write failed, category = 0x%x, byte = 0x%x value = 0x!\n", category, byte, value);
		}

	}

	mdelay(20);


	return rc;
}

static int32_t m4mo_i2c_write_memory(int address, short size, char *value)
{
	int32_t rc = -EFAULT;
	char *buf;
	int i;

	PGH_DEBUG("START");

	buf = kmalloc(8 + size, GFP_KERNEL);
	if(buf <0) {
		printk("[PGH] m4mo_i2c_write_memory  memory alloc fail!\n");
		return -1;
	}

	buf[0] = 0x00;
	buf[1] = 0x04;

	buf[2] = (address & 0xFF000000) >> 24;
	buf[3] = (address & 0x00FF0000) >> 16;
	buf[4] = (address & 0x0000FF00) >> 8;
	buf[5] = (address & 0x000000FF) ;

	buf[6] = (size & 0xFF00) >> 8;
	buf[7] = (size & 0x00FF);

	memcpy(buf+8 , value, size);


	for(i=0; i<10; i++)
	{
	rc = m4mo_i2c_txdata(0x1F, buf,  8 + size);

		if(rc == 0)
		{
			mdelay(5);
	kfree(buf);
			return rc;
		}
		
		else
		{
			mdelay(i*2);
			printk("m4mo_i2c_write_memory fail!\n");
		}
	}


	mdelay(20);

	kfree(buf);
	return rc;
}



static int m4mo_i2c_rxdata(unsigned short saddr, unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 5,
		.buf   = rxdata,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(m4mo_client->adapter, msgs, 2) < 0) {
		printk("m4mo_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}


static int32_t m4mo_i2c_read(unsigned short category, unsigned short byte, unsigned short *puData)
{
	int32_t rc = 0;

	unsigned char buf[5];
	int i;

	if (!puData)
		return -EIO;


	memset(buf, 0, sizeof(buf));

	buf[0] = 0x5; //just protocol
	buf[1] = 0x1; //just protocol
	buf[2] = (category & 0xFF);
	buf[3] = (byte & 0xFF);
	buf[4] = 0x1; //read one byte


	for(i=0; i<10; i++)
	{
		rc = m4mo_i2c_rxdata(0x1F, buf, 5);

		if (rc == 0 )
		{
			mdelay(2);
			//PGH	*puData = buf[0] << 8 | buf[1];
			*puData = buf[1];
			return rc;
		}
	
		else
		{
			mdelay(i*2);
			printk("m4mo_i2c_read failed!\n");
		}

	}



	return rc;
}



#if defined(CONFIG_SAMSUNG_TARGET)
int cam_pm_lp8720_i2c_tx_data(char* txData, int length)
{
	int rc; 

	struct i2c_msg msg[] = {
		{
			.addr = cam_pm_lp8720_pclient->addr,  
			.flags = 0,
			.len = length,
			.buf = txData,		
		},
	};
    
	rc = i2c_transfer(cam_pm_lp8720_pclient->adapter, msg, 1);
	if (rc < 0) {
		printk(KERN_ERR "cam_pm_lp8720: cam_pm_lp8720_i2c_tx_data error %d\n", rc);
		return rc;
	}

#if 0
	else {
		int i;
		/* printk(KERN_INFO "mt_i2c_lens_tx_data: af i2c client addr = %x,"
		   " register addr = 0x%02x%02x:\n", slave_addr, txData[0], txData[1]); 
		   */
		for (i = 0; i < length; i++)
			printk("\tdata[%d]: 0x%02x\n", i, txData[i]);
	}
#endif
	return 0;
}

static int cam_pm_lp8720_i2c_write(unsigned char u_addr, unsigned char u_data)
{
	int rc;
	unsigned char buf[2];

	PGH_DEBUG("START");

	buf[0] = u_addr;
	buf[1] = u_data;
    
	rc = cam_pm_lp8720_i2c_tx_data(buf, 2);

	if(rc < 0)
		printk(KERN_ERR "cam_pm_lp8720 : txdata error %d add:0x%02x data:0x%02x\n", rc, u_addr, u_data);

	PGH_DEBUG("SUCCESS");
	
	return rc;	
}


#if 0 //not used
static int cam_pm_lp8720_i2c_rx_data(char* rxData, int length)
{
	int rc;

	struct i2c_msg msgs[] = {
		{
			.addr = cam_pm_lp8720_pclient->addr,
			.flags = 0,      
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = cam_pm_lp8720_pclient->addr,
			.flags = I2C_M_RD|I2C_M_NO_RD_ACK,  //CHECK!!
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(cam_pm_lp8720_pclient->adapter, msgs, 2);
      
	if (rc < 0) {
		printk(KERN_ERR "cam_pm_lp8720: cam_pm_lp8720_i2c_rx_data error %d\n", rc);
		return rc;
	}
      
#if 0
	else {
		int i;
		for (i = 0; i < length; i++)
			printk(KERN_INFO "\tdata[%d]: 0x%02x\n", i, rxData[i]);
	}
#endif

	return 0;
}


static int cam_pm_lp8720_i2c_read(unsigned char u_addr, unsigned char *pu_data)
{
	int rc;
	unsigned char buf;

	buf = u_addr;
	rc = cam_pm_lp8720_i2c_rx_data(&buf, 1);
	if (!rc)
		*pu_data = buf;
	else printk(KERN_ERR "cam_pm_lp8720: i2c read failed\n");
	return rc;	
}
#endif
#endif//PGH FOR CAM PM LP8720 /////////////////////////////////////////////////////////////////








static long m4mo_set_sensor_mode(enum sensor_mode_t mode)
{
	int i=0;
	unsigned short read_value_1=0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:

	PGH_DEBUG("SENSOR_PREVIEW_MODE");

#if 1//PGH
	if(preview_flag == 0)
	{
// CAPTURE->PARAMETER  IS IMPOSSIBLE  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//		m4mo_i2c_write_8bit(0x00, 0x0b, 0x01); //parameter_setting_mode
//		m4mo_i2c_write_8bit(0x01, 0x01, 0x17); //480x320
		

		if( m4mo_i2c_write_8bit(0x00, 0x11, 0x01) < 0 )
			printk("[PGH] Fail YUV INT ENABLE \n");
		
		if( m4mo_i2c_write_8bit(0x00, 0x12, 0x01) < 0 )
			printk("[PGH] Fail ROOT INT ENABLE \n");

		if( m4mo_i2c_write_8bit(0x00, 0x0B, 0x02) < 0 )
			printk("[PGH] Monitor Mode \n");


		for(i=0; i<150; i++)
		{
			mdelay(10);
			if( m4mo_i2c_read(0x00, 0x0C, &read_value_1) < 0 )
				printk("Fail read status! \n");

			if(read_value_1 == 0x02||read_value_1==0x03||read_value_1==0x04) //MONITOR MODE //for Behold2 but just safty for i7500
				break;		

			PGH_DEBUG("in,  waiting for monitor mode : %x ", read_value_1);
		}

		if( m4mo_i2c_read(0x00, 0x10, &read_value_1) < 0 )
			printk("[PGH] Fail INT CLEAR \n");


#if 1//PGH FOR AE & AWB UNLOCK
		printk("=================AE & AWB UNLOCK ============\n");

		if( m4mo_i2c_write_8bit(0x03, 0x00, 0x00) < 0 )
			printk("[PGH] Fail AE UNLOCK \n");

		if( m4mo_i2c_write_8bit(0x06, 0x00, 0x00) < 0 )
			printk("[PGH] Fail AWB UNLOCK\n");


#endif//PGH


		preview_flag = 1;
	}

#endif//PGH

		break;


	case SENSOR_SNAPSHOT_MODE:
    	PGH_DEBUG("SENSOR_SNAPSHOT_MODE START");
		/* Switch to lower fps for Snapshot */		

		preview_flag = 0; //

		break;

	default:
		return -EFAULT;
	}

	return 0;
}


static long m4mo_set_effect(
	enum sensor_mode_t mode,
	int8_t effect
)
{
	long rc = 0;


#if 0//PGH
	uint16_t reg_addr;
	uint16_t reg_val;


	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		reg_addr = 0x2799;
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		reg_addr = 0x279B;
		break;

	default :
		reg_addr = 0x2799;
		break;
	}

	switch ((enum camera_effect_t)effect) {
	case CAMERA_EFFECT_OFF: {
		reg_val = 0x6440;

		rc = m4mo_i2c_write_8bit(M4MO_IMG_I2C_ADDR,
			0x338C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

	}
			break;

case CAMERA_EFFECT_MOSAIC:
	case CAMERA_EFFECT_RESIZE:
	default: {
		reg_val = 0x6440;
		rc = m4mo_i2c_write_8bit(M4MO_IMG_I2C_ADDR,
			0x338C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = m4mo_i2c_write_8bit(M4MO_IMG_I2C_ADDR,
			0x3390, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;

		return -EFAULT;
	}
	}

  /* Refresh Sequencer */
	rc = m4mo_i2c_write_8bit(M4MO_IMG_I2C_ADDR,
		0x338C, 0xA103, WORD_LEN);
	if (rc < 0)
		return rc;

	rc = m4mo_i2c_write_8bit(M4MO_IMG_I2C_ADDR,
		0x3390, 0x0005, WORD_LEN);

#endif//PGH
	return rc;
}


int get_camera_fw_id(void)
{
	int rc = 0;
	unsigned short read_value_1 = 0;
	unsigned short read_value_2 = 0;
	int firmware_version;

	
	if( m4mo_i2c_read(0x00, 0x01, &read_value_1) < 0 )
	{
		printk("[PGH] Fail read FW upper\n");
		return -1;
	}
	PGH_DEBUG("M4MO FW upper : %x", read_value_1);

	
	if( m4mo_i2c_read(0x00, 0x02, &read_value_2) < 0 )
	{
		printk("[PGH] Fail read FW lower\n");
		return -1;
	}
	PGH_DEBUG("M4MO FW lower : %x", read_value_2);

	firmware_version = (read_value_1*0x100)|(read_value_2);
	printk("F/W version : %x \n", firmware_version);


	return rc;
}



static int m4mo_sensor_init_probe(struct msm_camera_sensor_info *data)
{
	int rc = 0;

	unsigned short read_value_1 = 0;
	int i; //for loop
	



	CDBG("init entry \n");

	PGH_DEBUG("//////////////////////////////////////////////////");
	PGH_DEBUG("sensor_init start");
	PGH_DEBUG("//////////////////////////////////////////////////");


	if( gpio_direction_output(CAM_RST, LOW) < 0 )
	{
		PGH_DEBUG("Fail RST => LOW");
		return -1;
	}
#if 0//unuse standby pin
	if( gpio_direction_output(CAM_STB, LOW) < 0 )
	{
		PGH_DEBUG("CAM_STB => LOW");
		return -1;
	}
#endif
	PGH_DEBUG("CAM_PM_LP8720 INIT FOR CAPELA'S 5M CAM");

	if( gpio_direction_output(CAM_ON, LOW) < 0 )
	{
		PGH_DEBUG("Fail CAM_ON => LOW");
		return -1;
	}
	mdelay(2);

#if 0//PGH NO DELAY VERSION
	cam_pm_lp8720_i2c_write(0x01, 0x1F); // LDO1 :3.3V, no delay
	cam_pm_lp8720_i2c_write(0x02, 0x19); // LDO2 :2.8V, no delay
	cam_pm_lp8720_i2c_write(0x03, 0x0C); // LDO3 :1.8V, no delay
	// M4MO does not use a LDO4's power
	cam_pm_lp8720_i2c_write(0x05, 0x19); // LDO5 :2.8V, no delay
	cam_pm_lp8720_i2c_write(0x06, 0x09); // BUCKS1:1.2V, no delay
	cam_pm_lp8720_i2c_write(0x07, 0x09); // BUCKS2:1.2V, no delay
	cam_pm_lp8720_i2c_write(0x08, 0xB7); // Enable all power without LDO4
#else //PGH 1.2V FIRST
    if( cam_pm_lp8720_i2c_write(0x01, 0x7F) < 0 )
    {
		PGH_DEBUG("Fail LDO1 :3 delays 3.3V  011 1 1111  == DF ");
		return -1;
    }
#endif//PGH

	if( gpio_direction_output(CAM_ON, HIGH) < 0 )
	{
		PGH_DEBUG("Fail CAM_ON => HIGH");
		return -1;
	}
	mdelay(5);

#if 1//Mclk_timing for M4Mo spec.
	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
#endif	

	PGH_DEBUG("START MCLK:24Mhz~~~~~");
	msm_camio_clk_rate_set(24000000);
	mdelay(5);
	msm_camio_camif_pad_reg_reset();
	mdelay(10);

	if( gpio_direction_output(CAM_RST, HIGH) < 0 )
	{
		PGH_DEBUG("Fail RST => HIGH");
		return -1;
	}
	mdelay(30); // min 350ns


//PGH_TEMP		mdelay(100); // from RoxyRL_M4MO_090128_Command_List.xls

	if( m4mo_i2c_write_8bit(0x0F, 0x12, 0x01) < 0 )
	{
		PGH_DEBUG("Fail Camera mode start");
		return -1;
	}
	
	mdelay(150); // from RoxyRL_M4MO_090128_Command_List.xls
//		mdelay(50); // from RoxyRL_M4MO_090128_Command_List.xls


	rc = get_camera_fw_id();
	if(rc < 0)
	{
		PGH_DEBUG("Fail get_camera_fw_id fail!");
		return -1;
	}


	if( m4mo_i2c_write_8bit(0x00, 0x0b, 0x01) < 0 )
	{
		PGH_DEBUG("Fail parameter_setting_mode	!");
		return -1;
	}
	
#if 1//PGH ADDED FOR FLICKER(Manual 50Hz for Europe)
	if( m4mo_i2c_write_8bit(0x03, 0x06, 0x01) < 0 )
	{
		PGH_DEBUG("Fail set a Manual Flicker 50Hz");
		return -1;
	}
#endif//PGH


#if 1//PGH FOR TEST CAPTURE	
		PGH_DEBUG("136 msec delay, 1 times");
		m4mo_i2c_write_8bit(0x0C, 0x0B, 0x24); // 0x24 * 4 msec delay
//		m4mo_i2c_write_8bit(0x0D, 0x1B, 0x01); // 1 == 2times
#endif//PGH


#if 1
	if( m4mo_i2c_write_8bit(0x01, 0x01, 0x0D) < 0 )
	{
		PGH_DEBUG("Fail set dimension 640 x 480");
		return -1;
	}
#else
	if( m4mo_i2c_write_8bit(0x01, 0x01, 0x17) < 0 )
	{
		PGH_DEBUG("Fail set dimension 480 x 320");
		return -1;
	}
#endif		

	if( m4mo_i2c_write_8bit(0x01, 0x18, 0x08) < 0 )
	{
		PGH_DEBUG("Fail 0x08XX == 2048");
		return -1;
	}

	if( m4mo_i2c_write_8bit(0x01, 0x19, 0x00) < 0 )
	{
		PGH_DEBUG("Fail 0xXX00 == 2048");
		return -1;
	}


	if( m4mo_i2c_write_8bit(0x00, 0x11, 0x01) < 0 )
	{
		PGH_DEBUG("Fail YUV INT ENABLE!");
		return -1;
	}
	
	if( m4mo_i2c_write_8bit(0x00, 0x12, 0x01) < 0 )
	{
		PGH_DEBUG("Fail ROOT INT ENABLE!");
		return -1;
	}

	if( m4mo_i2c_write_8bit(0x00, 0x0B, 0x02) < 0 )
	{
		PGH_DEBUG("Fail Change to Monitor Mode");
		return -1;
	}

	for(i=0; i<150; i++)
	{
		mdelay(10);
		if( m4mo_i2c_read(0x00, 0x0C, &read_value_1) < 0 )
		{
			PGH_DEBUG("Fail read sensor_init status!");
			return -1;
		}
		
		if(read_value_1 == 0x02) //MONITOR MODE
			break;		

		PGH_DEBUG("waiting for monitor mode : %x ", read_value_1);
	}


	if( m4mo_i2c_read(0x00, 0x10, &read_value_1) < 0 )
	{
		PGH_DEBUG("Fail CLERA INT");
		return -1;
	}

	preview_flag = 1;


#if 0//PGH I2C SPEED TEST
	before_time = get_jiffies_64();
    for (i = 0; i < 3000; i++) 
	{
        m4mo_i2c_write_8bit((M4MO_IMG_I2C_ADDR), s5k4ca_init1[1].subaddr, s5k4ca_init1[1].value, WORD_LEN);
	}	

	after_time = get_jiffies_64();
	printk("[PGH] Total Time 3000: %d\n",  jiffies_to_msecs(after_time-before_time));
#endif//PGH I2C SPEED TEST


	if (rc < 0)
		goto init_probe_fail;



#if 0//PGH
	/* Check if it matches it with the value in Datasheet */
	if (model_id != MT9D112_MODEL_ID) {
		rc = -EFAULT;
		goto init_probe_fail;
	}

	rc = m4mo_reg_init();
	if (rc < 0)
		goto init_probe_fail;
#endif//PGH
	return rc;

init_probe_fail:
	return rc;
}




int m4mo_sensor_init(struct msm_camera_sensor_info *data)
{
	int rc = 0;

	printk("[PGH] %s 1111111\n", __func__);
	m4mo_ctrl = kzalloc(sizeof(struct m4mo_ctrl_t), GFP_KERNEL);
	if (!m4mo_ctrl) {
		CDBG("m4mo_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		m4mo_ctrl->sensordata = data;

#if 0//PGH
	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	msm_camio_camif_pad_reg_reset();
#endif//PGH

	printk("[PGH] %s 222222\n", __func__);
  rc = m4mo_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("m4mo_sensor_init failed!\n");
		goto init_fail;
	}

	printk("[PGH] %s 3333  rc:%d\n", __func__, rc);
init_done:
	return rc;

init_fail:
	kfree(m4mo_ctrl);
	return rc;
}

static int m4mo_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&m4mo_wait_queue);
	return 0;
}


int m4mo_sensor_config(void __user *argp)
{
	struct sensor_cfg_data_t cfg_data;
	long   rc = 0;


#if 1//PGH
	if (copy_from_user(
				&cfg_data,
				(void *)argp,
				sizeof(struct sensor_cfg_data_t)))
		return -EFAULT;

	/* down(&m4mo_sem); */

	CDBG("m4mo_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = m4mo_set_sensor_mode(
					cfg_data.mode);
		break;

	case CFG_SET_EFFECT:
		rc = m4mo_set_effect(
					cfg_data.mode,
					cfg_data.cfg.effect);
		break;

	default:
		rc = -EFAULT;
		break;
	}

	/* up(&m4mo_sem); */
#endif//PGH

	return rc;
}

int m4mo_sensor_release(void)
{
	int rc = 0;

	/* down(&m4mo_sem); */

	PGH_DEBUG("POWER OFF");
	printk("camera turn off\n");
//unuse standby pin	gpio_direction_output(CAM_STB, LOW);


	kfree(m4mo_ctrl);
	
	/* up(&m4mo_sem); */

	return rc;
}






#if defined(CONFIG_SAMSUNG_TARGET)
static int cam_pm_lp8720_init_client(struct i2c_client *client)
{
	/* Initialize the cam_pm_lp8720 Chip */
	init_waitqueue_head(&cam_pm_lp8720_wait_queue);
	return 0;
}
#endif 




//PGH:KERNEL2.6.25static int m4mo_probe(struct i2c_client *client)
static int m4mo_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;

	PGH_DEBUG("START\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	m4mo_sensorw =
		kzalloc(sizeof(struct m4mo_work_t), GFP_KERNEL);

	if (!m4mo_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, m4mo_sensorw);
	m4mo_init_client(client);
	m4mo_client = client;

	CDBG("m4mo_probe successed!\n");
	PGH_DEBUG("SUCCESS\n");


	return 0;

probe_failure:
	kfree(m4mo_sensorw);
	m4mo_sensorw = NULL;
	CDBG("m4mo_probe failed!\n");
	PGH_DEBUG("m4mo_probe failed!\n");
	return rc;
}


static int __exit m4mo_remove(struct i2c_client *client)
{

	struct m4mo_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	i2c_detach_client(client);
	m4mo_client = NULL;
	m4mo_sensorw = NULL;
	kfree(sensorw);
	return 0;

}


static const struct i2c_device_id m4mo_id[] = {
    { "m4mo", 0 },
    { }
};

//PGH MODULE_DEVICE_TABLE(i2c, m4mo);

static struct i2c_driver m4mo_driver = {
	.id_table	= m4mo_id,
	.probe  	= m4mo_probe,
	.remove 	= __exit_p(m4mo_remove),
	.driver 	= {
		.name = "m4mo",
	},
};



#if defined(CONFIG_SAMSUNG_TARGET)
static int cam_pm_lp8720_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cam_pm_lp8720_data *mt;
	int err = 0;

	PGH_DEBUG("START......\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENOTSUPP;
		goto cam_pm_lp8720_probe_failure;
	}

	if(!(mt = kzalloc( sizeof(struct cam_pm_lp8720_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto cam_pm_lp8720_alloc_data_failed;
	}
	
	i2c_set_clientdata(client, mt);
	cam_pm_lp8720_init_client(client);
	cam_pm_lp8720_pclient = client;	

	return 0;

cam_pm_lp8720_probe_failure:
cam_pm_lp8720_alloc_data_failed:
	return err;
	
}
#endif



#if defined(CONFIG_SAMSUNG_TARGET)
static int cam_pm_lp8720_remove(struct i2c_client *client)
{
	struct cam_pm_lp8720_data *mt = i2c_get_clientdata(client);

// free_irq(client->irq, mt);
	i2c_detach_client(client);
	cam_pm_lp8720_pclient = NULL;
//	misc_deregister(&m4mo_device);
	kfree(mt);
	return 0;
}
#endif//PGH FOR CAM PMIC ////////////////////////////////////////


#if defined(CONFIG_SAMSUNG_TARGET)

static const struct i2c_device_id cam_pm_lp8720_id[] = {
    { "cam_pm_lp8720_i2c", 0 },
    { }
};

//PGH MODULE_DEVICE_TABLE(i2c, cam_pm_lp8720_id);


static struct i2c_driver cam_pm_lp8720_driver = {
	.id_table 	= cam_pm_lp8720_id,
	.probe  	= cam_pm_lp8720_probe,
	.remove 	= cam_pm_lp8720_remove,
	.driver 	= {
		.name = "cam_pm_lp8720_i2c",
	},
};
#endif//PGH FOR CAM PM LP8720 ////////////////////////////////////////







int32_t m4mo_init(void)
{
	int32_t rc = 0;


	rc = i2c_add_driver(&m4mo_driver);

	if (IS_ERR_VALUE(rc))
		goto init_failure;

#if defined(CONFIG_SAMSUNG_TARGET)
	rc = i2c_add_driver(&cam_pm_lp8720_driver);

	if (IS_ERR_VALUE(rc))
		goto init_failure_cam_pm;
#endif


	return rc;



init_failure:
	CDBG("m4mo_init, rc = %d\n", rc);
	return rc;

init_failure_cam_pm:
	CDBG("cam_pm_init, rc = %d\n", rc);
	return rc;
}


void m4mo_exit(void)
{
	i2c_del_driver(&m4mo_driver);
//PGH CHECK 	i2c_del_driver(&cam_pm_lp8720_driver);
	
}


int m4mo_probe_init(void *dev, void *ctrl)
{
	int rc = 0;
/*	struct msm_camera_sensor_info *info =
		(struct msm_camera_sensor_info *)dev; */

	struct msm_sensor_ctrl_t *s =
		(struct msm_sensor_ctrl_t *)ctrl;


#if 1//PGH
	PGH_DEBUG("START 5M SWI2C VER0.2 : %d", rc);

	old_hw = gpio_get_value(99);
	PGH_DEBUG("OLD HW : %d\n", old_hw);


	gpio_direction_output(CAM_FLASH_EN, LOW); //CAM_FLASH_EN
	gpio_direction_output(CAM_FLASH_SET, LOW); //CAM_FLASH_SET
#endif//PGH

	rc = m4mo_init();
	if (rc < 0)
		goto probe_done;

	/* Input MCLK = 24MHz */
#if 0//PGH
	msm_camio_clk_rate_set(24000000);
	mdelay(5);
#endif//PGH

#if 0
	rc = m4mo_sensor_init_probe(info);
	if (rc < 0)
		goto probe_done;
#endif
	s->s_init		= m4mo_sensor_init;
	s->s_release	= m4mo_sensor_release;
	s->s_config	= m4mo_sensor_config;

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
	
}



int msgProcess_Flash(int codeB, int codeC)
{
	int i;
	
	PGH_DEBUG("msgProcess_Flash Torch");

	gpio_direction_output(CAM_FLASH_SET, LOW); //CAM_FLASH_SET	MOVIE			
	gpio_direction_output(CAM_FLASH_EN, LOW); //CAM_FLASH_EN CAMERA


#if 1//PGH
	switch(codeB)
	{
		case FLASH_CAMERA :
		{			
			if(codeC == FLASH_CMD_ON)
			{
				PGH_DEBUG("FLASH_CAMERA ON");
				gpio_direction_output(CAM_FLASH_EN, HIGH); //CAM_FLASH_EN
			}
			else //OFF
			{
				PGH_DEBUG("FLASH_CAMERA OFF");
				gpio_direction_output(CAM_FLASH_EN, LOW); //CAM_FLASH_EN
			}
		}
			break;


		case FLASH_MOVIE :
		{
			if(codeC == FLASH_CMD_ON)
			{
				PGH_DEBUG("FLASH_MOVIE ON");
				
				for(i=0; i<7; i++)
				{
					gpio_direction_output(CAM_FLASH_SET, LOW);  //CAM_FLASH_SET					
					udelay(1);
					gpio_direction_output(CAM_FLASH_SET, HIGH);  //CAM_FLASH_SET					
					udelay(1);
				}
			}	
			else
			{
				PGH_DEBUG("FLASH_MOVIE OFF");

				gpio_direction_output(CAM_FLASH_EN, LOW); //CAM_FLASH_EN
				gpio_direction_output(CAM_FLASH_SET, LOW); //CAM_FLASH_SET					
			}

		}
			break;
					
		default :
			PGH_DEBUG("unknown msgProcess_Flash CMD");
			break;

	}

#endif//PGH

	return 0;
}








int m4mo_i2c_read_8bit_from_apps(void __user *arg)
{
	int 				rc;
	unsigned short	read_value;	
	ioctl_m4mo_info_8bit		ctrl_info;


//	PGH_DEBUG("START");

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
		return -EFAULT;



	rc = m4mo_i2c_read(ctrl_info.category, ctrl_info.byte, &read_value);

	if(rc <0)
	{
		PGH_DEBUG("pgh_read fail in pgh_read");
		return -1;
	}	
//	else
//		PGH_DEBUG("read_value : %x\n", read_value);


	ctrl_info.value = read_value;
	if(copy_to_user((void *)arg, (const void *) &ctrl_info, sizeof(ctrl_info)))
		return -EFAULT;

	return 0;
}


int m4mo_memory_read(void __user *arg)
{
/*
	int 				read_value, rc;
	ioctl_m4mo_info		ctrl_info;


	//printk("[PGH] pgh_word_read in kernel~~~~~~~~~~~~~~~~~~~~~\n");

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
		return -EFAULT;
	//printk("[PGH] pgh_word_read From user address : %x\n", ctrl_info.address);


	rc = m4mo_i2c_word_read(M4MO_IMG_I2C_ADDR, ctrl_info.address, &read_value, WORD_LEN);
	if(rc <0)
		printk("[PGH] pgh_word_read fail in pgh_read\n");
	//else
	//	printk("[PGH] pgh_word_read read_value :%x\n", read_value);


	ctrl_info.value = read_value;
	copy_to_user((void *)arg, (const void *) &ctrl_info, sizeof(ctrl_info));

*/
	return 0;
}


int m4mo_i2c_write_8bit_from_apps(void __user *arg)
{
	int 				rc;
	ioctl_m4mo_info_8bit		ctrl_info;

//	PGH_DEBUG("START");

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
		return -EFAULT;

	rc = m4mo_i2c_write_8bit(ctrl_info.category, ctrl_info.byte, ctrl_info.value); 
	
	if (rc < 0)
	{
		PGH_DEBUG("m4mo_i2c_write_8bit_from_apps fail");
		return -1;
	}

	return 0;
}


int m4mo_i2c_write_16bit_from_apps(void __user *arg)
{

	int 				rc;
	ioctl_m4mo_info		ctrl_info;

	PGH_DEBUG("m4mo_i2c_write_16bit_from_apps in kernel");

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
		return -EFAULT;

	rc = m4mo_i2c_write_16bit(ctrl_info.category, ctrl_info.byte, ctrl_info.value); 
	
	if (rc < 0)
	{
		PGH_DEBUG("m4m_i2c_write_16bit_from_apps fail");
		return -1;
	}

	return 0;
}


int m4mo_i2c_write_32bit_from_apps(void __user *arg)
{
	int 				rc;
	ioctl_m4mo_info		ctrl_info;

	PGH_DEBUG("m4mo_i2c_write_32bit_from_apps in kernel");

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
		return -EFAULT;

	rc = m4mo_i2c_write_32bit(ctrl_info.category, ctrl_info.byte, ctrl_info.value); 
	
	if (rc < 0)
	{
		PGH_DEBUG("m4m_i2c_write_32bit_from_apps fail");
		return -1;
	}

	return 0;
}



int m4mo_i2c_write_category_parameter_from_apps(void __user *arg)
{
	return 0;
}


int m4mo_i2c_write_memory_from_apps(void __user *arg)
{
	int 							rc;
	ioctl_m4mo_i2c_memory_info		ctrl_info;

	PGH_DEBUG("m4mo_i2c_write_memory_from_apps in kernel");

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
		return -EFAULT;

#if 0//PGH DEBUG
	printk("[PGH] m4mo_i2c_write_memory_from_apps address: %x\n", (ctrl_info.address));
	printk("[PGH] write_memory_from_apps size: %x\n", (ctrl_info.size));
	printk("[PGH] write_memory_from_apps ctrl_info.value: %x\n", (ctrl_info.value));
	printk("[PGH] write_memory_from_apps *(ctrl_info.value): %x\n", *(ctrl_info.value));
	printk("[PGH] write_memory_from_apps *(ctrl_info.(value+1)): %x\n", *(ctrl_info.value+1)   );
	printk("[PGH] write_memory_from_apps *(ctrl_info.(value+2)): %x\n", *(ctrl_info.value+2)   );
	printk("[PGH] write_memory_from_apps *(ctrl_info.(value+3)): %x\n", *(ctrl_info.value+3)   );
	printk("[PGH] write_memory_from_apps *(ctrl_info.(value+4)): %x\n", *(ctrl_info.value+4)   );
	printk("[PGH] write_memory_from_apps *(ctrl_info.value+5): %x\n", *(ctrl_info.value+5));
	printk("[PGH] write_memory_from_apps *(ctrl_info.value+6): %x\n", *(ctrl_info.value+6));
	printk("[PGH] write_memory_from_apps *(ctrl_info.value+7): %x\n", *(ctrl_info.value+7));
	printk("[PGH] write_memory_from_apps *(ctrl_info.value+8): %x\n", *(ctrl_info.value+8));

	printk("[PGH] m4mo_i2c_write_memory_from_apps ctrl_info.value[0]: %x\n", ctrl_info.value[0]);
	printk("[PGH] m4mo_i2c_write_memory_from_apps ctrl_info.value[1]: %x\n", ctrl_info.value[1]);
	printk("[PGH] m4mo_i2c_write_memory_from_apps ctrl_info.value[2]: %x\n", ctrl_info.value[2]);
	printk("[PGH] m4mo_i2c_write_memory_from_apps ctrl_info.value[3]: %x\n", ctrl_info.value[3]);
	printk("[PGH] m4mo_i2c_write_memory_from_apps ctrl_info.value[4]: %x\n", ctrl_info.value[4]);
	printk("[PGH] m4mo_i2c_write_memory_from_apps ctrl_info.value[5]: %x\n", ctrl_info.value[5]);
	printk("[PGH] m4mo_i2c_write_memory_from_apps ctrl_info.value[6]: %x\n", ctrl_info.value[6]);
	printk("[PGH] m4mo_i2c_write_memory_from_apps ctrl_info.value[7]: %x\n", ctrl_info.value[7]);

	printk("[PGH] write_memory_from_apps ctrl_info.value[4091]: %x\n", ctrl_info.value[4091]);
	printk("[PGH] write_memory_from_apps ctrl_info.value[4092]: %x\n", ctrl_info.value[4092]);
	printk("[PGH] write_memory_from_apps ctrl_info.value[4093]: %x\n", ctrl_info.value[4093]);
	printk("[PGH] write_memory_from_apps ctrl_info.value[4094]: %x\n", ctrl_info.value[4094]);
	printk("[PGH] write_memory_from_apps ctrl_info.value[4095]: %x\n", ctrl_info.value[4095]);
	printk("[PGH] write_memory_from_apps ctrl_info.value[4096]: %x\n", ctrl_info.value[4096]);

#endif//PGH DEBUG

	rc = m4mo_i2c_write_memory(ctrl_info.address, ctrl_info.size, ctrl_info.value); 
	if (rc < 0)
	{
		PGH_DEBUG("m4m_i2c_write_memory_from_apps fail");
		return -1;
	}

	return 0;
}




int pgh_command(void __user *arg)
{
/*
	int cam_status, rc, size;

	int zoom;

	ioctl_m4mo_info		ctrl_info;

	PGH_DEBUG("START");


	switch(ctrl_info.pgh_magic) 
	{
		case 1: //VA_CheckReadyForCmd()
			VA_CheckReadyForCmd();
			break;


		default:
			printk("[PGH] pgh_command is working without purpose!\n");
			return -1;
	}
*/
	return 0;
}



int pgh_msg(void __user *arg)
{
	//int rc, size; 
	int	result;
	ioctl_msg_info		ctrl_info;

	PGH_DEBUG("START");


	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
		return -EFAULT;


	switch(ctrl_info.codeA)
	{

		case FLASH_CMD:
			{
				printk("CodeA : FLASH_MODE" ); 
				result = msgProcess_Flash(ctrl_info.codeB, ctrl_info.codeC);                
			}
			break;

		case RESET_FOR_FW:
		    {    
		        PGH_DEBUG("~~~~RESET FOR FW HIGH->LOW->HIGH");
		        gpio_direction_output(CAM_RST, LOW);
		        mdelay(10);
		        gpio_direction_output(CAM_RST, HIGH);
		    }     
		    break;


		default :
			PGH_DEBUG("unknown command from pgh_msg");
			break;
		
	}// switch


	return 0;
}//end of pgh_msg





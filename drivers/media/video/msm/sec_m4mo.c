/*
  SEC M4MO
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <asm/gpio.h> 
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/board.h>
#include "sec_m4mo.h"



#define CAM_RST						17
#define CAM_ON						76
#define CAM_FLASH_EN				23
#define CAM_FLASH_SET				31



#define LOW							0
#define HIGH						1
#define M4MO_IMG_I2C_ADDR			0x3F



static int preview_flag = 0;
static int old_hw = 0;


#if defined(CONFIG_SAMSUNG_CAPELA)
static struct i2c_client *cam_pm_lp8720_pclient; 

struct cam_pm_lp8720_data {
	struct work_struct work;
};

static DECLARE_WAIT_QUEUE_HEAD(cam_pm_lp8720_wait_queue);
DECLARE_MUTEX(cam_pm_lp8720_sem);

#endif


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
		printk("m4mo_i2c_write_memory  memory alloc fail!\n");
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



#if defined(CONFIG_SAMSUNG_CAPELA)
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

#endif








static long m4mo_set_sensor_mode(enum sensor_mode_t mode)
{
	int i=0;
	unsigned short read_value_1=0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		
	PGH_DEBUG("SENSOR_PREVIEW_MODE");

#if 1
	if(preview_flag == 0)
	{

		if( m4mo_i2c_write_8bit(0x00, 0x11, 0x01) < 0 )
			printk("Fail YUV INT ENABLE \n");
		
		if( m4mo_i2c_write_8bit(0x00, 0x12, 0x01) < 0 )
			printk("Fail ROOT INT ENABLE \n");

		if( m4mo_i2c_write_8bit(0x00, 0x0B, 0x02) < 0 )
			printk("Monitor Mode \n");


		for(i=0; i<150; i++)
		{
			mdelay(10);
			if( m4mo_i2c_read(0x00, 0x0C, &read_value_1) < 0 )
				printk("Fail read status! \n");

			if(read_value_1 == 0x02) //MONITOR MODE
				break;		

			PGH_DEBUG("in,  waiting for monitor mode : %x ", read_value_1);
		}

		if( m4mo_i2c_read(0x00, 0x10, &read_value_1) < 0 )
			printk("Fail INT CLEAR \n");


	printk("=================AE & AWB UNLOCK ============\n");

	if( m4mo_i2c_write_8bit(0x03, 0x00, 0x00) < 0 )
		printk("Fail AE UNLOCK \n");

	if( m4mo_i2c_write_8bit(0x06, 0x00, 0x00) < 0 )
		printk("Fail AWB UNLOCK\n");


		preview_flag = 1;
	}

#endif

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
		printk("Fail read FW upper\n");
		return -1;
	}
	PGH_DEBUG("M4MO FW upper : %x", read_value_1);

	
	if( m4mo_i2c_read(0x00, 0x02, &read_value_2) < 0 )
	{
		printk("Fail read FW lower\n");
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
	PGH_DEBUG("CAM_PM_LP8720 INIT FOR CAPELA'S 5M CAM");

	if( gpio_direction_output(CAM_ON, LOW) < 0 )
	{
		PGH_DEBUG("Fail CAM_ON => LOW");
		return -1;
	}
	mdelay(2);

    if( cam_pm_lp8720_i2c_write(0x01, 0x7F) < 0 )
    {
		PGH_DEBUG("Fail LDO1 :3 delays 3.3V  011 1 1111  == DF ");
		return -1;
    }

	if( gpio_direction_output(CAM_ON, HIGH) < 0 )
	{
		PGH_DEBUG("Fail CAM_ON => HIGH");
		return -1;
	}
	mdelay(5);

	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	
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
	mdelay(30);



	if( m4mo_i2c_write_8bit(0x0F, 0x12, 0x01) < 0 )
	{
		PGH_DEBUG("Fail Camera mode start");
		return -1;
	}
	
	mdelay(150);


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
	
	if( m4mo_i2c_write_8bit(0x03, 0x06, 0x01) < 0 )
	{
		PGH_DEBUG("Fail set a Manual Flicker 50Hz");
		return -1;
	}


		PGH_DEBUG("136 msec delay, 1 times");
		m4mo_i2c_write_8bit(0x0C, 0x0B, 0x22); 
		m4mo_i2c_write_8bit(0x0D, 0x1B, 0x01);


	if( m4mo_i2c_write_8bit(0x01, 0x01, 0x17) < 0 )
	{
		PGH_DEBUG("Fail set dimension 480 x 320");
		return -1;
	}
		

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

	if (rc < 0)
		goto init_probe_fail;



	return rc;

init_probe_fail:
	return rc;
}




int m4mo_sensor_init(struct msm_camera_sensor_info *data)
{
	int rc = 0;

	m4mo_ctrl = kzalloc(sizeof(struct m4mo_ctrl_t), GFP_KERNEL);
	if (!m4mo_ctrl) {
		CDBG("m4mo_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		m4mo_ctrl->sensordata = data;


  rc = m4mo_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("m4mo_sensor_init failed!\n");
		goto init_fail;
	}

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

	return rc;
}

int m4mo_sensor_release(void)
{
	int rc = 0;

	/* down(&m4mo_sem); */

	PGH_DEBUG("POWER OFF");
	printk("camera turn off\n");


	kfree(m4mo_ctrl);
	
	/* up(&m4mo_sem); */

	return rc;
}






#if defined(CONFIG_SAMSUNG_CAPELA)
static int cam_pm_lp8720_init_client(struct i2c_client *client)
{
	init_waitqueue_head(&cam_pm_lp8720_wait_queue);
	return 0;
}
#endif




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


static struct i2c_driver m4mo_driver = {
	.id_table	= m4mo_id,
	.probe  	= m4mo_probe,
	.remove 	= __exit_p(m4mo_remove),
	.driver 	= {
		.name = "m4mo",
	},
};



#if defined(CONFIG_SAMSUNG_CAPELA)
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



#if defined(CONFIG_SAMSUNG_CAPELA)
static int cam_pm_lp8720_remove(struct i2c_client *client)
{
	struct cam_pm_lp8720_data *mt = i2c_get_clientdata(client);

	i2c_detach_client(client);
	cam_pm_lp8720_pclient = NULL;
	kfree(mt);
	return 0;
}
#endif


#if defined(CONFIG_SAMSUNG_CAPELA)

static const struct i2c_device_id cam_pm_lp8720_id[] = {
    { "cam_pm_lp8720_i2c", 0 },
    { }
};


static struct i2c_driver cam_pm_lp8720_driver = {
	.id_table 	= cam_pm_lp8720_id,
	.probe  	= cam_pm_lp8720_probe,
	.remove 	= cam_pm_lp8720_remove,
	.driver 	= {
		.name = "cam_pm_lp8720_i2c",
	},
};
#endif







int32_t m4mo_init(void)
{
	int32_t rc = 0;


	rc = i2c_add_driver(&m4mo_driver);

	if (IS_ERR_VALUE(rc))
		goto init_failure;

#if defined(CONFIG_SAMSUNG_CAPELA)
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
	
}


int m4mo_probe_init(void *dev, void *ctrl)
{
	int rc = 0;
/*	struct msm_camera_sensor_info *info =
		(struct msm_camera_sensor_info *)dev; */

	struct msm_sensor_ctrl_t *s =
		(struct msm_sensor_ctrl_t *)ctrl;


	PGH_DEBUG("START 5M SWI2C VER0.2 : %d", rc);

	old_hw = gpio_get_value(99);
	PGH_DEBUG("OLD HW : %d\n", old_hw);


	gpio_direction_output(CAM_FLASH_EN, LOW); //CAM_FLASH_EN
	gpio_direction_output(CAM_FLASH_SET, LOW); //CAM_FLASH_SET

	rc = m4mo_init();
	if (rc < 0)
		goto probe_done;


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

	gpio_direction_output(CAM_FLASH_SET, LOW); 		
	gpio_direction_output(CAM_FLASH_EN, LOW); 


	switch(codeB)
	{
		case FLASH_CAMERA :
		{			
			if(codeC == FLASH_CMD_ON)
			{
				PGH_DEBUG("FLASH_CAMERA ON");
				gpio_direction_output(CAM_FLASH_EN, HIGH); 
			}
			else //OFF
			{
				PGH_DEBUG("FLASH_CAMERA OFF");
				gpio_direction_output(CAM_FLASH_EN, LOW); 
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
					gpio_direction_output(CAM_FLASH_SET, LOW); 
					udelay(1);
					gpio_direction_output(CAM_FLASH_SET, HIGH); 
					udelay(1);
				}
			}	
			else
			{
				PGH_DEBUG("FLASH_MOVIE OFF");

				gpio_direction_output(CAM_FLASH_EN, LOW); 
				gpio_direction_output(CAM_FLASH_SET, LOW); 
			}

		}
			break;
					
		default :
			PGH_DEBUG("unknown msgProcess_Flash CMD");
			break;

	}


	return 0;
}








int m4mo_i2c_read_8bit_from_apps(void __user *arg)
{
	int 				rc;
	unsigned short	read_value;	
	ioctl_m4mo_info_8bit		ctrl_info;



	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
		return -EFAULT;



	rc = m4mo_i2c_read(ctrl_info.category, ctrl_info.byte, &read_value);

	if(rc <0)
	{
		PGH_DEBUG("pgh_read fail in pgh_read");
		return -1;
	}	

	ctrl_info.value = read_value;
	if(copy_to_user((void *)arg, (const void *) &ctrl_info, sizeof(ctrl_info)))
		return -EFAULT;

	return 0;
}


int m4mo_memory_read(void __user *arg)
{
	return 0;
}


int m4mo_i2c_write_8bit_from_apps(void __user *arg)
{
	int 				rc;
	ioctl_m4mo_info_8bit		ctrl_info;


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
	int	result = 0;
	ioctl_msg_info		ctrl_info;


	ctrl_info.value = old_hw;


	if(copy_to_user((void *)arg, (const void *)&ctrl_info,  sizeof(ctrl_info)))
		return -EFAULT;



	return 0;
}



int pgh_msg(void __user *arg)
{
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
		
	}


	return 0;
}





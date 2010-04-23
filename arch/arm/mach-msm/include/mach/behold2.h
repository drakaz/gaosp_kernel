#ifndef ASM_MACH_BEHOLD2_H
#define ASM_MACH_BEHOLD2_H

#include <linux/types.h>
#include <linux/list.h>
#include <asm/setup.h>
#include <linux/init.h>

struct msm_pmem_setting{
	resource_size_t pmem_start;
	resource_size_t pmem_size;
	resource_size_t pmem_adsp_start;
	resource_size_t pmem_adsp_size;
	resource_size_t pmem_gpu0_start;
	resource_size_t pmem_gpu0_size;
	resource_size_t pmem_gpu1_start;
	resource_size_t pmem_gpu1_size;
	resource_size_t pmem_camera_start;
	resource_size_t pmem_camera_size;
	resource_size_t ram_console_start;
	resource_size_t ram_console_size;
};

enum {
	MSM_SERIAL_UART1	= 0,
	MSM_SERIAL_UART2,
	MSM_SERIAL_UART3,
#ifdef CONFIG_SERIAL_MSM_HS
	MSM_SERIAL_UART1DM,
	MSM_SERIAL_UART2DM,
#endif
	MSM_SERIAL_NUM,
};


/* common init routines for use by arch/arm/mach-msm/board-*.c */
void __init msm_add_usb_devices(void (*phy_reset) (void));
void __init msm_add_mem_devices(struct msm_pmem_setting *setting);
void __init msm_init_pmic_vibrator(void);

struct mmc_platform_data;
void notify_usb_connected(int online);



/** GPIO definitions for Behold2 board ***/
#ifdef CONFIG_MACH_BEHOLD2_REV01
#define T_FLASH_DETECT		20

// Related to BT/WLAN
#define GPIO_WLAN_BT_REG_ON	85
#define GPIO_WLAN_HOST_WAKE	28
//#define BCM4325_WLAN_WAKE	28

#define BCM4325_BT_RESET	109

#define GPIO_BT_UART_RTS	43
#define GPIO_BT_UART_CTS	44
#define GPIO_BT_UART_RXD	45
#define GPIO_BT_UART_TXD	46

#define GPIO_BT_PCM_DOUT	68
#define GPIO_BT_PCM_DIN		69
#define GPIO_BT_PCM_SYNC	70
#define GPIO_BT_PCM_CLK		71

#define BCM4325_BT_WAKE		77
#define BCM4325_WLAN_RESET	81
#define GPIO_BT_HOST_WAKE	94


// Related to I2C sensors
#define SENSOR_SCL	2
#define SENSOR_SDA	3

#define AUDIO_AMP_SCL	82
#define AUDIO_AMP_SDA	83

#define INPUT_TOUCH_SCL	29
#define INPUT_TOUCH_SDA	30


//Accelerometer Sensor
#define SENSOR_INT	84
//Compass Sensor
#define SENSOR_INT_1	1 // Modified (iks.kim 20090715)
//Proximity Sensor
#define PROXIMITY_SENSOR_INT 57

#define SENSOR_RESET	108


// Related to Camera
#define CAM_5M_SCL	60
#define CAM_5M_SDA	61

#define CAM_PM_LP8720_SCL           36
#define CAM_PM_LP8720_SDA           37

// Related to Audio
#define GPIO_JACK_S_35 21
#define GPIO_SEND_END 38

#else

#define GPIO_WLAN_BT_REG_ON 20 
#define GPIO_WLAN_HOST_WAKE	28

#define BCM4325_BT_RESET 38

#define GPIO_BT_UART_RTS	43
#define GPIO_BT_UART_CTS	44
#define GPIO_BT_UART_RXD	45
#define GPIO_BT_UART_TXD	46

#define GPIO_BT_PCM_DOUT	68
#define GPIO_BT_PCM_DIN		69
#define GPIO_BT_PCM_SYNC	70
#define GPIO_BT_PCM_CLK		71

#define BCM4325_BT_WAKE		77
#define BCM4325_WLAN_RESET	81
#define GPIO_BT_HOST_WAKE	94


// Related to I2C sensors
#define SENSOR_SCL	2
#define SENSOR_SDA	3

#define AUDIO_AMP_SCL 82
#define AUDIO_AMP_SDA 83

#define INPUT_TOUCH_SCL	29
#define INPUT_TOUCH_SDA	30

//Accelerometer Sensor
#define SENSOR_INT	 84
//Compass Sensor
#define SENSOR_INT_1 102 
//Proximity Sensor
#define PROXIMITY_SENSOR_INT 57

#define SENSOR_RESET 108


// Related to Camera
#define CAM_5M_SCL	60
#define CAM_5M_SDA	61

#define CAM_PM_LP8720_SCL           36
#define CAM_PM_LP8720_SDA           37

// Related to Audio
#define GPIO_JACK_S_35 21
#define GPIO_SEND_END 109
#endif


#endif  // ASM_MACH_BEHOLD2_H

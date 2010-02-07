obj-$(CONFIG_MSM_CAMERA) += mt9d112.o mt9d112_reg.o flash.o
obj-$(CONFIG_ARCH_MSM) += msm_vfe7x.o msm_io7x.o
obj-$(CONFIG_ARCH_QSD) += msm_vfe8x.o msm_vfe8x_proc.o msm_io8x.o 






ifeq ($(CONFIG_SENSOR_AIT848), y)
obj-$(CONFIG_SENSOR_AIT848) += pgh_ait848.o
endif

ifeq ($(CONFIG_SENSOR_M4MO), y)
#obj-$(CONFIG_SENSOR_M4MO) += pgh_m4mo.o
obj-$(CONFIG_SENSOR_M4MO) += sec_m4mo.o
endif


ifeq ($(CONFIG_SENSOR_S5K4CA), y)
obj-$(CONFIG_SENSOR_S5K4CA) += s5k4ca_I2Cf.o s5k4ca_I2Cs.o
endif


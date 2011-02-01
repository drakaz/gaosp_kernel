#ifndef PROXIMITY_H
#define PROXIMITY_H

#define GP2AP_IOCTL_MAGIC 'c'
#define GP2AP_IOCTL_GET_ENABLED \
		_IOR(GP2AP_IOCTL_MAGIC, 1, int *)
#define GP2AP_IOCTL_ENABLE \
		_IOW(GP2AP_IOCTL_MAGIC, 2, int *)

enum gp2ap_mod {
	PROXIMITY = 1,
	LIGHT
};

int sharp_gp2ap002_enable(int enable, enum gp2ap_mod mode);

#endif

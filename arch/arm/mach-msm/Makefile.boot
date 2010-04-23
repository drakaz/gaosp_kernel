ifeq ($(CONFIG_MACH_BEHOLD2),y)
  zreladdr-y		:= 0x18008000
params_phys-y		:= 0x18000100
initrd_phys-y		:= 0x19000000
else
  zreladdr-y		:= 0x10008000
params_phys-y		:= 0x10000100
initrd_phys-y		:= 0x11000000
endif

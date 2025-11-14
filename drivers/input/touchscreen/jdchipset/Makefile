# Makefile for the Jadard touchscreen drivers.
ccflags-y += -DCONFIG_TOUCHSCREEN_JADARD_DEBUG=y
ccflags-y += -DCONFIG_TOUCHSCREEN_JADARD_SORTING=y
ccflags-y += -DDEBUG
ccflags-y += -D__JADARD_GKI__

# Makefile for the Jadard touchscreen drivers.
#obj-$(CONFIG_TOUCHSCREEN_JDCHIPSET)	+= jadard_module.o
#obj-$(CONFIG_TOUCHSCREEN_JDCHIPSET)	+= jadard_ic_JD9365TX.o
#obj-$(CONFIG_TOUCHSCREEN_JDCHIPSET)	+= jadard_common.o jadard_platform.o
#obj-$(CONFIG_TOUCHSCREEN_JDCHIPSET)	+= jadard_debug.o
#obj-$(CONFIG_TOUCHSCREEN_JDCHIPSET)	+= jadard_sorting.o


ccflags-y += -D__JADARD_KMODULE__

	jdchipset-objs := jadard_module.o
	jdchipset-objs += jadard_ic_JD9365TX.o
	jdchipset-objs += jadard_common.o jadard_platform.o
	jdchipset-objs += jadard_debug.o
	jdchipset-objs += jadard_sorting.o
	obj-m += jdchipset.o

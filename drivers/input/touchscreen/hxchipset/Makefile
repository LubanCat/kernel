# Makefile for the Himax touchscreen drivers.

ifneq ($(filter y, $(CONFIG_KALLSYMS_ALL)),)
	ccflags-y += -D__KERNEL_KALLSYMS_ALL_ENABLED__
endif

ifneq ($(filter y, $(CONFIG_FB)),)
	ccflags-y += -DHX_CONFIG_FB
endif

ifneq ($(filter y, $(CONFIG_DRM)),)
	ccflags-y += -DHX_CONFIG_DRM
endif

ifneq ($(filter y, $(CONFIG_PM)),)
	ccflags-y += -DHX_CONFIG_PM
endif

# obj-y += himax_modular_table.o

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)),)
	hxchipset-objs += himax_debug.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_INSPECT)),)
	hxchipset-objs += himax_inspection.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83102E)),)
  ccflags-y += -D__HIMAX_HX83102E_MOD__
	hxchipset-objs += himax_ic_HX83102e.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83102G)),)
  ccflags-y += -D__HIMAX_HX83102G_MOD__
	hxchipset-objs += himax_ic_HX83102g.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83108)),)
  ccflags-y += -D__HIMAX_HX83108_MOD__
	hxchipset-objs += himax_ic_HX83108.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83102J)),)
  ccflags-y += -D__HIMAX_HX83102J_MOD__
	hxchipset-objs += himax_ic_HX83102j.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83112F)),)
  ccflags-y += -D__HIMAX_HX83112F_MOD__
	hxchipset-objs += himax_ic_HX83112f.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83121A)),)
  ccflags-y += -D__HIMAX_HX83121A_MOD__
	hxchipset-objs += himax_ic_HX83121a.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83122A)),)
  ccflags-y += -D__HIMAX_HX83122A_MOD__
	hxchipset-objs += himax_ic_HX83122a.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83123A)),)
  ccflags-y += -D__HIMAX_HX83123A_MOD__
	hxchipset-objs += himax_ic_HX83123a.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX85200A)),)
  ccflags-y += -D__HIMAX_HX85200A_MOD__
	hxchipset-objs += himax_ic_HX85200a.o
endif

ifneq ($(filter y m, $(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83132A)),)
  ccflags-y += -D__HIMAX_HX83132A_MOD__
	hxchipset-objs += himax_ic_HX83132a.o
endif

ifneq ($(filter m, $(CONFIG_TOUCHSCREEN_HIMAX_COMMON)),)
  ccflags-y += -D__HIMAX_MOD__
  hxchipset-objs += himax_ic_core.o
  hxchipset-objs += himax_common.o
  hxchipset-objs += himax_platform.o
  ifneq ($(filter y, $(CONFIG_TOUCHSCREEN_HIMAX_EMBEDDED_FIRMWARE)),)
    ccflags-y += -DHX_FIRMWARE_HEADER
    hxchipset-objs += himax_firmware.o
  endif
  obj-m += hxchipset.o
endif
ifneq ($(filter y, $(CONFIG_TOUCHSCREEN_HIMAX_COMMON)),)
  hxchipset-objs += himax_ic_core.o
  hxchipset-objs += himax_common.o
  hxchipset-objs += himax_platform.o
  ifneq ($(filter y, $(CONFIG_TOUCHSCREEN_HIMAX_EMBEDDED_FIRMWARE)),)
	ccflags-y += -DHX_FIRMWARE_HEADER
	hxchipset-objs += himax_firmware.o
  endif
  obj-y += hxchipset.o
endif


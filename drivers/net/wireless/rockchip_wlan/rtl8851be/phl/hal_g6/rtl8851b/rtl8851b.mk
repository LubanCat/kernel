ifeq ($(CONFIG_RTL8851B), y)
EXTRA_CFLAGS += -DCONFIG_RTL8851B
endif

ifeq ($(CONFIG_RTL8851BP), y)
EXTRA_CFLAGS += -DCONFIG_RTL8851BP
endif

IC_NAME := rtl8851b

ifeq ($(CONFIG_MP_INCLUDED), y)
### 8851B Default Enable VHT MP HW TX MODE ###
#EXTRA_CFLAGS += -DCONFIG_MP_VHT_HW_TX_MODE
#CONFIG_MP_VHT_HW_TX_MODE = y
endif

ifeq ($(CONFIG_PHL_ARCH), y)
HAL = phl/hal_g6
else
HAL = hal_g6
endif

ifeq ($(CONFIG_USB_HCI), y)
	FILE_NAME = rtl8851bu
endif
ifeq ($(CONFIG_PCI_HCI), y)
	FILE_NAME = rtl8851be
endif
ifeq ($(CONFIG_SDIO_HCI), y)
	FILE_NAME = rtl8851bs
endif


_HAL_IC_FILES +=	$(HAL)/$(IC_NAME)/$(IC_NAME)_halinit.o \
			$(HAL)/$(IC_NAME)/$(IC_NAME)_ops.o \
			$(HAL)/$(IC_NAME)/$(IC_NAME)_regulation.o \
			$(HAL)/$(IC_NAME)/hal_trx_8851b.o

_HAL_IC_FILES +=	$(HAL)/$(IC_NAME)/$(HCI_NAME)/$(FILE_NAME)_halinit.o \
			$(HAL)/$(IC_NAME)/$(HCI_NAME)/$(FILE_NAME)_ops.o

#ifeq ($(CONFIG_SDIO_HCI), y)
#_HAL_IC_FILES += $(HAL)/$(IC_NAME)/$(HCI_NAME)/hal_trx_8851bs.o
#endif

ifeq ($(CONFIG_USB_HCI), y)
_HAL_IC_FILES += $(HAL)/$(IC_NAME)/$(HCI_NAME)/hal_trx_8851bu.o
endif

ifeq ($(CONFIG_PCI_HCI), y)
_HAL_IC_FILES += $(HAL)/$(IC_NAME)/$(HCI_NAME)/hal_trx_8851be.o
endif

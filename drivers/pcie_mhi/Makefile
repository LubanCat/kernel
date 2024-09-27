#ccflags-y += -g
obj-m += pcie_mhi.o
pcie_mhi-objs := core/mhi_init.o core/mhi_main.o core/mhi_pm.o core/mhi_boot.o core/mhi_dtr.o  controllers/mhi_qti.o
pcie_mhi-objs += devices/mhi_uci.o

ifeq (1,1)
pcie_mhi-objs += devices/mhi_netdev_quectel.o
else
pcie_mhi-objs += devices/mhi_netdev.o
pcie_mhi-objs += devices/rmnet_handler.o
endif


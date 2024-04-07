# SPDX-License-Identifier: GPL-2.0
ifeq ($(CONFIG_ARCH_ROCKCHIP), y)

dtbo-$(CONFIG_CPU_RK312X) += \
	rk3128-lubancat-uart0-overlay.dtbo

endif

dtbotxt-$(CONFIG_ARCH_ROCKCHIP) += \
	README.rockchip-overlays.md

targets += $(dtbo-y) $(dtbotxt-y)

always-y		:= $(dtbo-y) $(dtbotxt-y)
clean-files		:= *.dtbo

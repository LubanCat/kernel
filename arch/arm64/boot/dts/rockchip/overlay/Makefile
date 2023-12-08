# SPDX-License-Identifier: GPL-2.0
ifeq ($(CONFIG_ARCH_ROCKCHIP), y)

dtbo-$(CONFIG_CPU_RK3588) += \
	rk3588-lubancat-i2c2-m4-overlay.dtbo \
	rk3588-lubancat-i2c3-m1-overlay.dtbo \
	rk3588-lubancat-i2c5-m3-overlay.dtbo \
	rk3588-lubancat-i2c6-m3-overlay.dtbo \
	rk3588-lubancat-i2c8-m2-overlay.dtbo \
	rk3588-lubancat-pwm3-ir-m3-overlay.dtbo \
	rk3588-lubancat-pwm10-m2-overlay.dtbo \
	rk3588-lubancat-pwm11-ir-m3-overlay.dtbo \
	rk3588-lubancat-pwm14-m2-overlay.dtbo \
	rk3588-lubancat-pwm15-ir-m3-overlay.dtbo \
	rk3588-lubancat-spi0-m2-overlay.dtbo \
	rk3588-lubancat-uart0-m2-overlay.dtbo \
	rk3588-lubancat-uart4-m2-overlay.dtbo \
	rk3588-lubancat-uart6-m1-overlay.dtbo \
	rk3588-lubancat-uart7-m1-overlay.dtbo \
	rk3588-lubancat-uart7-m2-overlay.dtbo \
	rk3588-lubancat-uart9-m2-overlay.dtbo \
	rk3588s-lubancat-4-dp0-in-vp2-overlay.dtbo \
	rk3588s-lubancat-4-dsi0-800x1280-overlay.dtbo \
	rk3588s-lubancat-4-dsi0-1024x600-overlay.dtbo \
	rk3588s-lubancat-4-dsi0-1080p-overlay.dtbo \
	rk3588s-lubancat-4-dsi1-800x1280-overlay.dtbo \
	rk3588s-lubancat-4-dsi1-1024x600-overlay.dtbo \
	rk3588s-lubancat-4-dsi1-1080p-overlay.dtbo \
	rk3588s-lubancat-4-hdmi0-8k-overlay.dtbo \
	rk3588-lubancat-msata-overlay.dtbo \
	rk3588-lubancat-5-hdmi-8k-overlay.dtbo \
	rk3588-lubancat-5-dsi0-1080p-overlay.dtbo \
	rk3588-lubancat-5-dsi1-1080p-overlay.dtbo \
	rk3588-lubancat-5-edp-overlay.dtbo \
	rk3588-lubancat-5-hdmi0-overlay.dtbo \
	rk3588-lubancat-5-hdmi1-vp0-overlay.dtbo \
	rk3588-lubancat-5-hdmi1-vp2-overlay.dtbo \
	rk3588-lubancat-5-hdmi-rx-overlay.dtbo \
	rk3588-lubancat-5-cam0-imx415-overlay.dtbo \
	rk3588-lubancat-5-cam1-imx415-overlay.dtbo \
	rk3588-lubancat-5-cam2-ov8858-overlay.dtbo \
	rk3588-lubancat-5-cam3-ov8858-overlay.dtbo \
	rk3588-lubancat-5-cam4-ov8858-overlay.dtbo \
	rk3588-lubancat-5-cam5-ov8858-overlay.dtbo

endif

dtbotxt-$(CONFIG_ARCH_ROCKCHIP) += \
	README.rockchip-overlays.md

targets += $(dtbo-y) $(dtbotxt-y)

always-y		:= $(dtbo-y) $(dtbotxt-y)
clean-files		:= *.dtbo

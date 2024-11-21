// SPDX-License-Identifier: GPL-2.0
/*
 * ov50h40 driver
 *
 * Copyright (C) 2020 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 * V0.0X01.0X01 support conversion gain switch.
 * V0.0X01.0X02 add debug interface for conversion gain switch.
 * V0.0X01.0X03 support enum sensor fmt
 * V0.0X01.0X04 add quick stream on/off
 * V0.0X01.0X05 support get dcg ratio from sensor
 * V0.0X01.0X06
 * 1. fix 8K@12 mipi freq index.
 * 2. fix set_fmt & ioctl get mode unmatched issue.
 * 3. add debug info.
 * V0.0X01.0X07 correct bayer pattern to match register setting
 * V0.0X01.0X08 adjust some config for cts.
 * 1. only enable 8K@12fps & 4K@30fps setting for use, other for debug.
 */
//#define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"
#include <linux/of_graph.h>
#include "otp_eeprom.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x08)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define MIPI_FREQ_356M			356000000
#define MIPI_FREQ_384M			384000000
#define MIPI_FREQ_750M			750000000
#define MIPI_FREQ_1250M			1250000000

#define PIXEL_RATE_WITH_1250M		(MIPI_FREQ_1250M / 10 * 4 * 2)

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define OV50H40_XVCLK_FREQ		19200000

#define CHIP_ID					0x564041
#define OV50H40_REG_CHIP_ID		0x300a

#define OV50H40_REG_CTRL_MODE	0x0100
#define OV50H40_MODE_SW_STANDBY	0x0
#define OV50H40_MODE_STREAMING	BIT(0)

#define	OV50H40_EXPOSURE_MIN		4
#define	OV50H40_EXPOSURE_STEP		1
#define OV50H40_VTS_MAX				0xffff

#define OV50H40_REG_EXP_LONG_H		0x3500

#define OV50H40_REG_AGAIN_LONG_H	0x3508
#define OV50H40_REG_DGAIN_LONG_H	0x350A
#define OV50H40_GAIN_MIN			0x80
#define OV50H40_GAIN_MAX			0x7C00
#define OV50H40_GAIN_STEP			1
#define OV50H40_GAIN_DEFAULT		0x80

#define OV50H40_GROUP_UPDATE_ADDRESS		0x3208
#define OV50H40_GROUP_UPDATE_START_DATA		0x00
#define OV50H40_GROUP_UPDATE_END_DATA		0x10
#define OV50H40_GROUP_UPDATE_END_LAUNCH		0xA0

#define OV50H40_SOFTWARE_RESET_REG		0x0103

#define OV50H40_FETCH_MSB_BYTE_EXP(VAL)	(((VAL) >> 8) & 0xFF)	/* 8 Bits */
#define OV50H40_FETCH_LSB_BYTE_EXP(VAL)	((VAL) & 0xFF)	/* 8 Bits */

#define OV50H40_FETCH_LSB_GAIN(VAL)	(((VAL) << 4) & 0xf0)
#define OV50H40_FETCH_MSB_GAIN(VAL)	(((VAL) >> 4) & 0x1f)

#define OV50H40_REG_TEST_PATTERN		0x50C1
#define OV50H40_TEST_PATTERN_ENABLE		0x01
#define OV50H40_TEST_PATTERN_DISABLE	0x0

#define OV50H40_REG_VTS			0x380e

#define REG_NULL				0xFFFF

#define OV50H40_REG_VALUE_08BIT		1
#define OV50H40_REG_VALUE_16BIT		2
#define OV50H40_REG_VALUE_24BIT		3

#define OV50H40_LANES				3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define OV50H40_NAME			"ov50h40"

static const char * const ov50h40_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OV50H40_NUM_SUPPLIES ARRAY_SIZE(ov50h40_supply_names)

#define OV50H40_FLIP_REG		0x3820
#define OV50H40_MIRROR_REG		0x3821
#define FLIP_BIT_MASK			BIT(2)

struct regval {
	u16 addr;
	u8 val;
};

struct other_data {
	u32 width;
	u32 height;
	u32 bus_fmt;
	u32 data_type;
	u32 data_bit;
};

struct ov50h40_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 mipi_freq_idx;
	u32 bpp;
	const struct regval *reg_list;
	u32 hdr_mode;
	const struct other_data *spd;
	u32 vc[PAD_MAX];
};

struct ov50h40 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OV50H40_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ov50h40_mode *cur_mode;
	const struct ov50h40_mode *support_modes;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	struct v4l2_fwnode_endpoint bus_cfg;
	bool			is_thunderboot;
	bool			is_thunderboot_ng;
	bool			is_first_streamoff;
	struct otp_info		*otp;
	u32			spd_id;
};

#define to_ov50h40(sd) container_of(sd, struct ov50h40, subdev)

static const struct regval ov50h40_10bit_4096x3072_dphy_30fps_regs[] = {
	{REG_NULL, 0x00},
};

static const struct regval ov50h40_10bit_8192x6144_dphy_12fps_regs[] = {
	{REG_NULL, 0x00},
};

static const struct regval ov50h40_10bit_4096x3072_cphy_regs[] = {
	{0x0103, 0x01},
	{0x0102, 0x01},
	{0x6a03, 0x00},
	{0x0304, 0x02},
	{0x0305, 0xd0},
	{0x0306, 0x03},
	{0x0307, 0x00},
	{0x0308, 0x03},
	{0x0323, 0x12},
	{0x0324, 0x02},
	{0x0325, 0x58},
	{0x0327, 0x09},
	{0x0328, 0x9f},
	{0x0329, 0x01},
	{0x032a, 0x0f},
	{0x032b, 0x09},
	{0x032c, 0x00},
	{0x032e, 0x01},
	{0x0343, 0x02},
	{0x0344, 0x01},
	{0x0345, 0x20},
	{0x0346, 0xdf},
	{0x0347, 0x0f},
	{0x0348, 0x7f},
	{0x0349, 0x0f},
	{0x034a, 0x03},
	{0x034b, 0x02},
	{0x034c, 0x03},
	{0x034d, 0x01},
	{0x034e, 0x01},
	{0x0360, 0x09},
	{0x300d, 0x11},
	{0x300d, 0x11},
	{0x300e, 0x11},
	{0x3012, 0x31},
	{0x3014, 0xf0},
	{0x3015, 0x00},
	{0x3016, 0xf0},
	{0x3017, 0xf0},
	{0x301c, 0x01},
	{0x301d, 0x02},
	{0x301f, 0x98},
	{0x3020, 0x01},
	{0x3025, 0x03},
	{0x3026, 0x80},
	{0x3027, 0x00},
	{0x302c, 0x01},
	{0x302d, 0x00},
	{0x302e, 0x00},
	{0x302f, 0x00},
	{0x3030, 0x03},
	{0x3031, 0x00},
	{0x3044, 0xc2},
	{0x3047, 0x07},
	{0x3102, 0x0d},
	{0x3106, 0x80},
	{0x3400, 0x0c},
	{0x3401, 0x00},
	{0x3406, 0x08},
	{0x3407, 0x08},
	{0x3408, 0x08},
	{0x3409, 0x02},
	{0x340a, 0x03},
	{0x340e, 0x60},
	{0x3420, 0x03},
	{0x3421, 0x08},
	{0x3422, 0x08},
	{0x3423, 0x00},
	{0x3426, 0x15},
	{0x342b, 0x40},
	{0x342c, 0x15},
	{0x342d, 0x01},
	{0x342e, 0x00},
	{0x3500, 0x00},
	{0x3501, 0x00},
	{0x3502, 0x40},
	{0x3504, 0x4c},
	{0x3506, 0x78},
	{0x3507, 0x00},
	{0x3508, 0x01},
	{0x3509, 0x00},
	{0x350a, 0x01},
	{0x350b, 0x00},
	{0x350c, 0x00},
	{0x350d, 0x01},
	{0x350e, 0x00},
	{0x350f, 0x00},
	{0x3519, 0x01},
	{0x351a, 0x71},
	{0x351b, 0x40},
	{0x3540, 0x00},
	{0x3541, 0x00},
	{0x3542, 0x30},
	{0x3544, 0x4c},
	{0x3546, 0x78},
	{0x3548, 0x01},
	{0x3549, 0x00},
	{0x354a, 0x01},
	{0x354b, 0x00},
	{0x354d, 0x01},
	{0x354e, 0x00},
	{0x354f, 0x00},
	{0x3559, 0x01},
	{0x355a, 0x71},
	{0x355b, 0x40},
	{0x3580, 0x00},
	{0x3581, 0x00},
	{0x3582, 0x20},
	{0x3584, 0x4c},
	{0x3586, 0x78},
	{0x3588, 0x01},
	{0x3589, 0x00},
	{0x358a, 0x01},
	{0x358b, 0x00},
	{0x358d, 0x01},
	{0x358e, 0x00},
	{0x358f, 0x00},
	{0x3599, 0x01},
	{0x359a, 0x71},
	{0x359b, 0x40},
	{0x3600, 0xe4},
	{0x3602, 0xe4},
	{0x3603, 0x80},
	{0x3605, 0x38},
	{0x3607, 0x10},
	{0x3608, 0x30},
	{0x3609, 0x80},
	{0x360a, 0xfa},
	{0x360b, 0xc7},
	{0x360c, 0x0f},
	{0x360d, 0xf4},
	{0x360e, 0x2b},
	{0x3610, 0x08},
	{0x3612, 0x00},
	{0x3614, 0x0c},
	{0x3616, 0x8c},
	{0x3617, 0x0d},
	{0x3618, 0xcf},
	{0x3619, 0x44},
	{0x361a, 0x81},
	{0x361b, 0x04},
	{0x361d, 0x1f},
	{0x3622, 0x00},
	{0x3627, 0xa0},
	{0x363b, 0x6a},
	{0x363c, 0x6a},
	{0x3640, 0x00},
	{0x3641, 0x02},
	{0x3643, 0x01},
	{0x3644, 0x00},
	{0x3645, 0x06},
	{0x3646, 0x40},
	{0x3647, 0x01},
	{0x3648, 0x8e},
	{0x364d, 0x10},
	{0x3650, 0xbf},
	{0x3651, 0x00},
	{0x3653, 0x03},
	{0x3657, 0x40},
	{0x3680, 0x00},
	{0x3682, 0x80},
	{0x3683, 0x00},
	{0x3684, 0x01},
	{0x3685, 0x04},
	{0x3688, 0x00},
	{0x3689, 0x88},
	{0x368a, 0x0e},
	{0x368b, 0xef},
	{0x368d, 0x00},
	{0x368e, 0x70},
	{0x3696, 0x41},
	{0x369a, 0x00},
	{0x369f, 0x20},
	{0x36a4, 0x00},
	{0x36a5, 0x00},
	{0x36d0, 0x00},
	{0x36d3, 0x80},
	{0x36d4, 0x00},
	{0x3700, 0x1c},
	{0x3701, 0x13},
	{0x3702, 0x30},
	{0x3703, 0x34},
	{0x3704, 0x03},
	{0x3706, 0x1c},
	{0x3707, 0x04},
	{0x3708, 0x25},
	{0x3709, 0x70},
	{0x370b, 0x3a},
	{0x370c, 0x04},
	{0x3712, 0x01},
	{0x3714, 0xf8},
	{0x3715, 0x00},
	{0x3716, 0x40},
	{0x3720, 0x0b},
	{0x3722, 0x05},
	{0x3724, 0x12},
	{0x372b, 0x00},
	{0x372e, 0x1c},
	{0x372f, 0x13},
	{0x3733, 0x00},
	{0x3735, 0x00},
	{0x373f, 0x00},
	{0x374b, 0x04},
	{0x374c, 0x0c},
	{0x374f, 0x58},
	{0x3754, 0x30},
	{0x3755, 0xb1},
	{0x3756, 0x00},
	{0x3757, 0x30},
	{0x3758, 0x00},
	{0x3759, 0x50},
	{0x375e, 0x00},
	{0x375f, 0x00},
	{0x3760, 0x10},
	{0x3761, 0x30},
	{0x3762, 0x10},
	{0x3763, 0x10},
	{0x3765, 0x20},
	{0x3766, 0x30},
	{0x3767, 0x20},
	{0x3768, 0x00},
	{0x3769, 0x10},
	{0x376a, 0x10},
	{0x376c, 0x00},
	{0x376e, 0x00},
	{0x3770, 0x01},
	{0x3780, 0x5c},
	{0x3782, 0x01},
	{0x378a, 0x01},
	{0x3791, 0x30},
	{0x3793, 0x1c},
	{0x3795, 0x1c},
	{0x3797, 0x8e},
	{0x3799, 0x3a},
	{0x379b, 0x3a},
	{0x379c, 0x01},
	{0x379d, 0x01},
	{0x379f, 0x01},
	{0x37a0, 0x70},
	{0x37a9, 0x01},
	{0x37b2, 0xc8},
	{0x37b7, 0x02},
	{0x37bd, 0x00},
	{0x37c1, 0x1a},
	{0x37c3, 0x1a},
	{0x37ca, 0xc4},
	{0x37cb, 0x02},
	{0x37cc, 0x51},
	{0x37cd, 0x01},
	{0x37d0, 0x00},
	{0x37d4, 0x00},
	{0x37d8, 0x00},
	{0x37d9, 0x08},
	{0x37da, 0x14},
	{0x37db, 0x10},
	{0x37dc, 0x1a},
	{0x37dd, 0x86},
	{0x37e0, 0x68},
	{0x37e3, 0x30},
	{0x37e4, 0xf6},
	{0x37f0, 0x01},
	{0x37f1, 0xe0},
	{0x37f2, 0x24},
	{0x37f6, 0x1a},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x20},
	{0x3805, 0x1f},
	{0x3806, 0x18},
	{0x3807, 0x3f},
	{0x3808, 0x20},
	{0x3809, 0x00},
	{0x380a, 0x18},
	{0x380b, 0x00},
	{0x380c, 0x03},
	{0x380d, 0x00},
	{0x380e, 0x0c},
	{0x380f, 0x80},
	{0x3810, 0x00},
	{0x3811, 0x0f},
	{0x3812, 0x00},
	{0x3813, 0x20},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x381a, 0x0c},
	{0x381b, 0x70},
	{0x381c, 0x01},
	{0x381d, 0x80},
	{0x381f, 0x00},
	{0x3820, 0x40},
	{0x3821, 0x04},
	{0x3822, 0x00},
	{0x3823, 0x04},
	{0x3827, 0x40},
	{0x3828, 0x27},
	{0x382a, 0x80},
	{0x382e, 0x49},
	{0x3830, 0x20},
	{0x3831, 0x10},
	{0x3837, 0x20},
	{0x383f, 0x08},
	{0x3840, 0x00},
	{0x3847, 0x00},
	{0x384a, 0x00},
	{0x384c, 0x03},
	{0x384d, 0x00},
	{0x3858, 0x00},
	{0x3860, 0x00},
	{0x3867, 0x11},
	{0x386a, 0x00},
	{0x386b, 0x00},
	{0x386c, 0x00},
	{0x386d, 0x7c},
	{0x3888, 0x00},
	{0x3889, 0x10},
	{0x388a, 0x00},
	{0x388b, 0x20},
	{0x388c, 0x20},
	{0x388d, 0x00},
	{0x388e, 0x18},
	{0x388f, 0x00},
	{0x3890, 0x11},
	{0x3894, 0x02},
	{0x3895, 0x80},
	{0x3896, 0x00},
	{0x3899, 0x00},
	{0x38a0, 0x00},
	{0x38a1, 0x1d},
	{0x38a2, 0x98},
	{0x38a3, 0x00},
	{0x38a4, 0x1d},
	{0x38a5, 0x98},
	{0x38ac, 0x40},
	{0x38ad, 0x00},
	{0x38ae, 0x00},
	{0x38af, 0x00},
	{0x38b0, 0x00},
	{0x38b1, 0x00},
	{0x38b2, 0x00},
	{0x38b3, 0x00},
	{0x38b4, 0x20},
	{0x38b5, 0x1f},
	{0x38b6, 0x18},
	{0x38b7, 0x1f},
	{0x38b8, 0x20},
	{0x38b9, 0x00},
	{0x38ba, 0x18},
	{0x38bb, 0x00},
	{0x38bc, 0x00},
	{0x38bd, 0x10},
	{0x38be, 0x00},
	{0x38bf, 0x10},
	{0x38c0, 0x11},
	{0x38c1, 0x11},
	{0x38c2, 0x00},
	{0x38c3, 0x00},
	{0x38c4, 0x00},
	{0x38c5, 0x00},
	{0x38c6, 0x11},
	{0x38c7, 0x00},
	{0x38c8, 0x11},
	{0x38c9, 0x00},
	{0x38ca, 0x11},
	{0x38cb, 0x00},
	{0x38cc, 0x11},
	{0x38cd, 0x00},
	{0x38ce, 0x11},
	{0x38cf, 0x00},
	{0x38d1, 0x11},
	{0x38d2, 0x00},
	{0x38d3, 0x00},
	{0x38d4, 0x08},
	{0x38d5, 0x00},
	{0x38d6, 0x08},
	{0x38db, 0x20},
	{0x38dd, 0x10},
	{0x38de, 0x0c},
	{0x38df, 0x20},
	{0x38e0, 0x00},
	{0x38f3, 0x00},
	{0x3900, 0x40},
	{0x3906, 0x24},
	{0x3907, 0x00},
	{0x390a, 0x05},
	{0x3913, 0x0c},
	{0x3918, 0x00},
	{0x3919, 0x15},
	{0x395b, 0x05},
	{0x3982, 0x40},
	{0x398b, 0x00},
	{0x3994, 0x0b},
	{0x3995, 0x30},
	{0x399d, 0x05},
	{0x39a0, 0x0b},
	{0x39dc, 0x01},
	{0x39fb, 0x01},
	{0x39fc, 0x01},
	{0x39fd, 0x06},
	{0x39fe, 0x06},
	{0x3a1d, 0x01},
	{0x3a1e, 0x01},
	{0x3a1f, 0x03},
	{0x3a21, 0x01},
	{0x3a22, 0x06},
	{0x3a23, 0x03},
	{0x3a68, 0x05},
	{0x3a69, 0x20},
	{0x3a6d, 0x50},
	{0x3a78, 0x03},
	{0x3a79, 0x03},
	{0x3a7c, 0x04},
	{0x3a7d, 0x04},
	{0x3a94, 0x04},
	{0x3ab5, 0x00},
	{0x3ab6, 0x01},
	{0x3ab7, 0x01},
	{0x3ab8, 0x01},
	{0x3ab9, 0x01},
	{0x3af2, 0x03},
	{0x3b01, 0x00},
	{0x3b02, 0x00},
	{0x3b16, 0x00},
	{0x3b3d, 0x07},
	{0x3b4a, 0x38},
	{0x3b4b, 0x38},
	{0x3b56, 0x20},
	{0x3b57, 0x21},
	{0x3b58, 0x21},
	{0x3b59, 0x21},
	{0x3b5a, 0x14},
	{0x3b5b, 0x14},
	{0x3b5c, 0x14},
	{0x3b5d, 0x14},
	{0x3b82, 0x14},
	{0x3ba1, 0x20},
	{0x3ba4, 0x77},
	{0x3ba5, 0x77},
	{0x3ba6, 0x00},
	{0x3ba7, 0x00},
	{0x3baa, 0x33},
	{0x3bab, 0x37},
	{0x3bac, 0x77},
	{0x3baf, 0x00},
	{0x3bba, 0x4c},
	{0x3bde, 0x01},
	{0x3be0, 0x30},
	{0x3be7, 0x08},
	{0x3be8, 0x0f},
	{0x3beb, 0x00},
	{0x3bf2, 0x03},
	{0x3bf3, 0x01},
	{0x3bf4, 0x50},
	{0x3bfb, 0x01},
	{0x3bfc, 0x50},
	{0x3bff, 0x08},
	{0x3d84, 0x00},
	{0x3d85, 0x0b},
	{0x3d8c, 0x9b},
	{0x3d8d, 0xa0},
	{0x3daa, 0x00},
	{0x3dab, 0x00},
	{0x3f00, 0x10},
	{0x4008, 0x00},
	{0x4009, 0x02},
	{0x400e, 0x14},
	{0x4010, 0x34},
	{0x4011, 0x01},
	{0x4012, 0x17},
	{0x4015, 0x00},
	{0x4016, 0x1f},
	{0x4017, 0x00},
	{0x4018, 0x0f},
	{0x401a, 0x40},
	{0x401b, 0x04},
	{0x40f8, 0x04},
	{0x40f9, 0x00},
	{0x40fa, 0x02},
	{0x40fb, 0x00},
	{0x4100, 0x00},
	{0x4101, 0x00},
	{0x4102, 0x00},
	{0x4103, 0x00},
	{0x4105, 0x00},
	{0x4288, 0x27},
	{0x4504, 0x80},
	{0x4505, 0x0c},
	{0x4506, 0x01},
	{0x4509, 0x07},
	{0x450c, 0x00},
	{0x450d, 0x30},
	{0x450e, 0x00},
	{0x450f, 0x20},
	{0x4510, 0x00},
	{0x4511, 0x00},
	{0x4512, 0x00},
	{0x4513, 0x00},
	{0x4514, 0x00},
	{0x4515, 0x00},
	{0x4516, 0x00},
	{0x4517, 0x00},
	{0x4518, 0x00},
	{0x4519, 0x00},
	{0x451a, 0x00},
	{0x451b, 0x00},
	{0x451c, 0x00},
	{0x451d, 0x00},
	{0x451e, 0x00},
	{0x451f, 0x00},
	{0x4520, 0x00},
	{0x4521, 0x00},
	{0x4522, 0x00},
	{0x4523, 0x00},
	{0x4524, 0x00},
	{0x4525, 0x00},
	{0x4526, 0x00},
	{0x4527, 0x18},
	{0x4545, 0x00},
	{0x4546, 0x07},
	{0x4547, 0x33},
	{0x4549, 0x00},
	{0x454a, 0x00},
	{0x454b, 0x00},
	{0x454c, 0x00},
	{0x454d, 0x00},
	{0x454e, 0x00},
	{0x454f, 0x00},
	{0x4550, 0x00},
	{0x4551, 0x00},
	{0x4552, 0x00},
	{0x4553, 0x00},
	{0x4554, 0x00},
	{0x4555, 0x00},
	{0x4556, 0x00},
	{0x4557, 0x00},
	{0x4558, 0x00},
	{0x4559, 0x00},
	{0x455a, 0x00},
	{0x455b, 0x00},
	{0x455c, 0x00},
	{0x455d, 0x00},
	{0x455e, 0x00},
	{0x455f, 0x00},
	{0x4560, 0x00},
	{0x4561, 0x00},
	{0x4562, 0x00},
	{0x4563, 0x00},
	{0x4564, 0x00},
	{0x4565, 0x00},
	{0x4580, 0x01},
	{0x4583, 0x00},
	{0x4584, 0x00},
	{0x4585, 0x00},
	{0x4586, 0x00},
	{0x458c, 0x02},
	{0x458d, 0x00},
	{0x458e, 0x00},
	{0x45c0, 0x1c},
	{0x45c1, 0x80},
	{0x45c2, 0x0a},
	{0x45c3, 0x84},
	{0x45c4, 0x10},
	{0x45c5, 0x80},
	{0x45c6, 0x08},
	{0x45c7, 0x00},
	{0x45c8, 0x00},
	{0x45c9, 0x00},
	{0x45ca, 0x00},
	{0x45cb, 0x00},
	{0x45cc, 0x00},
	{0x45cd, 0x07},
	{0x45ce, 0x13},
	{0x45cf, 0x13},
	{0x45d0, 0x13},
	{0x45d2, 0x00},
	{0x45d3, 0x00},
	{0x45d4, 0x00},
	{0x45d5, 0x00},
	{0x45d6, 0x00},
	{0x45d7, 0x00},
	{0x45d8, 0x00},
	{0x45d9, 0x00},
	{0x45da, 0x00},
	{0x45dd, 0x00},
	{0x45de, 0x00},
	{0x45df, 0x00},
	{0x45e0, 0x00},
	{0x45e1, 0x00},
	{0x45e2, 0x00},
	{0x45e3, 0x00},
	{0x45e4, 0x00},
	{0x45e5, 0x00},
	{0x45e7, 0x00},
	{0x4602, 0x00},
	{0x4603, 0x15},
	{0x460b, 0x07},
	{0x4640, 0x01},
	{0x4641, 0x00},
	{0x4643, 0x08},
	{0x4644, 0xe0},
	{0x4645, 0xbf},
	{0x4647, 0x02},
	{0x464a, 0x00},
	{0x464b, 0x00},
	{0x464c, 0x01},
	{0x4680, 0x11},
	{0x4681, 0x80},
	{0x4684, 0x2b},
	{0x4685, 0x17},
	{0x4686, 0x00},
	{0x4687, 0x00},
	{0x4688, 0x00},
	{0x4689, 0x00},
	{0x468e, 0x30},
	{0x468f, 0x00},
	{0x4690, 0x00},
	{0x4691, 0x00},
	{0x4694, 0x04},
	{0x4800, 0x64},
	{0x4802, 0x02},
	{0x4806, 0x40},
	{0x4813, 0x10},
	{0x481b, 0x25},
	{0x4825, 0x32},
	{0x4826, 0x32},
	{0x4829, 0x64},
	{0x4836, 0x32},
	{0x4837, 0x04},
	{0x4840, 0x00},
	{0x4850, 0x42},
	{0x4851, 0xaa},
	{0x4853, 0x10},
	{0x4854, 0x05},
	{0x4855, 0x1c},
	{0x4860, 0x01},
	{0x4861, 0xec},
	{0x4862, 0x3a},
	{0x4883, 0x24},
	{0x4884, 0x11},
	{0x4888, 0x10},
	{0x4889, 0x00},
	{0x4911, 0x00},
	{0x491a, 0x40},
	{0x49f5, 0x00},
	{0x49f8, 0x04},
	{0x49f9, 0x00},
	{0x49fa, 0x02},
	{0x49fb, 0x00},
	{0x4a11, 0x00},
	{0x4a1a, 0x40},
	{0x4af8, 0x04},
	{0x4af9, 0x00},
	{0x4afa, 0x02},
	{0x4afb, 0x00},
	{0x4d00, 0x04},
	{0x4d01, 0x9d},
	{0x4d02, 0xbb},
	{0x4d03, 0x6c},
	{0x4d04, 0xc4},
	{0x4d05, 0x71},
	{0x5000, 0x5b},
	{0x5001, 0x28},
	{0x5002, 0x00},
	{0x5003, 0x0e},
	{0x5004, 0x02},
	{0x5007, 0x06},
	{0x5009, 0x2e},
	{0x5053, 0x05},
	{0x5060, 0x10},
	{0x5069, 0x10},
	{0x506a, 0x20},
	{0x506b, 0x04},
	{0x506c, 0x04},
	{0x506d, 0x0c},
	{0x506e, 0x0c},
	{0x506f, 0x04},
	{0x5070, 0x0c},
	{0x5071, 0x14},
	{0x5072, 0x1c},
	{0x5091, 0x00},
	{0x50c1, 0x00},
	{0x5110, 0x90},
	{0x5111, 0x14},
	{0x5112, 0x9b},
	{0x5113, 0x27},
	{0x5114, 0x01},
	{0x5155, 0x08},
	{0x5156, 0x0c},
	{0x5157, 0x0c},
	{0x5159, 0x08},
	{0x515a, 0x0c},
	{0x515b, 0x0c},
	{0x5180, 0xc0},
	{0x518a, 0x04},
	{0x51d3, 0x0a},
	{0x5251, 0x00},
	{0x5312, 0x00},
	{0x53c1, 0x00},
	{0x5410, 0x90},
	{0x5411, 0x14},
	{0x5412, 0x9b},
	{0x5413, 0x27},
	{0x5455, 0x08},
	{0x5456, 0x0c},
	{0x5457, 0x0c},
	{0x5459, 0x08},
	{0x545a, 0x0c},
	{0x545b, 0x0c},
	{0x5480, 0xc0},
	{0x548a, 0x04},
	{0x56c1, 0x00},
	{0x5710, 0x90},
	{0x5711, 0x14},
	{0x5712, 0x9b},
	{0x5713, 0x27},
	{0x5755, 0x08},
	{0x5756, 0x0c},
	{0x5757, 0x0c},
	{0x5759, 0x08},
	{0x575a, 0x0c},
	{0x575b, 0x0c},
	{0x5780, 0xc0},
	{0x578a, 0x04},
	{0x5853, 0xfe},
	{0x5854, 0xfe},
	{0x5855, 0xfe},
	{0x5856, 0xff},
	{0x5857, 0xff},
	{0x5858, 0xff},
	{0x587b, 0x16},
	{0x58a7, 0x11},
	{0x58c0, 0x3f},
	{0x58fd, 0x0a},
	{0x5925, 0x00},
	{0x5926, 0x00},
	{0x5927, 0x00},
	{0x5928, 0x00},
	{0x5929, 0x00},
	{0x592c, 0x06},
	{0x592d, 0x00},
	{0x592e, 0x03},
	{0x59c2, 0x00},
	{0x59c3, 0xce},
	{0x59c4, 0x01},
	{0x59c5, 0x20},
	{0x59c6, 0x01},
	{0x59c7, 0x91},
	{0x59c8, 0x02},
	{0x59c9, 0x2f},
	{0x59ca, 0x03},
	{0x59cb, 0x0a},
	{0x59cc, 0x04},
	{0x59cd, 0x3d},
	{0x59ce, 0x05},
	{0x59cf, 0xe8},
	{0x59d0, 0x08},
	{0x59d1, 0x3c},
	{0x59d2, 0x0b},
	{0x59d3, 0x7a},
	{0x59d4, 0x0f},
	{0x59d5, 0xff},
	{0x59d6, 0x0f},
	{0x59d7, 0xff},
	{0x59d8, 0x0f},
	{0x59d9, 0xff},
	{0x59da, 0x0f},
	{0x59db, 0xff},
	{0x59ef, 0x5f},
	{0x6901, 0x18},
	{0x6924, 0x00},
	{0x6925, 0x00},
	{0x6926, 0x00},
	{0x6942, 0x00},
	{0x6943, 0x00},
	{0x6944, 0x00},
	{0x694b, 0x00},
	{0x6a20, 0x03},
	{0x6a21, 0x04},
	{0x6a22, 0x00},
	{0x6a53, 0xfe},
	{0x6a54, 0xfe},
	{0x6a55, 0xfe},
	{0x6a56, 0xff},
	{0x6a57, 0xff},
	{0x6a58, 0xff},
	{0x6a7b, 0x16},
	{0x6aa7, 0x11},
	{0x6ac0, 0x3f},
	{0x6afd, 0x0a},
	{0x6b25, 0x00},
	{0x6b26, 0x00},
	{0x6b27, 0x00},
	{0x6b28, 0x00},
	{0x6b29, 0x00},
	{0x6b2c, 0x06},
	{0x6b2d, 0x00},
	{0x6b2e, 0x03},
	{0x6bc2, 0x00},
	{0x6bc3, 0xce},
	{0x6bc4, 0x01},
	{0x6bc5, 0x20},
	{0x6bc6, 0x01},
	{0x6bc7, 0x91},
	{0x6bc8, 0x02},
	{0x6bc9, 0x2f},
	{0x6bca, 0x03},
	{0x6bcb, 0x0a},
	{0x6bcc, 0x04},
	{0x6bcd, 0x3d},
	{0x6bce, 0x05},
	{0x6bcf, 0xe8},
	{0x6bd0, 0x08},
	{0x6bd1, 0x3c},
	{0x6bd2, 0x0b},
	{0x6bd3, 0x7a},
	{0x6bd4, 0x0f},
	{0x6bd5, 0xff},
	{0x6bd6, 0x0f},
	{0x6bd7, 0xff},
	{0x6bd8, 0x0f},
	{0x6bd9, 0xff},
	{0x6bda, 0x0f},
	{0x6bdb, 0xff},
	{0x6bef, 0x5f},
	{0xc200, 0x00},
	{0xc201, 0x00},
	{0xc202, 0x00},
	{0xc203, 0x00},
	{0xc210, 0x00},
	{0xc211, 0x00},
	{0xc212, 0x00},
	{0xc213, 0x00},
	{0xc214, 0x00},
	{0xc230, 0x00},
	{0xc231, 0x00},
	{0xc232, 0x00},
	{0xc233, 0x00},
	{0xc240, 0x00},
	{0xc241, 0x00},
	{0xc242, 0x00},
	{0xc243, 0x00},
	{0xc250, 0x00},
	{0xc251, 0x00},
	{0xc252, 0x00},
	{0xc253, 0x00},
	{0xc260, 0x00},
	{0xc261, 0x00},
	{0xc262, 0x00},
	{0xc263, 0x00},
	{0xc270, 0x00},
	{0xc271, 0x00},
	{0xc272, 0x00},
	{0xc273, 0x00},
	{0xc40e, 0xa0},
	{0xc418, 0x02},
	{0xc42f, 0x00},
	{0xc448, 0x00},
	{0xc44e, 0x03},
	{0xc44f, 0x03},
	{0xc450, 0x04},
	{0xc451, 0x04},
	{0xc46e, 0x01},
	{0xc478, 0x01},
	{0xc49c, 0x00},
	{0xc49d, 0x00},
	{0xc49e, 0x1c},
	{0xc49f, 0x30},
	{0xc4a2, 0x3a},
	{0xc4a3, 0x8e},
	{0xc4b9, 0x09},
	{0xc4bf, 0x01},
	{0xc4c1, 0x07},
	{0xc4c2, 0x07},
	{0xc4c3, 0x77},
	{0xc4c4, 0x77},
	{0xc4d2, 0x38},
	{0xc4d3, 0x38},
	{0xc4d4, 0x38},
	{0xc4d5, 0x38},
	{0xc4e3, 0x14},
	{0xc4e9, 0x20},
	{0xc4f8, 0x01},
	{0xc500, 0x01},
	{0xc506, 0x14},
	{0xc507, 0x02},
	{0xc50b, 0x77},
	{0xc50e, 0x00},
	{0xc50f, 0x00},
	{0xc510, 0x00},
	{0xc511, 0x00},
	{0xc512, 0x00},
	{0xc513, 0x4e},
	{0xc514, 0x4f},
	{0xc515, 0x2a},
	{0xc516, 0x16},
	{0xc517, 0x0b},
	{0xc518, 0x33},
	{0xc519, 0x33},
	{0xc51a, 0x33},
	{0xc51b, 0x33},
	{0xc51c, 0x33},
	{0xc51d, 0x37},
	{0xc51e, 0x37},
	{0xc51f, 0x3a},
	{0xc520, 0x3a},
	{0xc521, 0x3a},
	{0xc52e, 0x0e},
	{0xc52f, 0x0e},
	{0xc530, 0x0e},
	{0xc531, 0x0e},
	{0xc532, 0x0e},
	{0xc533, 0x0e},
	{0xc534, 0x0e},
	{0xc535, 0x0e},
	{0xc53a, 0x0e},
	{0xc53b, 0x0e},
	{0xc53c, 0x0e},
	{0xc53d, 0x0e},
	{0xc53e, 0x0e},
	{0xc53f, 0x0e},
	{0xc540, 0x0e},
	{0xc541, 0x0e},
	{0xc542, 0x0e},
	{0xc543, 0x0e},
	{0xc544, 0x0e},
	{0xc545, 0x0e},
	{0xc546, 0x0e},
	{0xc547, 0x0e},
	{0xc548, 0x0e},
	{0xc549, 0x0e},
	{0xc57d, 0x80},
	{0xc57f, 0x18},
	{0xc580, 0x18},
	{0xc581, 0x18},
	{0xc582, 0x18},
	{0xc583, 0x01},
	{0xc584, 0x01},
	{0xc586, 0x0a},
	{0xc587, 0x18},
	{0xc588, 0x18},
	{0xc589, 0x18},
	{0xc58a, 0x0c},
	{0xc58b, 0x08},
	{0xc58c, 0x04},
	{0xc58e, 0x0a},
	{0xc58f, 0x28},
	{0xc590, 0x28},
	{0xc591, 0x28},
	{0xc592, 0x28},
	{0xc593, 0x04},
	{0xc594, 0x04},
	{0xc597, 0x2c},
	{0xc598, 0x2c},
	{0xc599, 0x2c},
	{0xc59a, 0x28},
	{0xc59b, 0x20},
	{0xc59c, 0x18},
	{0xc5e3, 0x07},
	{0xc5e4, 0x00},
	{0xc5e5, 0x01},
	{0xc5e8, 0x01},
	{0xc5eb, 0x55},
	{0xc5ec, 0x05},
	{0xc624, 0xf8},
	{0xc638, 0x01},
	{0xc639, 0x00},
	{0xc63c, 0x01},
	{0xc63d, 0x00},
	{0xc640, 0x01},
	{0xc641, 0x00},
	{0xc64c, 0x08},
	{0xc64d, 0x08},
	{0xc64e, 0x08},
	{0xc64f, 0x08},
	{0xc650, 0x08},
	{0xc651, 0x08},
	{0xc664, 0x00},
	{0xc66b, 0x00},
	{0xc66c, 0x00},
	{0xc66d, 0x00},
	{0xc66e, 0x01},
	{0xc66f, 0x00},
	{0xc700, 0x80},
	{0xc702, 0x00},
	{0xc703, 0x00},
	{0xc726, 0x03},
	{0xc72b, 0xff},
	{0xc72c, 0xff},
	{0xc72d, 0xff},
	{0xc72f, 0x08},
	{0xc730, 0x00},
	{0xc731, 0x00},
	{0xc732, 0x00},
	{0xc733, 0x00},
	{0xc734, 0x00},
	{0xc735, 0x00},
	{0xc736, 0x01},
	{0xc739, 0x18},
	{0xc73a, 0x49},
	{0xc73b, 0x92},
	{0xc73c, 0x24},
	{0xc73d, 0x00},
	{0xc73e, 0x00},
	{0xc73f, 0x00},
	{0xc740, 0x00},
	{0xc741, 0x00},
	{0xc742, 0x00},
	{0xc743, 0x00},
	{0xc744, 0x00},
	{0xc745, 0x00},
	{0xc746, 0x01},
	{0xc747, 0x04},
	{0xc749, 0x1c},
	{0xc74c, 0x40},
	{0xc74e, 0x00},
	{0xc750, 0x55},
	{0xc751, 0x00},
	{0xc758, 0x40},
	{0xc75b, 0x01},
	{0xc75c, 0x05},
	{0xc765, 0x2a},
	{0xc773, 0x02},
	{0xc774, 0x03},
	{0xc78a, 0x03},
	{0xc78b, 0x04},
	{0xc797, 0x03},
	{0xc798, 0x03},
	{0xc79c, 0x00},
	{0xc79e, 0x01},
	{0xc7a0, 0x12},
	{0xc7a2, 0x01},
	{0xc7a3, 0x01},
	{0xc7a6, 0x02},
	{0xc7a7, 0xff},
	{0xc7a8, 0xff},
	{0xc7a9, 0xff},
	{0xc7aa, 0xff},
	{0xc7ab, 0xff},
	{0xc7ac, 0x02},
	{0xc7ad, 0xff},
	{0xc7ae, 0xff},
	{0xc7af, 0xff},
	{0xc7b0, 0xff},
	{0xc7b1, 0xff},
	{0xc7b2, 0x01},
	{0xc7b3, 0xff},
	{0xc7b4, 0xff},
	{0xc7b5, 0xff},
	{0xc7b6, 0xff},
	{0xc7c3, 0xff},
	{0xc7c4, 0x00},
	{0xc7c5, 0xff},
	{0xc7d9, 0x50},
	{0xc7da, 0xaa},
	{0xc7db, 0x0a},
	{0xc7dc, 0xa0},
	{0xc7e2, 0x01},
	{0xc7e4, 0x01},
	{0xc7e8, 0x12},
	{0xc7fd, 0x12},
	{0xc855, 0x07},
	{0xc8a4, 0x07},
	{0xc95a, 0x77},
	{0xc95b, 0x77},
	{0xc95c, 0x77},
	{0xc95d, 0x77},
	{0xc97b, 0x10},
	{0xc9a8, 0x1c},
	{0xc9b9, 0x28},
	{0xc9be, 0x01},
	{0xc9f3, 0x01},
	{0xc9fe, 0x0a},
	{0xc9ff, 0x0e},
	{0xca00, 0x1a},
	{0xca01, 0x1a},
	{0xca02, 0x1a},
	{0xca02, 0x1a},
	{0xca17, 0x03},
	{0xca18, 0x1a},
	{0xca19, 0x1a},
	{0xca1a, 0x1a},
	{0xca1b, 0x1a},
	{0xca22, 0x12},
	{0xca23, 0x12},
	{0xca24, 0x12},
	{0xca25, 0x12},
	{0xca26, 0x12},
	{0xca31, 0x12},
	{0xca32, 0x12},
	{0xca33, 0x12},
	{0xca34, 0x12},
	{0xca35, 0x12},
	{0xca36, 0x12},
	{0xca37, 0x12},
	{0xca38, 0x12},
	{0xca39, 0x12},
	{0xca3a, 0x12},
	{0xca45, 0x12},
	{0xca46, 0x12},
	{0xca47, 0x12},
	{0xca48, 0x12},
	{0xca49, 0x12},
	{0xcaab, 0x18},
	{0xcaca, 0x0f},
	{0xcada, 0x03},
	{REG_NULL, 0x00},
};

static const struct regval ov50h40_10bit_4096x3072_cphy_30fps_regs[] = { 
    {0x0304, 0x02}, 
    {0x0305, 0xd0}, 
    {0x0327, 0x0e}, 
    {0x0329, 0x01}, 
    {0x032c, 0x00}, 
    {0x0344, 0x01}, 
    {0x0345, 0x10}, 
    {0x0360, 0x09}, 
    {0x3027, 0x00}, 
    {0x3400, 0x0c}, 
    {0x3422, 0x08}, 
    {0x3423, 0x00}, 
    {0x3506, 0xf8}, 
    {0x350d, 0x00}, 
    {0x350e, 0xb2}, 
    {0x350f, 0x40}, 
    {0x3546, 0xf8}, 
    {0x354d, 0x00}, 
    {0x354e, 0xb2}, 
    {0x354f, 0x40}, 
    {0x3586, 0xf8}, 
    {0x358d, 0x00}, 
    {0x358e, 0xb2}, 
    {0x358f, 0x40}, 
    {0x3609, 0x80}, 
    {0x360c, 0x4f}, 
    {0x3610, 0x08}, 
    {0x3614, 0x10}, 
    {0x3618, 0xcf}, 
    {0x3619, 0x40}, 
    {0x361a, 0x01}, 
    {0x361d, 0x1f}, 
    {0x363b, 0x9f}, 
    {0x363c, 0x6e}, 
    {0x3640, 0x00}, 
    {0x3641, 0x02}, 
    {0x3644, 0x00}, 
    {0x3645, 0x06}, 
    {0x3647, 0x01}, 
    {0x3650, 0xbf}, 
    {0x3653, 0x03}, 
    {0x3680, 0x00}, 
    {0x3682, 0x80}, 
    {0x3684, 0x01}, 
    {0x3688, 0x00}, 
    {0x368a, 0x0e}, 
    {0x3696, 0x41}, 
    {0x369a, 0x00}, 
    {0x36d0, 0x00}, 
    {0x36d3, 0x40}, 
    {0x3700, 0x1c}, 
    {0x3701, 0x13}, 
    {0x3704, 0x03}, 
    {0x3706, 0x34}, 
    {0x3707, 0x04}, 
    {0x3709, 0x7c}, 
    {0x370b, 0x94}, 
    {0x3712, 0x00}, 
    {0x3714, 0xf2}, 
    {0x3716, 0x40}, 
    {0x3722, 0x05}, 
    {0x3724, 0x08}, 
    {0x372b, 0x00}, 
    {0x372e, 0x1c}, 
    {0x372f, 0x13}, 
    {0x373f, 0x00}, 
    {0x374f, 0x58}, 
    {0x3755, 0x7c}, 
    {0x3757, 0x7f}, 
    {0x3759, 0x50}, 
    {0x375e, 0x0d}, 
    {0x375f, 0x00}, 
    {0x3770, 0x04}, 
    {0x3780, 0x5e}, 
    {0x3782, 0x01}, 
    {0x378a, 0x01}, 
    {0x3791, 0x34}, 
    {0x3793, 0x1c}, 
    {0x3795, 0x1c}, 
    {0x3797, 0x94}, 
    {0x3799, 0x3a}, 
    {0x379b, 0x3a}, 
    {0x379c, 0x01}, 
    {0x379f, 0x01}, 
    {0x37a0, 0x9b}, 
    {0x37a9, 0x01}, 
    {0x37b2, 0xc8}, 
    {0x37b7, 0x02}, 
    {0x37bd, 0x00}, 
    {0x37c1, 0x1a}, 
    {0x37c3, 0x1a}, 
    {0x37cb, 0x02}, 
    {0x37cd, 0x02}, 
    {0x37d0, 0x22}, 
    {0x37d4, 0x00}, 
    {0x37db, 0x10}, 
    {0x37dc, 0x1a}, 
    {0x37e3, 0x30}, 
    {0x37f0, 0x01}, 
    {0x37f6, 0x1a}, 
    {0x3800, 0x00}, 
    {0x3801, 0x00}, 
    {0x3802, 0x00}, 
    {0x3803, 0x00}, 
    {0x3804, 0x20}, 
    {0x3805, 0x1f}, 
    {0x3806, 0x18}, 
    {0x3807, 0x3f}, 
    {0x3808, 0x10}, 
    {0x3809, 0x00}, 
    {0x380a, 0x0c}, 
    {0x380b, 0x00}, 
    {0x380c, 0x04}, 
    {0x380d, 0x4c}, 
    {0x380e, 0x08}, 
    {0x380f, 0xe0}, 
    {0x3810, 0x00}, 
    {0x3811, 0x07}, 
    {0x3813, 0x10}, 
    {0x3815, 0x11}, 
    {0x3820, 0x46}, 
    {0x3821, 0x10}, 
    {0x3822, 0x10}, 
    {0x3823, 0x04}, 
    {0x3827, 0x40}, 
    {0x3828, 0x21}, 
    {0x3830, 0x20}, 
    {0x3831, 0x12}, 
    {0x3837, 0x20}, 
    {0x383f, 0x08}, 
    {0x384c, 0x04}, 
    {0x384d, 0x4c}, 
    {0x3888, 0x00}, 
    {0x3889, 0x08}, 
    {0x388b, 0x10}, 
    {0x388c, 0x10}, 
    {0x388d, 0x00}, 
    {0x388e, 0x0c}, 
    {0x388f, 0x00}, 
    {0x3896, 0x00}, 
    {0x38db, 0x08}, 
    {0x38dd, 0x04}, 
    {0x38de, 0x03}, 
    {0x38df, 0x08}, 
    {0x3906, 0x24}, 
    {0x390a, 0x15}, 
    {0x3919, 0x11}, 
    {0x3982, 0x40}, 
    {0x398b, 0x00}, 
    {0x399d, 0x13}, 
    {0x39dc, 0x00}, 
    {0x39fb, 0x01}, 
    {0x39fc, 0x01}, 
    {0x39fd, 0x01}, 
    {0x39fe, 0x01}, 
    {0x3a1d, 0x01}, 
    {0x3a1e, 0x01}, 
    {0x3a21, 0x01}, 
    {0x3a22, 0x01}, 
    {0x3a68, 0x13}, 
    {0x3a69, 0x20}, 
    {0x3ab6, 0x01}, 
    {0x3ab7, 0x01}, 
    {0x3af2, 0x03}, 
    {0x3b01, 0x1d}, 
    {0x3b02, 0x00}, 
    {0x3b3d, 0x07}, 
    {0x3b4a, 0x00}, 
    {0x3b4b, 0x00}, 
    {0x3b56, 0x1f}, 
    {0x3b57, 0x1f}, 
    {0x3b58, 0x20}, 
    {0x3b59, 0x20}, 
    {0x3b5a, 0x19}, 
    {0x3b5b, 0x19}, 
    {0x3b5c, 0x19}, 
    {0x3b5d, 0x19}, 
    {0x3b82, 0x19}, 
    {0x3ba1, 0x1e}, 
    {0x3ba6, 0x77}, 
    {0x3ba7, 0x77}, 
    {0x3baa, 0x33}, 
    {0x3bab, 0x2f}, 
    {0x3baf, 0x16}, 
    {0x3bba, 0x48}, 
    {0x3bf3, 0x01}, 
    {0x3bfb, 0x01}, 
    {0x3bfc, 0x50}, 
    {0x3bff, 0x08}, 
    {0x400e, 0x1c}, 
    {0x4010, 0x34}, 
    {0x4012, 0x17}, 
    {0x4015, 0x08}, 
    {0x4016, 0x17}, 
    {0x4018, 0x07}, 
    {0x4506, 0x01}, 
    {0x4509, 0x07}, 
    {0x450c, 0x00}, 
    {0x450d, 0x60}, 
    {0x4510, 0x03}, 
    {0x4516, 0x55}, 
    {0x4517, 0x55}, 
    {0x4518, 0x55}, 
    {0x4519, 0x55}, 
    {0x451a, 0xaa}, 
    {0x451b, 0xaa}, 
    {0x451c, 0xaa}, 
    {0x451d, 0xaa}, 
    {0x451e, 0xff}, 
    {0x451f, 0xff}, 
    {0x4520, 0xff}, 
    {0x4521, 0xff}, 
    {0x4522, 0x29}, 
    {0x4523, 0x08}, 
    {0x4524, 0xbb}, 
    {0x4525, 0x0c}, 
    {0x4545, 0x00}, 
    {0x4546, 0x03}, 
    {0x4547, 0x9a}, 
    {0x4549, 0x00}, 
    {0x454a, 0x29}, 
    {0x454b, 0x08}, 
    {0x454c, 0xbb}, 
    {0x454d, 0x0c}, 
    {0x454e, 0x29}, 
    {0x454f, 0x08}, 
    {0x4550, 0xbb}, 
    {0x4551, 0x0c}, 
    {0x4552, 0x29}, 
    {0x4553, 0x08}, 
    {0x4554, 0xbb}, 
    {0x4555, 0x0c}, 
    {0x4556, 0x29}, 
    {0x4557, 0x08}, 
    {0x4558, 0xbb}, 
    {0x4559, 0x0c}, 
    {0x455a, 0x29}, 
    {0x455b, 0x08}, 
    {0x455c, 0xbb}, 
    {0x455d, 0x0c}, 
    {0x455e, 0x29}, 
    {0x455f, 0x08}, 
    {0x4560, 0xbb}, 
    {0x4561, 0x0c}, 
    {0x4562, 0x29}, 
    {0x4563, 0x08}, 
    {0x4564, 0xbb}, 
    {0x4565, 0x0c}, 
    {0x45c0, 0x8e}, 
    {0x45c1, 0x80}, 
    {0x45c2, 0x0a}, 
    {0x45c3, 0x04}, 
    {0x45c4, 0x13}, 
    {0x45c5, 0x40}, 
    {0x45c6, 0x01}, 
    {0x4602, 0x00}, 
    {0x4603, 0x15}, 
    {0x460b, 0x07}, 
    {0x4640, 0x01}, 
    {0x4641, 0x00}, 
    {0x4643, 0x0c}, 
    {0x4680, 0x11}, 
    {0x4684, 0x2b}, 
    {0x468e, 0x30}, 
    {0x4813, 0x10}, 
    {0x4836, 0x32}, 
    {0x4837, 0x04}, 
    {0x49f5, 0x00}, 
    {0x5000, 0x2b}, 
    {0x5001, 0x08}, 
    {0x5002, 0x00}, 
    {0x5007, 0x06}, 
    {0x5009, 0x40}, 
    {0x5091, 0x00}, 
    {0x5180, 0xc0}, 
    {0x5480, 0xc0}, 
    {0x5780, 0xc0}, 
    {0x6a03, 0x00}, 
    {0xc200, 0x00}, 
    {0xc201, 0x00}, 
    {0xc202, 0x00}, 
    {0xc203, 0x00}, 
    {0xc210, 0x00}, 
    {0xc211, 0x00}, 
    {0xc212, 0x00}, 
    {0xc213, 0x00}, 
    {0xc214, 0x00}, 
    {0xc230, 0x00}, 
    {0xc231, 0x00}, 
    {0xc232, 0x00}, 
    {0xc233, 0x00}, 
    {0xc240, 0x00}, 
    {0xc241, 0x00}, 
    {0xc242, 0x00}, 
    {0xc243, 0x00}, 
    {0xc250, 0x00}, 
    {0xc251, 0x00}, 
    {0xc252, 0x00}, 
    {0xc253, 0x00}, 
    {0xc260, 0x00}, 
    {0xc261, 0x00}, 
    {0xc262, 0x00}, 
    {0xc263, 0x00}, 
    {0xc270, 0x00}, 
    {0xc271, 0x00}, 
    {0xc272, 0x00}, 
    {0xc273, 0x00}, 
    {0xc40e, 0x00}, 
    {0xc448, 0x00}, 
    {0xc46e, 0x01}, 
    {0xc478, 0x01}, 
    {0xc49e, 0x34}, 
    {0xc49f, 0x34}, 
    {0xc4a2, 0x94}, 
    {0xc4a3, 0x94}, 
    {0xc4c1, 0x07}, 
    {0xc4c2, 0x07}, 
    {0xc4c3, 0x77}, 
    {0xc4c4, 0x77}, 
    {0xc4d2, 0x00}, 
    {0xc4d3, 0x00}, 
    {0xc4d4, 0x00}, 
    {0xc4d5, 0x00}, 
    {0xc4e3, 0x19}, 
    {0xc4e9, 0x1e}, 
    {0xc506, 0x16}, 
    {0xc50e, 0x1f}, 
    {0xc50f, 0x1f}, 
    {0xc510, 0x0f}, 
    {0xc511, 0x07}, 
    {0xc512, 0x03}, 
    {0xc513, 0x4e}, 
    {0xc514, 0x4e}, 
    {0xc515, 0x27}, 
    {0xc516, 0x16}, 
    {0xc517, 0x0c}, 
    {0xc518, 0x33}, 
    {0xc519, 0x33}, 
    {0xc51a, 0x33}, 
    {0xc51b, 0x3b}, 
    {0xc51c, 0x3b}, 
    {0xc51d, 0x2f}, 
    {0xc51e, 0x2f}, 
    {0xc51f, 0x2f}, 
    {0xc520, 0x2f}, 
    {0xc521, 0x30}, 
    {0xc52e, 0x0e}, 
    {0xc52f, 0x0e}, 
    {0xc530, 0x0e}, 
    {0xc531, 0x0e}, 
    {0xc532, 0x0e}, 
    {0xc533, 0x0e}, 
    {0xc534, 0x0e}, 
    {0xc535, 0x0e}, 
    {0xc542, 0x0e}, 
    {0xc543, 0x0e}, 
    {0xc544, 0x0e}, 
    {0xc545, 0x0e}, 
    {0xc546, 0x0e}, 
    {0xc547, 0x0e}, 
    {0xc548, 0x0e}, 
    {0xc549, 0x0e}, 
    {0xc57d, 0x00}, 
    {0xc581, 0x18}, 
    {0xc582, 0x18}, 
    {0xc583, 0x02}, 
    {0xc584, 0x01}, 
    {0xc587, 0x18}, 
    {0xc589, 0x18}, 
    {0xc58a, 0x10}, 
    {0xc58b, 0x08}, 
    {0xc58c, 0x01}, 
    {0xc58f, 0x28}, 
    {0xc590, 0x28}, 
    {0xc591, 0x28}, 
    {0xc592, 0x28}, 
    {0xc593, 0x0a}, 
    {0xc594, 0x06}, 
    {0xc597, 0x2e}, 
    {0xc598, 0x2e}, 
    {0xc599, 0x2e}, 
    {0xc59a, 0x18}, 
    {0xc59b, 0x0e}, 
    {0xc59c, 0x08}, 
    {0xc5e4, 0x00}, 
    {0xc5e5, 0x07}, 
    {0xc5e8, 0x01}, 
    {0xc702, 0x10}, 
    {0xc726, 0x03}, 
    {0xc72b, 0xff}, 
    {0xc72c, 0xff}, 
    {0xc72d, 0xff}, 
    {0xc72f, 0x08}, 
    {0xc736, 0x01}, 
    {0xc739, 0x18}, 
    {0xc73a, 0xa6}, 
    {0xc73b, 0x00}, 
    {0xc73c, 0x00}, 
    {0xc746, 0x01}, 
    {0xc747, 0x04}, 
    {0xc749, 0x1c}, 
    {0xc75b, 0x01}, 
    {0xc75c, 0x05}, 
    {0xc765, 0x2a}, 
    {0xc773, 0x02}, 
    {0xc774, 0x03}, 
    {0xc78a, 0x03}, 
    {0xc78b, 0x04}, 
    {0xc798, 0x03}, 
    {0xc7a2, 0x01}, 
    {0xc7a6, 0x02}, 
    {0xc7a7, 0x02}, 
    {0xc7a8, 0xff}, 
    {0xc7a9, 0xff}, 
    {0xc7aa, 0xff}, 
    {0xc7ac, 0x02}, 
    {0xc7ad, 0x08}, 
    {0xc7ae, 0xff}, 
    {0xc7af, 0xff}, 
    {0xc7b0, 0xff}, 
    {0xc7b2, 0x01}, 
    {0xc7b3, 0x02}, 
    {0xc7b4, 0xff}, 
    {0xc7b5, 0xff}, 
    {0xc7b6, 0xff}, 
    {0xc7c4, 0x01}, 
    {0xc7c5, 0x00}, 
    {0xc7e2, 0x01}, 
    {0xc855, 0x77}, 
    {0xc8a4, 0x77}, 
    {0xc95a, 0x77}, 
    {0xc95b, 0x77}, 
    {0xc9b9, 0x18}, 
    {0xc9fe, 0x0a}, 
    {0xc9ff, 0x12}, 
    {0xca00, 0x1a}, 
    {0xca02, 0x1a}, 
    {0xca17, 0x04}, 
    {0xca18, 0x1a}, 
    {0xca19, 0x1a}, 
    {0x3501, 0x08}, 
    {0x3502, 0x00}, 
    {0x3508, 0x01}, 
    {0x3509, 0x00},
	{REG_NULL, 0x00},
};

static const struct regval ov50h40_10bit_8192x6144_cphy_30fps_regs[] = {
	{0x0304, 0x02},
	{0x0305, 0xd0},
	{0x0327, 0x0e},
	{0x0329, 0x01},
	{0x032c, 0x00},
	{0x0344, 0x01},
	{0x0345, 0x20},
	{0x0360, 0x09},
	{0x3027, 0x00},
	{0x3400, 0x0c},
	{0x3422, 0x08},
	{0x3423, 0x00},
	{0x3506, 0x78},
	{0x350d, 0x01},
	{0x350e, 0x00},
	{0x350f, 0x00},
	{0x3546, 0x78},
	{0x354d, 0x01},
	{0x354e, 0x00},
	{0x354f, 0x00},
	{0x3586, 0x78},
	{0x358d, 0x01},
	{0x358e, 0x00},
	{0x358f, 0x00},
	{0x3609, 0x80},
	{0x360c, 0x0f},
	{0x3610, 0x08},
	{0x3614, 0x0c},
	{0x3618, 0xcf},
	{0x3619, 0x44},
	{0x361a, 0x81},
	{0x361d, 0x1f},
	{0x363b, 0x6a},
	{0x363c, 0x6a},
	{0x3640, 0x00},
	{0x3641, 0x02},
	{0x3644, 0x00},
	{0x3645, 0x06},
	{0x3647, 0x01},
	{0x3650, 0xbf},
	{0x3653, 0x03},
	{0x3680, 0x00},
	{0x3682, 0x80},
	{0x3684, 0x00},
	{0x3688, 0x00},
	{0x368a, 0x0e},
	{0x3696, 0x41},
	{0x369a, 0x00},
	{0x36d0, 0x00},
	{0x36d3, 0x80},
	{0x3700, 0x1c},
	{0x3701, 0x13},
	{0x3704, 0x03},
	{0x3706, 0x1c},
	{0x3707, 0x04},
	{0x3709, 0x70},
	{0x370b, 0x3a},
	{0x3712, 0x01},
	{0x3714, 0xf8},
	{0x3716, 0x40},
	{0x3722, 0x05},
	{0x3724, 0x5d},
	{0x372b, 0x00},
	{0x372e, 0x1c},
	{0x372f, 0x13},
	{0x373f, 0x00},
	{0x374f, 0x58},
	{0x3755, 0xb1},
	{0x3757, 0x30},
	{0x3759, 0x50},
	{0x375e, 0x00},
	{0x375f, 0x00},
	{0x3770, 0x01},
	{0x3780, 0x5c},
	{0x3782, 0x01},
	{0x378a, 0x01},
	{0x3791, 0x30},
	{0x3793, 0x1c},
	{0x3795, 0x1c},
	{0x3797, 0x8e},
	{0x3799, 0x3a},
	{0x379b, 0x3a},
	{0x379c, 0x01},
	{0x379f, 0x01},
	{0x37a0, 0x70},
	{0x37a9, 0x01},
	{0x37b2, 0xc8},
	{0x37b7, 0x02},
	{0x37bd, 0x00},
	{0x37c1, 0x1a},
	{0x37c3, 0x1a},
	{0x37cb, 0x02},
	{0x37cd, 0x01},
	{0x37d0, 0x00},
	{0x37d4, 0x00},
	{0x37db, 0x10},
	{0x37dc, 0x1a},
	{0x37e3, 0x30},
	{0x37f0, 0x01},
	{0x37f6, 0x1a},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x20},
	{0x3805, 0x1f},
	{0x3806, 0x18},
	{0x3807, 0x3f},
	{0x3808, 0x20},
	{0x3809, 0x00},
	{0x380a, 0x18},
	{0x380b, 0x00},
	{0x380c, 0x03},
	{0x380d, 0x06},
	{0x380e, 0x0c},
	{0x380f, 0x96},
	{0x3810, 0x00},
	{0x3811, 0x0f},
	{0x3813, 0x20},
	{0x3815, 0x11},
	{0x3820, 0x44},
	{0x3821, 0x00},
	{0x3822, 0x00},
	{0x3823, 0x04},
	{0x3827, 0x40},
	{0x3828, 0x27},
	{0x3830, 0x20},
	{0x3831, 0x10},
	{0x3837, 0x20},
	{0x383f, 0x08},
	{0x384c, 0x03},
	{0x384d, 0x06},
	{0x3888, 0x00},
	{0x3889, 0x10},
	{0x388b, 0x20},
	{0x388c, 0x20},
	{0x388d, 0x00},
	{0x388e, 0x18},
	{0x388f, 0x00},
	{0x3896, 0x00},
	{0x38db, 0x20},
	{0x38dd, 0x10},
	{0x38de, 0x0c},
	{0x38df, 0x20},
	{0x3906, 0x24},
	{0x390a, 0x05},
	{0x3919, 0x15},
	{0x3982, 0x40},
	{0x398b, 0x00},
	{0x399d, 0x05},
	{0x39dc, 0x01},
	{0x39fb, 0x01},
	{0x39fc, 0x01},
	{0x39fd, 0x06},
	{0x39fe, 0x06},
	{0x3a1d, 0x01},
	{0x3a1e, 0x01},
	{0x3a21, 0x01},
	{0x3a22, 0x06},
	{0x3a68, 0x05},
	{0x3a69, 0x20},
	{0x3ab6, 0x01},
	{0x3ab7, 0x01},
	{0x3af2, 0x03},
	{0x3b01, 0x00},
	{0x3b02, 0x00},
	{0x3b3d, 0x07},
	{0x3b4a, 0x38},
	{0x3b4b, 0x38},
	{0x3b56, 0x20},
	{0x3b57, 0x21},
	{0x3b58, 0x21},
	{0x3b59, 0x21},
	{0x3b5a, 0x14},
	{0x3b5b, 0x14},
	{0x3b5c, 0x14},
	{0x3b5d, 0x14},
	{0x3b82, 0x14},
	{0x3ba1, 0x20},
	{0x3ba6, 0x00},
	{0x3ba7, 0x00},
	{0x3baa, 0x33},
	{0x3bab, 0x37},
	{0x3baf, 0x00},
	{0x3bba, 0x4c},
	{0x3bf3, 0x01},
	{0x3bfb, 0x01},
	{0x3bfc, 0x50},
	{0x3bff, 0x08},
	{0x400e, 0x14},
	{0x4010, 0x34},
	{0x4012, 0x17},
	{0x4015, 0x10},
	{0x4016, 0x2f},
	{0x4018, 0x0f},
	{0x4506, 0x01},
	{0x4509, 0x07},
	{0x450c, 0x00},
	{0x450d, 0x30},
	{0x4510, 0x00},
	{0x4516, 0x00},
	{0x4517, 0x00},
	{0x4518, 0x00},
	{0x4519, 0x00},
	{0x451a, 0x00},
	{0x451b, 0x00},
	{0x451c, 0x00},
	{0x451d, 0x00},
	{0x451e, 0x00},
	{0x451f, 0x00},
	{0x4520, 0x00},
	{0x4521, 0x00},
	{0x4522, 0x00},
	{0x4523, 0x00},
	{0x4524, 0x00},
	{0x4525, 0x00},
	{0x4545, 0x00},
	{0x4546, 0x04},
	{0x4547, 0xcc},
	{0x4549, 0x00},
	{0x454a, 0x00},
	{0x454b, 0x00},
	{0x454c, 0x00},
	{0x454d, 0x00},
	{0x454e, 0x00},
	{0x454f, 0x00},
	{0x4550, 0x00},
	{0x4551, 0x00},
	{0x4552, 0x00},
	{0x4553, 0x00},
	{0x4554, 0x00},
	{0x4555, 0x00},
	{0x4556, 0x00},
	{0x4557, 0x00},
	{0x4558, 0x00},
	{0x4559, 0x00},
	{0x455a, 0x00},
	{0x455b, 0x00},
	{0x455c, 0x00},
	{0x455d, 0x00},
	{0x455e, 0x00},
	{0x455f, 0x00},
	{0x4560, 0x00},
	{0x4561, 0x00},
	{0x4562, 0x00},
	{0x4563, 0x00},
	{0x4564, 0x00},
	{0x4565, 0x00},
	{0x45c0, 0x9c},
	{0x45c1, 0x80},
	{0x45c2, 0x0a},
	{0x45c3, 0x04},
	{0x45c4, 0x13},
	{0x45c5, 0x80},
	{0x45c6, 0x08},
	{0x4602, 0x00},
	{0x4603, 0x15},
	{0x460b, 0x07},
	{0x4640, 0x01},
	{0x4641, 0x00},
	{0x4643, 0x08},
	{0x4680, 0x11},
	{0x4684, 0x2b},
	{0x468e, 0x30},
	{0x4813, 0x10},
	{0x4836, 0x32},
	{0x4837, 0x04},
	{0x49f5, 0x00},
	{0x5000, 0x5b},
	{0x5001, 0x28},
	{0x5002, 0x00},
	{0x5007, 0x06},
	{0x5009, 0x2e},
	{0x5091, 0x00},
	{0x5180, 0xc0},
	{0x5480, 0xc0},
	{0x5780, 0xc0},
	{0x6a03, 0x00},
	{0xc200, 0x00},
	{0xc201, 0x00},
	{0xc202, 0x00},
	{0xc203, 0x00},
	{0xc210, 0x00},
	{0xc211, 0x00},
	{0xc212, 0x00},
	{0xc213, 0x00},
	{0xc214, 0x00},
	{0xc230, 0x00},
	{0xc231, 0x00},
	{0xc232, 0x00},
	{0xc233, 0x00},
	{0xc240, 0x00},
	{0xc241, 0x00},
	{0xc242, 0x00},
	{0xc243, 0x00},
	{0xc250, 0x00},
	{0xc251, 0x00},
	{0xc252, 0x00},
	{0xc253, 0x00},
	{0xc260, 0x00},
	{0xc261, 0x00},
	{0xc262, 0x00},
	{0xc263, 0x00},
	{0xc270, 0x00},
	{0xc271, 0x00},
	{0xc272, 0x00},
	{0xc273, 0x00},
	{0xc40e, 0xa0},
	{0xc448, 0x00},
	{0xc46e, 0x01},
	{0xc478, 0x01},
	{0xc49e, 0x1c},
	{0xc49f, 0x30},
	{0xc4a2, 0x3a},
	{0xc4a3, 0x8e},
	{0xc4c1, 0x07},
	{0xc4c2, 0x07},
	{0xc4c3, 0x77},
	{0xc4c4, 0x77},
	{0xc4d2, 0x38},
	{0xc4d3, 0x38},
	{0xc4d4, 0x38},
	{0xc4d5, 0x38},
	{0xc4e3, 0x14},
	{0xc4e9, 0x20},
	{0xc506, 0x14},
	{0xc50e, 0x00},
	{0xc50f, 0x00},
	{0xc510, 0x00},
	{0xc511, 0x00},
	{0xc512, 0x00},
	{0xc513, 0x4e},
	{0xc514, 0x4f},
	{0xc515, 0x2a},
	{0xc516, 0x16},
	{0xc517, 0x0b},
	{0xc518, 0x33},
	{0xc519, 0x33},
	{0xc51a, 0x33},
	{0xc51b, 0x33},
	{0xc51c, 0x33},
	{0xc51d, 0x37},
	{0xc51e, 0x37},
	{0xc51f, 0x3a},
	{0xc520, 0x3a},
	{0xc521, 0x3a},
	{0xc52e, 0x0e},
	{0xc52f, 0x0e},
	{0xc530, 0x0e},
	{0xc531, 0x0e},
	{0xc532, 0x0e},
	{0xc533, 0x0e},
	{0xc534, 0x0e},
	{0xc535, 0x0e},
	{0xc542, 0x0e},
	{0xc543, 0x0e},
	{0xc544, 0x0e},
	{0xc545, 0x0e},
	{0xc546, 0x0e},
	{0xc547, 0x0e},
	{0xc548, 0x0e},
	{0xc549, 0x0e},
	{0xc57d, 0x80},
	{0xc581, 0x18},
	{0xc582, 0x18},
	{0xc583, 0x01},
	{0xc584, 0x01},
	{0xc587, 0x18},
	{0xc589, 0x18},
	{0xc58a, 0x0c},
	{0xc58b, 0x08},
	{0xc58c, 0x04},
	{0xc58f, 0x28},
	{0xc590, 0x28},
	{0xc591, 0x28},
	{0xc592, 0x28},
	{0xc593, 0x04},
	{0xc594, 0x04},
	{0xc597, 0x2c},
	{0xc598, 0x2c},
	{0xc599, 0x2c},
	{0xc59a, 0x28},
	{0xc59b, 0x20},
	{0xc59c, 0x18},
	{0xc5e4, 0x00},
	{0xc5e5, 0x01},
	{0xc5e8, 0x01},
	{0xc702, 0x00},
	{0xc726, 0x03},
	{0xc72b, 0xff},
	{0xc72c, 0xff},
	{0xc72d, 0xff},
	{0xc72f, 0x08},
	{0xc736, 0x01},
	{0xc739, 0x18},
	{0xc73a, 0x49},
	{0xc73b, 0x92},
	{0xc73c, 0x24},
	{0xc746, 0x01},
	{0xc747, 0x04},
	{0xc749, 0x1c},
	{0xc75b, 0x01},
	{0xc75c, 0x05},
	{0xc765, 0x2a},
	{0xc773, 0x02},
	{0xc774, 0x03},
	{0xc78a, 0x03},
	{0xc78b, 0x04},
	{0xc798, 0x03},
	{0xc7a2, 0x01},
	{0xc7a6, 0x02},
	{0xc7a7, 0xff},
	{0xc7a8, 0xff},
	{0xc7a9, 0xff},
	{0xc7aa, 0xff},
	{0xc7ac, 0x02},
	{0xc7ad, 0xff},
	{0xc7ae, 0xff},
	{0xc7af, 0xff},
	{0xc7b0, 0xff},
	{0xc7b2, 0x01},
	{0xc7b3, 0xff},
	{0xc7b4, 0xff},
	{0xc7b5, 0xff},
	{0xc7b6, 0xff},
	{0xc7c4, 0x00},
	{0xc7c5, 0xff},
	{0xc7e2, 0x01},
	{0xc855, 0x07},
	{0xc8a4, 0x07},
	{0xc95a, 0x77},
	{0xc95b, 0x77},
	{0xc9b9, 0x28},
	{0xc9fe, 0x0a},
	{0xc9ff, 0x0e},
	{0xca00, 0x1a},
	{0xca02, 0x1a},
	{0xca17, 0x03},
	{0xca18, 0x1a},
	{0xca19, 0x1a},
	{0x3501, 0x0c},
	{0x3502, 0x00},
	{0x3508, 0x01},
	{0x3509, 0x00},
	{REG_NULL, 0x00},
};

static const struct other_data ov50h40_spd = {
	.width = 4096,
	.height = 768,
	.bus_fmt = MEDIA_BUS_FMT_SPD_2X8,
	.data_type = 0x19,
	.data_bit = 10,
};

/*
 * The width and height must be configured to be
 * the same as the current output resolution of the sensor.
 * The input width of the isp needs to be 16 aligned.
 * The input height of the isp needs to be 8 aligned.
 * If the width or height does not meet the alignment rules,
 * you can configure the cropping parameters with the following function to
 * crop out the appropriate resolution.
 * struct v4l2_subdev_pad_ops {
 *	.get_selection
 * }
 */
static const struct ov50h40_mode supported_modes_dphy[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 4096,
		.height = 3072,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0840,
		.hts_def = 0x41a * 4,
		.vts_def = 0x0c66,
		.mipi_freq_idx = 2,
		.bpp = 10,
		.reg_list = ov50h40_10bit_4096x3072_dphy_30fps_regs,
		.hdr_mode = NO_HDR,
		.spd = &ov50h40_spd,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 8192,
		.height = 6144,
		.max_fps = {
			.numerator = 10000,
			.denominator = 120000,
		},
		.exp_def = 0x0240,
		.hts_def = 0x9f6 * 4,
		.vts_def = 0x0cc3 * 2,
		.mipi_freq_idx = 3,
		.bpp = 10,
		.reg_list = ov50h40_10bit_8192x6144_dphy_12fps_regs,
		.hdr_mode = NO_HDR,
		.spd = &ov50h40_spd,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
#ifdef DEBUG
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 4096,
		.height = 3072,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 0x0240,
		.hts_def = 0x0834 * 4,
		.vts_def = 0x0c66,
		.mipi_freq_idx = 1,
		.bpp = 10,
		.reg_list = ov50h40_10bit_4096x3072_dphy_regs,
		.hdr_mode = NO_HDR,
		.spd = &ov50h40_spd,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 8192,
		.height = 6144,
		.max_fps = {
			.numerator = 10000,
			.denominator = 30000,
		},
		.exp_def = 0x0240,
		.hts_def = 0x09f6 * 4,
		.vts_def = 0x0cc3 * 2,
		.mipi_freq_idx = 1,
		.bpp = 10,
		.reg_list = ov50h40_10bit_8192x6144_dphy_regs,
		.hdr_mode = NO_HDR,
		.spd = &ov50h40_spd,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 4096,
		.height = 3072,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0840,
		.hts_def = 0x3e8 * 8,
		.vts_def = 0x0d05,
		.mipi_freq_idx = 2,
		.bpp = 10,
		.reg_list = ov50h40_10bit_4096x3072_dphy_30fps_nopd_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
#endif
};

static const struct ov50h40_mode supported_modes_cphy[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 4096,
		.height = 3072,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 0x0C00,
		.hts_def = 0x044c,
		.vts_def = 0x08e0,
		.mipi_freq_idx = 0,
		.bpp = 10,
		.reg_list = ov50h40_10bit_4096x3072_cphy_regs,
		.hdr_mode = NO_HDR,
		.spd = &ov50h40_spd,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 4096,
		.height = 3072,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0c00,
		.hts_def = 0x044c,
		.vts_def = 0x08e0,
		.mipi_freq_idx = 2,
		.bpp = 10,
		.reg_list = ov50h40_10bit_4096x3072_cphy_30fps_regs,
		.hdr_mode = NO_HDR,
		.spd = &ov50h40_spd,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 8192,
		.height = 6144,
		.max_fps = {
			.numerator = 10000,
			.denominator = 120000,
		},
		.exp_def = 0x0c00,
		.hts_def = 0x0306,
		.vts_def = 0x0c96,
		.mipi_freq_idx = 3,
		.bpp = 10,
		.reg_list = ov50h40_10bit_8192x6144_cphy_30fps_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_items[] = {
	MIPI_FREQ_356M,
	MIPI_FREQ_384M,
	MIPI_FREQ_750M,
	MIPI_FREQ_1250M,
};

static const char * const ov50h40_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

static int __ov50h40_power_on(struct ov50h40 *ov50h40);

/* Write registers up to 4 at a time */
static int ov50h40_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2) {
		dev_err(&client->dev, "Failed to write 0x%04x,0x%x\n", reg, val);
		return -EIO;
	}
	return 0;
}

static int ov50h40_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret |= ov50h40_write_reg(client, regs[i].addr,
			OV50H40_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int ov50h40_read_reg(struct i2c_client *client,
			    u16 reg,
			    unsigned int len,
			    u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int ov50h40_get_reso_dist(const struct ov50h40_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ov50h40_mode *
ov50h40_find_best_fit(struct ov50h40 *ov50h40, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ov50h40->cfg_num; i++) {
		dist = ov50h40_get_reso_dist(&ov50h40->support_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist < cur_best_fit_dist) &&
			(ov50h40->support_modes[i].bus_fmt == framefmt->code)) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}
	dev_info(&ov50h40->client->dev, "%s: cur_best_fit(%d)",
		 __func__, cur_best_fit);
	return &ov50h40->support_modes[cur_best_fit];
}

static int ov50h40_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	const struct ov50h40_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	u32 lane_num = ov50h40->bus_cfg.bus.mipi_csi2.num_data_lanes;

	mutex_lock(&ov50h40->mutex);

	mode = ov50h40_find_best_fit(ov50h40, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ov50h40->mutex);
		return -ENOTTY;
#endif
	} else {
		ov50h40->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov50h40->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov50h40->vblank, vblank_def,
					 OV50H40_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(ov50h40->vblank, vblank_def);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * lane_num;
		__v4l2_ctrl_s_ctrl_int64(ov50h40->pixel_rate,
					 pixel_rate);
		__v4l2_ctrl_s_ctrl(ov50h40->link_freq,
				   mode->mipi_freq_idx);
	}
	dev_info(&ov50h40->client->dev, "%s: mode->mipi_freq_idx(%d)",
		 __func__, mode->mipi_freq_idx);

	mutex_unlock(&ov50h40->mutex);

	return 0;
}

static int ov50h40_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	const struct ov50h40_mode *mode = ov50h40->cur_mode;

	mutex_lock(&ov50h40->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ov50h40->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&ov50h40->mutex);

	return 0;
}

static int ov50h40_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = ov50h40->cur_mode->bus_fmt;

	return 0;
}

static int ov50h40_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);

	if (fse->index >= ov50h40->cfg_num)
		return -EINVAL;

	if (fse->code != ov50h40->support_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = ov50h40->support_modes[fse->index].width;
	fse->max_width  = ov50h40->support_modes[fse->index].width;
	fse->max_height = ov50h40->support_modes[fse->index].height;
	fse->min_height = ov50h40->support_modes[fse->index].height;

	return 0;
}

static int ov50h40_enable_test_pattern(struct ov50h40 *ov50h40, u32 pattern)
{
	u32 val;
	int ret = 0;

	if (pattern)
		val = ((pattern - 1) << 4) | OV50H40_TEST_PATTERN_ENABLE;
	else
		val = OV50H40_TEST_PATTERN_DISABLE;
	ret = ov50h40_write_reg(ov50h40->client, OV50H40_REG_TEST_PATTERN,
				OV50H40_REG_VALUE_08BIT, val);
	return ret;
}

static int ov50h40_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	const struct ov50h40_mode *mode = ov50h40->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int ov50h40_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	u32 lane_num = ov50h40->bus_cfg.bus.mipi_csi2.num_data_lanes;
	u32 val = 0;

	val = 1 << (lane_num - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = ov50h40->bus_cfg.bus_type;
	config->flags = val;

	return 0;
}

static void ov50h40_get_otp(struct otp_info *otp,
			       struct rkmodule_inf *inf)
{
	u32 i, j;
	u32 w, h;

	/* awb */
	if (otp->awb_data.flag) {
		inf->awb.flag = 1;
		inf->awb.r_value = otp->awb_data.r_ratio;
		inf->awb.b_value = otp->awb_data.b_ratio;
		inf->awb.gr_value = otp->awb_data.g_ratio;
		inf->awb.gb_value = 0x0;

		inf->awb.golden_r_value = otp->awb_data.r_golden;
		inf->awb.golden_b_value = otp->awb_data.b_golden;
		inf->awb.golden_gr_value = otp->awb_data.g_golden;
		inf->awb.golden_gb_value = 0x0;
	}

	/* lsc */
	if (otp->lsc_data.flag) {
		inf->lsc.flag = 1;
		inf->lsc.width = otp->basic_data.size.width;
		inf->lsc.height = otp->basic_data.size.height;
		inf->lsc.table_size = otp->lsc_data.table_size;

		for (i = 0; i < 289; i++) {
			inf->lsc.lsc_r[i] = (otp->lsc_data.data[i * 2] << 8) |
					     otp->lsc_data.data[i * 2 + 1];
			inf->lsc.lsc_gr[i] = (otp->lsc_data.data[i * 2 + 578] << 8) |
					      otp->lsc_data.data[i * 2 + 579];
			inf->lsc.lsc_gb[i] = (otp->lsc_data.data[i * 2 + 1156] << 8) |
					      otp->lsc_data.data[i * 2 + 1157];
			inf->lsc.lsc_b[i] = (otp->lsc_data.data[i * 2 + 1734] << 8) |
					     otp->lsc_data.data[i * 2 + 1735];
		}
	}

	/* pdaf */
	if (otp->pdaf_data.flag) {
		inf->pdaf.flag = 1;
		inf->pdaf.gainmap_width = otp->pdaf_data.gainmap_width;
		inf->pdaf.gainmap_height = otp->pdaf_data.gainmap_height;
		inf->pdaf.dcc_mode = otp->pdaf_data.dcc_mode;
		inf->pdaf.dcc_dir = otp->pdaf_data.dcc_dir;
		inf->pdaf.dccmap_width = otp->pdaf_data.dccmap_width;
		inf->pdaf.dccmap_height = otp->pdaf_data.dccmap_height;
		w = otp->pdaf_data.gainmap_width;
		h = otp->pdaf_data.gainmap_height;
		for (i = 0; i < h; i++) {
			for (j = 0; j < w; j++) {
				inf->pdaf.gainmap[i * w + j] =
					(otp->pdaf_data.gainmap[(i * w + j) * 2] << 8) |
					otp->pdaf_data.gainmap[(i * w + j) * 2 + 1];
			}
		}
		w = otp->pdaf_data.dccmap_width;
		h = otp->pdaf_data.dccmap_height;
		for (i = 0; i < h; i++) {
			for (j = 0; j < w; j++) {
				inf->pdaf.dccmap[i * w + j] =
					(otp->pdaf_data.dccmap[(i * w + j) * 2] << 8) |
					otp->pdaf_data.dccmap[(i * w + j) * 2 + 1];
			}
		}
	}

	/* af */
	if (otp->af_data.flag) {
		inf->af.flag = 1;
		inf->af.dir_cnt = 1;
		inf->af.af_otp[0].vcm_start = otp->af_data.af_inf;
		inf->af.af_otp[0].vcm_end = otp->af_data.af_macro;
		inf->af.af_otp[0].vcm_dir = 0;
	}

}

static void ov50h40_get_module_inf(struct ov50h40 *ov50h40,
				  struct rkmodule_inf *inf)
{
	struct otp_info *otp = ov50h40->otp;

	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, OV50H40_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, ov50h40->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, ov50h40->len_name, sizeof(inf->base.lens));
	if (otp)
		ov50h40_get_otp(otp, inf);
}

static int ov50h40_get_channel_info(struct ov50h40 *ov50h40, struct rkmodule_channel_info *ch_info)
{
	const struct ov50h40_mode *mode = ov50h40->cur_mode;

	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;

	if (ch_info->index == ov50h40->spd_id && mode->spd) {
		ch_info->vc = V4L2_MBUS_CSI2_CHANNEL_1;
		ch_info->width = mode->spd->width;
		ch_info->height = mode->spd->height;
		ch_info->bus_fmt = mode->spd->bus_fmt;
		ch_info->data_type = mode->spd->data_type;
		ch_info->data_bit = mode->spd->data_bit;
	} else {
		ch_info->vc = ov50h40->cur_mode->vc[ch_info->index];
		ch_info->width = ov50h40->cur_mode->width;
		ch_info->height = ov50h40->cur_mode->height;
		ch_info->bus_fmt = ov50h40->cur_mode->bus_fmt;
	}
	return 0;
}

static long ov50h40_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	struct rkmodule_channel_info *ch_info;
	long ret = 0;
	u32 i, h, w;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		w = ov50h40->cur_mode->width;
		h = ov50h40->cur_mode->height;
		for (i = 0; i < ov50h40->cfg_num; i++) {
			if (w == ov50h40->support_modes[i].width &&
			h == ov50h40->support_modes[i].height &&
			ov50h40->support_modes[i].hdr_mode == hdr_cfg->hdr_mode) {
				ov50h40->cur_mode = &ov50h40->support_modes[i];
				break;
			}
		}
		if (i == ov50h40->cfg_num) {
			dev_err(&ov50h40->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr_cfg->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = ov50h40->cur_mode->hts_def - ov50h40->cur_mode->width;
			h = ov50h40->cur_mode->vts_def - ov50h40->cur_mode->height;
			__v4l2_ctrl_modify_range(ov50h40->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(ov50h40->vblank, h,
				OV50H40_VTS_MAX - ov50h40->cur_mode->height,
				1, h);
			dev_info(&ov50h40->client->dev,
				"sensor mode: %d\n",
				ov50h40->cur_mode->hdr_mode);
		}
		dev_info(&ov50h40->client->dev, "%s: matched mode index(%d)",
			 __func__, i);
		break;
	case RKMODULE_GET_MODULE_INFO:
		ov50h40_get_module_inf(ov50h40, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = ov50h40->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = ov50h40_write_reg(ov50h40->client, OV50H40_REG_CTRL_MODE,
				OV50H40_REG_VALUE_08BIT, OV50H40_MODE_STREAMING);
		else
			ret = ov50h40_write_reg(ov50h40->client, OV50H40_REG_CTRL_MODE,
				OV50H40_REG_VALUE_08BIT, OV50H40_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = ov50h40_get_channel_info(ov50h40, ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov50h40_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov50h40_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = ov50h40_ioctl(sd, cmd, cfg);
		else
			ret = -EFAULT;
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov50h40_ioctl(sd, cmd, hdr);
		if (!ret) {
			ret = copy_to_user(up, hdr, sizeof(*hdr));
			if (ret)
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = ov50h40_ioctl(sd, cmd, hdr);
		else
			ret = -EFAULT;
		kfree(hdr);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = ov50h40_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(ch_info, up, sizeof(*ch_info));
		if (ret) {
			ret = -EFAULT;
			return ret;
		}

		ret = ov50h40_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __ov50h40_start_stream(struct ov50h40 *ov50h40)
{
	int ret;

	if (!ov50h40->is_thunderboot) {
		ret = ov50h40_write_array(ov50h40->client, ov50h40->cur_mode->reg_list);
		if (ret)
			return ret;
	}

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&ov50h40->ctrl_handler);
	if (ret)
		return ret;

	return ov50h40_write_reg(ov50h40->client, OV50H40_REG_CTRL_MODE,
		OV50H40_REG_VALUE_08BIT, OV50H40_MODE_STREAMING);
}

static int __ov50h40_stop_stream(struct ov50h40 *ov50h40)
{
	if (ov50h40->is_thunderboot)
		ov50h40->is_first_streamoff = true;
	return ov50h40_write_reg(ov50h40->client, OV50H40_REG_CTRL_MODE,
		OV50H40_REG_VALUE_08BIT, OV50H40_MODE_SW_STANDBY);
}

static int ov50h40_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	struct i2c_client *client = ov50h40->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				ov50h40->cur_mode->width,
				ov50h40->cur_mode->height,
		DIV_ROUND_CLOSEST(ov50h40->cur_mode->max_fps.denominator,
				  ov50h40->cur_mode->max_fps.numerator));

	mutex_lock(&ov50h40->mutex);
	on = !!on;
	if (on == ov50h40->streaming)
		goto unlock_and_return;

	if (on) {
		if (ov50h40->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			ov50h40->is_thunderboot = false;
			__ov50h40_power_on(ov50h40);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ov50h40_start_stream(ov50h40);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ov50h40_stop_stream(ov50h40);
		pm_runtime_put(&client->dev);
	}

	ov50h40->streaming = on;

unlock_and_return:
	mutex_unlock(&ov50h40->mutex);

	return ret;
}

static int ov50h40_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	struct i2c_client *client = ov50h40->client;
	int ret = 0;

	mutex_lock(&ov50h40->mutex);

	/* If the power state is not modified - no work to do. */
	if (ov50h40->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		if (!ov50h40->is_thunderboot) {
			ret |= ov50h40_write_reg(ov50h40->client,
						 OV50H40_SOFTWARE_RESET_REG,
						 OV50H40_REG_VALUE_08BIT,
						 0x01);
			usleep_range(100, 200);
		}

		ov50h40->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ov50h40->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ov50h40->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ov50h40_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OV50H40_XVCLK_FREQ / 1000 / 1000);
}

static int __ov50h40_power_on(struct ov50h40 *ov50h40)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ov50h40->client->dev;

	if (ov50h40->is_thunderboot)
		return 0;

	if (!IS_ERR_OR_NULL(ov50h40->pins_default)) {
		ret = pinctrl_select_state(ov50h40->pinctrl,
					   ov50h40->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(ov50h40->xvclk, OV50H40_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ov50h40->xvclk) != OV50H40_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(ov50h40->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(ov50h40->reset_gpio))
		gpiod_direction_output(ov50h40->reset_gpio, 1);

	ret = regulator_bulk_enable(OV50H40_NUM_SUPPLIES, ov50h40->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(ov50h40->reset_gpio))
		gpiod_direction_output(ov50h40->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(ov50h40->pwdn_gpio))
		gpiod_direction_output(ov50h40->pwdn_gpio, 0);
	/*
	 * There is no need to wait for the delay of RC circuit
	 * if the reset signal is directly controlled by GPIO.
	 */
	if (!IS_ERR(ov50h40->reset_gpio))
		usleep_range(8000, 10000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ov50h40_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ov50h40->xvclk);

	return ret;
}

static void __ov50h40_power_off(struct ov50h40 *ov50h40)
{
	int ret;
	struct device *dev = &ov50h40->client->dev;

	if (ov50h40->is_thunderboot) {
		if (ov50h40->is_first_streamoff) {
			ov50h40->is_thunderboot = false;
			ov50h40->is_first_streamoff = false;
		} else {
			return;
		}
	}

	if (!IS_ERR(ov50h40->pwdn_gpio))
		gpiod_direction_output(ov50h40->pwdn_gpio, 1);

	clk_disable_unprepare(ov50h40->xvclk);

	if (!IS_ERR(ov50h40->reset_gpio))
		gpiod_direction_output(ov50h40->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(ov50h40->pins_sleep)) {
		ret = pinctrl_select_state(ov50h40->pinctrl,
					   ov50h40->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}

	if (ov50h40->is_thunderboot_ng) {
		ov50h40->is_thunderboot_ng = false;
	}
	regulator_bulk_disable(OV50H40_NUM_SUPPLIES, ov50h40->supplies);
}

static int __maybe_unused ov50h40_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov50h40 *ov50h40 = to_ov50h40(sd);

	return __ov50h40_power_on(ov50h40);
}

static int __maybe_unused ov50h40_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov50h40 *ov50h40 = to_ov50h40(sd);

	__ov50h40_power_off(ov50h40);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov50h40_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ov50h40_mode *def_mode = &ov50h40->support_modes[0];

	mutex_lock(&ov50h40->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov50h40->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ov50h40_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);

	if (fie->index >= ov50h40->cfg_num)
		return -EINVAL;

	fie->code = ov50h40->support_modes[fie->index].bus_fmt;
	fie->width = ov50h40->support_modes[fie->index].width;
	fie->height = ov50h40->support_modes[fie->index].height;
	fie->interval = ov50h40->support_modes[fie->index].max_fps;
	fie->reserved[0] = ov50h40->support_modes[fie->index].hdr_mode;
	return 0;
}
//#define RK356X_TEST
#ifdef RK356X_TEST
#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
#define DST_WIDTH 4096
#define DST_HEIGHT 2304
static int ov50h40_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct ov50h40 *ov50h40 = to_ov50h40(sd);

	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sel->r.left = CROP_START(ov50h40->cur_mode->width, DST_WIDTH);
		sel->r.width = DST_WIDTH;
		sel->r.top = CROP_START(ov50h40->cur_mode->height, DST_HEIGHT);
		sel->r.height = DST_HEIGHT;
		return 0;
	}
	return -EINVAL;
}
#endif

static const struct dev_pm_ops ov50h40_pm_ops = {
	SET_RUNTIME_PM_OPS(ov50h40_runtime_suspend,
			   ov50h40_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ov50h40_internal_ops = {
	.open = ov50h40_open,
};
#endif

static const struct v4l2_subdev_core_ops ov50h40_core_ops = {
	.s_power = ov50h40_s_power,
	.ioctl = ov50h40_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov50h40_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ov50h40_video_ops = {
	.s_stream = ov50h40_s_stream,
	.g_frame_interval = ov50h40_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov50h40_pad_ops = {
	.enum_mbus_code = ov50h40_enum_mbus_code,
	.enum_frame_size = ov50h40_enum_frame_sizes,
	.enum_frame_interval = ov50h40_enum_frame_interval,
	.get_fmt = ov50h40_get_fmt,
	.set_fmt = ov50h40_set_fmt,
#ifdef RK356X_TEST
	.get_selection = ov50h40_get_selection,
#endif
	.get_mbus_config = ov50h40_g_mbus_config,
};

static const struct v4l2_subdev_ops ov50h40_subdev_ops = {
	.core	= &ov50h40_core_ops,
	.video	= &ov50h40_video_ops,
	.pad	= &ov50h40_pad_ops,
};

static int ov50h40_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov50h40 *ov50h40 = container_of(ctrl->handler,
					     struct ov50h40, ctrl_handler);
	struct i2c_client *client = ov50h40->client;
	s64 max;
	int ret = 0;
	u32 again, dgain;
	u32 val = 0;
	u32 vts = 0;
	u32 exp = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		if (ov50h40->cur_mode->height == 6144)
			max = ov50h40->cur_mode->height + ctrl->val - 44;
		else
			max = ov50h40->cur_mode->height + ctrl->val - 22;
		__v4l2_ctrl_modify_range(ov50h40->exposure,
					 ov50h40->exposure->minimum, max,
					 ov50h40->exposure->step,
					 ov50h40->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (ov50h40->cur_mode->height == 6144)
			exp = ctrl->val / 2;
		else
			exp = ctrl->val;
		ret = ov50h40_write_reg(ov50h40->client,
					OV50H40_REG_EXP_LONG_H,
					OV50H40_REG_VALUE_24BIT,
					exp);
		dev_dbg(&client->dev, "set exposure 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (ctrl->val > 1984) {
			dgain = ctrl->val * 1024 / 1984;
			again = 1984;
		} else {
			dgain = 1024;
			again = ctrl->val;
		}
		ret = ov50h40_write_reg(ov50h40->client,
					OV50H40_REG_AGAIN_LONG_H,
					OV50H40_REG_VALUE_16BIT,
					(again << 1) & 0x7ffe);
		ret |= ov50h40_write_reg(ov50h40->client,
					OV50H40_REG_DGAIN_LONG_H,
					OV50H40_REG_VALUE_24BIT,
					(dgain << 6) & 0xfffc0);
		dev_dbg(&client->dev, "set analog gain 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + ov50h40->cur_mode->height;
		if (ov50h40->cur_mode->height == 6144)
			vts /= 2;
		ret = ov50h40_write_reg(ov50h40->client, OV50H40_REG_VTS,
					OV50H40_REG_VALUE_16BIT,
					vts);
		dev_dbg(&client->dev, "set vblank 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov50h40_enable_test_pattern(ov50h40, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = ov50h40_read_reg(ov50h40->client, OV50H40_MIRROR_REG,
				       OV50H40_REG_VALUE_08BIT,
				       &val);
		if (ctrl->val)
			val |= FLIP_BIT_MASK;
		else
			val &= ~FLIP_BIT_MASK;
		ret = ov50h40_write_reg(ov50h40->client, OV50H40_MIRROR_REG,
					OV50H40_REG_VALUE_08BIT,
					val);
		break;
	case V4L2_CID_VFLIP:
		ret = ov50h40_read_reg(ov50h40->client, OV50H40_FLIP_REG,
				       OV50H40_REG_VALUE_08BIT,
				       &val);
		if (ctrl->val)
			val |= FLIP_BIT_MASK;
		else
			val &= ~FLIP_BIT_MASK;
		ret = ov50h40_write_reg(ov50h40->client, OV50H40_FLIP_REG,
					OV50H40_REG_VALUE_08BIT,
					val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov50h40_ctrl_ops = {
	.s_ctrl = ov50h40_set_ctrl,
};

static int ov50h40_initialize_controls(struct ov50h40 *ov50h40)
{
	const struct ov50h40_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_pixel_rate = 0;
	u32 lane_num = ov50h40->bus_cfg.bus.mipi_csi2.num_data_lanes;

	handler = &ov50h40->ctrl_handler;
	mode = ov50h40->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &ov50h40->mutex;

	ov50h40->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			V4L2_CID_LINK_FREQ,
			3, 0, link_freq_items);

	dst_pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * lane_num;
	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	ov50h40->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
			V4L2_CID_PIXEL_RATE,
			0, PIXEL_RATE_WITH_1250M,
			1, dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(ov50h40->link_freq,
			   mode->mipi_freq_idx);

	h_blank = mode->hts_def - mode->width;
	ov50h40->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ov50h40->hblank)
		ov50h40->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ov50h40->vblank = v4l2_ctrl_new_std(handler, &ov50h40_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				OV50H40_VTS_MAX - mode->height,
				1, vblank_def);
	if (mode->height == 6144)
		exposure_max = mode->vts_def - 44;
	else
		exposure_max = mode->vts_def - 22;
	ov50h40->exposure = v4l2_ctrl_new_std(handler, &ov50h40_ctrl_ops,
				V4L2_CID_EXPOSURE, OV50H40_EXPOSURE_MIN,
				exposure_max, OV50H40_EXPOSURE_STEP,
				mode->exp_def);

	ov50h40->anal_gain = v4l2_ctrl_new_std(handler, &ov50h40_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, OV50H40_GAIN_MIN,
				OV50H40_GAIN_MAX, OV50H40_GAIN_STEP,
				OV50H40_GAIN_DEFAULT);

	ov50h40->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ov50h40_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ov50h40_test_pattern_menu) - 1,
				0, 0, ov50h40_test_pattern_menu);

	ov50h40->h_flip = v4l2_ctrl_new_std(handler, &ov50h40_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);

	ov50h40->v_flip = v4l2_ctrl_new_std(handler, &ov50h40_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&ov50h40->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ov50h40->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov50h40_check_sensor_id(struct ov50h40 *ov50h40,
				  struct i2c_client *client)
{
	struct device *dev = &ov50h40->client->dev;
	u32 id = 0;
	int ret;

	if (ov50h40->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = ov50h40_read_reg(client, OV50H40_REG_CHIP_ID,
			       OV50H40_REG_VALUE_24BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV%06x sensor\n", CHIP_ID);

	return 0;
}

static int ov50h40_configure_regulators(struct ov50h40 *ov50h40)
{
	unsigned int i;

	for (i = 0; i < OV50H40_NUM_SUPPLIES; i++)
		ov50h40->supplies[i].supply = ov50h40_supply_names[i];

	return devm_regulator_bulk_get(&ov50h40->client->dev,
				       OV50H40_NUM_SUPPLIES,
				       ov50h40->supplies);
}

static int ov50h40_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ov50h40 *ov50h40;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	struct device_node *endpoint;
	struct device_node *eeprom_ctrl_node;
	struct i2c_client *eeprom_ctrl_client;
	struct v4l2_subdev *eeprom_ctrl;
	struct otp_info *otp_ptr;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ov50h40 = devm_kzalloc(dev, sizeof(*ov50h40), GFP_KERNEL);
	if (!ov50h40)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ov50h40->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ov50h40->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ov50h40->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ov50h40->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ov50h40->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &ov50h40->bus_cfg);
	if (ov50h40->bus_cfg.bus_type == V4L2_MBUS_CSI2_DPHY) {
		ov50h40->support_modes = supported_modes_dphy;
		ov50h40->cfg_num = ARRAY_SIZE(supported_modes_dphy);
	} else {
		ov50h40->support_modes = supported_modes_cphy;
		ov50h40->cfg_num = ARRAY_SIZE(supported_modes_cphy);
	}

	ov50h40->client = client;
	ov50h40->cur_mode = &ov50h40->support_modes[0];

	ov50h40->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov50h40->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ov50h40->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ov50h40->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ov50h40->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
	if (IS_ERR(ov50h40->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = of_property_read_u32(node,
				   "rockchip,spd-id",
				   &ov50h40->spd_id);
	if (ret != 0) {
		ov50h40->spd_id = PAD_MAX;
		dev_err(dev,
			"failed get spd_id, will not to use spd\n");
	}

	ov50h40->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov50h40->pinctrl)) {
		ov50h40->pins_default =
			pinctrl_lookup_state(ov50h40->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov50h40->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ov50h40->pins_sleep =
			pinctrl_lookup_state(ov50h40->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ov50h40->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = ov50h40_configure_regulators(ov50h40);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ov50h40->mutex);

	sd = &ov50h40->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov50h40_subdev_ops);
	ret = ov50h40_initialize_controls(ov50h40);
	if (ret)
		goto err_destroy_mutex;
	ret = __ov50h40_power_on(ov50h40);
	if (ret)
		goto err_free_handler;

	ret = ov50h40_check_sensor_id(ov50h40, client);
	if (ret)
		goto err_power_off;
	eeprom_ctrl_node = of_parse_phandle(node, "eeprom-ctrl", 0);
	if (eeprom_ctrl_node) {
		eeprom_ctrl_client =
			of_find_i2c_device_by_node(eeprom_ctrl_node);
		of_node_put(eeprom_ctrl_node);
		if (IS_ERR_OR_NULL(eeprom_ctrl_client)) {
			dev_err(dev, "can not get node\n");
			goto continue_probe;
		}
		eeprom_ctrl = i2c_get_clientdata(eeprom_ctrl_client);
		if (IS_ERR_OR_NULL(eeprom_ctrl)) {
			dev_err(dev, "can not get eeprom i2c client\n");
		} else {
			otp_ptr = devm_kzalloc(dev, sizeof(*otp_ptr), GFP_KERNEL);
			if (!otp_ptr)
				return -ENOMEM;
			ret = v4l2_subdev_call(eeprom_ctrl,
				core, ioctl, 0, otp_ptr);
			if (!ret) {
				ov50h40->otp = otp_ptr;
			} else {
				ov50h40->otp = NULL;
				devm_kfree(dev, otp_ptr);
			}
		}
	}
continue_probe:

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ov50h40_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ov50h40->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov50h40->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov50h40->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ov50h40->module_index, facing,
		 OV50H40_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ov50h40_power_off(ov50h40);
err_free_handler:
	v4l2_ctrl_handler_free(&ov50h40->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov50h40->mutex);

	return ret;
}

static int ov50h40_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov50h40 *ov50h40 = to_ov50h40(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ov50h40->ctrl_handler);
	mutex_destroy(&ov50h40->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ov50h40_power_off(ov50h40);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov50h40_of_match[] = {
	{ .compatible = "ovti,ov50h40" },
	{},
};
MODULE_DEVICE_TABLE(of, ov50h40_of_match);
#endif

static const struct i2c_device_id ov50h40_match_id[] = {
	{ "ovti,ov50h40", 0 },
	{ },
};

static struct i2c_driver ov50h40_i2c_driver = {
	.driver = {
		.name = OV50H40_NAME,
		.pm = &ov50h40_pm_ops,
		.of_match_table = of_match_ptr(ov50h40_of_match),
	},
	.probe		= &ov50h40_probe,
	.remove		= &ov50h40_remove,
	.id_table	= ov50h40_match_id,
};

#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
module_i2c_driver(ov50h40_i2c_driver);
#else
static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov50h40_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov50h40_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);
#endif

MODULE_DESCRIPTION("OmniVision ov50h40 sensor driver");
MODULE_LICENSE("GPL v2");

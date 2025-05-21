// SPDX-License-Identifier: GPL-2.0
/*
 * GC08a8 driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 fix gain range.
 * V0.0X01.0X04 add enum_frame_interval function.
 * V0.0X01.0X05 support enum sensor fmt
 * V0.0X01.0X06 support mirror and flip
 * V0.0X01.0X07 add quick stream on/off
 */

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
 #include <linux/rk-preisp.h>
 #include <media/media-entity.h>
 #include <media/v4l2-async.h>
 #include <media/v4l2-ctrls.h>
 #include <media/v4l2-subdev.h>
 #include <linux/pinctrl/consumer.h>
 
 #define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x07)
 
 #ifndef V4L2_CID_DIGITAL_GAIN
 #define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
 #endif
 
 #define GC08a8_LANES			2
 #define GC08a8_BITS_PER_SAMPLE		10
 #define GC08a8_LINK_FREQ_LINEAR		336000000   //3264*2448
 #define GC08a8_PIXEL_RATE_LINEAR	(GC08a8_LINK_FREQ_LINEAR * 2 / 10 * 2)
 
 #define GC08a8_XVCLK_FREQ		24000000
 
 #define CHIP_ID				0x08a8
 #define GC08a8_REG_CHIP_ID_H		0x03f0
 #define GC08a8_REG_CHIP_ID_L		0x03f1
 
 #define GC08a8_REG_CTRL_MODE		0x0100
 #define GC08a8_MODE_SW_STANDBY		0x00
 #define GC08a8_MODE_STREAMING		0x01
 
 #define GC08a8_REG_EXPOSURE_H		0x0202
 #define GC08a8_REG_EXPOSURE_L		0x0203
 #define GC08a8_EXPOSURE_MIN		4
 #define GC08a8_EXPOSURE_STEP		1
 #define GC08a8_VTS_MAX			0xffff
 
 
 #define GC08a8_AGAIN_H		0x0204
 #define GC08a8_AGAIN_l		0x0205
 
 
 
 #define GC08a8_GAIN_MIN			1024
 #define GC08a8_GAIN_MAX			1024*16
 #define GC08a8_GAIN_STEP		1
 #define GC08a8_GAIN_DEFAULT		1024
 
 #define GC08a8_REG_TEST_PATTERN		0x008c
 #define GC08a8_TEST_PATTERN_ENABLE	0x01
 #define GC08a8_TEST_PATTERN_DISABLE	0x00
 
 #define GC08a8_REG_VTS_H		0x0340
 #define GC08a8_REG_VTS_L		0x0341
 
 /*
 #define GC08a8_FLIP_MIRROR_REG		0x0101
 #define GC08a8_MIRROR_BIT_MASK		BIT(0)  
 #define GC08a8_FLIP_BIT_MASK		BIT(1)
 */
 
 
 #define REG_NULL			0xFFFF
 
 #define GC08a8_REG_VALUE_08BIT		1
 #define GC08a8_REG_VALUE_16BIT		2
 #define GC08a8_REG_VALUE_24BIT		3
 
 #define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
 #define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
 #define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
 #define GC08a8_NAME			"gc08a8"
 
 
 /*
 #define GC08A8_MIRROR_NORMAL    1
 #define GC08A8_MIRROR_H         0
 #define GC08A8_MIRROR_V         0
 #define GC08A8_MIRROR_HV        0
 */
 
 /*
 #if GC08A8_MIRROR_NORMAL
 #define GC08A8_MIRROR	0x00
 #elif GC08A8_MIRROR_H
 #define GC08A8_MIRROR	0x01
 #elif GC08A8_MIRROR_V
 #define GC08A8_MIRROR	0x02
 #elif GC08A8_MIRROR_HV
 #define GC08A8_MIRROR	0x03
 #else
 #define GC08A8_MIRROR	0x00
 #endif
 */
 
 
 static const char * const gc08a8_supply_names[] = {
     "dovdd",	/* Digital I/O power */
     "dvdd",		/* Digital core power */
     "avdd",		/* Analog power */
 };
 
 #define GC08a8_NUM_SUPPLIES ARRAY_SIZE(gc08a8_supply_names)
 
 struct regval {
     u16 addr;
     u8 val;
 };
 
 struct gc08a8_mode {
     u32 bus_fmt;
     u32 width;
     u32 height;
     struct v4l2_fract max_fps;
     u32 hts_def;
     u32 vts_def;
     u32 exp_def;
     const struct regval *reg_list;
     u32 hdr_mode;
     u32 vc[PAD_MAX];
 };
 
 struct gc08a8 {
     struct i2c_client	*client;
     struct clk		*xvclk;
     struct gpio_desc	*reset_gpio;
     struct gpio_desc	*pwdn_gpio;
     struct gpio_desc	*pwren_gpio;
     struct regulator_bulk_data supplies[GC08a8_NUM_SUPPLIES];
 
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
     struct v4l2_ctrl	*pixel_rate;
     struct v4l2_ctrl	*link_freq;
     struct v4l2_ctrl	*h_flip;
     struct v4l2_ctrl	*v_flip;
     struct v4l2_ctrl	*test_pattern;
     struct mutex		mutex;
     bool			streaming;
     bool			power_on;
     const struct gc08a8_mode *cur_mode;
     u32			cfg_num;
     u32			module_index;
     u32			cur_vts;
     u32			cur_pixel_rate;
     u32			cur_link_freq;
     struct preisp_hdrae_exp_s init_hdrae_exp;
     const char		*module_facing;
     const char		*module_name;
     const char		*len_name;
     bool			has_init_exp;
     struct v4l2_fract	cur_fps;
 };
 
 #define to_gc08a8(sd) container_of(sd, struct gc08a8, subdev)
 
 /*
  * Xclk 24Mhz
  */
 static const struct regval gc08a8_global_regs[] = {
 
     {0x0202,0x04},
 
     {REG_NULL, 0x00},
 };
 /*
  * Xclk 24Mhz
  * max_framerate 30fps
  * mipi_datarate per lane 648Mbps, 4lane
  */
 static const struct regval gc08a8_linear10bit_3264x2448_regs[] = {
     {0x031c,0x60},
     {0x0337,0x04},
     {0x0335,0x51},
     {0x0336,0x70},
     {0x0383,0xbb},
     {0x031a,0x00},
     {0x0321,0x10},
     {0x0327,0x03},
     {0x0325,0x40},
     {0x0326,0x23},
     {0x0314,0x11},
     {0x0315, 0xd6},
     {0x0316, 0x01},
     {0x0334, 0xc0},
     {0x0324, 0x42},
     {0x031c, 0x00},
     {0x031c, 0x9f},
     {0x039a, 0x43},
     {0x0084, 0x30},
     {0x02b3, 0x08},
     {0x0057, 0x0c},
     {0x05c3, 0x50},
     {0x0311, 0x90},
     {0x05a0, 0x02},
     {0x0074, 0x0a},
     {0x0059, 0x11},
     {0x0070, 0x05},
     {0x0101, 0x03},
     {0x0344, 0x00},
     {0x0345, 0x06},
     {0x0346,0x00},
     {0x0347,0x04},
     {0x0348,0x0c},
     {0x0349,0xd0},
     {0x034a,0x09},
     {0x034b,0x9c},
     {0x0202, 0x09},
     {0x0203, 0x04},
     {0x0340, 0x09},
     {0x0341, 0xf4},
     {0x0342, 0x07},
     {0x0343, 0x1c},
     {0x0219, 0x05},
     {0x0226, 0x00},
     {0x0227, 0x28},
     {0x0e0a, 0x00},
     {0x0e0b, 0x00},
     {0x0e24, 0x04},
     {0x0e25, 0x04},
     {0x0e26, 0x00},
     {0x0e27, 0x10},
     {0x0e01, 0x74},
     {0x0e03, 0x47},
     {0x0e04, 0x33},
     {0x0e05, 0x44},
     {0x0e06, 0x44},
     {0x0e0c, 0x1e},
     {0x0e17, 0x3a},
     {0x0e18, 0x3c},
     {0x0e19, 0x40},
     {0x0e1a, 0x42},
     {0x0e28, 0x21},
     {0x0e2b, 0x68},
     {0x0e2c, 0x0d},
     {0x0e2d, 0x08},
     {0x0e34, 0xf4},
     {0x0e35, 0x44},
     {0x0e36, 0x07},
     {0x0e38, 0x39},
     {0x0210, 0x13},
     {0x0218, 0x00},
     {0x0241, 0x88},
     {0x0e32, 0x00},
     {0x0e33, 0x18},
     {0x0e42, 0x03},
     {0x0e43, 0x80},
     {0x0e44, 0x04},
     {0x0e45, 0x00},
     {0x0e4f, 0x04},
     {0x057a, 0x20},
     {0x0381, 0x7c},
     {0x0382, 0x9b},
     {0x0384, 0xfb},
     {0x0389, 0x38},
     {0x038a, 0x03},
     {0x0390, 0x6a},
     {0x0391, 0x0f},
     {0x0392, 0x60},
     {0x0393, 0xc1},
     {0x0396, 0x3f},
     {0x0398, 0x22},
     {0x031c, 0x80},
     {0x03fe, 0x10},
     {0x03fe, 0x00},
     {0x031c, 0x9f},
     {0x03fe, 0x00},
     {0x03fe, 0x00},
     {0x03fe, 0x00},
     {0x03fe, 0x00},
     {0x031c, 0x80},
     {0x03fe, 0x10},
     {0x03fe, 0x00},
     {0x031c, 0x9f},
     {0x0360, 0x01},
     {0x0360, 0x00},
     {0x0316, 0x09},
     {0x0a67, 0x80},
     {0x0313, 0x00},
     {0x0a53, 0x0e},
     {0x0a65, 0x17},
     {0x0a68, 0xa1},
     {0x0a58, 0x00},
     {0x0ace, 0x0c},
     {0x00a4, 0x00},
     {0x00a5, 0x01},
     {0x00a7, 0x09},
     {0x00a8, 0x9c},
     {0x00a9, 0x0c},
     {0x00aa, 0xd0},
     {0x0a8a, 0x00},
     {0x0a8b, 0xe0},
     {0x0a8c, 0x13},
     {0x0a8d, 0xe8},
     {0x0a90, 0x0a},
     {0x0a91, 0x10},
     {0x0a92, 0xf8},
     {0x0a71, 0xf2},
     {0x0a72, 0x12},
     {0x0a73, 0x64},
     {0x0a75, 0x41},
     {0x0a70, 0x07},
     {0x0313, 0x80},
     {0x00a0, 0x01},
     {0x0080, 0xd2},
     {0x0081, 0x3f},
     {0x0087, 0x51},
     {0x0089, 0x03},
     {0x009b, 0x40},
     {0x0096, 0x81},
     {0x0097, 0x08},
     {0x05a0, 0x82},
     {0x05ac, 0x00},
     {0x05ad, 0x01},
     {0x05ae, 0x00},
     {0x0800, 0x0a},
     {0x0801, 0x14},
     {0x0802, 0x28},
     {0x0803, 0x34},
     {0x0804, 0x0e},
     {0x0805, 0x33},
     {0x0806, 0x03},
     {0x0807, 0x8a},
     {0x0808, 0x3e},
     {0x0809, 0x00},
     {0x080a, 0x28},
     {0x080b, 0x03},
     {0x080c, 0x1d},
     {0x080d, 0x03},
     {0x080e, 0x16},
     {0x080f, 0x03},
     {0x0810, 0x10},
     {0x0811, 0x03},
     {0x0812, 0x00},
     {0x0813, 0x00},
     {0x0814, 0x01},
     {0x0815, 0x00},
     {0x0816, 0x01},
     {0x0817, 0x00},
     {0x0818, 0x00},
     {0x0819, 0x0a},
     {0x081a, 0x01},
     {0x081b, 0x6c},
     {0x081c, 0x00},
     {0x081d, 0x0b},
     {0x081e, 0x02},
     {0x081f, 0x00},
     {0x0820, 0x00},
     {0x0821, 0x0c},
     {0x0822, 0x02},
     {0x0823, 0xd9},
     {0x0824, 0x00},
     {0x0825, 0x0d},
     {0x0826, 0x03},
     {0x0827, 0xf0},
     {0x0828, 0x00},
     {0x0829, 0x0e},
     {0x082a, 0x05},
     {0x082b, 0x94},
     {0x082c, 0x09},
     {0x082d, 0x6e},
     {0x082e, 0x07},
     {0x082f, 0xe6},
     {0x0830, 0x10},
     {0x0831, 0x0e},
     {0x0832, 0x0b},
     {0x0833, 0x2c},
     {0x0834, 0x14},
     {0x0835, 0xae},
     {0x0836, 0x0f},
     {0x0837, 0xc4},
     {0x0838, 0x18},
     {0x0839, 0x0e},
     {0x05ac, 0x01},
     {0x059a, 0x00},
     {0x059b, 0x00},
     {0x059c, 0x01},
     {0x0598, 0x00},
     {0x0597, 0x14},
     {0x05ab, 0x09},
     {0x05a4, 0x02},
     {0x05a3, 0x05},
     {0x05a0, 0xc2},
     {0x0207, 0xc4},
     {0x0208, 0x01},
     {0x0209, 0x78},
     {0x0204, 0x04},
     {0x0205, 0x00},
     {0x0040, 0x22},
     {0x0041, 0x20},
     {0x0043, 0x10},
     {0x0044, 0x00},
     {0x0046, 0x08},
     {0x0047, 0xf0},
     {0x0048, 0x0f},
     {0x004b, 0x0f},
     {0x004c, 0x00},
     {0x0050, 0x5c},
     {0x0051, 0x44},
     {0x005b, 0x03},
     {0x00c0, 0x00},
     {0x00c1, 0x80},
     {0x00c2, 0x31},
     {0x00c3, 0x00},
     {0x0460, 0x04},
     {0x0462, 0x08},
     {0x0464, 0x0e},
     {0x0466, 0x0a},
     {0x0468, 0x12},
     {0x046a, 0x12},
     {0x046c, 0x10},
     {0x046e, 0x0c},
     {0x0461, 0x03},
     {0x0463, 0x03},
     {0x0465, 0x03},
     {0x0467, 0x03},
     {0x0469, 0x04},
     {0x046b, 0x04},
     {0x046d, 0x04},
     {0x046f, 0x04},
     {0x0470, 0x04},
     {0x0472, 0x10},
     {0x0474, 0x26},
     {0x0476, 0x38},
     {0x0478, 0x20},
     {0x047a, 0x30},
     {0x047c, 0x38},
     {0x047e, 0x60},
     {0x0471, 0x05},
     {0x0473, 0x05},
     {0x0475, 0x05},
     {0x0477, 0x05},
     {0x0479, 0x04},
     {0x047b, 0x04},
     {0x047d, 0x04},
     {0x047f, 0x04},
     {0x009a,0x00},
     {0x0351,0x00},
     {0x0352,0x06},
     {0x0353,0x00},
     {0x0354,0x08},
     {0x034c,0x0c},
     {0x034d,0xc0},
     {0x034e,0x09},
     {0x034f,0x90},
     {0x0114,0x03},
     {0x0180,0x6f},
     {0x0181,0x30},
     {0x0185,0x01},
     {0x0115,0x30},
     {0x011b,0x12},
     {0x011c,0x12},
     {0x0121,0x06},
     {0x0122,0x06},
     {0x0123,0x15},
     {0x0124,0x01},
     {0x0125,0x13},
     {0x0126,0x08},
     {0x0129,0x06},
     {0x012a,0x08},
     {0x012b,0x08},
     {0x0a73,0x60},
     {0x0a70,0x11},
     {0x0313,0x80},
     {0x0aff,0x00},
     {0x0aff,0x00},
     {0x0aff,0x00},
     {0x0aff,0x00},
     {0x0aff,0x00},
     {0x0aff,0x00},
     {0x0aff,0x00},
     {0x0aff,0x00},
     {0x0a70,0x00},
     {0x00a4,0x80},
     {0x0316,0x01},
     {0x0a67,0x00},
     {0x0084,0x10},
     {0x0102,0x09},
     {0x031c, 0x60},
     {0x0337, 0x04},
     {0x0335, 0x51},
     {0x0336, 0x70},
     {0x0383, 0xbb},
     {0x031a, 0x00},
     {0x0321, 0x10},
     {0x0327, 0x03},
     {0x0325, 0x40},
     {0x0326, 0x23},
     {0x0314, 0x11},
     {0x0315, 0xd6},
     {0x0316, 0x01},
     {0x0334, 0xc0},
     {0x0324, 0x42},
     {0x031c, 0x00},
     {0x031c, 0x9f},
     {0x0344, 0x00},
     {0x0345, 0x06},
     {0x0346, 0x00},
     {0x0347, 0x04},
     {0x0348, 0x0c},
     {0x0349, 0xd0},
     {0x034a, 0x09},
     {0x034b, 0x9c},
     {0x0202, 0x09},
     {0x0203, 0x04},
     {0x0340, 0x09},
     {0x0341, 0xf4},
     {0x0342, 0x07},
     {0x0343, 0x1c},
     {0x0226, 0x00},
     {0x0227, 0x28},
     {0x0e38, 0x39},
     {0x0210, 0x13},
     {0x0218, 0x00},
     {0x0241, 0x88},
     {0x0392, 0x60},
     {0x031c, 0x80},
     {0x03fe, 0x10},
     {0x03fe, 0x00},
     {0x031c, 0x9f},
     {0x03fe, 0x00},
     {0x03fe, 0x00},
     {0x03fe, 0x00},
     {0x03fe, 0x00},
     {0x031c, 0x80},
     {0x03fe, 0x10},
     {0x03fe, 0x00},
     {0x031c, 0x9f},
     {0x00a2, 0x00},
     {0x00a3, 0x00},
     {0x00ab, 0x00},
     {0x00ac, 0x00},
     {0x05a0, 0x82},
     {0x05ac, 0x00},
     {0x05ad, 0x01},
     {0x05ae, 0x00},
     {0x0800, 0x0a},
     {0x0801, 0x14},
     {0x0802, 0x28},
     {0x0803, 0x34},
     {0x0804, 0x0e},
     {0x0805, 0x33},
     {0x0806, 0x03},
     {0x0807, 0x8a},
     {0x0808, 0x3e},
     {0x0809, 0x00},
     {0x080a, 0x28},
     {0x080b, 0x03},
     {0x080c, 0x1d},
     {0x080d, 0x03},
     {0x080e, 0x16},
     {0x080f, 0x03},
     {0x0810, 0x10},
     {0x0811, 0x03},
     {0x0812, 0x00},
     {0x0813, 0x00},
     {0x0814, 0x01},
     {0x0815, 0x00},
     {0x0816, 0x01},
     {0x0817, 0x00},
     {0x0818, 0x00},
     {0x0819, 0x0a},
     {0x081a, 0x01},
     {0x081b, 0x6c},
     {0x081c, 0x00},
     {0x081d, 0x0b},
     {0x081e, 0x02},
     {0x081f, 0x00},
     {0x0820, 0x00},
     {0x0821, 0x0c},
     {0x0822, 0x02},
     {0x0823, 0xd9},
     {0x0824, 0x00},
     {0x0825, 0x0d},
     {0x0826, 0x03},
     {0x0827, 0xf0},
     {0x0828, 0x00},
     {0x0829, 0x0e},
     {0x082a, 0x05},
     {0x082b, 0x94},
     {0x082c, 0x09},
     {0x082d, 0x6e},
     {0x082e, 0x07},
     {0x082f, 0xe6},
     {0x0830, 0x10},
     {0x0831, 0x0e},
     {0x0832, 0x0b},
     {0x0833, 0x2c},
     {0x0834, 0x14},
     {0x0835, 0xae},
     {0x0836, 0x0f},
     {0x0837, 0xc4},
     {0x0838, 0x18},
     {0x0839, 0x0e},
     {0x05ac, 0x01},
     {0x059a, 0x00},
     {0x059b, 0x00},
     {0x059c, 0x01},
     {0x0598, 0x00},
     {0x0597, 0x14},
     {0x05ab, 0x09},
     {0x05a4, 0x02},
     {0x05a3, 0x05},
     {0x05a0, 0xc2},
     {0x0207, 0xc4},
     {0x0204, 0x04},
     {0x0205, 0x00},
     {0x0050, 0x5c},
     {0x0051, 0x44},
     {0x009a, 0x00},
     {0x0351, 0x00},
     {0x0352, 0x06},
     {0x0353, 0x00},
     {0x0354, 0x08},
     {0x034c, 0x0c},
     {0x034d, 0xc0},
     {0x034e, 0x09},
     {0x034f, 0x90},
     {0x0114, 0x01},
     {0x0180, 0x6f},
     {0x0181, 0x30},
     {0x0185, 0x01},
     {0x0115, 0x10},
     {0x011b, 0x12},
     {0x011c, 0x12},
     {0x0121, 0x0b},
     {0x0122, 0x0d},
     {0x0123, 0x2f},
     {0x0124, 0x01},
     {0x0125, 0x12},
     {0x0126, 0x0f},
     {0x0129, 0x0c},
     {0x012a, 0x13},
     {0x012b, 0x0f},
     {0x0a73, 0x60},
     {0x0a70, 0x11},
     {0x0313, 0x80},
     {0x0aff, 0x00},
     {0x0aff, 0x00},
     {0x0aff, 0x00},
     {0x0aff, 0x00},
     {0x0aff, 0x00},
     {0x0aff, 0x00},
     {0x0aff, 0x00},
     {0x0aff, 0x00},
     {0x0a70, 0x00},
     {0x00a4, 0x80},
     {0x0316, 0x01},
     {0x0a67, 0x00},
     {0x0084, 0x10},
     {0x0102, 0x09},
 
 //{0x0100,0x01},
 
 {REG_NULL, 0x00},
 };
 
 static const struct gc08a8_mode supported_modes[] = {
     {
         .width = 3264,
         .height = 2448,
         .max_fps = {
             .numerator = 10000,
             .denominator = 300000,
         },
         .exp_def = 0x0100,
         .hts_def = 0x0E38,
         .vts_def = 0x09F4,
         .bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
         .reg_list = gc08a8_linear10bit_3264x2448_regs,
         .hdr_mode = NO_HDR,
         .vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
     },
 };
 
 static const u32 bus_code[] = {
     MEDIA_BUS_FMT_SBGGR10_1X10,
 };
 
 static const s64 link_freq_menu_items[] = {
     GC08a8_LINK_FREQ_LINEAR,
 };
 
 static const char * const gc08a8_test_pattern_menu[] = {
     "Disabled",
     "Vertical Color Bar Type 1",
     "Vertical Color Bar Type 2",
     "Vertical Color Bar Type 3",
     "Vertical Color Bar Type 4"
 };
 
 /* Write registers up to 4 at a time */
 static int gc08a8_write_reg(struct i2c_client *client, u16 reg,
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
 
     if (i2c_master_send(client, buf, len + 2) != len + 2)
         return -EIO;
 
     return 0;
 }
 
 static int gc08a8_write_array(struct i2c_client *client,
                   const struct regval *regs)
 {
     u32 i;
     int ret = 0;
 
     for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
         ret = gc08a8_write_reg(client, regs[i].addr,
                        GC08a8_REG_VALUE_08BIT, regs[i].val);
 
     return ret;
 }
 
 /* Read registers up to 4 at a time */
 static int gc08a8_read_reg(struct i2c_client *client, u16 reg,
                unsigned int len, u32 *val)
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
 
 static int gc08a8_get_reso_dist(const struct gc08a8_mode *mode,
                 struct v4l2_mbus_framefmt *framefmt)
 {
     return abs(mode->width - framefmt->width) +
             abs(mode->height - framefmt->height);
 }
 
 static const struct gc08a8_mode *
 gc08a8_find_best_fit(struct gc08a8 *gc08a8, struct v4l2_subdev_format *fmt)
 {
     struct v4l2_mbus_framefmt *framefmt = &fmt->format;
     int dist;
     int cur_best_fit = 0;
     int cur_best_fit_dist = -1;
     unsigned int i;
 
     for (i = 0; i < gc08a8->cfg_num; i++) {
         dist = gc08a8_get_reso_dist(&supported_modes[i], framefmt);
         if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
             cur_best_fit_dist = dist;
             cur_best_fit = i;
         }
     }
 
     return &supported_modes[cur_best_fit];
 }
 
 static int gc08a8_set_fmt(struct v4l2_subdev *sd,
               struct v4l2_subdev_pad_config *cfg,
               struct v4l2_subdev_format *fmt)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
     const struct gc08a8_mode *mode;
     s64 h_blank, vblank_def;
 
     mutex_lock(&gc08a8->mutex);
 
     mode = gc08a8_find_best_fit(gc08a8, fmt);
     fmt->format.code = mode->bus_fmt;
     fmt->format.width = mode->width;
     fmt->format.height = mode->height;
     fmt->format.field = V4L2_FIELD_NONE;
     if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
 #ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
         *v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
 #else
         mutex_unlock(&gc08a8->mutex);
         return -ENOTTY;
 #endif
     } else {
         gc08a8->cur_mode = mode;
         h_blank = mode->hts_def - mode->width;
         __v4l2_ctrl_modify_range(gc08a8->hblank, h_blank,
                      h_blank, 1, h_blank);
         vblank_def = mode->vts_def - mode->height;
         __v4l2_ctrl_modify_range(gc08a8->vblank, vblank_def,
                      GC08a8_VTS_MAX - mode->height,
                      1, vblank_def);
 
         gc08a8->cur_link_freq = 0;
         gc08a8->cur_pixel_rate = GC08a8_PIXEL_RATE_LINEAR;
 
         __v4l2_ctrl_s_ctrl_int64(gc08a8->pixel_rate,
                      gc08a8->cur_pixel_rate);
         __v4l2_ctrl_s_ctrl(gc08a8->link_freq,
                    gc08a8->cur_link_freq);
         gc08a8->cur_vts = mode->vts_def;
         gc08a8->cur_fps = mode->max_fps;
     }
     mutex_unlock(&gc08a8->mutex);
 
     return 0;
 }
 
 static int gc08a8_get_fmt(struct v4l2_subdev *sd,
               struct v4l2_subdev_pad_config *cfg,
               struct v4l2_subdev_format *fmt)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
     const struct gc08a8_mode *mode = gc08a8->cur_mode;
 
     mutex_lock(&gc08a8->mutex);
     if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
 #ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
         fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
 #else
         mutex_unlock(&gc08a8->mutex);
         return -ENOTTY;
 #endif
     } else {
         fmt->format.width = mode->width;
         fmt->format.height = mode->height;
         fmt->format.code = mode->bus_fmt;
         fmt->format.field = V4L2_FIELD_NONE;
     }
     mutex_unlock(&gc08a8->mutex);
 
     return 0;
 }
 
 static int gc08a8_enum_mbus_code(struct v4l2_subdev *sd,
                  struct v4l2_subdev_pad_config *cfg,
                  struct v4l2_subdev_mbus_code_enum *code)
 {
     if (code->index >= ARRAY_SIZE(bus_code))
         return -EINVAL;
     code->code = bus_code[code->index];
 
     return 0;
 }
 
 static int gc08a8_enum_frame_sizes(struct v4l2_subdev *sd,
                    struct v4l2_subdev_pad_config *cfg,
                    struct v4l2_subdev_frame_size_enum *fse)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
 
     if (fse->index >= gc08a8->cfg_num)
         return -EINVAL;
 
     if (fse->code != supported_modes[0].bus_fmt)
         return -EINVAL;
 
     fse->min_width = supported_modes[fse->index].width;
     fse->max_width = supported_modes[fse->index].width;
     fse->max_height = supported_modes[fse->index].height;
     fse->min_height = supported_modes[fse->index].height;
 
     return 0;
 }
 
 static int gc08a8_enable_test_pattern(struct gc08a8 *gc08a8, u32 pattern)
 {
     //u32 val;
     return 0;
     
     #if  0
     if (pattern)
         val = GC08a8_TEST_PATTERN_ENABLE;
     else
         val = GC08a8_TEST_PATTERN_DISABLE;
 
     return gc08a8_write_reg(gc08a8->client, GC08a8_REG_TEST_PATTERN,
                 GC08a8_REG_VALUE_08BIT, val);
     #endif			
 }
 
 
 
 static int gc08a8_g_frame_interval(struct v4l2_subdev *sd,
                    struct v4l2_subdev_frame_interval *fi)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
     const struct gc08a8_mode *mode = gc08a8->cur_mode;
 
     if (gc08a8->streaming)
         fi->interval = gc08a8->cur_fps;
     else
         fi->interval = mode->max_fps;
 
     return 0;
 }
 
 static const struct gc08a8_mode *gc08a8_find_mode(struct gc08a8 *gc08a8, int fps)
 {
     const struct gc08a8_mode *mode = NULL;
     const struct gc08a8_mode *match = NULL;
     int cur_fps = 0;
     int i = 0;
 
     for (i = 0; i < gc08a8->cfg_num; i++) {
         mode = &supported_modes[i];
         if (mode->width == gc08a8->cur_mode->width &&
             mode->height == gc08a8->cur_mode->height &&
             mode->hdr_mode == gc08a8->cur_mode->hdr_mode &&
             mode->bus_fmt == gc08a8->cur_mode->bus_fmt) {
             cur_fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator, mode->max_fps.numerator);
             if (cur_fps == fps) {
                 match = mode;
                 break;
             }
         }
     }
     return match;
 }
 
 static int gc08a8_s_frame_interval(struct v4l2_subdev *sd,
                    struct v4l2_subdev_frame_interval *fi)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
     const struct gc08a8_mode *mode = NULL;
     struct v4l2_fract *fract = &fi->interval;
     s64 h_blank, vblank_def;
     int fps;
 
     if (gc08a8->streaming)
         return -EBUSY;
 
     if (fi->pad != 0)
         return -EINVAL;
 
     if (fract->numerator == 0) {
         v4l2_err(sd, "error param, check interval param\n");
         return -EINVAL;
     }
     fps = DIV_ROUND_CLOSEST(fract->denominator, fract->numerator);
     mode = gc08a8_find_mode(gc08a8, fps);
     if (mode == NULL) {
         v4l2_err(sd, "couldn't match fi\n");
         return -EINVAL;
     }
 
     gc08a8->cur_mode = mode;
 
     h_blank = mode->hts_def - mode->width;
     __v4l2_ctrl_modify_range(gc08a8->hblank, h_blank,
                  h_blank, 1, h_blank);
     vblank_def = mode->vts_def - mode->height;
     __v4l2_ctrl_modify_range(gc08a8->vblank, vblank_def,
                  GC08a8_VTS_MAX - mode->height,
                  1, vblank_def);
     gc08a8->cur_fps = mode->max_fps;
 
     return 0;
 }
 
 static int gc08a8_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
                 struct v4l2_mbus_config *config)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
     const struct gc08a8_mode *mode = gc08a8->cur_mode;
     u32 val = 0;
 
     if (mode->hdr_mode == NO_HDR)
         val = 1 << (GC08a8_LANES - 1) |
         V4L2_MBUS_CSI2_CHANNEL_0 |
         V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
 
     config->type = V4L2_MBUS_CSI2_DPHY;
     config->flags = val;
 
     return 0;
 }
 
 static void gc08a8_get_module_inf(struct gc08a8 *gc08a8,
                   struct rkmodule_inf *inf)
 {
     memset(inf, 0, sizeof(*inf));
     strscpy(inf->base.sensor, GC08a8_NAME, sizeof(inf->base.sensor));
     strscpy(inf->base.module, gc08a8->module_name,
         sizeof(inf->base.module));
     strscpy(inf->base.lens, gc08a8->len_name, sizeof(inf->base.lens));
 }
 
 static int gc08a8_get_channel_info(struct gc08a8 *gc08a8, struct rkmodule_channel_info *ch_info)
 {
     if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
         return -EINVAL;
     ch_info->vc = gc08a8->cur_mode->vc[ch_info->index];
     ch_info->width = gc08a8->cur_mode->width;
     ch_info->height = gc08a8->cur_mode->height;
     ch_info->bus_fmt = gc08a8->cur_mode->bus_fmt;
     return 0;
 }
 
 static long gc08a8_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
     struct rkmodule_hdr_cfg *hdr;
     u32 i, h, w;
     long ret = 0;
     u32 stream = 0;
     struct rkmodule_channel_info *ch_info;
 
     switch (cmd) {
     case RKMODULE_GET_MODULE_INFO:
         gc08a8_get_module_inf(gc08a8, (struct rkmodule_inf *)arg);
         break;
     case RKMODULE_GET_HDR_CFG:
         hdr = (struct rkmodule_hdr_cfg *)arg;
         hdr->esp.mode = HDR_NORMAL_VC;
         hdr->hdr_mode = gc08a8->cur_mode->hdr_mode;
         break;
     case RKMODULE_SET_HDR_CFG:
         hdr = (struct rkmodule_hdr_cfg *)arg;
         w = gc08a8->cur_mode->width;
         h = gc08a8->cur_mode->height;
         for (i = 0; i < gc08a8->cfg_num; i++) {
             if (w == supported_modes[i].width &&
                 h == supported_modes[i].height &&
                 supported_modes[i].hdr_mode == hdr->hdr_mode &&
                 supported_modes[i].bus_fmt == gc08a8->cur_mode->bus_fmt) {
                 gc08a8->cur_mode = &supported_modes[i];
                 break;
             }
         }
         if (i == gc08a8->cfg_num) {
             dev_err(&gc08a8->client->dev,
                 "not find hdr mode:%d %dx%d config\n",
                 hdr->hdr_mode, w, h);
             ret = -EINVAL;
         } else {
             w = gc08a8->cur_mode->hts_def -
                 gc08a8->cur_mode->width;
             h = gc08a8->cur_mode->vts_def -
                 gc08a8->cur_mode->height;
             __v4l2_ctrl_modify_range(gc08a8->hblank, w, w, 1, w);
             __v4l2_ctrl_modify_range(gc08a8->vblank, h,
                          GC08a8_VTS_MAX -
                          gc08a8->cur_mode->height,
                          1, h);
             gc08a8->cur_link_freq = 0;
             gc08a8->cur_pixel_rate = GC08a8_PIXEL_RATE_LINEAR;
 
         __v4l2_ctrl_s_ctrl_int64(gc08a8->pixel_rate,
                      gc08a8->cur_pixel_rate);
         __v4l2_ctrl_s_ctrl(gc08a8->link_freq,
                    gc08a8->cur_link_freq);
         gc08a8->cur_vts = gc08a8->cur_mode->vts_def;
         }
         break;
     case PREISP_CMD_SET_HDRAE_EXP:
         break;
     case RKMODULE_SET_QUICK_STREAM:
         stream = *((u32 *)arg);
         if (stream)
             ret = gc08a8_write_reg(gc08a8->client, GC08a8_REG_CTRL_MODE,
                 GC08a8_REG_VALUE_08BIT, GC08a8_MODE_STREAMING);
         else
             ret = gc08a8_write_reg(gc08a8->client, GC08a8_REG_CTRL_MODE,
                 GC08a8_REG_VALUE_08BIT, GC08a8_MODE_SW_STANDBY);
         break;
     case RKMODULE_GET_CHANNEL_INFO:
         ch_info = (struct rkmodule_channel_info *)arg;
         ret = gc08a8_get_channel_info(gc08a8, ch_info);
         break;
     default:
         ret = -ENOIOCTLCMD;
         break;
     }
 
     return ret;
 }
 
 #ifdef CONFIG_COMPAT
 static long gc08a8_compat_ioctl32(struct v4l2_subdev *sd,
                   unsigned int cmd, unsigned long arg)
 {
     void __user *up = compat_ptr(arg);
     struct rkmodule_inf *inf;
     struct rkmodule_awb_cfg *cfg;
     struct rkmodule_hdr_cfg *hdr;
     struct preisp_hdrae_exp_s *hdrae;
     long ret;
     u32 stream = 0;
     struct rkmodule_channel_info *ch_info;
 
     switch (cmd) {
     case RKMODULE_GET_MODULE_INFO:
         inf = kzalloc(sizeof(*inf), GFP_KERNEL);
         if (!inf) {
             ret = -ENOMEM;
             return ret;
         }
 
         ret = gc08a8_ioctl(sd, cmd, inf);
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
             ret = gc08a8_ioctl(sd, cmd, cfg);
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
 
         ret = gc08a8_ioctl(sd, cmd, hdr);
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
             ret = gc08a8_ioctl(sd, cmd, hdr);
         else
             ret = -EFAULT;
         kfree(hdr);
         break;
     case PREISP_CMD_SET_HDRAE_EXP:
         hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
         if (!hdrae) {
             ret = -ENOMEM;
             return ret;
         }
 
         ret = copy_from_user(hdrae, up, sizeof(*hdrae));
         if (!ret)
             ret = gc08a8_ioctl(sd, cmd, hdrae);
         else
             ret = -EFAULT;
         kfree(hdrae);
         break;
     case RKMODULE_SET_QUICK_STREAM:
         ret = copy_from_user(&stream, up, sizeof(u32));
         if (!ret)
             ret = gc08a8_ioctl(sd, cmd, &stream);
         else
             ret = -EFAULT;
         break;
     case RKMODULE_GET_CHANNEL_INFO:
         ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
         if (!ch_info) {
             ret = -ENOMEM;
             return ret;
         }
 
         ret = gc08a8_ioctl(sd, cmd, ch_info);
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
 
 static int __gc08a8_start_stream(struct gc08a8 *gc08a8)
 {
     int ret;
 
     ret = gc08a8_write_array(gc08a8->client, gc08a8->cur_mode->reg_list);
     if (ret)
         return ret;
 
     
     ret = __v4l2_ctrl_handler_setup(&gc08a8->ctrl_handler);
     if (gc08a8->has_init_exp && gc08a8->cur_mode->hdr_mode != NO_HDR) {
         ret = gc08a8_ioctl(&gc08a8->subdev, PREISP_CMD_SET_HDRAE_EXP,
             &gc08a8->init_hdrae_exp);
         if (ret) {
             dev_err(&gc08a8->client->dev,
                 "init exp fail in hdr mode\n");
             return ret;
         }
     }
     if (ret)
         return ret;
 
     ret |= gc08a8_write_reg(gc08a8->client, GC08a8_REG_CTRL_MODE,
                 GC08a8_REG_VALUE_08BIT, GC08a8_MODE_STREAMING);
     //if (gc08a8->cur_mode->hdr_mode == NO_HDR)
         //ret |= gc08a8_write_array(gc08a8->client, gc08a8_otp_regs);
     return ret;
 }
 
 static int __gc08a8_stop_stream(struct gc08a8 *gc08a8)
 {
     gc08a8->has_init_exp = false;
 
     return gc08a8_write_reg(gc08a8->client, GC08a8_REG_CTRL_MODE,
             GC08a8_REG_VALUE_08BIT, GC08a8_MODE_SW_STANDBY);
 }
 
 static int gc08a8_s_stream(struct v4l2_subdev *sd, int on)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
     struct i2c_client *client = gc08a8->client;
     int ret = 0;
 
     mutex_lock(&gc08a8->mutex);
     on = !!on;
     if (on == gc08a8->streaming)
         goto unlock_and_return;
 
     if (on) {
         ret = pm_runtime_get_sync(&client->dev);
         if (ret < 0) {
             pm_runtime_put_noidle(&client->dev);
             goto unlock_and_return;
         }
 
          ret = __gc08a8_start_stream(gc08a8);
         if (ret) {
             v4l2_err(sd, "start stream failed while write regs\n");
             pm_runtime_put(&client->dev);
             goto unlock_and_return;
         }
     } else {
         __gc08a8_stop_stream(gc08a8);
         pm_runtime_put(&client->dev);
     }
 
     gc08a8->streaming = on;
 
 unlock_and_return:
     mutex_unlock(&gc08a8->mutex);
 
     return ret;
 }
 
 static int gc08a8_s_power(struct v4l2_subdev *sd, int on)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
     struct i2c_client *client = gc08a8->client;
     int ret = 0;
 
     mutex_lock(&gc08a8->mutex);
 
     /* If the power state is not modified - no work to do. */
     if (gc08a8->power_on == !!on)
         goto unlock_and_return;
 
     if (on) {
         ret = pm_runtime_get_sync(&client->dev);
         if (ret < 0) {
             pm_runtime_put_noidle(&client->dev);
             goto unlock_and_return;
         }
 
         ret = gc08a8_write_array(gc08a8->client, gc08a8_global_regs);
         if (ret) {
             v4l2_err(sd, "could not set init registers\n");
             pm_runtime_put_noidle(&client->dev);
             goto unlock_and_return;
         }
 
         gc08a8->power_on = true;
     } else {
         pm_runtime_put(&client->dev);
         gc08a8->power_on = false;
     }
 
 unlock_and_return:
     mutex_unlock(&gc08a8->mutex);
 
     return ret;
 }
 
 /* Calculate the delay in us by clock rate and clock cycles */
 static inline u32 gc08a8_cal_delay(u32 cycles)
 {
     return DIV_ROUND_UP(cycles, GC08a8_XVCLK_FREQ / 1000 / 1000);
 }
 
 static int __gc08a8_power_on(struct gc08a8 *gc08a8)
 {
     int ret;
     u32 delay_us;
     struct device *dev = &gc08a8->client->dev;
 
     if (!IS_ERR_OR_NULL(gc08a8->pins_default)) {
         ret = pinctrl_select_state(gc08a8->pinctrl,
                        gc08a8->pins_default);
         if (ret < 0)
             dev_err(dev, "could not set pins\n");
     }
     ret = clk_set_rate(gc08a8->xvclk, GC08a8_XVCLK_FREQ);
     if (ret < 0)
         dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
     if (clk_get_rate(gc08a8->xvclk) != GC08a8_XVCLK_FREQ)
         dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
     ret = clk_prepare_enable(gc08a8->xvclk);
     if (ret < 0) {
         dev_err(dev, "Failed to enable xvclk\n");
         return ret;
     }
     if (!IS_ERR(gc08a8->reset_gpio))
         gpiod_set_value_cansleep(gc08a8->reset_gpio, 0);
 
     if (!IS_ERR(gc08a8->pwdn_gpio))
         gpiod_set_value_cansleep(gc08a8->pwdn_gpio, 0);
 
     usleep_range(500, 1000);
     ret = regulator_bulk_enable(GC08a8_NUM_SUPPLIES, gc08a8->supplies);
 
     if (ret < 0) {
         dev_err(dev, "Failed to enable regulators\n");
         goto disable_clk;
     }
 
     if (!IS_ERR(gc08a8->pwren_gpio))
         gpiod_set_value_cansleep(gc08a8->pwren_gpio, 1);
 
     usleep_range(1000, 1100);
     if (!IS_ERR(gc08a8->pwdn_gpio))
         gpiod_set_value_cansleep(gc08a8->pwdn_gpio, 1);
     usleep_range(100, 150);
     if (!IS_ERR(gc08a8->reset_gpio))
         gpiod_set_value_cansleep(gc08a8->reset_gpio, 1);
 
     /* 8192 cycles prior to first SCCB transaction */
     delay_us = gc08a8_cal_delay(8192);
     usleep_range(delay_us, delay_us * 2);
 
     return 0;
 
 disable_clk:
     clk_disable_unprepare(gc08a8->xvclk);
 
     return ret;
 }
 
 static void __gc08a8_power_off(struct gc08a8 *gc08a8)
 {
     int ret;
     struct device *dev = &gc08a8->client->dev;
 
     if (!IS_ERR(gc08a8->pwdn_gpio))
         gpiod_set_value_cansleep(gc08a8->pwdn_gpio, 0);
     clk_disable_unprepare(gc08a8->xvclk);
     if (!IS_ERR(gc08a8->reset_gpio))
         gpiod_set_value_cansleep(gc08a8->reset_gpio, 0);
     if (!IS_ERR_OR_NULL(gc08a8->pins_sleep)) {
         ret = pinctrl_select_state(gc08a8->pinctrl,
                        gc08a8->pins_sleep);
         if (ret < 0)
             dev_dbg(dev, "could not set pins\n");
     }
     regulator_bulk_disable(GC08a8_NUM_SUPPLIES, gc08a8->supplies);
     if (!IS_ERR(gc08a8->pwren_gpio))
         gpiod_set_value_cansleep(gc08a8->pwren_gpio, 0);
 }
 
 static int gc08a8_runtime_resume(struct device *dev)
 {
     struct i2c_client *client = to_i2c_client(dev);
     struct v4l2_subdev *sd = i2c_get_clientdata(client);
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
 
     return __gc08a8_power_on(gc08a8);
 }
 
 static int gc08a8_runtime_suspend(struct device *dev)
 {
     struct i2c_client *client = to_i2c_client(dev);
     struct v4l2_subdev *sd = i2c_get_clientdata(client);
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
 
     __gc08a8_power_off(gc08a8);
 
     return 0;
 }
 
 #ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
 static int gc08a8_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
     struct v4l2_mbus_framefmt *try_fmt =
                 v4l2_subdev_get_try_format(sd, fh->pad, 0);
     const struct gc08a8_mode *def_mode = &supported_modes[0];
 
     mutex_lock(&gc08a8->mutex);
     /* Initialize try_fmt */
     try_fmt->width = def_mode->width;
     try_fmt->height = def_mode->height;
     try_fmt->code = def_mode->bus_fmt;
     try_fmt->field = V4L2_FIELD_NONE;
 
     mutex_unlock(&gc08a8->mutex);
     /* No crop or compose */
 
     return 0;
 }
 #endif
 
 static int gc08a8_enum_frame_interval(struct v4l2_subdev *sd,
                       struct v4l2_subdev_pad_config *cfg,
                 struct v4l2_subdev_frame_interval_enum *fie)
 {
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
 
     if (fie->index >= gc08a8->cfg_num)
         return -EINVAL;
 
     fie->code = supported_modes[fie->index].bus_fmt;
     fie->width = supported_modes[fie->index].width;
     fie->height = supported_modes[fie->index].height;
     fie->interval = supported_modes[fie->index].max_fps;
     fie->reserved[0] = supported_modes[fie->index].hdr_mode;
     return 0;
 }
 
 static const struct dev_pm_ops gc08a8_pm_ops = {
     SET_RUNTIME_PM_OPS(gc08a8_runtime_suspend,
                gc08a8_runtime_resume, NULL)
 };
 
 #ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
 static const struct v4l2_subdev_internal_ops gc08a8_internal_ops = {
     .open = gc08a8_open,
 };
 #endif
 
 static const struct v4l2_subdev_core_ops gc08a8_core_ops = {
     .s_power = gc08a8_s_power,
     .ioctl = gc08a8_ioctl,
 #ifdef CONFIG_COMPAT
     .compat_ioctl32 = gc08a8_compat_ioctl32,
 #endif
 };
 
 static const struct v4l2_subdev_video_ops gc08a8_video_ops = {
     .s_stream = gc08a8_s_stream,
     .g_frame_interval = gc08a8_g_frame_interval,
     .s_frame_interval = gc08a8_s_frame_interval,
 };
 
 static const struct v4l2_subdev_pad_ops gc08a8_pad_ops = {
     .enum_mbus_code = gc08a8_enum_mbus_code,
     .enum_frame_size = gc08a8_enum_frame_sizes,
     .enum_frame_interval = gc08a8_enum_frame_interval,
     .get_fmt = gc08a8_get_fmt,
     .set_fmt = gc08a8_set_fmt,
     .get_mbus_config = gc08a8_g_mbus_config,
 };
 
 static const struct v4l2_subdev_ops gc08a8_subdev_ops = {
     .core	= &gc08a8_core_ops,
     .video	= &gc08a8_video_ops,
     .pad	= &gc08a8_pad_ops,
 };
 
 static int gc08a8_set_ctrl(struct v4l2_ctrl *ctrl)
 {
     struct gc08a8 *gc08a8 = container_of(ctrl->handler,
                          struct gc08a8, ctrl_handler);
     struct i2c_client *client = gc08a8->client;
     s64 max;
     int ret = 0;
     //int val = 0;
 
     /*Propagate change of current control to all related controls*/
     switch (ctrl->id) {
     case V4L2_CID_VBLANK:
         /*Update max exposure while meeting expected vblanking*/
         max = gc08a8->cur_mode->height + ctrl->val - 4;
         __v4l2_ctrl_modify_range(gc08a8->exposure,
                      gc08a8->exposure->minimum,
                      max,
                      gc08a8->exposure->step,
                      gc08a8->exposure->default_value);
         break;
     }
 
     if (!pm_runtime_get_if_in_use(&client->dev))
         return 0;
 
     switch (ctrl->id) {
     case V4L2_CID_EXPOSURE:
         /* 4 least significant bits of expsoure are fractional part */
         ret = gc08a8_write_reg(gc08a8->client, GC08a8_REG_EXPOSURE_H,
                        GC08a8_REG_VALUE_08BIT,
                        ctrl->val >> 8);
         ret |= gc08a8_write_reg(gc08a8->client, GC08a8_REG_EXPOSURE_L,
                     GC08a8_REG_VALUE_08BIT,
                     ctrl->val & 0xff);
         break;
     case V4L2_CID_ANALOGUE_GAIN:
         ret |= gc08a8_write_reg(gc08a8->client,
             GC08a8_AGAIN_H,GC08a8_REG_VALUE_08BIT,
             (ctrl->val&0xFF00) >> 8);
         ret |= gc08a8_write_reg(gc08a8->client,
             GC08a8_AGAIN_l,GC08a8_REG_VALUE_08BIT,
             (ctrl->val&0x00FF));
         break;
     case V4L2_CID_VBLANK:
         gc08a8->cur_vts = ctrl->val + gc08a8->cur_mode->height;
         ret = gc08a8_write_reg(gc08a8->client, GC08a8_REG_VTS_H,
                        GC08a8_REG_VALUE_08BIT,
                        gc08a8->cur_vts >> 8);
         ret |= gc08a8_write_reg(gc08a8->client, GC08a8_REG_VTS_L,
                     GC08a8_REG_VALUE_08BIT,
                     gc08a8->cur_vts & 0xff);
         break;
     case V4L2_CID_TEST_PATTERN:
         ret = gc08a8_enable_test_pattern(gc08a8, ctrl->val);
         break;
     case V4L2_CID_HFLIP:
         
         break;
     case V4L2_CID_VFLIP:
         
         break;
     default:
         dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
              __func__, ctrl->id, ctrl->val);
         break;
     }
 
     pm_runtime_put(&client->dev);
 
     return ret;
 }
 
 static const struct v4l2_ctrl_ops gc08a8_ctrl_ops = {
     .s_ctrl = gc08a8_set_ctrl,
 };
 
 static int gc08a8_initialize_controls(struct gc08a8 *gc08a8)
 {
     const struct gc08a8_mode *mode;
     struct v4l2_ctrl_handler *handler;
     s64 exposure_max, vblank_def;
     u32 h_blank;
     int ret;
 
     handler = &gc08a8->ctrl_handler;
     mode = gc08a8->cur_mode;
     ret = v4l2_ctrl_handler_init(handler, 9);
     if (ret)
         return ret;
     handler->lock = &gc08a8->mutex;
 
     gc08a8->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
                            0, 0, link_freq_menu_items);
     gc08a8->cur_link_freq = 0;
     gc08a8->cur_pixel_rate = GC08a8_PIXEL_RATE_LINEAR;
 
     __v4l2_ctrl_s_ctrl(gc08a8->link_freq,
                gc08a8->cur_link_freq);
 
     gc08a8->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
               0, GC08a8_PIXEL_RATE_LINEAR, 1, GC08a8_PIXEL_RATE_LINEAR);
 
     h_blank = mode->hts_def - mode->width;
     gc08a8->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
                        h_blank, h_blank, 1, h_blank);
     if (gc08a8->hblank)
         gc08a8->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
 
     vblank_def = mode->vts_def - mode->height;
     gc08a8->cur_vts = mode->vts_def;
     gc08a8->vblank = v4l2_ctrl_new_std(handler, &gc08a8_ctrl_ops,
                        V4L2_CID_VBLANK, vblank_def,
                        GC08a8_VTS_MAX - mode->height,
                         1, vblank_def);
 
     exposure_max = mode->vts_def - 4;
     gc08a8->exposure = v4l2_ctrl_new_std(handler, &gc08a8_ctrl_ops,
                          V4L2_CID_EXPOSURE,
                          GC08a8_EXPOSURE_MIN,
                          exposure_max,
                          GC08a8_EXPOSURE_STEP,
                          mode->exp_def);
 
     gc08a8->anal_gain = v4l2_ctrl_new_std(handler, &gc08a8_ctrl_ops,
                           V4L2_CID_ANALOGUE_GAIN,
                           GC08a8_GAIN_MIN,
                           GC08a8_GAIN_MAX,
                           GC08a8_GAIN_STEP,
                           GC08a8_GAIN_DEFAULT);
 
     gc08a8->test_pattern =
         v4l2_ctrl_new_std_menu_items(handler,
                          &gc08a8_ctrl_ops,
                 V4L2_CID_TEST_PATTERN,
                 ARRAY_SIZE(gc08a8_test_pattern_menu) - 1,
                 0, 0, gc08a8_test_pattern_menu);
 
     gc08a8->h_flip = v4l2_ctrl_new_std(handler, &gc08a8_ctrl_ops,
                 V4L2_CID_HFLIP, 0, 1, 1, 0);
 
     gc08a8->v_flip = v4l2_ctrl_new_std(handler, &gc08a8_ctrl_ops,
                 V4L2_CID_VFLIP, 0, 1, 1, 0);
     if (handler->error) {
         ret = handler->error;
         dev_err(&gc08a8->client->dev,
             "Failed to init controls(%d)\n", ret);
         goto err_free_handler;
     }
 
     gc08a8->subdev.ctrl_handler = handler;
     gc08a8->has_init_exp = false;
 
     return 0;
 
 err_free_handler:
     v4l2_ctrl_handler_free(handler);
 
     return ret;
 }
 
 static int gc08a8_check_sensor_id(struct gc08a8 *gc08a8,
                   struct i2c_client *client)
 {
     struct device *dev = &gc08a8->client->dev;
     u16 id = 0;
     u32 reg_H = 0;
     u32 reg_L = 0;
     int ret;
 
     ret = gc08a8_read_reg(client, GC08a8_REG_CHIP_ID_H,
                   GC08a8_REG_VALUE_08BIT, &reg_H);
     ret |= gc08a8_read_reg(client, GC08a8_REG_CHIP_ID_L,
                    GC08a8_REG_VALUE_08BIT, &reg_L);
 
     id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
     if (!(reg_H == (CHIP_ID >> 8) || reg_L == (CHIP_ID & 0xff))) {
         dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
         return -ENODEV;
     }
     dev_info(dev, "detected gc%04x sensor\n", id);
     return 0;
 }
 
 static int gc08a8_configure_regulators(struct gc08a8 *gc08a8)
 {
     unsigned int i;
 
     for (i = 0; i < GC08a8_NUM_SUPPLIES; i++)
         gc08a8->supplies[i].supply = gc08a8_supply_names[i];
 
     return devm_regulator_bulk_get(&gc08a8->client->dev,
                        GC08a8_NUM_SUPPLIES,
                        gc08a8->supplies);
 }
 
 static int gc08a8_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
 {
     struct device *dev = &client->dev;
     struct device_node *node = dev->of_node;
     struct gc08a8 *gc08a8;
     struct v4l2_subdev *sd;
     char facing[2];
     int ret;
     u32 i, hdr_mode = 0;
 
     dev_info(dev, "driver version: %02x.%02x.%02x",
          DRIVER_VERSION >> 16,
          (DRIVER_VERSION & 0xff00) >> 8,
          DRIVER_VERSION & 0x00ff);
     
     dev_info(dev, "driver updatetime 2024-12-26 15:33:00");
 
     gc08a8 = devm_kzalloc(dev, sizeof(*gc08a8), GFP_KERNEL);
     if (!gc08a8)
         return -ENOMEM;
 
     of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
     ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
                    &gc08a8->module_index);
     ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
                        &gc08a8->module_facing);
     ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
                        &gc08a8->module_name);
     ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
                        &gc08a8->len_name);
     if (ret) {
         dev_err(dev, "could not get module information!\n");
         return -EINVAL;
     }
 
     gc08a8->client = client;
     gc08a8->cfg_num = ARRAY_SIZE(supported_modes);
     for (i = 0; i < gc08a8->cfg_num; i++) {
         if (hdr_mode == supported_modes[i].hdr_mode) {
             gc08a8->cur_mode = &supported_modes[i];
             break;
         }
     }
     if (i == gc08a8->cfg_num)
         gc08a8->cur_mode = &supported_modes[0];
 
     gc08a8->xvclk = devm_clk_get(dev, "xvclk");
     if (IS_ERR(gc08a8->xvclk)) {
         dev_err(dev, "Failed to get xvclk\n");
         return -EINVAL;
     }
 
     gc08a8->pwren_gpio = devm_gpiod_get(dev, "pwren", GPIOD_OUT_LOW);
     if (IS_ERR(gc08a8->pwren_gpio))
         dev_warn(dev, "Failed to get pwren-gpios\n");
 
     gc08a8->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
     if (IS_ERR(gc08a8->reset_gpio))
         dev_warn(dev, "Failed to get reset-gpios\n");
 
     gc08a8->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
     if (IS_ERR(gc08a8->pwdn_gpio))
         dev_warn(dev, "Failed to get pwdn-gpios\n");
 
     gc08a8->pinctrl = devm_pinctrl_get(dev);
     if (!IS_ERR(gc08a8->pinctrl)) {
         gc08a8->pins_default =
             pinctrl_lookup_state(gc08a8->pinctrl,
                          OF_CAMERA_PINCTRL_STATE_DEFAULT);
         if (IS_ERR(gc08a8->pins_default))
             dev_err(dev, "could not get default pinstate\n");
 
         gc08a8->pins_sleep =
             pinctrl_lookup_state(gc08a8->pinctrl,
                          OF_CAMERA_PINCTRL_STATE_SLEEP);
         if (IS_ERR(gc08a8->pins_sleep))
             dev_err(dev, "could not get sleep pinstate\n");
     } else {
         dev_err(dev, "no pinctrl\n");
     }
 
     ret = gc08a8_configure_regulators(gc08a8);
     if (ret) {
         dev_err(dev, "Failed to get power regulators\n");
         return ret;
     }
 
     mutex_init(&gc08a8->mutex);
 
     sd = &gc08a8->subdev;
     v4l2_i2c_subdev_init(sd, client, &gc08a8_subdev_ops);
     ret = gc08a8_initialize_controls(gc08a8);
     if (ret)
         goto err_destroy_mutex;
 
     ret = __gc08a8_power_on(gc08a8);
     if (ret)
         goto err_free_handler;
 
     usleep_range(3000, 4000);
 
     ret = gc08a8_check_sensor_id(gc08a8, client);
     if (ret)
         goto err_power_off;
 
 #ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
     sd->internal_ops = &gc08a8_internal_ops;
     sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
              V4L2_SUBDEV_FL_HAS_EVENTS;
 #endif
 #if defined(CONFIG_MEDIA_CONTROLLER)
     gc08a8->pad.flags = MEDIA_PAD_FL_SOURCE;
     sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
     ret = media_entity_pads_init(&sd->entity, 1, &gc08a8->pad);
     if (ret < 0)
         goto err_power_off;
 #endif
 
     memset(facing, 0, sizeof(facing));
     if (strcmp(gc08a8->module_facing, "back") == 0)
         facing[0] = 'b';
     else
         facing[0] = 'f';
 
     snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
          gc08a8->module_index, facing,
          GC08a8_NAME, dev_name(sd->dev));
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
     __gc08a8_power_off(gc08a8);
 err_free_handler:
     v4l2_ctrl_handler_free(&gc08a8->ctrl_handler);
 err_destroy_mutex:
     mutex_destroy(&gc08a8->mutex);
 
     return ret;
 }
 
 static int gc08a8_remove(struct i2c_client *client)
 {
     struct v4l2_subdev *sd = i2c_get_clientdata(client);
     struct gc08a8 *gc08a8 = to_gc08a8(sd);
 
     v4l2_async_unregister_subdev(sd);
 #if defined(CONFIG_MEDIA_CONTROLLER)
     media_entity_cleanup(&sd->entity);
 #endif
     v4l2_ctrl_handler_free(&gc08a8->ctrl_handler);
     mutex_destroy(&gc08a8->mutex);
 
     pm_runtime_disable(&client->dev);
     if (!pm_runtime_status_suspended(&client->dev))
         __gc08a8_power_off(gc08a8);
     pm_runtime_set_suspended(&client->dev);
 
     return 0;
 }
 
 #if IS_ENABLED(CONFIG_OF)
 static const struct of_device_id gc08a8_of_match[] = {
     { .compatible = "galaxycore,gc08a8" },
     {},
 };
 MODULE_DEVICE_TABLE(of, gc08a8_of_match);
 #endif
 
 static const struct i2c_device_id gc08a8_match_id[] = {
     { "galaxycore,gc08a8", 0 },
     { },
 };
 
 static struct i2c_driver gc08a8_i2c_driver = {
     .driver = {
         .name = GC08a8_NAME,
         .pm = &gc08a8_pm_ops,
         .of_match_table = of_match_ptr(gc08a8_of_match),
     },
     .probe		= &gc08a8_probe,
     .remove		= &gc08a8_remove,
     .id_table	= gc08a8_match_id,
 };
 
 static int __init sensor_mod_init(void)
 {
     return i2c_add_driver(&gc08a8_i2c_driver);
 }
 
 static void __exit sensor_mod_exit(void)
 {
     i2c_del_driver(&gc08a8_i2c_driver);
 }
 
 device_initcall_sync(sensor_mod_init);
 module_exit(sensor_mod_exit);
 
 MODULE_DESCRIPTION("galaxycore gc08a8 sensor driver");
 MODULE_LICENSE("GPL");
 
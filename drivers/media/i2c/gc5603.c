// SPDX-License-Identifier: GPL-2.0
/*
 * gc5603 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 add enum_frame_interval function.
 * V0.0X01.0X04 add quick stream on/off
 * V0.0X01.0X05 add function g_mbus_config
 * V0.0X01.0X06
 * 1. add 2lane support.
 * 2. add some debug info.
 * 3. adjust gc5603_g_mbus_config function.
 * V0.0X01.0X07 support get channel info
 * V0.0X01.0X08
 * 1. default support 2lane full 30fps.
 * 2. default support rk otp spec.
 * V0.0X01.0X09 adjust supply sequence to suit spec
 */
//#define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/of_graph.h>

#define DRIVER_VERSION          KERNEL_VERSION(0, 0x01, 0x08)
#define GC5603_NAME             "gc5603"

#define MIPI_FREQ_848M          423000000

#define GC5603_PAGE_SELECT      0xFE

#define GC5603_REG_CHIP_ID_H    0x03F0
#define GC5603_REG_CHIP_ID_L    0x03F1

#define GC5603_REG_EXP_H        0x0202
#define GC5603_REG_EXP_L        0x0203

#define GC5603_REG_VTS_H        0x0340
#define GC5603_REG_VTS_L        0x0341

#define GC5603_REG_CTRL_MODE    0x0100
#define GC5603_MODE_SW_STANDBY  0x00
#define GC5603_MODE_STREAMING   0x09

#define REG_NULL                0xFFFF

#define GC5603_CHIP_ID          0x5603

#define GC5603_VTS_MAX          0x7fff
#define GC5603_HTS_MAX          0xFFF

#define GC5603_EXPOSURE_MAX     0x3FFF
#define GC5603_EXPOSURE_MIN     1
#define GC5603_EXPOSURE_STEP    1

#define GC5603_GAIN_MIN         64
#define GC5603_GAIN_MAX         0xffff
#define GC5603_GAIN_STEP        1
#define GC5603_GAIN_DEFAULT     64

#define gc5603_REG_VALUE_08BIT		1
#define gc5603_REG_VALUE_16BIT		2
#define gc5603_REG_VALUE_24BIT		3


#define GC5603_LANES            2

#define OF_CAMERA_PINCTRL_STATE_DEFAULT "rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP   "rockchip,camera_sleep"



#define GC5603_FLIP_MIRROR_REG  0x0101

#define GC_MIRROR_BIT_MASK      BIT(0)
#define GC_FLIP_BIT_MASK        BIT(1)

#define GC5603_XVCLK_FREQ_24M		24000000
#define GC5603_XVCLK_FREQ_27M       27000000

static const char * const gc5603_supply_names[] = {
	"dovdd",    /* Digital I/O power */
	"avdd",     /* Analog power */
	"dvdd",     /* Digital core power */
};

#define GC5603_NUM_SUPPLIES ARRAY_SIZE(gc5603_supply_names)

#define to_gc5603(sd) container_of(sd, struct gc5603, subdev)

struct regval {
	u16 addr;
	u8 val;
};

struct gc5603_mode {
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
	u32 xvclk;
};

struct gc5603 {
	struct i2c_client   *client;
	struct clk      *xvclk;
	struct gpio_desc    *reset_gpio;
	struct gpio_desc    *pwdn_gpio;
	struct gpio_desc    *pwren_gpio;
	struct regulator_bulk_data supplies[GC5603_NUM_SUPPLIES];

	struct pinctrl      	*pinctrl;
	struct pinctrl_state    *pins_default;
	struct pinctrl_state    *pins_sleep;

	struct v4l2_subdev  subdev;
	struct media_pad    pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl    *exposure;
	struct v4l2_ctrl    *anal_gain;
	struct v4l2_ctrl    *hblank;
	struct v4l2_ctrl    *vblank;
	struct v4l2_ctrl    *h_flip;
	struct v4l2_ctrl    *v_flip;
	struct mutex        mutex;
	bool            streaming;
	unsigned int        lane_num;
	unsigned int        cfg_num;
	unsigned int        pixel_rate;
	bool			power_on;
	const struct gc5603_mode *cur_mode;


	u32         	module_index;
	const char      *module_facing;
	const char      *module_name;
	const char      *len_name;
	struct rkmodule_awb_cfg awb_cfg;
	struct rkmodule_lsc_cfg lsc_cfg;
	u32			flip;
};


static const struct regval gc5603_2960x1666_regs_2lane[] = {
//version 1.3
//mclk 27Mhz
//mipi 2 lane  846Mbps/lane
//vts = 1750 ,row_time=19.05us
//window 2960x1666
//BGGR
    {0x03fe, 0xf0},
	{0x03fe, 0x00},
	{0x03fe, 0x10},
	{0x03fe, 0x00},
	{0x0a38, 0x02},
	{0x0a38, 0x03},
	{0x0a20, 0x07},
	{0x061b, 0x03},
	{0x061c, 0x50},
	{0x061d, 0x05},
	{0x061e, 0x70},
	{0x061f, 0x03},
	{0x0a21, 0x08},
	{0x0a34, 0x40},
	{0x0a35, 0x11},
	{0x0a36, 0x5e},
	{0x0a37, 0x03},
	{0x0314, 0x50},
	{0x0315, 0x32},
	{0x031c, 0xce},
	{0x0219, 0x57},
	{0x0342, 0x04},
	{0x0343, 0xb0},
	{0x0340, 0x06},
	{0x0341, 0xd6},
	{0x0345, 0x02},
	{0x0347, 0x02},
	{0x0348, 0x0b},
	{0x0349, 0x98},
	{0x034a, 0x06},
	{0x034b, 0x8a},
	{0x0094, 0x0b},
	{0x0095, 0x90},
	{0x0096, 0x06},
	{0x0097, 0x82},
	{0x0099, 0x04},
	{0x009b, 0x04},
	{0x060c, 0x01},
	{0x060e, 0xd2},
	{0x060f, 0x05},
	{0x070c, 0x01},
	{0x070e, 0xd2},
	{0x070f, 0x05},
	{0x0709, 0x40},
	{0x0719, 0x40},
	{0x0909, 0x07},
	{0x0902, 0x04},
	{0x0904, 0x0b},
	{0x0907, 0x54},
	{0x0908, 0x06},
	{0x0903, 0x9d},
	{0x072a, 0x1c},
	{0x072b, 0x1c},
	{0x0724, 0x2b},
	{0x0727, 0x2b},
	{0x1466, 0x18},
	{0x1467, 0x15},
	{0x1468, 0x15},
	{0x1469, 0x70},
	{0x146a, 0xe8},
	{0x0707, 0x07},
	{0x0737, 0x0f},
	{0x0704, 0x01},
	{0x0706, 0x02},
	{0x0716, 0x02},
	{0x0708, 0xc8},
	{0x0718, 0xc8},
	{0x061a, 0x02},
	{0x1430, 0x80},
	{0x1407, 0x10},
	{0x1408, 0x16},
	{0x1409, 0x03},
	{0x1438, 0x01},
	{0x02ce, 0x03},
	{0x0245, 0xc9},
	{0x023a, 0x08},
	{0x02cd, 0x88},
	{0x0612, 0x02},
	{0x0613, 0xc7},
	{0x0243, 0x03},
	{0x0089, 0x03},
	{0x0002, 0xab},		
	{0x0040, 0xa3},
	{0x0075, 0x64},
	{0x0004, 0x0f},
	{0x0053, 0x0a},
	{0x0205, 0x0c},
	{0x0a67, 0x80},
	{0x0a54, 0x0e},
	{0x0a65, 0x10},
	{0x0a98, 0x04},
	{0x05be, 0x00},
	{0x05a9, 0x01},
	{0x0023, 0x00},
	{0x0022, 0x00},
	{0x0025, 0x00},
	{0x0024, 0x00},
	{0x0028, 0x0b},
	{0x0029, 0x98},
	{0x002a, 0x06},
	{0x002b, 0x86},
	{0x0a83, 0xe0},
	{0x0a72, 0x02},
	{0x0a73, 0x60},
	{0x0a75, 0x41},
	{0x0a70, 0x03},
	{0x0a5a, 0x80},
	{0x0181, 0x30},
	{0x0182, 0x05},
	{0x0185, 0x01},
	{0x0180, 0x46},
	{0x0100, 0x08},
	{0x010d, 0x74},
	{0x010e, 0x0e},
	{0x0113, 0x02},
	{0x0114, 0x01},
	{0x0115, 0x10},
	{0x0100, 0x09},
	{0x0a70, 0x00},
	{0x0080, 0x02},
	{0x0a67, 0x00},
	{0x0052, 0x02},
	{0x0076, 0x01},
	{0x021a, 0x10},
	{0x0049, 0x0f},
	{0x004a, 0x3c},
	{0x004b, 0x00},
	{0x0430, 0x25},
	{0x0431, 0x25},
	{0x0432, 0x25},
	{0x0433, 0x25},
	{0x0434, 0x59},
	{0x0435, 0x59},
	{0x0436, 0x59},
	{0x0437, 0x59},
           
    {0x0100, 0x09},
    {REG_NULL, 0x00},
};

static const struct gc5603_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SGRBG10_1X10,
		.width = 2960,
		.height = 1666,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x6ce,
		.hts_def = 0x0C80,
		.vts_def = 0x06D6,
		.reg_list = gc5603_2960x1666_regs_2lane,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.xvclk = GC5603_XVCLK_FREQ_24M
	},
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_848M
};
static int gc5603_write_reg(struct i2c_client *client, u16 reg,
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

static int gc5603_write_array(struct i2c_client *client,
                               const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = gc5603_write_reg(client, regs[i].addr,
		                        gc5603_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int gc5603_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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
	dev_info(&client->dev,
		"gc5603 read reg(0x%x val:0x%x) \n", reg, *val);
}
static int gc5603_get_reso_dist(const struct gc5603_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		   abs(mode->height - framefmt->height);
}

static const struct gc5603_mode *
gc5603_find_best_fit(struct gc5603 *gc5603, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < gc5603->cfg_num; i++) {
		dist = gc5603_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static uint8_t regValTable[26][7] = {
   //0614, 0615, 0225, 1467  1468, 00b8, 00b9  
	{ 0x00, 0x00, 0x04, 0x15, 0x15, 0x01, 0x00},
	{ 0x90, 0x02, 0x04, 0x15, 0x15, 0x01, 0x0A},
	{ 0x00, 0x00, 0x00, 0x15, 0x15, 0x01, 0x12},
	{ 0x90, 0x02, 0x00, 0x15, 0x15, 0x01, 0x20},
	{ 0x01, 0x00, 0x00, 0x15, 0x15, 0x01, 0x30},
	{ 0x91, 0x02, 0x00, 0x15, 0x15, 0x02, 0x05},
	{ 0x02, 0x00, 0x00, 0x15, 0x15, 0x02, 0x19},
	{ 0x92, 0x02, 0x00, 0x16, 0x16, 0x02, 0x3F},
	{ 0x03, 0x00, 0x00, 0x16, 0x16, 0x03, 0x20},
	{ 0x93, 0x02, 0x00, 0x17, 0x17, 0x04, 0x0A},
	{ 0x00, 0x00, 0x01, 0x18, 0x18, 0x05, 0x02},
	{ 0x90, 0x02, 0x01, 0x19, 0x19, 0x05, 0x39},
	{ 0x01, 0x00, 0x01, 0x19, 0x19, 0x06, 0x3C},
	{ 0x91, 0x02, 0x01, 0x19, 0x19, 0x08, 0x0D},
	{ 0x02, 0x00, 0x01, 0x1a, 0x1a, 0x09, 0x21},
	{ 0x92, 0x02, 0x01, 0x1a, 0x1a, 0x0B, 0x0F},
	{ 0x03, 0x00, 0x01, 0x1c, 0x1c, 0x0D, 0x17},
	{ 0x93, 0x02, 0x01, 0x1c, 0x1c, 0x0F, 0x33},
	{ 0x04, 0x00, 0x01, 0x1d, 0x1d, 0x12, 0x30},
	{ 0x94, 0x02, 0x01, 0x1d, 0x1d, 0x16, 0x10},
	{ 0x05, 0x00, 0x01, 0x1e, 0x1e, 0x1A, 0x19},
	{ 0x95, 0x02, 0x01, 0x1e, 0x1e, 0x1F, 0x13},
	{ 0x06, 0x00, 0x01, 0x20, 0x20, 0x25, 0x08},
	{ 0x96, 0x02, 0x01, 0x20, 0x20, 0x2C, 0x03},
	{ 0xb6, 0x04, 0x01, 0x20, 0x20, 0x34, 0x0F},
	{ 0x86, 0x06, 0x01, 0x20, 0x20, 0x3D, 0x3D},
};

static uint32_t gain_level_table[27] = {
			64,
			74,
			82,
			96,
			112,
			133,	 
			153,
			191,
			224,
			266,
			322,
			377,
			444,
			525,
			609,
			719,
			855,
			1011,
			1200,
			1424,
			1689,
			2003,
			
			2376,
			2819,
			
			3343,
			3965,
			0xffffffff,
};
//static int total = sizeof(gain_level_table) / sizeof(uint32_t);

static int gc5603_set_gain(struct gc5603 *gc5603, u32 gain)
{
	int ret;
	uint16_t i = 0;
	uint16_t total = 0;
	uint16_t temp = 0;


	for (i = 0; i < total; i++) {
		if ((gain_level_table[i] <= gain) && (gain < gain_level_table[i+1]))
			break;
	}

	if((gain>3965)||(gain==3965))
		i =25;
	

	ret = gc5603_write_reg(gc5603->client, 0x031d,gc5603_REG_VALUE_08BIT, 0x2d);
	ret = gc5603_write_reg(gc5603->client, 0x0614,gc5603_REG_VALUE_08BIT,regValTable[i][0]);
	ret = gc5603_write_reg(gc5603->client, 0x0615,gc5603_REG_VALUE_08BIT,regValTable[i][1]);
	ret = gc5603_write_reg(gc5603->client, 0x0225,gc5603_REG_VALUE_08BIT,regValTable[i][2]);

	ret = gc5603_write_reg(gc5603->client, 0x031d,gc5603_REG_VALUE_08BIT, 0x28);
	ret = gc5603_write_reg(gc5603->client, 0x1467,gc5603_REG_VALUE_08BIT,regValTable[i][3]);
	ret = gc5603_write_reg(gc5603->client, 0x1468,gc5603_REG_VALUE_08BIT,regValTable[i][4]);
	ret = gc5603_write_reg(gc5603->client, 0x00b8,gc5603_REG_VALUE_08BIT,regValTable[i][5]);
	ret = gc5603_write_reg(gc5603->client, 0x00b9,gc5603_REG_VALUE_08BIT,regValTable[i][6]);
	

	temp = 64 * gain / gain_level_table[i];
	
	//dev_warn(&client->dev, "gc5603_set_gain gain=%d,i=%d,temp=%d \n", gain, i, temp);
	
	ret |= gc5603_write_reg(gc5603->client, 0x0064,gc5603_REG_VALUE_08BIT,(temp >> 6));
	ret |= gc5603_write_reg(gc5603->client, 0x0065,gc5603_REG_VALUE_08BIT,((temp&0x3f) << 2) );

	return ret;
}

static int gc5603_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc5603 *gc5603 = container_of(ctrl->handler,
						 struct gc5603, ctrl_handler);
	struct i2c_client *client = gc5603->client;
	s64 max;
	int ret = 0;
	u32 vts = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc5603->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(gc5603->exposure,
					 gc5603->exposure->minimum, max,
					 gc5603->exposure->step,
					 gc5603->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		
	#if 1
		ret = gc5603_write_reg(gc5603->client, GC5603_REG_EXP_H,gc5603_REG_VALUE_08BIT,
					   (ctrl->val >> 8));
		ret |= gc5603_write_reg(gc5603->client, GC5603_REG_EXP_L,gc5603_REG_VALUE_08BIT,
					   ctrl->val & 0xff);			   
    #endif

		break;
	case V4L2_CID_ANALOGUE_GAIN:
		gc5603_set_gain(gc5603, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + gc5603->cur_mode->height;

	#if 1
		ret = gc5603_write_reg(gc5603->client, GC5603_REG_VTS_H,gc5603_REG_VALUE_08BIT,(vts >> 8));
		ret |= gc5603_write_reg(gc5603->client, GC5603_REG_VTS_L, gc5603_REG_VALUE_08BIT,vts & 0xff);
    #endif
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			gc5603->flip |= GC_MIRROR_BIT_MASK;
		else
			gc5603->flip &= ~GC_MIRROR_BIT_MASK;
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			gc5603->flip |= GC_FLIP_BIT_MASK;
		else
			gc5603->flip &= ~GC_FLIP_BIT_MASK;
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gc5603_ctrl_ops = {
	.s_ctrl = gc5603_set_ctrl,
};

static int gc5603_configure_regulators(struct gc5603 *gc5603)
{
	unsigned int i;

	for (i = 0; i < GC5603_NUM_SUPPLIES; i++)
		gc5603->supplies[i].supply = gc5603_supply_names[i];

	return devm_regulator_bulk_get(&gc5603->client->dev,
					   GC5603_NUM_SUPPLIES,
					   gc5603->supplies);
}

static int gc5603_parse_of(struct gc5603 *gc5603)
{
	struct device *dev = &gc5603->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int rval;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	fwnode = of_fwnode_handle(endpoint);
	rval = fwnode_property_read_u32_array(fwnode, "data-lanes", NULL, 0);
	if (rval <= 0) {
		dev_warn(dev, " Get mipi lane num failed!\n");
		return -1;
	}

	gc5603->lane_num = rval;
	if (2 == gc5603->lane_num) {
		gc5603->cur_mode = &supported_modes[0];
		gc5603->cfg_num = ARRAY_SIZE(supported_modes);

		/*pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
		gc5603->pixel_rate = MIPI_FREQ_848M * 2U * (gc5603->lane_num) / 10U;
		dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
				 gc5603->lane_num, gc5603->pixel_rate);
	} else {
		dev_info(dev, "gc5603 can not support the lane num(%d)\n", gc5603->lane_num);
	}
	return 0;
}

static int gc5603_initialize_controls(struct gc5603 *gc5603)
{
	const struct gc5603_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc5603->ctrl_handler;
	mode = gc5603->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &gc5603->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
					  0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, gc5603->pixel_rate, 1, gc5603->pixel_rate);

	h_blank = mode->hts_def - mode->width;
	gc5603->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (gc5603->hblank)
		gc5603->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc5603->vblank = v4l2_ctrl_new_std(handler, &gc5603_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				GC5603_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	gc5603->exposure = v4l2_ctrl_new_std(handler, &gc5603_ctrl_ops,
				V4L2_CID_EXPOSURE, 
				GC5603_EXPOSURE_MIN,
				exposure_max, 
				GC5603_EXPOSURE_STEP,
				mode->exp_def);

	gc5603->anal_gain = v4l2_ctrl_new_std(handler, &gc5603_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN,
			    GC5603_GAIN_MIN,
				GC5603_GAIN_MAX, 
				GC5603_GAIN_STEP,
				GC5603_GAIN_DEFAULT);

	gc5603->h_flip = v4l2_ctrl_new_std(handler, &gc5603_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);

	gc5603->v_flip = v4l2_ctrl_new_std(handler, &gc5603_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	gc5603->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc5603->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc5603->subdev.ctrl_handler = handler;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc5603_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC5603_XVCLK_FREQ_24M / 1000 / 1000);
}

static int __gc5603_power_on(struct gc5603 *gc5603)
{
	int ret;
	u32 delay_us;
	struct device *dev = &gc5603->client->dev;

	if (!IS_ERR_OR_NULL(gc5603->pins_default)) {
		ret = pinctrl_select_state(gc5603->pinctrl,
					   gc5603->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(gc5603->xvclk, GC5603_XVCLK_FREQ_24M);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(gc5603->xvclk) != GC5603_XVCLK_FREQ_24M)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(gc5603->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(gc5603->reset_gpio))
		gpiod_set_value_cansleep(gc5603->reset_gpio, 0);

	if (!IS_ERR(gc5603->pwdn_gpio))
		gpiod_set_value_cansleep(gc5603->pwdn_gpio, 0);

	usleep_range(500, 1000);
	ret = regulator_bulk_enable(GC5603_NUM_SUPPLIES, gc5603->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}
	if (!IS_ERR(gc5603->pwren_gpio))
		gpiod_set_value_cansleep(gc5603->pwren_gpio, 1);

	usleep_range(1000, 1100);
	if (!IS_ERR(gc5603->pwdn_gpio))
		gpiod_set_value_cansleep(gc5603->pwdn_gpio, 1);
	usleep_range(100, 150);
	if (!IS_ERR(gc5603->reset_gpio))
		gpiod_set_value_cansleep(gc5603->reset_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc5603_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;

disable_clk:
	clk_disable_unprepare(gc5603->xvclk);
	return ret;
}

static void __gc5603_power_off(struct gc5603 *gc5603)
{
	int ret;
	struct device *dev = &gc5603->client->dev;

	if (!IS_ERR(gc5603->pwdn_gpio))
		gpiod_set_value_cansleep(gc5603->pwdn_gpio, 0);
	clk_disable_unprepare(gc5603->xvclk);

	if (!IS_ERR(gc5603->reset_gpio))
		gpiod_set_value_cansleep(gc5603->reset_gpio, 0);

	if (!IS_ERR_OR_NULL(gc5603->pins_sleep)) {
		ret = pinctrl_select_state(gc5603->pinctrl,
					   gc5603->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(GC5603_NUM_SUPPLIES, gc5603->supplies);
	if (!IS_ERR(gc5603->pwren_gpio))
		gpiod_set_value_cansleep(gc5603->pwren_gpio, 0);
	
}

   static int gc5603_check_sensor_id(struct gc5603 *gc5603,
				  struct i2c_client *client)
{
	struct device *dev = &gc5603->client->dev;
	u16 id = 0;
	u32 reg_H = 0;
	u32 reg_L = 0;
	int ret;

	ret = gc5603_read_reg(client, GC5603_REG_CHIP_ID_H,
			      gc5603_REG_VALUE_08BIT, &reg_H);
	ret |= gc5603_read_reg(client, GC5603_REG_CHIP_ID_L,
			       gc5603_REG_VALUE_08BIT, &reg_L);

	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (!(reg_H == (GC5603_CHIP_ID >> 8) || reg_L == (GC5603_CHIP_ID & 0xff))) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "detected gc%04x sensor\n", id);
	return 0;
}

static int gc5603_set_flip(struct gc5603 *gc5603, u8 mode)
{
u32 match_reg = 0;

	gc5603_read_reg(gc5603->client, GC5603_FLIP_MIRROR_REG,gc5603_REG_VALUE_08BIT,&match_reg);

	if (mode == GC_FLIP_BIT_MASK) {
		match_reg |= GC_FLIP_BIT_MASK;
		match_reg &= ~GC_MIRROR_BIT_MASK;
	} else if (mode == GC_MIRROR_BIT_MASK) {
		match_reg |= GC_MIRROR_BIT_MASK;
		match_reg &= ~GC_FLIP_BIT_MASK;
	} else if (mode == (GC_MIRROR_BIT_MASK |
		GC_FLIP_BIT_MASK)) {
		match_reg |= GC_FLIP_BIT_MASK;
		match_reg |= GC_MIRROR_BIT_MASK;
	} else {
		match_reg &= ~GC_FLIP_BIT_MASK;
		match_reg &= ~GC_MIRROR_BIT_MASK;
	}
	return gc5603_write_reg(gc5603->client, GC5603_FLIP_MIRROR_REG,gc5603_REG_VALUE_08BIT ,match_reg);
}

static int __gc5603_start_stream(struct gc5603 *gc5603)
{
	int ret;

	ret = gc5603_write_array(gc5603->client, gc5603->cur_mode->reg_list);
	if (ret)
		return ret;

	usleep_range(1000, 1100);


	gc5603_write_reg(gc5603->client, 0x0a70,gc5603_REG_VALUE_08BIT, 0x00);
	gc5603_write_reg(gc5603->client, 0x0080,gc5603_REG_VALUE_08BIT, 0x02);
	gc5603_write_reg(gc5603->client, 0x0a67,gc5603_REG_VALUE_08BIT, 0x00);




	/* In case these controls are set before streaming */
	mutex_unlock(&gc5603->mutex);
	ret = __v4l2_ctrl_handler_setup(&gc5603->ctrl_handler);
	mutex_lock(&gc5603->mutex);

	ret = gc5603_set_flip(gc5603, gc5603->flip);
	if (ret)
		return ret;
	return gc5603_write_reg(gc5603->client, GC5603_REG_CTRL_MODE,gc5603_REG_VALUE_08BIT,
							GC5603_MODE_STREAMING);
}

static int __gc5603_stop_stream(struct gc5603 *gc5603)
{
	return gc5603_write_reg(gc5603->client, GC5603_REG_CTRL_MODE,gc5603_REG_VALUE_08BIT,
							GC5603_MODE_SW_STANDBY);
}

static void gc5603_get_module_inf(struct gc5603 *gc5603,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, GC5603_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, gc5603->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, gc5603->len_name, sizeof(inf->base.lens));
}

static int gc5603_get_channel_info(struct gc5603 *gc5603, struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;
	ch_info->vc = gc5603->cur_mode->vc[ch_info->index];
	ch_info->width = gc5603->cur_mode->width;
	ch_info->height = gc5603->cur_mode->height;
	ch_info->bus_fmt = gc5603->cur_mode->bus_fmt;
	return 0;
}

static void gc5603_set_awb_cfg(struct gc5603 *gc5603,
				   struct rkmodule_awb_cfg *cfg)
{
	mutex_lock(&gc5603->mutex);
	memcpy(&gc5603->awb_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&gc5603->mutex);
}

static void gc5603_set_lsc_cfg(struct gc5603 *gc5603,
				   struct rkmodule_lsc_cfg *cfg)
{
	mutex_lock(&gc5603->mutex);
	memcpy(&gc5603->lsc_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&gc5603->mutex);
}

static long gc5603_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc5603 *gc5603 = to_gc5603(sd);
	long ret = 0;
	struct rkmodule_hdr_cfg *hdr_cfg;
	u32 stream = 0;
	struct rkmodule_channel_info *ch_info;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		gc5603_get_module_inf(gc5603, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = gc5603->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
	case RKMODULE_SET_CONVERSION_GAIN:
		break;

	case RKMODULE_AWB_CFG:
		gc5603_set_awb_cfg(gc5603, (struct rkmodule_awb_cfg *)arg);
		break;
	case RKMODULE_LSC_CFG:
		gc5603_set_lsc_cfg(gc5603, (struct rkmodule_lsc_cfg *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = gc5603_write_reg(gc5603->client, GC5603_REG_CTRL_MODE,gc5603_REG_VALUE_08BIT,
					       GC5603_MODE_STREAMING);
		else
			ret = gc5603_write_reg(gc5603->client, GC5603_REG_CTRL_MODE,gc5603_REG_VALUE_08BIT,
					       GC5603_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = gc5603_get_channel_info(gc5603, ch_info);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long gc5603_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *awb_cfg;
	struct rkmodule_lsc_cfg *lsc_cfg;
	struct rkmodule_hdr_cfg *hdr;
	long ret = 0;
	u32 cg = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc5603_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		awb_cfg = kzalloc(sizeof(*awb_cfg), GFP_KERNEL);
		if (!awb_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(awb_cfg, up, sizeof(*awb_cfg));
		if (!ret)
			ret = gc5603_ioctl(sd, cmd, awb_cfg);
		kfree(awb_cfg);
		break;
	case RKMODULE_LSC_CFG:
		lsc_cfg = kzalloc(sizeof(*lsc_cfg), GFP_KERNEL);
		if (!lsc_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(lsc_cfg, up, sizeof(*lsc_cfg));
		if (!ret)
			ret = gc5603_ioctl(sd, cmd, lsc_cfg);
		kfree(lsc_cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc5603_ioctl(sd, cmd, hdr);
		if (!ret)
			ret = copy_to_user(up, hdr, sizeof(*hdr));
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
			ret = gc5603_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		ret = copy_from_user(&cg, up, sizeof(cg));
		if (!ret)
			ret = gc5603_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = gc5603_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}
#endif

static int gc5603_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc5603 *gc5603 = to_gc5603(sd);
	struct i2c_client *client = gc5603->client;
	int ret = 0;

	mutex_lock(&gc5603->mutex);
	on = !!on;
	if (on == gc5603->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gc5603_start_stream(gc5603);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc5603_stop_stream(gc5603);
		pm_runtime_put(&client->dev);
	}

	gc5603->streaming = on;

unlock_and_return:
	mutex_unlock(&gc5603->mutex);
	return 0;
}

static int gc5603_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc5603 *gc5603 = to_gc5603(sd);
	const struct gc5603_mode *mode = gc5603->cur_mode;

	mutex_lock(&gc5603->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc5603->mutex);

	return 0;
}

static int gc5603_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct gc5603 *gc5603 = to_gc5603(sd);
	const struct gc5603_mode *mode = gc5603->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (GC5603_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;
	return 0;
}

static int gc5603_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = supported_modes[code->index].bus_fmt;
	return 0;
}

static int gc5603_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct gc5603 *gc5603 = to_gc5603(sd);

	if (fse->index >= gc5603->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

#define DST_WIDTH 2720
#define DST_HEIGHT 1616

/*
 * The resolution of the driver configuration needs to be exactly
 * the same as the current output resolution of the sensor,
 * the input width of the isp needs to be 16 aligned,
 * the input height of the isp needs to be 8 aligned.
 * Can be cropped to standard resolution by this function,
 * otherwise it will crop out strange resolution according
 * to the alignment rules.
 */
static int gc5603_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sel->r.left =120;
		sel->r.width = DST_WIDTH;
		sel->r.top = 25;
		sel->r.height = DST_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

static int gc5603_enum_frame_interval(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gc5603 *gc5603 = to_gc5603(sd);

	if (fie->index >= gc5603->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static int gc5603_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc5603 *gc5603 = to_gc5603(sd);
	const struct gc5603_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc5603->mutex);

	mode = gc5603_find_best_fit(gc5603, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc5603->mutex);
		return -ENOTTY;
#endif
	} else {
		gc5603->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc5603->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc5603->vblank, vblank_def,
					 GC5603_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&gc5603->mutex);
	return 0;
}

static int gc5603_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc5603 *gc5603 = to_gc5603(sd);
	const struct gc5603_mode *mode = gc5603->cur_mode;

	mutex_lock(&gc5603->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc5603->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;

		/* format info: width/height/data type/virctual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];

	}
	mutex_unlock(&gc5603->mutex);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc5603_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc5603 *gc5603 = to_gc5603(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc5603_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc5603->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&gc5603->mutex);
	/* No crop or compose */
	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc5603_internal_ops = {
	.open = gc5603_open,
};
#endif

static int gc5603_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc5603 *gc5603 = to_gc5603(sd);
	struct i2c_client *client = gc5603->client;
	int ret = 0;

	mutex_lock(&gc5603->mutex);

	/* If the power state is not modified - no work to do. */
	if (gc5603->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		gc5603->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		gc5603->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc5603->mutex);

	return ret;
}

static const struct v4l2_subdev_core_ops gc5603_core_ops = {
	.s_power = gc5603_s_power,
	.ioctl = gc5603_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc5603_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc5603_video_ops = {
	.s_stream = gc5603_s_stream,
	.g_frame_interval = gc5603_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc5603_pad_ops = {
	.enum_mbus_code = gc5603_enum_mbus_code,
	.enum_frame_size = gc5603_enum_frame_sizes,
	.enum_frame_interval = gc5603_enum_frame_interval,
	.get_fmt = gc5603_get_fmt,
	.set_fmt = gc5603_set_fmt,
	.get_selection = gc5603_get_selection,
	.get_mbus_config = gc5603_g_mbus_config,
};

static const struct v4l2_subdev_ops gc5603_subdev_ops = {
	.core   = &gc5603_core_ops,
	.video  = &gc5603_video_ops,
	.pad    = &gc5603_pad_ops,
};

static int gc5603_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc5603 *gc5603 = to_gc5603(sd);

	__gc5603_power_on(gc5603);
	return 0;
}

static int gc5603_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc5603 *gc5603 = to_gc5603(sd);

	__gc5603_power_off(gc5603);
	return 0;
}

static const struct dev_pm_ops gc5603_pm_ops = {
	SET_RUNTIME_PM_OPS(gc5603_runtime_suspend,
					   gc5603_runtime_resume, NULL)
};

static int gc5603_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc5603 *gc5603;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	gc5603 = devm_kzalloc(dev, sizeof(*gc5603), GFP_KERNEL);
	if (!gc5603)
		return -ENOMEM;

	gc5603->client = client;
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &gc5603->module_index);

	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
					   &gc5603->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
					   &gc5603->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
					   &gc5603->len_name);
	dev_info(dev, "Module Information: index = %d, Facing = %s, ModuleName = %s, LensName = %s", gc5603->module_index, gc5603->module_facing, gc5603->module_name, gc5603->len_name);
	if (ret) {
		dev_err(dev,
			"could not get module information!\n");
		return -EINVAL;
	}

	gc5603->xvclk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(gc5603->xvclk)) {
		dev_err(&client->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	gc5603->pwren_gpio = devm_gpiod_get(dev, "pwren", GPIOD_OUT_LOW);
	if (IS_ERR(gc5603->pwdn_gpio))
		dev_warn(dev, "Failed to get pwren-gpios\n");

	gc5603->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc5603->reset_gpio))
		dev_info(dev, "Failed to get reset-gpios, maybe no used\n");

	gc5603->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gc5603->pwdn_gpio))
		dev_warn(dev, "Failed to get power-gpios\n");



	ret = gc5603_parse_of(gc5603);
	if (ret != 0)
		return -EINVAL;

	gc5603->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(gc5603->pinctrl)) {
		gc5603->pins_default =
			pinctrl_lookup_state(gc5603->pinctrl,
						 OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(gc5603->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		gc5603->pins_sleep =
			pinctrl_lookup_state(gc5603->pinctrl,
						 OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(gc5603->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = gc5603_configure_regulators(gc5603);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}
	mutex_init(&gc5603->mutex);

	sd = &gc5603->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc5603_subdev_ops);
	ret = gc5603_initialize_controls(gc5603);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc5603_power_on(gc5603);
	if (ret)
		goto err_free_handler;

	usleep_range(3000, 4000);
    ret = gc5603_check_sensor_id(gc5603, client);
    if (ret)
	goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gc5603_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc5603->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc5603->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc5603->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc5603->module_index, facing,
		 GC5603_NAME, dev_name(sd->dev));

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
	__gc5603_power_off(gc5603);
err_free_handler:
	v4l2_ctrl_handler_free(&gc5603->ctrl_handler);

err_destroy_mutex:
	mutex_destroy(&gc5603->mutex);
	return ret;
}

static int gc5603_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc5603 *gc5603 = to_gc5603(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc5603->ctrl_handler);
	mutex_destroy(&gc5603->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc5603_power_off(gc5603);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc5603_of_match[] = {
	{ .compatible = "galaxycore,gc5603" },
	{},
};
MODULE_DEVICE_TABLE(of, gc5603_of_match);
#endif
static const struct i2c_device_id gc5603_match_id[] = {
	{ "galaxycore, gc5603", 0 },
	{ },
};



static struct i2c_driver gc5603_i2c_driver = {
	.driver = {
		.name = GC5603_NAME,
		.pm = &gc5603_pm_ops,
		.of_match_table = of_match_ptr(gc5603_of_match),
	},
	.probe      = &gc5603_probe,
	.remove     = &gc5603_remove,
	.id_table   = gc5603_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&gc5603_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gc5603_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("GC5603 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");


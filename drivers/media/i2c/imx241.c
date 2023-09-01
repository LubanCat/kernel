// SPDX-License-Identifier: GPL-2.0
/*
 * imx241 camera driver
 *
 * Copyright (C) 2022 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 * V0.0X01.0X01 fix compile errors.
 * V0.0X01.0X02 add 4lane mode support.
 *
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
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>
#include <linux/of_graph.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x02)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define IMX241_LINK_FREQ_400MHZ		400000000U
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define IMX241_PIXEL_RATE		(IMX241_LINK_FREQ_400MHZ * 2LL * 2LL / 10LL)
#define IMX241_XVCLK_FREQ		24000000

#define CHIP_ID				0x40
#define IMX241_REG_CHIP_ID		0x3032

#define IMX241_REG_CTRL_MODE		0x0100
#define IMX241_MODE_SW_STANDBY		0x0
#define IMX241_MODE_STREAMING		BIT(0)

#define IMX241_REG_EXPOSURE		0x0202
#define	IMX241_EXPOSURE_MIN		1
#define	IMX241_EXPOSURE_STEP		1
#define IMX241_VTS_MAX			0xffff

#define IMX241_REG_GAIN			0x0205
#define IMX241_GAIN_MIN			0x100
#define IMX241_GAIN_MAX			0x1000
#define IMX241_GAIN_STEP		0x1
#define IMX241_GAIN_DEFAULT		(8 * IMX241_GAIN_MIN)

#define IMX241_REG_TEST_PATTERN		0x0600
#define	IMX241_TEST_PATTERN_ENABLE	0x100
#define	IMX241_TEST_PATTERN_DISABLE	0x0

#define IMX241_REG_VTS			0x0340

#define REG_NULL			0xFFFF

#define IMX241_REG_VALUE_08BIT		1
#define IMX241_REG_VALUE_16BIT		2
#define IMX241_REG_VALUE_24BIT		3

#define IMX241_BITS_PER_SAMPLE		10

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define IMX241_NAME			"imx241"
#define IMX241_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SRGGB10_1X10

static const char * const imx241_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define IMX241_NUM_SUPPLIES ARRAY_SIZE(imx241_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct imx241_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 link_freq_idx;
	u32 bpp;
	const struct regval *reg_list;
};

struct imx241 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*power_gpio;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[IMX241_NUM_SUPPLIES];

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
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	struct v4l2_fwnode_endpoint bus_cfg;
	bool			streaming;
	bool			power_on;
	const struct imx241_mode *support_modes;
	const struct imx241_mode *cur_mode;
	u32			module_index;
	u32			cfg_num;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	struct rkmodule_inf	module_inf;
	struct rkmodule_awb_cfg	awb_cfg;
	struct rkmodule_lsc_cfg	lsc_cfg;
};

#define to_imx241(sd) container_of(sd, struct imx241, subdev)

static const struct regval imx241_2592x1944_30fps_regs_2lane[] = {
	{0x0101, 0x00},
	{0x303C, 0x4B}, // 24M MCLK
	{0x303D, 0x00},
	{0x3041, 0xD7},
	{0x30E0, 0x00},
	{0x30E1, 0x00},
	{0x30F6, 0x00}, //Embedded Data Line output control
	{0x34CE, 0xFF},

	// Mode Setting
	{0x0340, 0x08},
	{0x0341, 0xF0}, // 2288, lines  vts
	{0x0342, 0x05},
	{0x0343, 0xB0}, // 1456, line length / 2,   hts / 2
	{0x0344, 0x00}, // x_addr_start[12:8]
	{0x0345, 0x00}, // x_addr_start[7:0]
	{0x0346, 0x00}, // y_addr_start[11:8]
	{0x0347, 0x00}, // y_addr_start[7:0]


	{0x0348, 0x0A}, // x_addr_end[12:8]
	{0x0349, 0x1F}, // x_addr_end[7:0], 2591
	{0x034A, 0x07}, // y_addr_end[11:8]
	{0x034B, 0x97}, // y_addr_end[7:0], 1943

	/* binning setting */
	{0x0381, 0x01}, // x_even_inc[3:0]
	{0x0383, 0x01}, // x_odd_inc[3:0]
	{0x0385, 0x01}, // y_even_inc[3:0]
	{0x0387, 0x01}, // y_odd_inc[3:0]
	{0x3048, 0x20},
	{0x30D5, 0x00},
	{0x3165, 0x20},
	{0x30D1, 0x00},
	{0x30D0, 0x2A},
	{0x3102, 0x13},
	{0x3103, 0x47},
	{0x3049, 0x01},
	{0x304D, 0x02},
	{0x304C, 0xD7},
	{0x0112, 0x0A},
	{0x0113, 0x0A},

	{0x034C, 0x0A}, // x_output_size[12:8]
	{0x034D, 0x20}, // x_output_size[7:0], 2592
	{0x034E, 0x07}, // y_output_size[11:8]
	{0x034F, 0x98}, // y_output_size[7:0], 1944

	/*
	 * Frame Rate [frame/s] = Logic Clock/(frame_length_lines x line_length_pck)
	 * Logic Clock
	 *  = PLCK (PLL output clock frequency) x Logic clock division ratio
	 *  = (INCK frequency x PreDivider ratio setting x PLL multiplier setting ) x
	 *    Divider2 frequency division ratio x Divider4 frequency division ratio
	 *
	 * PLCK = 24M x (1/3) x 125 = 1000M
	 * Logic Clock = 1000M x (1/5) x (1/2) = 100M
	 * fps = 100M / 2288 / 1456 = 30fps
	 */
	{0x0305, 0x03}, // Pre Dividers setting, 1/3
	{0x0307, 0x7D}, // PLL multiplication setting, 125

	{0x3037, 0x0A}, // Divider2: Pre divider setting, 1/5
	{0x3038, 0x01}, // Divider4: Logic Clock divider setting, 1/2
	{0x303E, 0x01}, // Divider3: CK_PIXEL divider setting, 1/2
	{0x30A2, 0x0E},
	{0x30A5, 0x60},
	{0x30A7, 0x40},
	{0x31AA, 0x02},

	{0x3301, 0x00},
	{0x3318, 0x60}, // MIPI Global Timing


	{0x0202, 0x08}, // coarse_integration_time
	{0x0203, 0xEB},

	{0x0204, 0x00},
	{0x0205, 0x00}, // analogue_gain_code_global

	{0x020E, 0x01}, // DIG_GAIN_GR [15:8]
	{0x020F, 0x00}, // DIG_GAIN_GR [7:0]

	{0x0210, 0x01}, // DIG_GAIN_R [15:8]
	{0x0211, 0x00}, // DIG_GAIN_R [7:0]

	{0x0212, 0x01}, // DIG_GAIN_B [15:8]
	{0x0213, 0x00}, // DIG_GAIN_B [15:8]

	{0x0215, 0x00}, // DIG_GAIN_B [7:0]

	{0x0100, 0x00},

	{0xFFFF, 0xFF},
};

static const struct imx241_mode supported_modes_2lane[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0630,
		.hts_def = 0xBE0,
		.vts_def = 0x08F0,
		.bpp = 10,
		.bus_fmt = IMX241_MEDIA_BUS_FMT,
		.reg_list = imx241_2592x1944_30fps_regs_2lane,
		.link_freq_idx = 0,
	},
};

static const s64 link_freq_items[] = {
	IMX241_LINK_FREQ_400MHZ,
};

static const char * const imx241_test_pattern_menu[] = {
	"Disabled",
	"Solid Clolor",
	"100% Color Bar",
	"Fade to grey Color Bar",
	"PN9"
};

/* Write registers up to 4 at a time */
static int imx241_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	dev_dbg(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);

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

static int imx241_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = imx241_write_reg(client, regs[i].addr,
				       IMX241_REG_VALUE_08BIT,
				       regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int imx241_read_reg(struct i2c_client *client, u16 reg,
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
	msgs[0].addr  = client->addr;
	msgs[0].flags = 0;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr  = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len   = len;
	msgs[1].buf   = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int imx241_get_reso_dist(const struct imx241_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct imx241_mode *
imx241_find_best_fit(struct imx241 *imx241, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < imx241->cfg_num; i++) {
		dist = imx241_get_reso_dist(&imx241->support_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &imx241->support_modes[cur_best_fit];
}

static int imx241_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx241 *imx241 = to_imx241(sd);
	const struct imx241_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;
	u32 lane_num = imx241->bus_cfg.bus.mipi_csi2.num_data_lanes;

	mutex_lock(&imx241->mutex);

	mode = imx241_find_best_fit(imx241, fmt);
	fmt->format.code = IMX241_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx241->mutex);
		return -ENOTTY;
#endif
	} else {
		imx241->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx241->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx241->vblank, vblank_def,
					 IMX241_VTS_MAX - mode->height,
					 1, vblank_def);
		pixel_rate = (u32)link_freq_items[mode->link_freq_idx] / mode->bpp * 2 * lane_num;

		__v4l2_ctrl_s_ctrl_int64(imx241->pixel_rate,
					 pixel_rate);
		__v4l2_ctrl_s_ctrl(imx241->link_freq,
				   mode->link_freq_idx);
	}

	mutex_unlock(&imx241->mutex);

	return 0;
}

static int imx241_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx241 *imx241 = to_imx241(sd);
	const struct imx241_mode *mode = imx241->cur_mode;

	mutex_lock(&imx241->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx241->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = IMX241_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&imx241->mutex);

	return 0;
}

static int imx241_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = IMX241_MEDIA_BUS_FMT;

	return 0;
}

static int imx241_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx241 *imx241 = to_imx241(sd);

	if (fse->index >= imx241->cfg_num)
		return -EINVAL;

	if (fse->code != IMX241_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = imx241->support_modes[fse->index].width;
	fse->max_width  = imx241->support_modes[fse->index].width;
	fse->max_height = imx241->support_modes[fse->index].height;
	fse->min_height = imx241->support_modes[fse->index].height;

	return 0;
}

static int imx241_enable_test_pattern(struct imx241 *imx241, u32 pattern)
{
	if (pattern == 0)
		return 0;

	dev_err(&imx241->client->dev, "test pattern %u not implement yet.\n", pattern);

	return -EINVAL;
}

static int imx241_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx241 *imx241 = to_imx241(sd);
	const struct imx241_mode *mode = imx241->cur_mode;

	mutex_lock(&imx241->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&imx241->mutex);

	return 0;
}

static void imx241_get_module_inf(struct imx241 *imx241,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, IMX241_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, imx241->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, imx241->len_name, sizeof(inf->base.lens));
}

static void imx241_set_awb_cfg(struct imx241 *imx241,
			       struct rkmodule_awb_cfg *cfg)
{
	mutex_lock(&imx241->mutex);
	memcpy(&imx241->awb_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&imx241->mutex);
}

static void imx241_set_lsc_cfg(struct imx241 *imx241,
			       struct rkmodule_lsc_cfg *cfg)
{
	mutex_lock(&imx241->mutex);
	memcpy(&imx241->lsc_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&imx241->mutex);
}

static long imx241_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx241 *imx241 = to_imx241(sd);
	struct rkmodule_hdr_cfg *hdr;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx241_get_module_inf(imx241, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_AWB_CFG:
		imx241_set_awb_cfg(imx241, (struct rkmodule_awb_cfg *)arg);
		break;
	case RKMODULE_LSC_CFG:
		imx241_set_lsc_cfg(imx241, (struct rkmodule_lsc_cfg *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);

		if (stream)
			ret = imx241_write_reg(imx241->client,
				 IMX241_REG_CTRL_MODE,
				 IMX241_REG_VALUE_08BIT,
				 IMX241_MODE_STREAMING);
		else
			ret = imx241_write_reg(imx241->client,
				 IMX241_REG_CTRL_MODE,
				 IMX241_REG_VALUE_08BIT,
				 IMX241_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = NO_HDR;
		break;
	case RKMODULE_SET_HDR_CFG:
		ret = 0;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx241_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_lsc_cfg *lsc_cfg;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx241_ioctl(sd, cmd, inf);
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
			ret = imx241_ioctl(sd, cmd, cfg);
		else
			ret = -EFAULT;
		kfree(cfg);
		break;
	case RKMODULE_LSC_CFG:
		lsc_cfg = kzalloc(sizeof(*lsc_cfg), GFP_KERNEL);
		if (!lsc_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(lsc_cfg, up, sizeof(*lsc_cfg));
		if (!ret)
			ret = imx241_ioctl(sd, cmd, lsc_cfg);
		else
			ret = -EFAULT;
		kfree(lsc_cfg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = imx241_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __imx241_start_stream(struct imx241 *imx241)
{
	int ret;

	ret = imx241_write_array(imx241->client, imx241->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&imx241->mutex);
	ret = v4l2_ctrl_handler_setup(&imx241->ctrl_handler);
	mutex_lock(&imx241->mutex);
	if (ret)
		return ret;

	return imx241_write_reg(imx241->client,
				IMX241_REG_CTRL_MODE,
				IMX241_REG_VALUE_08BIT,
				IMX241_MODE_STREAMING);
}

static int __imx241_stop_stream(struct imx241 *imx241)
{
	return imx241_write_reg(imx241->client,
				IMX241_REG_CTRL_MODE,
				IMX241_REG_VALUE_08BIT,
				IMX241_MODE_SW_STANDBY);
}

static int imx241_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx241 *imx241 = to_imx241(sd);
	struct i2c_client *client = imx241->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				imx241->cur_mode->width,
				imx241->cur_mode->height,
		DIV_ROUND_CLOSEST(imx241->cur_mode->max_fps.denominator,
				  imx241->cur_mode->max_fps.numerator));


	mutex_lock(&imx241->mutex);
	on = !!on;
	if (on == imx241->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __imx241_start_stream(imx241);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__imx241_stop_stream(imx241);
		pm_runtime_put(&client->dev);
	}

	imx241->streaming = on;

unlock_and_return:
	mutex_unlock(&imx241->mutex);

	return ret;
}

static int imx241_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx241 *imx241 = to_imx241(sd);
	struct i2c_client *client = imx241->client;
	int ret = 0;

	mutex_lock(&imx241->mutex);

	/* If the power state is not modified - no work to do. */
	if (imx241->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		imx241->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		imx241->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx241->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 imx241_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, IMX241_XVCLK_FREQ / 1000 / 1000);
}

static int __imx241_power_on(struct imx241 *imx241)
{
	int ret;
	u32 delay_us;
	struct device *dev = &imx241->client->dev;

	if (!IS_ERR(imx241->power_gpio))
		gpiod_set_value_cansleep(imx241->power_gpio, 1);

	usleep_range(1000, 2000);

	if (!IS_ERR_OR_NULL(imx241->pins_default)) {
		ret = pinctrl_select_state(imx241->pinctrl,
					   imx241->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(imx241->xvclk, IMX241_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");

	if (clk_get_rate(imx241->xvclk) != IMX241_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

	ret = clk_prepare_enable(imx241->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(imx241->reset_gpio))
		gpiod_set_value_cansleep(imx241->reset_gpio, 0);

	ret = regulator_bulk_enable(IMX241_NUM_SUPPLIES, imx241->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(imx241->reset_gpio))
		gpiod_set_value_cansleep(imx241->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(imx241->pwdn_gpio))
		gpiod_set_value_cansleep(imx241->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = imx241_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(imx241->xvclk);

	return ret;
}

static void __imx241_power_off(struct imx241 *imx241)
{
	int ret;
	struct device *dev = &imx241->client->dev;

	if (!IS_ERR(imx241->pwdn_gpio))
		gpiod_set_value_cansleep(imx241->pwdn_gpio, 0);
	clk_disable_unprepare(imx241->xvclk);
	if (!IS_ERR(imx241->reset_gpio))
		gpiod_set_value_cansleep(imx241->reset_gpio, 0);

	if (!IS_ERR_OR_NULL(imx241->pins_sleep)) {
		ret = pinctrl_select_state(imx241->pinctrl,
					   imx241->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(imx241->power_gpio))
		gpiod_set_value_cansleep(imx241->power_gpio, 0);

	regulator_bulk_disable(IMX241_NUM_SUPPLIES, imx241->supplies);
}

static int imx241_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx241 *imx241 = to_imx241(sd);

	return __imx241_power_on(imx241);
}

static int imx241_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx241 *imx241 = to_imx241(sd);

	__imx241_power_off(imx241);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx241_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx241 *imx241 = to_imx241(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct imx241_mode *def_mode = &imx241->support_modes[0];

	mutex_lock(&imx241->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	//try_fmt->code = def_mode->bus_fmt;
	try_fmt->code = IMX241_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx241->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int imx241_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx241 *imx241 = to_imx241(sd);

	if (fie->index >= imx241->cfg_num)
		return -EINVAL;

	fie->code = imx241->support_modes[fie->index].bus_fmt;
	fie->width = imx241->support_modes[fie->index].width;
	fie->height = imx241->support_modes[fie->index].height;
	fie->interval = imx241->support_modes[fie->index].max_fps;

	return 0;
}

static int imx241_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct imx241 *imx241 = to_imx241(sd);
	u32 lane_num = imx241->bus_cfg.bus.mipi_csi2.num_data_lanes;
	u32 val = 0;

	val = 1 << (lane_num - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static const struct dev_pm_ops imx241_pm_ops = {
	SET_RUNTIME_PM_OPS(imx241_runtime_suspend,
			   imx241_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx241_internal_ops = {
	.open = imx241_open,
};
#endif

static const struct v4l2_subdev_core_ops imx241_core_ops = {
	.s_power = imx241_s_power,
	.ioctl = imx241_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx241_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx241_video_ops = {
	.s_stream = imx241_s_stream,
	.g_frame_interval = imx241_g_frame_interval,
	.g_mbus_config = imx241_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops imx241_pad_ops = {
	.enum_mbus_code = imx241_enum_mbus_code,
	.enum_frame_size = imx241_enum_frame_sizes,
	.enum_frame_interval = imx241_enum_frame_interval,
	.get_fmt = imx241_get_fmt,
	.set_fmt = imx241_set_fmt,
};

static const struct v4l2_subdev_ops imx241_subdev_ops = {
	.core	= &imx241_core_ops,
	.video	= &imx241_video_ops,
	.pad	= &imx241_pad_ops,
};

static int imx241_set_gain_reg(struct imx241 *imx241, u32 a_gain)
{
	int ret = 0;
	u32 gain_reg = 0;

	gain_reg = (256 - (256 * 256 / a_gain));
	if (gain_reg > 240)
		gain_reg = 240;

	ret |= imx241_write_reg(imx241->client,
		IMX241_REG_GAIN,
		IMX241_REG_VALUE_08BIT,
		(gain_reg & 0xff));
	return ret;
}

static int imx241_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx241 *imx241 = container_of(ctrl->handler,
					     struct imx241, ctrl_handler);
	struct i2c_client *client = imx241->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx241->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(imx241->exposure,
					 imx241->exposure->minimum, max,
					 imx241->exposure->step,
					 imx241->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = imx241_write_reg(imx241->client,
			IMX241_REG_EXPOSURE,
			IMX241_REG_VALUE_16BIT,
			ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx241_set_gain_reg(imx241, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx241_write_reg(imx241->client,
			IMX241_REG_VTS,
			IMX241_REG_VALUE_16BIT,
			ctrl->val + imx241->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx241_enable_test_pattern(imx241, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx241_ctrl_ops = {
	.s_ctrl = imx241_set_ctrl,
};

static int imx241_initialize_controls(struct imx241 *imx241)
{
	const struct imx241_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_pixel_rate = 0;
	u32 lane_num = imx241->bus_cfg.bus.mipi_csi2.num_data_lanes;

	handler = &imx241->ctrl_handler;
	mode = imx241->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &imx241->mutex;

	imx241->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			V4L2_CID_LINK_FREQ,
			1, 0, link_freq_items);

	dst_pixel_rate = (u32)link_freq_items[mode->link_freq_idx] / mode->bpp * 2 * lane_num;

	imx241->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
			V4L2_CID_PIXEL_RATE,
			0, IMX241_PIXEL_RATE,
			1, dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(imx241->link_freq,
			   mode->link_freq_idx);

	h_blank = mode->hts_def - mode->width;
	imx241->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (imx241->hblank)
		imx241->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	imx241->vblank = v4l2_ctrl_new_std(handler, &imx241_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				IMX241_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 5;
	imx241->exposure = v4l2_ctrl_new_std(handler, &imx241_ctrl_ops,
				V4L2_CID_EXPOSURE, IMX241_EXPOSURE_MIN,
				exposure_max, IMX241_EXPOSURE_STEP,
				mode->exp_def);

	imx241->anal_gain = v4l2_ctrl_new_std(handler, &imx241_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, IMX241_GAIN_MIN,
				IMX241_GAIN_MAX, IMX241_GAIN_STEP,
				IMX241_GAIN_DEFAULT);

	imx241->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&imx241_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(imx241_test_pattern_menu) - 1,
				0, 0, imx241_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&imx241->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	imx241->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int imx241_check_sensor_id(struct imx241 *imx241,
				  struct i2c_client *client)
{
	struct device *dev = &imx241->client->dev;
	u32 id = 0;
	int ret;

	ret = imx241_read_reg(client, IMX241_REG_CHIP_ID,
			      IMX241_REG_VALUE_08BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected IMX241 sensor\n");

	return 0;
}

static int imx241_configure_regulators(struct imx241 *imx241)
{
	unsigned int i;

	for (i = 0; i < IMX241_NUM_SUPPLIES; i++)
		imx241->supplies[i].supply = imx241_supply_names[i];

	return devm_regulator_bulk_get(&imx241->client->dev,
				       IMX241_NUM_SUPPLIES,
				       imx241->supplies);
}

static int imx241_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx241 *imx241;
	struct v4l2_subdev *sd;
	struct device_node *endpoint;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	imx241 = devm_kzalloc(dev, sizeof(*imx241), GFP_KERNEL);
	if (!imx241)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx241->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx241->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx241->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx241->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	imx241->client = client;
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
		&imx241->bus_cfg);
	if (ret) {
		dev_err(dev, "Failed to get bus cfg\n");
		return ret;
	}

	imx241->support_modes = supported_modes_2lane;
	imx241->cfg_num       = ARRAY_SIZE(supported_modes_2lane);
	imx241->cur_mode      = &imx241->support_modes[0];

	imx241->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(imx241->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	imx241->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(imx241->power_gpio))
		dev_warn(dev, "Failed to get power-gpios, maybe no use\n");

	imx241->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx241->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	imx241->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(imx241->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = imx241_configure_regulators(imx241);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	imx241->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(imx241->pinctrl)) {
		imx241->pins_default =
			pinctrl_lookup_state(imx241->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(imx241->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		imx241->pins_sleep =
			pinctrl_lookup_state(imx241->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(imx241->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&imx241->mutex);

	sd = &imx241->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx241_subdev_ops);
	ret = imx241_initialize_controls(imx241);
	if (ret)
		goto err_destroy_mutex;

	ret = __imx241_power_on(imx241);
	if (ret)
		goto err_free_handler;

	ret = imx241_check_sensor_id(imx241, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &imx241_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	imx241->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx241->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx241->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx241->module_index, facing,
		 IMX241_NAME, dev_name(sd->dev));
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
	__imx241_power_off(imx241);
err_free_handler:
	v4l2_ctrl_handler_free(&imx241->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx241->mutex);

	return ret;
}

static int imx241_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx241 *imx241 = to_imx241(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx241->ctrl_handler);
	mutex_destroy(&imx241->mutex);

	pm_runtime_disable(&client->dev);

	if (!pm_runtime_status_suspended(&client->dev))
		__imx241_power_off(imx241);

	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx241_of_match[] = {
	{ .compatible = "sony,imx241" },
	{},
};
MODULE_DEVICE_TABLE(of, imx241_of_match);
#endif

static const struct i2c_device_id imx241_match_id[] = {
	{ "sony,imx241", 0 },
	{},
};

static struct i2c_driver imx241_i2c_driver = {
	.driver = {
		.name = IMX241_NAME,
		.pm = &imx241_pm_ops,
		.of_match_table = of_match_ptr(imx241_of_match),
	},
	.probe    = &imx241_probe,
	.remove   = &imx241_remove,
	.id_table = imx241_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&imx241_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx241_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony imx241 sensor driver");
MODULE_LICENSE("GPL");

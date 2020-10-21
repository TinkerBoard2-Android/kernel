/*
 * A V4L2 driver for Asus Sensor(OV5647/IMX219) cameras.
 *
 * Based on Samsung S5K6AAFX SXGA 1/6" 1.3M CMOS Image Sensor driver
 * Copyright (C) 2011 Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Based on OmniVision OV5647 Camera Driver
 * Copyright (C) 2016, Synopsys, Inc.
 *
 * Based on Sony IMX219 Camera Driver
 * Copyright (C) 2014, Andrew Chew <achew@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-camera-module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>

#define SENSOR_NAME "ov5647"

#define REG_NULL 0xffff

#define OV5647_SW_RESET	0x0103
#define OV5647_REG_CHIPID_H	0x300A
#define OV5647_REG_CHIPID_L	0x300B
#define OV5647_REG_GAIN_H	0x350A
#define OV5647_REG_GAIN_L	0x350B
#define OV5647_REG_LINE_H	0x3500
#define OV5647_REG_LINE_M	0x3501
#define OV5647_REG_LINE_L	0x3502

#define OV5647_EXPOSURE_MIN	0x000000
#define OV5647_EXPOSURE_MAX	0x0fffff
#define OV5647_EXPOSURE_STEP	0x01
#define OV5647_EXPOSURE_DEFAULT	0x001000

#define OV5647_ANALOG_GAIN_MIN	0x0000
#define OV5647_ANALOG_GAIN_MAX	0x03ff
#define OV5647_ANALOG_GAIN_STEP	0x01
#define OV5647_ANALOG_GAIN_DEFAULT 0x100

#define OV5647_LINK_FREQ_150MHZ		150000000
static const s64 link_freq_menu_items[] = {
	OV5647_LINK_FREQ_150MHZ
};

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"

#define OV5647_XVCLK_FREQ		24000000

static int isOV5647 = 0;

/* IMX219 supported geometry */
#define IMX219_TABLE_END		0xffff
#define IMX219_ANALOGUE_GAIN_MULTIPLIER	256
#define IMX219_ANALOGUE_GAIN_MIN	(1 * IMX219_ANALOGUE_GAIN_MULTIPLIER)
#define IMX219_ANALOGUE_GAIN_MAX	(11 * IMX219_ANALOGUE_GAIN_MULTIPLIER)
#define IMX219_ANALOGUE_GAIN_DEFAULT	(2 * IMX219_ANALOGUE_GAIN_MULTIPLIER)

/* In dB*256 */
#define IMX219_DIGITAL_GAIN_MIN		256
#define IMX219_DIGITAL_GAIN_MAX		43663
#define IMX219_DIGITAL_GAIN_DEFAULT	256

#define IMX219_DIGITAL_EXPOSURE_MIN	0
#define IMX219_DIGITAL_EXPOSURE_MAX	4095
#define IMX219_DIGITAL_EXPOSURE_DEFAULT	1575

#define IMX219_EXP_LINES_MARGIN	4

#define IMX219_NAME			"imx219"

static const s64 imx219_link_freq_menu_items[] = {
	456000000,
};

struct imx219_reg {
	u16 addr;
	u8 val;
};

struct imx219_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	const struct imx219_reg *reg_list;
};

/* MCLK:24MHz  3280x2464  21.2fps   MIPI LANE2 */
static const struct imx219_reg imx219_init_tab_3280_2464_21fps[] = {
	{0x30EB, 0x05},		/* Access Code for address over 0x3000 */
	{0x30EB, 0x0C},		/* Access Code for address over 0x3000 */
	{0x300A, 0xFF},		/* Access Code for address over 0x3000 */
	{0x300B, 0xFF},		/* Access Code for address over 0x3000 */
	{0x30EB, 0x05},		/* Access Code for address over 0x3000 */
	{0x30EB, 0x09},		/* Access Code for address over 0x3000 */
	{0x0114, 0x01},		/* CSI_LANE_MODE[1:0} */
	{0x0128, 0x00},		/* DPHY_CNTRL */
	{0x012A, 0x18},		/* EXCK_FREQ[15:8] */
	{0x012B, 0x00},		/* EXCK_FREQ[7:0] */
//	{0x015A, 0x01},		/* INTEG TIME[15:8] */
//	{0x015B, 0xF4},		/* INTEG TIME[7:0] */
	{0x0160, 0x09},		/* FRM_LENGTH_A[15:8] */
	{0x0161, 0xD7},		/* FRM_LENGTH_A[7:0] */
	{0x0162, 0x0D},		/* LINE_LENGTH_A[15:8] */
	{0x0163, 0x78},		/* LINE_LENGTH_A[7:0] */
	{0x0260, 0x09},		/* FRM_LENGTH_B[15:8] */
	{0x0261, 0xC4},		/* FRM_LENGTH_B[7:0] */
	{0x0262, 0x0D},		/* LINE_LENGTH_B[15:8] */
	{0x0263, 0x78},		/* LINE_LENGTH_B[7:0] */
	{0x0170, 0x01},		/* X_ODD_INC_A[2:0] */
	{0x0171, 0x01},		/* Y_ODD_INC_A[2:0] */
	{0x0270, 0x01},		/* X_ODD_INC_B[2:0] */
	{0x0271, 0x01},		/* Y_ODD_INC_B[2:0] */
	{0x0174, 0x00},		/* BINNING_MODE_H_A */
	{0x0175, 0x00},		/* BINNING_MODE_V_A */
	{0x0274, 0x00},		/* BINNING_MODE_H_B */
	{0x0275, 0x00},		/* BINNING_MODE_V_B */
	{0x018C, 0x0A},		/* CSI_DATA_FORMAT_A[15:8] */
	{0x018D, 0x0A},		/* CSI_DATA_FORMAT_A[7:0] */
	{0x028C, 0x0A},		/* CSI_DATA_FORMAT_B[15:8] */
	{0x028D, 0x0A},		/* CSI_DATA_FORMAT_B[7:0] */
	{0x0301, 0x05},		/* VTPXCK_DIV */
	{0x0303, 0x01},		/* VTSYCK_DIV */
	{0x0304, 0x03},		/* PREPLLCK_VT_DIV[3:0] */
	{0x0305, 0x03},		/* PREPLLCK_OP_DIV[3:0] */
	{0x0306, 0x00},		/* PLL_VT_MPY[10:8] */
	{0x0307, 0x39},		/* PLL_VT_MPY[7:0] */
	{0x0309, 0x0A},		/* OPPXCK_DIV[4:0] */
	{0x030B, 0x01},		/* OPSYCK_DIV */
	{0x030C, 0x00},		/* PLL_OP_MPY[10:8] */
	{0x030D, 0x72},		/* PLL_OP_MPY[7:0] */
	{0x455E, 0x00},		/* CIS Tuning */
	{0x471E, 0x4B},		/* CIS Tuning */
	{0x4767, 0x0F},		/* CIS Tuning */
	{0x4750, 0x14},		/* CIS Tuning */
	{0x47B4, 0x14},		/* CIS Tuning */
	{IMX219_TABLE_END, 0x00}
};

/* MCLK:24MHz  1920x1080  30fps   MIPI LANE2 */
static const struct imx219_reg imx219_init_tab_1920_1080_30fps[] = {
	{0x30EB, 0x05},
	{0x30EB, 0x0C},
	{0x300A, 0xFF},
	{0x300B, 0xFF},
	{0x30EB, 0x05},
	{0x30EB, 0x09},
	{0x0114, 0x01},
	{0x0128, 0x00},
	{0x012A, 0x18},
	{0x012B, 0x00},
	{0x0160, 0x06},
	{0x0161, 0xE6},
	{0x0162, 0x0D},
	{0x0163, 0x78},
	{0x0164, 0x02},
	{0x0165, 0xA8},
	{0x0166, 0x0A},
	{0x0167, 0x27},
	{0x0168, 0x02},
	{0x0169, 0xB4},
	{0x016A, 0x06},
	{0x016B, 0xEB},
	{0x016C, 0x07},
	{0x016D, 0x80},
	{0x016E, 0x04},
	{0x016F, 0x38},
	{0x0170, 0x01},
	{0x0171, 0x01},
	{0x0174, 0x00},
	{0x0175, 0x00},
	{0x018C, 0x0A},
	{0x018D, 0x0A},
	{0x0301, 0x05},
	{0x0303, 0x01},
	{0x0304, 0x03},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x39},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x030C, 0x00},
	{0x030D, 0x72},
	{0x455E, 0x00},
	{0x471E, 0x4B},
	{0x4767, 0x0F},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47B4, 0x14},
	{IMX219_TABLE_END, 0x00}
};

static const struct imx219_reg start[] = {
	{0x0100, 0x01},		/* mode select streaming on */
	{IMX219_TABLE_END, 0x00}
};

static const struct imx219_reg stop[] = {
	{0x0100, 0x00},		/* mode select streaming off */
	{IMX219_TABLE_END, 0x00}
};

enum {
	TEST_PATTERN_DISABLED,
	TEST_PATTERN_SOLID_BLACK,
	TEST_PATTERN_SOLID_WHITE,
	TEST_PATTERN_SOLID_RED,
	TEST_PATTERN_SOLID_GREEN,
	TEST_PATTERN_SOLID_BLUE,
	TEST_PATTERN_COLOR_BAR,
	TEST_PATTERN_FADE_TO_GREY_COLOR_BAR,
	TEST_PATTERN_PN9,
	TEST_PATTERN_16_SPLIT_COLOR_BAR,
	TEST_PATTERN_16_SPLIT_INVERTED_COLOR_BAR,
	TEST_PATTERN_COLUMN_COUNTER,
	TEST_PATTERN_INVERTED_COLUMN_COUNTER,
	TEST_PATTERN_PN31,
	TEST_PATTERN_MAX
};

static const char *const tp_qmenu[] = {
	"Disabled",
	"Solid Black",
	"Solid White",
	"Solid Red",
	"Solid Green",
	"Solid Blue",
	"Color Bar",
	"Fade to Grey Color Bar",
	"PN9",
	"16 Split Color Bar",
	"16 Split Inverted Color Bar",
	"Column Counter",
	"Inverted Column Counter",
	"PN31",
};

#define SIZEOF_I2C_TRANSBUF 32

struct imx219 {
	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct clk *clk;
	struct v4l2_rect crop_rect;
	int hflip;
	int vflip;
	u8 analogue_gain;
	u16 digital_gain;	/* bits 11:0 */
	u16 exposure_time;
	u16 test_pattern;
	u16 test_pattern_solid_color_r;
	u16 test_pattern_solid_color_gr;
	u16 test_pattern_solid_color_b;
	u16 test_pattern_solid_color_gb;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
	const struct imx219_mode *cur_mode;
	u32 cfg_num;
	u16 cur_vts;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;

	struct gpio_desc        *enable_gpio;
	struct pinctrl			*pinctrl;
	struct pinctrl_state	*pins_default;
};

static const struct imx219_mode imx219_supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.hts_def = 0x0d78 - IMX219_EXP_LINES_MARGIN,
		.vts_def = 0x06E6,
		.reg_list = imx219_init_tab_1920_1080_30fps,
	},
	{
		.width = 3280,
		.height = 2464,
		.max_fps = {
			.numerator = 10000,
			.denominator = 210000,
		},
		.hts_def = 0x0d78 - IMX219_EXP_LINES_MARGIN,
		.vts_def = 0x09d7,
		.reg_list = imx219_init_tab_3280_2464_21fps,
	},
};

static struct imx219 *to_imx219(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct imx219, subdev);
}

static int reg_write(struct i2c_client *client, const u16 addr, const u8 data)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	u8 tx[3];
	int ret;

	msg.addr = client->addr;
	msg.buf = tx;
	msg.len = 3;
	msg.flags = 0;
	tx[0] = addr >> 8;
	tx[1] = addr & 0xff;
	tx[2] = data;
	ret = i2c_transfer(adap, &msg, 1);
	mdelay(2);

	return ret == 1 ? 0 : -EIO;
}

static int reg_read(struct i2c_client *client, const u16 addr)
{
	u8 buf[2] = {addr >> 8, addr & 0xff};
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = buf,
		}, {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_warn(&client->dev, "Reading register %x from %x failed\n",
			 addr, client->addr);
		return ret;
	}

	return buf[0];
}

static int reg_write_table(struct i2c_client *client,
			   const struct imx219_reg table[])
{
	const struct imx219_reg *reg;
	int ret;

	for (reg = table; reg->addr != IMX219_TABLE_END; reg++) {
		ret = reg_write(client, reg->addr, reg->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* V4L2 subdev video operations */
static int imx219_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	u8 reg = 0x00;
	int ret;

	if (!enable)
		return reg_write_table(client, stop);

	ret = reg_write_table(client, priv->cur_mode->reg_list);
	if (ret)
		return ret;

	/* Handle crop */
	ret = reg_write(client, 0x0164, priv->crop_rect.left >> 8);
	ret |= reg_write(client, 0x0165, priv->crop_rect.left & 0xff);
	ret |= reg_write(client, 0x0166, (priv->crop_rect.left + priv->crop_rect.width - 1) >> 8);
	ret |= reg_write(client, 0x0167, (priv->crop_rect.left + priv->crop_rect.width - 1) & 0xff);
	ret |= reg_write(client, 0x0168, priv->crop_rect.top >> 8);
	ret |= reg_write(client, 0x0169, priv->crop_rect.top & 0xff);
	ret |= reg_write(client, 0x016A, (priv->crop_rect.top + priv->crop_rect.height - 1) >> 8);
	ret |= reg_write(client, 0x016B, (priv->crop_rect.top + priv->crop_rect.height - 1) & 0xff);
	ret |= reg_write(client, 0x016C, priv->crop_rect.width >> 8);
	ret |= reg_write(client, 0x016D, priv->crop_rect.width & 0xff);
	ret |= reg_write(client, 0x016E, priv->crop_rect.height >> 8);
	ret |= reg_write(client, 0x016F, priv->crop_rect.height & 0xff);

	if (ret)
		return ret;

	/* Handle flip/mirror */
	if (priv->hflip)
		reg |= 0x1;
	if (priv->vflip)
		reg |= 0x2;

	ret = reg_write(client, 0x0172, reg);
	if (ret)
		return ret;

	/* Handle test pattern */
	if (priv->test_pattern) {
		ret = reg_write(client, 0x0600, priv->test_pattern >> 8);
		ret |= reg_write(client, 0x0601, priv->test_pattern & 0xff);
		ret |= reg_write(client, 0x0602,
				 priv->test_pattern_solid_color_r >> 8);
		ret |= reg_write(client, 0x0603,
				 priv->test_pattern_solid_color_r & 0xff);
		ret |= reg_write(client, 0x0604,
				 priv->test_pattern_solid_color_gr >> 8);
		ret |= reg_write(client, 0x0605,
				 priv->test_pattern_solid_color_gr & 0xff);
		ret |= reg_write(client, 0x0606,
				 priv->test_pattern_solid_color_b >> 8);
		ret |= reg_write(client, 0x0607,
				 priv->test_pattern_solid_color_b & 0xff);
		ret |= reg_write(client, 0x0608,
				 priv->test_pattern_solid_color_gb >> 8);
		ret |= reg_write(client, 0x0609,
				 priv->test_pattern_solid_color_gb & 0xff);
		ret |= reg_write(client, 0x0620, priv->crop_rect.left >> 8);
		ret |= reg_write(client, 0x0621, priv->crop_rect.left & 0xff);
		ret |= reg_write(client, 0x0622, priv->crop_rect.top >> 8);
		ret |= reg_write(client, 0x0623, priv->crop_rect.top & 0xff);
		ret |= reg_write(client, 0x0624, priv->crop_rect.width >> 8);
		ret |= reg_write(client, 0x0625, priv->crop_rect.width & 0xff);
		ret |= reg_write(client, 0x0626, priv->crop_rect.height >> 8);
		ret |= reg_write(client, 0x0627, priv->crop_rect.height & 0xff);
	} else {
		ret = reg_write(client, 0x0600, 0x00);
		ret |= reg_write(client, 0x0601, 0x00);
	}

	priv->cur_vts = priv->cur_mode->vts_def - IMX219_EXP_LINES_MARGIN;
	if (ret)
		return ret;

	return reg_write_table(client, start);
}

/* V4L2 subdev core operations */
static int imx219_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);

	if (on)	{
		dev_dbg(&client->dev, "imx219 power on\n");
		clk_prepare_enable(priv->clk);
	} else if (!on) {
		dev_dbg(&client->dev, "imx219 power off\n");
		clk_disable_unprepare(priv->clk);
	}

	return 0;
}

/* V4L2 ctrl operations */
static int imx219_s_ctrl_test_pattern(struct v4l2_ctrl *ctrl)
{
	struct imx219 *priv =
	    container_of(ctrl->handler, struct imx219, ctrl_handler);

	switch (ctrl->val) {
	case TEST_PATTERN_DISABLED:
		priv->test_pattern = 0x0000;
		break;
	case TEST_PATTERN_SOLID_BLACK:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_SOLID_WHITE:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0fff;
		priv->test_pattern_solid_color_gr = 0x0fff;
		priv->test_pattern_solid_color_b = 0x0fff;
		priv->test_pattern_solid_color_gb = 0x0fff;
		break;
	case TEST_PATTERN_SOLID_RED:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0fff;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_SOLID_GREEN:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0fff;
		priv->test_pattern_solid_color_b = 0x0000;
		priv->test_pattern_solid_color_gb = 0x0fff;
		break;
	case TEST_PATTERN_SOLID_BLUE:
		priv->test_pattern = 0x0001;
		priv->test_pattern_solid_color_r = 0x0000;
		priv->test_pattern_solid_color_gr = 0x0000;
		priv->test_pattern_solid_color_b = 0x0fff;
		priv->test_pattern_solid_color_gb = 0x0000;
		break;
	case TEST_PATTERN_COLOR_BAR:
		priv->test_pattern = 0x0002;
		break;
	case TEST_PATTERN_FADE_TO_GREY_COLOR_BAR:
		priv->test_pattern = 0x0003;
		break;
	case TEST_PATTERN_PN9:
		priv->test_pattern = 0x0004;
		break;
	case TEST_PATTERN_16_SPLIT_COLOR_BAR:
		priv->test_pattern = 0x0005;
		break;
	case TEST_PATTERN_16_SPLIT_INVERTED_COLOR_BAR:
		priv->test_pattern = 0x0006;
		break;
	case TEST_PATTERN_COLUMN_COUNTER:
		priv->test_pattern = 0x0007;
		break;
	case TEST_PATTERN_INVERTED_COLUMN_COUNTER:
		priv->test_pattern = 0x0008;
		break;
	case TEST_PATTERN_PN31:
		priv->test_pattern = 0x0009;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int imx219_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	const struct imx219_mode *mode = priv->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int imx219_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx219 *priv =
	    container_of(ctrl->handler, struct imx219, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
	u8 reg;
	int ret;
	u16 gain = 256;
	u16 a_gain = 256;
	u16 d_gain = 1;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		priv->hflip = ctrl->val;
		break;

	case V4L2_CID_VFLIP:
		priv->vflip = ctrl->val;
		break;

	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_GAIN:
		/*
		 * hal transfer (gain * 256)  to kernel
		 * than divide into analog gain & digital gain in kernel
		 */

		gain = ctrl->val;
		if (gain < 256)
			gain = 256;
		if (gain > 43663)
			gain = 43663;
		if (gain >= 256 && gain <= 2728) {
			a_gain = gain;
			d_gain = 1 * 256;
		} else {
			a_gain = 2728;
			d_gain = (gain * 256) / a_gain;
		}

		/*
		 * Analog gain, reg range[0, 232], gain value[1, 10.66]
		 * reg = 256 - 256 / again
		 * a_gain here is 256 multify
		 * so the reg = 256 - 256 * 256 / a_gain
		 */
		priv->analogue_gain = (256 - (256 * 256) / a_gain);
		if (a_gain < 256)
			priv->analogue_gain = 0;
		if (priv->analogue_gain > 232)
			priv->analogue_gain = 232;

		/*
		 * Digital gain, reg range[256, 4095], gain rage[1, 16]
		 * reg = dgain * 256
		 */
		priv->digital_gain = d_gain;
		if (priv->digital_gain < 256)
			priv->digital_gain = 256;
		if (priv->digital_gain > 4095)
			priv->digital_gain = 4095;

		/*
		 * for bank A and bank B switch
		 * exposure time , gain, vts must change at the same time
		 * so the exposure & gain can reflect at the same frame
		 */

		ret = reg_write(client, 0x0157, priv->analogue_gain);
		ret |= reg_write(client, 0x0158, priv->digital_gain >> 8);
		ret |= reg_write(client, 0x0159, priv->digital_gain & 0xff);

		return ret;

	case V4L2_CID_EXPOSURE:
		priv->exposure_time = ctrl->val;

		ret = reg_write(client, 0x015a, priv->exposure_time >> 8);
		ret |= reg_write(client, 0x015b, priv->exposure_time & 0xff);
		return ret;

	case V4L2_CID_TEST_PATTERN:
		return imx219_s_ctrl_test_pattern(ctrl);

	case V4L2_CID_VBLANK:
		if (ctrl->val < priv->cur_mode->vts_def)
			ctrl->val = priv->cur_mode->vts_def;
		if ((ctrl->val - IMX219_EXP_LINES_MARGIN) != priv->cur_vts)
			priv->cur_vts = ctrl->val - IMX219_EXP_LINES_MARGIN;
		ret = reg_write(client, 0x0160, ((priv->cur_vts >> 8) & 0xff));
		ret |= reg_write(client, 0x0161, (priv->cur_vts & 0xff));
		return ret;

	default:
		return -EINVAL;
	}
	/* If enabled, apply settings immediately */
	reg = reg_read(client, 0x0100);
	if ((reg & 0x1f) == 0x01)
		imx219_s_stream(&priv->subdev, 1);

	return 0;
}

static int imx219_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int imx219_get_reso_dist(const struct imx219_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct imx219_mode *imx219_find_best_fit(
					struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(imx219_supported_modes); i++) {
		dist = imx219_get_reso_dist(&imx219_supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &imx219_supported_modes[cur_best_fit];
}

static int imx219_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	const struct imx219_mode *mode;
	s64 h_blank, v_blank, pixel_rate;
	u32 fps = 0;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	mode = imx219_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	priv->cur_mode = mode;
	h_blank = mode->hts_def - mode->width;
	__v4l2_ctrl_modify_range(priv->hblank, h_blank,
					h_blank, 1, h_blank);
	v_blank = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(priv->vblank, v_blank,
					v_blank,
					1, v_blank);
	fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
		mode->max_fps.numerator);
	pixel_rate = mode->vts_def * mode->hts_def * fps;
	__v4l2_ctrl_modify_range(priv->pixel_rate, pixel_rate,
					pixel_rate, 1, pixel_rate);

	/* reset crop window */
	priv->crop_rect.left = 1640 - (mode->width / 2);
	if (priv->crop_rect.left < 0)
		priv->crop_rect.left = 0;
	priv->crop_rect.top = 1232 - (mode->height / 2);
	if (priv->crop_rect.top < 0)
		priv->crop_rect.top = 0;
	priv->crop_rect.width = mode->width;
	priv->crop_rect.height = mode->height;

	return 0;
}

static int imx219_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	const struct imx219_mode *mode = priv->cur_mode;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->format.field = V4L2_FIELD_NONE;

	return 0;
}

static void imx219_get_module_inf(struct imx219 *imx219,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX219_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx219->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx219->len_name, sizeof(inf->base.lens));
}

static long imx219_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *imx219 = to_imx219(client);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx219_get_module_inf(imx219, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx219_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx219_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
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
			ret = imx219_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int imx219_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);

	if (fie->index >= priv->cfg_num)
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fie->width = imx219_supported_modes[fie->index].width;
	fie->height = imx219_supported_modes[fie->index].height;
	fie->interval = imx219_supported_modes[fie->index].max_fps;
	return 0;
}

static int imx219_enum_frame_sizes(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);

	if (fse->index >= priv->cfg_num)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width  = imx219_supported_modes[fse->index].width;
	fse->max_width  = imx219_supported_modes[fse->index].width;
	fse->max_height = imx219_supported_modes[fse->index].height;
	fse->min_height = imx219_supported_modes[fse->index].height;

	return 0;
}

/* Various V4L2 operations tables */
static struct v4l2_subdev_video_ops imx219_subdev_video_ops = {
	.s_stream = imx219_s_stream,
	.g_frame_interval = imx219_g_frame_interval,
};

static struct v4l2_subdev_core_ops imx219_subdev_core_ops = {
	.s_power = imx219_s_power,
	.ioctl = imx219_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx219_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_pad_ops imx219_subdev_pad_ops = {
	.enum_mbus_code = imx219_enum_mbus_code,
	.enum_frame_interval = imx219_enum_frame_interval,
	.enum_frame_size = imx219_enum_frame_sizes,
	.set_fmt = imx219_set_fmt,
	.get_fmt = imx219_get_fmt,
};

static struct v4l2_subdev_ops imx219_subdev_ops = {
	.core = &imx219_subdev_core_ops,
	.video = &imx219_subdev_video_ops,
	.pad = &imx219_subdev_pad_ops,
};

static const struct v4l2_ctrl_ops imx219_ctrl_ops = {
	.s_ctrl = imx219_s_ctrl,
};

static int imx219_ctrls_init(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx219 *priv = to_imx219(client);
	const struct imx219_mode *mode = priv->cur_mode;
	s64 pixel_rate, h_blank, v_blank;
	int ret;
	u32 fps = 0;

	v4l2_ctrl_handler_init(&priv->ctrl_handler, 10);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	/* exposure */
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN,
			  IMX219_ANALOGUE_GAIN_MIN,
			  IMX219_ANALOGUE_GAIN_MAX,
			  1, IMX219_ANALOGUE_GAIN_DEFAULT);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_GAIN,
			  IMX219_DIGITAL_GAIN_MIN,
			  IMX219_DIGITAL_GAIN_MAX, 1,
			  IMX219_DIGITAL_GAIN_DEFAULT);
	v4l2_ctrl_new_std(&priv->ctrl_handler, &imx219_ctrl_ops,
			  V4L2_CID_EXPOSURE,
			  IMX219_DIGITAL_EXPOSURE_MIN,
			  IMX219_DIGITAL_EXPOSURE_MAX, 1,
			  IMX219_DIGITAL_EXPOSURE_DEFAULT);

	/* blank */
	h_blank = mode->hts_def - mode->width;
	priv->hblank = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_HBLANK,
			  h_blank, h_blank, 1, h_blank);
	v_blank = mode->vts_def - mode->height;
	priv->vblank = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_VBLANK,
			  v_blank, v_blank, 1, v_blank);

	/* freq */
	v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL, V4L2_CID_LINK_FREQ,
			       0, 0, imx219_link_freq_menu_items);
	fps = DIV_ROUND_CLOSEST(mode->max_fps.denominator,
		mode->max_fps.numerator);
	pixel_rate = mode->vts_def * mode->hts_def * fps;
	priv->pixel_rate = v4l2_ctrl_new_std(&priv->ctrl_handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, pixel_rate, 1, pixel_rate);

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &imx219_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(tp_qmenu) - 1, 0, 0, tp_qmenu);

	priv->subdev.ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",
			priv->ctrl_handler.error);
		ret = priv->ctrl_handler.error;
		goto error;
	}

	ret = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (ret < 0) {
		dev_err(&client->dev, "Error %d setting default controls\n",
			ret);
		goto error;
	}

	return 0;
error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return ret;
}

struct regval_list {
	u16 addr;
	u8 data;
};

struct ov5647_state {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct mutex lock;
	struct v4l2_mbus_framefmt format;
	unsigned int width;
	unsigned int height;
	int power_count;
	struct clk *xclk;

	struct gpio_desc	*enable_gpio;

	struct pinctrl			*pinctrl;
	struct pinctrl_state	*pins_default;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *anal_gain;

	const struct ov5647_mode *cur_mode;

	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
};

struct ov5647_mode {
	u32 width;
	u32 height;
	u32 max_fps;
	u32 hts_def;
	u32 vts_def;
	struct regval_list *reg_list;
};

static inline struct ov5647_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5647_state, sd);
}

static struct regval_list sensor_oe_disable_regs[] = {
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{REG_NULL, 0x00}
};

static struct regval_list sensor_oe_enable_regs[] = {
	{0x3000, 0x0f},
	{0x3001, 0xff},
	{0x3002, 0xe4},
	{REG_NULL, 0x00}
};

static struct regval_list ov5647_common_regs[] = {
	/* upstream */
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x370c, 0x03},
	{0x5000, 0x06},
	{0x5003, 0x08},
	{0x5a00, 0x08},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x301d, 0xf0},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3c01, 0x80},
	{0x3b07, 0x0c},
	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	{0x3634, 0x44},
	{0x3620, 0x64},
	{0x3621, 0xe0},
	{0x3600, 0x37},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x3731, 0x02},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3f05, 0x02},
	{0x3f06, 0x10},
	{0x3f01, 0x0a},
	{0x3a0f, 0x58},
	{0x3a10, 0x50},
	{0x3a1b, 0x58},
	{0x3a1e, 0x50},
	{0x3a11, 0x60},
	{0x3a1f, 0x28},
	{0x4001, 0x02},
	{0x4000, 0x09},
#if 1 //0: use auto AE/AWB control from sensor
	{0x3503, 0x03},		/* manual,0xAE */
	{0x3500, 0x00},
	{0x3501, 0x6f},
	{0x3502, 0x00},
	{0x350a, 0x00},
	{0x350b, 0x6f},
	{0x5001, 0x01},		/* manual,0xAWB */
	{0x5180, 0x08},
	{0x5186, 0x04},
	{0x5187, 0x00},
	{0x5188, 0x04},
	{0x5189, 0x00},
	{0x518a, 0x04},
	{0x518b, 0x00},
	{0x5000, 0x00},		/* lenc WBC on */
#endif
	{0x3011, 0x62},
	/* mipi */
	{0x3016, 0x08},
	{0x3017, 0xe0},
	{0x3018, 0x44},
	{0x3034, 0x08},
	{0x3106, 0xf5},
	{REG_NULL, 0x00}
};

static struct regval_list ov5647_1296x972[] = {
	{0x0100, 0x00},
	{0x3035, 0x21},		/* PLL */
	{0x3036, 0x60},		/* PLL */
	{0x303c, 0x11},		/* PLL */
	{0x3821, 0x07},		/* ISP mirror on, Sensor mirror on, H bin on */
	{0x3820, 0x41},		/* ISP flip off, Sensor flip off, V bin on */
	{0x3612, 0x59},		/* analog control */
	{0x3618, 0x00},		/* analog control */
	{0x380c, 0x07},		/* HTS = 1896 */
	{0x380d, 0x68},		/* HTS */
	{0x380e, 0x06},		/* VTS = 1757 */
	{0x380f, 0xdd},		/* VTS */
	{0x3814, 0x31},		/* X INC */
	{0x3815, 0x31},		/* X INC */
	{0x3708, 0x64},		/* analog control */
	{0x3709, 0x52},		/* analog control */
	{0x3808, 0x05},		/* DVPHO = 1296 */
	{0x3809, 0x10},		/* DVPHO */
	{0x380a, 0x03},		/* DVPVO = 984 */
	{0x380b, 0xcc},		/* DVPVO */
	{0x3800, 0x00},		/* X Start */
	{0x3801, 0x08},		/* X Start */
	{0x3802, 0x00},		/* Y Start */
	{0x3803, 0x02},		/* Y Start */
	{0x3804, 0x0a},		/* X End */
	{0x3805, 0x37},		/* X End */
	{0x3806, 0x07},		/* Y End */
	{0x3807, 0xa1},		/* Y End */
	/* banding filter */
	{0x3a08, 0x01},		/* B50 */
	{0x3a09, 0x27},		/* B50 */
	{0x3a0a, 0x00},		/* B60 */
	{0x3a0b, 0xf6},		/* B60 */
	{0x3a0d, 0x04},		/* B60 max */
	{0x3a0e, 0x03},		/* B50 max */
	{0x4004, 0x02},		/* black line number */
	{0x4837, 0x24},		/* MIPI pclk period */
	{0x0100, 0x01},
	{REG_NULL, 0x00}
};

static struct regval_list ov5647_2592x1944[] = {
	{0x0100, 0x00},
	{0x3035, 0x21},
	{0x3036, 0x60},
	{0x303c, 0x11},
	{0x3612, 0x5b},
	{0x3618, 0x04},
	{0x380c, 0x0a},
	{0x380d, 0x8c},
	{0x380e, 0x09},
	{0x380f, 0xA4},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3708, 0x64},
	{0x3709, 0x12},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x07},
	{0x380b, 0x98},
	{0x3800, 0x00},
	{0x3801, 0x0c},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x33},
	{0x3806, 0x07},
	{0x3807, 0xa3},
	{0x3a08, 0x01},
	{0x3a09, 0x28},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0d, 0x08},
	{0x3a0e, 0x06},
	{0x4004, 0x04},
	{0x4837, 0x19},
	{0x0100, 0x01},
	{REG_NULL, 0x00}
};

static const struct ov5647_mode supported_modes[] = {
	{
	 .width = 1296,
	 .height = 972,
	 .max_fps = 30,
	 .hts_def = 0x0768,
	 .vts_def = 0x06dd,
	 .reg_list = ov5647_1296x972,
	 },
	{
	 .width = 2592,
	 .height = 1944,
	 .max_fps = 15,
	 .hts_def = 0x0a8c,
	 .vts_def = 0x09a4,
	 .reg_list = ov5647_2592x1944,
	 },
};

static int ov5647_write(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = i2c_master_send(client, data, 3);
	if (ret < 0)
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
			__func__, reg);

	return ret;
}

static int ov5647_read(struct v4l2_subdev *sd, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = i2c_master_send(client, data_w, 2);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
			__func__, reg);
		return ret;
	}

	ret = i2c_master_recv(client, val, 1);
	if (ret < 0)
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
			__func__, reg);

	return ret;
}

static int ov5647_read_client(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };

	ret = i2c_master_send(client, data_w, 2);
	if (ret < 0) {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
			__func__, reg);
		return ret;
	}

	ret = i2c_master_recv(client, val, 1);
	if (ret < 0)
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
			__func__, reg);

	return ret;
}

static int ov5647_write_array(struct v4l2_subdev *sd, struct regval_list *regs)
{
	int i, ret;

	for (i = 0; regs[i].addr != REG_NULL; i++) {
		ret = ov5647_write(sd, regs[i].addr, regs[i].data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ov5647_set_virtual_channel(struct v4l2_subdev *sd, int channel)
{
	u8 channel_id;
	int ret;

	ret = ov5647_read(sd, 0x4814, &channel_id);
	if (ret < 0)
		return ret;

	channel_id &= ~(3 << 6);
	return ov5647_write(sd, 0x4814, channel_id | (channel << 6));
}

static int ov5647_stream_on(struct v4l2_subdev *sd)
{
	int ret;

	ret = ov5647_write(sd, 0x4800, 0x04);
	if (ret < 0)
		return ret;

	ret = ov5647_write(sd, 0x4202, 0x00);
	if (ret < 0)
		return ret;

	return ov5647_write(sd, 0x300D, 0x00);
}

static int ov5647_stream_off(struct v4l2_subdev *sd)
{
	int ret;

	ret = ov5647_write(sd, 0x4800, 0x25);
	if (ret < 0)
		return ret;

	ret = ov5647_write(sd, 0x4202, 0x0f);
	if (ret < 0)
		return ret;

	return ov5647_write(sd, 0x300D, 0x01);
}

static int set_sw_standby(struct v4l2_subdev *sd, bool standby)
{
	int ret;
	u8 rdval;

	ret = ov5647_read(sd, 0x0100, &rdval);
	if (ret < 0)
		return ret;

	if (standby)
		rdval &= ~0x01;
	else
		rdval |= 0x01;

	return ov5647_write(sd, 0x0100, rdval);
}

static int ov5647_set_exposure(struct v4l2_subdev *sd, s32 val)
{
	int ret;

	ret = ov5647_write(sd, OV5647_REG_LINE_L, val & 0x00FF);
	if (ret < 0)
		return ret;

	ret = ov5647_write(sd, OV5647_REG_LINE_M, (val & 0xFF00) >> 8);
	if (ret < 0)
		return ret;

	return ov5647_write(sd, OV5647_REG_LINE_H, val >> 16);
}

static int ov5647_set_analog_gain(struct v4l2_subdev *sd, s32 val)
{
	int ret;

	ret = ov5647_write(sd, OV5647_REG_GAIN_L, val & 0xff);
	if (ret < 0)
		return ret;

	return ov5647_write(sd, OV5647_REG_GAIN_H, val >> 8);
}

static int __sensor_init(struct v4l2_subdev *sd)
{
	int ret;
	u8 resetval, rdval;
	struct ov5647_state *ov5647 = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = ov5647_read(sd, 0x0100, &rdval);
	if (ret < 0)
		return ret;

	ret = ov5647_write_array(sd, ov5647_common_regs);
	if (ret < 0) {
		dev_err(&client->dev, "write sensor common regs error\n");
		return ret;
	}

	ret = ov5647_write_array(sd, ov5647->cur_mode->reg_list);
	if (ret < 0) {
		dev_err(&client->dev, "write sensor mode regs error\n");
		return ret;
	}

	ret = ov5647_set_virtual_channel(sd, 0);
	if (ret < 0)
		return ret;
	ret = ov5647_set_exposure(sd, ov5647->exposure->val * 16);
	if (ret < 0)
		return ret;
	ret = ov5647_set_analog_gain(sd, ov5647->anal_gain->val);
	if (ret < 0)
		return ret;

	ret = ov5647_read(sd, 0x0100, &resetval);
	if (ret < 0)
		return ret;

	if (!(resetval & 0x01)) {
		dev_err(&client->dev, "Device was in SW standby");
		ret = ov5647_write(sd, 0x0100, 0x01);
		if (ret < 0)
			return ret;
	}

	return ov5647_stream_off(sd);
}

static int ov5647_sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	struct ov5647_state *ov5647 = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	mutex_lock(&ov5647->lock);

	if (on && !ov5647->power_count) {
		dev_dbg(&client->dev, "OV5647 power on\n");

		ret = clk_prepare_enable(ov5647->xclk);
		if (ret < 0) {
			dev_err(&client->dev, "clk prepare enable failed\n");
			goto out;
		}

		ret = ov5647_write_array(sd, sensor_oe_enable_regs);
		if (ret < 0) {
			clk_disable_unprepare(ov5647->xclk);
			dev_err(&client->dev,
				"write sensor_oe_enable_regs error\n");
			goto out;
		}

		ret = __sensor_init(sd);
		if (ret < 0) {
			clk_disable_unprepare(ov5647->xclk);
			dev_err(&client->dev,
				"Camera not available, check Power\n");
			goto out;
		}
	} else if (!on && ov5647->power_count == 1) {
		dev_dbg(&client->dev, "OV5647 power off\n");

		ret = ov5647_write_array(sd, sensor_oe_disable_regs);

		if (ret < 0)
			dev_dbg(&client->dev, "disable oe failed\n");

		ret = set_sw_standby(sd, true);

		if (ret < 0)
			dev_dbg(&client->dev, "soft stby failed\n");

		clk_disable_unprepare(ov5647->xclk);
	}

	/* Update the power count. */
	ov5647->power_count += on ? 1 : -1;
	WARN_ON(ov5647->power_count < 0);

out:
	mutex_unlock(&ov5647->lock);

	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5647_sensor_get_register(struct v4l2_subdev *sd,
				      struct v4l2_dbg_register *reg)
{
	u8 val;
	int ret;

	ret = ov5647_read(sd, reg->reg & 0xff, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = 1;

	return 0;
}

static int ov5647_sensor_set_register(struct v4l2_subdev *sd,
				      const struct v4l2_dbg_register *reg)
{
	return ov5647_write(sd, reg->reg & 0xff, reg->val & 0xff);
}
#endif

static void ov5647_get_module_inf(struct ov5647_state *ov5647,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, SENSOR_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ov5647->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ov5647->len_name, sizeof(inf->base.lens));
}

static long ov5647_sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov5647_state *ov5647 = to_state(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ov5647_get_module_inf(ov5647, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov5647_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov5647_sensor_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

/**
 * @short Subdev core operations registration
 */
static const struct v4l2_subdev_core_ops ov5647_subdev_core_ops = {
	.s_power = ov5647_sensor_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov5647_sensor_get_register,
	.s_register = ov5647_sensor_set_register,
#endif
	.ioctl = ov5647_sensor_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov5647_compat_ioctl32,
#endif
};

static int ov5647_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (enable)
		return ov5647_stream_on(sd);
	else
		return ov5647_stream_off(sd);
}

static int ov5647_g_frame_interval(struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *fi)
{
	struct ov5647_state *ov5647 = to_state(sd);
	const struct ov5647_mode *mode = ov5647->cur_mode;

	fi->interval.numerator = 10000;
	fi->interval.denominator = mode->max_fps * 10000;

	return 0;
}

static const struct v4l2_subdev_video_ops ov5647_subdev_video_ops = {
	.s_stream = ov5647_s_stream,
	.g_frame_interval = ov5647_g_frame_interval,
};

static int ov5647_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR8_1X8;

	return 0;
}

static int ov5647_get_reso_dist(const struct ov5647_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	    abs(mode->height - framefmt->height);
}

static const struct ov5647_mode *ov5647_find_best_fit(struct v4l2_subdev_format
						      *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ov5647_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ov5647_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov5647_state *ov5647 = to_state(sd);
	const struct ov5647_mode *mode;
	s64 h_blank, v_blank, pixel_rate;

	mutex_lock(&ov5647->lock);

	mode = ov5647_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SBGGR8_1X8;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
	} else {
		ov5647->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov5647->hblank, h_blank,
					 h_blank, 1, h_blank);
		v_blank = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov5647->vblank, v_blank,
					 v_blank, 1, v_blank);
		pixel_rate = mode->vts_def * mode->hts_def * mode->max_fps;
		__v4l2_ctrl_modify_range(ov5647->pixel_rate, pixel_rate,
					 pixel_rate, 1, pixel_rate);
	}

	mutex_unlock(&ov5647->lock);

	return 0;
}

static int ov5647_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov5647_state *ov5647 = to_state(sd);
	const struct ov5647_mode *mode = ov5647->cur_mode;

	mutex_lock(&ov5647->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SBGGR8_1X8;
		fmt->format.field = V4L2_FIELD_NONE;
	}

	mutex_unlock(&ov5647->lock);

	return 0;
}

static int ov5647_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct v4l2_fract max_fps;
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_SBGGR8_1X8)
		return -EINVAL;

	max_fps.numerator = 10000;
	max_fps.denominator = supported_modes[fie->index].max_fps * 10000;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = max_fps;
	return 0;
}

static int ov5647_enum_frame_sizes(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR8_1X8)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static const struct v4l2_subdev_pad_ops ov5647_subdev_pad_ops = {
	.enum_mbus_code = ov5647_enum_mbus_code,
	.enum_frame_interval = ov5647_enum_frame_interval,
	.enum_frame_size = ov5647_enum_frame_sizes,
	.get_fmt = ov5647_get_fmt,
	.set_fmt = ov5647_set_fmt,
};

static const struct v4l2_subdev_ops ov5647_subdev_ops = {
	.core = &ov5647_subdev_core_ops,
	.video = &ov5647_subdev_video_ops,
	.pad = &ov5647_subdev_pad_ops,
};

static int ov5647_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5647_state *ov5647 =
	    container_of(ctrl->handler, struct ov5647_state, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ov5647_set_exposure(&ov5647->sd, ctrl->val * 16);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ov5647_set_analog_gain(&ov5647->sd, ctrl->val);
		break;
	default:
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ov5647_ctrl_ops = {
	.s_ctrl = ov5647_set_ctrl,
};

static int ov5647_initialize_controls(struct v4l2_subdev *sd)
{
	struct v4l2_ctrl_handler *handler;
	struct ov5647_state *ov5647 = to_state(sd);
	const struct ov5647_mode *mode = ov5647->cur_mode;
	s64 pixel_rate, h_blank, v_blank;
	int ret;

	handler = &ov5647->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 1);
	if (ret)
		return ret;

	/* freq */
	ov5647->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
						   V4L2_CID_LINK_FREQ,
						   0, 0, link_freq_menu_items);
	if (ov5647->link_freq)
		ov5647->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	pixel_rate = mode->vts_def * mode->hts_def * mode->max_fps;
	ov5647->pixel_rate =
	    v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0, pixel_rate,
			      1, pixel_rate);

	/* blank */
	h_blank = mode->hts_def - mode->width;
	ov5647->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	v_blank = mode->vts_def - mode->height;
	ov5647->vblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_VBLANK,
					   v_blank, v_blank, 1, v_blank);

	/* exposure */
	ov5647->exposure = v4l2_ctrl_new_std(handler, &ov5647_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     OV5647_EXPOSURE_MIN,
					     OV5647_EXPOSURE_MAX,
					     OV5647_EXPOSURE_STEP,
					     OV5647_EXPOSURE_DEFAULT);
	ov5647->anal_gain =
	    v4l2_ctrl_new_std(handler, &ov5647_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			      OV5647_ANALOG_GAIN_MIN, OV5647_ANALOG_GAIN_MAX,
			      OV5647_ANALOG_GAIN_STEP,
			      OV5647_ANALOG_GAIN_DEFAULT);

	if (handler->error) {
		v4l2_ctrl_handler_free(handler);
		return handler->error;
	}

	sd->ctrl_handler = handler;

	return 0;
}

static int ov5647_detect(struct i2c_client *client)
{
	u8 read;
	int ret;

	ret = ov5647_read_client(client, OV5647_REG_CHIPID_H, &read);
	if (ret < 0)
		return ret;

	dev_err(&client->dev, "ID High expected 0x56 got %x", read);
	if (read != 0x56) {
		return -ENODEV;
	}

	ret = ov5647_read_client(client, OV5647_REG_CHIPID_L, &read);
	if (ret < 0)
		return ret;

	dev_err(&client->dev, "ID Low expected 0x47 got %x", read);
	if (read != 0x47) {
		return -ENODEV;
	}

	return ret;
}

static int ov5647_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov5647_state *ov5647 = to_state(sd);
	struct v4l2_mbus_framefmt *try_fmt =
	    v4l2_subdev_get_try_format(sd, fh->pad, 0);

	mutex_lock(&ov5647->lock);
	/* Initialize try_fmt */
	try_fmt->width = ov5647->cur_mode->width;
	try_fmt->height = ov5647->cur_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SBGGR8_1X8;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov5647->lock);
	/* No crop or compose */
	return 0;
}

static const struct v4l2_subdev_internal_ops ov5647_subdev_internal_ops = {
	.open = ov5647_open,
};

static int ov5647_parse_dt(struct device_node *np)
{
	struct v4l2_fwnode_endpoint bus_cfg;
	struct device_node *ep;

	int ret;

	ep = of_graph_get_next_endpoint(np, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &bus_cfg);

	of_node_put(ep);

	return ret;
}

static int imx219_detect(struct i2c_client *client)
{
	u16 model_id;
	int ret;

	/* Check and show model, lot, and chip ID. */
	ret = reg_read(client, 0x0000);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Model ID (high byte)\n");
		return -ENODEV;
	}
	model_id = ret << 8;

	ret = reg_read(client, 0x0001);
	if (ret < 0) {
		dev_err(&client->dev, "Failure to read Model ID (low byte)\n");
		return -ENODEV;
	}
	model_id |= ret;

	if (model_id != 0x0219) {
		dev_err(&client->dev, "Model ID: %x not supported!\n",
			model_id);
		return -ENODEV;
	}
	dev_info(&client->dev, "Model ID 0x%04x\n", model_id);

	if(model_id != 0x219)
		return -ENODEV;

	return ret;
}

static int cam_sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct ov5647_state *ov5647;
	struct imx219 *priv;
	struct v4l2_subdev *sd;
	struct device_node *np = client->dev.of_node;
	struct gpio_desc	*enable_gpio;
	int ret;
	char facing[2];

	enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(enable_gpio))
		dev_warn(dev, "Failed to get enable_gpios\n");

	ret = ov5647_detect(client);
	if (ret < 0) {
		client->addr = 0x10;
		ret = imx219_detect(client);
		if(ret < 0) return ret;
		isOV5647 = 0;
	} else {
		isOV5647 = 1;
	}

	if(isOV5647 == 1) {
		ov5647 = devm_kzalloc(dev, sizeof(*ov5647), GFP_KERNEL);
		if (!ov5647)
			return -ENOMEM;

		if (IS_ENABLED(CONFIG_OF) && np) {
			ret = ov5647_parse_dt(np);
			if (ret) {
				dev_err(dev, "DT parsing error: %d\n", ret);
				return ret;
			}
		}

		ov5647->enable_gpio = enable_gpio;
		ov5647->module_index = 0;
		ov5647->module_facing = "back";
		ov5647->module_name = "RPi_v13";
		ov5647->len_name = "default";

		/* get system clock (xclk) */
		ov5647->xclk = devm_clk_get(dev, "xvclk");
		if (IS_ERR(ov5647->xclk)) {
			dev_err(dev, "could not get xclk");
			return PTR_ERR(ov5647->xclk);
		}
		ret = clk_set_rate(ov5647->xclk, OV5647_XVCLK_FREQ);
		if (ret < 0) {
			dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
			return ret;
		}
		if (clk_get_rate(ov5647->xclk) != OV5647_XVCLK_FREQ)
			dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

		ov5647->pinctrl = devm_pinctrl_get(dev);
		if (!IS_ERR(ov5647->pinctrl)) {
			ov5647->pins_default =
				pinctrl_lookup_state(ov5647->pinctrl,
							 OF_CAMERA_PINCTRL_STATE_DEFAULT);
			if (IS_ERR(ov5647->pins_default))
				dev_err(dev, "could not get default pinstate\n");
		}
		if (!IS_ERR_OR_NULL(ov5647->pins_default)) {
			ret = pinctrl_select_state(ov5647->pinctrl,
						   ov5647->pins_default);
			if (ret < 0)
				dev_err(dev, "could not set pins\n");
		}

		mutex_init(&ov5647->lock);

		ov5647->cur_mode = &supported_modes[0];
		sd = &ov5647->sd;
		v4l2_i2c_subdev_init(sd, client, &ov5647_subdev_ops);

		ret = ov5647_initialize_controls(sd);
		if (ret)
			return ret;

		ov5647->sd.internal_ops = &ov5647_subdev_internal_ops;
		ov5647->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

		ov5647->pad.flags = MEDIA_PAD_FL_SOURCE;
		sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
		ret = media_entity_pads_init(&sd->entity, 1, &ov5647->pad);
		if (ret < 0)
			goto mutex_remove;

		memset(facing, 0, sizeof(facing));
		if (strcmp(ov5647->module_facing, "back") == 0)
			facing[0] = 'b';
		else
			facing[0] = 'f';

		snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
			 ov5647->module_index, facing,
			 SENSOR_NAME, dev_name(sd->dev));

		ret = v4l2_async_register_subdev(sd);
		if (ret < 0)
			goto error;

		dev_info(dev, "%s probe done.\n", sd->name);
		return 0;
error:
		media_entity_cleanup(&sd->entity);
mutex_remove:
		mutex_destroy(&ov5647->lock);
	} else {
		if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
			dev_warn(&adapter->dev,
				 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_BYTE\n");
			return -EIO;
		}
		priv = devm_kzalloc(&client->dev, sizeof(struct imx219), GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		priv->enable_gpio = enable_gpio;
		priv->module_index = 0;
		priv->module_facing = "back";
		priv->module_name = "RPi_v21";
		priv->len_name = "default";

		priv->pinctrl = devm_pinctrl_get(dev);
		if (!IS_ERR(priv->pinctrl)) {
			priv->pins_default =
				pinctrl_lookup_state(priv->pinctrl,
							 OF_CAMERA_PINCTRL_STATE_DEFAULT);
			if (IS_ERR(priv->pins_default))
				dev_err(dev, "could not get default pinstate\n");
		}
		if (!IS_ERR_OR_NULL(priv->pins_default)) {
			ret = pinctrl_select_state(priv->pinctrl,
						   priv->pins_default);
			if (ret < 0)
				dev_err(dev, "could not set pins\n");
		}

		priv->clk = devm_clk_get(&client->dev, "xvclk");
		if (IS_ERR(priv->clk)) {
			dev_info(&client->dev, "Error %ld getting clock\n",
				 PTR_ERR(priv->clk));
			return -EPROBE_DEFER;
		}

		/* 1920 * 1080 by default */
		priv->cur_mode = &imx219_supported_modes[0];
		priv->cfg_num = ARRAY_SIZE(imx219_supported_modes);

		priv->crop_rect.left = 680;
		priv->crop_rect.top = 692;
		priv->crop_rect.width = priv->cur_mode->width;
		priv->crop_rect.height = priv->cur_mode->height;

		v4l2_i2c_subdev_init(&priv->subdev, client, &imx219_subdev_ops);
		ret = imx219_ctrls_init(&priv->subdev);
		if (ret < 0)
			return ret;

		priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				 V4L2_SUBDEV_FL_HAS_EVENTS;

		priv->pad.flags = MEDIA_PAD_FL_SOURCE;
		priv->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
		ret = media_entity_pads_init(&priv->subdev.entity, 1, &priv->pad);
		if (ret < 0)
			return ret;

		sd = &priv->subdev;
		memset(facing, 0, sizeof(facing));
		if (strcmp(priv->module_facing, "back") == 0)
			facing[0] = 'b';
		else
			facing[0] = 'f';

		snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
			 priv->module_index, facing,
			 IMX219_NAME, dev_name(sd->dev));
		ret = v4l2_async_register_subdev_sensor_common(sd);
		if (ret < 0)
			return ret;

		dev_info(&client->dev, "%s probe done.\n", sd->name);
		return ret;
	}

	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	if(isOV5647 == 1) {
		struct v4l2_subdev *sd = i2c_get_clientdata(client);
		struct ov5647_state *ov5647 = to_state(sd);
		v4l2_async_unregister_subdev(&ov5647->sd);
		media_entity_cleanup(&ov5647->sd.entity);
		v4l2_device_unregister_subdev(sd);
		mutex_destroy(&ov5647->lock);
	} else {
		struct imx219 *priv = to_imx219(client);
		v4l2_async_unregister_subdev(&priv->subdev);
		media_entity_cleanup(&priv->subdev.entity);
		v4l2_ctrl_handler_free(&priv->ctrl_handler);
	}
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{"cam_sensor", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sensor_of_match[] = {
	{.compatible = "asus,cam_sensor"},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, sensor_of_match);
#endif

static struct i2c_driver sensor_driver = {
	.driver = {
		   .of_match_table = of_match_ptr(sensor_of_match),
		   .name = "cam_sensor",
		   },
	.probe = cam_sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

module_i2c_driver(sensor_driver);

MODULE_AUTHOR("Xuanhao Zhang <xuanhao_zhang@asus.com>");
MODULE_DESCRIPTION("A low-level driver for Asus camera sensors");
MODULE_LICENSE("GPL v2");

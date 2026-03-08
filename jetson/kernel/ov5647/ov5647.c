// SPDX-License-Identifier: GPL-2.0
/*
 * ov5647.c - OV5647 camera driver for NVIDIA Tegra (JetPack 4.6 / L4T R32.7)
 *
 * Adapted from:
 *  - Raspberry Pi OV5647 driver (drivers/media/i2c/ov5647.c)
 *  - NVIDIA IMX219 tegracam driver
 *
 * Copyright (c) 2013 Raspberry Pi Foundation
 * Copyright (c) 2023 sensor_fusion project
 *
 * Supports: 1920x1080 @ 30fps, 1280x720 @ 60fps (RAW10, 2-lane MIPI)
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <media/camera_common.h>

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("sensor_fusion project");
MODULE_DESCRIPTION("OV5647 camera driver for NVIDIA Tegra (JetPack 4.6)");

/* OV5647 register map (16-bit address, 8-bit value) */
#define OV5647_CHIPID_H         0x300A
#define OV5647_CHIPID_L         0x300B
#define OV5647_CHIP_ID_H_VAL    0x56
#define OV5647_CHIP_ID_L_VAL    0x47

#define OV5647_SW_RESET         0x0103
#define OV5647_STREAM_ON        0x0100
#define OV5647_REG_GAIN_H       0x350A
#define OV5647_REG_GAIN_L       0x350B
#define OV5647_REG_EXP_H        0x3501
#define OV5647_REG_EXP_L        0x3502
#define OV5647_REG_VTS_H        0x380E
#define OV5647_REG_VTS_L        0x380F

#define OV5647_GAIN_MIN         0
#define OV5647_GAIN_MAX         1023
#define OV5647_GAIN_DEFAULT     64   /* 1x */

#define OV5647_EXPO_MIN         1
#define OV5647_EXPO_MAX         1100
#define OV5647_EXPO_DEFAULT     400

#define OV5647_FPS_MIN          2000000
#define OV5647_FPS_MAX          30000000
#define OV5647_FPS_DEFAULT      30000000

/* VTS for 30 fps at the given HTS (HTS=2500) */
#define OV5647_VTS_30FPS        1104
#define OV5647_VTS_60FPS        572

struct ov5647_reg {
	u16 addr;
	u8  val;
};

/* ---------------------------------------------------------------
 * Register tables
 * --------------------------------------------------------------- */

/* Common init: applied before mode-specific table */
static const struct ov5647_reg ov5647_common_regs[] = {
	{0x0100, 0x00},  /* software standby */
	{0x0103, 0x01},  /* software reset */
	/* Short delay for reset to complete */
	{0x3034, 0x1a},  /* MIPI 10-bit mode */
	{0x3035, 0x21},  /* PLL1 pre-divider=2, mult=1 */
	{0x3036, 0x62},  /* PLL multiplier = 98 → 588 MHz VCO */
	{0x303c, 0x11},  /* PLL ctrl */
	{0x3106, 0xf5},  /* SRB clock divider */
	{0x3827, 0xec},
	{0x370c, 0x03},
	{0x3612, 0x5b},
	{0x3618, 0x04},
	{0x5000, 0x06},  /* ISP ctrl: AWB off, color matrix on */
	{0x5002, 0x40},
	{0x5003, 0x08},
	{0x5a00, 0x08},
	{0x3000, 0x00},  /* system clock enable */
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3016, 0x08},  /* drive strength */
	{0x3017, 0xe0},
	{0x3018, 0x44},  /* MIPI 2 data lanes */
	{0x301c, 0xf8},
	{0x301d, 0xf0},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},  /* AEC gain ceiling */
	{0x3c01, 0x80},
	{0x3b07, 0x0c},
	{0x3630, 0x2e},
	{0x3632, 0xe2},
	{0x3633, 0x23},
	{0x3634, 0x44},
	{0x3636, 0x06},
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
	{0x3a0f, 0x58},  /* AEC in-zone high */
	{0x3a10, 0x50},  /* AEC in-zone low */
	{0x3a1b, 0x58},
	{0x3a1e, 0x50},
	{0x3a11, 0x60},  /* AEC fast zone high */
	{0x3a1f, 0x28},  /* AEC fast zone low */
	{0x4001, 0x02},  /* BLC start line */
	{0x4004, 0x04},  /* BLC line number */
	{0x4000, 0x09},
	{0x4837, 0x19},  /* MIPI pclk period */
	{0x4800, 0x34},  /* MIPI LPDT + non-continuous clock */
	{0x3503, 0x03},  /* AEC/AGC manual */
	{0x3820, 0x41},  /* no flip */
	{0x3821, 0x07},  /* no mirror */
};

/* 1920x1080 @ 30fps, RAW10, 2-lane MIPI */
static const struct ov5647_reg ov5647_mode_1080p[] = {
	{0x3808, 0x07},  /* output width  = 0x0780 = 1920 */
	{0x3809, 0x80},
	{0x380a, 0x04},  /* output height = 0x0438 = 1080 */
	{0x380b, 0x38},
	{0x3800, 0x01},  /* x start = 0x015c = 348 */
	{0x3801, 0x5c},
	{0x3802, 0x01},  /* y start = 0x01b2 = 434 */
	{0x3803, 0xb2},
	{0x3804, 0x08},  /* x end   = 0x08e3 = 2275 */
	{0x3805, 0xe3},
	{0x3806, 0x05},  /* y end   = 0x05f1 = 1521 */
	{0x3807, 0xf1},
	{0x3811, 0x04},  /* x win offset */
	{0x3813, 0x02},  /* y win offset */
	{0x3814, 0x11},  /* x inc odd=1, even=1 (no binning) */
	{0x3815, 0x11},  /* y inc */
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x380c, 0x09},  /* HTS high = 0x09c4 = 2500 */
	{0x380d, 0xc4},
	{0x380e, 0x04},  /* VTS high = 0x0450 = 1104 → ~30fps */
	{0x380f, 0x50},
	{0x3a08, 0x01},  /* AEC banding filter 50Hz step */
	{0x3a09, 0x27},
	{0x3a0a, 0x00},  /* AEC banding filter 60Hz step */
	{0x3a0b, 0xf6},
	{0x3a0d, 0x04},
	{0x3a0e, 0x03},
	{0x3501, 0x43},  /* exposure = 0x430 = 1072 lines */
	{0x3502, 0x00},
	{0x350a, 0x00},  /* gain high */
	{0x350b, 0x40},  /* gain low = 64 = 1x */
};

/* 1280x720 @ 60fps, RAW10, 2-lane MIPI */
static const struct ov5647_reg ov5647_mode_720p[] = {
	{0x3808, 0x05},  /* output width  = 1280 */
	{0x3809, 0x00},
	{0x380a, 0x02},  /* output height = 720 */
	{0x380b, 0xd0},
	{0x3800, 0x01},
	{0x3801, 0xb4},
	{0x3802, 0x02},
	{0x3803, 0x22},
	{0x3804, 0x07},
	{0x3805, 0x8b},
	{0x3806, 0x04},
	{0x3807, 0xdd},
	{0x3811, 0x08},
	{0x3813, 0x02},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x380c, 0x07},  /* HTS = 0x07b4 = 1972 */
	{0x380d, 0xb4},
	{0x380e, 0x02},  /* VTS = 0x02f4 = 756 → ~60fps */
	{0x380f, 0xf4},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0d, 0x04},
	{0x3a0e, 0x03},
	{0x3501, 0x2e},
	{0x3502, 0x80},
	{0x350a, 0x00},
	{0x350b, 0x40},
};

/* ---------------------------------------------------------------
 * Mode table for tegracam
 * --------------------------------------------------------------- */
static const int ov5647_60fps_fr[] = {60};
static const int ov5647_30fps_fr[] = {30};

static const struct camera_common_frmfmt ov5647_frmfmt[] = {
	{{1920, 1080}, ov5647_30fps_fr, 1, 0, 0},  /* mode 0: 1080p30 */
	{{1280,  720}, ov5647_60fps_fr, 1, 0, 1},  /* mode 1: 720p60  */
};

/* ---------------------------------------------------------------
 * Driver private data
 * --------------------------------------------------------------- */
struct ov5647 {
	struct i2c_client           *i2c_client;
	struct v4l2_subdev          *subdev;
	struct camera_common_data   *s_data;
	struct tegracam_device      *tc_dev;

	/* GPIOs (from device tree) */
	int                          reset_gpio;
	int                          pwdn_gpio;

	/* Regulators */
	struct regulator            *vana;   /* analogue 2.8V */
	struct regulator            *vdig;   /* digital  1.8V */
	struct regulator            *vddio;  /* IO       1.8V */

	/* Current state */
	u32 frame_length;
	s64 last_wdr_et_val;
};

/* ---------------------------------------------------------------
 * I2C helpers
 * --------------------------------------------------------------- */
static int ov5647_write_reg(struct camera_common_data *s_data,
			    u16 addr, u8 val)
{
	int err;
	u32 reg32 = val;

	err = regmap_write(s_data->regmap, addr, reg32);
	if (err)
		dev_err(s_data->dev, "regmap_write 0x%04x failed: %d\n",
			addr, err);
	return err;
}

static int ov5647_read_reg(struct camera_common_data *s_data,
			   u16 addr, u8 *val)
{
	int err;
	u32 reg_val;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	if (err) {
		dev_err(s_data->dev, "regmap_read 0x%04x failed: %d\n",
			addr, err);
		return err;
	}
	*val = reg_val & 0xFF;
	return 0;
}

static int ov5647_write_table(struct ov5647 *priv,
			      const struct ov5647_reg *table,
			      int len)
{
	int i, err;

	for (i = 0; i < len; i++) {
		err = ov5647_write_reg(priv->s_data, table[i].addr, table[i].val);
		if (err)
			return err;
	}
	return 0;
}

/* ---------------------------------------------------------------
 * Power management
 * --------------------------------------------------------------- */
static int ov5647_power_on(struct camera_common_data *s_data)
{
	struct ov5647 *priv = (struct ov5647 *)s_data->priv;
	struct device *dev = s_data->dev;
	int err;

	dev_dbg(dev, "%s\n", __func__);

	/* Enable regulators if present */
	if (!IS_ERR(priv->vdig)) {
		err = regulator_enable(priv->vdig);
		if (err) {
			dev_err(dev, "vdig enable failed: %d\n", err);
			return err;
		}
		usleep_range(1000, 1200);
	}

	if (!IS_ERR(priv->vana)) {
		err = regulator_enable(priv->vana);
		if (err) {
			dev_err(dev, "vana enable failed: %d\n", err);
			goto err_vana;
		}
	}

	if (!IS_ERR(priv->vddio)) {
		err = regulator_enable(priv->vddio);
		if (err) {
			dev_err(dev, "vddio enable failed: %d\n", err);
			goto err_vddio;
		}
	}

	/* Deassert reset (active low) */
	if (gpio_is_valid(priv->reset_gpio)) {
		gpio_set_value(priv->reset_gpio, 0);
		usleep_range(1000, 1200);
		gpio_set_value(priv->reset_gpio, 1);
		usleep_range(30000, 31000);  /* 30ms for sensor to init */
	}

	/* Enable MCLK */
	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev, "mclk enable failed: %d\n", err);
		goto err_mclk;
	}
	usleep_range(1000, 1200);

	return 0;

err_mclk:
	if (!IS_ERR(priv->vddio))
		regulator_disable(priv->vddio);
err_vddio:
	if (!IS_ERR(priv->vana))
		regulator_disable(priv->vana);
err_vana:
	if (!IS_ERR(priv->vdig))
		regulator_disable(priv->vdig);
	return err;
}

static int ov5647_power_off(struct camera_common_data *s_data)
{
	struct ov5647 *priv = (struct ov5647 *)s_data->priv;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s\n", __func__);

	camera_common_mclk_disable(s_data);

	if (gpio_is_valid(priv->reset_gpio))
		gpio_set_value(priv->reset_gpio, 0);

	if (!IS_ERR(priv->vddio))
		regulator_disable(priv->vddio);
	if (!IS_ERR(priv->vana))
		regulator_disable(priv->vana);
	if (!IS_ERR(priv->vdig))
		regulator_disable(priv->vdig);

	return 0;
}

static int ov5647_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct ov5647 *priv = (struct ov5647 *)tc_dev->priv;
	int err = 0;

	/* tegracam_device_register does not allocate s_data->power — do it here */
	if (!pw) {
		pw = devm_kzalloc(dev, sizeof(*pw), GFP_KERNEL);
		if (!pw)
			return -ENOMEM;
		s_data->power = pw;
	}
	pw->state = SWITCH_OFF;

	/* Regulators are optional — sensor works without them on the RPi flat cable */
	priv->vana  = devm_regulator_get_optional(dev, "vana");
	priv->vdig  = devm_regulator_get_optional(dev, "vdig");
	priv->vddio = devm_regulator_get_optional(dev, "vddio");

	/* Reset GPIO — try DT first, fall back to GPIO 151 (CSI0 on Jetson Nano B01) */
	priv->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpios", 0);
	if (!gpio_is_valid(priv->reset_gpio)) {
		priv->reset_gpio = 151;  /* TEGRA_GPIO(S,7): Jetson Nano B01 CSI0 reset */
		dev_info(dev, "reset-gpios not in DT, using hardcoded GPIO %d\n",
			 priv->reset_gpio);
	}
	err = devm_gpio_request_one(dev, priv->reset_gpio,
				    GPIOF_OUT_INIT_LOW, "ov5647-reset");
	if (err) {
		dev_warn(dev, "reset GPIO %d request failed: %d — continuing\n",
			 priv->reset_gpio, err);
		priv->reset_gpio = -EINVAL;
	}

	/* Power-down GPIO (optional) */
	priv->pwdn_gpio = of_get_named_gpio(dev->of_node, "pwdn-gpios", 0);

	return 0;
}

static int ov5647_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

/* ---------------------------------------------------------------
 * parse_dt
 * --------------------------------------------------------------- */
static struct camera_common_pdata *ov5647_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	int err;

	board_priv_pdata = devm_kzalloc(dev,
			sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_dbg(dev, "mclk not in DT, using default\n");

	return board_priv_pdata;
}

/* ---------------------------------------------------------------
 * Streaming
 * --------------------------------------------------------------- */
static int ov5647_set_mode(struct tegracam_device *tc_dev)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	int mode_id = s_data->sensor_mode_id;
	int err;

	dev_dbg(tc_dev->dev, "%s: mode %d\n", __func__, mode_id);

	/* Write common init registers */
	err = ov5647_write_table(priv, ov5647_common_regs,
				 ARRAY_SIZE(ov5647_common_regs));
	if (err)
		return err;

	/* Short delay after SW reset */
	usleep_range(5000, 6000);

	/* Write mode-specific registers */
	switch (mode_id) {
	case 0:
		err = ov5647_write_table(priv, ov5647_mode_1080p,
					 ARRAY_SIZE(ov5647_mode_1080p));
		break;
	case 1:
		err = ov5647_write_table(priv, ov5647_mode_720p,
					 ARRAY_SIZE(ov5647_mode_720p));
		break;
	default:
		dev_err(tc_dev->dev, "unknown mode %d\n", mode_id);
		return -EINVAL;
	}

	return err;
}

static int ov5647_start_streaming(struct tegracam_device *tc_dev)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);

	dev_dbg(tc_dev->dev, "%s\n", __func__);
	return ov5647_write_reg(priv->s_data, OV5647_STREAM_ON, 0x01);
}

static int ov5647_stop_streaming(struct tegracam_device *tc_dev)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);

	dev_dbg(tc_dev->dev, "%s\n", __func__);
	return ov5647_write_reg(priv->s_data, OV5647_STREAM_ON, 0x00);
}

/* ---------------------------------------------------------------
 * Controls
 * --------------------------------------------------------------- */
static int ov5647_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	u16 gain = (u16)val;
	int err;

	/* Gain: 10-bit value in regs 0x350A[1:0] | 0x350B[7:0] */
	err = ov5647_write_reg(priv->s_data, OV5647_REG_GAIN_H,
			       (gain >> 8) & 0x03);
	if (err)
		return err;
	return ov5647_write_reg(priv->s_data, OV5647_REG_GAIN_L,
				gain & 0xFF);
}

static int ov5647_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	u16 exp = (u16)(val >> 4);  /* exposure in line units, val is in µs */
	int err;

	/* Exposure: 16-bit line count in 0x3501[7:0] | 0x3502[7:4] */
	err = ov5647_write_reg(priv->s_data, OV5647_REG_EXP_H,
			       (exp >> 8) & 0xFF);
	if (err)
		return err;
	return ov5647_write_reg(priv->s_data, OV5647_REG_EXP_L,
				(exp & 0xFF) << 4);
}

static int ov5647_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct ov5647 *priv = (struct ov5647 *)tegracam_get_privdata(tc_dev);
	/* val is in milli-fps. Adjust VTS to achieve requested fps.
	 * pclk = ~81.67 MHz, HTS = 2500 → fps = pclk / (HTS * VTS)
	 * VTS = pclk / (HTS * fps_hz)
	 */
	u32 fps_hz   = (u32)(val / 1000000);
	u32 pclk_hz  = 81666666;
	u32 hts      = 2500;
	u32 vts;
	int err;

	if (!fps_hz)
		fps_hz = 30;

	vts = pclk_hz / (hts * fps_hz);
	if (vts < 720)
		vts = 720;
	if (vts > 0x7FFF)
		vts = 0x7FFF;

	priv->frame_length = vts;

	err = ov5647_write_reg(priv->s_data, OV5647_REG_VTS_H,
			       (vts >> 8) & 0xFF);
	if (err)
		return err;
	return ov5647_write_reg(priv->s_data, OV5647_REG_VTS_L,
				vts & 0xFF);
}

static int ov5647_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	/* OV5647 doesn't have a dedicated group hold register — no-op */
	return 0;
}

/* ---------------------------------------------------------------
 * Ops tables
 * --------------------------------------------------------------- */
static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
};

static struct tegracam_ctrl_ops ov5647_ctrl_ops = {
	.numctrls       = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list  = ctrl_cid_list,
	.set_gain       = ov5647_set_gain,
	.set_exposure   = ov5647_set_exposure,
	.set_frame_rate = ov5647_set_frame_rate,
	.set_group_hold = ov5647_set_group_hold,
};

static struct camera_common_sensor_ops ov5647_common_ops = {
	.numfrmfmts  = ARRAY_SIZE(ov5647_frmfmt),
	.frmfmt_table = ov5647_frmfmt,
	.power_on    = ov5647_power_on,
	.power_off   = ov5647_power_off,
	.write_reg   = ov5647_write_reg,
	.read_reg    = ov5647_read_reg,
	.parse_dt    = ov5647_parse_dt,
	.power_get   = ov5647_power_get,
	.power_put   = ov5647_power_put,
	.set_mode    = ov5647_set_mode,
	.start_streaming  = ov5647_start_streaming,
	.stop_streaming   = ov5647_stop_streaming,
};

/* ---------------------------------------------------------------
 * Regmap config (16-bit register, 8-bit value)
 * --------------------------------------------------------------- */
static const struct regmap_config ov5647_regmap_config = {
	.reg_bits    = 16,
	.val_bits    = 8,
	.cache_type  = REGCACHE_RBTREE,
};

/* ---------------------------------------------------------------
 * V4L2 subdev ops (minimal — tegracam provides most)
 * --------------------------------------------------------------- */
static struct v4l2_subdev_ops ov5647_subdev_ops = {
	.core  = NULL,
	.video = NULL,
	.pad   = NULL,
};

static const struct v4l2_subdev_internal_ops ov5647_internal_ops = {
	.open  = NULL,
};

/* ---------------------------------------------------------------
 * I2C probe
 * --------------------------------------------------------------- */
static int ov5647_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct ov5647 *priv;
	int err;
	u8 chip_id_h, chip_id_l;

	dev_info(dev, "probing OV5647 at I2C addr 0x%02x\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev, sizeof(*tc_dev), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev      = dev;
	strncpy(tc_dev->name, "ov5647", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config    = &ov5647_regmap_config;
	tc_dev->sensor_ops           = &ov5647_common_ops;
	tc_dev->v4l2sd_ops           = &ov5647_subdev_ops;
	tc_dev->v4l2sd_internal_ops  = &ov5647_internal_ops;
	tc_dev->tcctrl_ops           = &ov5647_ctrl_ops;
	/* Must be set before tegracam_device_register — power_get is called from inside it */
	tc_dev->priv = priv;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegracam_device_register failed: %d\n", err);
		return err;
	}

	priv->tc_dev = tc_dev;
	priv->s_data  = tc_dev->s_data;
	priv->subdev  = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	/* Verify chip ID (requires power to be up — do after device_register
	 * so regulators/mclk are available via power_get) */
	err = ov5647_power_on(priv->s_data);
	if (err) {
		dev_warn(dev, "power_on for chip ID check failed: %d\n", err);
	} else {
		usleep_range(5000, 6000);
		err  = ov5647_read_reg(priv->s_data, OV5647_CHIPID_H, &chip_id_h);
		err |= ov5647_read_reg(priv->s_data, OV5647_CHIPID_L, &chip_id_l);
		if (err || chip_id_h != OV5647_CHIP_ID_H_VAL ||
		    chip_id_l != OV5647_CHIP_ID_L_VAL) {
			dev_err(dev, "wrong chip ID: 0x%02x%02x (expected 0x5647)\n",
				chip_id_h, chip_id_l);
			ov5647_power_off(priv->s_data);
			tegracam_device_unregister(tc_dev);
			return -ENODEV;
		}
		dev_info(dev, "OV5647 chip ID: 0x%02x%02x — OK\n",
			 chip_id_h, chip_id_l);
		ov5647_power_off(priv->s_data);
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegracam_v4l2subdev_register failed: %d\n", err);
		tegracam_device_unregister(tc_dev);
		return err;
	}

	dev_info(dev, "OV5647 probe succeeded\n");
	return 0;
}

static int ov5647_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ov5647 *priv;

	if (!s_data)
		return -EINVAL;

	priv = (struct ov5647 *)s_data->priv;
	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);
	return 0;
}

/* ---------------------------------------------------------------
 * I2C driver registration
 * --------------------------------------------------------------- */
static const struct i2c_device_id ov5647_id[] = {
	{"ov5647", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ov5647_id);

static const struct of_device_id ov5647_of_match[] = {
	{.compatible = "nvidia,ov5647"},
	{}
};
MODULE_DEVICE_TABLE(of, ov5647_of_match);

static struct i2c_driver ov5647_i2c_driver = {
	.driver = {
		.name           = "ov5647",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(ov5647_of_match),
	},
	.probe    = ov5647_probe,
	.remove   = ov5647_remove,
	.id_table = ov5647_id,
};

module_i2c_driver(ov5647_i2c_driver);

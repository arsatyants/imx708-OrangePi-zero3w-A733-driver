// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner VIN sensor driver for the Sony IMX708 (Raspberry Pi Camera Module 3).
 *
 * Port of the Raspberry Pi imx708 driver (Copyright (C) 2022, Raspberry Pi Ltd)
 * to the Allwinner BSP "vin" camera framework, modelled on the BSP imx219 driver.
 * Register tables: imx708_regs.h (verbatim from the Raspberry Pi driver).
 *
 * 2 MIPI CSI-2 lanes, 450 MHz link (900 Mbit/s per lane), RAW10, 24 MHz INCK.
 * Non-HDR modes only. PDAF and the focus motor (DW9817 VCM) are not driven.
 */
#include "../../utility/vin_log.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>
#include "camera.h"
#include "sensor_helper.h"
#include "imx708_regs.h"

MODULE_AUTHOR("h.264-video-test");
MODULE_DESCRIPTION("A low-level driver for IMX708 sensors (Allwinner VIN)");
MODULE_LICENSE("GPL");

#define MCLK			(24 * 1000 * 1000)
#define I2C_ADDR		0x34	/* 8-bit form of 0x1a */
#define SENSOR_NUM		2
#define SENSOR_NAME		"imx708"
#define SENSOR_NAME_2		"imx708_2"

#define IMX708_REG_CHIP_ID	0x0016
#define IMX708_CHIP_ID		0x0708
#define IMX708_REG_MODE_SELECT	0x0100
#define IMX708_REG_ORIENTATION	0x0101
#define IMX708_REG_HOLD		0x0104
#define IMX708_REG_EXPOSURE	0x0202
#define IMX708_REG_ANALOG_GAIN	0x0204
#define IMX708_REG_DIGITAL_GAIN	0x020e
#define IMX708_REG_FRAME_LENGTH	0x0340
#define IMX708_REG_LONG_EXP_SHIFT 0x3100
#define IMX708_REG_TEST_PATTERN	0x0600
#define IMX708_REG_SPC_GAINS_L	0x7b10
#define IMX708_REG_SPC_GAINS_R	0x7c00
#define IMX708_LPF_INTENSITY_EN	0xc428
#define IMX708_LPF_INTENSITY	0xc429

#define IMX708_EXPOSURE_OFFSET	48	/* exposure <= frame_length - 48 */
#define IMX708_FRAME_LENGTH_MAX	0xffff
#define IMX708_ANA_GAIN_MIN	112	/* 1024 / (1024 - code) => 1.12x */
#define IMX708_ANA_GAIN_MAX	960	/* 16x */

static int flip;
module_param(flip, int, 0444);
MODULE_PARM_DESC(flip, "Sensor readout flip: 0 none, 1 h, 2 v, 3 h+v (rotate 180)");

static int qbc_adjust = 2;
module_param(qbc_adjust, int, 0644);
MODULE_PARM_DESC(qbc_adjust, "Quad Bayer broken line correction strength [0,2-5] (full-res mode)");

static int test_pattern;
module_param(test_pattern, int, 0644);
MODULE_PARM_DESC(test_pattern, "0 off, 1 solid, 2 colour bars, 3 grey bars, 4 PN9");

/* Per-mode data that struct sensor_win_size has no field for. */
struct imx708_mode_ext {
	struct regval_list *regs;
	int regs_size;
	unsigned int exp_min;	/* lines */
	unsigned int exp_step;	/* lines */
	bool remosaic;
};

enum { MODE_FULL, MODE_BIN, MODE_BIN_CROP };

static const struct imx708_mode_ext mode_ext[] = {
	[MODE_FULL] = { imx708_4608x2592_regs, ARRAY_SIZE(imx708_4608x2592_regs), 8, 1, true },
	[MODE_BIN] = { imx708_2304x1296_regs, ARRAY_SIZE(imx708_2304x1296_regs), 4, 2, false },
	[MODE_BIN_CROP] = { imx708_1536x864_regs, ARRAY_SIZE(imx708_1536x864_regs), 4, 2, false },
};

/* The sensor's common registers are lost on power-off. */
static bool common_regs_written[SENSOR_NUM];

static struct regval_list sensor_fmt_raw[] = {
};

/*
 * Timing (from the RPi driver):
 *   full  4608x2592  line_length 15648  pixel_rate 595.2 MHz
 *   bin   2304x1296  line_length  7824  pixel_rate 585.6 MHz
 *   crop  1536x864   line_length  5216  pixel_rate 566.4 MHz
 * fps = pixel_rate / (line_length * frame_length); vts = frame_length.
 * fps_fixed is the frame rate VIN matches against the requested one.
 */
#define WIN(w, h, hts_, vts_, pclk_, fps_, bin_, mode_)		\
	{							\
		.width = (w), .height = (h),			\
		.hoffset = 0, .voffset = 0,			\
		.hts = (hts_), .vts = (vts_),			\
		.pclk = (pclk_),				\
		.mipi_bps = 900 * 1000 * 1000,			\
		.fps_fixed = (fps_),				\
		.bin_factor = (bin_),				\
		.intg_min = 8 << 4,				\
		.intg_max = ((vts_) - IMX708_EXPOSURE_OFFSET) << 4, \
		.gain_min = 1 << 4,				\
		.gain_max = 64 << 4,				\
		.regs = (void *)&mode_ext[mode_],		\
		.regs_size = 0,					\
		.set_size = NULL,				\
	}

static struct sensor_win_size sensor_win_sizes[] = {
	WIN(2304, 1296, 7824, 2495, 585600000, 30, 2, MODE_BIN),
	WIN(2304, 1296, 7824, 1336, 585600000, 56, 2, MODE_BIN),
	WIN(1536, 864, 5216, 1810, 566400000, 60, 2, MODE_BIN_CROP),
	WIN(1536, 864, 5216, 905, 566400000, 120, 2, MODE_BIN_CROP),
	WIN(1536, 864, 5216, 3619, 566400000, 30, 2, MODE_BIN_CROP),
	WIN(4608, 2592, 15648, 2650, 595200000, 14, 1, MODE_FULL),
};
#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static const struct imx708_mode_ext *cur_mode(struct sensor_info *info)
{
	if (info->current_wins && info->current_wins->regs)
		return info->current_wins->regs;
	return &mode_ext[MODE_BIN];
}

static unsigned int cur_vts(struct sensor_info *info)
{
	return info->current_wins ? info->current_wins->vts : 2495;
}

static int write16(struct v4l2_subdev *sd, addr_type reg, unsigned int val)
{
	int ret = sensor_write(sd, reg, (val >> 8) & 0xff);

	return ret ? ret : sensor_write(sd, reg + 1, val & 0xff);
}

/* VIN gain is Q4 (16 = 1x). Analog up to 16x, the rest digital (Q8). */
static void gain_to_regs(int gain_q4, unsigned int *ana, unsigned int *dig)
{
	int code;

	if (gain_q4 < 16)
		gain_q4 = 16;
	if (gain_q4 <= 16 * 16) {
		code = 1024 - (1024 * 16) / gain_q4;
		*dig = 0x100;
	} else {
		code = IMX708_ANA_GAIN_MAX;
		*dig = gain_q4;		/* (gain / 16x) * 256 */
	}
	*ana = clamp(code, IMX708_ANA_GAIN_MIN, IMX708_ANA_GAIN_MAX);
	if (*dig > 0xffff)
		*dig = 0xffff;
}

static unsigned int exp_lines(struct sensor_info *info, unsigned int exp_q4)
{
	const struct imx708_mode_ext *m = cur_mode(info);
	unsigned int lines = exp_q4 >> 4;

	if (lines < m->exp_min)
		lines = m->exp_min;
	lines -= lines % m->exp_step;
	if (lines > IMX708_FRAME_LENGTH_MAX - IMX708_EXPOSURE_OFFSET)
		lines = IMX708_FRAME_LENGTH_MAX - IMX708_EXPOSURE_OFFSET;
	return lines;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd,
			     struct sensor_exp_gain *exp_gain)
{
	struct sensor_info *info = to_state(sd);
	unsigned int lines, frame_length, ana, dig;

	lines = exp_lines(info, exp_gain->exp_val);
	frame_length = max(cur_vts(info), lines + IMX708_EXPOSURE_OFFSET);
	gain_to_regs(exp_gain->gain_val, &ana, &dig);

	sensor_write(sd, IMX708_REG_HOLD, 0x01);
	write16(sd, IMX708_REG_FRAME_LENGTH, frame_length);
	write16(sd, IMX708_REG_EXPOSURE, lines);
	write16(sd, IMX708_REG_ANALOG_GAIN, ana);
	write16(sd, IMX708_REG_DIGITAL_GAIN, dig);
	sensor_write(sd, IMX708_REG_HOLD, 0x00);

	info->exp = exp_gain->exp_val;
	info->gain = exp_gain->gain_val;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	struct sensor_info *info = to_state(sd);
	unsigned int lines = exp_lines(info, exp_val);
	unsigned int frame_length = max(cur_vts(info), lines + IMX708_EXPOSURE_OFFSET);

	sensor_write(sd, IMX708_REG_HOLD, 0x01);
	write16(sd, IMX708_REG_FRAME_LENGTH, frame_length);
	write16(sd, IMX708_REG_EXPOSURE, lines);
	sensor_write(sd, IMX708_REG_HOLD, 0x00);
	info->exp = exp_val;
	return 0;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	*value = to_state(sd)->exp;
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);
	unsigned int ana, dig;

	gain_to_regs(gain_val, &ana, &dig);
	sensor_write(sd, IMX708_REG_HOLD, 0x01);
	write16(sd, IMX708_REG_ANALOG_GAIN, ana);
	write16(sd, IMX708_REG_DIGITAL_GAIN, dig);
	sensor_write(sd, IMX708_REG_HOLD, 0x00);
	info->gain = gain_val;
	return 0;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	*value = to_state(sd)->gain;
	return 0;
}

static int sensor_index(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return (client && !strcmp(client->name, SENSOR_NAME_2)) ? 1 : 0;
}

/*
 * Camera Module 3 has its own regulators and 24 MHz oscillator; the
 * connector's power-enable line (PWDN in the DT, PE6 on the Zero 3W) switches
 * them on and releases XCLR. Same sequence as the BSP imx219 driver.
 */
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	switch (on) {
	case STBY_ON:
		cci_lock(sd);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		cci_unlock(sd);
		vin_set_mclk(sd, OFF);
		break;
	case STBY_OFF:
		cci_lock(sd);
		vin_set_mclk_freq(sd, MCLK);
		vin_set_mclk(sd, ON);
		msleep(20);
		cci_unlock(sd);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		break;
	case PWR_ON:
		cci_lock(sd);
		vin_gpio_set_status(sd, PWDN, 1);
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(1000, 1200);
		vin_set_mclk_freq(sd, MCLK);
		vin_set_mclk(sd, ON);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		vin_set_pmu_channel(sd, IOVDD, ON);
		vin_set_pmu_channel(sd, AVDD, ON);
		vin_set_pmu_channel(sd, DVDD, ON);
		vin_set_pmu_channel(sd, AFVDD, ON);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		/* T7 (XCLR high -> standby exit) is 8 ms; regulators need ramp too */
		usleep_range(30000, 31000);
		cci_unlock(sd);
		common_regs_written[sensor_index(sd)] = false;
		break;
	case PWR_OFF:
		cci_lock(sd);
		vin_set_mclk(sd, OFF);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);
		vin_set_pmu_channel(sd, AFVDD, OFF);
		vin_set_pmu_channel(sd, DVDD, OFF);
		vin_set_pmu_channel(sd, AVDD, OFF);
		vin_set_pmu_channel(sd, IOVDD, OFF);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_set_status(sd, RESET, 0);
		vin_gpio_set_status(sd, PWDN, 0);
		cci_unlock(sd);
		common_regs_written[sensor_index(sd)] = false;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	switch (val) {
	case 0:
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(10000, 12000);
		break;
	case 1:
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(10000, 12000);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	data_type hi = 0, lo = 0;
	int ret;

	ret = sensor_read(sd, IMX708_REG_CHIP_ID, &hi);
	ret |= sensor_read(sd, IMX708_REG_CHIP_ID + 1, &lo);
	if (ret) {
		sensor_err("imx708: no i2c answer at 0x%02x\n", I2C_ADDR >> 1);
		return -ENODEV;
	}
	if (((hi << 8) | lo) != IMX708_CHIP_ID) {
		sensor_err("imx708: chip id 0x%02x%02x, expected 0x0708\n", hi, lo);
		return -ENODEV;
	}
	sensor_print("find the sony IMX708 (Camera Module 3)\n");
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	struct sensor_info *info = to_state(sd);
	int ret;

	ret = sensor_detect(sd);
	if (ret) {
		sensor_err("chip found is not an target chip.\n");
		return ret;
	}

	info->focus_status = 0;
	info->low_speed = 0;
	info->width = 2304;
	info->height = 1296;
	info->hflip = flip & 1;
	info->vflip = (flip >> 1) & 1;
	info->gain = 0;
	info->tpf.numerator = 1;
	info->tpf.denominator = 30;
	info->preview_first_flag = 1;
	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sensor_info *info = to_state(sd);
	int ret = 0;

	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		if (info->current_wins) {
			memcpy(arg, info->current_wins, sizeof(*info->current_wins));
		} else {
			sensor_err("empty wins!\n");
			ret = -1;
		}
		break;
	case SET_FPS:
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, (struct sensor_config *)arg);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

/* Bayer order follows the readout flip (no shift compensation on IMX708). */
static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "Raw RGB Bayer",
		.mbus_code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.regs = sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp = 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

static const u32 flip_codes[4] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,	/* none */
	MEDIA_BUS_FMT_SGRBG10_1X10,	/* h */
	MEDIA_BUS_FMT_SGBRG10_1X10,	/* v */
	MEDIA_BUS_FMT_SBGGR10_1X10,	/* h+v */
};

static int write_common_regs(struct v4l2_subdev *sd)
{
	data_type val = 0;
	int i, ret;

	ret = sensor_write_array(sd, imx708_common_regs,
				 ARRAY_SIZE(imx708_common_regs));
	if (ret < 0)
		return ret;

	/* PDAF pixel gain correction, as in the RPi driver */
	if (sensor_read(sd, IMX708_REG_SPC_GAINS_L, &val) == 0 && val == 0x40) {
		for (i = 0; i < 54; i++)
			sensor_write(sd, IMX708_REG_SPC_GAINS_L + i,
				     imx708_pdaf_gains[0][i % 9]);
		for (i = 0; i < 54; i++)
			sensor_write(sd, IMX708_REG_SPC_GAINS_R + i,
				     imx708_pdaf_gains[1][i % 9]);
	}
	return 0;
}

static int sensor_reg_init(struct sensor_info *info)
{
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_win_size *wsize = info->current_wins;
	const struct imx708_mode_ext *m = wsize->regs;
	int idx = sensor_index(sd);
	unsigned int ana, dig;
	int ret;

	if (!common_regs_written[idx]) {
		ret = write_common_regs(sd);
		if (ret < 0) {
			sensor_err("imx708: common regs failed\n");
			return ret;
		}
		common_regs_written[idx] = true;
	}

	ret = sensor_write_array(sd, m->regs, m->regs_size);
	if (ret < 0) {
		sensor_err("imx708: mode regs failed\n");
		return ret;
	}
	sensor_write_array(sd, imx708_link_450mhz_regs,
			   ARRAY_SIZE(imx708_link_450mhz_regs));

	if (m->remosaic && qbc_adjust > 0) {
		sensor_write(sd, IMX708_LPF_INTENSITY, qbc_adjust);
		sensor_write(sd, IMX708_LPF_INTENSITY_EN, 0x00);
	} else {
		sensor_write(sd, IMX708_LPF_INTENSITY_EN, 0x01);
	}

	sensor_write(sd, IMX708_REG_ORIENTATION, flip & 3);
	write16(sd, IMX708_REG_TEST_PATTERN,
		(test_pattern >= 0 && test_pattern <= 4) ? test_pattern : 0);

	/* frame length for the requested fps; mode tables carry the minimum */
	write16(sd, IMX708_REG_FRAME_LENGTH, wsize->vts);
	sensor_write(sd, IMX708_REG_LONG_EXP_SHIFT, 0);

	/* sane start values until the ISP's AE takes over: ~10 ms, 1x */
	if (info->exp == 0)
		info->exp = min(wsize->vts - IMX708_EXPOSURE_OFFSET,
				(unsigned int)(10ULL * wsize->pclk / 1000 / wsize->hts)) << 4;
	if (info->gain == 0)
		info->gain = 16;
	write16(sd, IMX708_REG_EXPOSURE, exp_lines(info, info->exp));
	gain_to_regs(info->gain, &ana, &dig);
	write16(sd, IMX708_REG_ANALOG_GAIN, ana);
	write16(sd, IMX708_REG_DIGITAL_GAIN, dig);

	info->width = wsize->width;
	info->height = wsize->height;

	sensor_print("imx708 mode %dx%d@%d vts %d\n", wsize->width,
		     wsize->height, wsize->fps_fixed, wsize->vts);

	return sensor_write(sd, IMX708_REG_MODE_SELECT, 0x01);
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);

	sensor_print("%s on = %d, %d*%d %x\n", __func__, enable,
		     info->current_wins->width, info->current_wins->height,
		     info->fmt->mbus_code);

	if (!enable)
		return sensor_write(sd, IMX708_REG_MODE_SELECT, 0x00);
	return sensor_reg_init(info);
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	cfg->bus.mipi_csi2.num_data_lanes = 0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;
#else
	cfg->flags = 0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;
#endif
	return 0;
}

static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->val);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_s_stream,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
	.get_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

static struct cci_driver cci_drv[] = {
	{
		.name = SENSOR_NAME,
		.addr_width = CCI_BITS_16,
		.data_width = CCI_BITS_8,
	}, {
		.name = SENSOR_NAME_2,
		.addr_width = CCI_BITS_16,
		.data_width = CCI_BITS_8,
	}
};

static const struct v4l2_ctrl_config sensor_custom_ctrls[] = {
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_FRAME_RATE,
		.name = "frame rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 14,
		.max = 120,
		.step = 1,
		.def = 30,
	},
};

static int sensor_init_controls(struct v4l2_subdev *sd, const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int i, ret = 0;

	v4l2_ctrl_handler_init(handler, 2 + ARRAY_SIZE(sensor_custom_ctrls));

	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 0,
				 65536 * 16, 1, 0);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 * 16,
			  64 * 16, 1, 1 * 16);
	for (i = 0; i < ARRAY_SIZE(sensor_custom_ctrls); i++)
		v4l2_ctrl_new_custom(handler, &sensor_custom_ctrls[i], NULL);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}
	sd->ctrl_handler = handler;
	return ret;
}

static int sensor_dev_id;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
static int sensor_probe(struct i2c_client *client)
#else
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
#endif
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
	int i;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	sd = &info->sd;

	if (client) {
		for (i = 0; i < SENSOR_NUM; i++)
			if (!strcmp(cci_drv[i].name, client->name))
				break;
		cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv[i]);
	} else {
		cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv[sensor_dev_id++]);
	}

	sensor_init_controls(sd, &sensor_ctrl_ops);
	mutex_init(&info->lock);

#if IS_ENABLED(CONFIG_SAME_I2C)
	info->sensor_i2c_addr = I2C_ADDR >> 1;
#endif
	sensor_formats[0].mbus_code = flip_codes[flip & 3];
	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;
	info->af_first_flag = 1;
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int sensor_remove(struct i2c_client *client)
#else
static void sensor_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd;
	int i;

	if (client) {
		for (i = 0; i < SENSOR_NUM; i++)
			if (!strcmp(cci_drv[i].name, client->name))
				break;
		sd = cci_dev_remove_helper(client, &cci_drv[i]);
	} else {
		sd = cci_dev_remove_helper(client, &cci_drv[sensor_dev_id++]);
	}
	kfree(to_state(sd));
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{}
};

static const struct i2c_device_id sensor_id_2[] = {
	{SENSOR_NAME_2, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver[] = {
	{
		.driver = {
			.owner = THIS_MODULE,
			.name = SENSOR_NAME,
		},
		.probe = sensor_probe,
		.remove = sensor_remove,
		.id_table = sensor_id,
	}, {
		.driver = {
			.owner = THIS_MODULE,
			.name = SENSOR_NAME_2,
		},
		.probe = sensor_probe,
		.remove = sensor_remove,
		.id_table = sensor_id_2,
	},
};

static __init int init_sensor(void)
{
	int i, ret = 0;

	sensor_dev_id = 0;
	for (i = 0; i < SENSOR_NUM; i++)
		ret = cci_dev_init_helper(&sensor_driver[i]);
	return ret;
}

static __exit void exit_sensor(void)
{
	int i;

	sensor_dev_id = 0;
	for (i = 0; i < SENSOR_NUM; i++)
		cci_dev_exit_helper(&sensor_driver[i]);
}

VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);

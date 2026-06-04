// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 Avnet EMG GmbH 
 * Copyright (C) 2022 - 2025 Allied Vision Technologies GmbH
 */

/*
 * Allied Vision CSI2 Camera
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

//#define DEBUG
#define ENABLE_STEPWISE_IMAGE_SIZE
#define AVT_MAX_FORMAT_ENTRIES 40

#include <linux/kernel.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
#include <asm/unaligned.h>
#else 
#include <linux/unaligned.h>
#endif

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/regmap.h>

//#include <linux/sched/task.h>
#include <linux/workqueue.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-rect.h>
#include <linux/lcm.h>
#include <linux/crc32.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>


#include "avt-mipi-csi2.h"

#include "avt-csi2.h"

#define AVT_DBG_LVL 2

#ifdef DEBUG
static int debug = AVT_DBG_LVL;
#else
static int debug = 0;
#endif
module_param(debug, int, 0644); /* S_IRUGO */
MODULE_PARM_DESC(debug, "Debug level (0-2)");

static int add_wait_time_ms = 2000;
module_param(add_wait_time_ms, int, 0600);

static bool power_save_reset_controls = false;
module_param(power_save_reset_controls, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(power_save_reset_controls,
		 "Reset controls on return from power save mode");


#define avt_dbg(sd, fmt, args...)                       \
	v4l2_dbg(AVT_DBG_LVL, debug, sd, "%s[%d]: " fmt "", \
			 __func__, __LINE__, ##args)

#define avt_err(sd, fmt, args...) \
	v4l2_err(sd, "%s[%d]: " fmt "", __func__, __LINE__, ##args)

#define avt_warn(sd, fmt, args...) \
	v4l2_warn(sd, "%s[%d]: " fmt "", __func__, __LINE__, ##args)

#define avt_info(sd, fmt, args...) \
	v4l2_info(sd, "%s[%d]: " fmt "", __func__, __LINE__, ##args)

#define adev_info(dev, fmt, args...) \
	dev_info(dev, "%s[%d]: " fmt "", __func__, __LINE__, ##args)

struct avt_val64
{
	union
	{
		__s8 s8[8];
		__s16 s16[4];
		__s32 s32[2];
		__s64 s64;
		__u8 u8[8];
		__u16 u16[4];
		__u32 u32[2];
		__u64 u64;
	};
} __attribute__((packed));

#define BCRM_VERSION(a, b) \
	((((a) << 16) & BCRM_VERSION_MAJOR) + (b & BCRM_VERSION_MINOR))

#define BCRM_WAIT_HANDSHAKE_TIMEOUT_MS 	3000

#define MODE_SWITCH_TIMEOUT_US		5 * USEC_PER_SEC
#define MODE_SWTICH_POLL_INTERVAL_US	10 * USEC_PER_MSEC 

#define BOOT_TIMEOUT_US			10 * USEC_PER_SEC
#define BOOT_POLL_INTERVAL_US		500 * USEC_PER_MSEC

//Define formats for GenICam for CSI2, if they not exist
#ifndef V4L2_PIX_FMT_CUSTOM
#define V4L2_PIX_FMT_CUSTOM    v4l2_fourcc('T', 'P', '3', '1') /* 0x31 mipi datatype  */
#endif

#ifndef MEDIA_BUS_FMT_CUSTOM
#define MEDIA_BUS_FMT_CUSTOM        		0x5002
#endif

#define AVT_BINNING_MODE_FLAG_AVERAGE 		0b01
#define AVT_BINNING_MODE_FLAG_SUM 		0b10

#define LINE_OFFSET		8

#define LINE_DIR_INPUT		0
#define LINE_DIR_OUTPUT(x)	(BIT(0) << (x *LINE_OFFSET))

#define LINE_INVERT(x)		(BIT(1) << (x *LINE_OFFSET))

#define LINE_MASK(x) \
	(LINE_DIR_OUTPUT(x) | LINE_INVERT(x))


#define avt_get_mode_fmt(camera) (&camera->fmt[camera->mode])

#define test_feature_inq(c, inq) \
	(!!(camera->feature_inquiry_reg.value & BCRM_FEATURE_INQ_ ## inq))


enum avt_binning_type {
	NONE = -1,
	DIGITAL,
	SENSOR,
};

enum avt_reset_type {
	RESET_TYPE_SOFT,
	RESET_TYPE_HARD
};

struct avt_binning_setting {
	int inq;
	u8 sel;
	u32 hfact;
	u32 vfact;
	enum avt_binning_type type;
};

struct avt_mode_info
{
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
};


static const long binning_modes_enabled[AVT_BINNING_TYPE_CNT] = {
	[DIGITAL] = AVT_BINNING_MODE_FLAG_AVERAGE | AVT_BINNING_MODE_FLAG_SUM,
	[SENSOR] = AVT_BINNING_MODE_FLAG_SUM,
};

static const char * binning_type_str[AVT_BINNING_TYPE_CNT] = {
	[DIGITAL] = "Digital",
	[SENSOR] = "Sensor",
};

static const struct avt_binning_setting avt_binning_settings[] = {
	{
		.inq = -1,
		.sel = 0,
		.vfact = 1,
		.hfact = 1,
		.type = NONE,
	}, {
		.inq = 0,
		.sel = 1,
		.vfact = 2,
		.hfact = 2,
		.type = DIGITAL,
	}, {
		.inq = 1,
		.sel = 2,
		.vfact = 3,
		.hfact = 3,
		.type = DIGITAL,
	}, {
		.inq = 2,
		.sel = 3,
		.vfact = 4,
		.hfact = 4,
		.type = DIGITAL,
	}, {
		.inq = 3,
		.sel = 4,
		.vfact = 5,
		.hfact = 5,
		.type = DIGITAL,
	}, {
		.inq = 4,
		.sel = 5,
		.vfact = 6,
		.hfact = 6,
		.type = DIGITAL,
	}, {
		.inq = 5,
		.sel = 6,
		.vfact = 7,
		.hfact = 7,
		.type = DIGITAL,
	}, {
		.inq = 6,
		.sel = 7,
		.vfact = 8,
		.hfact = 8,
		.type = DIGITAL,
	}, {
		.inq = 7,
		.sel = 8,
		.vfact = 2,
		.hfact = 2,
		.type = SENSOR,
	}, {
		.inq = 8,
		.sel = 9,
		.vfact = 4,
		.hfact = 4,
		.type = SENSOR,
	},
};

static const size_t avt_binning_setting_cnt = ARRAY_SIZE(avt_binning_settings);

static int bcrm_write(struct avt_dev *camera, u16 reg, u64 val, size_t len);

static int avt_do_softreset(struct avt_dev *camera);
static int avt_reinit(struct avt_dev *camera);
static void avt_dphy_reset(struct avt_dev *camera, bool bResetPhy);

static void avt_ctrl_changed(struct avt_dev *camera, const struct v4l2_ctrl * const ctrl);
static struct v4l2_ctrl* avt_ctrl_find(struct avt_dev *camera,u32 id);
static int avt_write_media_bus_format(struct avt_dev *camera, int code);
static int avt_get_camera_capabilities(struct v4l2_subdev *sd);
static int avt_update_format(struct avt_dev *camera, const struct v4l2_rect *roi, const struct avt_binning_info *info);
static int __set_crop(struct avt_dev *camera, struct v4l2_rect *rect,
	struct v4l2_mbus_framefmt *frmfmt, struct v4l2_rect *crop,
	unsigned int which);

#define DUMP_BCRM_REG8(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_8)
#define DUMP_BCRM_REG16(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_16)
#define DUMP_BCRM_REG32(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_32)
#define DUMP_BCRM_REG64(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_64)

static void dump_bcrm_reg(struct i2c_client *client, u16 nOffset, const char *pRegName, int regsize);

static inline struct avt_dev* to_avt_dev(struct v4l2_subdev *sd)
{
#ifdef NVIDIA
	return container_of(sd, struct avt_dev, s_data.subdev);
#else
	return container_of(sd, struct avt_dev, subdev);
#endif
}

static inline struct avt_dev* client_to_avt_dev(struct i2c_client *client)
{
	return to_avt_dev(i2c_get_clientdata(client));
}

static inline struct v4l2_subdev* get_sd(struct avt_dev *priv)
{
#ifdef NVIDIA
	return &priv->s_data.subdev;
#else
	return &priv->subdev;
#endif // #ifdef NVIDIA
}


static inline void set_flag(u32 *pval, u32 mask, int set)
{
	if (set) 
		*pval |= mask;
	else 
		*pval &= ~mask;
}

static ssize_t avt_read_raw(struct avt_dev *camera, u16 reg,
	u8 *buf, size_t len)
{
	int ret;

	dev_dbg(&camera->i2c_client->dev, "read raw reg %x len %lu", reg, len);

	ret = regmap_bulk_read(camera->regmap, reg, buf, len);
	if (ret) {
		return ret;
	}

	return len;
}

static ssize_t avt_write_raw(struct avt_dev *camera, u16 reg,
	const u8 *buf, size_t len)
{
	int ret;

	dev_dbg(&camera->i2c_client->dev, "write raw reg %x len %lu", reg, len);

	ret = regmap_bulk_write(camera->regmap, reg, buf, len);
	if (ret) {
		return ret;
	}

	return len;
}

static ssize_t avt_read(struct avt_dev *camera, u16 reg, void *val, size_t len)
{
	u8 tmp[8];
	int ret;

	ret = avt_read_raw(camera, reg, tmp, len);
	if (ret < 0)
		goto out;

	switch (len)
	{
	case 1:
		*((u8*)val) = tmp[0];
		break;
	case 2:
		*((u16*)val) = get_unaligned_be16(tmp);
		break;
	case 4:
		*((u32*)val) = get_unaligned_be32(tmp);
		break;
	case 8:
		*((u64*)val) = get_unaligned_be64(tmp);
		break;
	default:
		break;
	}

	// TODO: For compatibility reason, check if can be removed
	ret = 0;

out:
	return ret;
}

static inline ssize_t avt_read8(struct avt_dev *camera, u16 reg, u8 *val)
{
	return avt_read(camera, reg, (u8*)val, sizeof(*val));
}

static inline ssize_t avt_read16(struct avt_dev *camera, u16 reg, u16 *val)
{
	return avt_read(camera, reg, (u8*)val, sizeof(*val));
}

static inline ssize_t avt_read32(struct avt_dev *camera, u16 reg, u32 *val)
{
	return avt_read(camera, reg, (u8*)val, sizeof(*val));
}

static inline ssize_t avt_read64(struct avt_dev *camera, u16 reg, u64 *val) 
{
	return avt_read(camera, reg, (u8*)val, sizeof(*val));
}

static ssize_t avt_write(struct avt_dev *camera, u16 reg, u64 val, size_t len)
{
	struct device *dev = &camera->i2c_client->dev;
	u8 buf[8];
	int ret;

	switch (len)
	{
	// 8-bit register
	case 1:
		buf[0] = val;
		break;
	// 16-bit register
	case 2:
		put_unaligned_be16(val, buf);
		break;
	// 32-bit register
	case 4:
		put_unaligned_be32(val, buf);
		break;
	// 64-bit register
	case 8:
		put_unaligned_be64(val, buf);
		break;
	default:
		dev_err(dev, "%s: Invalid data size!\n", __func__);
		return -EINVAL;
	}

	ret = avt_write_raw(camera, reg, buf, len);
	// TODO: For compatibility reason, check if can be removed 
	if (ret < 0)
		return ret;

	return 0;
}

static ssize_t avt_write8(struct avt_dev *camera, u16 reg, u8 val)
{
	return avt_write(camera, reg, val, sizeof(val));
}

static inline u16 get_bcrm_addr(struct avt_dev *camera,u16 reg)
{
	return camera->cci_reg.reg.bcrm_addr + reg;
}

static inline int bcrm_read(struct avt_dev *camera, u16 reg, u8 *val, size_t len)
{
	WARN_ON(camera->mode != AVT_BCRM_MODE);

	return avt_read(camera, get_bcrm_addr(camera, reg), val, len);
}

static inline int bcrm_read8(struct avt_dev *camera,u16 reg,u8 *val)
{
	return bcrm_read(camera, reg, (u8*)val, sizeof(*val));
}

static inline int bcrm_read16(struct avt_dev *camera,u16 reg,u16 *val)
{
	return bcrm_read(camera, reg, (u8*)val, sizeof(*val));
}

static inline int bcrm_read32(struct avt_dev *camera,u16 reg,u32 *val)
{
	return bcrm_read(camera, reg, (u8*)val, sizeof(*val));
}

static inline int bcrm_read64(struct avt_dev *camera,u16 reg,u64 *val)
{
	return bcrm_read(camera, reg, (u8*)val, sizeof(*val));
}

static inline int bcrm_write8(struct avt_dev *camera, u16 reg, u8 val)
{
	return bcrm_write(camera, reg, val, sizeof(val));
}

static inline int bcrm_write16(struct avt_dev *camera, u16 reg, u16 val)
{
	return bcrm_write(camera, reg, val, sizeof(val));
}

static inline int bcrm_write32(struct avt_dev *camera, u16 reg, u32 val)
{
	return bcrm_write(camera, reg, val, sizeof(val));
}

static inline int bcrm_write64(struct avt_dev *camera, u16 reg, u64 val)
{
	return bcrm_write(camera, reg, val, sizeof(val));
}

static int avt_change_mode(struct avt_dev *camera, u8 req_mode)
{	
	int ret;
	u8 cur_mode;

	if (req_mode == camera->mode)
		return 0;

	ret = avt_write(camera, GENCP_CHANGEMODE_8W, req_mode, AV_CAM_DATA_SIZE_8);
	if (ret < 0)
		goto out;


	ret = read_poll_timeout(avt_read8, ret, cur_mode == req_mode,
				MODE_SWTICH_POLL_INTERVAL_US,
				MODE_SWITCH_TIMEOUT_US, true,
				camera, GENCP_CURRENTMODE_8R, &cur_mode);
	if (ret < 0)
		goto out;

	camera->mode = req_mode;

	if (req_mode == AVT_BCRM_MODE) {
		const int mbus_code = avt_get_mode_fmt(camera)->code;
		ret = avt_write_media_bus_format(camera, mbus_code);

		if (ret < 0) {
			avt_err(get_sd(camera),"Failed to set pixelformat!");
		}
	}

out:
	return ret;
}

static void bcrm_dump(struct i2c_client *client)
{
	/* Dump all BCRM registers */

	DUMP_BCRM_REG32(client, BCRM_VERSION_32R);
	DUMP_BCRM_REG64(client, BCRM_FEATURE_INQUIRY_64R);
	DUMP_BCRM_REG64(client, BCRM_DEVICE_FIRMWARE_VERSION_64R);
	DUMP_BCRM_REG8(client, BCRM_WRITE_HANDSHAKE_8RW);

	/* Streaming Control Registers */
	DUMP_BCRM_REG8(client, BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R);
	DUMP_BCRM_REG8(client, BCRM_CSI2_LANE_COUNT_8RW);
	DUMP_BCRM_REG32(client, BCRM_CSI2_CLOCK_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_CSI2_CLOCK_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_CSI2_CLOCK_32RW);
	DUMP_BCRM_REG32(client, BCRM_BUFFER_SIZE_32R);
	DUMP_BCRM_REG32(client, BCRM_PHY_RESET_8RW);
	DUMP_BCRM_REG32(client, BCRM_STREAM_ON_DELAY_32RW);

	DUMP_BCRM_REG32(client, BCRM_IPU_X_MIN_32W);
	DUMP_BCRM_REG32(client, BCRM_IPU_X_MAX_32W);
	DUMP_BCRM_REG32(client, BCRM_IPU_X_INC_32W);
	DUMP_BCRM_REG32(client, BCRM_IPU_Y_MIN_32W);
	DUMP_BCRM_REG32(client, BCRM_IPU_Y_MAX_32W);
	DUMP_BCRM_REG32(client, BCRM_IPU_Y_INC_32W);
	DUMP_BCRM_REG32(client, BCRM_IPU_X_32R);
	DUMP_BCRM_REG32(client, BCRM_IPU_Y_32R);

	/* Acquisition Control Registers */
	DUMP_BCRM_REG8(client, BCRM_ACQUISITION_START_8RW);
	DUMP_BCRM_REG8(client, BCRM_ACQUISITION_STOP_8RW);
	DUMP_BCRM_REG8(client, BCRM_ACQUISITION_ABORT_8RW);
	DUMP_BCRM_REG8(client, BCRM_ACQUISITION_STATUS_8R);
	DUMP_BCRM_REG64(client, BCRM_ACQUISITION_FRAME_RATE_64RW);
	DUMP_BCRM_REG64(client, BCRM_ACQUISITION_FRAME_RATE_MIN_64R);
	DUMP_BCRM_REG64(client, BCRM_ACQUISITION_FRAME_RATE_MAX_64R);
	DUMP_BCRM_REG64(client, BCRM_ACQUISITION_FRAME_RATE_INC_64R);
	DUMP_BCRM_REG8(client, BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW);

	DUMP_BCRM_REG8(client, BCRM_FRAME_START_TRIGGER_MODE_8RW);
	DUMP_BCRM_REG8(client, BCRM_FRAME_START_TRIGGER_SOURCE_8RW);
	DUMP_BCRM_REG8(client, BCRM_FRAME_START_TRIGGER_ACTIVATION_8RW);
	DUMP_BCRM_REG8(client, BCRM_FRAME_START_TRIGGER_SOFTWARE_8W);
	DUMP_BCRM_REG32(client, BCRM_FRAME_START_TRIGGER_DELAY_32RW);

	DUMP_BCRM_REG8(client, BCRM_EXPOSURE_ACTIVE_LINE_MODE_8RW);
	DUMP_BCRM_REG8(client, BCRM_EXPOSURE_ACTIVE_LINE_SELECTOR_8RW);
	DUMP_BCRM_REG32(client, BCRM_LINE_CONFIGURATION_32RW);

	/* Image Format Control Registers */
	DUMP_BCRM_REG32(client, BCRM_IMG_WIDTH_32RW);
	DUMP_BCRM_REG32(client, BCRM_IMG_WIDTH_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_IMG_WIDTH_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_IMG_WIDTH_INC_32R);

	DUMP_BCRM_REG32(client, BCRM_IMG_HEIGHT_32RW);
	DUMP_BCRM_REG32(client, BCRM_IMG_HEIGHT_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_IMG_HEIGHT_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_IMG_HEIGHT_INC_32R);

	DUMP_BCRM_REG32(client, BCRM_IMG_OFFSET_X_32RW);
	DUMP_BCRM_REG32(client, BCRM_IMG_OFFSET_X_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_IMG_OFFSET_X_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_IMG_OFFSET_X_INC_32R);

	DUMP_BCRM_REG32(client, BCRM_IMG_OFFSET_Y_32RW);
	DUMP_BCRM_REG32(client, BCRM_IMG_OFFSET_Y_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_IMG_OFFSET_Y_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_IMG_OFFSET_Y_INC_32R);

	DUMP_BCRM_REG32(client, BCRM_IMG_MIPI_DATA_FORMAT_32RW);
	DUMP_BCRM_REG64(client, BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R);

	DUMP_BCRM_REG8(client, BCRM_IMG_BAYER_PATTERN_INQUIRY_8R);
	DUMP_BCRM_REG8(client, BCRM_IMG_BAYER_PATTERN_8RW);

	DUMP_BCRM_REG8(client, BCRM_IMG_REVERSE_X_8RW);
	DUMP_BCRM_REG8(client, BCRM_IMG_REVERSE_Y_8RW);

	DUMP_BCRM_REG32(client, BCRM_SENSOR_WIDTH_32R);
	DUMP_BCRM_REG32(client, BCRM_SENSOR_HEIGHT_32R);

	DUMP_BCRM_REG32(client, BCRM_WIDTH_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_HEIGHT_MAX_32R);

	/* Brightness Control Registers */
	DUMP_BCRM_REG64(client, BCRM_EXPOSURE_TIME_64RW);
	DUMP_BCRM_REG64(client, BCRM_EXPOSURE_TIME_MIN_64R);
	DUMP_BCRM_REG64(client, BCRM_EXPOSURE_TIME_MAX_64R);
	DUMP_BCRM_REG64(client, BCRM_EXPOSURE_TIME_64RW);
	DUMP_BCRM_REG8(client, BCRM_EXPOSURE_AUTO_8RW);

	DUMP_BCRM_REG8(client, BCRM_INTENSITY_AUTO_PRECEDENCE_8RW);
	DUMP_BCRM_REG32(client, BCRM_INTENSITY_AUTO_PRECEDENCE_VALUE_32RW);
	DUMP_BCRM_REG32(client, BCRM_INTENSITY_AUTO_PRECEDENCE_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_INTENSITY_AUTO_PRECEDENCE_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_INTENSITY_AUTO_PRECEDENCE_INC_32R);

	DUMP_BCRM_REG32(client, BCRM_BLACK_LEVEL_32RW);
	DUMP_BCRM_REG32(client, BCRM_BLACK_LEVEL_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_BLACK_LEVEL_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_BLACK_LEVEL_INC_32R);

	DUMP_BCRM_REG64(client, BCRM_GAIN_64RW);
	DUMP_BCRM_REG64(client, BCRM_GAIN_MIN_64R);
	DUMP_BCRM_REG64(client, BCRM_GAIN_MAX_64R);
	DUMP_BCRM_REG64(client, BCRM_GAIN_INC_64R);
	DUMP_BCRM_REG8(client, BCRM_GAIN_AUTO_8RW);

	DUMP_BCRM_REG64(client, BCRM_GAMMA_64RW);
	DUMP_BCRM_REG64(client, BCRM_GAMMA_MIN_64R);
	DUMP_BCRM_REG64(client, BCRM_GAMMA_MAX_64R);
	DUMP_BCRM_REG64(client, BCRM_GAMMA_INC_64R);

	DUMP_BCRM_REG32(client, BCRM_CONTRAST_VALUE_32RW);
	DUMP_BCRM_REG32(client, BCRM_CONTRAST_VALUE_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_CONTRAST_VALUE_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_CONTRAST_VALUE_INC_32R);

	/* Color Management Registers */
	DUMP_BCRM_REG32(client, BCRM_SATURATION_32RW);
	DUMP_BCRM_REG32(client, BCRM_SATURATION_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_SATURATION_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_SATURATION_INC_32R);

	DUMP_BCRM_REG32(client, BCRM_HUE_32RW);
	DUMP_BCRM_REG32(client, BCRM_HUE_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_HUE_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_HUE_INC_32R);

	DUMP_BCRM_REG64(client, BCRM_ALL_BALANCE_RATIO_64RW);
	DUMP_BCRM_REG64(client, BCRM_ALL_BALANCE_RATIO_MIN_64R);
	DUMP_BCRM_REG64(client, BCRM_ALL_BALANCE_RATIO_MAX_64R);

	DUMP_BCRM_REG64(client, BCRM_RED_BALANCE_RATIO_64RW);
	DUMP_BCRM_REG64(client, BCRM_RED_BALANCE_RATIO_MIN_64R);
	DUMP_BCRM_REG64(client, BCRM_RED_BALANCE_RATIO_MAX_64R);
	DUMP_BCRM_REG64(client, BCRM_RED_BALANCE_RATIO_INC_64R);

	DUMP_BCRM_REG64(client, BCRM_GREEN_BALANCE_RATIO_64RW);
	DUMP_BCRM_REG64(client, BCRM_GREEN_BALANCE_RATIO_MIN_64R);
	DUMP_BCRM_REG64(client, BCRM_GREEN_BALANCE_RATIO_MAX_64R);
	DUMP_BCRM_REG64(client, BCRM_GREEN_BALANCE_RATIO_INC_64R);

	DUMP_BCRM_REG64(client, BCRM_BLUE_BALANCE_RATIO_64RW);
	DUMP_BCRM_REG64(client, BCRM_BLUE_BALANCE_RATIO_MIN_64R);
	DUMP_BCRM_REG64(client, BCRM_BLUE_BALANCE_RATIO_MAX_64R);
	DUMP_BCRM_REG64(client, BCRM_BLUE_BALANCE_RATIO_INC_64R);

	DUMP_BCRM_REG8(client, BCRM_WHITE_BALANCE_AUTO_8RW);

	/* Other Registers */
	DUMP_BCRM_REG32(client, BCRM_SHARPNESS_32RW);
	DUMP_BCRM_REG32(client, BCRM_SHARPNESS_MIN_32R);
	DUMP_BCRM_REG32(client, BCRM_SHARPNESS_MAX_32R);
	DUMP_BCRM_REG32(client, BCRM_SHARPNESS_INC_32R);

	DUMP_BCRM_REG32(client, BCRM_DEVICE_TEMPERATURE_32R);

	DUMP_BCRM_REG64(client, BCRM_EXPOSURE_AUTO_MIN_64RW);
	DUMP_BCRM_REG64(client, BCRM_EXPOSURE_AUTO_MAX_64RW);

	DUMP_BCRM_REG16(client, BCRM_BINNING_INQ_16R);
	DUMP_BCRM_REG16(client, BCRM_BINNING_INQ_16R);
}

static void dump_bcrm_reg(struct i2c_client *client, u16 nOffset, const char *pRegName, int regsize)
{
	struct avt_dev *camera = client_to_avt_dev(client);
	int status = 0;
	struct avt_val64 val64;

	CLEAR(val64);

	if (status >= 0)
		switch (regsize)
		{
		case AV_CAM_DATA_SIZE_8:
			bcrm_read8(camera, nOffset, &val64.u8[0]);
			avt_info(get_sd(camera), "%44s: %u (0x%x)", pRegName, val64.u8[0], val64.u8[0]);
			break;
		case AV_CAM_DATA_SIZE_16:
			bcrm_read16(camera, nOffset, &val64.u16[0]);
			avt_info(get_sd(camera), "%44s: %u (0x%08x)", pRegName, val64.u16[0], val64.u16[0]);
			break;
		case AV_CAM_DATA_SIZE_32:
			bcrm_read32(camera, nOffset, &val64.u32[0]);
			avt_info(get_sd(camera), "%44s: %u (0x%08x)", pRegName, val64.u32[0], val64.u32[0]);
			break;
		case AV_CAM_DATA_SIZE_64:
			bcrm_read64(camera, nOffset, &val64.u64);
			avt_info(get_sd(camera), "%44s: %llu (0x%016llx)", pRegName, val64.u64, val64.u64);
			break;
		}
	else
		avt_err(get_sd(camera), "%s: ERROR", pRegName);
}

static bool bcrm_get_write_handshake_availibility(struct i2c_client *client)
{
	struct avt_dev *camera = client_to_avt_dev(client);
	u8 value = 0;
	int status;

	if (!camera)
	{
		avt_err(get_sd(camera), "camera == NULL!!!\n");
		return -EINVAL;
	}
	/* check of camera supports write_done_handshake register */
	status = bcrm_read8(camera, BCRM_WRITE_HANDSHAKE_8RW, &value);

	if ((status >= 0) && (value & BCRM_HANDSHAKE_AVAILABLE_MASK))
	{
		v4l2_info(get_sd(camera), "BCRM write handshake supported!");
		return true;
	}
	else
	{
		v4l2_info(get_sd(camera), "BCRM write handshake NOT supported!");
		return false;
	}
}

static int read_cci_registers(struct i2c_client *client)
{
	struct avt_dev *camera = client_to_avt_dev(client);

	int ret = 0;
	uint32_t crc = 0;
	uint32_t crc_byte_count = 0;

	if (!camera)
	{
		avt_err(get_sd(camera), "camera == NULL!!!");
		return -EINVAL;
	}

	mutex_lock(&camera->lock);

	/*
	 * ToDO: Check against latest spec!!
	 * Avoid last 3 bytes read as its WRITE only register except
	 * CURRENT MODE REG
	 */
	/* Calculate byte per byte CRC from each reg up to the CRC reg */
	crc_byte_count =
		(uint32_t)((char *)&camera->cci_reg.reg.checksum - (char *)&camera->cci_reg);

	avt_dbg(get_sd(camera), "crc_byte_count: %d", crc_byte_count);
	avt_dbg(get_sd(camera), "0x%08X, 0x%08X",
			cci_cmd_tbl[CCI_REGISTER_LAYOUT_VERSION].address,
			cci_cmd_tbl[CHANGE_MODE].address);

	// read only until CHANGE_MODE because it's writeonly
	ret = avt_read_raw(camera, cci_cmd_tbl[CCI_REGISTER_LAYOUT_VERSION].address,
						   (char *)&camera->cci_reg, cci_cmd_tbl[CHANGE_MODE].address);

	avt_dbg(get_sd(camera), "regmap_bulk_read(camera->regmap8, cci_cmd_tbl[CCI_REGISTER_LAYOUT_VERSION].address ret %d\n", ret);

	avt_dbg(get_sd(camera), "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X - 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			camera->cci_reg.buf[0x00], camera->cci_reg.buf[0x01], camera->cci_reg.buf[0x02], camera->cci_reg.buf[0x03],
			camera->cci_reg.buf[0x04], camera->cci_reg.buf[0x05], camera->cci_reg.buf[0x06], camera->cci_reg.buf[0x07],
			camera->cci_reg.buf[0x08], camera->cci_reg.buf[0x09], camera->cci_reg.buf[0x0a], camera->cci_reg.buf[0x0b],
			camera->cci_reg.buf[0x0c], camera->cci_reg.buf[0x0d], camera->cci_reg.buf[0x0e], camera->cci_reg.buf[0x0f]);

	avt_dbg(get_sd(camera), "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X - 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			camera->cci_reg.buf[0x10], camera->cci_reg.buf[0x11], camera->cci_reg.buf[0x12], camera->cci_reg.buf[0x13],
			camera->cci_reg.buf[0x14], camera->cci_reg.buf[0x15], camera->cci_reg.buf[0x16], camera->cci_reg.buf[0x17],
			camera->cci_reg.buf[0x18], camera->cci_reg.buf[0x19], camera->cci_reg.buf[0x1a], camera->cci_reg.buf[0x1b],
			camera->cci_reg.buf[0x1c], camera->cci_reg.buf[0x1d], camera->cci_reg.buf[0x1e], camera->cci_reg.buf[0x1f]);

	avt_dbg(get_sd(camera), "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X - 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			camera->cci_reg.buf[0x20], camera->cci_reg.buf[0x21], camera->cci_reg.buf[0x22], camera->cci_reg.buf[0x23],
			camera->cci_reg.buf[0x24], camera->cci_reg.buf[0x25], camera->cci_reg.buf[0x26], camera->cci_reg.buf[0x27],
			camera->cci_reg.buf[0x28], camera->cci_reg.buf[0x29], camera->cci_reg.buf[0x1a], camera->cci_reg.buf[0x2b],
			camera->cci_reg.buf[0x2c], camera->cci_reg.buf[0x2d], camera->cci_reg.buf[0x2e], camera->cci_reg.buf[0x2f]);

	avt_dbg(get_sd(camera), "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X - 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			camera->cci_reg.buf[0x30], camera->cci_reg.buf[0x31], camera->cci_reg.buf[0x32], camera->cci_reg.buf[0x33],
			camera->cci_reg.buf[0x34], camera->cci_reg.buf[0x35], camera->cci_reg.buf[0x36], camera->cci_reg.buf[0x37],
			camera->cci_reg.buf[0x38], camera->cci_reg.buf[0x39], camera->cci_reg.buf[0x1a], camera->cci_reg.buf[0x3b],
			camera->cci_reg.buf[0x3c], camera->cci_reg.buf[0x3d], camera->cci_reg.buf[0x3e], camera->cci_reg.buf[0x3f]);

	if (ret < 0)
	{
		avt_err(get_sd(camera), "regmap_read failed (%d)\n", ret);
		goto err_out;
	}

	/* CRC calculation */
	crc = crc32(U32_MAX, &camera->cci_reg, crc_byte_count);

	/* Swap bytes if neccessary */
	cpu_to_be32s(&camera->cci_reg.reg.layout_version);

	cpu_to_be64s(&camera->cci_reg.reg.device_capabilities.value);
	cpu_to_be16s(&camera->cci_reg.reg.gcprm_address);
	cpu_to_be16s(&camera->cci_reg.reg.bcrm_addr);
	cpu_to_be32s(&camera->cci_reg.reg.checksum);

	/* Check the checksum of received with calculated. */
	if (crc != camera->cci_reg.reg.checksum)
	{
		avt_err(get_sd(camera), "wrong CCI CRC value! calculated = 0x%x, received = 0x%x\n",
				crc, camera->cci_reg.reg.checksum);
		ret = -EINVAL;
		goto err_out;
	}

	avt_dbg(get_sd(camera), "cci layout version: 0x%08X\ncci device capabilities: %llx\ncci device guid: %s\ncci gcprm_address: 0x%x\n",
			camera->cci_reg.reg.layout_version,
			camera->cci_reg.reg.device_capabilities.value,
			camera->cci_reg.reg.device_guid,
			camera->cci_reg.reg.gcprm_address);

	ret = 0;
err_out:

	mutex_unlock(&camera->lock);

	return ret;
}

static int read_gencp_registers(struct i2c_client *client)
{
	struct avt_dev *camera = client_to_avt_dev(client);

	int ret = 0;
	uint32_t crc = 0;
	uint32_t crc_byte_count = 0;

	uint32_t i2c_reg;
	uint32_t i2c_reg_size;
	uint32_t i2c_reg_count;

	char *i2c_reg_buf;

	mutex_lock(&camera->lock);
	avt_dbg(get_sd(camera), "+");

	i2c_reg = camera->cci_reg.reg.gcprm_address + 0x0000;
	i2c_reg_size = AV_CAM_REG_SIZE;
	i2c_reg_count = sizeof(camera->gencp_reg);
	i2c_reg_buf = (char *)&camera->gencp_reg;

	/* Calculate CRC from each reg up to the CRC reg */
	crc_byte_count =
		(uint32_t)((char *)&camera->gencp_reg.checksum - (char *)&camera->gencp_reg);

	ret = avt_read_raw(camera, camera->cci_reg.reg.gcprm_address + 0x0000, 
		(char *)&camera->gencp_reg, sizeof(camera->gencp_reg));

	if (ret < 0)
	{
		avt_err(get_sd(camera), "regmap_read failed, ret %d", ret);
		goto err_out;
	}

	crc = crc32(U32_MAX, &camera->gencp_reg, crc_byte_count);

	be32_to_cpus(&camera->gencp_reg.gcprm_layout_version);
	be16_to_cpus(&camera->gencp_reg.gencp_out_buffer_address);
	be16_to_cpus(&camera->gencp_reg.gencp_in_buffer_address);
	be16_to_cpus(&camera->gencp_reg.gencp_out_buffer_size);
	be16_to_cpus(&camera->gencp_reg.gencp_in_buffer_size);
	be32_to_cpus(&camera->gencp_reg.checksum);

	if (crc != camera->gencp_reg.checksum)
	{
		avt_err(get_sd(camera), "wrong GENCP CRC value! calculated = 0x%x, received = 0x%x\n",
				crc, camera->gencp_reg.checksum);
		ret = -EINVAL;
		goto err_out;
	}

	avt_dbg(get_sd(camera), "gcprm layout version: %x\n",
			camera->gencp_reg.gcprm_layout_version);
	avt_dbg(get_sd(camera), "gcprm out buf addr: %x\n",
			camera->gencp_reg.gencp_out_buffer_address);
	avt_dbg(get_sd(camera), "gcprm out buf size: %x\n",
			camera->gencp_reg.gencp_out_buffer_size);
	avt_dbg(get_sd(camera), "gcprm in buf addr: %x\n",
			camera->gencp_reg.gencp_in_buffer_address);
	avt_dbg(get_sd(camera), "gcprm in buf size: %x\n",
			camera->gencp_reg.gencp_in_buffer_size);

err_out:
	mutex_unlock(&camera->lock);

	return ret;
}

static int cci_version_check(struct i2c_client *client)
{
	struct avt_dev *camera = client_to_avt_dev(client);
	uint32_t cci_minver, cci_majver;
	int ret = 0;

	mutex_lock(&camera->lock);

	cci_minver = (camera->cci_reg.reg.layout_version & CCI_REG_LAYOUT_MINVER_MASK) >> CCI_REG_LAYOUT_MINVER_SHIFT;

	if (cci_minver >= CCI_REG_LAYOUT_MINVER)
	{
		avt_dbg(get_sd(camera), "correct cci register minver: %d (0x%x)\n",
				cci_minver, camera->cci_reg.reg.layout_version);
	}
	else
	{
		avt_err(get_sd(camera), "cci reg minver mismatch! read: %d (0x%x) expected: %d\n",
				cci_minver, camera->cci_reg.reg.layout_version, CCI_REG_LAYOUT_MINVER);
		ret = -EINVAL;
		goto err_out;
	}

	cci_majver = (camera->cci_reg.reg.layout_version & CCI_REG_LAYOUT_MAJVER_MASK) >> CCI_REG_LAYOUT_MAJVER_SHIFT;

	if (cci_majver == CCI_REG_LAYOUT_MAJVER)
	{
		avt_dbg(get_sd(camera), "correct cci register majver: %d (0x%x)\n",
				cci_majver, camera->cci_reg.reg.layout_version);
	}
	else
	{
		avt_err(get_sd(camera), "cci reg majver mismatch! read: %d (0x%x) expected: %d\n",
				cci_majver, camera->cci_reg.reg.layout_version, CCI_REG_LAYOUT_MAJVER);
		ret = -EINVAL;
		goto err_out;
	}

err_out:
	mutex_unlock(&camera->lock);

	return ret;
}

static int bcrm_version_check(struct i2c_client *client)
{
	struct avt_dev *camera = client_to_avt_dev(client);
	u32 value = 0;
	int ret;

	mutex_lock(&camera->lock);
	/* reading the BCRM version */
	ret = bcrm_read32(camera, BCRM_VERSION_32R, &value);

	if (ret < 0)
	{
		avt_err(get_sd(camera), "regmap_read failed (%d)", ret);
		goto err_out;
	}

	avt_dbg(get_sd(camera), "bcrm version (driver): 0x%x (maj: 0x%x min: 0x%x)\n",
			BCRM_DEVICE_VERSION,
			BCRM_MAJOR_VERSION,
			BCRM_MINOR_VERSION);

	avt_info(get_sd(camera), "camera bcrm version %lu.%lu\n", 
		 FIELD_GET(BCRM_VERSION_MAJOR, value),
		 FIELD_GET(BCRM_VERSION_MINOR, value));
	
	ret = FIELD_GET(BCRM_VERSION_MAJOR, value) == BCRM_MAJOR_VERSION;

	camera->bcrm_version = value;

err_out:
	mutex_unlock(&camera->lock);

	return ret;
}

static int gcprm_version_check(struct i2c_client *client)
{
	struct avt_dev *camera = client_to_avt_dev(client);
	u32 value = camera->gencp_reg.gcprm_layout_version;

	mutex_lock(&camera->lock);
	avt_dbg(get_sd(camera), "gcprm version (driver): 0x%x (maj: 0x%x min: 0x%x)\n",
			GCPRM_DEVICE_VERSION,
			GCPRM_MAJOR_VERSION,
			GCPRM_MINOR_VERSION);

	avt_dbg(get_sd(camera), "gcprm version (camera): 0x%x (maj: 0x%x min: 0x%x)\n",
			value,
			(value & 0xffff0000) >> 16,
			(value & 0x0000ffff));
	mutex_unlock(&camera->lock);

	return (value & 0xffff0000) >> 16 == GCPRM_MAJOR_VERSION ? 1 : 0;
}

/* implementation of driver attibutes published in sysfs */

static ssize_t availability_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	mutex_lock(&camera->lock);

	ret = sprintf(buf, "%d\n", camera->open_refcnt == 0 ? 1 : 0);

	mutex_unlock(&camera->lock);

	return ret;
}

static ssize_t cci_register_layout_version_show(struct device *dev,
												struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	mutex_lock(&camera->lock);
	ret = sprintf(buf, "%d\n", camera->cci_reg.reg.layout_version);
	mutex_unlock(&camera->lock);

	return ret;
}

static ssize_t bcrm_feature_inquiry_reg_show(struct device *dev,
											 struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;
	union bcrm_feature_inquiry_reg feature_inquiry_reg;

	/* reading the Feature inquiry register */
	ret = bcrm_read64(camera, BCRM_FEATURE_INQUIRY_64R, &feature_inquiry_reg.value);
	
	if (ret < 0)
	{
		avt_err(get_sd(camera), "regmap_bulk_read BCRM_FEATURE_INQUIRY_64R failed (%ld)", ret);
		return ret;
	}

	ret = sprintf(buf, "0x%016llX\n", feature_inquiry_reg.value);

	return ret;
}

static ssize_t bcrm_feature_inquiry_reg_text_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	union bcrm_feature_inquiry_reg *inq_reg = &camera->feature_inquiry_reg;
	
	ssize_t ret = 0;

	ret = sprintf(buf,
		      "reverse_x_avail                 %d\n"
		      "reverse_y_avail                 %d\n"
		      "intensity_auto_precedence_avail %d\n"
		      "black_level_avail               %d\n"
		      "gain_avail                      %d\n"
		      "gamma_avail                     %d\n"
		      "contrast_avail                  %d\n"
		      "saturation_avail                %d\n"
		      "hue_avail                       %d\n"
		      "white_balance_avail             %d\n"
		      "sharpness_avail                 %d\n"
		      "exposure_auto                   %d\n"
		      "gain_auto                       %d\n"
		      "white_balance_auto_avail        %d\n"
		      "device_temperature_avail        %d\n"
		      "acquisition_abort               %d\n"
		      "acquisition_frame_rate          %d\n"
		      "frame_trigger                   %d\n"
		      "exposure active line available  %d\n"
		      "auto region                     %d\n"
		      "frame trigger wait line         %d\n"
		      "color transformation matrix     %d\n"
		      "user data storage               %d\n"
		      "device status                   %d\n"
		      "revision id                     %d\n"
		      "direct memory access            %d\n"
		      "exposure mode                   %d\n"
		      "power save mode                 %d\n"
		      "sensorboard temperature         %d\n"
		      "temperature warning level       %d\n",
		      inq_reg->feature_inq.reverse_x_avail,
		      inq_reg->feature_inq.reverse_y_avail,
		      inq_reg->feature_inq.intensity_auto_precedence_avail,
		      inq_reg->feature_inq.black_level_avail,
		      inq_reg->feature_inq.gain_avail,
		      inq_reg->feature_inq.gamma_avail,
		      inq_reg->feature_inq.contrast_avail,
		      inq_reg->feature_inq.saturation_avail,
		      inq_reg->feature_inq.hue_avail,
		      inq_reg->feature_inq.white_balance_avail,
		      inq_reg->feature_inq.sharpness_avail,
		      inq_reg->feature_inq.exposure_auto_avail,
		      inq_reg->feature_inq.gain_auto_avail,
		      inq_reg->feature_inq.white_balance_auto_avail,
		      inq_reg->feature_inq.device_temperature_avail,
		      inq_reg->feature_inq.acquisition_abort,
		      inq_reg->feature_inq.acquisition_frame_rate,
		      inq_reg->feature_inq.frame_trigger,
		      inq_reg->feature_inq.exposure_active_line_available,
		      inq_reg->feature_inq.auto_region,
		      inq_reg->feature_inq.frame_trigger_wait_line,
		      inq_reg->feature_inq.color_transformation_matrix,
		      inq_reg->feature_inq.user_data_storage,
		      inq_reg->feature_inq.device_status,
		      inq_reg->feature_inq.revision_id,
		      inq_reg->feature_inq.direct_memory_access,
		      inq_reg->feature_inq.exposure_mode,
		      inq_reg->feature_inq.power_save_mode,
		      inq_reg->feature_inq.sensorboard_temperature,
		      inq_reg->feature_inq.temperature_warning_level);
	return ret;
}

static ssize_t bcrm_bayer_formats_show(struct device *dev,
									   struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "0x%04X\n", camera->bayer_inquiry_reg.value);

	return ret;
}

static ssize_t bcrm_bayer_formats_text_show(struct device *dev,
											struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "monochrome_avail %d\n"
					   "bayer_GR_avail   %d\n"
					   "bayer_RG_avail   %d\n"
					   "bayer_GB_avail   %d\n"
					   "bayer_BG_avail   %d\n",
				  camera->bayer_inquiry_reg.bayer_pattern.monochrome_avail,
				  camera->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail,
				  camera->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail,
				  camera->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail,
				  camera->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail);

	return ret;
}

static ssize_t bcrm_mipi_formats_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	mutex_lock(&camera->lock);
	ret = sprintf(buf, "0x%016llX\n", camera->avail_mipi_reg.value);
	mutex_unlock(&camera->lock);

	return ret;
}

static ssize_t bcrm_mipi_formats_text_show(struct device *dev,
										   struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));

	return sprintf(buf, "yuv420_8_leg_avail   %d\n"
						"yuv420_8_avail       %d\n"
						"yuv420_10_avail      %d\n"
						"yuv420_8_csps_avail  %d\n"
						"yuv420_10_csps_avail %d\n"
						"yuv422_8_avail       %d\n"
						"yuv422_10_avail      %d\n"
						"rgb888_avail         %d\n"
						"rgb666_avail         %d\n"
						"rgb565_avail         %d\n"
						"rgb555_avail         %d\n"
						"rgb444_avail         %d\n"
						"raw6_avail           %d\n"
						"raw7_avail           %d\n"
						"raw8_avail           %d\n"
						"raw10_avail          %d\n"
						"raw12_avail          %d\n"
						"raw14_avail          %d\n"
						"jpeg_avail           %d\n",
				   camera->avail_mipi_reg.avail_mipi.yuv420_8_leg_avail,
				   camera->avail_mipi_reg.avail_mipi.yuv420_8_avail,
				   camera->avail_mipi_reg.avail_mipi.yuv420_10_avail,
				   camera->avail_mipi_reg.avail_mipi.yuv420_8_csps_avail,
				   camera->avail_mipi_reg.avail_mipi.yuv420_10_csps_avail,
				   camera->avail_mipi_reg.avail_mipi.yuv422_8_avail,
				   camera->avail_mipi_reg.avail_mipi.yuv422_10_avail,
				   camera->avail_mipi_reg.avail_mipi.rgb888_avail,
				   camera->avail_mipi_reg.avail_mipi.rgb666_avail,
				   camera->avail_mipi_reg.avail_mipi.rgb565_avail,
				   camera->avail_mipi_reg.avail_mipi.rgb555_avail,
				   camera->avail_mipi_reg.avail_mipi.rgb444_avail,
				   camera->avail_mipi_reg.avail_mipi.raw6_avail,
				   camera->avail_mipi_reg.avail_mipi.raw7_avail,
				   camera->avail_mipi_reg.avail_mipi.raw8_avail,
				   camera->avail_mipi_reg.avail_mipi.raw10_avail,
				   camera->avail_mipi_reg.avail_mipi.raw12_avail,
				   camera->avail_mipi_reg.avail_mipi.raw14_avail,
				   camera->avail_mipi_reg.avail_mipi.jpeg_avail);
}

static ssize_t device_capabilities_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));

	return sprintf(buf, "0x%016llX\n", camera->cci_reg.reg.device_capabilities.value);
}

static ssize_t device_capabilities_text_show(struct device *dev,
											 struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	mutex_lock(&camera->lock);

	ret = sprintf(buf, "user_name        %d\n"
					   "bcrm             %d\n"
					   "gencp            %d\n"
					   "string_encoding  %s\n"
					   "family_name      %d\n",
				  camera->cci_reg.reg.device_capabilities.caps.user_name,
				  camera->cci_reg.reg.device_capabilities.caps.bcrm,
				  camera->cci_reg.reg.device_capabilities.caps.gencp,
				  camera->cci_reg.reg.device_capabilities.caps.string_encoding == CCI_CAPS_SE_ASCII ? "ASCII" : camera->cci_reg.reg.device_capabilities.caps.string_encoding == CCI_CAPS_SE_UTF8 ? "UTF8"
																											: camera->cci_reg.reg.device_capabilities.caps.string_encoding == CCI_CAPS_SE_UTF16	 ? "UTF16"
																																																 : "unknown string encoding",
				  camera->cci_reg.reg.device_capabilities.caps.family_name);

	mutex_unlock(&camera->lock);

	return ret;
}

static ssize_t device_guid_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%s\n", camera->cci_reg.reg.device_guid);

	return ret;
}

static ssize_t manufacturer_name_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%s\n", camera->cci_reg.reg.manufacturer_name);

	return ret;
}

static ssize_t model_name_show(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%s\n", camera->cci_reg.reg.model_name);

	return ret;
}

static ssize_t family_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%s\n", camera->cci_reg.reg.family_name);

	return ret;
}

static ssize_t lane_count_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%d\n", camera->num_lanes);

	return ret;
}

static ssize_t lane_capabilities_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));

	return sprintf(buf, "0x%02X\n", camera->lane_capabilities.value);
}

static ssize_t device_version_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));

	return sprintf(buf, "%s\n", camera->cci_reg.reg.device_version);
}

static ssize_t firmware_version_show(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%u.%u.%u.%u\n",
				  camera->cam_firmware_version.device_firmware.special_version,
				  camera->cam_firmware_version.device_firmware.major_version,
				  camera->cam_firmware_version.device_firmware.minor_version,
				  camera->cam_firmware_version.device_firmware.patch_version);

	return ret;
}

static ssize_t manufacturer_info_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));

	return sprintf(buf, "%s\n", camera->cci_reg.reg.manufacturer_info);
}

static ssize_t serial_number_show(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));

	return sprintf(buf, "%s\n", camera->cci_reg.reg.serial_number);
}

static ssize_t user_defined_name_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));

	return sprintf(buf, "%s\n", camera->cci_reg.reg.user_defined_name);
}

static ssize_t driver_version_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_VERSION);
}

static ssize_t debug_en_show(struct device *dev,
							 struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d\n", debug);
}

static ssize_t debug_en_store(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;

	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	mutex_lock(&camera->lock);

	ret = kstrtoint(buf, 10, &debug);
	if (ret < 0)
	{

		mutex_unlock(&camera->lock);
		return ret;
	}

	mutex_unlock(&camera->lock);

	return count;
}

static ssize_t mipiclk_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));

	ret = sysfs_emit(buf, "%llu\n", camera->link_freq);

	return ret;
}

static ssize_t mipiclk_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	ssize_t ret;
	uint32_t avt_next_clk = 0;
	uint32_t avt_current_clk = 0;

	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	struct i2c_client *client = to_i2c_client(dev);
	mutex_lock(&camera->lock);

	ret = kstrtouint(buf, 10, &avt_next_clk);
	if (ret < 0)
	{
		goto out;
	}

	dev_dbg(&client->dev, "request %s %u  0x%08X",
			 buf, avt_next_clk, avt_next_clk);

	if ((avt_next_clk < camera->avt_min_clk) ||
		(avt_next_clk > camera->avt_max_clk))
	{
		dev_err(&client->dev, "%s[%d]: unsupported csi clock frequency (%u Hz, range: %u:%u Hz)!\n",
				__func__, __LINE__,
				avt_next_clk, camera->avt_min_clk,
				camera->avt_max_clk);
		ret = -EINVAL;
	}
	else
	{
		ret = bcrm_write32(camera, BCRM_CSI2_CLOCK_32RW, avt_next_clk);

		ret = bcrm_read32(camera, BCRM_CSI2_CLOCK_32RW, &avt_current_clk);

		adev_info(&client->dev, "csi clock frequency requested %u Hz, applied %u Hz)\n", avt_next_clk, avt_current_clk);

		if (0 < avt_current_clk)
			camera->link_freq = avt_current_clk;
	}

out:
	mutex_unlock(&camera->lock);

	return count;
}

static ssize_t device_temperature_show(struct device *dev,
									   struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	int device_temperature;

	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	mutex_lock(&camera->lock);

	ret = bcrm_read32(camera, BCRM_DEVICE_TEMPERATURE_32R, &device_temperature);

	ret = sprintf(buf, "%d.%d\n", device_temperature / 10, device_temperature % 10);

	mutex_unlock(&camera->lock);
	return ret;
}

static ssize_t softreset_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%d\n", camera->pending_softreset_request);

	return ret;
}

static ssize_t softreset_store(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;
	int value;

	ret = kstrtoint(buf, 10, &value);
	if (ret < 0)
	{
		return ret;
	}

	if (value > 0) {
		mutex_lock(&camera->lock);
		ret = avt_do_softreset(camera);
		if (ret < 0)
			goto err;

		/* Reinit v4l2 settings */
		avt_reinit(camera);                
err:
		mutex_unlock(&camera->lock);
	}

	return ret ? ret : count;
}

static ssize_t dphyreset_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	mutex_lock(&camera->lock);

	ret = sprintf(buf, "%d\n", camera->pending_dphyreset_request);

	mutex_unlock(&camera->lock);

	return ret;
}

static ssize_t dphyreset_store(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	mutex_lock(&camera->lock);
	ret = kstrtoint(buf, 10, &camera->pending_dphyreset_request);
	if (ret < 0)
	{
		mutex_unlock(&camera->lock);
		return ret;
	}

	if (camera->pending_dphyreset_request > 0)
	{
		avt_dphy_reset(camera, true);
		avt_dphy_reset(camera, false);
	}
	mutex_unlock(&camera->lock);
	return count;
}

static ssize_t streamon_delay_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	mutex_lock(&camera->lock);

	ret = bcrm_read32(camera, BCRM_STREAM_ON_DELAY_32RW, &camera->streamon_delay);

	ret = sprintf(buf, "%u\n", camera->streamon_delay);

	mutex_unlock(&camera->lock);

	return ret;
}

static ssize_t streamon_delay_store(struct device *dev,
									struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt_dev *camera = client_to_avt_dev(to_i2c_client(dev));
	ssize_t ret;

	mutex_lock(&camera->lock);
	ret = kstrtoint(buf, 10, &camera->streamon_delay);
	if (ret < 0)
	{
		mutex_unlock(&camera->lock);
		return ret;
	}

	ret = bcrm_write32(camera, BCRM_STREAM_ON_DELAY_32RW,
			   camera->streamon_delay);

	
	mutex_unlock(&camera->lock);
	return count;
}


static ssize_t bcrm_dump_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	bcrm_dump(to_i2c_client(dev));

	return 0;
}


static DEVICE_ATTR_RO(availability);
static DEVICE_ATTR_RO(bcrm_dump);
static DEVICE_ATTR_RO(cci_register_layout_version);
static DEVICE_ATTR_RO(device_capabilities);
static DEVICE_ATTR_RO(firmware_version);
static DEVICE_ATTR_RO(device_capabilities_text);
static DEVICE_ATTR_RO(bcrm_feature_inquiry_reg);
static DEVICE_ATTR_RO(bcrm_feature_inquiry_reg_text);
static DEVICE_ATTR_RO(device_guid);
static DEVICE_ATTR_RO(device_version);
static DEVICE_ATTR_RO(driver_version);
static DEVICE_ATTR_RO(family_name);
static DEVICE_ATTR_RO(lane_count);
static DEVICE_ATTR_RO(lane_capabilities);
static DEVICE_ATTR_RO(manufacturer_info);
static DEVICE_ATTR_RO(manufacturer_name);
static DEVICE_ATTR_RO(model_name);
static DEVICE_ATTR_RO(serial_number);
static DEVICE_ATTR_RO(user_defined_name);
static DEVICE_ATTR_RO(bcrm_mipi_formats);
static DEVICE_ATTR_RO(bcrm_mipi_formats_text);
static DEVICE_ATTR_RO(bcrm_bayer_formats);
static DEVICE_ATTR_RO(bcrm_bayer_formats_text);
static DEVICE_ATTR_RW(debug_en);
static DEVICE_ATTR_RW(softreset);
static DEVICE_ATTR_RW(dphyreset);
static DEVICE_ATTR_RW(streamon_delay);
static DEVICE_ATTR_RO(device_temperature);
static DEVICE_ATTR_RW(mipiclk);

static struct attribute *avt_attrs[] = {
	&dev_attr_availability.attr,
	&dev_attr_bcrm_dump.attr,
	&dev_attr_cci_register_layout_version.attr,
	&dev_attr_device_capabilities.attr,
	&dev_attr_device_capabilities_text.attr,
	&dev_attr_firmware_version.attr,
	&dev_attr_device_guid.attr,
	&dev_attr_device_version.attr,
	&dev_attr_driver_version.attr,
	&dev_attr_bcrm_feature_inquiry_reg.attr,
	&dev_attr_bcrm_feature_inquiry_reg_text.attr,
	&dev_attr_family_name.attr,
	&dev_attr_lane_count.attr,
	&dev_attr_lane_capabilities.attr,
	&dev_attr_manufacturer_info.attr,
	&dev_attr_manufacturer_name.attr,
	&dev_attr_model_name.attr,
	&dev_attr_serial_number.attr,
	&dev_attr_user_defined_name.attr,
	&dev_attr_bcrm_mipi_formats.attr,
	&dev_attr_bcrm_mipi_formats_text.attr,
	&dev_attr_bcrm_bayer_formats.attr,
	&dev_attr_bcrm_bayer_formats_text.attr,
	&dev_attr_debug_en.attr,
	&dev_attr_dphyreset.attr,
	&dev_attr_streamon_delay.attr,
	&dev_attr_softreset.attr,
	&dev_attr_device_temperature.attr,
	&dev_attr_mipiclk.attr,
	NULL};

static struct attribute_group avt_attr_grp = {
	.attrs = avt_attrs,
};

static int avt_get_fmt_available(struct i2c_client *client)
{
	struct avt_dev *camera = client_to_avt_dev(client);
	u8 bayer_val = 0;
	int ret;
	u64 avail_mipi = 0;

	mutex_lock(&camera->lock);

	ret = bcrm_read64(camera, BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R, &avail_mipi);

	if (ret < 0)
	{
		dev_err(&client->dev, "%s[%d]regmap_bulk_read (%d)\n",
				__func__, __LINE__, ret);
		goto out;
	}

	camera->avail_mipi_reg.value = avail_mipi;

	/* read the Bayer Inquiry register to check whether the camera
	 * really support the requested RAW format
	 */
	ret = bcrm_read8(camera, BCRM_IMG_BAYER_PATTERN_INQUIRY_8R, &bayer_val);

	if (ret < 0)
	{
		dev_err(&client->dev, "%s[%d]: regmap_read (%d)\n",
				__func__, __LINE__, ret);
		goto out;
	}

	camera->bayer_inquiry_reg.value = bayer_val;

out:
	avt_dbg(get_sd(camera), "avail_mipi 0x%016llX bayer_val 0x%02X ret %d",
			camera->avail_mipi_reg.value,
			camera->bayer_inquiry_reg.value, ret);

	mutex_unlock(&camera->lock);
	return ret;
}

static int lookup_media_bus_format_index(struct avt_dev *camera, u32 mbus_code)
{

	int i;

	for (i = 0; i < camera->available_fmts_cnt; i++)
	{
		if (mbus_code == camera->available_fmts[i].mbus_code)
			return i;
	}

	return -EINVAL;
}

static void set_mode_mapping(struct avt_csi_mipi_mode_mapping *pfmt,
			     u32 mbus_code, u16 mipi_fmt, u32 colorspace,
			     u32 fourcc, enum bayer_format bayer_pattern,
			     const char *name)
{
	pfmt->mbus_code = mbus_code;
	pfmt->mipi_fmt = mipi_fmt;
	pfmt->fourcc = fourcc;
	pfmt->colorspace = colorspace;
	pfmt->bayer_pattern = bayer_pattern;
	strcpy(pfmt->name, name);
}

/* ToDo: read available formats from Cam */
static int avt_init_avail_formats(struct v4l2_subdev *sd)
{
	struct avt_dev *camera = to_avt_dev(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct avt_csi_mipi_mode_mapping *pfmt;
	union bcrm_bayer_inquiry_reg old_bayer;

	if (sd == NULL)
	{
		return -EINVAL;
	}

	avt_dbg(sd, "camera->available_fmts_cnt %d", camera->available_fmts_cnt);

	camera->available_fmts_cnt = 0;

	avt_dbg(sd, "%s %s %s %s\n",
		camera->cci_reg.reg.manufacturer_name,
		camera->cci_reg.reg.family_name,
		camera->cci_reg.reg.model_name,
		camera->cci_reg.reg.device_guid);

	avt_dbg(sd, "Camera bayer pattern:");
	avt_dbg(sd, "monochrome_avail %d", camera->bayer_inquiry_reg.bayer_pattern.monochrome_avail);
	avt_dbg(sd, "bayer_GR_avail   %d", camera->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail);
	avt_dbg(sd, "bayer_RG_avail   %d", camera->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail);
	avt_dbg(sd, "bayer_GB_avail   %d", camera->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail);
	avt_dbg(sd, "bayer_BG_avail   %d", camera->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail);

	avt_dbg(sd, "reverse_x %d", camera->reverse_x_reg);
	avt_dbg(sd, "reverse_y %d", camera->reverse_y_reg);

	/* The state of reverse x and reverse y affects the bayer pattern.
	   Since the camera is not modifying it, we need to do this here.
	*/
        old_bayer = camera->bayer_inquiry_reg;

	// Due to this change the user will be able to select all bayer patterns, indepenently of their availability
	camera->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail = 1;
	camera->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail = 1;
	camera->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail = 1;
	camera->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail = 1;

	avt_dbg(sd, "Modified bayer pattern:");
	avt_dbg(sd, "monochrome_avail %d", camera->bayer_inquiry_reg.bayer_pattern.monochrome_avail);
	avt_dbg(sd, "bayer_GR_avail   %d", camera->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail);
	avt_dbg(sd, "bayer_RG_avail   %d", camera->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail);
	avt_dbg(sd, "bayer_GB_avail   %d", camera->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail);
	avt_dbg(sd, "bayer_BG_avail   %d", camera->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail);

	avt_dbg(sd, "Camera MIPI formats:");
	avt_dbg(sd, "yuv420_8_leg_avail   %d", camera->avail_mipi_reg.avail_mipi.yuv420_8_leg_avail);
	avt_dbg(sd, "yuv420_8_avail       %d", camera->avail_mipi_reg.avail_mipi.yuv420_8_avail);
	avt_dbg(sd, "yuv420_10_avail      %d", camera->avail_mipi_reg.avail_mipi.yuv420_10_avail);
	avt_dbg(sd, "yuv420_8_csps_avail  %d", camera->avail_mipi_reg.avail_mipi.yuv420_8_csps_avail);
	avt_dbg(sd, "yuv420_10_csps_avail %d", camera->avail_mipi_reg.avail_mipi.yuv420_10_csps_avail);
	avt_dbg(sd, "yuv422_8_avail       %d", camera->avail_mipi_reg.avail_mipi.yuv422_8_avail);
	avt_dbg(sd, "yuv422_10_avail      %d", camera->avail_mipi_reg.avail_mipi.yuv422_10_avail);
	avt_dbg(sd, "rgb888_avail         %d", camera->avail_mipi_reg.avail_mipi.rgb888_avail);
	avt_dbg(sd, "rgb666_avail         %d", camera->avail_mipi_reg.avail_mipi.rgb666_avail);
	avt_dbg(sd, "rgb565_avail         %d", camera->avail_mipi_reg.avail_mipi.rgb565_avail);
	avt_dbg(sd, "rgb555_avail         %d", camera->avail_mipi_reg.avail_mipi.rgb555_avail);
	avt_dbg(sd, "rgb444_avail         %d", camera->avail_mipi_reg.avail_mipi.rgb444_avail);
	avt_dbg(sd, "raw6_avail           %d", camera->avail_mipi_reg.avail_mipi.raw6_avail);
	avt_dbg(sd, "raw7_avail           %d", camera->avail_mipi_reg.avail_mipi.raw7_avail);
	avt_dbg(sd, "raw8_avail           %d", camera->avail_mipi_reg.avail_mipi.raw8_avail);
	avt_dbg(sd, "raw10_avail          %d", camera->avail_mipi_reg.avail_mipi.raw10_avail);
	avt_dbg(sd, "raw12_avail          %d", camera->avail_mipi_reg.avail_mipi.raw12_avail);
	avt_dbg(sd, "raw14_avail          %d", camera->avail_mipi_reg.avail_mipi.raw14_avail);
	avt_dbg(sd, "jpeg_avail           %d", camera->avail_mipi_reg.avail_mipi.jpeg_avail);

	camera->available_fmts = kmalloc(sizeof(camera->available_fmts[0]) * AVT_MAX_FORMAT_ENTRIES, GFP_KERNEL);

	if (!camera->available_fmts)
	{
		dev_err(&client->dev,
			"%s[%d]: not enough memory to store list of available formats",
			__func__, __LINE__);
		return -ENOMEM;
	}

	pfmt = camera->available_fmts;

  #define add_format_unconditional(mbus_code, mipi_fmt, colorspace, fourcc, bayer_pattern) \
	set_mode_mapping(pfmt, mbus_code, mipi_fmt, colorspace, fourcc, bayer_pattern, #mbus_code); \
	camera->available_fmts_cnt++; \
	pfmt++;

  #define add_format_gen(avail_field_name, mbus_code, mipi_fmt, colorspace, fourcc, bayer_pattern) \
    if(camera->avail_mipi_reg.avail_mipi.avail_field_name) { \
      add_format_unconditional(MEDIA_BUS_FMT_ ## mbus_code, MIPI_CSI2_DT_ ## mipi_fmt, colorspace, V4L2_PIX_FMT_ ## fourcc, bayer_pattern); \
    }

  #define add_format_srgb(avail_field_name, mbus_code, mipi_fmt, fourcc) \
    	add_format_gen(avail_field_name, mbus_code, mipi_fmt, V4L2_COLORSPACE_SRGB, fourcc, bayer_ignore)

  #define add_format_raw(pattern_avail_field, avail_field_name, mbus_code, mipi_fmt, fourcc, bayer_format) \
	if(camera->bayer_inquiry_reg.bayer_pattern.pattern_avail_field) {\
      		add_format_gen(avail_field_name, mbus_code, mipi_fmt, V4L2_COLORSPACE_RAW, fourcc, bayer_format); \
    	}

	// YUV formats
	add_format_srgb(yuv422_8_avail,  UYVY8_2X8,   YUV422_8B, UYVY);
	add_format_srgb(yuv422_8_avail,  UYVY8_1X16,  YUV422_8B, UYVY);
	add_format_srgb(yuv422_8_avail,  YUYV8_1X16,  YUV422_8B, YUV422P);
	add_format_srgb(yuv422_8_avail,  YUYV8_2X8,   YUV422_8B, YUYV);
	add_format_srgb(yuv422_8_avail,  VYUY8_2X8,   YUV422_8B, VYUY);

	add_format_srgb(yuv422_10_avail, YUYV10_1X20, YUV422_8B, YUV410);

	// RGB formats
	add_format_srgb(rgb888_avail,    RGB888_1X24, RGB888,    RGB24);
	add_format_srgb(rgb888_avail,    RBG888_1X24, RGB888,    RGB24);
	add_format_srgb(rgb888_avail,    BGR888_1X24, RGB888,    RGB24);
	add_format_srgb(rgb888_avail,    RGB888_3X8,  RGB888,    RGB24);

	// 8 bit raw formats (mono / bayer)
	add_format_raw(monochrome_avail, raw8_avail,  Y8_1X8,       RAW8,  GREY,    monochrome);
	add_format_raw(bayer_GR_avail,   raw8_avail,  SGRBG8_1X8,   RAW8,  SGRBG8,  bayer_gr);
	add_format_raw(bayer_RG_avail,   raw8_avail,  SRGGB8_1X8,   RAW8,  SRGGB8,  bayer_rg);
	add_format_raw(bayer_BG_avail,   raw8_avail,  SBGGR8_1X8,   RAW8,  SBGGR8,  bayer_bg);
	add_format_raw(bayer_GB_avail,   raw8_avail,  SGBRG8_1X8,   RAW8,  SGBRG8,  bayer_gb);

	// 10 bit raw formats (mono / bayer)
	add_format_raw(monochrome_avail, raw10_avail, Y10_1X10,     RAW10, Y10,     monochrome);
	add_format_raw(bayer_GR_avail,   raw10_avail, SGRBG10_1X10, RAW10, SGRBG10, bayer_gr);
	add_format_raw(bayer_RG_avail,   raw10_avail, SRGGB10_1X10, RAW10, SRGGB10, bayer_rg);
	add_format_raw(bayer_BG_avail,   raw10_avail, SBGGR10_1X10, RAW10, SGRBG10, bayer_bg);
	add_format_raw(bayer_GB_avail,   raw10_avail, SGBRG10_1X10, RAW10, SGBRG10, bayer_gb);

	// 12 bit raw formats (mono / bayer)
	add_format_raw(monochrome_avail, raw12_avail, Y12_1X12,     RAW12, Y12,     monochrome);
	add_format_raw(bayer_GR_avail,   raw12_avail, SGRBG12_1X12, RAW12, SGRBG12, bayer_gr);
	add_format_raw(bayer_RG_avail,   raw12_avail, SRGGB12_1X12, RAW12, SRGGB12, bayer_rg);
	add_format_raw(bayer_BG_avail,   raw12_avail, SBGGR12_1X12, RAW12, SGRBG12, bayer_bg);
	add_format_raw(bayer_GB_avail,   raw12_avail, SGBRG12_1X12, RAW12, SGBRG12, bayer_gb);

	// 14 bit raw formats (mono / bayer)
	add_format_raw(monochrome_avail, raw14_avail, Y14_1X14,     RAW14, Y14,     monochrome);
	add_format_raw(bayer_GR_avail,   raw14_avail, SGRBG14_1X14, RAW14, SGRBG14, bayer_gr);
	add_format_raw(bayer_RG_avail,   raw14_avail, SRGGB14_1X14, RAW14, SRGGB14, bayer_rg);
	add_format_raw(bayer_BG_avail,   raw14_avail, SBGGR14_1X14, RAW14, SGRBG14, bayer_bg);
	add_format_raw(bayer_GB_avail,   raw14_avail, SGBRG14_1X14, RAW14, SGBRG14, bayer_gb);

  	// GenICam
	add_format_unconditional(MEDIA_BUS_FMT_CUSTOM, 0x31, V4L2_COLORSPACE_DEFAULT, V4L2_PIX_FMT_CUSTOM, bayer_ignore);

  #undef add_format_raw
  #undef add_format
  #undef add_format_gen
  #undef add_format_unconditional

	// Restore camera bayer pattern
  	camera->bayer_inquiry_reg = old_bayer;

	pfmt->mbus_code = -EINVAL;

	avt_dbg(get_sd(camera), "available_fmts_cnt %d", camera->available_fmts_cnt);

	return camera->available_fmts_cnt;
}

static int avt_init_current_format(struct avt_dev *camera, struct v4l2_mbus_framefmt *fmt)
{
	u32 current_mipi_format;
	u32 current_max_width;
	u32 current_max_height;
	u8 current_bayer_pattern;
	int ret, i;
	
	ret = bcrm_read32(camera, BCRM_IMG_MIPI_DATA_FORMAT_32RW, &current_mipi_format);
	if (unlikely(ret))
	{
		dev_err(&camera->i2c_client->dev, "Failed to read current mipi format!");
		return ret;
	}

	ret = bcrm_read8(camera, BCRM_IMG_BAYER_PATTERN_8RW, &current_bayer_pattern);
	if (unlikely(ret))
	{
		dev_err(&camera->i2c_client->dev, "Failed to read current bayer pattern!");
		return ret;
	}

	ret = bcrm_read32(camera, BCRM_IMG_WIDTH_MAX_32R, &current_max_width);
	if (unlikely(ret))
	{
		dev_err(&camera->i2c_client->dev, "Failed to read current max image width!");
		return ret;
	}

	ret = bcrm_read32(camera, BCRM_IMG_HEIGHT_MAX_32R, &current_max_height);
	if (unlikely(ret))
	{
		dev_err(&camera->i2c_client->dev, "Failed to read current max image height!");
		return ret;
	}

	for (i = 0;i < camera->available_fmts_cnt; i++)
	{
		const struct avt_csi_mipi_mode_mapping *mapping = &camera->available_fmts[i];
		bool const bayer_correct = 
			mapping->bayer_pattern == current_bayer_pattern
			|| mapping->bayer_pattern == bayer_ignore;

		if (mapping->mipi_fmt == current_mipi_format && bayer_correct)
		{
			fmt->code = mapping->mbus_code;
			fmt->colorspace = V4L2_COLORSPACE_SRGB;
			fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
			fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
			fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
			fmt->width = current_max_width;
			fmt->height = current_max_height;
			fmt->field = V4L2_FIELD_NONE;

			return 0;
		}
	}

	return -EINVAL;
}

static int avt_do_softreset(struct avt_dev *camera)
{
	struct device *dev = &camera->i2c_client->dev;
	int ret;
	u8 val;
	u64 start;

	ret = avt_write8(camera, CCI_HEARTBEAT_8RW, 0x80);
	if (ret < 0)
		return ret;

	ret = avt_read8(camera, CCI_HEARTBEAT_8RW, &val);
	if (ret < 0)
		return ret;
	
	if (!(val >= 0x80))
		return -ENOTSUPP;	

	dev_info(dev, "Heartbeat supported, performing softreset...\n");
	
	start = ktime_get_ns();

	ret = avt_write8(camera, CCI_SOFTRESET_8W, 1);
	if (ret < 0)
		return ret;

	ret = read_poll_timeout(avt_read8, ret, val > 0 && val < 0x80, 
				BOOT_POLL_INTERVAL_US, BOOT_TIMEOUT_US,
			  	true, camera, CCI_HEARTBEAT_8RW, &val);
	if (ret < 0) {
		dev_err(dev, "Softreset failed with err: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Camera boottime %llu ms\n", 
		 (ktime_get_ns() - start) / NSEC_PER_MSEC);

	return 0;
}

static int __reset_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret;

	if (!ctrl)
		return 0;

	if (ctrl->flags & V4L2_CTRL_FLAG_READ_ONLY)
		return 0;

	switch(ctrl->type) {
	case V4L2_CTRL_TYPE_INTEGER:
	case V4L2_CTRL_TYPE_INTEGER_MENU:
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_BITMASK:
		ret = __v4l2_ctrl_s_ctrl(ctrl, ctrl->default_value);
		break;
	case V4L2_CTRL_TYPE_INTEGER64:
		ret = __v4l2_ctrl_s_ctrl_int64(ctrl, ctrl->default_value);
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static int avt_reset_ctrls(struct avt_dev *camera)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(camera->avt_ctrls); i++) {
		avt_dbg(get_sd(camera), "reset control %s\n",
			camera->avt_ctrls[i]->name);

		ret = __reset_ctrl(camera->avt_ctrls[i]);
		if (ret)
			break;
	}

	return ret;
}

static int avt_reinit(struct avt_dev *camera)
{
	int ret;

	// Re-read and configure MIPI configuration
	avt_get_camera_capabilities(get_sd(camera));

	// Re-init
	ret = avt_update_format(camera, &camera->curr_rect, camera->curr_binning_info);
	if (ret < 0)
	{
		dev_err(&camera->i2c_client->dev, "%s[%d]: Error while updating format",
			__func__, __LINE__);
		return ret;
	}

	ret = avt_write_media_bus_format(camera,
					 avt_get_mode_fmt(camera)->code);
	if (ret < 0)
	{
		dev_err(&camera->i2c_client->dev, "%s[%d]: Error while writing media bus format",
			__func__, __LINE__);
		return ret;
	}
	
	ret = avt_reset_ctrls(camera);

	return ret;
}

static void avt_dphy_reset(struct avt_dev *camera, bool bResetPhy)
{
	struct i2c_client *client = camera->i2c_client;
	int ret;
	int ival = bResetPhy;

	ret = bcrm_write8(camera, BCRM_PHY_RESET_8RW, ival);
	
	if (ret < 0)
	{
		dev_err(&client->dev, "%s[%d]: avt_dphy_reset request by calling regmap_write CSI2_PHY_RESET_32RW failed (%d)\n",
				__func__, __LINE__, ret);
		goto out;
	}

out:
	camera->pending_dphyreset_request = 0;
}

static struct v4l2_mbus_framefmt *
avt_get_pad_fmt(struct avt_dev *camera, 
		struct v4l2_subdev_state *state,
		u32 pad, u32 which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		return v4l2_subdev_get_try_format(get_sd(camera), state, pad);
#else		
		if (!state->sd)
			state->sd = get_sd(camera);
		return v4l2_subdev_state_get_format(state, pad);
#endif
	}

	return avt_get_mode_fmt(camera);
}

static struct v4l2_rect *
avt_get_pad_crop(struct avt_dev *camera, 
		 struct v4l2_subdev_state *state,
		 u32 pad, u32 which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
		return v4l2_subdev_get_try_crop(get_sd(camera), state, pad);
#else	
		if (!state->sd)
			state->sd = get_sd(camera);
		return v4l2_subdev_state_get_crop(state, pad);
#endif
	}

	return &camera->curr_rect;
}


/* --------------- Subdev Operations --------------- */
static int avt_pad_ops_get_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_format *format)
{
	struct avt_dev *camera = to_avt_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != 0) {
		avt_err(sd, "format->pad != 0");
		return -EINVAL;
	}

	mutex_lock(&camera->lock);

	
	fmt = avt_get_pad_fmt(camera, sd_state, format->pad, format->which);
	
	format->format = *fmt;
	
	mutex_unlock(&camera->lock);


	return 0;
}

static void avt_calc_compose(const struct avt_dev * const camera,
			      const struct v4l2_rect * const crop,
			      u32 *width,u32 *height,
			      const struct avt_binning_info **info)
{
	const u32 type = camera->curr_binning_type;
	const struct avt_binning_info * const infos = camera->binning_infos[type];
	const size_t cnt = camera->binning_info_cnt[type];
	const struct v4l2_rect * const min = &camera->min_rect;
	const struct v4l2_rect * const max = &camera->sensor_rect;
	const struct v4l2_mbus_framefmt *fmt = avt_get_mode_fmt(camera);
	const bool x_changed = *width != fmt->width;
	const bool y_changed = *height != fmt->height;
	const bool type_changed = type != camera->curr_binning_info->type;
	const struct avt_binning_info *best;
	struct v4l2_rect scaled_crop = *crop;
	struct v4l2_rect binning_rect = {0};

	best = camera->curr_binning_info;

	if (x_changed || y_changed || type_changed) {
		u32 min_error = U32_MAX;
		int i;

		for (i = 0; i < cnt; i++) {
			const struct avt_binning_info * const cur = &infos[i];
			const u32 s_width = camera->curr_rect.width / cur->vfact;
			const u32 s_height = camera->curr_rect.height / cur->hfact;
			u32 error = 0;

			if (x_changed || type_changed)
				error += abs(s_width - *width);

			if (y_changed || type_changed)
				error += abs(s_height - *height);

			if (error > min_error)
				continue;

			min_error = error;
			best = cur;
			if (error == 0)
				break;
		}
	}

	dev_dbg(&camera->i2c_client->dev,"Selected binning %dx%d type: %s\n",
		best->vfact,best->hfact,binning_type_str[type]);

	binning_rect.width = best->max_width;
	binning_rect.height = best->max_height;

	v4l2_rect_scale(&scaled_crop,max,&binning_rect);

	v4l_bound_align_image(&scaled_crop.width,min->width,
			      binning_rect.width,3,
			      &scaled_crop.height,min->height,
			      binning_rect.height,3,0);

	*width = scaled_crop.width;
	*height = scaled_crop.height;

	dev_dbg(&camera->i2c_client->dev,"Selected crop (%u,%u) %ux%u\n",
		 scaled_crop.left,scaled_crop.top,
		 scaled_crop.width,scaled_crop.height);

	*info = best;
}

static int avt_update_format(struct avt_dev *camera,
	const struct v4l2_rect *roi,
	const struct avt_binning_info *info)
{
	int ret = 0;
	struct v4l2_ctrl *ctrl;
	struct v4l2_rect scaled_roi = *roi;
	const struct v4l2_rect binning_rect = {
		.width = info->max_width,
		.height = info->max_height,
	};

	v4l2_rect_scale(&scaled_roi, &camera->sensor_rect, &binning_rect);

	v4l_bound_align_image(
		&scaled_roi.width,camera->min_rect.width,
		binning_rect.width,3,
		&scaled_roi.height,camera->min_rect.height,
		binning_rect.height,3,0);
	
	camera->curr_binning_info = info;

	if (camera->power_state == POWER_STATE_STANDBY)
		return 0;


	ret = bcrm_write8(camera, BCRM_BINNING_SETTING_8RW, info->sel);
	if (unlikely(ret)) 
		return ret;

	ret = bcrm_write32(camera, BCRM_IMG_WIDTH_32RW, scaled_roi.width);
	if (unlikely(ret)) 
		return ret;

	ret = bcrm_write32(camera, BCRM_IMG_HEIGHT_32RW, scaled_roi.height);
	if (unlikely(ret)) 
		return ret;

	ret = bcrm_write32(camera, BCRM_IMG_OFFSET_X_32RW, scaled_roi.left);
	if (unlikely(ret)) 
		return ret;

	ret = bcrm_write32(camera, BCRM_IMG_OFFSET_Y_32RW, scaled_roi.top);
	if (unlikely(ret)) 
		return ret;


	ctrl = avt_ctrl_find(camera, AVT_CID_AUTO_REGION_TOP);
	if (ctrl) {
		__v4l2_ctrl_s_ctrl(ctrl, 0);
		__v4l2_ctrl_modify_range(ctrl, ctrl->minimum, 0, ctrl->step, 0);
	}
	
	ctrl = avt_ctrl_find(camera, AVT_CID_AUTO_REGION_LEFT);
	if (ctrl) {
		__v4l2_ctrl_s_ctrl(ctrl, 0);
		__v4l2_ctrl_modify_range(ctrl, ctrl->minimum, 0, ctrl->step, 0);
	}

	ctrl = avt_ctrl_find(camera, AVT_CID_AUTO_REGION_WIDTH);
	if (ctrl) {
		__v4l2_ctrl_s_ctrl(ctrl, scaled_roi.width);
		__v4l2_ctrl_modify_range(ctrl, ctrl->minimum, scaled_roi.width,
					 ctrl->step, scaled_roi.width);
	}


	ctrl = avt_ctrl_find(camera, AVT_CID_AUTO_REGION_HEIGHT);
	if (ctrl) {
		__v4l2_ctrl_s_ctrl(ctrl, scaled_roi.height);
		__v4l2_ctrl_modify_range(ctrl, ctrl->minimum, scaled_roi.height,
					 ctrl->step, scaled_roi.height);
	}

	return ret;
}

static void transform_mbus_code(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct avt_dev *camera = to_avt_dev(sd);
	bool transformed = true;
	u32 old_code = fmt->code;

	// No transformation needed if we are not using a bayer format
	if (fmt->code != MEDIA_BUS_FMT_SRGGB8_1X8 &&
		fmt->code != MEDIA_BUS_FMT_SGRBG8_1X8 &&
		fmt->code != MEDIA_BUS_FMT_SBGGR8_1X8 &&
		fmt->code != MEDIA_BUS_FMT_SGBRG8_1X8 &&
		fmt->code != MEDIA_BUS_FMT_SRGGB10_1X10 &&
		fmt->code != MEDIA_BUS_FMT_SGRBG10_1X10 &&
		fmt->code != MEDIA_BUS_FMT_SBGGR10_1X10 &&
		fmt->code != MEDIA_BUS_FMT_SGBRG10_1X10 &&
		fmt->code != MEDIA_BUS_FMT_SRGGB12_1X12 &&
		fmt->code != MEDIA_BUS_FMT_SGRBG12_1X12 &&
		fmt->code != MEDIA_BUS_FMT_SBGGR12_1X12 &&
		fmt->code != MEDIA_BUS_FMT_SGBRG12_1X12) {
			transformed = false;
			camera->mbus_fmt_code = fmt->code;
	}
	else {
		if (camera->mbus_fmt_transformed == false) {
			// Store real mbus value
			camera->mbus_fmt_code = fmt->code;
		}
		else {
			fmt->code = camera->mbus_fmt_code;
		}

		if (camera->reverse_x_reg == 0 && camera->reverse_y_reg == 1) {
			/* Swap G and B for 8-bit, 10-bit, and 12-bit formats
				RG -> REV_Y -> GB
				GR -> REV_Y -> BG
				BG -> REV_Y -> GR
				GB -> REV_Y -> RG
			*/
			if (fmt->code == MEDIA_BUS_FMT_SRGGB8_1X8) { fmt->code = MEDIA_BUS_FMT_SGBRG8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SGRBG8_1X8) { fmt->code = MEDIA_BUS_FMT_SBGGR8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SBGGR8_1X8) { fmt->code = MEDIA_BUS_FMT_SGRBG8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SGBRG8_1X8) { fmt->code = MEDIA_BUS_FMT_SRGGB8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SRGGB10_1X10) { fmt->code = MEDIA_BUS_FMT_SGBRG10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SGRBG10_1X10) { fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SBGGR10_1X10) { fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SGBRG10_1X10) { fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SRGGB12_1X12) { fmt->code = MEDIA_BUS_FMT_SGBRG12_1X12; }
			else if (fmt->code == MEDIA_BUS_FMT_SGRBG12_1X12) { fmt->code = MEDIA_BUS_FMT_SBGGR12_1X12; }
			else if (fmt->code == MEDIA_BUS_FMT_SBGGR12_1X12) { fmt->code = MEDIA_BUS_FMT_SGRBG12_1X12; }
			else if (fmt->code == MEDIA_BUS_FMT_SGBRG12_1X12) { fmt->code = MEDIA_BUS_FMT_SRGGB12_1X12; }
			else transformed = false;
		} else if (camera->reverse_x_reg == 1 && camera->reverse_y_reg == 0) {
			/* Swap R and B for 8-bit, 10-bit, and 12-bit formats
				RG -> REV_X -> GR
				GR -> REV_X -> RG
				BG -> REV_X -> GB
				GB -> REV_X -> BG
			*/
			if (fmt->code == MEDIA_BUS_FMT_SRGGB8_1X8) { fmt->code = MEDIA_BUS_FMT_SGRBG8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SGRBG8_1X8) { fmt->code = MEDIA_BUS_FMT_SRGGB8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SBGGR8_1X8) { fmt->code = MEDIA_BUS_FMT_SGBRG8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SGBRG8_1X8) { fmt->code = MEDIA_BUS_FMT_SBGGR8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SRGGB10_1X10) { fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SGRBG10_1X10) { fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SBGGR10_1X10) { fmt->code = MEDIA_BUS_FMT_SGBRG10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SGBRG10_1X10) { fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SRGGB12_1X12) { fmt->code = MEDIA_BUS_FMT_SGRBG12_1X12; }
			else if (fmt->code == MEDIA_BUS_FMT_SGRBG12_1X12) { fmt->code = MEDIA_BUS_FMT_SRGGB12_1X12; }
			else if (fmt->code == MEDIA_BUS_FMT_SBGGR12_1X12) { fmt->code = MEDIA_BUS_FMT_SGBRG12_1X12; }
			else if (fmt->code == MEDIA_BUS_FMT_SGBRG12_1X12) { fmt->code = MEDIA_BUS_FMT_SBGGR12_1X12; }
			else transformed = false;
		} else if (camera->reverse_x_reg == 1 && camera->reverse_y_reg == 1) {
			/* Swap R and B, and G and B for 8-bit, 10-bit, and 12-bit formats
				RG -> REV_XY -> BG
				GR -> REV_XY -> GB
				BG -> REV_XY -> RG
				GB -> REV_XY -> GR
			*/
			if (fmt->code == MEDIA_BUS_FMT_SRGGB8_1X8) { fmt->code = MEDIA_BUS_FMT_SBGGR8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SGRBG8_1X8) { fmt->code = MEDIA_BUS_FMT_SGBRG8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SBGGR8_1X8) { fmt->code = MEDIA_BUS_FMT_SRGGB8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SGBRG8_1X8) { fmt->code = MEDIA_BUS_FMT_SGRBG8_1X8; }
			else if (fmt->code == MEDIA_BUS_FMT_SRGGB10_1X10) { fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SGRBG10_1X10) { fmt->code = MEDIA_BUS_FMT_SGBRG10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SBGGR10_1X10) { fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SGBRG10_1X10) { fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10; }
			else if (fmt->code == MEDIA_BUS_FMT_SRGGB12_1X12) { fmt->code = MEDIA_BUS_FMT_SBGGR12_1X12; }
			else if (fmt->code == MEDIA_BUS_FMT_SGRBG12_1X12) { fmt->code = MEDIA_BUS_FMT_SGBRG12_1X12; }
			else if (fmt->code == MEDIA_BUS_FMT_SBGGR12_1X12) { fmt->code = MEDIA_BUS_FMT_SRGGB12_1X12; }
			else if (fmt->code == MEDIA_BUS_FMT_SGBRG12_1X12) { fmt->code = MEDIA_BUS_FMT_SGRBG12_1X12; }
			else transformed = false;
		}
		else {
			transformed = false;
		}
	}

	avt_dbg(get_sd(camera), "fmt->code 0x%04X -> 0x%04X. rev_x %d rev_y %d, transformed %d", 
		old_code, fmt->code, 
		camera->reverse_x_reg, camera->reverse_y_reg,
		transformed); 

	camera->mbus_fmt_transformed = transformed;
}



static int avt_try_fmt_internal(struct v4l2_subdev *sd,
				 struct v4l2_mbus_framefmt *fmt,
				 const struct avt_binning_info **new_binning)
{
	struct avt_dev *camera = to_avt_dev(sd);
	int i;

	avt_calc_compose(camera,&camera->curr_rect,&fmt->width,&fmt->height,
		new_binning);

	avt_dbg(get_sd(camera), 
		"fmt->width %d, fmt->height %d",
		fmt->width, fmt->height);
	avt_dbg(get_sd(camera), 
		"camera->available_fmts_cnt %d",
		camera->available_fmts_cnt);

	avt_dbg(get_sd(camera), "Incoming fmt->code    0x%04x", fmt->code);

	transform_mbus_code(sd, fmt);

	avt_dbg(get_sd(camera), "Transformed fmt->code 0x%04x", fmt->code);

	for (i = 0; i < camera->available_fmts_cnt; i++)
	{
		
		if (camera->available_fmts[i].mbus_code == fmt->code)
		{
			break;
		}
	}

	if (i == camera->available_fmts_cnt)
	{
		avt_dbg(sd, "format fmt->code 0x%04X not found in available formats [ToDo: error handling incomplete]", fmt->code);
		fmt->code = avt_get_mode_fmt(camera)->code;
		//return -EINVAL;
	}

	memset(fmt->reserved, 0, sizeof(fmt->reserved));

	fmt->colorspace = camera->available_fmts[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return 0;
}

static int avt_update_exposure_limits(struct v4l2_subdev *sd) {
	struct avt_dev *camera = to_avt_dev(sd);
	int ret;
	u64 exp_min, exp_max, exp_inc;

	ret = bcrm_read64(camera, BCRM_EXPOSURE_TIME_MIN_64R, &exp_min);
	if(ret < 0) {
		avt_err(sd, "Failed to read minimum exposure: %d", ret);
		goto err;
	}

	ret = bcrm_read64(camera, BCRM_EXPOSURE_TIME_MAX_64R, &exp_max);
	if(ret < 0) {
		avt_err(sd, "Failed to read maximum exposure: %d", ret);
		goto err;
	}

	ret = bcrm_read64(camera, BCRM_EXPOSURE_TIME_INC_64R, &exp_inc);
	if(ret < 0) {
		avt_err(sd, "Failed to read exposure increment: %d", ret);
		goto err;
	}

	{
		struct v4l2_ctrl * exp_ctrl = avt_ctrl_find(camera, V4L2_CID_EXPOSURE);
		if(exp_ctrl != NULL) {
			__v4l2_ctrl_modify_range(exp_ctrl, exp_min, exp_max, exp_inc, exp_ctrl->default_value);
		}
	}

	{
		struct v4l2_ctrl* exp_abs_ctrl = avt_ctrl_find(camera, V4L2_CID_EXPOSURE_ABSOLUTE);
		if(exp_abs_ctrl != NULL) {
			__v4l2_ctrl_modify_range(exp_abs_ctrl, exp_min / EXP_ABS, exp_max / EXP_ABS, exp_inc / EXP_ABS, exp_abs_ctrl->default_value);
		}
	}

	{
		struct v4l2_ctrl *exp_auto_min_ctrl = 
			avt_ctrl_find(camera, AVT_CID_EXPOSURE_AUTO_MIN);
		struct v4l2_ctrl *exp_auto_max_ctrl =
			avt_ctrl_find(camera, AVT_CID_EXPOSURE_AUTO_MAX);

		if(exp_auto_min_ctrl != NULL && exp_auto_max_ctrl != NULL) {
			__v4l2_ctrl_modify_range(exp_auto_min_ctrl, exp_min, exp_max, exp_inc, exp_min);
			__v4l2_ctrl_modify_range(exp_auto_max_ctrl, exp_min, exp_max, exp_inc, exp_max);
		}
	}

err:
	return ret;
}

static int avt_write_media_bus_format(struct avt_dev *camera, int code)
{
	struct device *dev = &camera->i2c_client->dev;
	const struct avt_csi_mipi_mode_mapping *fmt_mapping;
	int idx = lookup_media_bus_format_index(camera, code);
	int ret = 0;
	u8 bayer_pattern = 0;

	if (idx < 0) {
		return -EINVAL;
	}

	fmt_mapping = &camera->available_fmts[idx];
	bayer_pattern = fmt_mapping->bayer_pattern;

	ret = bcrm_write32(camera, BCRM_IMG_MIPI_DATA_FORMAT_32RW, 
		fmt_mapping->mipi_fmt);

	if (unlikely(ret)) {
		dev_err(dev, "Failed to set mipi format to %x with %d\n",
			fmt_mapping->mipi_fmt, ret);

		goto exit;
	}

	dev_info(dev, "fmt_mapping->bayer_pattern %d, rev_x %d rev_y %d, transformed %d", 
		fmt_mapping->bayer_pattern, 
		camera->reverse_x_reg, camera->reverse_y_reg,
		camera->mbus_fmt_transformed); 


	if (fmt_mapping->bayer_pattern != bayer_ignore) {
		if (fmt_mapping->bayer_pattern != monochrome) {
			if (camera->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail) {
				bayer_pattern = bayer_bg;
			}
			else
			if (camera->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail) {
				bayer_pattern = bayer_gb;
			}
			else
			if (camera->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail) {
				bayer_pattern = bayer_gr;
			}
			else
			if (camera->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail) {
				bayer_pattern = bayer_rg;
			}
		}

		ret = bcrm_write8(camera, BCRM_IMG_BAYER_PATTERN_8RW, 
			bayer_pattern);
		if (unlikely(ret)) {
			dev_err(dev,
				"Failed to set bayer pattern to %x with %d\n",
				fmt_mapping->bayer_pattern, ret);

			goto exit;
		}
	}

exit:
	return ret;
}

static int avt_set_fmt_internal_bcrm(struct avt_dev *camera,
				     struct v4l2_subdev_format *format)
{
	struct v4l2_subdev *sd = get_sd(camera);
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	const struct avt_binning_info *new_binning = NULL;
	int ret = 0;

	if (mbus_fmt->code == MEDIA_BUS_FMT_CUSTOM) {
		if (format->which != V4L2_SUBDEV_FORMAT_TRY) {
			*mbus_fmt = *avt_get_mode_fmt(camera);
		}
		goto out;
	} else {
		ret = avt_try_fmt_internal(sd, mbus_fmt, &new_binning);
		if (ret)
			goto out;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (new_binning != camera->curr_binning_info) {
			ret = avt_update_format(camera, &camera->curr_rect, new_binning);
			if (ret < 0)
				goto out;
		}

		if (mbus_fmt->code != avt_get_mode_fmt(camera)->code 
			&& camera->power_state != POWER_STATE_STANDBY) {
			ret = avt_write_media_bus_format(camera,
							 mbus_fmt->code);

			if(ret < 0) {
				avt_err(sd, "failed to set mipi datatype: %d",
					ret);
				goto out;
			}

			ret = avt_update_exposure_limits(sd);
		}
	}
out:
	return ret;
}

static int avt_set_fmt_internal_gencp(struct avt_dev *camera,
				      struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;

	switch (mbus_fmt->code) {
		case MEDIA_BUS_FMT_Y8_1X8:
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SBGGR8_1X8:
		case MEDIA_BUS_FMT_SGRBG8_1X8: 
		case MEDIA_BUS_FMT_SGBRG8_1X8:
		case MEDIA_BUS_FMT_CUSTOM:
			break;
		default:
			mbus_fmt->code = MEDIA_BUS_FMT_CUSTOM;
			break;
	}	

	return 0;
}


static int avt_pad_ops_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *format)
{
	struct avt_dev *camera = to_avt_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	int ret;

	avt_dbg(sd, "%s[%d]",
			 __func__, __LINE__);
	avt_dbg(sd, "%d x %d, format.code 0x%04X, format.pad %d",
			format->format.width, format->format.height, format->format.code, format->pad);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&camera->lock);
	if (camera->is_streaming) {
		ret = -EBUSY;
		goto out;
	}

	fmt = avt_get_pad_fmt(camera, state, format->pad,format->which);

	if (camera->mode == AVT_BCRM_MODE) {
		ret = avt_set_fmt_internal_bcrm(camera, format);
	} else if (camera->mode == AVT_GENCP_MODE) {
		ret = avt_set_fmt_internal_gencp(camera, format);
	} else {
		ret = -EINVAL;
	}

	if (!ret)
		*fmt = format->format;
out:
	mutex_unlock(&camera->lock);

	return ret;
}
static int read_control_value(struct avt_dev *camera,s64 *value, const u16 reg,
			      const u8 size)
{
	int ret = 0;
	u8 tmp[8];

	if (size > AV_CAM_DATA_SIZE_64) 
		return -EINVAL;

	ret = avt_read_raw(camera, get_bcrm_addr(camera, reg), tmp, size);

	if (ret < 0)
		return ret;

	switch (size)
	{
	case AV_CAM_DATA_SIZE_8:
		*value = tmp[0];
		break;
	case AV_CAM_DATA_SIZE_16:
		*value = get_unaligned_be16(tmp);
		break;
	case AV_CAM_DATA_SIZE_32:
		*value = (s32)get_unaligned_be32(tmp);
		break;
	case AV_CAM_DATA_SIZE_64:
		*value = (s64)get_unaligned_be64(tmp);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void avt_ctrl_to_reg(const u32 cid,s64 * value)
{
	switch (cid) {
	case V4L2_CID_EXPOSURE_AUTO:
		if (*value == V4L2_EXPOSURE_MANUAL)
			*value = 0;
		else
			*value = 2;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
	case V4L2_CID_AUTOGAIN:
		if (*value)
			*value = 2;
		else
			*value = 0;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		*value = *value * EXP_ABS;
		break;
	default:
		break;
	}
}

static void avt_ctrl_from_reg(const u32 cid,s64 * value)
{
	switch (cid) {
	case V4L2_CID_EXPOSURE_AUTO:
		if (*value)
			*value = V4L2_EXPOSURE_AUTO;
		else
			*value = V4L2_EXPOSURE_MANUAL;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
	case V4L2_CID_AUTOGAIN:
		if (*value)
			*value = 1;
		else
			*value = 0;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		*value = *value / EXP_ABS;
		break;
	default:
		break;
	}
}

static int avt_update_ctrl_value(struct avt_dev *camera,
				  struct v4l2_ctrl *ctrl,
				  const struct avt_ctrl_mapping *mapping)
{
	const u16 reg = mapping->reg_offset;
	const u8 len = mapping->reg_length;
	int ret = 0;
	s64 value = 0;

	ret = read_control_value(camera, &value, reg, len);

	if (ret < 0) {
		avt_err(get_sd(camera),"Reading ctrl %x (reg: %x) failed with: %d",
			ctrl->id,reg,ret);
		return ret;
	}

	avt_ctrl_from_reg(ctrl->id,&value);

	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_INTEGER:
	case V4L2_CTRL_TYPE_BITMASK:
		ctrl->val = (s32)value;
		break;
	case V4L2_CTRL_TYPE_INTEGER64:
		*ctrl->p_cur.p_s64 = value;
		*ctrl->p_new.p_s64 = value;
		break;
	case V4L2_CTRL_TYPE_U8:
		*ctrl->p_cur.p_u8 = value;
		*ctrl->p_new.p_u8 = value;
		break;
	case V4L2_CTRL_TYPE_U16:
		*ctrl->p_cur.p_u16 = value;
		*ctrl->p_new.p_u16 = value;
		break;
	case V4L2_CTRL_TYPE_U32:
		*ctrl->p_cur.p_u32 = value;
		*ctrl->p_new.p_u32 = value;
		break;
	default:
		break;
	}

	return 0;
}

static int avt_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	const struct avt_ctrl_mapping * const ctrl_mapping = ctrl->priv;
	struct avt_dev *camera = container_of(ctrl->handler, struct avt_dev, v4l2_ctrl_hdl);

	avt_dbg(get_sd(camera), "ctrl->id %d", ctrl->id);

	if (camera->mode != AVT_BCRM_MODE) {
		return -EBUSY;
	}

	if (camera->power_state != POWER_STATE_ACTIVE)
		return 0;

	if (ctrl->id == AVT_CID_BINNING_SETTING) {
		ctrl->p_new.p_area->width = camera->curr_binning_info->hfact;
		ctrl->p_new.p_area->height = camera->curr_binning_info->vfact;
		return 0;
	}

	if (unlikely(!ctrl_mapping)) {
		avt_warn(get_sd(camera), "Invalid control mapping!\n");
		return -EINVAL;
	}

	return avt_update_ctrl_value(camera, ctrl, ctrl_mapping);
}

static struct v4l2_ctrl* avt_ctrl_find(struct avt_dev *camera,u32 id)
{
	int i;

	for (i = 0; i < AVT_MAX_CTRLS; i++)
	{
		struct v4l2_ctrl * ctrl = camera->avt_ctrls[i];

		if (ctrl && ctrl->id == id)
		{
			return ctrl;
		}
	}

	return NULL;
}

static inline int avt_trigger_mode_enabled(struct avt_dev *camera)
{
	u8 tmp = 0;
	int ret = 0;

	const struct v4l2_ctrl * trigger_mode_ctrl = 
			avt_ctrl_find(camera, AVT_CID_TRIGGER_MODE);
	
	if (trigger_mode_ctrl) {
		return trigger_mode_ctrl->val != 0;
	}

	ret = bcrm_read8(camera, BCRM_FRAME_START_TRIGGER_MODE_8RW, &tmp);

	if (unlikely(ret)) {
		return ret;
	}

	return tmp != 0;
}

static inline int avt_test_trigger_source(struct avt_dev *camera, int source)
{
	u8 tmp = 0;
	int ret = 0;

	const struct v4l2_ctrl * trigger_source_ctrl = 
		avt_ctrl_find(camera, AVT_CID_TRIGGER_SOURCE);

	if (trigger_source_ctrl) {
		return trigger_source_ctrl->val == source;
	}

	ret = bcrm_read8(camera, BCRM_FRAME_START_TRIGGER_SOURCE_8RW, &tmp);

	if (unlikely(ret)) {
		return ret;
	}

	return tmp == source;
}

static void avt_update_sw_ctrl_state(struct avt_dev *camera)
{
	int trigger_en = 0, trigger_sw_source = 0;
	struct v4l2_ctrl * sw_trigger_ctrl =
		avt_ctrl_find(camera, AVT_CID_TRIGGER_SOFTWARE);

	if (!sw_trigger_ctrl) {
		return;
	}

	trigger_en = avt_trigger_mode_enabled(camera);
	if (trigger_en < 0) {
		return;
	}

	trigger_sw_source =
		avt_test_trigger_source(camera, AVT_TRIGGER_SOURCE_SOFTWARE);
	if (trigger_sw_source < 0) {
		return;
	}
	
	v4l2_ctrl_activate(sw_trigger_ctrl, trigger_en && trigger_sw_source);
}

static const struct v4l2_event avt_source_change_event = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static void __auto_region_update_limits(struct avt_dev *camera,
					const struct v4l2_ctrl *parent_ctrl, 
					int id, u32 max)
{
	const u32 new_max = max - parent_ctrl->val;
	struct v4l2_ctrl *ctrl;
	
	ctrl = avt_ctrl_find(camera, id);
	if (!ctrl)
		return;

		
	__v4l2_ctrl_modify_range(ctrl, ctrl->minimum, new_max, 
				 ctrl->step, new_max);
}					

static struct v4l2_event avt_pixelformat_change_event = {
	.type = AVT_V4L2_EVENT_PIXELFORMAT_CHANGE,
};

static struct v4l2_event src_change_event = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static void __reverse_xy_roi_change(struct avt_dev *camera, int id)
{
	struct v4l2_mbus_framefmt *fmt = avt_get_mode_fmt(camera);
	struct v4l2_rect r;
	u32 val;
	int ret;

	r = camera->curr_rect;

	if (id == V4L2_CID_HFLIP) {
		ret = bcrm_read32(camera, BCRM_IMG_WIDTH_MAX_32R, &val);
		
		if (camera->max_rect.width == val)
			return;
		
		// Override width if set to maximum
		if (r.width == camera->max_rect.width)
			r.width = val;

		camera->max_rect.width = val;
	} else if (id == V4L2_CID_VFLIP) {
		ret = bcrm_read32(camera, BCRM_IMG_HEIGHT_MAX_32R, &val);
		
		if (camera->max_rect.height == val)
			return;
		
		// Override height if set to maximum
		if (r.height == camera->max_rect.height)
			r.height = val;
		
		camera->max_rect.height = val;
	}

	__set_crop(camera, &r, fmt, &camera->curr_rect,
		V4L2_SUBDEV_FORMAT_ACTIVE);

	v4l2_subdev_notify_event(get_sd(camera), 
				&src_change_event);
}

static void __reverse_xy_changed(struct avt_dev *camera,
				 const struct v4l2_ctrl *ctrl)
{
	struct v4l2_mbus_framefmt *fmt = avt_get_mode_fmt(camera);

	if (camera->bcrm_version >= BCRM_VERSION(1, 28)) {
		avt_info(get_sd(camera), "handle roi change\n");

		__reverse_xy_roi_change(camera, ctrl->id);
		return;
	}

	if (ctrl->id == V4L2_CID_HFLIP) {
		camera->reverse_x_reg = (u8)ctrl->val;
		avt_info(get_sd(camera), 
			"V4L2_CID_HFLIP %d\n", camera->reverse_x_reg);
	}
	else if (ctrl->id == V4L2_CID_VFLIP) {
		camera->reverse_y_reg = (u8)ctrl->val;
		avt_info(get_sd(camera), 
			"V4L2_CID_VFLIP %d\n", camera->reverse_y_reg);
	}

	/* Notify user if we are currently using a bayer format */
	switch (fmt->code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		avt_dbg(get_sd(camera), 
			"Changed reverse x/y using "
			"camera->mbus_framefmt.code 0x%04x. "
			"Notify event AVT_V4L2_EVENT_PIXELFORMAT_CHANGE\n", 
			fmt->code);

		v4l2_subdev_notify_event(get_sd(camera),
			&avt_pixelformat_change_event);

		break;
	}
}

static void avt_ctrl_changed(struct avt_dev *camera,
			      const struct v4l2_ctrl * const ctrl)
{
	switch (ctrl->id)
	{
	case AVT_CID_TRIGGER_MODE:
		avt_update_sw_ctrl_state(camera);
		break;
	case AVT_CID_TRIGGER_SOURCE:
		avt_update_sw_ctrl_state(camera);
		break;
	case AVT_CID_EXPOSURE_AUTO_MIN: {
		struct v4l2_ctrl *max_ctrl;

		max_ctrl = avt_ctrl_find(camera, AVT_CID_EXPOSURE_AUTO_MAX);

		if (max_ctrl == NULL)
			break;

		__v4l2_ctrl_modify_range(max_ctrl,*ctrl->p_new.p_s64,
					 max_ctrl->maximum,max_ctrl->step,
					 max_ctrl->default_value);

		break;
	}
	case AVT_CID_EXPOSURE_AUTO_MAX: {
		struct v4l2_ctrl *min_ctrl;

		min_ctrl = avt_ctrl_find(camera, AVT_CID_EXPOSURE_AUTO_MIN);

		if (min_ctrl == NULL)
			break;

		__v4l2_ctrl_modify_range(min_ctrl,min_ctrl->minimum,
					 *ctrl->p_new.p_s64,min_ctrl->step,
					 min_ctrl->default_value);

		break;
	}
	case AVT_CID_GAIN_AUTO_MIN: {
		struct v4l2_ctrl *max_ctrl;

		max_ctrl = avt_ctrl_find(camera, AVT_CID_GAIN_AUTO_MAX);

		if (max_ctrl == NULL)
			break;

		__v4l2_ctrl_modify_range(max_ctrl,*ctrl->p_new.p_s64,
					 max_ctrl->maximum,max_ctrl->step,
					 max_ctrl->default_value);

		break;
	}
	case AVT_CID_GAIN_AUTO_MAX: {
		struct v4l2_ctrl *min_ctrl;

		min_ctrl = avt_ctrl_find(camera, AVT_CID_GAIN_AUTO_MIN);

		if (min_ctrl == NULL)
			break;

		__v4l2_ctrl_modify_range(min_ctrl,min_ctrl->minimum,
					 *ctrl->p_new.p_s64,min_ctrl->step,
					 min_ctrl->default_value);

		break;
	}
	case V4L2_CID_AUTOGAIN: {
		struct v4l2_ctrl *gain_ctrl;

		gain_ctrl = avt_ctrl_find(camera,V4L2_CID_GAIN);

		if (gain_ctrl != NULL)
			__v4l2_ctrl_grab(gain_ctrl,ctrl->val);

		break;
	}
	case V4L2_CID_EXPOSURE_AUTO: {
		struct v4l2_ctrl *exp_ctrl,*exp_abs_ctrl;
		bool grabbed = (ctrl->val == V4L2_EXPOSURE_AUTO);

		exp_ctrl = avt_ctrl_find(camera,V4L2_CID_EXPOSURE);

		if (exp_ctrl != NULL)
			__v4l2_ctrl_grab(exp_ctrl,grabbed);

		exp_abs_ctrl = avt_ctrl_find(camera,V4L2_CID_EXPOSURE_ABSOLUTE);

		if (exp_abs_ctrl != NULL)
			__v4l2_ctrl_grab(exp_abs_ctrl,grabbed);

		break;
	}
	case AVT_CID_BINNING_SELECTOR: {
		const struct avt_binning_info *info;
		struct v4l2_ctrl *binning_mode_ctrl;
		struct v4l2_mbus_framefmt *fmt = avt_get_mode_fmt(camera);
		u32 width = fmt->width;
		u32 height = fmt->height;

		camera->curr_binning_type = ctrl->val;

		avt_calc_compose(camera, &camera->curr_rect, &width, &height,
				  &info);

		camera->curr_binning_info = info;

		if (fmt->width != width
		    || fmt->height != height) {

			fmt->width = width;
			fmt->height = height;

			v4l2_subdev_notify_event(get_sd(camera),
						 &avt_source_change_event);
		}

		binning_mode_ctrl =
			avt_ctrl_find(camera, AVT_CID_BINNING_MODE);
		if (binning_mode_ctrl != NULL)
		{
			const long modes_enabled = binning_modes_enabled[ctrl->val];
			const u32 new_mode = find_first_bit(&modes_enabled,sizeof(modes_enabled));

			__v4l2_ctrl_s_ctrl(binning_mode_ctrl,new_mode);

			__v4l2_ctrl_modify_range(binning_mode_ctrl,
						 binning_mode_ctrl->minimum,
						 binning_mode_ctrl->maximum,
						 ~modes_enabled,
						 new_mode);
		}

	}
		break;
	case AVT_CID_AUTO_REGION_LEFT: {
		__auto_region_update_limits(camera, ctrl, 
					    AVT_CID_AUTO_REGION_WIDTH, 
					    camera->curr_rect.width);

		break;
	}
	case AVT_CID_AUTO_REGION_TOP: {
		__auto_region_update_limits(camera, ctrl,
					    AVT_CID_AUTO_REGION_HEIGHT,
					    camera->curr_rect.height);

		break;
	}
	case AVT_CID_AUTO_REGION_WIDTH: {
		__auto_region_update_limits(camera, ctrl,
					    AVT_CID_AUTO_REGION_LEFT,
					    camera->curr_rect.width);

		break;
	}
	case AVT_CID_AUTO_REGION_HEIGHT: {
		__auto_region_update_limits(camera, ctrl,
					    AVT_CID_AUTO_REGION_TOP, 
					    camera->curr_rect.height);
		break;
	}
	case AVT_CID_POWER_SAVE_MODE: 
		camera->power_state = ctrl->val ? POWER_STATE_STANDBY : POWER_STATE_ACTIVE;
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		__reverse_xy_changed(camera, ctrl);
		break;

	{
		
		break;

	}

	default:
		break;
	}

}

static int write_ctrl_value(struct avt_dev *camera,struct v4l2_ctrl *ctrl,
		      const struct avt_ctrl_mapping * const ctrl_mapping)
{
	const u16 reg = ctrl_mapping->reg_offset;
	const u8 reg_length = ctrl_mapping->reg_length;
	s64 temp;
	int ret = 0;

	if (ctrl->type == V4L2_CTRL_TYPE_INTEGER64)
		temp = *ctrl->p_new.p_s64;
	else
		temp = ctrl->val;

	avt_ctrl_to_reg(ctrl->id,&temp);

	if (ctrl_mapping->type == V4L2_CTRL_TYPE_BUTTON) {
		ret = bcrm_write(camera, reg, 1, reg_length);
	} else {
		ret = bcrm_write(camera, reg, temp, reg_length);
	}

	if (ret < 0)
		return ret;

	if (ctrl_mapping->avt_flags & AVT_CTRL_FLAG_READ_BACK) {
		ret =  avt_update_ctrl_value(camera, ctrl, ctrl_mapping);
		if (ret < 0)
			dev_err(&camera->i2c_client->dev,
				"Control read back failed with %d",
				ret);
	}


	return ret;
}

static int avt_line_get(struct avt_dev *camera, int line, bool output,
			bool invert, enum line_usage usage)
{
	int ret;
	u32 config;

	if (line >= ARRAY_SIZE(camera->line_usage)) 
		return -EINVAL;

	if (camera->line_usage[line] != LINE_USAGE_NONE)
		return -EBUSY;

	ret = bcrm_read32(camera, BCRM_LINE_CONFIGURATION_32RW, &config);
	if (ret < 0) 
		return ret;

	// Clear all line bits and apply configuration

	set_flag(&config, LINE_DIR_OUTPUT(line), output);
	set_flag(&config, LINE_INVERT(line), invert);

	avt_info(get_sd(camera), "Set line configuration %x\n", config);

	ret = bcrm_write32(camera, BCRM_LINE_CONFIGURATION_32RW, config);
	if (ret < 0)
		return ret;

	camera->line_usage[line] = usage;

	return 0;
}

static int avt_line_put(struct avt_dev *camera, int line)
{
	int ret;
	u32 config;

	if (line >= ARRAY_SIZE(camera->line_usage)) 
		return -EINVAL;
	
	
	ret = bcrm_read32(camera, BCRM_LINE_CONFIGURATION_32RW, &config);
	if (ret < 0) 
		return ret;
	
	set_flag(&config, LINE_DIR_OUTPUT(line), false);
	set_flag(&config, LINE_INVERT(line), false);

	ret = bcrm_write32(camera, BCRM_LINE_CONFIGURATION_32RW, config);
	if (ret < 0) 
		return ret;

	camera->line_usage[line] = LINE_USAGE_NONE;

	return 0;
}

static int __set_frame_trigger_wait_line_mode(struct avt_dev *camera,
					      bool active)
{
	int ret;
	struct v4l2_ctrl *line_ctrl, *invert_ctrl;

	line_ctrl = avt_ctrl_find(camera,
				  AVT_CID_FRAME_TRIGGER_WAIT_OUTPUT_LINE);
	if (!line_ctrl)
		return -EINVAL;
	
	invert_ctrl = avt_ctrl_find(camera, AVT_CID_FRAME_TRIGGER_WAIT_INVERT);
	if (!invert_ctrl)
		return -EINVAL;

	if (active) {
		ret = avt_line_get(camera, line_ctrl->val, 
				   true, invert_ctrl->val,
				   LINE_USAGE_FRAME_TRIGGER_WAIT);
		if (ret < 0)
			return ret;
	}

	ret = bcrm_write8(camera, BCRM_FRAME_TRIGGER_WAIT_LINE_MODE_8RW, active);
	if (ret < 0)
		return ret;

	if (!active) {
		ret = avt_line_put(camera, line_ctrl->val);
		if (ret < 0)
			return ret;
	}

	__v4l2_ctrl_grab(line_ctrl, active);
	__v4l2_ctrl_grab(invert_ctrl, active);

	return 0;
}					

static int __set_trigger_mode(struct avt_dev *camera, bool active)
{
	int ret;
	struct v4l2_ctrl *src_ctrl;

	src_ctrl = avt_ctrl_find(camera, AVT_CID_TRIGGER_SOURCE);
	if (!src_ctrl)
		return -EINVAL;

	if (src_ctrl->val <= AVT_TRIGGER_SOURCE_LINE3 && active) {
		ret = avt_line_get(camera, src_ctrl->val, false, false,
				   LINE_USAGE_TRIGGER);
		if (ret < 0)
			return ret;
	}

	ret = bcrm_write8(camera, BCRM_FRAME_START_TRIGGER_MODE_8RW, active);
	if (ret < 0)
		return ret;

	if (src_ctrl->val <= AVT_TRIGGER_SOURCE_LINE3 && !active) {
		ret = avt_line_put(camera, src_ctrl->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int __set_exposure_active_mode(struct avt_dev *camera, bool active)
{
	struct v4l2_ctrl *sel_ctrl,*invert_ctrl;
	int ret = 0;
	
	sel_ctrl = avt_ctrl_find(camera, AVT_CID_EXPOSURE_ACTIVE_LINE_SELECTOR);

	if (sel_ctrl == NULL)
		return -EINVAL;

	invert_ctrl = avt_ctrl_find(camera, AVT_CID_EXPOSURE_ACTIVE_INVERT);

	if (invert_ctrl == NULL)
		return -EINVAL;

	if (active) {
		ret = avt_line_get(camera, sel_ctrl->val,
				   true, invert_ctrl->val,
				   LINE_USAGE_EXPOSURE_ACTIVE);
		if (ret < 0)
			return ret;
	}
	
	ret = bcrm_write8(camera, BCRM_EXPOSURE_ACTIVE_LINE_MODE_8RW, active);
	if (ret < 0)
		return ret;

	if (!active) {
		ret = avt_line_put(camera, sel_ctrl->val);
		if (ret < 0)
			return ret;
	}

	__v4l2_ctrl_grab(sel_ctrl, active);
	__v4l2_ctrl_grab(invert_ctrl, active);

	return 0;
}

static int __write_color_transform_word(struct avt_dev *camera,
					int idx, u32 word)
{
	const int reg_idx = (idx / 2);
	const int reg_off = reg_idx * sizeof(word);
	const u16 reg = BCRM_COLOR_TRANSFORM_MATRIX_0_1_32RW + reg_off;

	avt_info(get_sd(camera), "Write value [%d]=%u\n", reg_idx, word);

	return bcrm_write32(camera, reg, word);
}

static int __set_color_transform_matrix(struct avt_dev *camera,
					s32 *cur, s32* new)
{
	int i, ret;
	u32 tmp = 0;

	for (i = 0; i < BCRM_COLOR_TRANSFORM_MATRIX_SIZE; i++, new++, cur++) {
		if (i % 2) {
			tmp |= ((((s16)*new) & 0xFFFF) << 16) ;

			ret = __write_color_transform_word(camera, i, tmp);
			if (ret < 0)
				return ret;
			tmp = 0;
		} else {
			tmp = ((s16)*new) & 0xFFFF;

			if (i == (BCRM_COLOR_TRANSFORM_MATRIX_SIZE - 1)) {
				ret = __write_color_transform_word(camera, i,	
								   tmp);
				if (ret < 0)
					return ret;
			}
		}

	}

	return 0;
}

static inline int __write_user_data_value(struct avt_dev *camera,
				 	  int idx, u32 val)
{
	int ret;

	ret = bcrm_write8(camera, BCRM_USER_DATA_INDEX_8RW, idx);
	if (ret < 0)
		return ret;
	
	ret = bcrm_write32(camera, BCRM_USER_DATA_VALUE_32RW, val);
	if (ret < 0)
		return ret;

	return 0;
}

static int __set_user_data_ctrl(struct avt_dev *camera, u32 *cur, u32 *new)
{
	int i, ret = 0;

	for (i = 0; i < BCRM_USER_DATA_INDEX_COUNT; i++, cur++, new++) {
		if (*cur != *new) {
			ret = __write_user_data_value(camera, i, *new);
			if (ret < 0)
				break;
		}
	}

	return ret;
}

static int avt_get_device_status(struct avt_dev *camera)
{
	u32 val;
	int ret;

	ret = bcrm_read32(camera, BCRM_DEVICE_STATUS_32R, &val);
	if (ret)
		return ret;

	return val;
}

static int return_from_power_save(struct avt_dev *camera,
				  struct v4l2_ctrl *pwr_save_ctrl)
{
	int ret;

	ret = read_poll_timeout(avt_get_device_status, ret,
				ret & BCRM_DEVICE_STATUS_STREAM_READY,
				5000, 10000000, false, camera);

	if (ret) {
		avt_err(get_sd(camera),
			"return from power save mode timeout\n");

		return ret;
	}

	ret = bcrm_write32(camera, BCRM_CSI2_CLOCK_32RW, camera->link_freq);
	if (ret) {
		avt_err(get_sd(camera), "restore mipi clock failed\n");
		return ret;
	}
	
	ret = bcrm_write8(camera, BCRM_CSI2_LANE_COUNT_8RW, camera->num_lanes);
	if (ret) {
		avt_err(get_sd(camera), "restore lane count failed\n");
		return ret;
	}

	ret = avt_update_format(camera, &camera->curr_rect,
				camera->curr_binning_info);
	if (ret) {
		avt_err(get_sd(camera), "update format failed\n");
		return ret;			
	}

	ret = avt_write_media_bus_format(camera,
					 avt_get_mode_fmt(camera)->code);
	if (ret) {
		avt_err(get_sd(camera), "set pixelformat failed\n");
		return ret;			
	}

	if (power_save_reset_controls) {
		ret = avt_reset_ctrls(camera);
	} else {
		// Set power save mode control to readonly while setting 
		// all controls as we are currently already in the s_ctrl 
		// handler and having it set again would casue the power save
		// mode to be activated again
		pwr_save_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

		ret = __v4l2_ctrl_handler_setup(&camera->v4l2_ctrl_hdl);

		pwr_save_ctrl->flags &= ~V4L2_CTRL_FLAG_READ_ONLY;
	}

	if (ret) {
		avt_err(get_sd(camera), "control setup failed %d\n", ret);
		return ret;
	}
	
	return 0;
}



static int __set_power_save_mode(struct avt_dev *camera,
				 struct v4l2_ctrl *pwr_save_ctrl)
{
	u64 start;
	int ret;
	u8 val = pwr_save_ctrl->val;

	if (camera->power_state == POWER_STATE_RESTORING)
		return 0;

	start = ktime_get_ns();
	ret = bcrm_write8(camera, BCRM_DEVICE_POWER_SAVE_MODE_32RW, val);
	if (ret < 0)
		return ret;

	if (camera->power_state == POWER_STATE_STANDBY && 
		val == AVT_POWER_SAVE_DISABLED) {

		u64 diff;

		camera->power_state = POWER_STATE_RESTORING;

		ret = return_from_power_save(camera, pwr_save_ctrl);
		if (ret)
			return ret;
		
		diff = ktime_get_ns() - start;
		avt_info(get_sd(camera),
			 "Return from power save mode took %llu us\n",
			 diff / 1000);
	}

	return 0;
}

static int __set_special_ctrl(struct avt_dev *camera, struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {	
	case AVT_CID_TRIGGER_MODE:
		return __set_trigger_mode(camera, ctrl->val);
	case AVT_CID_EXPOSURE_ACTIVE_LINE_MODE:
		return __set_exposure_active_mode(camera, ctrl->val);
	case AVT_CID_COLOR_TRANSFORM_MATRIX:
		return  __set_color_transform_matrix(camera,
						     ctrl->p_cur.p_s32,
						     ctrl->p_new.p_s32);
	case AVT_CID_USER_DATA_STORAGE: 
		return __set_user_data_ctrl(camera, ctrl->p_cur.p_u32,
					    ctrl->p_new.p_u32);
	case AVT_CID_FRAME_TRIGGER_WAIT_LINE_MODE:
		return __set_frame_trigger_wait_line_mode(camera, ctrl->val);
	case AVT_CID_POWER_SAVE_MODE:
		return __set_power_save_mode(camera, ctrl);
	// TODO: Move binning handling here from avt_ctrl_changed
	case AVT_CID_BINNING_SELECTOR: 
	case AVT_CID_EXPOSURE_ACTIVE_INVERT:
	case AVT_CID_FRAME_TRIGGER_WAIT_INVERT:
		return 0;
	default:
		return -ENOTTY;
	}
}

static bool __can_set_ctrl(struct avt_dev *camera, struct v4l2_ctrl *ctrl)
{
	// Power save mode control can always be changed
	if (ctrl->id == AVT_CID_POWER_SAVE_MODE)
		return true;

	if (camera->power_state == POWER_STATE_STANDBY)
		return false;

	return true;
}

static int avt_v4l2_ctrl_ops_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avt_dev *camera = container_of(ctrl->handler, struct avt_dev, v4l2_ctrl_hdl);
	struct i2c_client *client = camera->i2c_client;
	int ret = 0;

	if (camera->mode != AVT_BCRM_MODE) {
		return -EBUSY;
	}

	if (ctrl->priv != NULL)
	{
		const struct avt_ctrl_mapping * const ctrl_mapping = ctrl->priv;


		dev_dbg(&client->dev, "%s[%d]: Write custom ctrl %s (%x)\n",
			__func__, __LINE__, ctrl_mapping->name, ctrl->id);

		if (!__can_set_ctrl(camera, ctrl))
			goto done;

		ret = __set_special_ctrl(camera, ctrl);

		// Check if control was handled by __set_special_ctrl
		if (ret != -ENOTTY)
			goto done;

		if (!ctrl_mapping->reg_length)
			goto done;
		
		ret = write_ctrl_value(camera, ctrl, ctrl_mapping);
		
done:
		avt_ctrl_changed(camera,ctrl);
	}
	else
	{
		dev_err(&camera->i2c_client->dev,
			"%s[%d]: case default or not supported id %d, val %d\n",
			__func__, __LINE__, ctrl->id, ctrl->val);
		return -EINVAL;
	}

	return ret;
}

static const struct v4l2_ctrl_ops avt_ctrl_ops = {
	.g_volatile_ctrl = avt_g_volatile_ctrl,
	.s_ctrl = avt_v4l2_ctrl_ops_s_ctrl,
};


static int avt_fill_ctrl_config(struct avt_dev *camera,
				 struct v4l2_ctrl_config *config,
				 const struct avt_ctrl_mapping *mapping)
{
	int ret;



	config->ops = &avt_ctrl_ops;
	config->id = mapping->id;
	config->name = mapping->name;
	config->type = mapping->type;
	config->flags = mapping->flags;
	if (mapping->dims[0])
		memcpy(config->dims, mapping->dims, sizeof(config->dims));

	switch (mapping->type)
	{
	case V4L2_CTRL_TYPE_MENU:
		config->min = mapping->min_value;
		config->menu_skip_mask = 0;
		config->max = mapping->max_value;
		config->qmenu = mapping->qmenu;
		ret = read_control_value(camera, &config->def,
					 mapping->reg_offset,
					 mapping->reg_length);
		if (ret < 0)
			return ret;

		avt_ctrl_from_reg(mapping->id,&config->def);

		break;
	case V4L2_CTRL_TYPE_BOOLEAN:
		config->min = 0;
		config->max = 1;
		config->step = 1;
		if (!mapping->reg_offset) {
			config->def = mapping->default_value;
		} else {
			ret = read_control_value(camera, &config->def,
					 	 mapping->reg_offset,
					 	 mapping->reg_length);

			if (ret < 0)
				return ret;

			config->def = config->def ? 1 : 0;
		}
		break;
	case V4L2_CTRL_TYPE_INTEGER:
	case V4L2_CTRL_TYPE_INTEGER64:
	case V4L2_CTRL_TYPE_BITMASK:
	case V4L2_CTRL_TYPE_U8:
	case V4L2_CTRL_TYPE_U16:
	case V4L2_CTRL_TYPE_U32:
	case V4L2_CTRL_TYPE_STRING:
		if (!mapping->min_offset)
			config->min = mapping->min_value;
		else {
			ret = read_control_value(camera, &config->min,
						 mapping->min_offset,
						 mapping->reg_length);
			if (ret < 0)
				return ret;

			avt_ctrl_from_reg(mapping->id,&config->min);
		}

		if (!mapping->max_offset)
			config->max = mapping->max_value;
		else {
			ret = read_control_value(camera, &config->max,
						 mapping->max_offset,
						 mapping->reg_length);
			if (ret < 0)
				return ret;

			avt_ctrl_from_reg(mapping->id,&config->max);
		}

		if (!mapping->step_offset)
			config->step = mapping->step_value;
		else {
			ret = read_control_value(camera, &config->step,
						 mapping->step_offset,
						 mapping->reg_length);
			if (ret < 0)
				return ret;

			avt_ctrl_from_reg(mapping->id,&config->step);
		}

		if (!mapping->reg_offset)
			config->def = mapping->default_value;
		else {
			ret = read_control_value(camera, &config->def,
						 mapping->reg_offset,
						 mapping->reg_length);
			if (ret < 0)
				return ret;

			avt_ctrl_from_reg(mapping->id,&config->def);

			if (config->def < config->min) {
				config->def = config->min;
			}

			if (config->def > config->max) {
				config->def = config->max;
			}
		}

		break;
	default:
		break;
	}

	return 0;
}

static void avt_ctrl_added(struct avt_dev *camera,struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id)
	{
	case AVT_CID_TRIGGER_MODE: {	
		struct device *dev = &camera->i2c_client->dev;
		int ret;
		u8 val = 0; 

		ret = bcrm_read8(camera,
				 BCRM_FRAME_START_TRIGGER_MODE_8RW, &val);
		if (ret < 0)
			dev_err(dev, "Failed to update default value\n");

		__v4l2_ctrl_modify_range(ctrl, 0, 1, 1, val);
		*ctrl->p_cur.p_s32 = val;
		*ctrl->p_new.p_s32 = val;

		avt_update_sw_ctrl_state(camera);
		break;
	}
	case AVT_CID_TRIGGER_SOURCE: {
		struct v4l2_ctrl *mode_ctrl;

		mode_ctrl = avt_ctrl_find(camera, AVT_CID_TRIGGER_MODE);
		if (mode_ctrl) {
			__v4l2_ctrl_grab(ctrl, mode_ctrl->val);
		}

		avt_update_sw_ctrl_state(camera);
		ctrl->menu_skip_mask = BIT(AVT_TRIGGER_SOURCE_LINE2) 
				     | BIT(AVT_TRIGGER_SOURCE_LINE3);
		break;
	}
	case AVT_CID_TRIGGER_ACTIVATION: {
		struct v4l2_ctrl *mode_ctrl;

		mode_ctrl = avt_ctrl_find(camera, AVT_CID_TRIGGER_MODE);
		if (mode_ctrl) {
			__v4l2_ctrl_grab(ctrl, mode_ctrl->val);
		}

		break;
	}
	case AVT_CID_TRIGGER_SOFTWARE:
		avt_update_sw_ctrl_state(camera);
		break;
	case AVT_CID_FIRMWARE_VERSION: {
		const union device_firmware_version_reg *fw_version =
			&camera->cam_firmware_version;
		snprintf(ctrl->p_cur.p_char,ctrl->maximum + 1,
			"%02u.%02u.%02u.%08x",
			 fw_version->device_firmware.special_version,
			 fw_version->device_firmware.major_version,
			 fw_version->device_firmware.minor_version,
			 fw_version->device_firmware.patch_version);
		break;
	}
	case AVT_CID_CAMERA_NAME:  {
		snprintf(ctrl->p_cur.p_char,ctrl->maximum + 1,"%s %s",
			 camera->cci_reg.reg.family_name,
			 camera->cci_reg.reg.model_name);

		break;
	}
	case AVT_CID_SERIAL_NUMBER:  {
		strscpy(ctrl->p_cur.p_char,
			camera->cci_reg.reg.serial_number,
			ctrl->maximum + 1);

		break;
	}
	case AVT_CID_EXPOSURE_AUTO_MIN: {
		struct v4l2_ctrl *max_ctrl = NULL;

		max_ctrl = avt_ctrl_find(camera, AVT_CID_EXPOSURE_AUTO_MAX);

		if (max_ctrl == NULL)
			return;

		v4l2_ctrl_modify_range(ctrl,ctrl->minimum,
					 max_ctrl->default_value,ctrl->step,
					 ctrl->default_value);

		break;
	}
	case AVT_CID_EXPOSURE_AUTO_MAX: {
		struct v4l2_ctrl *min_ctrl = NULL;

		min_ctrl = avt_ctrl_find(camera, AVT_CID_EXPOSURE_AUTO_MIN);

		if (min_ctrl == NULL) {
			avt_warn(get_sd(camera),"V4L2_CID_EXPOSURE_AUTO_MIN not found!");
			return;
		}

		v4l2_ctrl_modify_range(ctrl,min_ctrl->default_value,
					 ctrl->maximum,ctrl->step,
					 ctrl->default_value);

		break;
	}
	case AVT_CID_GAIN_AUTO_MIN: {
		struct v4l2_ctrl *max_ctrl = NULL;

		max_ctrl = avt_ctrl_find(camera, AVT_CID_GAIN_AUTO_MAX);

		if (max_ctrl == NULL)
			return;

		v4l2_ctrl_modify_range(ctrl,ctrl->minimum,
				       max_ctrl->default_value,ctrl->step,
				       ctrl->default_value);

		break;
	}
	case AVT_CID_GAIN_AUTO_MAX: {
		struct v4l2_ctrl *min_ctrl = NULL;

		min_ctrl = avt_ctrl_find(camera, AVT_CID_GAIN_AUTO_MIN);

		if (min_ctrl == NULL) {
			avt_warn(get_sd(camera),"V4L2_CID_EXPOSURE_AUTO_MIN not found!");
			return;
		}

		v4l2_ctrl_modify_range(ctrl,min_ctrl->default_value,
				       ctrl->maximum,ctrl->step,
				       ctrl->default_value);

		break;
	}
	case V4L2_CID_TEST_PATTERN: {
		int ret;
		u32 inq;

		ret = bcrm_read32(camera, BCRM_TEST_PATTERN_INQ_32R, &inq);
		if (ret < 0) {
			break;
		}

		ctrl->menu_skip_mask = ((~inq) << 1);

		break;
	}
	case AVT_CID_EXPOSURE_ACTIVE_LINE_MODE: {
		struct device *dev = &camera->i2c_client->dev;
		int ret;
		u8 val = 0; 

		ret = bcrm_read8(camera,
				 BCRM_EXPOSURE_ACTIVE_LINE_MODE_8RW, &val);
		if (ret < 0)
			dev_err(dev, "Failed to update default value\n");

		__v4l2_ctrl_modify_range(ctrl, 0, 1, 1, val);
		*ctrl->p_cur.p_s32 = val;
		*ctrl->p_new.p_s32 = val;
		break;
	}
	case AVT_CID_FRAME_TRIGGER_WAIT_LINE_MODE: {
		struct device *dev = &camera->i2c_client->dev;
		int ret;
		u8 val = 0; 

		ret = bcrm_read8(camera,
				 BCRM_FRAME_TRIGGER_WAIT_LINE_MODE_8RW, &val);
		if (ret < 0)
			dev_err(dev, "Failed to update default value\n");

		__v4l2_ctrl_modify_range(ctrl, 0, 1, 1, val);
		*ctrl->p_cur.p_s32 = val;
		*ctrl->p_new.p_s32 = val;
		break;
	}	
	case AVT_CID_FRAME_TRIGGER_WAIT_OUTPUT_LINE: {
		struct v4l2_ctrl *mode_ctrl;

		mode_ctrl = avt_ctrl_find(camera,
					  AVT_CID_FRAME_TRIGGER_WAIT_LINE_MODE);
		if (mode_ctrl) {
			__v4l2_ctrl_grab(ctrl, mode_ctrl->val);
		}

		ctrl->menu_skip_mask = BIT(AVT_FRAME_TRIGGER_WAIT_OUTPUT_LINE2)
				     | BIT(AVT_FRAME_TRIGGER_WAIT_OUTPUT_LINE3);
		break;
	}
	case AVT_CID_FRAME_TRIGGER_WAIT_INVERT: {
		struct v4l2_ctrl *mode_ctrl;

		mode_ctrl = avt_ctrl_find(camera,
					  AVT_CID_FRAME_TRIGGER_WAIT_LINE_MODE);
		if (mode_ctrl) {
			__v4l2_ctrl_grab(ctrl, mode_ctrl->val);
		}

		break;
	}
	case AVT_CID_COLOR_TRANSFORM_MATRIX: {
		int i, ret;
		s32 *val = ctrl->p_cur.p_s32;
		u32 tmp;

		for (i = 0; i < BCRM_COLOR_TRANSFORM_MATRIX_SIZE; i++, val++) {
			if ((i % 2) == 0) {
				u16 reg = BCRM_COLOR_TRANSFORM_MATRIX_0_1_32RW 
					  + (i / 2) * sizeof(tmp);
				ret = bcrm_read32(camera, reg, &tmp);
				if (ret < 0)
					break;

				*val = ((s16)(tmp & 0xFFFF));
			} else {
				*val = ((s16)((tmp >> 16) & 0xFFFF));
			}
		}

		break;
	}
	case AVT_CID_REVISION_ID: {
		int ret;
		char revid[3] = {};

		ret = bcrm_read16(camera, BCRM_REVISION_ID_16R, (u16*)revid);
		if (ret < 0)
			break;

		strscpy(ctrl->p_cur.p_char, revid, ctrl->maximum + 1);

		break;
	}	
	case AVT_CID_USER_DATA_STORAGE: {
		int i, ret;
		u32 *ptr = ctrl->p_cur.p_u32;

		for (i = 0; i < BCRM_USER_DATA_INDEX_COUNT; i++, ptr++) {
			ret = bcrm_write8(camera, BCRM_USER_DATA_INDEX_8RW, i);
			if (ret < 0)
				break;
			
			ret = bcrm_read32(camera, BCRM_USER_DATA_VALUE_32RW,
					  ptr);
		}

		break;
	}	
	case AVT_CID_SENSOR_ID: {
		int ret; 
		u32 val;

		ret = bcrm_read32(camera, BCRM_SENSOR_IDENTIFICATION_32R, &val);
		if (ret < 0) 
			break;

		val = FIELD_GET(BCRM_SENSOR_IDENTIFICATION_SENSOR_ID, val);

		*ctrl->p_cur.p_s32 = val;
		*ctrl->p_new.p_s32 = val;


		break;
	}
	case AVT_CID_SENSOR_PLATFORM_ID: {
		int ret; 
		u32 val;

		ret = bcrm_read32(camera, BCRM_SENSOR_IDENTIFICATION_32R, &val);
		if (ret < 0) 
			break;

		val = FIELD_GET(BCRM_SENSOR_IDENTIFICATION_SENSOR_PLATFORM_ID,
				val);
		
		*ctrl->p_cur.p_s32 = val;
		*ctrl->p_new.p_s32 = val;

		break;
	}
	case AVT_CID_SENSOR_FLAGS: {
		int ret; 
		u32 val;

		ret = bcrm_read32(camera, BCRM_SENSOR_IDENTIFICATION_32R, &val);
		if (ret < 0) 
			break;

		val = FIELD_GET(BCRM_SENSOR_IDENTIFICATION_SENSOR_INFO, val);
		
		*ctrl->p_cur.p_s32 = val;
		*ctrl->p_new.p_s32 = val;

		break;
	}
	default:
		break;
	}
}

static int avt_init_controls(struct avt_dev *camera)
{
	struct v4l2_subdev *sd = get_sd(camera);
	struct v4l2_ctrl_handler *hdl = &camera->v4l2_ctrl_hdl;
	struct v4l2_ctrl_config config;
	struct v4l2_ctrl *ctrl;
	int ret;
	int i, j;

	avt_dbg(sd, "code uses now v4l2_ctrl_new_std and v4l2_query_ext_ctrl (VIDIOC_QUERY_EXT_CTRL / s64) ");

	ret = v4l2_ctrl_handler_init(hdl, ARRAY_SIZE(avt_ctrl_mappings));
	if (ret < 0)
	{
		avt_err(sd, "v4l2_ctrl_handler_init Failed");
		goto free_ctrls;
	}
	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &camera->lock;

	for (i = 0, j = 0; j < ARRAY_SIZE(avt_ctrl_mappings); ++j)
	{
		const struct avt_ctrl_mapping * const ctrl_mapping
			= &avt_ctrl_mappings[j];
		const u64 mask = ctrl_mapping->inq_mask;
		const u64 inq_reg = camera->feature_inquiry_reg.value;

		if (mask && ((inq_reg & mask) == 0)) {
			avt_info(sd, "%s not supported\n", 
				ctrl_mapping->name);
			continue;
		}

		CLEAR(config);

		avt_dbg(sd, "Init ctrl %s (0x%x)\n",
			 ctrl_mapping->name,ctrl_mapping->id);


		avt_fill_ctrl_config(camera,&config,ctrl_mapping);

		ctrl = v4l2_ctrl_new_custom(hdl, &config,(void*)ctrl_mapping);

		if (ctrl == NULL)
		{
			avt_err(sd, "Failed to init %s ctrl %d 0x%08x\n",
				config.name, hdl->error, hdl->error);

			if (hdl->error == -ERANGE) {
				avt_err(sd,
					"Invalid ctrl range min: %lld max: %lld "
					"step: %lld def: %lld",
					config.min,config.max,config.step,config.def);
			}

	    		//Clear error
			hdl->error = 0;
			continue;
		}


		avt_ctrl_added(camera, ctrl);

		camera->avt_ctrls[i] = ctrl;
		i++;
	}

	ctrl = v4l2_ctrl_new_int_menu(hdl, &avt_ctrl_ops, V4L2_CID_LINK_FREQ, 
				      0, 0, &camera->link_freq);

	if (ctrl) 
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	

	return ret;
free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static void set_frameinterval(struct v4l2_fract *interval,const u64 framerate)
{
	const u64 factor = UHZ_TO_HZ;

	interval->denominator = (framerate * interval->numerator) / factor;

	// If the denominator and minimal framerate is not zero, try to increase the numerator by 1000
	while (interval->denominator == 0 && interval->numerator < factor)
	{
		interval->numerator *= 1000;
		interval->denominator = (framerate * interval->numerator) / factor;
	}
}

static inline u64 frame_interval_to_rate_uhz(struct v4l2_fract *interval)
{
	const u64 fac = UHZ_TO_HZ;
	return mult_frac(fac, interval->denominator, interval->numerator);
}

static int avt_pad_ops_enum_frame_size(struct v4l2_subdev *sd,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
	struct v4l2_subdev_state *sd_state,
#else
	struct v4l2_subdev_pad_config *cfg,
#endif
	struct v4l2_subdev_frame_size_enum *fse)
{
	struct avt_dev *camera = to_avt_dev(sd);
	const struct v4l2_rect *min = &camera->min_rect;
	const struct v4l2_rect *max = &camera->sensor_rect;
	struct avt_binning_info *binning_info;
	struct v4l2_rect binning_rect,scaled_crop = camera->curr_rect;
	size_t max_frame_size;

	avt_dbg(sd, "fse->index %d, fse->which %s", fse->index,
		fse->which == V4L2_SUBDEV_FORMAT_TRY ? "V4L2_SUBDEV_FORMAT_TRY" : "V4L2_SUBDEV_FORMAT_ACTIVE");

	if (fse->pad != 0)
	{
		avt_warn(sd, "Requested pad %d not supported",fse->pad);
		return -EINVAL;
	}

#ifdef ENABLE_STEPWISE_IMAGE_SIZE
	max_frame_size = camera->binning_info_cnt[camera->curr_binning_type];

	if (fse->index >= max_frame_size)
	{
		avt_dbg(get_sd(camera), "fse->index(%d) >= %lu.",
			 fse->index, max_frame_size);
		return -EINVAL;
	}

	binning_info
		= &camera->binning_infos[camera->curr_binning_type][fse->index];


	binning_rect.width = binning_info->max_width;
	binning_rect.height =  binning_info->max_height;

	v4l2_rect_scale(&scaled_crop,max,&binning_rect);

	v4l_bound_align_image(&scaled_crop.width,min->width,
			      binning_rect.width,3,
			      &scaled_crop.height,min->height,
			      binning_rect.height,3,0);


	fse->min_width = scaled_crop.width;
	fse->max_width = scaled_crop.width;
	fse->min_height = scaled_crop.height;
	fse->max_height = scaled_crop.height;
#else
	if (fse->index >= 1)
	{
		avt_dbg(sd_of(camera), "fse->index(%d) >= 1.", fse->index);
		return -EINVAL;
	}
	fse->min_width = camera->min_rect.width;
	fse->max_width = camera->max_rect.width;
	fse->min_height = camera->min_rect.height;
	fse->max_height = camera->max_rect.height;

#endif
	return 0;
}

static int avt_pad_ops_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct avt_dev *camera = to_avt_dev(sd);
	bool is_auto = camera->framerate_auto;
	u32 width = fie->width;
	u32 height = fie->height;
	const struct avt_binning_info *new_binning;
	u64 framerate;
	int ret;

	if (fie->pad != 0)
		return -EINVAL;

	if (fie->index > AVT_FRAME_INTERVAL_MAXIMUM_INDEX)
		return -EINVAL;

	ret = lookup_media_bus_format_index(camera, fie->code);
	if (ret < 0)
		return ret;
	

	// Get matching binning config for requested resolution
	avt_calc_compose(camera, &camera->curr_rect, &width, &height,
			 &new_binning);

	if (fie->width != width || fie->height != height)
	{
		avt_err(get_sd(camera),
			"width (%u) or height (%u) not supported",
			fie->width, fie->height);
		return -EINVAL;
	}

	if (fie->index == AVT_FRAME_INTERVAL_CURRENT_INDEX && !is_auto) {
		fie->interval = camera->frame_interval;
	} else {
		ret = bcrm_read64(camera,
			BCRM_ACQUISITION_FRAME_RATE_MAX_64R,
			&framerate);

		if (ret < 0)
			return ret;

		fie->interval.numerator = 1000;
		set_frameinterval(&fie->interval, framerate);	
	}
	
	return 0;
}

static int avt_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fi)
{
	struct avt_dev *camera = to_avt_dev(sd);
	int ret = 0;

	mutex_lock(&camera->lock);

	if (avt_trigger_mode_enabled(camera)) {
		ret = -EINVAL;
		goto exit;
	}

	fi->interval = camera->frame_interval;

exit:
	mutex_unlock(&camera->lock);

	return ret;
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
static int avt_get_frame_interval(struct v4l2_subdev *sd, 
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_interval *fi)
{
	if (fi->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_fract *interval;
		
		if (!state->sd)
			state->sd = sd;
		interval = v4l2_subdev_state_get_interval(state, fi->pad);

		fi->interval = *interval;

		return 0;
	}

	return avt_g_frame_interval(sd, fi);
}
#endif

static struct v4l2_fract *
avt_get_pad_interval(struct avt_dev *camera,
		     struct v4l2_subdev_state *state,
		     u32 pad, u32 which)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	if (which == V4L2_SUBDEV_FORMAT_TRY) {
		if (!state->sd)
			state->sd = get_sd(camera);
		return v4l2_subdev_state_get_interval(state, pad);
	}
#endif

	return &camera->frame_interval;
}

static int __avt_set_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_frame_interval *fi,
				    u32 pad, u32 which)					   
{
	struct avt_dev *camera = to_avt_dev(sd);
	int ret = 0;
	u64 framerate_req, framerate_min, framerate_max;
	struct v4l2_fract *interval;

	interval = avt_get_pad_interval(camera, state, pad, which);

	avt_dbg(sd, "fie->num %d fie->denom %d",
			fi->interval.numerator, fi->interval.denominator);
	

	mutex_lock(&camera->lock);
	if (which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (camera->is_streaming)
		{
			ret = -EBUSY;
			goto out;
		}

		if (avt_trigger_mode_enabled(camera)) {
			ret = -EINVAL;
			goto out;
		}
	}

	// For now block frame rate changes in the power save mode,
	// because their is no way to determine the limits
	if (camera->power_state != POWER_STATE_ACTIVE) {
		ret = -EPERM;
		goto out;
	}

	ret = bcrm_read64(camera,BCRM_ACQUISITION_FRAME_RATE_MIN_64R,
			  &framerate_min);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
		goto out;
	}

	ret = bcrm_read64(camera,BCRM_ACQUISITION_FRAME_RATE_MAX_64R,
			  &framerate_max);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
		goto out;
	}

	if (fi->interval.numerator == 0 || fi->interval.denominator == 0) {
		if (which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			camera->framerate_auto = true;
		}
	}
	else {
		framerate_req = frame_interval_to_rate_uhz(&fi->interval);
		framerate_req = clamp(framerate_req, 
				      framerate_min,
				      framerate_max);

		set_frameinterval(&fi->interval, framerate_req);

		if (which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			camera->framerate_auto = false;
		}
	}

	*interval = fi->interval;

	avt_dbg(sd, "set fie->num %d fie->denom %d",
			fi->interval.numerator, fi->interval.denominator);

out:
	mutex_unlock(&camera->lock);
	
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
static int avt_s_frame_interval(struct v4l2_subdev *sd, 
				struct v4l2_subdev_frame_interval *fi)
{
	return __avt_set_frame_interval(sd, NULL, fi, fi->pad,
					V4L2_SUBDEV_FORMAT_ACTIVE);
}
#else
static int avt_set_frame_interval(struct v4l2_subdev *sd, 
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_interval *fi)
{
	return __avt_set_frame_interval(sd, state, fi, fi->pad, fi->which);
}
#endif

static int avt_pad_ops_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	struct avt_dev *camera = to_avt_dev(sd);
	struct i2c_client *client = camera->i2c_client;

	if (NULL == code)
	{
		dev_warn(&client->dev, "%s[%d]: code == NULL", __func__, __LINE__);
		return -EINVAL;
	}
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
	if (NULL == sd_state)
	{
		dev_warn(&client->dev, "%s[%d]: sd_state == NULL", __func__, __LINE__);
	}
#else
	if (NULL == cfg)
	{
		dev_warn(&client->dev, "%s[%d]: cfg == NULL", __func__, __LINE__);
	}
#endif

	if (code->pad != 0)
	{
		dev_warn(&client->dev, "%s[%d]: code->pad != 0 fse->index %d, code 0x%04X camera->available_fmts_cnt %d",
				 __func__, __LINE__, code->index, code->code, camera->available_fmts_cnt);

		return -EINVAL;
	}

	if (code->index >= camera->available_fmts_cnt)
	{
		dev_warn(&client->dev, "%s[%d]: code->index >= camera->available_fmts_cnt fse->index %d, code 0x%04X camera->available_fmts_cnt %d",
				 __func__, __LINE__, code->index, code->code, camera->available_fmts_cnt);
		return -EINVAL;
	}

	code->code = camera->available_fmts[code->index].mbus_code;

	return 0;
}

static void avt_controls_stream_grab(struct avt_dev *camera,bool grabbed)
{
	int i;

	for (i = 0;i < AVT_MAX_CTRLS;i++)
	{
		struct v4l2_ctrl *ctrl = camera->avt_ctrls[i];

		if (ctrl && ctrl->priv)
		{
			const struct avt_ctrl_mapping * const ctrl_mapping = ctrl->priv;

			if (ctrl_mapping->avt_flags &
			    AVT_CTRL_FLAG_STREAM_DISABLED)
			{
				__v4l2_ctrl_grab(ctrl,grabbed);
			}
		}
	}
}

static inline int set_auto_framerate(struct avt_dev *camera, bool enabled)
{
	const u8 val = enabled ? 0 : 1;
	return bcrm_write8(camera, BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW, val);
}

static int write_framerate(struct avt_dev *camera)
{
	int ret = 0;

	if (camera->framerate_auto) {
		ret = set_auto_framerate(camera, true);
	} else {
		struct v4l2_fract *interval = &camera->frame_interval;
		u64 rate_uhz = frame_interval_to_rate_uhz(interval);

		ret = set_auto_framerate(camera, false);
		if (unlikely(ret)) {
			goto exit;
		}

		ret = bcrm_write64(camera, BCRM_ACQUISITION_FRAME_RATE_64RW, 
			rate_uhz);
	}

exit:
	return ret;
}

static int avt_video_ops_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct avt_dev *camera = to_avt_dev(sd);
	struct i2c_client *client = camera->i2c_client;
	int ret = 0;
	if (camera->flash_sd) {
		ret = v4l2_subdev_call(camera->flash_sd, video, s_stream, enable);
		if (ret && ret != -ENOIOCTLCMD)
			return ret;
	}

	if (camera->mode == AVT_GENCP_MODE)
		return 0;

	mutex_lock(&camera->lock);

	if (!enable && camera->is_streaming)
	{
		ret = bcrm_write8(camera, BCRM_ACQUISITION_STOP_8RW, 1);
		camera->is_streaming = false;

		// ToDo: eventually wait until cam has stopped streaming
	}

	if (enable && !camera->is_streaming)
	{
		struct v4l2_rect crop_rect = camera->curr_rect;
		struct v4l2_rect binning_rect = {0};
		const struct avt_binning_info *binning_info = camera->curr_binning_info;

		if (camera->power_state != POWER_STATE_ACTIVE) {
			ret = -EBUSY;
			goto out;
		}
			

		binning_rect.width = binning_info->max_width;
		binning_rect.height = binning_info->max_height;

		v4l2_rect_scale(&crop_rect,&camera->sensor_rect,&binning_rect);


		v4l_bound_align_image(&crop_rect.width,camera->min_rect.width,
				      binning_rect.width,3,
				      &crop_rect.height,camera->min_rect.height,
				      binning_rect.height,3,0);

		if (!avt_trigger_mode_enabled(camera)) {
			ret = write_framerate(camera);
			if (unlikely(ret))
				goto out;
		}

		if (debug >= 2)
			bcrm_dump(client);

		if (camera->stream_start_phy_reset) {
			avt_dphy_reset(camera,1);

			usleep_range(100,1000);

			avt_dphy_reset(camera,0);
		}

		/* start streaming */
		ret = bcrm_write8(camera, BCRM_ACQUISITION_START_8RW, 1);

		// ToDo: probably it's better to check the status here. but this conflicts with the workaround for imx8mp delayed start
		if (!ret)
			camera->is_streaming = enable;
	}

	avt_controls_stream_grab(camera,enable);

out:
	mutex_unlock(&camera->lock);

	return ret;
}

static int avt_core_ops_reset(struct v4l2_subdev *sd, u32 val)
{
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int avt_core_ops_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct avt_dev *camera = to_avt_dev(sd);
	int ret = 0;
	adev_info(sd->dev, "Register = %llu\n", reg->reg);

	if (reg->reg & ~0xffff)
			return -EINVAL;

	if (reg->size != 1 && reg->size != 2 &&
		reg->size != 4 && reg->size != 8)
	{
		ret = -EINVAL;
	}

	ret = avt_read(camera, reg->reg, &reg->val, reg->size);

	return ret;
}

static int avt_core_ops_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	adev_info(&client->dev, "reg 0x%04llX, size %u", reg->reg, reg->size);

	return 0;
}
#endif // CONFIG_VIDEO_ADV_DEBUG

static int avt_core_ops_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
					struct v4l2_event_subscription *sub)
{
	avt_dbg(sd, "event type %u", sub->type);

	switch (sub->type)
	{
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	case AVT_V4L2_EVENT_PIXELFORMAT_CHANGE:
    		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return -EINVAL;
	}
}

static inline const char *log_val(u32 val, u32 mask, bool avail)
{
	if (!avail)
		return "n/a";

	return (val & mask) ? "true" : "false";
}

static inline void print_status(struct device *dev, const char *str,
				u32 val, u32 mask, bool avail)
{
	dev_info(dev, "%s = %s\n", str, log_val(val, mask, avail));
}

static int avt_log_status(struct v4l2_subdev *sd)
{
	struct avt_dev *camera = to_avt_dev(sd);
	struct device *dev = sd->dev;
	u8 acq_active;
	u32 val;
	int ret;
	bool has_device_status, has_sensor_indent;

	has_device_status = test_feature_inq(camera, DEVICE_STATUS);
	has_sensor_indent = test_feature_inq(camera, SENSOR_IDENTIFICATION);

	ret = bcrm_read32(camera, BCRM_DEVICE_STATUS_32R, &val);
	if (ret < 0)
		return ret;

	dev_info(dev, "**** Device status ****\n");
	
	print_status(dev, "Backend buffer okay", val,
		     BCRM_DEVICE_STATUS_BACKEND_BUFFER_OKAY,
		     has_device_status);
	
	print_status(dev, "Mainboard temperature okay", val,
		     BCRM_DEVICE_STATUS_MAINBOARD_TEMPERATURE_OKAY, 
		     has_device_status);

	print_status(dev, "Stream ready", val,
		     BCRM_DEVICE_STATUS_STREAM_READY,
		     has_device_status);	

	print_status(dev, "MIPI Phy okay", val,
		     BCRM_DEVICE_STATUS_MIPI_PHY_OKAY,
		     has_device_status);	

	print_status(dev, "Sensorboard temperature okay", val,
		     BCRM_DEVICE_STATUS_SENSORBOARD_TEMPERATURE_OKAY,
		     has_device_status);
	
	print_status(dev, "Sensor communication okay", val,
		     BCRM_DEVICE_STATUS_SENSOR_COMMUNINICATION_OKAY,
		     has_device_status);		

	ret = bcrm_read8(camera, BCRM_ACQUISITION_STATUS_8R, &acq_active);
	if (ret < 0)
		return ret;

	dev_info(dev, "**** Acqusition status ****\n");
	dev_info(dev, "Acqusition active = %s\n",
		 acq_active ? "true" : "false");

	ret = bcrm_read32(camera, BCRM_SENSOR_IDENTIFICATION_32R, &val);
	if (ret < 0)
		return ret;

	dev_info(dev, "**** Sensor status ****");
	print_status(dev, "Sensor detection okay", val,
		     BCRM_SENSOR_IDENTIFICATION_SENSOR_DETECT_OK,
		     has_sensor_indent);
	
	print_status(dev, "Sensor fits firmware", val,
		     BCRM_SENSOR_IDENTIFICATION_SENSOR_FITS_FIRMWARE,
		     has_sensor_indent);

	return 0;
}

// Provide dummy implementation for the s_power core operations 
// as some platform driver require the call to succeed.
// This applies for the imx8-isi driver.
static int avt_s_power(struct v4l2_subdev *sd,int on) 
{
	return 0;
}

static const struct v4l2_subdev_core_ops avt_core_ops = {
	.log_status = avt_log_status,
	.reset = avt_core_ops_reset,
	.subscribe_event = avt_core_ops_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.s_power = avt_s_power,

#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = avt_core_ops_g_register,
	.s_register = avt_core_ops_s_register,
#endif
};

static int avt_subdev_internal_ops_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct avt_dev *camera = to_avt_dev(sd);
	int ret = 0;

	avt_dbg(sd, "camera->open_refcnt %d, camera->is_streaming %d",
			camera->open_refcnt, camera->is_streaming);

	// stop the stream if just streaming
	if (camera->is_streaming)
	{
		avt_err(sd, "camera->is_streaming %d",
				camera->is_streaming);
		// ret = avt_video_ops_s_stream(sd, false);
	}

	camera->open_refcnt--;
	return ret;
}
//TODO: Support multiple opens
static int avt_subdev_internal_ops_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	// called when userspace app calls 'open'
	struct avt_dev *camera = to_avt_dev(sd);

	avt_dbg(sd, "camera->open_refcnt %d", camera->open_refcnt);

	if (camera->open_refcnt)
	{
		avt_dbg(sd, "device already opened %d", camera->open_refcnt);
		return -EBUSY;
	}

	if (!camera->is_streaming)
	{
		avt_dbg(sd, "force bcrm mode");
		// set BCRM mode only when camera is not streaming
		mutex_lock(&camera->lock);

		avt_change_mode(camera, AVT_BCRM_MODE);

		mutex_unlock(&camera->lock);
	}

	camera->open_refcnt++;

	return 0;
}

static const struct v4l2_subdev_internal_ops avt_subdev_internal_ops = {
	.open = avt_subdev_internal_ops_open,
	.close = avt_subdev_internal_ops_close,
};

static const struct v4l2_subdev_video_ops avt_video_ops = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8 , 0)
	.g_frame_interval = avt_g_frame_interval,
	.s_frame_interval = avt_s_frame_interval,
#endif
	.s_stream = avt_video_ops_s_stream,
};

static void avt_get_compose(struct avt_dev *camera,
		   struct v4l2_subdev_state *sd_state,
		   struct v4l2_subdev_selection *sel)
{
	const struct v4l2_mbus_framefmt *frmfmt;

	frmfmt = avt_get_pad_fmt(camera, sd_state, sel->pad, sel->which);

	sel->r.left = 0;
	sel->r.top = 0;
	sel->r.width = frmfmt->width;
	sel->r.height = frmfmt->height;
}

static void avt_get_crop(struct avt_dev * camera,
		   struct v4l2_subdev_state *sd_state,
		   struct v4l2_subdev_selection *sel)
{
	const struct v4l2_rect *rect;

	rect = avt_get_pad_crop(camera, sd_state, sel->pad, sel->which);

	sel->r = *rect;
}

static int avt_pad_ops_get_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_selection *sel)
{
	struct avt_dev *camera = to_avt_dev(sd);

	if (sel->pad > 0)
		return -EINVAL;

	//No cropping or binning in genicam for csi2 mode
	if (camera->mode == AVT_GENCP_MODE)
		return -ENODATA;

	switch (sel->target)
	{
	/* Composing bounds */
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	/* Default composing area */
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		v4l2_rect_set_size_to(&sel->r,&camera->curr_rect);
		break;
	/* Current composing area */
	case V4L2_SEL_TGT_COMPOSE:
		avt_get_compose(camera,sd_state,sel);
		break;

	/* Current cropping area */
	case V4L2_SEL_TGT_CROP:
		avt_get_crop(camera,sd_state,sel);
		break;

	/* Cropping bounds */
	case V4L2_SEL_TGT_CROP_BOUNDS:
	/* Default cropping area */
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = camera->max_rect;
		break;
	/* Native frame size */
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = camera->sensor_rect;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int avt_set_compose(struct avt_dev *camera,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_selection *sel)
{
	int ret = 0;
	struct v4l2_mbus_framefmt *frmfmt;
	const struct avt_binning_info *info;
	const struct v4l2_rect *crop;

	crop = avt_get_pad_crop(camera, sd_state, sel->pad, sel->which);
	frmfmt = avt_get_pad_fmt(camera, sd_state, sel->pad, sel->which);

	sel->r.left = 0;
	sel->r.top = 0;

	avt_calc_compose(camera,crop,&sel->r.width,&sel->r.height,&info);

	if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = avt_update_format(camera, crop, info);
		if (ret < 0)
			goto exit;
	}

	frmfmt->width = sel->r.width;
	frmfmt->height = sel->r.height;
		
exit: 
	return ret;
}

static int __set_crop(struct avt_dev *camera, struct v4l2_rect *rect,
		      struct v4l2_mbus_framefmt *frmfmt, struct v4l2_rect *crop,
		      unsigned int which)
{
	
	const struct v4l2_rect *min = &camera->min_rect;
	const struct v4l2_rect *max = &camera->max_rect;
	const struct avt_binning_info *info;
	u32 width = max->width, height = max->height;

	v4l_bound_align_image(&rect->width, min->width, max->width,3,
			      &rect->height, min->height, max->height,3,0);

	v4l2_rect_map_inside(rect, max);

	avt_calc_compose(camera, rect, &width, &height, &info);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		int ret;
		ret = avt_update_format(camera, rect, info);
		if (ret < 0)
			return ret;
	}

	frmfmt->width = width;
	frmfmt->height = height;

	*crop = *rect;

	return 0;
}

static int avt_set_crop(struct avt_dev *camera,
			 struct v4l2_subdev_state *sd_state,
			 struct v4l2_subdev_selection *sel)
{
	struct v4l2_rect *crop;
	struct v4l2_mbus_framefmt *frmfmt;
	
	crop = avt_get_pad_crop(camera, sd_state, sel->pad, sel->which);
	frmfmt = avt_get_pad_fmt(camera, sd_state, sel->pad, sel->which);

	return __set_crop(camera, &sel->r, frmfmt, crop, sel->which);;
}

static int avt_pad_ops_set_selection(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_selection *sel)
{
	struct avt_dev *camera = to_avt_dev(sd);
	int ret = -EINVAL;

	avt_dbg(sd, "set selection tgt: %d, which: %d, rect: (%u, %u)/%ux%u",
		sel->target, sel->which, sel->r.left, sel->r.top,
		sel->r.width, sel->r.height);

	if (camera->is_streaming && sel->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EBUSY;

	if (sel->pad > 0)
		return -EINVAL;

	//No cropping or binning in genicam for csi2 mode
	if (camera->mode == AVT_GENCP_MODE)
		return -EINVAL;

	mutex_lock(&camera->lock);

	if (sel->target == V4L2_SEL_TGT_CROP)
		ret = avt_set_crop(camera,sd_state, sel);
	else if (sel->target == V4L2_SEL_TGT_COMPOSE)
		ret = avt_set_compose(camera,sd_state,sel);

	mutex_unlock(&camera->lock);

	return ret;
}

static int avt_pad_ops_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				      struct v4l2_mbus_frame_desc *fd)
{
	struct avt_dev *camera = to_avt_dev(sd);
	const int code = avt_get_mode_fmt(camera)->code;
	const struct avt_csi_mipi_mode_mapping *fmt_mapping;
	int idx = lookup_media_bus_format_index(camera, code);
	
	fmt_mapping = &camera->available_fmts[idx];

	fd->num_entries = 1;
	
	fd->entry[0].pixelcode = code;
	
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 18, 0))
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = fmt_mapping->mipi_fmt;
#endif

	return 0;
}

#ifdef CONFIG_MEDIA_CONTROLLER
static int avt_pad_ops_link_validate(struct v4l2_subdev *sd, struct media_link *link,
							   struct v4l2_subdev_format *source_fmt,
							   struct v4l2_subdev_format *sink_fmt)
{

	v4l2_dbg(2, debug, sd, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);
	return 0;
}
#endif /* CONFIG_MEDIA_CONTROLLER */

static const struct v4l2_subdev_pad_ops avt_pad_ops = {
	.enum_mbus_code = avt_pad_ops_enum_mbus_code,
	.enum_frame_size = avt_pad_ops_enum_frame_size,
	.enum_frame_interval = avt_pad_ops_enum_frame_interval,
	.get_fmt = avt_pad_ops_get_fmt,
	.set_fmt = avt_pad_ops_set_fmt,
	.get_selection = avt_pad_ops_get_selection,
	.set_selection = avt_pad_ops_set_selection,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0))
	.g_mbus_config = v4l2_subdev_video_ops_g_mbus_config,
	.s_mbus_config = v4l2_subdev_video_ops_s_mbus_config,
#endif
	.get_frame_desc = avt_pad_ops_get_frame_desc,
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = avt_pad_ops_link_validate,
#endif /* CONFIG_MEDIA_CONTROLLER */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0)
	.get_frame_interval = avt_get_frame_interval,
	.set_frame_interval = avt_set_frame_interval,
#endif
};
static const struct v4l2_subdev_ops avt_subdev_ops = {
	.core = &avt_core_ops,
	.video = &avt_video_ops,
	.pad = &avt_pad_ops,
};

static int avt_meo_link_setup(struct media_entity *entity,
							   const struct media_pad *local,
							   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations avt_sd_media_ops = {
	.link_setup = avt_meo_link_setup,
};


static int avt_get_camera_capabilities(struct v4l2_subdev *sd)
{
	struct avt_dev *camera = to_avt_dev(sd);
	int ret = 0;

	u64 value64;
	u8 avt_supported_lane_mask = 0;
	u32 avt_current_clk = 0;
	u8 current_mode;
	u32 temp;

	ret = avt_read8(camera, GENCP_CHANGEMODE_8W, &current_mode);

	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> camera mode (%d)\n", ret);
		return ret;
	}

	if (current_mode != AVT_BCRM_MODE)
	{
		avt_err(sd, "Camera not in BCRM mode\n");
		return -ENOTSUPP;
	}

	/* reading the Feature inquiry register */
	ret = bcrm_read64(camera, BCRM_FEATURE_INQUIRY_64R,
		&camera->feature_inquiry_reg.value);

	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> feature inquiry (%d)\n", ret);
		return ret;
	}
	avt_dbg(sd, "BCRM_FEATURE_INQUIRY_64R %llu\n", camera->feature_inquiry_reg.value);

	/* Check if requested number of lanes is supported */
	ret = bcrm_read8(camera, BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R,
		&avt_supported_lane_mask);
	
	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> csi2 supported lane counts (%d)\n", ret);
		return ret;
	}

	camera->lane_capabilities.value = avt_supported_lane_mask;

	avt_dbg(sd, "supported lane config: %x", (uint32_t)avt_supported_lane_mask);

	// To avoid any issues when num_lanes is 0, the lane count mask is left
	// shifted by 1 as bit 0 equals a lane count of 1 in the register
	if (!((avt_supported_lane_mask << 1) & BIT(camera->num_lanes)))
	{
		avt_err(sd, "requested number of lanes (%u) not supported by camera!\n",
				camera->num_lanes);
		return -EINVAL;
	}

	/* Set number of lanes */
	ret = bcrm_write8(camera, BCRM_CSI2_LANE_COUNT_8RW, camera->num_lanes);
	
	if (ret < 0)
	{
		avt_err(sd, "Reg write failed -> csi2 lane count (%d)\n", ret);
		return ret;
	}


	ret = bcrm_read32(camera, BCRM_CSI2_CLOCK_MIN_32R, &camera->avt_min_clk);
	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> csi2 lane count min (%d)\n", ret);
		return ret;
	}

	ret = bcrm_read32(camera, BCRM_CSI2_CLOCK_MAX_32R, &camera->avt_max_clk);
	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> csi2 lane count max (%d)\n", ret);
		return ret;
	}

	avt_info(sd, "Csi clocks\n"
		     "Camera range:           %9d:%9d Hz\n"
		     "Requested mipi clock    %lld",
		      camera->avt_min_clk, camera->avt_max_clk, camera->link_freq);

	if (camera->link_freq < camera->avt_min_clk ||
		camera->link_freq > camera->avt_max_clk)
	{

		avt_err(sd, "unsupported csi clock frequency (%lld Hz, range: %d:%d Hz)!\n",
			camera->link_freq, camera->avt_min_clk, camera->avt_max_clk);
		return -EINVAL;
	}

	ret = bcrm_write32(camera, BCRM_CSI2_CLOCK_32RW, camera->link_freq);	
	if (ret < 0)
	{
		avt_err(sd, "Reg write failed -> csi2 clock (%d)\n", ret);
		return ret;
	}

	ret = bcrm_read32(camera, BCRM_CSI2_CLOCK_32RW, &avt_current_clk);
	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> csi2 clock (%d)\n", ret);
		return ret;
	}

	avt_dbg(sd, "csi clock frequency (req: %lld Hz, cur: %d Hz, range: %d:%d Hz)!\n",
			camera->link_freq,
			avt_current_clk,
			camera->avt_min_clk,
			camera->avt_max_clk);

	avt_info(sd, "Camera CSI2 clock: %u Hz\n", avt_current_clk);

	camera->min_rect.left = camera->min_rect.top = 0;

	ret = bcrm_read32(camera, BCRM_IMG_WIDTH_MIN_32R, &camera->min_rect.width);
	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> width min (%d)\n", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_IMG_WIDTH_MIN_32R %u", camera->min_rect.width);

	ret = bcrm_read32(camera, BCRM_IMG_WIDTH_MAX_32R, &camera->max_rect.width);
	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> width max (%d)\n", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_IMG_WIDTH_MAX_32R %u", camera->max_rect.width);

	camera->max_rect.left = camera->max_rect.top = 0;

	ret = bcrm_read32(camera, BCRM_IMG_HEIGHT_MIN_32R, &camera->min_rect.height);
	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> height min (%d)\n", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_IMG_HEIGHT_MIN_32R %u", camera->min_rect.height);

	ret = bcrm_read32(camera, BCRM_IMG_HEIGHT_MAX_32R, &camera->max_rect.height);
	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> height max (%d)\n", ret);
		// goto err_out;
	}

	ret = device_property_read_u32(&camera->i2c_client->dev,"avt,max-width",
				 &temp);

	if (ret == 0)
	{
		if (camera->max_rect.width > temp)
			camera->max_rect.width = temp;
	}

	ret = device_property_read_u32(&camera->i2c_client->dev,"avt,max-height",
				 &temp);
	if (ret == 0)
	{
		if (camera->max_rect.height > temp)
			camera->max_rect.height = temp;
	}

	avt_dbg(sd, "BCRM_IMG_HEIGHT_MAX_32R %u", camera->max_rect.height);

	ret = bcrm_read64(camera, BCRM_GAIN_MIN_64R, &value64);

	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> gain min (%d)\n", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_GAIN_MIN_64R %llu", value64);

	ret = bcrm_read64(camera, BCRM_GAIN_MAX_64R, &value64);
	
	if (ret < 0)
	{
		avt_err(sd, "Reg read failed -> gain max (%d)\n", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_GAIN_MAX_64R %llu", value64);

	ret = bcrm_read32(camera,BCRM_SENSOR_WIDTH_32R,
			  &camera->sensor_rect.width);

	if (ret < 0)
		return ret;

	ret = bcrm_read32(camera,BCRM_SENSOR_HEIGHT_32R,
			  &camera->sensor_rect.height);

	if (ret < 0)
		return ret;

	camera->sensor_rect.left = 0;
	camera->sensor_rect.top = 0;

	camera->curr_rect = camera->max_rect;
	camera->curr_rect.left = 0;
	camera->curr_rect.top = 0;

	
	camera->mode = AVT_BCRM_MODE;

	return 0;
}

static int avt_csi2_check_mipicfg(struct avt_dev *camera)
{
	struct i2c_client *client = camera->i2c_client;
	struct device *dev = &client->dev;
	struct v4l2_fwnode_endpoint vep = {0};
	int ret = -EINVAL;


	camera->endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!camera->endpoint)
	{
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	vep.bus_type = V4L2_MBUS_CSI2_DPHY;
	if (v4l2_fwnode_endpoint_alloc_parse(camera->endpoint, &vep))
	{
		dev_err(dev, "failed to parse endpoint\n");
		goto error_out;
	}

	/* Check the MIPI CSI2 data lanes count set in device tree */
	if (vep.bus.mipi_csi2.num_data_lanes > 4) {
		dev_err(dev, "only up to 4 lanes supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (vep.nr_of_link_frequencies != 1) {
		dev_err(dev, "invalid number of link frequencies specifed\n");
		goto error_out;
	}

	camera->num_lanes = vep.bus.mipi_csi2.num_data_lanes;
	camera->link_freq = vep.link_frequencies[0];

	ret = 0;
	return ret;

error_out:
	v4l2_fwnode_endpoint_free(&vep);
	fwnode_handle_put(camera->endpoint);

	return ret;
}


static int avt_query_binning(struct avt_dev *camera)
{
	int ret,i,j;
	int type_idx[AVT_BINNING_TYPE_CNT];
	u16 binning_inq;
	u32 width_inc,height_inc;
	const struct v4l2_rect *sensor_rect = &camera->sensor_rect;

	ret = bcrm_read16(camera,BCRM_BINNING_INQ_16R,&binning_inq);

	if (ret < 0)
		return ret;

	// In the firmware version without sensor binning the byteorder of the
	// inquiry register is swapped.
	// If the digital binning fields are zero and the bits outside the
	// allowed range are set, then the byteorder will be swapped.
	if ((binning_inq & 0x7f) == 0 && (binning_inq & 0xffe) != 0) {
		__swab16s(&binning_inq);
	}

	dev_dbg(&camera->i2c_client->dev,"Binning inq 0x%x\n",binning_inq);

	ret = bcrm_read32(camera,BCRM_IMG_WIDTH_INC_32R,&width_inc);

	if (ret < 0)
		return ret;

	width_inc = ilog2(width_inc);

	ret = bcrm_read32(camera,BCRM_IMG_HEIGHT_INC_32R,&height_inc);

	if (ret < 0)
		return ret;

	height_inc = ilog2(height_inc);

	for (i = 0;i < avt_binning_setting_cnt;i++) {
		const struct avt_binning_setting *setting =
			&avt_binning_settings[i];

		if (setting->inq == -1 || binning_inq & (1 << setting->inq)) {
			if (setting->type == NONE) {
				for (j = 0;j < AVT_BINNING_TYPE_CNT;j++)
					camera->binning_info_cnt[j]++;
			} else {
				camera->binning_info_cnt[setting->type]++;
			}
		}
	}

	for (i = 0;i < AVT_BINNING_TYPE_CNT;i++) {
		camera->binning_infos[i] = kcalloc(camera->binning_info_cnt[i],
			sizeof(struct avt_binning_info),GFP_KERNEL);
	}

	memset(type_idx,0,sizeof(type_idx[0]) * AVT_BINNING_TYPE_CNT);
	for (i = 0;i < avt_binning_setting_cnt;i++) {
		const struct avt_binning_setting *setting = &avt_binning_settings[i];
		if (setting->inq == -1 || binning_inq & (1<<setting->inq)) {
			struct avt_binning_info info = {0};

			info.vfact = setting->vfact;
			info.hfact = setting->hfact;
			info.sel = setting->sel;

			info.max_width = sensor_rect->width / setting->hfact;
			info.max_height = sensor_rect->height / setting->vfact;

			v4l_bound_align_image(&info.max_width,0,
					      info.max_width,3,
					      &info.max_height,0,
					      info.max_height,3,0);


			if (setting->type == NONE) {
				int l;
				for (l = 0; l < AVT_BINNING_TYPE_CNT; l++) {
					const int idx = type_idx[l]++;

					dev_dbg(&camera->i2c_client->dev,
						"Binning setting %dx%d: width %u "
						"height %u type: %s\n",
						setting->hfact,setting->vfact,
						info.max_width,info.max_height,
						binning_type_str[l]);

					info.type = l;
					camera->binning_infos[l][idx] = info;
				}
			} else {
				const u32 type = setting->type;
				const int idx = type_idx[type]++;


				dev_dbg(&camera->i2c_client->dev,
					"Binning setting %dx%d: width %u "
					"height %u type: %s\n",
					setting->hfact,setting->vfact,
					info.max_width,info.max_height,
					binning_type_str[type]);

				info.type = type;
				camera->binning_infos[type][idx] = info;
			}


		}
	}


	camera->curr_binning_info = &camera->binning_infos[0][0];

	return 0;
}



static const struct regmap_config alvium_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register = 0xffff,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.name = "alvium_regmap",
	.cache_type = REGCACHE_NONE,
};


static int prepare_write_handshake(struct avt_dev *camera)
{
	int ret;
	u8 handshake_val;

	ret = bcrm_read8(camera,BCRM_WRITE_HANDSHAKE_8RW,&handshake_val);

	if (ret < 0)
	{
		dev_err(&camera->i2c_client->dev,
			"%s[%d]: Reading handshake value failed with: %d\n",
			__func__, __LINE__,ret);
		return ret;
	}

	if ((handshake_val & BCRM_HANDSHAKE_STATUS_MASK) != 0)
	{
		dev_warn(&camera->i2c_client->dev,
			 "%s[%d]: Write handshake still in progress",
			 __func__, __LINE__);
	}

	/* reset only handshake status */
	ret = avt_write(camera, get_bcrm_addr(camera, BCRM_WRITE_HANDSHAKE_8RW), 
		handshake_val & ~BCRM_HANDSHAKE_STATUS_MASK, 1);
	
	if (ret < 0)
	{
		dev_err(&camera->i2c_client->dev,"%s[%d]: Clearing handshake status failed with: %d\n",__func__, __LINE__,ret);
		return ret;
	}


	/* wait for bcrm handshake */
	reinit_completion(&camera->bcrm_wrhs_completion);

	if (!queue_work(camera->bcrm_wrhs_queue, &camera->bcrm_wrhs_work))
	{
		dev_err(&camera->i2c_client->dev,
			"Write handshake already in progress!");
		return -EINVAL;
	}

	return 0;
}

static int wait_for_write_handshake(struct avt_dev *camera)
{
	ulong ret;

	ret = wait_for_completion_timeout(&camera->bcrm_wrhs_completion,
					  msecs_to_jiffies(camera->bcrm_handshake_timeout_ms));

	// If wait_for_completion_timeout returns a positive value, then the handshake was successfully
	// and ret contains the remaining time before the timeout would occur
	if (ret > 0)
	{
		return 0;
	}

	atomic_set(&camera->bcrm_wrhs_enabled,0);
	flush_work(&camera->bcrm_wrhs_work);

	dev_err(&camera->i2c_client->dev,
		"%s[%d]: Write handshake timeout\n",
		__func__, __LINE__);

	return -EIO;
}

static int bcrm_write(struct avt_dev *camera, u16 reg, u64 val, size_t len)
{
	struct device *dev = &camera->i2c_client->dev;
	int ret;

	WARN_ON(camera->mode != AVT_BCRM_MODE);

	ret = prepare_write_handshake(camera);

	if (ret < 0)
		return ret;

	ret = avt_write(camera, get_bcrm_addr(camera, reg), val, len);

	if (ret < 0)
	{
		dev_err(dev,"%s[%d]: Writing value failed with: %d\n",__func__, __LINE__,ret);
		return ret;
	}

	if (!camera->bcrm_write_handshake)
	{
		adev_info(dev, "bcrm write handshake not supported. Using %u ms sleep as fallback.",
				 camera->bcrm_handshake_timeout_ms);
		/* Handshake not supported. Use static sleep at least once as fallback */
		msleep(camera->bcrm_handshake_timeout_ms);
	}

	return wait_for_write_handshake(camera);
}

static void bcrm_wrhs_work_func(struct work_struct *work)
{
	u8 handshake_val = 0;
	static const int poll_interval_ms = 5;
	int ret = 0;
	int i = 0;

	struct avt_dev *camera =
		container_of(work, struct avt_dev, bcrm_wrhs_work);

	atomic_set(&camera->bcrm_wrhs_enabled,1);

	do
	{
		//TODO: Must we check the return value here ?
		ret = bcrm_read8(camera, BCRM_WRITE_HANDSHAKE_8RW, &handshake_val);
	
		if (handshake_val & BCRM_HANDSHAKE_STATUS_MASK)
		{
			//TODO: Must we check the return value here ?
			ret = avt_write(camera, 
				get_bcrm_addr(camera, BCRM_WRITE_HANDSHAKE_8RW),
				handshake_val & ~BCRM_HANDSHAKE_STATUS_MASK,
				AV_CAM_DATA_SIZE_8);

			complete(&camera->bcrm_wrhs_completion);

			dev_dbg(&camera->i2c_client->dev, "%s[%d]: Handshake ok\n",
						__func__, __LINE__);

			break;
		}
		msleep(poll_interval_ms);
		i++;
	} while (atomic_read(&camera->bcrm_wrhs_enabled) != 0);

	if (i == 300)
		adev_info(&camera->i2c_client->dev, "0x%08llx current->pid 0x%08x %d\n",
				(u64)work, current->pid, i);
}


static int avt_detect(struct avt_dev *camera)
{
	u32 version;
	int ret;
	
	ret = avt_read32(camera, CCI_REGISTER_LAYOUT_VERSION_32R, &version);

	if (ret < 0)
	{
		return ret;
	}

	if (version == 0)
	{
		return -ENODEV;
	}

	return 0;
}

static ssize_t avt_i2c_xfer_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *battr, char *buf, loff_t off, size_t len)
{
	struct avt_dev *camera = battr->private;
	struct avt_i2c_xfer *xfer = &camera->next_fw_rd_transfer;
	int ret = -EINVAL; 

	WARN_ON(off != 0);

	mutex_lock(&camera->lock);

	if (xfer->len == len) {
		ret = avt_read_raw(camera, xfer->addr,
			buf, xfer->len);

		memset(xfer, 0, sizeof(*xfer));
	}

	mutex_unlock(&camera->lock);

	return ret;
}

static ssize_t avt_i2c_xfer_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *battr, char *buf, loff_t off, size_t len)
{
	const struct {
		struct avt_i2c_xfer xfer;
		u8 buf[];
	} __packed *payload;
	const struct avt_i2c_xfer *xfer;
	struct avt_dev *camera = battr->private;
	ssize_t ret = -EINVAL;

	WARN_ON(off != 0);

	if (len < sizeof(payload)) 
		return -EINVAL;
	

	payload = (typeof(payload))buf;
	xfer = &payload->xfer;

	if (xfer->len + sizeof(*xfer) > battr->size)
		return -EINVAL;

	mutex_lock(&camera->lock);

	if (payload->xfer.rd) {
		memcpy(&camera->next_fw_rd_transfer, xfer, sizeof(*xfer));

		ret = 0;
	} else {
		if (xfer->len + sizeof(*xfer) != len)
			goto out;
		
		ret = avt_write_raw(camera, xfer->addr, payload->buf, xfer->len);
		if (ret < 0) 
			goto out;	

		ret = xfer->len;
	}

out:	
	mutex_unlock(&camera->lock);

	return ret;
}

static int avt_i2c_xfer_init(struct avt_dev *camera) 
{
	struct device *dev = &camera->i2c_client->dev;
	struct bin_attribute *i2c_xfer_attr;
	int ret;

	i2c_xfer_attr = devm_kzalloc(dev, sizeof(*i2c_xfer_attr), GFP_KERNEL);
	if (!i2c_xfer_attr) 
		return -ENOMEM;

	sysfs_bin_attr_init(i2c_xfer_attr);
	i2c_xfer_attr->attr.name = "i2c_xfer";
	i2c_xfer_attr->attr.mode = 0666; // Other read 
	i2c_xfer_attr->private = camera;
	// TODO: Change to dynamic size
	i2c_xfer_attr->size = sizeof(struct avt_i2c_xfer) + 1024; 
	i2c_xfer_attr->read = avt_i2c_xfer_read;
	i2c_xfer_attr->write = avt_i2c_xfer_write;

	ret = device_create_bin_file(dev, i2c_xfer_attr);
	if (ret) {
		devm_kfree(dev, i2c_xfer_attr);
	} else {
		camera->i2c_xfer_attr = i2c_xfer_attr;
	}
	
	return ret;
}


static ssize_t avt_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct avt_dev *camera = client_to_avt_dev(client);
	
	mutex_lock(&camera->lock);
	if (camera->mode == AVT_GENCP_MODE)
		sysfs_emit(buf, "%s\n","gencp");
	else
		sysfs_emit(buf, "%s\n", "bcm");

	mutex_unlock(&camera->lock);

	return strlen(buf);
}


static ssize_t avt_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct avt_dev *camera = client_to_avt_dev(client);
	u8 mode_req;
	int ret;
	const char *modestr = strim((char*)buf);

	if (strcmp(modestr, "bcm") == 0) {
		mode_req = 0;
	} else if (strcmp(modestr, "gencp") == 0) {
		mode_req = 1;
	} else {
		return -EINVAL;
	}

	mutex_lock(&camera->lock);
	
	ret = avt_change_mode(camera, mode_req);
	if (ret < 0)
		goto out;

	ret = len;
out: 
	mutex_unlock(&camera->lock);

	return ret;
}

static int avt_mode_attr_init(struct avt_dev *camera) 
{
	struct device *dev = &camera->i2c_client->dev;
	struct device_attribute *mode_attr;
	int ret;

	mode_attr = devm_kzalloc(dev, sizeof(*mode_attr), GFP_KERNEL);
	if (!mode_attr) 
		return -ENOMEM;

	sysfs_attr_init(&mode_attr->attr);
	mode_attr->attr.name = "mode";
	mode_attr->attr.mode = 0666;
	mode_attr->show = avt_mode_show;
	mode_attr->store = avt_mode_store;

	ret = device_create_file(dev, mode_attr);
	if (ret) {
		devm_kfree(dev, mode_attr);
	} else {
		camera->mode_attr = mode_attr;
	}

	return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0))

static int avt_flash_notify_bound(struct v4l2_async_notifier *notifier,
				  struct v4l2_subdev *sd,
				  struct v4l2_async_subdev *asd)
{
	struct avt_dev *camera =
		container_of(notifier, struct avt_dev, flash_notifier);

	camera->flash_sd = sd;

	return 0;
}

static const struct v4l2_async_notifier_operations avt_flash_notify_ops = {
	.bound = avt_flash_notify_bound
};

static int avt_flash_notifier_setup(struct avt_dev *camera,
				    struct device_node *node)
{
	struct v4l2_subdev *sd = get_sd(camera);
	struct v4l2_async_notifier *notifier = &camera->flash_notifier;
	struct device *dev = &camera->i2c_client->dev;
	struct v4l2_async_subdev *asd;
	int ret = 0;


	v4l2_async_notifier_init(notifier);

	asd = v4l2_async_notifier_add_fwnode_subdev(
		notifier, of_fwnode_handle(node),
		struct v4l2_async_subdev);
	of_node_put(node);

	if (IS_ERR(asd)) {
		dev_err(dev, "failed to add notifier with %ld\n",
			PTR_ERR(asd));

		return PTR_ERR(asd);
	}

	notifier->ops = &avt_flash_notify_ops;

	ret = v4l2_async_subdev_notifier_register(sd, notifier);
	if (ret) {
		dev_err(dev, "subdev notifier register failed with %d", ret);
		return ret;
	}

	return 0;
}

#else
static int avt_flash_notifier_setup(struct avt_dev *camera,
				    struct device_node *node) 
{
	return -ENOTSUPP;
}
#endif


static int avt_flash_init(struct avt_dev *camera)
{
	struct device *dev = &camera->i2c_client->dev;
	struct device_node *node;

	if (!dev->of_node)
		return -EINVAL;

	node = of_parse_phandle(dev->of_node, "flash", 0);
	if (!node) {
		dev_info(dev, "Failed to get flash node\n");
		return 0;
	}

	return avt_flash_notifier_setup(camera, node);
}

static bool has_jetson_nodes(struct device *dev) {
	struct fwnode_handle *child;

	device_for_each_child_node(dev, child) {
		if (!strncmp("mode", fwnode_get_name(child), 4)) {
			fwnode_handle_put(child);
			return true;
		}
	}

	return false;
}

static int avt_probe(struct i2c_client *client)
{

	struct device *dev = &client->dev;
	struct avt_dev *camera;
	struct v4l2_mbus_framefmt *fmt;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct v4l2_subdev *sd;
	struct regulator *reg_vcc_ext;
	int ret;

	camera = devm_kzalloc(dev, sizeof(*camera), GFP_KERNEL);
	if (!camera)
		return -ENOMEM;

	camera->i2c_client = client;
	camera->streamon_delay = 0;
	camera->framerate_auto = true;
	camera->reverse_x_reg = 0;
        camera->reverse_y_reg = 0;
	camera->mbus_fmt_code = 0;
	camera->mbus_fmt_transformed = false;

	camera->regmap = devm_regmap_init_i2c(client, &alvium_regmap_config);
	if (IS_ERR(camera->regmap))
	{
		return dev_err_probe(dev, PTR_ERR(camera->regmap), 
				     "i2c regmap init failed\n");
	}

	reg_vcc_ext = devm_regulator_get_optional(dev, "vcc-ext");
	if (!IS_ERR(reg_vcc_ext)) {
		ret = regulator_enable(reg_vcc_ext);
		if (ret)
			return dev_err_probe(dev, ret, 
					     "failed to enable regulator\n");

		camera->reg_vcc_ext = reg_vcc_ext;

		ret = read_poll_timeout(avt_detect, ret, !ret,
					BOOT_POLL_INTERVAL_US, 
			  	BOOT_TIMEOUT_US, false, camera);
	} else {
		ret = avt_detect(camera);
	}

	if (ret) {
		dev_warn(&client->dev,"No camera detected!");
		ret = -ENODEV;
		goto regulator_cleanup;
	}


	sd = get_sd(camera);

	ret = fwnode_property_read_u32(fwnode,"streamon_delay",
				       &camera->streamon_delay);
	if (camera->streamon_delay)
	{
		adev_info(dev, "use acquisition start delay of %u us\n", camera->streamon_delay);
	}

	camera->stream_start_phy_reset
		= fwnode_property_present(fwnode,"phy_reset_on_start");

	ret = fwnode_property_read_u32(dev_fwnode(&client->dev),
		"bcrm_wait_timeout", &camera->bcrm_handshake_timeout_ms);

	if (ret)
	{
		camera->bcrm_handshake_timeout_ms = BCRM_WAIT_HANDSHAKE_TIMEOUT_MS;
		dev_warn(dev, "Using default value for BCRM wait timeout, %d ms\n",
			camera->bcrm_handshake_timeout_ms);
	}
	else
	{
		adev_info(dev, "BCRM wait timeout = %d ms\n", camera->bcrm_handshake_timeout_ms);
	}

	ret = avt_csi2_check_mipicfg(camera);
	if (ret)
	{
		dev_err(dev, "%s[%d]: failed to parse endpoint\n", __func__, __LINE__);
		ret = -EINVAL;
		goto err_exit;
	}

#ifdef NVIDIA
	if (!has_jetson_nodes(dev)) 
		dev_warn(dev, "NVIDIA support enabled, "
			 "but modeX node not found\n");

	camera->s_data.priv = camera;
	camera->s_data.dev = &camera->i2c_client->dev;
	camera->s_data.ctrl_handler = &camera->v4l2_ctrl_hdl;

	ret = camera_common_initialize(&camera->s_data, "avt_csi2");

	if (unlikely(ret)) {
		goto fwnode_cleanup;
	}

#else
	if (has_jetson_nodes(dev)) {
		dev_err(dev, "found NVIDIA device tree nodes, "
			"but driver is not built with NVIDIA support\n");
		ret = -EINVAL;
		goto fwnode_cleanup;
	}
#endif 

	/* now create the subdevice on i2c*/
	v4l2_i2c_subdev_init(sd, client, &avt_subdev_ops);
	sd->dev = &client->dev;
	sd->internal_ops = &avt_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE;
	camera->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.ops = &avt_sd_media_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->owner = NULL;
	ret = media_entity_pads_init(&sd->entity, 1, &camera->pad);
	if (ret < 0)
		goto fwnode_cleanup;

	mutex_init(&camera->lock);

	// No regulator specified, but camera is reachable 
	// -> Must be externally powered 
	// -> Do softreset so camera is in good state
	if (!camera->reg_vcc_ext) {
		ret = avt_do_softreset(camera);
		if(ret < 0) {
			avt_err(sd, "Camera reset failed");
			goto fwnode_cleanup;
		}
	}
	
	ret = read_cci_registers(client);

	if (ret < 0)
	{
		dev_err(dev, "CCI registers read failed - %d\n", ret);
		goto entity_cleanup;
	}
	dev_info(dev, "CCI registers read successful\n");

	ret = cci_version_check(client);
	if (ret < 0)
	{
		dev_err(&client->dev, "CCI version mismatch - %d\n", ret);
		goto entity_cleanup;
	}

	ret = bcrm_version_check(client);
	if (ret < 0)
	{
		dev_err(&client->dev, "BCRM version mismatch - %d\n", ret);
		goto entity_cleanup;
	}
	dev_info(dev, "BCRM version check successful\n");

	camera->bcrm_write_handshake =
		bcrm_get_write_handshake_availibility(client);


	dev_info(dev,"Camera model %s %s",camera->cci_reg.reg.family_name,
		 camera->cci_reg.reg.model_name);

	/* reading the Firmware Version register */
	ret = bcrm_read64(camera,BCRM_DEVICE_FIRMWARE_VERSION_64R,
			  &camera->cam_firmware_version.value);

	dev_info(&client->dev, "Firmware version: %02u.%02u.%02u.%08x\n",
			 camera->cam_firmware_version.device_firmware.special_version,
			 camera->cam_firmware_version.device_firmware.major_version,
			 camera->cam_firmware_version.device_firmware.minor_version,
			 camera->cam_firmware_version.device_firmware.patch_version);

	if (camera->cci_reg.reg.device_capabilities.caps.gencp)
	{
		ret = read_gencp_registers(client);
		if (ret < 0)
		{
			dev_err(dev, "%s: read_gencp_registers failed: %d\n",
					__func__, ret);
			goto entity_cleanup;
		}

		ret = gcprm_version_check(client);
		if (ret < 0)
		{
			dev_err(&client->dev, "gcprm version mismatch!\n");
			goto free_ctrls;
		}

		dev_info(&client->dev, "GCPRM version correct\n");
	}

	init_completion(&camera->bcrm_wrhs_completion);

	camera->bcrm_wrhs_queue = create_singlethread_workqueue(sd->name);
	if (!camera->bcrm_wrhs_queue)
	{
		dev_err(&client->dev, "%s[%d]: Could not create work queue\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto fwnode_cleanup;
	}

	INIT_WORK(&camera->bcrm_wrhs_work, bcrm_wrhs_work_func);
	atomic_set(&camera->bcrm_wrhs_enabled,0);

	CLEAR(camera->max_rect);
	CLEAR(camera->min_rect);
	CLEAR(camera->curr_rect);

	ret = avt_get_camera_capabilities(sd);
	if (ret)
		goto entity_cleanup;

	ret = avt_query_binning(camera);
	if (ret)
		goto entity_cleanup;

	ret = avt_get_fmt_available(client);

	ret = avt_init_avail_formats(sd);
	if (ret < 0)
	{
		dev_err(dev, "%s[%d]: avt_init_avail_formats failed with %d\n",
				__func__, __LINE__, ret);
		goto entity_cleanup;
	}

	sd->ctrl_handler = &camera->v4l2_ctrl_hdl;

	fmt = &camera->fmt[AVT_BCRM_MODE];

	ret = avt_init_current_format(camera, fmt);
	if (ret)
	{
		goto entity_cleanup;
	}

	// Init controls before registering the device, because the control handler must be fully initialized before
	// the subdevice is registered.
	ret = avt_init_controls(camera);
	if (ret)
	{
		dev_err(dev, "%s[%d]: avt_init_controls failed with (%d)\n", __func__, __LINE__, ret);
		goto entity_cleanup;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
	ret = v4l2_subdev_init_finalize(sd);

	if (ret) {
		dev_err(dev, "Failed to finalize subdev init!");
		goto free_ctrls;
	}
#endif
	ret = avt_flash_init(camera);
	if (ret)
		goto sd_cleanup;

	ret = v4l2_async_register_subdev(sd);

	if (ret)
	{
		dev_err(dev, "%s[%d]: v4l2_async_register_subdev failed with (%d)\n", __func__, __LINE__, ret);
		goto sd_cleanup;
	}
	dev_info(&client->dev, "Camera registered\n");

	ret = device_add_group(dev, &avt_attr_grp);
	adev_info(dev, "sysfs group created! (%d)\n", ret);
	if (ret)
	{
		dev_err(dev, "%s[%d]: Failed to create sysfs group (%d)\n", __func__, __LINE__, ret);
		goto sd_cleanup;
	}

	ret = avt_mode_attr_init(camera);
	if (ret) {
		dev_err(dev, "Failed to create mode attribute!\n");
		goto sysfs_cleanup;
	}

	ret = avt_i2c_xfer_init(camera);
	if (ret) {
		dev_err(dev, "Failed to create fw_transfer attribute!\n");
		goto sysfs_cleanup;
	}


	ret = bcrm_write32(camera, BCRM_STREAM_ON_DELAY_32RW, camera->streamon_delay);

	return 0;

sysfs_cleanup:
	device_remove_group(dev, &avt_attr_grp);

sd_cleanup:
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
	v4l2_subdev_cleanup(sd);
#endif

free_ctrls:

	v4l2_ctrl_handler_free(&camera->v4l2_ctrl_hdl);

entity_cleanup:
	media_entity_cleanup(&sd->entity);

fwnode_cleanup:
	if (camera->bcrm_wrhs_queue)
		destroy_workqueue(camera->bcrm_wrhs_queue);
	fwnode_handle_put(camera->endpoint);

err_exit:
	mutex_destroy(&camera->lock);

regulator_cleanup:
	if (camera->reg_vcc_ext) 
		regulator_disable(camera->reg_vcc_ext);
	return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
static int avt_remove(struct i2c_client *client)
#else
static void avt_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct avt_dev *camera = to_avt_dev(sd);
	struct device *dev = &client->dev;

	fwnode_handle_put(camera->endpoint);

	device_remove_file(dev, camera->mode_attr);
	device_remove_bin_file(dev, camera->i2c_xfer_attr);

	device_remove_group(dev, &avt_attr_grp);
	media_entity_cleanup(&sd->entity);

#ifdef NVIDIA
	camera_common_cleanup(&camera->s_data);
#endif // NVIDIA

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0))
	v4l2_subdev_cleanup(sd);
#endif

	v4l2_ctrl_handler_free(&camera->v4l2_ctrl_hdl);

#ifdef DPHY_RESET_WORKAROUND
	avt_streamon_thread_disable(sd);
#endif

	if (camera->bcrm_wrhs_queue)
		destroy_workqueue(camera->bcrm_wrhs_queue);

	mutex_destroy(&camera->lock);

	v4l2_async_unregister_subdev(sd);

	if (camera->reg_vcc_ext) 
		regulator_disable(camera->reg_vcc_ext);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	return 0;
#endif
}


static const struct i2c_device_id avt_id[] = {
	{"avt_csi2", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, avt_id);

static const struct of_device_id avt_dt_ids[] = {
	{
		.compatible = "alliedvision,avt_csi2",
	},
	{}};
MODULE_DEVICE_TABLE(of, avt_dt_ids);

static struct i2c_driver avt_i2c_driver = {
	.driver = {
		.name = "avt_csi2",
		.of_match_table = avt_dt_ids,
	},
	.id_table = avt_id,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0))
	.probe_new = avt_probe,
#else
	.probe = avt_probe,
#endif
	.remove = avt_remove,
};

module_i2c_driver(avt_i2c_driver);

MODULE_DESCRIPTION("Allied Vision's MIPI-CSI2 Camera Driver");
MODULE_AUTHOR("Allied Vision Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

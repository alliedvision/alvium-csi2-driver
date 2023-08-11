// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 Avnet EMG GmbH 
 * Copyright (C) 2022 - 2023 Allied Vision Technologies
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

#define ENABLE_STEPWISE_IMAGE_SIZE
#define AUTO_ACQUISITION_FRAME_RATE
#define xI2C_READ_COMPATIBLE_MODE
#define xMUTEX_DEBUG_MODE

#define AVT3_DEFAULT_FPS AVT3_10_FPS
#define AVT3_DEFAULT_MODE AVT3_MODE_NTSC_720_480

#define AVT3_DEFAULT_EXPOSURETIME 25000000
#define AVT3_DEFAULT_GAIN 1000
#define AVT3_MAX_FORMAT_ENTRIES 40

#define AVT3_DRIVER_NAME "avt3"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
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
#include <media/mipi-csi2.h>
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

// only for dma_get_cache_alignment();
#include <linux/dma-mapping.h>

#include "avt-csi2.h"
#include <uapi/linux/libcsi_ioctl.h>

static int debug = 0;
module_param(debug, int, 0644); /* S_IRUGO */
MODULE_PARM_DESC(debug, "Debug level (0-2)");

static int add_wait_time_ms = 2000;
module_param(add_wait_time_ms, int, 0600);


#define AVT_DBG_LVL 2

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

#ifdef MUTEX_DEBUG_MODE
#define MUTEX_LOCK(a)                                  \
	{                                                  \
		printk("l+ %s[%d]\n", __func__, __LINE__);     \
		if (!(a))                                      \
		{                                              \
			printk("%s[%d]: MUTEX_LOCK(NULL)!!!\n",    \
				   __func__, __LINE__);                \
			dump_stack();                              \
		}                                              \
		else                                           \
		{                                              \
			mutex_lock((a));                           \
			printk("l- %s[%d]\n", __func__, __LINE__); \
		}                                              \
	}

#define MUTEX_UNLOCK(a)                                \
	{                                                  \
		printk("u+ %s[%d]\n", __func__, __LINE__);     \
		if (!(a))                                      \
		{                                              \
			printk("%s[%d]: MUTEX_UNLOCK(NULL)!!!\n",  \
				   __func__, __LINE__);                \
		}                                              \
		else                                           \
		{                                              \
			mutex_unlock((a));                         \
			printk("u- %s[%d]\n", __func__, __LINE__); \
		}                                              \
	}
#else
#define MUTEX_LOCK(a)                               \
	{                                               \
		if (!(a))                                   \
		{                                           \
			printk("%s[%d]: MUTEX_LOCK(NULL)!!!\n", \
				   __func__, __LINE__);             \
			dump_stack();                           \
		}                                           \
		else                                        \
		{                                           \
			mutex_lock((a));                        \
		}                                           \
	}

#define MUTEX_UNLOCK(a)                               \
	{                                                 \
		if (!(a))                                     \
		{                                             \
			printk("%s[%d]: MUTEX_UNLOCK(NULL)!!!\n", \
				   __func__, __LINE__);               \
		}                                             \
		else                                          \
		{                                             \
			mutex_unlock((a));                        \
		}                                             \
	}
#endif

#define BCRM_WAIT_HANDSHAKE_TIMEOUT_MS 3000000

//Define formats for GenICam for CSI2, if they not exist
#ifndef V4L2_PIX_FMT_CUSTOM
#define V4L2_PIX_FMT_CUSTOM    v4l2_fourcc('T', 'P', '3', '1') /* 0x31 mipi datatype  */
#endif

#ifndef MEDIA_BUS_FMT_CUSTOM
#define MEDIA_BUS_FMT_CUSTOM        		0x5002
#endif

#define AVT_BINNING_MODE_FLAG_AVERAGE 		0b01
#define AVT_BINNING_MODE_FLAG_SUM 		0b10

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

struct avt3_mode_info
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

static int bcrm_regmap_write64(struct avt3_dev *sensor,
							   struct regmap *map,
							   unsigned int reg,
							   unsigned long long val);

static int bcrm_regmap_write(struct avt3_dev *sensor,
							 struct regmap *map,
							 unsigned int reg,
							 unsigned int val);

static int avt3_detect(struct i2c_client *client);
static int avt3_reset(struct avt3_dev *sensor, enum avt_reset_type reset_type);
static void avt3_dphy_reset(struct avt3_dev *sensor, bool bResetPhy);

static void avt3_ctrl_changed(struct avt3_dev *camera, const struct v4l2_ctrl * const ctrl);
static struct v4l2_ctrl* avt3_ctrl_find(struct avt3_dev *camera,u32 id);
static int avt3_ctrl_write(struct i2c_client *client, enum avt_ctrl ctrl_id, __u32 value);
static int avt3_get_sensor_capabilities(struct v4l2_subdev *sd);
static inline struct avt3_dev *to_avt3_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct avt3_dev, sd);
}

static struct avt3_dev *client_to_avt3_dev(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct avt3_dev, sd);
}

#define DUMP_BCRM_REG8(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_8)
#define DUMP_BCRM_REG16(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_16)
#define DUMP_BCRM_REG32(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_32)
#define DUMP_BCRM_REG64(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_64)

static void dump_bcrm_reg(struct i2c_client *client, u16 nOffset, const char *pRegName, int regsize);

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
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	int status = 0;
	struct avt_val64 val64;

	CLEAR(val64);

	if (status >= 0)
		switch (regsize)
		{
		case AV_CAM_DATA_SIZE_8:
			regmap_read(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + nOffset, &val64.u32[0]);
			avt_info(&sensor->sd, "%44s: %u (0x%x)", pRegName, val64.u32[0], val64.u32[0]);
			break;
		case AV_CAM_DATA_SIZE_16:
			regmap_read(sensor->regmap16, sensor->cci_reg.reg.bcrm_addr + nOffset, &val64.u32[0]);
			avt_info(&sensor->sd, "%44s: %u (0x%08x)", pRegName, val64.u32[0], val64.u32[0]);
			break;
		case AV_CAM_DATA_SIZE_32:
			regmap_read(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + nOffset, &val64.u32[0]);
			avt_info(&sensor->sd, "%44s: %u (0x%08x)", pRegName, val64.u32[0], val64.u32[0]);
			break;
		case AV_CAM_DATA_SIZE_64:
			regmap_bulk_read(sensor->regmap64, sensor->cci_reg.reg.bcrm_addr + nOffset, &val64, 1);
			avt_info(&sensor->sd, "%44s: %llu (0x%016llx)", pRegName, val64.u64, val64.u64);
			break;
		}
	else
		avt_err(&sensor->sd, "%s: ERROR", pRegName);
}

static bool bcrm_get_write_handshake_availibility(struct i2c_client *client)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	u32 value = 0;
	int status;

	if (!sensor)
	{
		avt_err(&sensor->sd, "sensor == NULL!!!\n");
		return -EINVAL;
	}
	/* check of camera supports write_done_handshake register */
	status = regmap_read(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW, &value);

	if ((status >= 0) && (value & BCRM_HANDSHAKE_AVAILABLE_MASK))
	{
		avt_info(&sensor->sd, "BCRM write handshake supported!");
		return true;
	}
	else
	{
		avt_info(&sensor->sd, "BCRM write handshake NOT supported!");
		return false;
	}
}

static inline u16 get_bcrm_addr(struct avt3_dev *camera,u16 reg)
{
	return camera->cci_reg.reg.bcrm_addr + reg;
}

static inline int bcrm_read8(struct avt3_dev *camera,u16 reg,u8 *val)
{
	const u16 bcrm_addr = get_bcrm_addr(camera,reg);
	return regmap_bulk_read(camera->regmap8,bcrm_addr,val,1);
}

static inline int bcrm_read16(struct avt3_dev *camera,u16 reg,u16 *val)
{
	const u16 bcrm_addr = get_bcrm_addr(camera,reg);
	return regmap_bulk_read(camera->regmap16,bcrm_addr,val,1);
}

static inline int bcrm_read32(struct avt3_dev *camera,u16 reg,u32 *val)
{
	const u16 bcrm_addr = get_bcrm_addr(camera,reg);
	return regmap_read(camera->regmap32,bcrm_addr,val);
}

static inline int bcrm_read64(struct avt3_dev *camera,u16 reg,u64 *val)
{
	const u16 bcrm_addr = get_bcrm_addr(camera,reg);
	return regmap_bulk_read(camera->regmap64,bcrm_addr,val,1);
}


static int read_cci_registers(struct i2c_client *client)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);

	int ret = 0;
	uint32_t crc = 0;
	uint32_t crc_byte_count = 0;

	if (!sensor)
	{
		avt_err(&sensor->sd, "sensor == NULL!!!");
		return -EINVAL;
	}

	if (!sensor->regmap8)
	{
		avt_err(&sensor->sd, "sensor->regmap8 == NULL!!!");
		return -EINVAL;
	}

	if (!sensor->regmap16)
	{
		avt_err(&sensor->sd, "sensor->regmap16 == NULL!!!");
		return -EINVAL;
	}

	if (!sensor->regmap32)
	{
		avt_err(&sensor->sd, "sensor->regmap32 == NULL!!!");
		return -EINVAL;
	}

	if (!sensor->regmap64)
	{
		avt_err(&sensor->sd, "sensor->regmap64 == NULL!!!");
		return -EINVAL;
	}

	MUTEX_LOCK(&sensor->lock);

	/*
	 * ToDO: Check against latest spec!!
	 * Avoid last 3 bytes read as its WRITE only register except
	 * CURRENT MODE REG
	 */
	/* Calculate byte per byte CRC from each reg up to the CRC reg */
	crc_byte_count =
		(uint32_t)((char *)&sensor->cci_reg.reg.checksum - (char *)&sensor->cci_reg);

	avt_dbg(&sensor->sd, "crc_byte_count: %d", crc_byte_count);
	avt_dbg(&sensor->sd, "0x%08X, 0x%08X",
			cci_cmd_tbl[CCI_REGISTER_LAYOUT_VERSION].address,
			cci_cmd_tbl[CHANGE_MODE].address);

	// read only until CHANGE_MODE because it's writeonly
	ret = regmap_bulk_read(sensor->regmap8, cci_cmd_tbl[CCI_REGISTER_LAYOUT_VERSION].address,
						   (char *)&sensor->cci_reg, cci_cmd_tbl[CHANGE_MODE].address);

	avt_info(&sensor->sd, "regmap_bulk_read(sensor->regmap8, cci_cmd_tbl[CCI_REGISTER_LAYOUT_VERSION].address ret %d\n", ret);

	avt_dbg(&sensor->sd, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X - 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			sensor->cci_reg.buf[0x00], sensor->cci_reg.buf[0x01], sensor->cci_reg.buf[0x02], sensor->cci_reg.buf[0x03],
			sensor->cci_reg.buf[0x04], sensor->cci_reg.buf[0x05], sensor->cci_reg.buf[0x06], sensor->cci_reg.buf[0x07],
			sensor->cci_reg.buf[0x08], sensor->cci_reg.buf[0x09], sensor->cci_reg.buf[0x0a], sensor->cci_reg.buf[0x0b],
			sensor->cci_reg.buf[0x0c], sensor->cci_reg.buf[0x0d], sensor->cci_reg.buf[0x0e], sensor->cci_reg.buf[0x0f]);

	avt_dbg(&sensor->sd, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X - 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			sensor->cci_reg.buf[0x10], sensor->cci_reg.buf[0x11], sensor->cci_reg.buf[0x12], sensor->cci_reg.buf[0x13],
			sensor->cci_reg.buf[0x14], sensor->cci_reg.buf[0x15], sensor->cci_reg.buf[0x16], sensor->cci_reg.buf[0x17],
			sensor->cci_reg.buf[0x18], sensor->cci_reg.buf[0x19], sensor->cci_reg.buf[0x1a], sensor->cci_reg.buf[0x1b],
			sensor->cci_reg.buf[0x1c], sensor->cci_reg.buf[0x1d], sensor->cci_reg.buf[0x1e], sensor->cci_reg.buf[0x1f]);

	avt_dbg(&sensor->sd, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X - 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			sensor->cci_reg.buf[0x20], sensor->cci_reg.buf[0x21], sensor->cci_reg.buf[0x22], sensor->cci_reg.buf[0x23],
			sensor->cci_reg.buf[0x24], sensor->cci_reg.buf[0x25], sensor->cci_reg.buf[0x26], sensor->cci_reg.buf[0x27],
			sensor->cci_reg.buf[0x28], sensor->cci_reg.buf[0x29], sensor->cci_reg.buf[0x1a], sensor->cci_reg.buf[0x2b],
			sensor->cci_reg.buf[0x2c], sensor->cci_reg.buf[0x2d], sensor->cci_reg.buf[0x2e], sensor->cci_reg.buf[0x2f]);

	avt_dbg(&sensor->sd, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X - 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			sensor->cci_reg.buf[0x30], sensor->cci_reg.buf[0x31], sensor->cci_reg.buf[0x32], sensor->cci_reg.buf[0x33],
			sensor->cci_reg.buf[0x34], sensor->cci_reg.buf[0x35], sensor->cci_reg.buf[0x36], sensor->cci_reg.buf[0x37],
			sensor->cci_reg.buf[0x38], sensor->cci_reg.buf[0x39], sensor->cci_reg.buf[0x1a], sensor->cci_reg.buf[0x3b],
			sensor->cci_reg.buf[0x3c], sensor->cci_reg.buf[0x3d], sensor->cci_reg.buf[0x3e], sensor->cci_reg.buf[0x3f]);

	if (ret < 0)
	{
		avt_err(&sensor->sd, "regmap_read failed (%d)\n", ret);
		goto err_out;
	}

	/* CRC calculation */
	crc = crc32(U32_MAX, &sensor->cci_reg, crc_byte_count);

	dev_info(&client->dev, "cci layout version b: 0x%08X\n",
			 sensor->cci_reg.reg.layout_version);
	/* Swap bytes if neccessary */
	cpu_to_be32s(&sensor->cci_reg.reg.layout_version);

	dev_info(&client->dev, "cci layout version a: 0x%08X\n",
			 sensor->cci_reg.reg.layout_version);

	cpu_to_be64s(&sensor->cci_reg.reg.device_capabilities.value);
	cpu_to_be16s(&sensor->cci_reg.reg.gcprm_address);
	cpu_to_be16s(&sensor->cci_reg.reg.bcrm_addr);
	cpu_to_be32s(&sensor->cci_reg.reg.checksum);

	/* Check the checksum of received with calculated. */
	if (crc != sensor->cci_reg.reg.checksum)
	{
		avt_err(&sensor->sd, "wrong CCI CRC value! calculated = 0x%x, received = 0x%x\n",
				crc, sensor->cci_reg.reg.checksum);
		ret = -EINVAL;
		goto err_out;
	}

	avt_dbg(&sensor->sd, "cci layout version: 0x%08X\ncci device capabilities: %llx\ncci device guid: %s\ncci gcprm_address: 0x%x\n",
			sensor->cci_reg.reg.layout_version,
			sensor->cci_reg.reg.device_capabilities.value,
			sensor->cci_reg.reg.device_guid,
			sensor->cci_reg.reg.gcprm_address);

	ret = 0;
err_out:

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static int read_gencp_registers(struct i2c_client *client)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);

	int ret = 0;
	uint32_t crc = 0;
	uint32_t crc_byte_count = 0;

	uint32_t i2c_reg;
	uint32_t i2c_reg_size;
	uint32_t i2c_reg_count;

	char *i2c_reg_buf;

	MUTEX_LOCK(&sensor->lock);
	avt_dbg(&sensor->sd, "+");

	i2c_reg = sensor->cci_reg.reg.gcprm_address + 0x0000;
	i2c_reg_size = AV_CAM_REG_SIZE;
	i2c_reg_count = sizeof(sensor->gencp_reg);
	i2c_reg_buf = (char *)&sensor->gencp_reg;

	/* Calculate CRC from each reg up to the CRC reg */
	crc_byte_count =
		(uint32_t)((char *)&sensor->gencp_reg.checksum - (char *)&sensor->gencp_reg);

	ret = regmap_bulk_read(sensor->regmap8,
					 sensor->cci_reg.reg.gcprm_address + 0x0000,
					 (char *)&sensor->gencp_reg,
					 sizeof(sensor->gencp_reg));

	if (ret < 0)
	{
		avt_err(&sensor->sd, "regmap_read failed, ret %d", ret);
		goto err_out;
	}

	crc = crc32(U32_MAX, &sensor->gencp_reg, crc_byte_count);

	be32_to_cpus(&sensor->gencp_reg.gcprm_layout_version);
	be16_to_cpus(&sensor->gencp_reg.gencp_out_buffer_address);
	be16_to_cpus(&sensor->gencp_reg.gencp_in_buffer_address);
	be16_to_cpus(&sensor->gencp_reg.gencp_out_buffer_size);
	be16_to_cpus(&sensor->gencp_reg.gencp_in_buffer_size);
	be32_to_cpus(&sensor->gencp_reg.checksum);

	if (crc != sensor->gencp_reg.checksum)
	{
		avt_err(&sensor->sd, "wrong GENCP CRC value! calculated = 0x%x, received = 0x%x\n",
				crc, sensor->gencp_reg.checksum);
		ret = -EINVAL;
		goto err_out;
	}

	avt_dbg(&sensor->sd, "gcprm layout version: %x\n",
			sensor->gencp_reg.gcprm_layout_version);
	avt_dbg(&sensor->sd, "gcprm out buf addr: %x\n",
			sensor->gencp_reg.gencp_out_buffer_address);
	avt_dbg(&sensor->sd, "gcprm out buf size: %x\n",
			sensor->gencp_reg.gencp_out_buffer_size);
	avt_dbg(&sensor->sd, "gcprm in buf addr: %x\n",
			sensor->gencp_reg.gencp_in_buffer_address);
	avt_dbg(&sensor->sd, "gcprm in buf size: %x\n",
			sensor->gencp_reg.gencp_in_buffer_size);

err_out:
	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static int read_current_mode(struct avt3_dev *camera)
{
	int ret;
	uint current_mode;

	ret = regmap_read(camera->regmap8, GENCP_CURRENTMODE_8R, &current_mode);

	if (ret < 0)
		return ret;

	return current_mode;
}

static int avt3_set_bcrm(struct i2c_client *client)
{
	uint8_t mode = 0;
	int current_mode = 0;
	int elapsed = 0;
	int const timeout = 5000;
	int const delay = 50;
	int ret;

	struct avt3_dev *sensor = client_to_avt3_dev(client);

	current_mode = read_current_mode(sensor);
	if (current_mode < 0)
	{
		avt_err(&sensor->sd, "Failed to get mode: regmap_read on GENCP_CURRENTMODE_8R failed (%d)", ret);
		return current_mode;
	}

	if (AVT_BCRM_MODE == current_mode && current_mode != sensor->mode)
		avt_dbg(&sensor->sd, "Sensor is already in BCRM mode but driver has inconsitent mode value.");

	if (AVT_BCRM_MODE == current_mode)
	{
		avt_warn(&sensor->sd, "Sensor is already in BCRM mode. Ignore request to set BCRM mode.");
		sensor->mode = AVT_BCRM_MODE;
		return ret;
	}

	mode = AVT_BCRM_MODE;
	ret = regmap_write(sensor->regmap8, GENCP_CHANGEMODE_8W, mode);

	if (ret < 0)
	{
		avt_err(&sensor->sd, "Failed to set BCRM mode: i2c write failed (%d)", ret);
		return ret;
	}

	do
	{
		msleep(delay);

		current_mode = read_current_mode(sensor);

		if (current_mode < 0) {
			avt_err(&sensor->sd,"Failed to read current mode!");
			return current_mode;
		}

		if (current_mode == mode) {
			break;
		}

		elapsed += delay;
	} while (elapsed < timeout);

	if (elapsed >= timeout) {
		avt_err(&sensor->sd,"Waiting for mode change timeout!");
		return -EIO;
	}

	ret = avt3_ctrl_write(sensor->i2c_client, V4L2_AV_CSI2_PIXELFORMAT, sensor->mbus_framefmt.code);

	if (ret < 0) {
		avt_err(&sensor->sd,"Failed to set pixelformat!");
		return ret;
	}

	sensor->mode = AVT_BCRM_MODE;
	return ret;
}

static int avt3_set_gencp(struct i2c_client *client)
{
	uint8_t mode = 0;
	uint32_t current_mode = 0;

	int ret;
	struct avt3_dev *sensor = client_to_avt3_dev(client);

	ret = regmap_read(sensor->regmap8,
					  GENCP_CURRENTMODE_8R, &current_mode);
	if (ret < 0)
	{
		avt_err(&sensor->sd, "Failed to get mode: regmap_read on GENCP_CURRENTMODE_8R failed (%d)", ret);
		return ret;
	}

	if (AVT_GENCP_MODE == current_mode && current_mode != sensor->mode)
		avt_info(&sensor->sd, "Sensor is already in GenCP mode but driver has inconsitent mode value.");

	if (AVT_GENCP_MODE == current_mode)
	{
		avt_info(&sensor->sd, "Sensor is already in GenCP mode. Ignore request to set GenCP mode.");
		sensor->mode = AVT_GENCP_MODE;
		return ret;
	}

	mode = AVT_GENCP_MODE;
	ret = bcrm_regmap_write(sensor, sensor->regmap8,
							GENCP_CHANGEMODE_8W, AVT_GENCP_MODE);

	if (ret < 0)
	{
		avt_dbg(&sensor->sd, "Failed to set GenCP mode: i2c write failed (%d)", ret);
		return ret;
	}
	sensor->mode = AVT_GENCP_MODE;
	return ret;
}

static int cci_version_check(struct i2c_client *client)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	uint32_t cci_minver, cci_majver;
	int ret = 0;

	MUTEX_LOCK(&sensor->lock);

	cci_minver = (sensor->cci_reg.reg.layout_version & CCI_REG_LAYOUT_MINVER_MASK) >> CCI_REG_LAYOUT_MINVER_SHIFT;

	if (cci_minver >= CCI_REG_LAYOUT_MINVER)
	{
		avt_dbg(&sensor->sd, "correct cci register minver: %d (0x%x)\n",
				cci_minver, sensor->cci_reg.reg.layout_version);
	}
	else
	{
		avt_err(&sensor->sd, "cci reg minver mismatch! read: %d (0x%x) expected: %d\n",
				cci_minver, sensor->cci_reg.reg.layout_version, CCI_REG_LAYOUT_MINVER);
		ret = -EINVAL;
		goto err_out;
	}

	cci_majver = (sensor->cci_reg.reg.layout_version & CCI_REG_LAYOUT_MAJVER_MASK) >> CCI_REG_LAYOUT_MAJVER_SHIFT;

	if (cci_majver == CCI_REG_LAYOUT_MAJVER)
	{
		avt_dbg(&sensor->sd, "correct cci register majver: %d (0x%x)\n",
				cci_majver, sensor->cci_reg.reg.layout_version);
	}
	else
	{
		avt_err(&sensor->sd, "cci reg majver mismatch! read: %d (0x%x) expected: %d\n",
				cci_majver, sensor->cci_reg.reg.layout_version, CCI_REG_LAYOUT_MAJVER);
		ret = -EINVAL;
		goto err_out;
	}

err_out:
	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static int bcrm_version_check(struct i2c_client *client)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	u32 value = 0;
	int ret;

	MUTEX_LOCK(&sensor->lock);
	/* reading the BCRM version */
	ret = bcrm_read32(sensor,BCRM_VERSION_32R,&value);

	if (ret < 0)
	{
		avt_err(&sensor->sd, "regmap_read failed (%d)", ret);
		goto err_out;
	}

	avt_dbg(&sensor->sd, "bcrm version (driver): 0x%x (maj: 0x%x min: 0x%x)\n",
			BCRM_DEVICE_VERSION,
			BCRM_MAJOR_VERSION,
			BCRM_MINOR_VERSION);

	avt_dbg(&sensor->sd, "bcrm version (camera): 0x%x (maj: 0x%x min: 0x%x)\n",
			value,
			(value & 0xffff0000) >> 16,
			(value & 0x0000ffff));

	ret = (value >> 16) == BCRM_MAJOR_VERSION ? 1 : 0;

err_out:
	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static int gcprm_version_check(struct i2c_client *client)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	u32 value = sensor->gencp_reg.gcprm_layout_version;

	MUTEX_LOCK(&sensor->lock);
	avt_dbg(&sensor->sd, "gcprm version (driver): 0x%x (maj: 0x%x min: 0x%x)\n",
			GCPRM_DEVICE_VERSION,
			GCPRM_MAJOR_VERSION,
			GCPRM_MINOR_VERSION);

	avt_dbg(&sensor->sd, "gcprm version (camera): 0x%x (maj: 0x%x min: 0x%x)\n",
			value,
			(value & 0xffff0000) >> 16,
			(value & 0x0000ffff));
	MUTEX_UNLOCK(&sensor->lock);

	return (value & 0xffff0000) >> 16 == GCPRM_MAJOR_VERSION ? 1 : 0;
}

/* implementation of driver attibutes published in sysfs */

static ssize_t availability_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	MUTEX_LOCK(&sensor->lock);

	dev_info(dev, "%s[%d]: %s", __func__, __LINE__, __FILE__);
	ret = sprintf(buf, "%d\n", sensor->open_refcnt == 0 ? 1 : 0);

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t cci_register_layout_version_show(struct device *dev,
												struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	MUTEX_LOCK(&sensor->lock);
	ret = sprintf(buf, "%d\n", sensor->cci_reg.reg.layout_version);
	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t bcrm_feature_inquiry_reg_show(struct device *dev,
											 struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;
	union bcrm_feature_inquiry_reg feature_inquiry_reg;

	/* reading the Feature inquiry register */
	ret = regmap_bulk_read(sensor->regmap64,
						   sensor->cci_reg.reg.bcrm_addr + BCRM_FEATURE_INQUIRY_64R,
						   &feature_inquiry_reg.value, 1);

	if (ret < 0)
	{
		avt_err(&sensor->sd, "regmap_bulk_read BCRM_FEATURE_INQUIRY_64R failed (%ld)", ret);
		return ret;
	}

	ret = sprintf(buf, "0x%016llX\n", feature_inquiry_reg.value);

	return ret;
}

static ssize_t bcrm_feature_inquiry_reg_text_show(struct device *dev,
												  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret = 0;

	ret = sprintf(buf, "reverse_x_avail                 %d\n"
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
					   "exposure active line available  %d\n",
				  sensor->feature_inquiry_reg.feature_inq.reverse_x_avail,
				  sensor->feature_inquiry_reg.feature_inq.reverse_y_avail,
				  sensor->feature_inquiry_reg.feature_inq.intensity_auto_precedence_avail,
				  sensor->feature_inquiry_reg.feature_inq.black_level_avail,
				  sensor->feature_inquiry_reg.feature_inq.gain_avail,
				  sensor->feature_inquiry_reg.feature_inq.gamma_avail,
				  sensor->feature_inquiry_reg.feature_inq.contrast_avail,
				  sensor->feature_inquiry_reg.feature_inq.saturation_avail,
				  sensor->feature_inquiry_reg.feature_inq.hue_avail,
				  sensor->feature_inquiry_reg.feature_inq.white_balance_avail,
				  sensor->feature_inquiry_reg.feature_inq.sharpness_avail,
				  sensor->feature_inquiry_reg.feature_inq.exposure_auto_avail,
				  sensor->feature_inquiry_reg.feature_inq.gain_auto_avail,
				  sensor->feature_inquiry_reg.feature_inq.white_balance_auto_avail,
				  sensor->feature_inquiry_reg.feature_inq.device_temperature_avail,
				  sensor->feature_inquiry_reg.feature_inq.acquisition_abort,
				  sensor->feature_inquiry_reg.feature_inq.acquisition_frame_rate,
				  sensor->feature_inquiry_reg.feature_inq.frame_trigger,
				  sensor->feature_inquiry_reg.feature_inq.exposure_active_line_available);
	return ret;
}

static ssize_t bcrm_bayer_formats_show(struct device *dev,
									   struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "0x%04X\n", sensor->bayer_inquiry_reg.value);

	return ret;
}

static ssize_t bcrm_bayer_formats_text_show(struct device *dev,
											struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "monochrome_avail %d\n"
					   "bayer_GR_avail   %d\n"
					   "bayer_RG_avail   %d\n"
					   "bayer_GB_avail   %d\n"
					   "bayer_BG_avail   %d\n",
				  sensor->bayer_inquiry_reg.bayer_pattern.monochrome_avail,
				  sensor->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail,
				  sensor->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail,
				  sensor->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail,
				  sensor->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail);

	return ret;
}

static ssize_t bcrm_mipi_formats_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	MUTEX_LOCK(&sensor->lock);
	ret = sprintf(buf, "0x%016llX\n", sensor->avail_mipi_reg.value);
	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t bcrm_mipi_formats_text_show(struct device *dev,
										   struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));

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
				   sensor->avail_mipi_reg.avail_mipi.yuv420_8_leg_avail,
				   sensor->avail_mipi_reg.avail_mipi.yuv420_8_avail,
				   sensor->avail_mipi_reg.avail_mipi.yuv420_10_avail,
				   sensor->avail_mipi_reg.avail_mipi.yuv420_8_csps_avail,
				   sensor->avail_mipi_reg.avail_mipi.yuv420_10_csps_avail,
				   sensor->avail_mipi_reg.avail_mipi.yuv422_8_avail,
				   sensor->avail_mipi_reg.avail_mipi.yuv422_10_avail,
				   sensor->avail_mipi_reg.avail_mipi.rgb888_avail,
				   sensor->avail_mipi_reg.avail_mipi.rgb666_avail,
				   sensor->avail_mipi_reg.avail_mipi.rgb565_avail,
				   sensor->avail_mipi_reg.avail_mipi.rgb555_avail,
				   sensor->avail_mipi_reg.avail_mipi.rgb444_avail,
				   sensor->avail_mipi_reg.avail_mipi.raw6_avail,
				   sensor->avail_mipi_reg.avail_mipi.raw7_avail,
				   sensor->avail_mipi_reg.avail_mipi.raw8_avail,
				   sensor->avail_mipi_reg.avail_mipi.raw10_avail,
				   sensor->avail_mipi_reg.avail_mipi.raw12_avail,
				   sensor->avail_mipi_reg.avail_mipi.raw14_avail,
				   sensor->avail_mipi_reg.avail_mipi.jpeg_avail);
}

static ssize_t ignore_mipi_formats_text_show(struct device *dev,
											 struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));

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
				   sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_8_leg_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_8_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_10_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_8_csps_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_10_csps_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_8_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_10_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.rgb888_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.rgb666_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.rgb565_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.rgb555_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.rgb444_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.raw6_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.raw7_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.raw8_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.raw10_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.raw14_avail,
				   sensor->ignore_avail_mipi_reg.avail_mipi.jpeg_avail);
}

static ssize_t device_capabilities_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));

	return sprintf(buf, "0x%016llX\n", sensor->cci_reg.reg.device_capabilities.value);
}

static ssize_t device_capabilities_text_show(struct device *dev,
											 struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	MUTEX_LOCK(&sensor->lock);

	ret = sprintf(buf, "user_name        %d\n"
					   "bcrm             %d\n"
					   "gencp            %d\n"
					   "string_encoding  %s\n"
					   "family_name      %d\n",
				  sensor->cci_reg.reg.device_capabilities.caps.user_name,
				  sensor->cci_reg.reg.device_capabilities.caps.bcrm,
				  sensor->cci_reg.reg.device_capabilities.caps.gencp,
				  sensor->cci_reg.reg.device_capabilities.caps.string_encoding == CCI_CAPS_SE_ASCII ? "ASCII" : sensor->cci_reg.reg.device_capabilities.caps.string_encoding == CCI_CAPS_SE_UTF8 ? "UTF8"
																											: sensor->cci_reg.reg.device_capabilities.caps.string_encoding == CCI_CAPS_SE_UTF16	 ? "UTF16"
																																																 : "unknown string encoding",
				  sensor->cci_reg.reg.device_capabilities.caps.family_name);

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t device_guid_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%s\n", sensor->cci_reg.reg.device_guid);

	return ret;
}

static ssize_t manufacturer_name_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%s\n", sensor->cci_reg.reg.manufacturer_name);

	return ret;
}

static ssize_t model_name_show(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%s\n", sensor->cci_reg.reg.model_name);

	return ret;
}

static ssize_t family_name_show(struct device *dev,
								struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%s\n", sensor->cci_reg.reg.family_name);

	return ret;
}

static ssize_t lane_count_show(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%d\n", sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes);

	return ret;
}

static ssize_t lane_capabilities_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));

	return sprintf(buf, "0x%02X\n", sensor->lane_capabilities.value);
}

static ssize_t device_version_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));

	return sprintf(buf, "%s\n", sensor->cci_reg.reg.device_version);
}

static ssize_t firmware_version_show(struct device *dev,
									 struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%u.%u.%u.%u\n",
				  sensor->cam_firmware_version.device_firmware.special_version,
				  sensor->cam_firmware_version.device_firmware.major_version,
				  sensor->cam_firmware_version.device_firmware.minor_version,
				  sensor->cam_firmware_version.device_firmware.patch_version);

	return ret;
}

static ssize_t manufacturer_info_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));

	return sprintf(buf, "%s\n", sensor->cci_reg.reg.manufacturer_info);
}

static ssize_t serial_number_show(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));

	return sprintf(buf, "%s\n", sensor->cci_reg.reg.serial_number);
}

static ssize_t user_defined_name_show(struct device *dev,
									  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));

	return sprintf(buf, "%s\n", sensor->cci_reg.reg.user_defined_name);
}

static ssize_t driver_version_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d:%d:%d:%d\n",
				   DRV_VER_MAJOR, DRV_VER_MINOR,
				   DRV_VER_PATCH, DRV_VER_BUILD);
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

	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	MUTEX_LOCK(&sensor->lock);

	ret = kstrtoint(buf, 10, &debug);
	if (ret < 0)
	{

		MUTEX_UNLOCK(&sensor->lock);
		return ret;
	}

	MUTEX_UNLOCK(&sensor->lock);

	return count;
}

static ssize_t mipiclk_show(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	uint32_t avt_current_clk = 0;

	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));

	ret = regmap_read(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_CLOCK_32RW, &avt_current_clk);

	ret = sprintf(buf, "%d\n", avt_current_clk);

	return ret;
}

static ssize_t mipiclk_store(struct device *dev,
							 struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	uint32_t avt_next_clk = 0;
	uint32_t avt_current_clk = 0;

	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	struct i2c_client *client = to_i2c_client(dev);
	MUTEX_LOCK(&sensor->lock);

	ret = kstrtouint(buf, 10, &avt_next_clk);
	if (ret < 0)
	{
		goto out;
	}

	dev_info(&client->dev, "%s+[%d] request %s %u  0x%08X",
			 __func__, __LINE__,
			 buf, avt_next_clk, avt_next_clk);

	if ((avt_next_clk < sensor->avt_min_clk) ||
		(avt_next_clk > sensor->avt_max_clk))
	{
		dev_err(&client->dev, "%s[%d]: unsupported csi clock frequency (%u Hz, range: %u:%u Hz)!\n",
				__func__, __LINE__,
				avt_next_clk, sensor->avt_min_clk,
				sensor->avt_max_clk);
		ret = -EINVAL;
	}
	else
	{
		ret = bcrm_regmap_write(sensor, sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_CLOCK_32RW, avt_next_clk);

		dev_info(&client->dev, "%s[%d]: requested csi clock frequency %u Hz, retval %ld)\n",
				 __func__, __LINE__, avt_next_clk, ret);

		ret = regmap_read(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_CLOCK_32RW, &avt_current_clk);
		dev_info(&client->dev, "%s[%d]: requested csi clock frequency %u Hz, got %u Hz)\n",
				 __func__, __LINE__, avt_next_clk, avt_current_clk);

		if (0 < avt_current_clk)
			sensor->v4l2_fwnode_ep.link_frequencies[0] = avt_current_clk;
	}

out:
	MUTEX_UNLOCK(&sensor->lock);

	return count;
}

static ssize_t device_temperature_show(struct device *dev,
									   struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	int device_temperature;

	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	MUTEX_LOCK(&sensor->lock);

	ret = regmap_read(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_DEVICE_TEMPERATURE_32R, &device_temperature);

	ret = sprintf(buf, "%d.%d\n", device_temperature / 10, device_temperature % 10);

	MUTEX_UNLOCK(&sensor->lock);
	return ret;
}

static ssize_t softreset_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	ret = sprintf(buf, "%d\n", sensor->pending_softreset_request);

	return ret;
}

static ssize_t softreset_store(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;
	int value;

	ret = kstrtoint(buf, 10, &value);
	if (ret < 0)
	{
		return ret;
	}

	if (value > 0) {
		avt3_reset(sensor, RESET_TYPE_SOFT);

		/* Re-read and configure MIPI configuration */
		avt3_get_sensor_capabilities(&sensor->sd);
	}

	return count;
}

static ssize_t dphyreset_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	MUTEX_LOCK(&sensor->lock);

	ret = sprintf(buf, "%d\n", sensor->pending_dphyreset_request);

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t dphyreset_store(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	MUTEX_LOCK(&sensor->lock);
	ret = kstrtoint(buf, 10, &sensor->pending_dphyreset_request);
	if (ret < 0)
	{
		MUTEX_UNLOCK(&sensor->lock);
		return ret;
	}

	if (sensor->pending_dphyreset_request > 0)
	{
		avt3_dphy_reset(sensor, true);
		avt3_dphy_reset(sensor, false);
	}
	MUTEX_UNLOCK(&sensor->lock);
	return count;
}

static ssize_t streamon_delay_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	MUTEX_LOCK(&sensor->lock);

	ret = regmap_read(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_STREAM_ON_DELAY_32RW, &sensor->streamon_delay);

	ret = sprintf(buf, "%u\n", sensor->streamon_delay);

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t streamon_delay_store(struct device *dev,
									struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	MUTEX_LOCK(&sensor->lock);
	ret = kstrtoint(buf, 10, &sensor->streamon_delay);
	if (ret < 0)
	{
		MUTEX_UNLOCK(&sensor->lock);
		return ret;
	}

	ret = regmap_write(sensor->regmap32,
					   sensor->cci_reg.reg.bcrm_addr + BCRM_STREAM_ON_DELAY_32RW,
					   sensor->streamon_delay);

	MUTEX_UNLOCK(&sensor->lock);
	return count;
}

static ssize_t trigger_mode_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;
	int itmp;

	MUTEX_LOCK(&sensor->lock);

	ret = regmap_read(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_MODE_8RW, &itmp);
	if (ret < 0)
	{
		MUTEX_UNLOCK(&sensor->lock);
		return ret;
	}

	sensor->avt_trigger_status.trigger_mode_enabled = itmp;

	ret = sprintf(buf, "%d\n", (int)sensor->avt_trigger_status.trigger_mode_enabled);

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t trigger_mode_store(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;
	int itmp;

	if (sensor->is_streaming)
	{
		return -EBUSY;
	}
	MUTEX_LOCK(&sensor->lock);
	ret = kstrtoint(buf, 10, &itmp);

	if (ret < 0)
	{
		MUTEX_UNLOCK(&sensor->lock);
		return ret;
	}

	sensor->avt_trigger_status.trigger_mode_enabled = itmp ? 1 : 0;

	if (sensor->avt_trigger_status.trigger_mode_enabled)
	{
		ret = bcrm_regmap_write(sensor, sensor->regmap8,
								sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW,
								0);
		ret = bcrm_regmap_write(sensor, sensor->regmap8,
								sensor->cci_reg.reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_MODE_8RW,
								sensor->avt_trigger_status.trigger_mode_enabled);
	}
	else
	{
		ret = bcrm_regmap_write(sensor, sensor->regmap8,
								sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW,
								1);
		ret = bcrm_regmap_write(sensor, sensor->regmap8,
								sensor->cci_reg.reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_MODE_8RW,
								sensor->avt_trigger_status.trigger_mode_enabled);
	}

	dev_info(dev, "%s[%d]: sensor->avt_trigger_status.trigger_mode_enabled: %d\n",
			 __func__, __LINE__,
			 sensor->avt_trigger_status.trigger_mode_enabled);

	MUTEX_UNLOCK(&sensor->lock);
	return count;
}

static ssize_t trigger_source_show(struct device *dev,
								   struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;
	int itmp;

	MUTEX_LOCK(&sensor->lock);

	ret = regmap_read(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_SOURCE_8RW, &itmp);
	if (ret < 0)
	{
		MUTEX_UNLOCK(&sensor->lock);
		return ret;
	}

	sensor->avt_trigger_status.trigger_source = itmp;

	ret = sprintf(buf, "%d\n", (int)sensor->avt_trigger_status.trigger_source);

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t trigger_source_store(struct device *dev,
									struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;
	int itmp;

	if (sensor->is_streaming)
	{
		return -EINVAL;
	}

	MUTEX_LOCK(&sensor->lock);
	ret = kstrtoint(buf, 10, &itmp);

	if (ret < 0)
	{
		MUTEX_UNLOCK(&sensor->lock);
		return ret;
	}

	sensor->avt_trigger_status.trigger_source = itmp;

	ret = bcrm_regmap_write(sensor, sensor->regmap8,
							sensor->cci_reg.reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_SOURCE_8RW,
							sensor->avt_trigger_status.trigger_source);

	dev_info(dev, "%s[%d]: sensor->avt_trigger_status.trigger_source: %d\n",
			 __func__, __LINE__,
			 sensor->avt_trigger_status.trigger_source);

	MUTEX_UNLOCK(&sensor->lock);
	return count;
}

static ssize_t sw_trigger_store(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	if (!sensor->is_streaming)
	{
		dev_err(dev, "%s[%d]: sensor->is_streaming: %d, sensor->avt_trigger_status.trigger_mode_enabled %d, sensor->avt_trigger_status.trigger_source %d\n",
				__func__, __LINE__,
				sensor->is_streaming,
				sensor->avt_trigger_status.trigger_mode_enabled,
				sensor->avt_trigger_status.trigger_source);
		return -EINVAL;
	}

	MUTEX_LOCK(&sensor->lock);

	ret = bcrm_regmap_write(sensor, sensor->regmap8,
							sensor->cci_reg.reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_SOFTWARE_8W, 1);

	MUTEX_UNLOCK(&sensor->lock);
	return count;
}

static ssize_t hardreset_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	MUTEX_LOCK(&sensor->lock);

	ret = sprintf(buf, "%d\n", sensor->pending_softreset_request);

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t hardreset_store(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;
	int value;

	ret = kstrtoint(buf, 10, &value);
	if (ret < 0)
	{
		return ret;
	}

	if (value > 0) {
		avt3_reset(sensor, RESET_TYPE_HARD);

		/* Re-read and configure MIPI configuration */
		avt3_get_sensor_capabilities(&sensor->sd);
	}

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
static DEVICE_ATTR_RO(ignore_mipi_formats_text);
static DEVICE_ATTR_RO(bcrm_bayer_formats);
static DEVICE_ATTR_RO(bcrm_bayer_formats_text);
static DEVICE_ATTR_RW(debug_en);
static DEVICE_ATTR_RW(softreset);
static DEVICE_ATTR_RW(dphyreset);
static DEVICE_ATTR_RW(streamon_delay);
static DEVICE_ATTR_RW(hardreset);
static DEVICE_ATTR_RO(device_temperature);
static DEVICE_ATTR_RW(mipiclk);
static DEVICE_ATTR_RW(trigger_mode);
static DEVICE_ATTR_RW(trigger_source);
static DEVICE_ATTR_WO(sw_trigger);

static struct attribute *avt3_attrs[] = {
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
	&dev_attr_ignore_mipi_formats_text.attr,
	&dev_attr_bcrm_bayer_formats.attr,
	&dev_attr_bcrm_bayer_formats_text.attr,
	&dev_attr_debug_en.attr,
	&dev_attr_dphyreset.attr,
	&dev_attr_streamon_delay.attr,
	&dev_attr_softreset.attr,
	&dev_attr_hardreset.attr,
	&dev_attr_device_temperature.attr,
	&dev_attr_mipiclk.attr,
	&dev_attr_trigger_mode.attr,
	&dev_attr_trigger_source.attr,
	&dev_attr_sw_trigger.attr,
	NULL};

static struct attribute_group avt3_attr_grp = {
	.attrs = avt3_attrs,
};

static int avt3_get_fmt_available(struct i2c_client *client)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	u32 bayer_val = 0;
	int ret;
	struct avt_val64 avail_mipi;

	CLEAR(avail_mipi);

	MUTEX_LOCK(&sensor->lock);

	ret = regmap_bulk_read(sensor->regmap64,
						   sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R,
						   &avail_mipi, 1);

	if (ret < 0)
	{
		dev_err(&client->dev, "%s[%d]regmap_bulk_read (%d)\n",
				__func__, __LINE__, ret);
		goto out;
	}

	sensor->avail_mipi_reg.value = avail_mipi.u64;

	/* read the Bayer Inquiry register to check whether the camera
	 * really support the requested RAW format
	 */
	ret = regmap_read(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_BAYER_PATTERN_INQUIRY_8R, &bayer_val);

	if (ret < 0)
	{
		dev_err(&client->dev, "%s[%d]: regmap_read (%d)\n",
				__func__, __LINE__, ret);
		goto out;
	}

	sensor->bayer_inquiry_reg.value = bayer_val;

out:
	avt_dbg(&sensor->sd, "avail_mipi 0x%016llX bayer_val 0x%02X ret %d",
			sensor->avail_mipi_reg.value,
			sensor->bayer_inquiry_reg.value, ret);

	MUTEX_UNLOCK(&sensor->lock);
	return ret;
}

static int set_bayer_format(struct i2c_client *client, __u8 value)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	int ret = 0;

	avt_dbg(&sensor->sd, " %d", (uint32_t)value);

	ret = bcrm_regmap_write(sensor, sensor->regmap8,
							sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_BAYER_PATTERN_8RW, value);

	if (ret < 0)
	{
		avt_err(&sensor->sd, "i2c write failed");
		return ret;
	}

	return 0;
}

static int lockup_media_bus_fmt(struct avt3_dev *sensor, u32 mbus_code)
{

	int i;

	for (i = 0; i < sensor->available_fmts_cnt; i++)
	{
		adev_info(&sensor->i2c_client->dev, "mbus_code 0x%04x against test MEDIA_BUS_FMT 0x%04x / V4L2_PIX_FMT_ %c%c%c%c / MIPI_CSI2_DT 0x%02x",
				  mbus_code,
				  sensor->available_fmts[i].mbus_code,
				  sensor->available_fmts[i].fourcc & 0x0ff, (sensor->available_fmts[i].fourcc >> 8) & 0x0ff,
				  (sensor->available_fmts[i].fourcc >> 16) & 0x0ff, (sensor->available_fmts[i].fourcc >> 24) & 0x0ff,
				  sensor->available_fmts[i].mipi_fmt);
		if (mbus_code == sensor->available_fmts[i].mbus_code)
			return i;
	}

	return -EINVAL;
}

void set_mode_mapping(struct avt_csi_mipi_mode_mapping *pfmt,
					  u32 mbus_code, u16 mipi_fmt, u32 colorspace,
					  u32 fourcc,
					  enum bayer_format bayer_pattern, const char *name)
{
	pfmt->mbus_code = mbus_code;
	pfmt->mipi_fmt = mipi_fmt;
	pfmt->fourcc = fourcc;
	pfmt->colorspace = colorspace;
	pfmt->bayer_pattern = bayer_pattern;
	strcpy(pfmt->name, name);
}

/* ToDo: read available formats from Cam */
static int avt3_init_avail_formats(struct v4l2_subdev *sd)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct avt_csi_mipi_mode_mapping *pfmt;

	if (sd == NULL)
	{
		return -EINVAL;
	}

	avt_dbg(sd, "sensor->available_fmts_cnt %d", sensor->available_fmts_cnt);

	sensor->available_fmts_cnt = 0;

	avt_dbg(sd, "%s %s %s %s\n",
			sensor->cci_reg.reg.manufacturer_name,
			sensor->cci_reg.reg.family_name,
			sensor->cci_reg.reg.model_name,
			sensor->cci_reg.reg.device_guid);

	avt_dbg(sd, "monochrome_avail %d\n"
				"bayer_GR_avail   %d\n"
				"bayer_RG_avail   %d\n"
				"bayer_GB_avail   %d\n"
				"bayer_BG_avail   %d\n",
			sensor->bayer_inquiry_reg.bayer_pattern.monochrome_avail,
			sensor->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail,
			sensor->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail,
			sensor->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail,
			sensor->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail);

	avt_dbg(sd, "\n"
				"yuv420_8_leg_avail   %d\n"
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
			sensor->avail_mipi_reg.avail_mipi.yuv420_8_leg_avail,
			sensor->avail_mipi_reg.avail_mipi.yuv420_8_avail,
			sensor->avail_mipi_reg.avail_mipi.yuv420_10_avail,
			sensor->avail_mipi_reg.avail_mipi.yuv420_8_csps_avail,
			sensor->avail_mipi_reg.avail_mipi.yuv420_10_csps_avail,
			sensor->avail_mipi_reg.avail_mipi.yuv422_8_avail,
			sensor->avail_mipi_reg.avail_mipi.yuv422_10_avail,
			sensor->avail_mipi_reg.avail_mipi.rgb888_avail,
			sensor->avail_mipi_reg.avail_mipi.rgb666_avail,
			sensor->avail_mipi_reg.avail_mipi.rgb565_avail,
			sensor->avail_mipi_reg.avail_mipi.rgb555_avail,
			sensor->avail_mipi_reg.avail_mipi.rgb444_avail,
			sensor->avail_mipi_reg.avail_mipi.raw6_avail,
			sensor->avail_mipi_reg.avail_mipi.raw7_avail,
			sensor->avail_mipi_reg.avail_mipi.raw8_avail,
			sensor->avail_mipi_reg.avail_mipi.raw10_avail,
			sensor->avail_mipi_reg.avail_mipi.raw12_avail,
			sensor->avail_mipi_reg.avail_mipi.raw14_avail,
			sensor->avail_mipi_reg.avail_mipi.jpeg_avail);

	sensor->available_fmts = kmalloc(sizeof(sensor->available_fmts[0]) * AVT3_MAX_FORMAT_ENTRIES, GFP_KERNEL);

	if (!sensor->available_fmts)
	{
		dev_err(&client->dev,
				"%s[%d]: not enough memory to store list of available formats",
				__func__, __LINE__);
		return -ENOMEM;
	}

	pfmt = sensor->available_fmts;

  #define add_format_unconditional(mbus_code, mipi_fmt, colorspace, fourcc, bayer_pattern) \
    set_mode_mapping(pfmt, mbus_code, mipi_fmt, colorspace, fourcc, bayer_pattern, #mbus_code); \
    sensor->available_fmts_cnt++; \
    pfmt++;

  #define add_format_gen(avail_field_name, mbus_code, mipi_fmt, colorspace, fourcc, bayer_pattern) \
    if(sensor->avail_mipi_reg.avail_mipi.avail_field_name && !sensor->ignore_avail_mipi_reg.avail_mipi.avail_field_name) { \
      adev_info(&client->dev, "add MEDIA_BUS_FMT_" #mbus_code "/V4L2_PIX_FMT_" #fourcc "/MIPI_CSI2_DT_" #mipi_fmt " to list of available formats %d - %d:%d", bayer_pattern, \
                sensor->avail_mipi_reg.avail_mipi.avail_field_name, sensor->ignore_avail_mipi_reg.avail_mipi.avail_field_name); \
      add_format_unconditional(MEDIA_BUS_FMT_ ## mbus_code, MIPI_CSI2_DT_ ## mipi_fmt, colorspace, V4L2_PIX_FMT_ ## fourcc, bayer_pattern); \
    }

  #define add_format_srgb(avail_field_name, mbus_code, mipi_fmt, fourcc) \
    add_format_gen(avail_field_name, mbus_code, mipi_fmt, V4L2_COLORSPACE_SRGB, fourcc, bayer_ignore)

  #define add_format_raw(pattern_avail_field, avail_field_name, mbus_code, mipi_fmt, fourcc, bayer_format) \
		if(sensor->bayer_inquiry_reg.bayer_pattern.pattern_avail_field) {\
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

	pfmt->mbus_code = -EINVAL;

	avt_dbg(&sensor->sd, "available_fmts_cnt %d", sensor->available_fmts_cnt);

	return sensor->available_fmts_cnt;
}

/* hard reset depends on gpio-pins, needs to be completed on
   suitable board instead of imx8mp-evk */
static int perform_hard_reset(struct avt3_dev *sensor)
{
	dev_info(&sensor->i2c_client->dev, "%s[%d]",
			 __func__, __LINE__);

	if (!sensor->reset_gpio)
	{
		dev_info(&sensor->i2c_client->dev, "%s[%d]: - ignore reset request because missing reset gpio",
				 __func__, __LINE__);
		sensor->pending_hardtreset_request = 0;

		return -1;
	}

	dev_info(&sensor->i2c_client->dev, "%s[%d]: - request hard reset by triggering reset gpio",
			 __func__, __LINE__);
	gpiod_set_value_cansleep(sensor->reset_gpio, GPIOD_OUT_HIGH);

	/* Todo: implement usefull camera power cycle timing,
	 eventually based on additional dts parameters,
	 can't be checked on imx8mp-evk because shared GPIO lines */
	//	avt3_power(sensor, false);
	usleep_range(5000, 10000);
	//	avt3_power(sensor, true);
	//	usleep_range(5000, 10000);

	gpiod_set_value_cansleep(sensor->reset_gpio, GPIOD_OUT_LOW);
	//	usleep_range(1000, 2000);

	//	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(20000, 25000);

	return 0;
}

static const int heartbeat_default = 0x80;

static int heartbeat_write_default(struct avt3_dev *sensor) {
	int ret = regmap_write(sensor->regmap8, cci_cmd_tbl[HEARTBEAT].address, heartbeat_default);
	if(ret != 0) {
		avt_err(&sensor->sd, "Heartbeat write failed (regmap_write returned %d)", ret);
		return -1;
	}
	return 0;
}

static int heartbeat_read(struct avt3_dev *sensor, int *heartbeat) {
	int ret = regmap_read(sensor->regmap8, cci_cmd_tbl[HEARTBEAT].address, heartbeat);
	if(ret != 0) {
		avt_err(&sensor->sd, "Heartbeat read failed (regmap_read returned %d)", ret);
		return -1;
	}
	return 0;
}

static int heartbeat_supported(struct avt3_dev *sensor) {
	unsigned int heartbeat;

	int ret = heartbeat_write_default(sensor);
	if(ret != 0) {
		avt_err(&sensor->sd, "Heartbeat support detection failed (heartbeat_write returned %d)", ret);
		return ret;
	}

	ret = heartbeat_read(sensor, &heartbeat);
	if(ret != 0) {
		avt_err(&sensor->sd, "Heartbeat support detection failed (heartbeat_read returned %d)", ret);
		return -1;
	}

	return heartbeat != 0;
}

static int wait_camera_available(struct avt3_dev *sensor, bool use_heartbeat) {
	static const unsigned long max_time_ms = 10000;
	static const unsigned long delay_ms = 400;
	u64 const start_jiffies = get_jiffies_64();
	bool device_available = false;
	u64 duration_ms = 0;

	avt_info(&sensor->sd, "Waiting for camera to respond to I2C transfers...");
	do
	{
		usleep_range(delay_ms*1000, (delay_ms+1)*1000);
		device_available = avt3_detect(sensor->i2c_client) == 0;
		duration_ms = jiffies_to_msecs(get_jiffies_64() - start_jiffies);
	} while((duration_ms < max_time_ms) && !device_available);

	avt_dbg(&sensor->sd, "Camera is responding again");

	if(!device_available) {
		return -1;
	}

	if(!use_heartbeat) {
		avt_info(&sensor->sd, "Heartbeat NOT supported, waiting %dms before continuing", add_wait_time_ms);
		usleep_range(add_wait_time_ms*1000, (add_wait_time_ms+1)*1000);
		avt_info(&sensor->sd, "Done waiting, let's hope for the best...");

	} else {
		int heartbeat, ret;
		avt_info(&sensor->sd, "Heartbeat supported, waiting for heartbeat to become active");

		ret = heartbeat_write_default(sensor);
		if(ret != 0) {
			avt_err(&sensor->sd, "Heartbeat write failed (regmap_write returned %d)", ret);
			return ret;
		}

		do
		{
			usleep_range(delay_ms*1000, (delay_ms+1)*1000);
			heartbeat_read(sensor, &heartbeat);
			if(ret < 0) {
				return -1;
			}
			duration_ms = jiffies_to_msecs(get_jiffies_64() - start_jiffies);
		} while((duration_ms < max_time_ms) && ((heartbeat == 0) || (heartbeat == heartbeat_default)));

		if(heartbeat == 0) {
			avt_err(&sensor->sd, "Camera not reconnected (heartbeat timeout)");
			return -1;
		}

		avt_info(&sensor->sd, "Heartbeat active");
	}

	return 0;
}

static int avt3_reset(struct avt3_dev *sensor, enum avt_reset_type reset_type)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;
	int heartbeat;

	dev_info(&client->dev, "%s[%d]",
			 __func__, __LINE__);

	MUTEX_LOCK(&sensor->lock);

	heartbeat = heartbeat_supported(sensor);
	if(heartbeat < 0) {
		avt_err(&sensor->sd, "Heartbeat detection failed");
		ret = -1;
		goto out;
	}

	if(reset_type == RESET_TYPE_HARD) {
		sensor->pending_hardtreset_request = 1;
		ret = perform_hard_reset(sensor);
		if (ret < 0) {
			dev_err(&client->dev, "perform_hard_reset request failed (%d)\n", ret);
			goto out;
		}
	} else {
		sensor->pending_softreset_request = 1;
		ret = regmap_write(sensor->regmap8, cci_cmd_tbl[SOFT_RESET].address, 1);
		if (ret < 0) {
			dev_err(&client->dev, "avt3_soft_reset request by calling regmap_write failed (%d)\n", ret);
			goto out;
		}
	}

	ret = wait_camera_available(sensor, heartbeat == 1);

	if(ret != 0) {
		avt_err(&sensor->sd, "Camera failed to come back online");
		goto out;
	}

	if(reset_type == RESET_TYPE_HARD) {
		sensor->pending_hardtreset_request = 0;
	} else {
		sensor->pending_softreset_request = 0;
	}

out:
	MUTEX_UNLOCK(&sensor->lock);
	return ret;
}

static void avt3_dphy_reset(struct avt3_dev *sensor, bool bResetPhy)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;
	int ival = bResetPhy;

	dev_info(&client->dev, "%s[%d]", __func__, __LINE__);

	ret = regmap_write(sensor->regmap32,
					   sensor->cci_reg.reg.bcrm_addr + BCRM_PHY_RESET_8RW,
					   ival);

	if (ret < 0)
	{
		dev_err(&client->dev, "%s[%d]: avt3_dphy_reset request by calling regmap_write CSI2_PHY_RESET_32RW failed (%d)\n",
				__func__, __LINE__, ret);
		goto out;
	}

out:
	sensor->pending_dphyreset_request = 0;
}

/* --------------- Subdev Operations --------------- */
/* -- Code needs to be completed, e.g. power off the cam and setup on power on to support standby, hybernate, ... --  */
static int avt3_core_ops_s_power(struct v4l2_subdev *sd, int on)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	int ret = 0;

	MUTEX_LOCK(&sensor->lock);

	dev_info(&sensor->i2c_client->dev, "%s[%d]+: on %d, sensor->power_count %d",
			 __func__, __LINE__, on, sensor->power_count);

	/* Update the power count. */
	if (on)
		sensor->power_count++;
	else
		sensor->power_count--;

	WARN_ON(sensor->power_count < 0);
	WARN_ON(sensor->power_count > 1);

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static int avt3_pad_ops_get_fmt(struct v4l2_subdev *sd,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
								struct v4l2_subdev_state *sd_state,
#else
								struct v4l2_subdev_pad_config *cfg,
#endif
								struct v4l2_subdev_format *format)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	dev_info(&sensor->i2c_client->dev, "%s[%d]",
			 __func__, __LINE__);

	if (format->pad != 0)
	{
		avt_err(sd, "format->pad != 0");
		return -EINVAL;
	}

	MUTEX_LOCK(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		dev_info(&sensor->i2c_client->dev, "%s[%d]", __func__, __LINE__);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
		fmt = v4l2_subdev_get_try_format(&sensor->sd, sd_state, format->pad);
#else
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg, format->pad);
#endif
	}
	else
	{
		dev_info(&sensor->i2c_client->dev, "%s[%d]: %u x %u 0x%04X", __func__, __LINE__,
				 sensor->mbus_framefmt.width, sensor->mbus_framefmt.height,
				 sensor->mbus_framefmt.code);
		fmt = &sensor->mbus_framefmt;
	}

	format->format = *fmt;

	MUTEX_UNLOCK(&sensor->lock);

	return 0;
}

static void avt3_calc_compose(const struct avt3_dev * const camera,
			      const struct v4l2_rect * const crop,
			      u32 *width,u32 *height,
			      const struct avt3_binning_info **info)
{
	const u32 type = camera->curr_binning_type;
	const struct avt3_binning_info * const infos = camera->binning_infos[type];
	const size_t cnt = camera->binning_info_cnt[type];
	const struct v4l2_rect * const min = &camera->min_rect;
	const struct v4l2_rect * const max = &camera->sensor_rect;
	const bool x_changed = *width != camera->mbus_framefmt.width;
	const bool y_changed = *height != camera->mbus_framefmt.height;
	const bool type_changed = type != camera->curr_binning_info->type;
	const struct avt3_binning_info *best;
	struct v4l2_rect scaled_crop = *crop;
	struct v4l2_rect binning_rect = {0};

	best = camera->curr_binning_info;

	if (x_changed || y_changed || type_changed) {
		u32 min_error = U32_MAX;
		int i;

		for (i = 0; i < cnt; i++) {
			const struct avt3_binning_info * const cur = &infos[i];
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

	dev_info(&camera->i2c_client->dev,"Selected binning %dx%d type: %s\n",
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

	dev_info(&camera->i2c_client->dev,"Selected crop (%u,%u) %ux%u\n",
		 scaled_crop.left,scaled_crop.top,
		 scaled_crop.width,scaled_crop.height);

	*info = best;
}

static int avt3_try_fmt_internal(struct v4l2_subdev *sd,
				 struct v4l2_mbus_framefmt *fmt,
				 const struct avt3_binning_info **new_binning)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	int i;

	avt3_calc_compose(sensor,&sensor->curr_rect,&fmt->width,&fmt->height,
			  new_binning);

	dev_info(&sensor->i2c_client->dev, "%s[%d]",
			 __func__, __LINE__);
	avt_dbg(&sensor->sd, "fmt->width %d, fmt->height %d, fmt->code 0x%04X, "
		"sensor->available_fmts_cnt %d, sensor->mbus_framefmt.code 0x%04X",
			  fmt->width, fmt->height, fmt->code,
			  sensor->available_fmts_cnt,
			  sensor->mbus_framefmt.code);


	for (i = 0; i < sensor->available_fmts_cnt; i++)
	{
		avt_dbg(&sensor->sd, "loop %d: fmt->width %d, fmt->height %d, "
		 	"sensor->mbus_framefmt.code 0x%04X, "
			"sensor->available_fmts[%d].mbus_code 0x%04X, "
			"fmt->code 0x%04X",
				  i, fmt->width, fmt->height,
				  sensor->mbus_framefmt.code,
				  i,
				  sensor->available_fmts[i].mbus_code,
				  fmt->code);
		if (sensor->available_fmts[i].mbus_code == fmt->code)
		{
			break;
		}
	}

	if (i == sensor->available_fmts_cnt)
	{
		avt_dbg(sd, "format fmt->code 0x%04X not found in available formats [ToDo: error handling incomplete]", fmt->code);
		fmt->code = sensor->mbus_framefmt.code;
		//return -EINVAL;
	}

	memset(fmt->reserved, 0, sizeof(fmt->reserved));

	fmt->colorspace = sensor->available_fmts[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return 0;
}

static int avt_update_exposure_limits(struct v4l2_subdev *sd) {
	struct avt3_dev *sensor = to_avt3_dev(sd);
	int ret;
	u64 exp_min, exp_max, exp_inc;

	ret = bcrm_read64(sensor, BCRM_EXPOSURE_TIME_MIN_64R, &exp_min);
	if(ret < 0) {
		avt_err(sd, "Failed to read minimum exposure: %d", ret);
		goto err;
	}

	ret = bcrm_read64(sensor, BCRM_EXPOSURE_TIME_MAX_64R, &exp_max);
	if(ret < 0) {
		avt_err(sd, "Failed to read maximum exposure: %d", ret);
		goto err;
	}

	ret = bcrm_read64(sensor, BCRM_EXPOSURE_TIME_INC_64R, &exp_inc);
	if(ret < 0) {
		avt_err(sd, "Failed to read exposure increment: %d", ret);
		goto err;
	}

	{
		struct v4l2_ctrl * exp_ctrl = avt3_ctrl_find(sensor, V4L2_CID_EXPOSURE);
		if(exp_ctrl != NULL) {
			__v4l2_ctrl_modify_range(exp_ctrl, exp_min, exp_max, exp_inc, exp_ctrl->default_value);
		}
	}

	{
		struct v4l2_ctrl* exp_abs_ctrl = avt3_ctrl_find(sensor, V4L2_CID_EXPOSURE_ABSOLUTE);
		if(exp_abs_ctrl != NULL) {
			__v4l2_ctrl_modify_range(exp_abs_ctrl, exp_min / EXP_ABS, exp_max / EXP_ABS, exp_inc / EXP_ABS, exp_abs_ctrl->default_value);
		}
	}

	{
		struct v4l2_ctrl *exp_auto_min_ctrl = avt3_ctrl_find(sensor, V4L2_CID_EXPOSURE_AUTO_MIN);
		struct v4l2_ctrl *exp_auto_max_ctrl = avt3_ctrl_find(sensor, V4L2_CID_EXPOSURE_AUTO_MAX);

		if(exp_auto_min_ctrl != NULL && exp_auto_max_ctrl != NULL) {
			__v4l2_ctrl_modify_range(exp_auto_min_ctrl, exp_min, exp_max, exp_inc, exp_min);
			__v4l2_ctrl_modify_range(exp_auto_max_ctrl, exp_min, exp_max, exp_inc, exp_max);
			__v4l2_ctrl_s_ctrl_int64(exp_auto_min_ctrl, exp_min);
			__v4l2_ctrl_s_ctrl_int64(exp_auto_max_ctrl, exp_max);
		}
	}

err:
	return ret;
}


static int avt3_pad_ops_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_format *format)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	const struct avt3_binning_info *new_binning = NULL;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_mbus_framefmt *fmt;
	bool pending_fmt_change = false;

	int ret;

	avt_dbg(sd, "%s[%d]",
			 __func__, __LINE__);
	avt_dbg(sd, "%d x %d, format.code 0x%04X, format.pad %d",
			format->format.width, format->format.height, format->format.code, format->pad);

	if (format->pad != 0)
		return -EINVAL;

	MUTEX_LOCK(&sensor->lock);
	if (sensor->is_streaming) {
		ret = -EBUSY;
		goto out;
	}

	if (mbus_fmt->code != MEDIA_BUS_FMT_CUSTOM) {
		ret = avt3_try_fmt_internal(sd, mbus_fmt, &new_binning);
		if (ret)
			goto out;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		avt_dbg(sd,  "format->which == V4L2_SUBDEV_FORMAT_TRY");
		fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
	} else {
		avt_dbg(sd,  "format->which != V4L2_SUBDEV_FORMAT_TRY");
		fmt = &sensor->mbus_framefmt;


		if (new_binning != sensor->curr_binning_info) {
			sensor->curr_binning_info = new_binning;
			sensor->pending_mode_change = true;
		}

		if (mbus_fmt->code != sensor->mbus_framefmt.code) {
			pending_fmt_change = true;
		}
	}


	if (mbus_fmt->code == MEDIA_BUS_FMT_CUSTOM) {
		if (sensor->mode == AVT_BCRM_MODE ) {
			*mbus_fmt = sensor->mbus_framefmt;
		} else {
			//Reset cropping if genicam for csi2 mode is selected
			sensor->curr_rect.left = 0;
			sensor->curr_rect.top = 0;
			sensor->curr_rect.width = mbus_fmt->width;
			sensor->curr_rect.height = mbus_fmt->height;
		}
	}

	*fmt = *mbus_fmt;

	if(pending_fmt_change && mbus_fmt->code != MEDIA_BUS_FMT_CUSTOM) {
		ret = avt3_ctrl_write(sensor->i2c_client, V4L2_AV_CSI2_PIXELFORMAT, sensor->mbus_framefmt.code);

		if(ret < 0) {
			avt_err(sd, "Failed setting pixel format in camera: %d", ret);
			goto out;
		}

		ret = avt_update_exposure_limits(sd);
	}

out:
	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static int avt3_ctrl_write(struct i2c_client *client, enum avt_ctrl ctrl_id, __u32 value)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	int ret = 0;
	unsigned int reg = 0;
	int length = 0;
	int fmtidx;

	__u8 bayer_temp = 0;

	avt_dbg(&sensor->sd, "switch (ctrl_id) %x ", ctrl_id);

	switch (ctrl_id)
	{

	case V4L2_AV_CSI2_STREAMON:
    {
      unsigned int	acquisition_state;

      ret = regmap_read(sensor->regmap8,
              BCRM_ACQUISITION_STOP_8RW, &acquisition_state);
      if (0 != acquisition_state) {
        adev_info(&client->dev, 
          "V4L2_AV_CSI2_STREAMON called but cam is streaming already. acquisition_state %d, sensor->is_streaming %d",
          acquisition_state, sensor->is_streaming);
        dump_stack();
        return -EINVAL;
      }
	  }

		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_STREAMON %d", value);
		reg = BCRM_ACQUISITION_START_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_AV_CSI2_STREAMOFF:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_STREAMOFF %d", value);
		reg = BCRM_ACQUISITION_STOP_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_AV_CSI2_ABORT:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_ABORT %d", value);
		reg = BCRM_ACQUISITION_ABORT_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_AV_CSI2_WIDTH:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_WIDTH %d", value);
		reg = BCRM_IMG_WIDTH_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_HEIGHT:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_HEIGHT %d", value);
		reg = BCRM_IMG_HEIGHT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_X:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_OFFSET_X %d", value);
		reg = BCRM_IMG_OFFSET_X_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_Y:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_OFFSET_Y %d", value);
		reg = BCRM_IMG_OFFSET_Y_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_HFLIP:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_HFLIP %d", value);
		reg = BCRM_IMG_REVERSE_X_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_VFLIP:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_VFLIP %d", value);
		reg = BCRM_IMG_REVERSE_Y_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_AV_CSI2_PIXELFORMAT:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_PIXELFORMAT %d 0x%04X", value, value);
		reg = BCRM_IMG_MIPI_DATA_FORMAT_32RW;
		length = AV_CAM_DATA_SIZE_32;

		fmtidx = lockup_media_bus_fmt(sensor, value);

		if (fmtidx == -EINVAL || fmtidx >= sensor->available_fmts_cnt)
		{
			adev_info(&client->dev, "not supported by the host, lockup_media_bus_fmt returned fmtidx %d for V4L2_AV_CSI2_PIXELFORMAT %d 0x%04X",
					  fmtidx,
					  value, value);
			dump_stack();
			return -EINVAL;
		}

		adev_info(&client->dev, "lockup_media_bus_fmt returned fmtidx %d for V4L2_AV_CSI2_PIXELFORMAT_W %d 0x%04X, V4L2_PIX_FMT %c%c%c%c, MIPI_CSI2_DT 0x%02x, bayer_pattern %d",
				  fmtidx,
				  sensor->available_fmts[fmtidx].mbus_code,
				  sensor->available_fmts[fmtidx].mbus_code,
				  sensor->available_fmts[fmtidx].fourcc & 0x0ff, (sensor->available_fmts[fmtidx].fourcc >> 8) & 0x0ff,
				  (sensor->available_fmts[fmtidx].fourcc >> 16) & 0x0ff, (sensor->available_fmts[fmtidx].fourcc >> 24) & 0x0ff,
				  sensor->available_fmts[fmtidx].mipi_fmt,
				  sensor->available_fmts[fmtidx].bayer_pattern);

		value = sensor->available_fmts[fmtidx].mipi_fmt;
		bayer_temp = sensor->available_fmts[fmtidx].bayer_pattern;
		break;

	case V4L2_AV_CSI2_CHANGEMODE:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_CHANGEMODE ", __func__, __LINE__);

		if (value == 1)
		{
			ret = avt3_set_gencp(client);
		}
		else
		{
			ret = avt3_set_bcrm(client);
		}
		return ret;

	default:
		dev_err(&client->dev, "%s[%d]: unknown ctrl 0x%x\n", __func__, __LINE__, ctrl_id);
		return -EINVAL;
	}

  avt_dbg(&sensor->sd, "reg %x, length %d, vc->value0 0x%x\n", reg, length, value);

  switch (length)
  {
  case AV_CAM_DATA_SIZE_8:
    ret = bcrm_regmap_write(sensor, sensor->regmap8,
                sensor->cci_reg.reg.bcrm_addr + reg, value);
    break;
  case AV_CAM_DATA_SIZE_16:
    ret = bcrm_regmap_write(sensor, sensor->regmap16,
                sensor->cci_reg.reg.bcrm_addr + reg, value);
    break;
  case AV_CAM_DATA_SIZE_32:
    ret = bcrm_regmap_write(sensor, sensor->regmap32,
                sensor->cci_reg.reg.bcrm_addr + reg, value);
    break;
  default:
    dev_err(&client->dev, "%s[%d]: unknown length %d\n", __func__, __LINE__, length);
  }

  if (ret < 0)
  {
    dev_err(&client->dev, "%s[%d]: bcrm_regmap_write failed\n",
        __func__, __LINE__);
    return ret;
  }

  /* set pixelformat followed by set matching bayer format */
  if (ctrl_id == V4L2_AV_CSI2_PIXELFORMAT && bayer_temp != bayer_ignore)
  {
    ret = set_bayer_format(client, bayer_temp);
    if (ret < 0)
    {
      dev_err(&client->dev, "%s[%d]: bcrm_regmap_write failed, ret %d\n",
          __func__, __LINE__, ret);
      return ret;
    }
  }

  return 0;
}

static int avt3_queryctrl(struct v4l2_subdev *sd,
						  struct v4l2_query_ext_ctrl *qctrl)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);

	int ret = 0;
	s64 vals64;
	s32 s32tmp;

	avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d - code should be rewritten",
			 qctrl->id, qctrl->type);

	switch (qctrl->id)
	{

	/* BLACK LEVEL is deprecated and thus we use Brightness */
	case V4L2_CID_BRIGHTNESS:
		if (!sensor->feature_inquiry_reg.feature_inq.black_level_avail)
			return -EINVAL;

		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_BLACK_LEVEL_MIN_32R, &s32tmp);
		qctrl->minimum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_BLACK_LEVEL_MAX_32R, &s32tmp);
		qctrl->maximum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_BLACK_LEVEL_INC_32R, &s32tmp);
		qctrl->step = s32tmp;
		qctrl->default_value = qctrl->minimum + (qctrl->maximum - qctrl->minimum) / 2;
		qctrl->flags = V4L2_CTRL_FLAG_SLIDER;
		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Brightness");

		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_BRIGHTNESS [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type, qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);

		ret = 0;
		break;

	case V4L2_CID_CONTRAST:
		if (!sensor->feature_inquiry_reg.feature_inq.contrast_avail)
			return -EINVAL;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_CONTRAST_VALUE_MIN_32R, &s32tmp);
		qctrl->minimum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_CONTRAST_VALUE_MAX_32R, &s32tmp);
		qctrl->maximum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_CONTRAST_VALUE_INC_32R, &s32tmp);
		qctrl->step = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_CONTRAST_VALUE_32RW, &s32tmp);
		qctrl->default_value = s32tmp;
		qctrl->flags = V4L2_CTRL_FLAG_SLIDER;
		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Contrast");

		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_CONTRAST [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type, qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);
		ret = 0;
		break;

	case V4L2_CID_HUE:
		if (!sensor->feature_inquiry_reg.feature_inq.hue_avail)
			return -EINVAL;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_HUE_MIN_32R, &s32tmp);
		qctrl->minimum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_HUE_MAX_32R, &s32tmp);
		qctrl->maximum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_HUE_INC_32R, &s32tmp);
		qctrl->step = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_HUE_32RW,
						  &s32tmp);
		qctrl->default_value = s32tmp;

		qctrl->flags = V4L2_CTRL_FLAG_SLIDER;
		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Hue");

		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_HUE [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type, qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);

		ret = 0;
		break;

	case V4L2_CID_SATURATION:
		if (!sensor->feature_inquiry_reg.feature_inq.saturation_avail)
			return -EINVAL;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_SATURATION_MIN_32R, &s32tmp);
		qctrl->minimum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_SATURATION_MAX_32R, &s32tmp);
		qctrl->maximum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_SATURATION_INC_32R, &s32tmp);
		qctrl->step = s32tmp;

		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_SATURATION_32RW, &s32tmp);
		qctrl->default_value = s32tmp;

		qctrl->default_value = qctrl->minimum + (qctrl->maximum - qctrl->minimum) / 2;
		qctrl->flags = V4L2_CTRL_FLAG_SLIDER;
		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Saturation");

		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_SATURATION [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type, qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);
		ret = 0;
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		if (!sensor->feature_inquiry_reg.feature_inq.white_balance_auto_avail)
			return -EINVAL;

		qctrl->flags = 0;
		qctrl->minimum = 0;
		qctrl->maximum = 2;
		qctrl->step = 1;
		qctrl->default_value = 0;
		qctrl->type = V4L2_CTRL_TYPE_BITMASK;
		strcpy(qctrl->name, "auto white balance");
		ret = 0;
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_AUTO_WHITE_BALANCE [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type, qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);

		break;

	case V4L2_CID_RED_BALANCE:
		qctrl->flags = V4L2_CTRL_FLAG_SLIDER;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_RED_BALANCE_RATIO_MIN_64R,
							   &vals64, 1);
		qctrl->minimum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_RED_BALANCE_RATIO_MAX_64R,
							   &vals64, 1);
		qctrl->maximum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_RED_BALANCE_RATIO_INC_64R,
							   &vals64, 1);
		qctrl->step = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_RED_BALANCE_RATIO_64RW,
							   &vals64, 1);
		qctrl->default_value = vals64;

		qctrl->type = V4L2_CTRL_TYPE_INTEGER64;
		strcpy(qctrl->name, "Red Balance");
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_RED_BALANCE [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);
		ret = 0;
		break;

	case V4L2_CID_BLUE_BALANCE:
		qctrl->flags = V4L2_CTRL_FLAG_SLIDER;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_BLUE_BALANCE_RATIO_MIN_64R,
							   &vals64, 1);
		qctrl->minimum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_BLUE_BALANCE_RATIO_MAX_64R,
							   &vals64, 1);
		qctrl->maximum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_BLUE_BALANCE_RATIO_INC_64R,
							   &vals64, 1);
		qctrl->step = vals64;

		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_BLUE_BALANCE_RATIO_64RW,
							   &vals64, 1);
		qctrl->default_value = vals64;

		qctrl->type = V4L2_CTRL_TYPE_INTEGER64;
		strcpy(qctrl->name, "Blue Balance");

		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_BLUE_BALANCE [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);

		ret = 0;
		break;

	case V4L2_CID_HFLIP:
		if (!sensor->feature_inquiry_reg.feature_inq.reverse_x_avail)
			return -EINVAL;
		qctrl->minimum = 0;
		qctrl->maximum = 1;
		qctrl->step = 1;
		qctrl->default_value = 0;
		qctrl->flags = 0;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_HFLIP [%lld, %lld]:%lld %lld: supported %d",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value,
				 sensor->feature_inquiry_reg.feature_inq.reverse_x_avail);
		strcpy(qctrl->name, "H-Flip");
		ret = 0;
		break;

	case V4L2_CID_VFLIP:
		if (!sensor->feature_inquiry_reg.feature_inq.reverse_y_avail)
			return -EINVAL;
		qctrl->minimum = 0;
		qctrl->maximum = 1;
		qctrl->step = 1;
		qctrl->default_value = 0;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_VFLIP [%lld, %lld]:%lld %lld: supported %d",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value,
				 sensor->feature_inquiry_reg.feature_inq.reverse_y_avail);
		strcpy(qctrl->name, "V-Flip");
		ret = 0;
		break;

	case V4L2_CID_AUTOGAIN:
		if (!sensor->feature_inquiry_reg.feature_inq.gain_auto_avail)
			return -EINVAL;
		qctrl->minimum = 0;
		qctrl->maximum = 2;
		qctrl->step = 1;
		qctrl->default_value = 0;
		qctrl->type = V4L2_CTRL_TYPE_BITMASK;
		strcpy(qctrl->name, "Auto Gain");
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_AUTOGAIN [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);
		ret = 0;
		break;

	case V4L2_CID_DO_WHITE_BALANCE:
		if (!sensor->feature_inquiry_reg.feature_inq.white_balance_avail)
			return -EINVAL;
		qctrl->minimum = 0;
		qctrl->maximum = 7;
		qctrl->step = 0;
		qctrl->default_value = 0;
		qctrl->type = V4L2_CTRL_TYPE_BITMASK;
		strcpy(qctrl->name, "do white balance");
		ret = 0;
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_DO_WHITE_BALANCE [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);
		break;

	case V4L2_CID_GAIN:
		if (!sensor->feature_inquiry_reg.feature_inq.gain_avail)
			return -EINVAL;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_GAIN_MIN_64R,
							   &vals64, 1);
		qctrl->minimum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_GAIN_MAX_64R,
							   &vals64, 1);
		qctrl->maximum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_GAIN_INC_64R,
							   &vals64, 1);
		qctrl->step = vals64;

		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_GAIN_64RW,
							   &vals64, 1);
		qctrl->default_value = vals64;

		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_GAIN [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);

		qctrl->type = V4L2_CTRL_TYPE_INTEGER64;
		strcpy(qctrl->name, "Gain");
		ret = 0;
		break;

	case V4L2_CID_EXPOSURE:
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_MIN_64R,
							   &vals64, 1);
		qctrl->minimum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_MAX_64R,
							   &vals64, 1);
		qctrl->maximum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_INC_64R,
							   &vals64, 1);
		qctrl->step = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_64RW,
							   &vals64, 1);
		qctrl->default_value = vals64;

		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_EXPOSURE [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);
		qctrl->type = V4L2_CTRL_TYPE_INTEGER64;
		strcpy(qctrl->name, "Exposure");
		ret = 0;
		break;

	case V4L2_CID_SHARPNESS:
		if (!sensor->feature_inquiry_reg.feature_inq.sharpness_avail)
			return -EINVAL;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_SHARPNESS_MIN_32R, &s32tmp);
		qctrl->minimum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_SHARPNESS_MAX_32R, &s32tmp);
		qctrl->maximum = s32tmp;
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_SHARPNESS_INC_32R, &s32tmp);
		qctrl->step = s32tmp;

		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + BCRM_SHARPNESS_32RW, &s32tmp);
		qctrl->default_value = s32tmp;

		qctrl->default_value = qctrl->minimum + (qctrl->maximum - qctrl->minimum) / 2;
		qctrl->flags = V4L2_CTRL_FLAG_SLIDER;
		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		strcpy(qctrl->name, "Sharpness");
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_SHARPNESS [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);
		ret = 0;
		break;

	case V4L2_CID_GAMMA:
		if (!sensor->feature_inquiry_reg.feature_inq.gamma_avail)
			return -EINVAL;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_GAMMA_MIN_64R,
							   &vals64, 1);
		qctrl->minimum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_GAMMA_MAX_64R,
							   &vals64, 1);
		qctrl->maximum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_GAMMA_INC_64R,
							   &vals64, 1);
		qctrl->step = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_GAMMA_64RW,
							   &vals64, 1);
		qctrl->default_value = vals64;

		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_GAMMA [%lld, %lld]:%lld %lld",
				 qctrl->id, qctrl->type,
				 qctrl->minimum, qctrl->maximum, qctrl->step, qctrl->default_value);
		qctrl->type = V4L2_CTRL_TYPE_INTEGER64;
		strcpy(qctrl->name, "Gamma");
		ret = 0;
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		if (!sensor->feature_inquiry_reg.feature_inq.exposure_auto_avail)
		{
			return -EINVAL;
		}
		qctrl->minimum = 0;
		qctrl->maximum = 3;
		qctrl->step = 0;
		qctrl->default_value = 0;
		qctrl->type = V4L2_CTRL_TYPE_BITMASK;
		strcpy(qctrl->name, "Exposure Auto");
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_EXPOSURE_AUTO: supported %d",
				 qctrl->id, qctrl->type, sensor->feature_inquiry_reg.feature_inq.exposure_auto_avail);
		ret = 0;
		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_EXPOSURE_ABSOLUTE",
				 qctrl->id, qctrl->type);
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_MIN_64R,
							   &vals64, 1);
		qctrl->minimum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_MAX_64R,
							   &vals64, 1);
		qctrl->maximum = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_INC_64R,
							   &vals64, 1);
		qctrl->step = vals64;
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_64RW,
							   &qctrl->default_value, 1);
		qctrl->default_value = vals64;

		qctrl->type = V4L2_CTRL_TYPE_INTEGER64;
		strcpy(qctrl->name, "Exposure Absolute");
		avt_info(sd, "V4L2_CID_EXPOSURE_ABSOLUTE: \n"
					 "   BCRM_EXPOSURE_TIME_MIN_64R       %lld, \n"
					 "   BCRM_EXPOSURE_TIME_MAX_64R       %lld, \n"
					 "   BCRM_EXPOSURE_TIME_INC_64R %lld, \n"
					 "   BCRM_EXPOSURE_TIME_64RW          %lld",
				 qctrl->minimum,
				 qctrl->maximum,
				 qctrl->step,
				 qctrl->default_value);
		ret = 0;
		break;

	case V4L2_CID_3A_LOCK:
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_3A_LOCK",
				 qctrl->id, qctrl->type);

		qctrl->minimum = 0;
		qctrl->maximum = 0x0111;
		qctrl->step = 0;
		qctrl->default_value = 0;
		qctrl->type = V4L2_CTRL_TYPE_BITMASK;
		strcpy(qctrl->name, "3A Lock");
		ret = 0;
		break;

	case V4L2_CID_LINK_FREQ:
		avt_err(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_LINK_FREQ -- call not supported",
				qctrl->id, qctrl->type);
		ret = -EINVAL;
		break;

	case V4L2_CID_PIXEL_RATE:
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_PIXEL_RATE",
				 qctrl->id, qctrl->type);
		ret = -EINVAL;
		break;

	case V4L2_CID_TEST_PATTERN:
		avt_info(sd, "qctrl->id 0x%08X, qctrl->type %d case V4L2_CID_TEST_PATTERN",
				 qctrl->id, qctrl->type);
		ret = -EINVAL;
		break;

	default:
		avt_info(sd, "not supported qctrl->id 0x%08X qctrl->type %d", qctrl->id, qctrl->type);
		qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
		ret = -EINVAL;
		break;
	}

	return 0;
}

static int avt3_ioctl_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *vc)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct avt3_dev *sensor = to_avt3_dev(sd);

	unsigned int reg = 0;
	int length = 0;
	struct v4l2_query_ext_ctrl qctrl;
	int ret = 0;
	uint64_t val64 = 0;

	vc->value = 0;

	switch (vc->id)
	{
		/* BLACK LEVEL is deprecated and thus we use Brightness */
	case V4L2_CID_BRIGHTNESS:
		avt_info(sd, "V4L2_CID_BRIGHTNESS");
		reg = BCRM_BLACK_LEVEL_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_CID_GAMMA:
		avt_info(sd, "V4L2_CID_GAMMA");
		reg = BCRM_GAMMA_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;
	case V4L2_CID_CONTRAST:
		avt_info(sd, "V4L2_CID_CONTRAST");
		reg = BCRM_CONTRAST_VALUE_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		avt_info(sd, "V4L2_CID_DO_WHITE_BALANCE");
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		avt_info(sd, "V4L2_CID_AUTO_WHITE_BALANCE");
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_CID_SATURATION:
		avt_info(sd, "V4L2_CID_SATURATION");
		reg = BCRM_SATURATION_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

	case V4L2_CID_HUE:
		avt_info(sd, "V4L2_CID_HUE");
		reg = BCRM_HUE_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

	case V4L2_CID_RED_BALANCE:
		avt_info(sd, "V4L2_CID_RED_BALANCE");
		reg = BCRM_RED_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;
	case V4L2_CID_BLUE_BALANCE:
		avt_info(sd, "V4L2_CID_BLUE_BALANCE");
		reg = BCRM_BLUE_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		avt_info(sd, "V4L2_CID_EXPOSURE_ABSOLUTE");
		reg = BCRM_EXPOSURE_TIME_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_EXPOSURE:
		avt_info(sd, "V4L2_CID_EXPOSURE");
		reg = BCRM_EXPOSURE_TIME_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_GAIN:
		avt_info(sd, "V4L2_CID_GAIN");
		reg = BCRM_GAIN_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_AUTOGAIN:
		avt_info(sd, "V4L2_CID_AUTOGAIN");
		reg = BCRM_GAIN_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_CID_SHARPNESS:
		avt_info(sd, "V4L2_CID_SHARPNESS");
		reg = BCRM_SHARPNESS_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

	default:
		dev_err(&sensor->i2c_client->dev, "%s[%d]: default or not supported %d 0x%08X\n",
				__func__, __LINE__, vc->id, vc->id);
		return -EINVAL;
	}

	CLEAR(qctrl);

	qctrl.id = vc->id;
	ret = avt3_queryctrl(sd, &qctrl);

	if (ret < 0)
	{
		dev_err(&sensor->i2c_client->dev, "%s[%d]: avt3_queryctrl failed: ret %d\n", __func__, __LINE__, ret);
		return ret;
	}

	/* overwrite the queryctrl max value for auto features by 2 because 1 is value for do */
	if (vc->id == V4L2_CID_AUTOGAIN ||
		vc->id == V4L2_CID_AUTO_WHITE_BALANCE)
		qctrl.maximum = 2;

	switch (length)
	{
	case AV_CAM_DATA_SIZE_8:
		ret = regmap_read(sensor->regmap8,
						  sensor->cci_reg.reg.bcrm_addr + reg, &vc->value);
		break;
	case AV_CAM_DATA_SIZE_16:
		ret = regmap_read(sensor->regmap16,
						  sensor->cci_reg.reg.bcrm_addr + reg, &vc->value);
		break;
	case AV_CAM_DATA_SIZE_32:
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + reg, &vc->value);
		break;
	case AV_CAM_DATA_SIZE_64:
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + reg, (char *)&val64, 1);
		break;
	default:
		dev_err(&client->dev, "%s[%d]: unknown length %d\n", __func__, __LINE__, length);
	}

	if (vc->id == V4L2_CID_EXPOSURE_ABSOLUTE)
	{
		dev_info(&sensor->i2c_client->dev, "%s[%d]: V4L2_CID_EXPOSURE_ABSOLUTE values need to be translated\n", __func__, __LINE__);
		//		/* Absolute (1ns to 100ns) */
		vc->value = val64 / 100;
	}

	/* BCRM Auto Exposure changes -> Refer to BCRM document */
	if (vc->id == V4L2_CID_EXPOSURE_AUTO)
	{

		if (vc->value && 0x02 == 2)
			/* continous mode, Refer BCRM doc */
			vc->value = V4L2_EXPOSURE_AUTO;
		else
			/* OFF for off & once mode, Refer BCRM doc */
			vc->value = V4L2_EXPOSURE_MANUAL;
	}

	/* BCRM Auto Gain/WB changes -> Refer to BCRM document */
	if (vc->id == V4L2_CID_AUTOGAIN ||
		vc->id == V4L2_CID_AUTO_WHITE_BALANCE)
	{

		if (vc->value && 0x02 == 2)
			/* continous mode, Refer BCRM doc */
			vc->value = true;
		else
			/* OFF for off & once mode, Refer BCRM doc */
			vc->value = false;
	}

	return ret;
}

static int read_control_value(struct avt3_dev *camera,s64 *value,
			      const u16 reg,const u8 size)
{
	int ret;
	switch (size) {
	case AV_CAM_DATA_SIZE_8: {
		s8 temp;
		ret = bcrm_read8(camera,reg,&temp);
		if (ret < 0)
			return ret;

		*value = temp;

		break;
	}
	case AV_CAM_DATA_SIZE_16: {
		s16 temp;
		ret = bcrm_read16(camera,reg,&temp);
		if (ret < 0)
			return ret;

		*value = temp;

		break;
	}
	case AV_CAM_DATA_SIZE_32: {
		s32 temp;
		ret = bcrm_read32(camera,reg,&temp);
		if (ret < 0)
			return ret;

		*value = temp;

		break;
	}
	case AV_CAM_DATA_SIZE_64: {
		s64 temp;
		ret = bcrm_read64(camera,reg,&temp);
		if (ret < 0)
			return ret;

		*value = temp;

		break;
	}
	default:
		return -EINVAL;
	}

	return 0;
}

static void avt3_ctrl_to_reg(const u32 cid,s64 * value)
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

static void avt3_ctrl_from_reg(const u32 cid,s64 * value)
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

static int avt3_update_ctrl_value(struct avt3_dev *camera,
				  struct v4l2_ctrl *ctrl,const u16 reg,
				  const u8 data_size )
{
	int ret;
	s64 value;

	ret = read_control_value(camera,&value,
				 reg,
				 data_size);

	if (ret < 0)
		return ret;

	avt3_ctrl_from_reg(ctrl->id,&value);

	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_INTEGER:
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

static int avt3_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{

	struct avt3_dev *sensor = container_of(ctrl->handler, struct avt3_dev, v4l2_ctrl_hdl);
	struct v4l2_control c;
	int ret = 0;

	avt_dbg(&sensor->sd, "ctrl->id %d", ctrl->id);


	if (ctrl->id == V4L2_CID_BINNING_SETTING) {
		ctrl->p_new.p_area->width = sensor->curr_binning_info->hfact;
		ctrl->p_new.p_area->height = sensor->curr_binning_info->vfact;
		return 0;
	}

	if (ctrl->priv) {
		const struct avt_ctrl_mapping * const ctrl_mapping = ctrl->priv;


		return avt3_update_ctrl_value(sensor,ctrl,
					      ctrl_mapping->reg_offset,
					      ctrl_mapping->data_size);
	}

	c.id = ctrl->id;
	ret = avt3_ioctl_g_ctrl(&sensor->sd, &c);
	ctrl->val = c.value;

	return ret;
}

static struct v4l2_ctrl* avt3_ctrl_find(struct avt3_dev *camera,u32 id)
{
	int i;

	for (i = 0; i < AVT_MAX_CTRLS; i++)
	{
		struct v4l2_ctrl * ctrl = camera->avt3_ctrls[i];

		if (ctrl && ctrl->id == id)
		{
			return ctrl;
		}
	}

	return NULL;
}

static struct regmap* avt3_get_regmap_by_size(struct avt3_dev *camera,u8 data_size)
{
	switch (data_size)
	{
		case AV_CAM_DATA_SIZE_8:
   			return camera->regmap8;
		case AV_CAM_DATA_SIZE_16:
			return camera->regmap16;
		case AV_CAM_DATA_SIZE_32:
			return camera->regmap32;
		case AV_CAM_DATA_SIZE_64:
			return camera->regmap64;
		default:
			return NULL;
	}
}

static void avt3_update_sw_ctrl_state(struct avt3_dev *camera)
{
	struct v4l2_ctrl * sw_trigger_ctrl = avt3_ctrl_find(camera,V4L2_CID_TRIGGER_SOFTWARE);

	if (sw_trigger_ctrl)
	{
		bool sw_trigger_enabled = (camera->avt_trigger_status.trigger_source == 4)
                && camera->avt_trigger_status.trigger_mode_enabled;

		v4l2_ctrl_activate(sw_trigger_ctrl,sw_trigger_enabled);
	}
	else
	{
		avt_warn(&camera->sd,"Software trigger control not found!");
	}
}

static const struct v4l2_event avt3_source_change_event = {
	.type = V4L2_EVENT_SOURCE_CHANGE,
	.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
};

static void avt3_ctrl_changed(struct avt3_dev *camera,
			      const struct v4l2_ctrl * const ctrl)
{
	switch (ctrl->id)
	{
	case V4L2_CID_TRIGGER_MODE:
		camera->avt_trigger_status.trigger_mode_enabled = ctrl->val;
		avt3_update_sw_ctrl_state(camera);
		break;
	case V4L2_CID_TRIGGER_SOURCE:
		camera->avt_trigger_status.trigger_source = ctrl->val;
		avt3_update_sw_ctrl_state(camera);
		break;
	case V4L2_CID_TRIGGER_ACTIVATION:
		camera->avt_trigger_status.trigger_activation = ctrl->val;
		break;
	case V4L2_CID_EXPOSURE_AUTO_MIN: {
		struct v4l2_ctrl *max_ctrl;

		max_ctrl = avt3_ctrl_find(camera,V4L2_CID_EXPOSURE_AUTO_MAX);

		if (max_ctrl == NULL)
			break;

		__v4l2_ctrl_modify_range(max_ctrl,*ctrl->p_new.p_s64,
					 max_ctrl->maximum,max_ctrl->step,
					 max_ctrl->default_value);

		break;
	}
	case V4L2_CID_EXPOSURE_AUTO_MAX: {
		struct v4l2_ctrl *min_ctrl;

		min_ctrl = avt3_ctrl_find(camera,V4L2_CID_EXPOSURE_AUTO_MIN);

		if (min_ctrl == NULL)
			break;

		__v4l2_ctrl_modify_range(min_ctrl,min_ctrl->minimum,
					 *ctrl->p_new.p_s64,min_ctrl->step,
					 min_ctrl->default_value);

		break;
	}
	case V4L2_CID_GAIN_AUTO_MIN: {
		struct v4l2_ctrl *max_ctrl;

		max_ctrl = avt3_ctrl_find(camera,V4L2_CID_GAIN_AUTO_MAX);

		if (max_ctrl == NULL)
			break;

		__v4l2_ctrl_modify_range(max_ctrl,*ctrl->p_new.p_s64,
					 max_ctrl->maximum,max_ctrl->step,
					 max_ctrl->default_value);

		break;
	}
	case V4L2_CID_GAIN_AUTO_MAX: {
		struct v4l2_ctrl *min_ctrl;

		min_ctrl = avt3_ctrl_find(camera,V4L2_CID_GAIN_AUTO_MIN);

		if (min_ctrl == NULL)
			break;

		__v4l2_ctrl_modify_range(min_ctrl,min_ctrl->minimum,
					 *ctrl->p_new.p_s64,min_ctrl->step,
					 min_ctrl->default_value);

		break;
	}
	case V4L2_CID_AUTOGAIN: {
		struct v4l2_ctrl *gain_ctrl;

		gain_ctrl = avt3_ctrl_find(camera,V4L2_CID_GAIN);

		if (gain_ctrl != NULL)
			__v4l2_ctrl_grab(gain_ctrl,ctrl->val);

		break;
	}
	case V4L2_CID_EXPOSURE_AUTO: {
		struct v4l2_ctrl *exp_ctrl,*exp_abs_ctrl;
		bool grabbed = (ctrl->val == V4L2_EXPOSURE_AUTO);

		exp_ctrl = avt3_ctrl_find(camera,V4L2_CID_EXPOSURE);

		if (exp_ctrl != NULL)
			__v4l2_ctrl_grab(exp_ctrl,grabbed);

		exp_abs_ctrl = avt3_ctrl_find(camera,V4L2_CID_EXPOSURE_ABSOLUTE);

		if (exp_abs_ctrl != NULL)
			__v4l2_ctrl_grab(exp_abs_ctrl,grabbed);

		break;
	}
	case AVT_CID_BINNING_SELECTOR: {
		const struct avt3_binning_info *info;
		struct v4l2_ctrl *binning_mode_ctrl;
		u32 width = camera->mbus_framefmt.width;
		u32 height = camera->mbus_framefmt.height;

		camera->curr_binning_type = ctrl->val;

		avt3_calc_compose(camera, &camera->curr_rect, &width, &height,
				  &info);

		camera->curr_binning_info = info;

		if (camera->mbus_framefmt.width != width
		    || camera->mbus_framefmt.height != height) {

			camera->mbus_framefmt.width = width;
			camera->mbus_framefmt.height = height;

			v4l2_subdev_notify_event(&camera->sd,
						 &avt3_source_change_event);
		}

		binning_mode_ctrl = avt3_ctrl_find(camera,V4L2_CID_BINNING_MODE);
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
	default:
		break;
	}

}

static int write_ctrl_value(struct avt3_dev *camera,struct v4l2_ctrl *ctrl,
		      const struct avt_ctrl_mapping * const ctrl_mapping)
{
	const u16 reg = ctrl_mapping->reg_offset;
	const u8 data_size = ctrl_mapping->data_size;
	struct regmap * const ctrl_regmap
		= avt3_get_regmap_by_size(camera,data_size);
	const u16 addr = get_bcrm_addr(camera,reg);
	s64 temp;
	int ret = 0;

	if (ctrl_mapping->data_size == AV_CAM_DATA_SIZE_64)
	{
		if (ctrl->type == V4L2_CTRL_TYPE_INTEGER64)
			temp = *ctrl->p_new.p_s64;
		else
			temp = ctrl->val;

		avt3_ctrl_to_reg(ctrl->id,&temp);

		if (ctrl_mapping->type == V4L2_CTRL_TYPE_BUTTON)
		{
			ret = bcrm_regmap_write64(camera, ctrl_regmap,
					    addr, 1);
		}
		else
		{

			ret = bcrm_regmap_write64(camera, ctrl_regmap,
					    addr, temp);
		}
	}
	else
	{
		temp = ctrl->val;
		avt3_ctrl_to_reg(ctrl->id,&temp);

		if (ctrl_mapping->type == V4L2_CTRL_TYPE_BUTTON)
		{
			ret = bcrm_regmap_write(camera, ctrl_regmap,
					  addr, 1);
		}
		else
		{
			ret = bcrm_regmap_write(camera, ctrl_regmap,
					  addr, temp);
		}
	}

	if (ret < 0)
		return ret;

	if (ctrl_mapping->avt_flags & AVT_CTRL_FLAG_READ_BACK) {
		ret =  avt3_update_ctrl_value(camera,ctrl,
					     reg,data_size);
		if (ret < 0)
			dev_err(&camera->i2c_client->dev,
				"Control read back failed with %d",
				ret);
	}


	return ret;
}

static int avt3_v4l2_ctrl_ops_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avt3_dev *sensor = container_of(ctrl->handler, struct avt3_dev, v4l2_ctrl_hdl);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;


	/* ignore if sensor is in sleep mode */
	if (sensor->power_count == 0)
	{
		avt_dbg(&sensor->sd, "ToDo: Sensor is in sleep mode. Maybe it is better to ignore ctrl->id 0x%08X, sensor->power_count %d",
				 ctrl->id, sensor->power_count);
		// return -EINVAL;
	}

	if (sensor->power_count > 1)
	{
		avt_info(&sensor->sd, "ctrl->id 0x%08X, sensor->power_count %d", ctrl->id, sensor->power_count);
	}

	if (ctrl->id == V4L2_CID_EXPOSURE_ACTIVE_LINE_MODE)
	{
		struct v4l2_ctrl *sel_ctrl,*invert_ctrl;
		u8 output_line_shift,invert,active = ctrl->val;
		u32 line_config;
		const u16 line_config_addr
			= get_bcrm_addr(sensor,BCRM_LINE_CONFIGURATION_32RW);

		sel_ctrl = avt3_ctrl_find(sensor,
					  V4L2_CID_EXPOSURE_ACTIVE_LINE_SELECTOR);

		if (sel_ctrl == NULL) {
			return -EINVAL;
		}

		output_line_shift = sel_ctrl->val * 8;

		invert_ctrl = avt3_ctrl_find(sensor,
					     V4L2_CID_EXPOSURE_ACTIVE_INVERT);

		if (invert_ctrl == NULL) {
			return -EINVAL;
		}

		invert = invert_ctrl->val ? 2 : 0;

		line_config = (active ? (1 | invert ) : 0) << output_line_shift;

		ret = bcrm_regmap_write(sensor,sensor->regmap32,
				  	line_config_addr,line_config);

		if (ret < 0)
			return ret;

		__v4l2_ctrl_grab(sel_ctrl,active);
		__v4l2_ctrl_grab(invert_ctrl,active);
	}

	if (ctrl->priv != NULL)
	{
		const struct avt_ctrl_mapping * const ctrl_mapping = ctrl->priv;


		dev_info(&client->dev, "%s[%d]: Write custom ctrl %s (%x)\n",
			 __func__, __LINE__, ctrl_mapping->attr.name, ctrl->id);

		if (ctrl_mapping->data_size != 0
		    && ctrl_mapping->reg_size != 0) {
			ret = write_ctrl_value(sensor,ctrl,ctrl_mapping);
		}


		avt3_ctrl_changed(sensor,ctrl);
	}
	else
	{
		dev_err(&sensor->i2c_client->dev,
			"%s[%d]: case default or not supported id %d, val %d\n",
			__func__, __LINE__, ctrl->id, ctrl->val);
		return -EINVAL;
	}

	return ret;
}

static const struct v4l2_ctrl_ops avt3_ctrl_ops = {
	.g_volatile_ctrl = avt3_g_volatile_ctrl,
	.s_ctrl = avt3_v4l2_ctrl_ops_s_ctrl,
};


static int avt3_fill_ctrl_config(struct avt3_dev *camera,
				 struct v4l2_ctrl_config *config,
				 const struct avt_ctrl_mapping *mapping)
{
	int ret;



	config->ops = &avt3_ctrl_ops;
	config->id = mapping->id;
	config->name = mapping->attr.name;
	config->type = mapping->type;
	config->flags = mapping->flags;

	switch (mapping->type)
	{
	case V4L2_CTRL_TYPE_MENU:
		config->min = mapping->min_value;
		config->menu_skip_mask = 0;
		config->max = mapping->max_value;
		config->qmenu = mapping->qmenu;
		ret = read_control_value(camera, &config->def,
					 mapping->reg_offset,
					 mapping->data_size);
		if (ret < 0)
			return ret;

		avt3_ctrl_from_reg(mapping->id,&config->def);

		break;
	case V4L2_CTRL_TYPE_BOOLEAN:
		config->min = 0;
		config->max = 1;
		config->step = 1;
		break;
	case V4L2_CTRL_TYPE_INTEGER:
	case V4L2_CTRL_TYPE_INTEGER64:
	case V4L2_CTRL_TYPE_STRING:
		if (!mapping->min_offset)
			config->min = mapping->min_value;
		else {
			ret = read_control_value(camera, &config->min,
						 mapping->min_offset,
						 mapping->data_size);
			if (ret < 0)
				return ret;

			avt3_ctrl_from_reg(mapping->id,&config->min);
		}

		if (!mapping->max_offset)
			config->max = mapping->max_value;
		else {
			ret = read_control_value(camera, &config->max,
						 mapping->max_offset,
						 mapping->data_size);
			if (ret < 0)
				return ret;

			avt3_ctrl_from_reg(mapping->id,&config->max);
		}

		if (!mapping->step_offset)
			config->step = mapping->step_value;
		else {
			ret = read_control_value(camera, &config->step,
						 mapping->step_offset,
						 mapping->data_size);
			if (ret < 0)
				return ret;

			avt3_ctrl_from_reg(mapping->id,&config->step);
		}

		if (!mapping->reg_offset)
			config->def = mapping->default_value;
		else {
			ret = read_control_value(camera, &config->def,
						 mapping->reg_offset,
						 mapping->data_size);
			if (ret < 0)
				return ret;

			avt3_ctrl_from_reg(mapping->id,&config->def);

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

static void avt3_ctrl_added(struct avt3_dev *camera,struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id)
	{
	case V4L2_CID_TRIGGER_MODE:
		camera->avt_trigger_status.trigger_mode_enabled = ctrl->val;
		avt3_update_sw_ctrl_state(camera);
		break;
	case V4L2_CID_TRIGGER_SOURCE:
		camera->avt_trigger_status.trigger_source = ctrl->val;
		avt3_update_sw_ctrl_state(camera);
		break;
	case V4L2_CID_TRIGGER_ACTIVATION:
		camera->avt_trigger_status.trigger_activation = ctrl->val;
		break;
	case V4L2_CID_TRIGGER_SOFTWARE:
		avt3_update_sw_ctrl_state(camera);
		break;
	case AVT_CID_FIRMWARE_VERSION: {
		const union device_firmware_version_reg *fw_version =
			&camera->cam_firmware_version;
		snprintf(ctrl->p_cur.p_char,ctrl->elem_size,
			"%02u.%02u.%02u.%08x",
			 fw_version->device_firmware.special_version,
			 fw_version->device_firmware.major_version,
			 fw_version->device_firmware.minor_version,
			 fw_version->device_firmware.patch_version);
		break;
	}
	case AVT_CID_CAMERA_NAME:  {
		snprintf(ctrl->p_cur.p_char,ctrl->elem_size,"%s %s",
			 camera->cci_reg.reg.family_name,
			 camera->cci_reg.reg.model_name);

		break;
	}
	case AVT_CID_SERIAL_NUMBER:  {
		snprintf(ctrl->p_cur.p_char,ctrl->elem_size,"%s",
			 camera->cci_reg.reg.serial_number);

		break;
	}
	case V4L2_CID_EXPOSURE_AUTO_MIN: {
		struct v4l2_ctrl *max_ctrl = NULL;

		max_ctrl = avt3_ctrl_find(camera,V4L2_CID_EXPOSURE_AUTO_MAX);

		if (max_ctrl == NULL)
			return;

		v4l2_ctrl_modify_range(ctrl,ctrl->minimum,
					 max_ctrl->default_value,ctrl->step,
					 ctrl->default_value);

		break;
	}
	case V4L2_CID_EXPOSURE_AUTO_MAX: {
		struct v4l2_ctrl *min_ctrl = NULL;

		min_ctrl = avt3_ctrl_find(camera,V4L2_CID_EXPOSURE_AUTO_MIN);

		if (min_ctrl == NULL) {
			avt_warn(&camera->sd,"V4L2_CID_EXPOSURE_AUTO_MIN not found!");
			return;
		}

		v4l2_ctrl_modify_range(ctrl,min_ctrl->default_value,
					 ctrl->maximum,ctrl->step,
					 ctrl->default_value);

		break;
	}
	case V4L2_CID_GAIN_AUTO_MIN: {
		struct v4l2_ctrl *max_ctrl = NULL;

		max_ctrl = avt3_ctrl_find(camera,V4L2_CID_GAIN_AUTO_MAX);

		if (max_ctrl == NULL)
			return;

		v4l2_ctrl_modify_range(ctrl,ctrl->minimum,
				       max_ctrl->default_value,ctrl->step,
				       ctrl->default_value);

		break;
	}
	case V4L2_CID_GAIN_AUTO_MAX: {
		struct v4l2_ctrl *min_ctrl = NULL;

		min_ctrl = avt3_ctrl_find(camera,V4L2_CID_GAIN_AUTO_MIN);

		if (min_ctrl == NULL) {
			avt_warn(&camera->sd,"V4L2_CID_EXPOSURE_AUTO_MIN not found!");
			return;
		}

		v4l2_ctrl_modify_range(ctrl,min_ctrl->default_value,
				       ctrl->maximum,ctrl->step,
				       ctrl->default_value);

		break;
	}
	default:
		break;
	}
}

static int avt3_init_controls(struct avt3_dev *sensor)
{
	struct v4l2_ctrl_config config;
	struct v4l2_ctrl *ctrl;
	int ret;
	int i, j;

	avt_dbg(&sensor->sd, "code uses now v4l2_ctrl_new_std and v4l2_query_ext_ctrl (VIDIOC_QUERY_EXT_CTRL / s64) ");

	ret = v4l2_ctrl_handler_init(&sensor->v4l2_ctrl_hdl, ARRAY_SIZE(avt_ctrl_mappings));
	if (ret < 0)
	{
		avt_err(&sensor->sd, "v4l2_ctrl_handler_init Failed");
		goto free_ctrls;
	}
	/* we can use our own mutex for the ctrl lock */
	sensor->v4l2_ctrl_hdl.lock = &sensor->lock;

	for (i = 0, j = 0; j < ARRAY_SIZE(avt_ctrl_mappings); ++j)
	{
		const struct avt_ctrl_mapping * const ctrl_mapping
			= &avt_ctrl_mappings[j];
		const s8 feat_bit = ctrl_mapping->attr.feature_avail;
		const u64 inq_reg = sensor->feature_inquiry_reg.value;

		if ((feat_bit != -1 && (inq_reg & (1 << feat_bit)) == 0)) {
			avt_info(&sensor->sd,
				 "Control %s (0x%x) not supported by camera\n",
				 ctrl_mapping->attr.name,ctrl_mapping->id);
			continue;
		}

		CLEAR(config);

		avt_dbg(&sensor->sd, "Init ctrl %s (0x%x)\n",
			 ctrl_mapping->attr.name,ctrl_mapping->id);


		avt3_fill_ctrl_config(sensor,&config,ctrl_mapping);


		sensor->avt3_ctrl_cfg[i] = config;

		ctrl = v4l2_ctrl_new_custom(&sensor->v4l2_ctrl_hdl,
					    &config,(void*)ctrl_mapping);

		if (ctrl == NULL)
		{
			avt_err(&sensor->sd,
				"Failed to init %s ctrl %d 0x%08x\n",
				sensor->avt3_ctrl_cfg[i].name,
				sensor->v4l2_ctrl_hdl.error,
				sensor->v4l2_ctrl_hdl.error);

			if (sensor->v4l2_ctrl_hdl.error == -ERANGE) {
				avt_err(&sensor->sd,
					"Invalid ctrl range min: %lld max: %lld "
					"step: %lld def: %lld",
					config.min,config.max,config.step,config.def);
			}

            		//Clear error
			sensor->v4l2_ctrl_hdl.error = 0;
			continue;
		}


		avt3_ctrl_added(sensor,ctrl);

		sensor->avt3_ctrls[i] = ctrl;
		i++;
	}

	return ret;
free_ctrls:
	v4l2_ctrl_handler_free(&sensor->v4l2_ctrl_hdl);
	return ret;
}

static void set_frameinterval(struct v4l2_fract *interval,const u64 framerate,const u64 factor)
{
	interval->denominator = (framerate * interval->numerator) / factor;

	// If the denominator and minimal framerate is not zero, try to increase the numerator by 1000
	while (interval->denominator == 0 && interval->numerator < factor)
	{
		interval->numerator *= 1000;
		interval->denominator = (framerate * interval->numerator) / factor;
	}
}

static int avt3_pad_ops_enum_frame_size(struct v4l2_subdev *sd,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
										struct v4l2_subdev_state *sd_state,
#else
										struct v4l2_subdev_pad_config *cfg,
#endif
										struct v4l2_subdev_frame_size_enum *fse)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	const struct v4l2_rect *min = &sensor->min_rect;
	const struct v4l2_rect *max = &sensor->sensor_rect;
	struct avt3_binning_info *binning_info;
	struct v4l2_rect binning_rect,scaled_crop = sensor->curr_rect;
	size_t max_frame_size;

	avt_dbg(sd, "fse->index %d, fse->which %s", fse->index,
		fse->which == V4L2_SUBDEV_FORMAT_TRY ? "V4L2_SUBDEV_FORMAT_TRY" : "V4L2_SUBDEV_FORMAT_ACTIVE");

	if (fse->pad != 0)
	{
		avt_warn(sd, "Requested pad %d not supported",fse->pad);
		return -EINVAL;
	}

#ifdef ENABLE_STEPWISE_IMAGE_SIZE
	max_frame_size = sensor->binning_info_cnt[sensor->curr_binning_type];

	if (fse->index >= max_frame_size)
	{
		avt_dbg(&sensor->sd, "fse->index(%d) >= %lu.",
			 fse->index, max_frame_size);
		return -EINVAL;
	}

	binning_info
		= &sensor->binning_infos[sensor->curr_binning_type][fse->index];


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
		avt_dbg(&sensor->sd, "fse->index(%d) >= 1.", fse->index);
		return -EINVAL;
	}
	fse->min_width = sensor->min_rect.width;
	fse->max_width = sensor->max_rect.width;
	fse->min_height = sensor->min_rect.height;
	fse->max_height = sensor->max_rect.height;

#endif
	return 0;
}

static int avt3_pad_ops_enum_frame_interval(
	struct v4l2_subdev *sd,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
	struct v4l2_subdev_state *sd_state,
#else
	struct v4l2_subdev_pad_config *cfg,
#endif
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	int i,ret;
	u64 max_framerate;

	if (fie->pad != 0)
	{
		avt_err(sd, "no pad availble. fie->index %d, fie->pad %d, fie->code 0x%04X, fie->width %d, fie->height %d",
				fie->index, fie->pad, fie->code, fie->width, fie->height);
		return -EINVAL;
	}

	if (fie->index >= 1)
	{
		avt_info(sd, "fie->index >= AVT3_NUM_FRAMERATES fie->index %d, AVT3_NUM_FRAMERATES %d, fie->pad %d, fie->code 0x%04X, fie->width %d, fie->height %d",
				 fie->index, 1, fie->pad, fie->code, fie->width, fie->height);
		return -EINVAL;
	}
	if (sensor->mbus_framefmt.code != fie->code)
	{
		avt_info(sd, "sensor->mbus_framefmt.code 0x%04X, fie->code 0x%04X",
				 sensor->mbus_framefmt.code, fie->code);
		return -EINVAL;
	}

	/*
	To enumerate frame intervals applications initialize the index, pad, which, code, width and height fields of
	struct v4l2_subdev_frame_interval_enum and call the ioctl VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL ioctl with a
	pointer to this structure. Drivers fill the rest of the structure or return an EINVAL error code if one of
	the input fields is invalid. All frame intervals are enumerable by beginning at index zero and incrementing
	by one until EINVAL is returned. */

	i = 0;
	do
	{
		if (sensor->available_fmts[i].mbus_code == fie->code)
			break;
		i++;
	} while (i < sensor->available_fmts_cnt);
	if (i == sensor->available_fmts_cnt)
	{
		avt_err(&sensor->sd, "sensor->available_fmts[%d].mbus_code unknown MEDIA_BUS_FMT_ fie->code 0x%04X", i, fie->code);
		return -EINVAL;
	}

	if (fie->width != clamp(fie->width,sensor->min_rect.width,sensor->max_rect.width)
        || fie->height != clamp(fie->height,sensor->min_rect.height,sensor->max_rect.height))
	{
		avt_err(&sensor->sd, "Frameintervals for unsupported width (%u) or height (%u) requested", fie->width,fie->height);
		return -EINVAL;
	}

	ret = bcrm_read64(sensor,BCRM_ACQUISITION_FRAME_RATE_MAX_64R,&max_framerate);

	if (ret < 0)
		return ret;

	fie->interval.numerator = 1;
	set_frameinterval(&fie->interval,max_framerate,1000000);

	return 0;
}

static int avt3_video_ops_g_frame_interval(struct v4l2_subdev *sd,
					   struct v4l2_subdev_frame_interval *fi)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);

	if (sensor->avt_trigger_status.trigger_mode_enabled) {
		return -EINVAL;
	}

	fi->interval = sensor->frame_interval;
	avt_dbg(sd, "sensor->frame_interval.denom %u, sensor->frame_interval.num %u, fi->num %d fi->denom %u",
			sensor->frame_interval.denominator, sensor->frame_interval.numerator,
			fi->interval.numerator, fi->interval.denominator);
	return 0;
}



static int avt3_video_ops_s_frame_interval(struct v4l2_subdev *sd,
					   struct v4l2_subdev_frame_interval *fi)
{
	struct avt3_dev *camera = to_avt3_dev(sd);
	int ret = 0;
	const u64 factor = 1000000L;
	u64 framerate_req,framerate_min,framerate_max;


	avt_dbg(sd, "fie->num %d fie->denom %d",
			fi->interval.numerator, fi->interval.denominator);

	MUTEX_LOCK(&camera->lock);
	if (camera->is_streaming)
	{
		ret = -EBUSY;
		goto out;
	}

	if (camera->avt_trigger_status.trigger_mode_enabled) {
		ret = -EINVAL;
		goto out;
	}

	ret = bcrm_read64(camera,BCRM_ACQUISITION_FRAME_RATE_MIN_64R,
			  &framerate_min);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
		return ret;
	}

	ret = bcrm_read64(camera,BCRM_ACQUISITION_FRAME_RATE_MAX_64R,
			  &framerate_max);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
		return ret;
	}

	if (fi->interval.numerator == 0 || fi->interval.denominator == 0) {
		camera->framerate_auto = true;
	}
	else {
		framerate_req = (fi->interval.denominator * factor) / fi->interval.numerator;
		framerate_req = clamp(framerate_req,framerate_min,framerate_max);

		set_frameinterval(&fi->interval,framerate_req,factor);

		camera->framerate_auto = false;
	}


	camera->frame_interval = fi->interval;

	avt_dbg(sd, "set fie->num %d fie->denom %d",
			fi->interval.numerator, fi->interval.denominator);

out:
	MUTEX_UNLOCK(&camera->lock);

	avt_dbg(&camera->sd, "- fie->num %d fie->denom %d --> idx",
			fi->interval.numerator, fi->interval.denominator);
	return ret;
}

static int avt3_pad_ops_enum_mbus_code(struct v4l2_subdev *sd,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
									   struct v4l2_subdev_state *sd_state,
#else
									   struct v4l2_subdev_pad_config *cfg,
#endif
									   struct v4l2_subdev_mbus_code_enum *code)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

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
		dev_warn(&client->dev, "%s[%d]: code->pad != 0 fse->index %d, code 0x%04X sensor->available_fmts_cnt %d",
				 __func__, __LINE__, code->index, code->code, sensor->available_fmts_cnt);

		return -EINVAL;
	}

	if (code->index >= sensor->available_fmts_cnt)
	{
		dev_warn(&client->dev, "%s[%d]: code->index >= sensor->available_fmts_cnt fse->index %d, code 0x%04X sensor->available_fmts_cnt %d",
				 __func__, __LINE__, code->index, code->code, sensor->available_fmts_cnt);
		return -EINVAL;
	}

	code->code = sensor->available_fmts[code->index].mbus_code;

	return 0;
}

static void avt3_controls_stream_grab(struct avt3_dev *camera,bool grabbed)
{
	int i;

	for (i = 0;i < AVT_MAX_CTRLS;i++)
	{
		struct v4l2_ctrl *ctrl = camera->avt3_ctrls[i];

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

static int avt3_video_ops_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	dev_info(&client->dev, "%s[%d]: enable %d, sensor->is_streaming %d\n"
						   "	sensor->mbus_framefmt.width     %d\n"
						   "	sensor->mbus_framefmt.height    %d\n"
						   "	sensor->mbus_framefmt.code      %d 0x%04X\n"
						   "	sensor->mbus_framefmt.ycbcr_enc %d\n",
			 __func__, __LINE__, enable, sensor->is_streaming,
			 sensor->mbus_framefmt.width,
			 sensor->mbus_framefmt.height,
			 sensor->mbus_framefmt.code,
			 sensor->mbus_framefmt.code,
			 sensor->mbus_framefmt.ycbcr_enc);

	if (sensor->mode == AVT_GENCP_MODE)
		return 0;

	MUTEX_LOCK(&sensor->lock);

	if (!enable && sensor->is_streaming)
	{
		ret = avt3_ctrl_write(sensor->i2c_client, V4L2_AV_CSI2_STREAMOFF, 1);
		sensor->is_streaming = false;

		// ToDo: eventually wait until cam has stopped streaming
	}

	if (enable && !sensor->is_streaming)
	{
		struct v4l2_ext_control vc;
		struct v4l2_rect crop_rect = sensor->curr_rect;
		struct v4l2_rect binning_rect = {0};
		const struct avt3_binning_info *binning_info = sensor->curr_binning_info;


		u64 u64FrMin = 0;
		u64 u64FrMax = 0;
		u64 value64;



		binning_rect.width = binning_info->max_width;
		binning_rect.height = binning_info->max_height;

		v4l2_rect_scale(&crop_rect,&sensor->sensor_rect,&binning_rect);


		v4l_bound_align_image(&crop_rect.width,sensor->min_rect.width,
				      binning_rect.width,3,
				      &crop_rect.height,sensor->min_rect.height,
				      binning_rect.height,3,0);

		dev_info(&sensor->i2c_client->dev,"Selected crop (%u,%u) %ux%u\n",crop_rect.left,crop_rect.top,crop_rect.width,crop_rect.height);

		dev_err(&client->dev, "%s[%d]: active area +%d:+%d %d x %d (max %d x %d, min %d x %d), code 0x%04X, HFLIP_W %d, VFLIP_W %d",
				__func__, __LINE__,
				crop_rect.left, crop_rect.top,
				crop_rect.width, crop_rect.height,
				sensor->max_rect.width, sensor->max_rect.height,
				sensor->min_rect.width, sensor->min_rect.height,
				sensor->mbus_framefmt.code,
				sensor->hflip, sensor->vflip);

		ret = bcrm_regmap_write(sensor,sensor->regmap8,
					sensor->cci_reg.reg.bcrm_addr + BCRM_BINNING_SETTING_8RW,binning_info->sel);

		if (ret < 0) {
			dev_err(&client->dev,"%s[%d]: Writing binning setting failed with: %d",__func__,__LINE__,ret);
		}


		ret = avt3_ctrl_write(sensor->i2c_client, V4L2_AV_CSI2_WIDTH, crop_rect.width);

		ret = avt3_ctrl_write(sensor->i2c_client, V4L2_AV_CSI2_OFFSET_X, crop_rect.left);

		ret = avt3_ctrl_write(sensor->i2c_client, V4L2_AV_CSI2_HEIGHT, crop_rect.height);

		ret = avt3_ctrl_write(sensor->i2c_client, V4L2_AV_CSI2_OFFSET_Y, crop_rect.top);

		ret = avt3_ctrl_write(sensor->i2c_client, V4L2_AV_CSI2_VFLIP, sensor->vflip);

		ret = avt3_ctrl_write(sensor->i2c_client, V4L2_AV_CSI2_HFLIP, sensor->hflip);


		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_MIN_64R,
							   &u64FrMin, 1);

		if (ret < 0)
		{
			dev_err(&client->dev, "regmap_read failed (%d)\n", ret);
			// goto err_out;
		}

		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_MAX_64R,
							   &u64FrMax, 1);

		if (ret < 0)
		{
			dev_err(&client->dev, "regmap_read failed (%d)\n", ret);
			// goto err_out;
		}

		if (sensor->avt_trigger_status.trigger_mode_enabled)
		{
			ret = bcrm_regmap_write(sensor, sensor->regmap8,
						sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW, 0);
			ret = bcrm_regmap_write(sensor, sensor->regmap8,
						sensor->cci_reg.reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_MODE_8RW,
						sensor->avt_trigger_status.trigger_mode_enabled);
			ret = bcrm_regmap_write(sensor, sensor->regmap8,
						sensor->cci_reg.reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_SOURCE_8RW,
						sensor->avt_trigger_status.trigger_source);
		}
		else
		{
			u32 temp32;

			/* Enable manual frame rate */
			ret = bcrm_regmap_write(sensor, sensor->regmap8,
									sensor->cci_reg.reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_MODE_8RW, 0);
			if (ret < 0)
			{
				dev_err(&client->dev, "%s[%d]: BCRM_FRAME_START_TRIGGER_MODE_8RW: bcrm_regmap_write failed (%d)\n",
						__func__, __LINE__,
						ret);
				goto out;
			}

			temp32 = sensor->framerate_auto ? 0 : 1;

			/* Enable manual frame rate */
			ret = bcrm_regmap_write(sensor, sensor->regmap8,
						sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW, temp32);
			if (ret < 0)
			{
				dev_err(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW: bcrm_regmap_write failed (%d)\n",
					__func__, __LINE__,
					ret);
				goto out;
			}

			if (!sensor->framerate_auto) {
				dev_info(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE: sensor->frame_interval.numerator %u, sensor->frame_interval.denominator %u\n",
				 	__func__, __LINE__,
				 	sensor->frame_interval.numerator, sensor->frame_interval.denominator);

				/* Save new frame rate to camera register */
				value64 = (((u64)sensor->frame_interval.denominator) * 1000000) / ((u64)sensor->frame_interval.numerator);

				dev_info(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_64RW (min: %llu req: %llu max: %llu) uHz\n",
					 __func__, __LINE__,
					 u64FrMin, value64, u64FrMax);

				vc.value64 = clamp(value64,u64FrMin,u64FrMax);

				ret = bcrm_regmap_write64(sensor, sensor->regmap64,
									  sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_64RW,
									  vc.value64);
				if (ret < 0)
				{
					dev_err(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_64RW: i2c write failed (%d)\n",
						__func__, __LINE__,
						ret);
					goto out;
				}
			}
		}


		if (debug >= 2)
			bcrm_dump(client);

		/* start streaming */
		ret = avt3_ctrl_write(client, V4L2_AV_CSI2_STREAMON, 1);

		// ToDo: probably it's better to check the status here. but this conflicts with the workaround for imx8mp delayed start
		if (!ret)
			sensor->is_streaming = enable;
	}

	avt3_controls_stream_grab(sensor,enable);

out:
	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}


int avt3_core_ops_reset(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_info(&client->dev, "%s[%d]+ %s", __func__, __LINE__, __FILE__);

	return 0;
}

int avt3_core_ops_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct avt3_dev *sensor = to_avt3_dev(sd);
	int ret = 0;
	unsigned int val = 0;

	dev_info(&client->dev, "%s[%d]: reg 0x%04llX, size %d",
			 __func__, __LINE__, reg->reg, reg->size);

	if (reg->reg & ~0xffff)
			return -EINVAL;

	if (reg->size != 1 && reg->size != 2 &&
		reg->size != 4 && reg->size != 8)
	{
		ret = -EINVAL;
	}

	switch (reg->size)
	{
	case 8:
		ret = regmap_bulk_read(sensor->regmap64,
							   reg->reg, &reg->val, 1);
		break;
	case 4:
		ret = regmap_read(sensor->regmap32, reg->reg, &val);
		reg->val = (__u64)val;
		break;
	case 2:
		ret = regmap_read(sensor->regmap16, reg->reg, &val);
		reg->val = (__u64)val;
		break;
	case 1:
		ret = regmap_read(sensor->regmap8, reg->reg, &val);
		reg->val = (__u64)val;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

int avt3_core_ops_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_info(&client->dev, "%s[%d]: reg 0x%04llX, size %u",
			 __func__, __LINE__, reg->reg, reg->size);

	return 0;
}


long avt3_core_ops_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = -ENOTTY;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct v4l2_capability *cap = arg;
	struct v4l2_i2c *i2c_reg;
	struct v4l2_csi_driver_info *info;
	struct v4l2_csi_config *config;

	char *i2c_reg_buf;

	avt_dbg(sd, "cmd 0x%08x %d %s", cmd, cmd & 0xff, __FILE__);

	switch (cmd)
	{
		/* ToDo: check to remove that code */
	case VIDIOC_QUERYCAP:
		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_QUERYCAP", __func__, __LINE__);

		strcpy(cap->driver, AVT3_DRIVER_NAME);
		cap->version = KERNEL_VERSION(0, 1, 1);
		cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
							V4L2_CAP_VIDEO_CAPTURE_MPLANE |
							V4L2_CAP_STREAMING |
							V4L2_CAP_READWRITE;
		cap->card[0] = '\0';
		cap->bus_info[0] = '\0';
		ret = 0;
		break;

	case VIDIOC_DBG_S_REGISTER:
	{
		struct v4l2_dbg_register *v4l2_dbg_reg = (struct v4l2_dbg_register *)arg;
		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_DBG_S_REGISTER reg 0x%04llX, size %u",
				 __func__, __LINE__,
				 v4l2_dbg_reg->reg, v4l2_dbg_reg->size);
		ret = 0;
		break;
	}

	case VIDIOC_DBG_G_REGISTER:
	{
		struct v4l2_dbg_register *v4l2_dbg_reg = (struct v4l2_dbg_register *)arg;
		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_DBG_G_REGISTER reg 0x%04llX, size %u",
				 __func__, __LINE__,
				 v4l2_dbg_reg->reg, v4l2_dbg_reg->size);
		ret = 0;
		break;
	}
		// client->adapter->bus_clk_rate
	case VIDIOC_R_I2C:
		//dev_info(&client->dev, "%s[%d]: cmd VIDIOC_R_I2C", __func__, __LINE__);
		i2c_reg = (struct v4l2_i2c *)arg;
		i2c_reg_buf = kzalloc(i2c_reg->num_bytes, GFP_KERNEL);
		if (!i2c_reg_buf)
			return -ENOMEM;

//		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_R_I2C i2c_reg->reg 0x%04x, i2c_reg->size %d, i2c_reg->num_bytes %d",
//				 __func__, __LINE__,
//				 i2c_reg->register_address, i2c_reg->register_size, i2c_reg->num_bytes);

		ret = regmap_bulk_read(sensor->regmap8, i2c_reg->register_address, i2c_reg_buf, i2c_reg->num_bytes);

		if (ret < 0)
			dev_info(&client->dev, "%s[%d]: i2c read failed (%d), bytes read = %d\n",
					 __func__, __LINE__, ret, i2c_reg->num_bytes);
		else
		{
			ret = copy_to_user((char *)i2c_reg->ptr_buffer, i2c_reg_buf, i2c_reg->num_bytes);

			if (ret < 0)
				dev_err(&client->dev, "%s[%d]: i2c read failed (%d), bytes read = %d\n",
						__func__, __LINE__, ret, i2c_reg->num_bytes);
		}

		kfree(i2c_reg_buf);
		break;

	case VIDIOC_W_I2C:
		i2c_reg = (struct v4l2_i2c *)arg;

		i2c_reg_buf = kzalloc(i2c_reg->num_bytes, GFP_KERNEL);
		if (!i2c_reg_buf)
			return -ENOMEM;

		ret = copy_from_user(i2c_reg_buf, (char *)i2c_reg->ptr_buffer, i2c_reg->num_bytes);

		/* TODO: check count, size and endianess!! */
		ret = regmap_bulk_write(sensor->regmap8, i2c_reg->register_address, i2c_reg_buf, i2c_reg->num_bytes);

		if (ret < 0)
		{
			dev_err(&client->dev, "%s[%d]: i2c write failed (%d), bytes written = %d\n",
					__func__, __LINE__, ret, i2c_reg->num_bytes);
		}
		else
		{
			// regmap_bulk_write returns 0 on success, but VIDIOC_W_I2C is expected to return number of bytes written
			ret = i2c_reg->num_bytes + AV_CAM_REG_SIZE;
		}

		if (i2c_reg->register_address == GENCP_CHANGEMODE_8W)
		{
			dev_info(&client->dev,"Switching mode according to ioctl request! New mode %d\n",
					 (int)i2c_reg_buf[0]);

			sensor->mode = i2c_reg_buf[0];
		}

		break;

	case VIDIOC_G_DRIVER_INFO:
		/***************************************
		 *  better to use values from sysfs as below
		 *	/sys/devices/soc0/
		 *	|-- family
		 *	|-- machine
		 *	|-- revision
		 *	|-- serial_number
		 *	|-- soc_id
		 ***************************************/

		dev_warn(&client->dev, "%s[%d]: cmd VIDIOC_G_DRIVER_INFO, better to read back from "
							   "/sys/devices/soc0/",
				 __func__, __LINE__);
		info = (struct v4l2_csi_driver_info *)arg;

		info->id.manufacturer_id = MANUFACTURER_ID_NXP;
		info->id.soc_family_id = SOC_FAMILY_ID_IMX8MP;
		info->id.driver_id = IMX8_DRIVER_ID_DEFAULT;

		info->driver_version = (DRV_VER_MAJOR << 16) + (DRV_VER_MINOR << 8) + DRV_VER_PATCH;
		info->driver_interface_version = (LIBCSI_DRV_SPEC_VERSION_MAJOR << 16) + (LIBCSI_DRV_SPEC_VERSION_MINOR << 8) + LIBCSI_DRV_SPEC_VERSION_PATCH;
		info->driver_caps = AVT_DRVCAP_MMAP;
		info->usrptr_alignment = dma_get_cache_alignment();

		ret = 0;
		break;

	case VIDIOC_G_CSI_CONFIG:
		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_G_CSI_CONFIG", __func__, __LINE__);
		config = (struct v4l2_csi_config *)arg;

		config->lane_count = sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes;
		config->csi_clock = sensor->v4l2_fwnode_ep.link_frequencies[0];

		ret = 0;
		break;

	case VIDIOC_S_CSI_CONFIG:
		// TBC: D-PHY configuration will be defined by devicetree
		config = (struct v4l2_csi_config *)arg;
		dev_warn(&client->dev, "%s[%d]: cmd VIDIOC_S_CSI_CONFIG. CSI-Config is set by devicetree.", __func__, __LINE__);

		ret = -EINVAL;
		break;
	default:
		dev_info(&client->dev, "%s[%d]: VIDIOC command %d 0x%08X not implemented yet", __func__, __LINE__, cmd & 0x0ff, cmd);
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static int avt3_core_ops_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
										 struct v4l2_event_subscription *sub)
{
	avt_dbg(sd, "event type %u", sub->type);

	switch (sub->type)
	{
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_subdev_core_ops avt3_core_ops = {
	.s_power = avt3_core_ops_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.ioctl = avt3_core_ops_ioctl,
	.reset = avt3_core_ops_reset,
	.subscribe_event = avt3_core_ops_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,

#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = avt3_core_ops_g_register,
	.s_register = avt3_core_ops_s_register,
#endif
};

static int avt3_subdev_internal_ops_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	avt_dbg(sd, "sensor->open_refcnt %d, sensor->is_streaming %d",
			sensor->open_refcnt, sensor->is_streaming);

	// stop the stream if just streaming
	if (sensor->is_streaming)
	{
		avt_err(sd, "sensor->is_streaming %d",
				sensor->is_streaming);
		// ret = avt3_video_ops_s_stream(sd, false);
	}

	if (!sensor->is_streaming)
	{
		avt3_set_bcrm(client);
	}

	sensor->open_refcnt--;
	return ret;
}
//TODO: Support multiple opens
static int avt3_subdev_internal_ops_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	// called when userspace app calls 'open'
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	avt_dbg(sd, "sensor->open_refcnt %d", sensor->open_refcnt);

	if (sensor->open_refcnt)
	{
		avt_dbg(sd, "device already opened %d", sensor->open_refcnt);
		return -EBUSY;
	}

	if (!sensor->is_streaming)
	{
		avt_dbg(sd, "force bcrm mode");
		// set BCRM mode only when sensor is not streaming
		avt3_set_bcrm(client);
	}

	sensor->open_refcnt++;

	return 0;
}

static const struct v4l2_subdev_internal_ops avt3_subdev_internal_ops = {
	.open = avt3_subdev_internal_ops_open,
	.close = avt3_subdev_internal_ops_close,
};

int avt3_video_ops_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	v4l2_dbg(2, debug, sd, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);
	return 0;
}

int v4l2_subdev_video_ops_s_mbus_config(struct v4l2_subdev *sd,
										const struct v4l2_mbus_config *cfg)
{
	v4l2_dbg(2, debug, sd, "%s[%d]: %s", __func__, __LINE__, __FILE__);
	return 0;
}

int avt3_video_ops_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parm)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	dev_info(&sensor->i2c_client->dev, "%s[%d]: %s", __func__, __LINE__, __FILE__);

	if (!parm)
		return -EINVAL;

	v4l2_dbg(2, debug, sd, "%s[%d]: parm->type %d", __func__, __LINE__, parm->type);

	if (!V4L2_TYPE_IS_CAPTURE(parm->type))
	{
		return -EINVAL;
	}

	memcpy(&parm->parm.capture, &sensor->streamcap, sizeof(struct v4l2_captureparm));

	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME | V4L2_MODE_HIGHQUALITY;
	parm->parm.capture.timeperframe = sensor->frame_interval;
	/* return latest format as has been set by avt3_video_ops_g_parm */

	return 0;
}

int avt3_video_ops_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parm)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct v4l2_fract *timeperframe = &parm->parm.capture.timeperframe;

	v4l2_dbg(2, debug, sd, "%s[%d]: %s", __func__, __LINE__, __FILE__);

	// TODO: parameter checking!!!
	if (!V4L2_TYPE_IS_CAPTURE(parm->type))
	{
		dev_info(&sensor->i2c_client->dev, "%s[%d]: wrong parm->type %d",
				 __func__, __LINE__, parm->type);
		return -EINVAL;
	}

	// TODO: parameter checking!!!!!!
	if ((timeperframe->numerator == 0) ||
		(timeperframe->denominator == 0))
	{
		timeperframe->denominator = 30; // DEFAULT_FPS;
		timeperframe->numerator = 1;
	}

	/* Copy new settings to internal structure */
	memcpy(&sensor->streamcap, &parm->parm.capture, sizeof(struct v4l2_captureparm));

	return 0;
}

static const struct v4l2_subdev_video_ops avt3_video_ops = {
	.g_frame_interval = avt3_video_ops_g_frame_interval,
	.s_frame_interval = avt3_video_ops_s_frame_interval,
	.s_stream = avt3_video_ops_s_stream,
	.querystd = avt3_video_ops_querystd,
#if !defined(CONFIG_ARCH_ZYNQMP) && !defined(DISABLE_PARM)
	.g_parm = avt3_video_ops_g_parm,
	.s_parm = avt3_video_ops_s_parm,
#endif
#if ((LINUX_VERSION_CODE) < (KERNEL_VERSION(5, 6, 0)))
	.g_mbus_config = v4l2_subdev_video_ops_g_mbus_config,
	.s_mbus_config = v4l2_subdev_video_ops_s_mbus_config,
#endif
};

static void avt3_get_compose(struct avt3_dev *camera,
		   struct v4l2_subdev_state *sd_state,
		   struct v4l2_subdev_selection *sel)
{
	const struct v4l2_mbus_framefmt *frmfmt;

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
		frmfmt = v4l2_subdev_get_try_format(&camera->sd,sd_state,
						    sel->pad);
	else
		frmfmt = &camera->mbus_framefmt;

	sel->r.left = 0;
	sel->r.top = 0;
	sel->r.width = frmfmt->width;
	sel->r.height = frmfmt->height;
}

static void avt3_get_crop(struct avt3_dev * camera,
		   struct v4l2_subdev_state *sd_state,
		   struct v4l2_subdev_selection *sel)
{
	const struct v4l2_rect *rect;

	dev_info(&camera->i2c_client->dev, "%s[%d]: %s",
		 __func__, __LINE__, __FILE__);

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY)
		rect = v4l2_subdev_get_try_crop(&camera->sd,sd_state,sel->pad);
	else
		rect = &camera->curr_rect;

	dev_info(&camera->i2c_client->dev,"%ux%u",rect->width,rect->height);

	sel->r = *rect;
}

int avt3_pad_ops_get_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_selection *sel)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	dev_info(&client->dev, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);

	if (sel->pad > 0)
		return -EINVAL;

	//No cropping or binning in genicam for csi2 mode
	if (sensor->mbus_framefmt.code == MEDIA_BUS_FMT_CUSTOM)
		return -ENODATA;

	switch (sel->target)
	{
	/* Composing bounds */
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	/* Default composing area */
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		v4l2_rect_set_size_to(&sel->r,&sensor->curr_rect);
		break;
	/* Current composing area */
	case V4L2_SEL_TGT_COMPOSE:
		avt3_get_compose(sensor,sd_state,sel);
		break;

	/* Current cropping area */
	case V4L2_SEL_TGT_CROP:
		avt3_get_crop(sensor,sd_state,sel);
		break;

	/* Cropping bounds */
	case V4L2_SEL_TGT_CROP_BOUNDS:
	/* Default cropping area */
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = sensor->max_rect;
		break;
	/* Native frame size */
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = sensor->sensor_rect;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int avt3_set_compose(struct avt3_dev *camera,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_selection *sel)
{
	struct v4l2_mbus_framefmt *frmfmt;
	const struct avt3_binning_info *info;
	const struct v4l2_rect *crop;

	if (sel->which  == V4L2_SUBDEV_FORMAT_TRY) {
		frmfmt = v4l2_subdev_get_try_format(&camera->sd, sd_state, sel->pad);
		crop = v4l2_subdev_get_try_crop(&camera->sd, sd_state, sel->pad);
	} else {
		frmfmt = &camera->mbus_framefmt;
		crop = &camera->curr_rect;
	}

	sel->r.left = 0;
	sel->r.top = 0;

	avt3_calc_compose(camera,crop,&sel->r.width,&sel->r.height,&info);

	frmfmt->width = sel->r.width;
	frmfmt->height = sel->r.height;

	if (sel->target == V4L2_SUBDEV_FORMAT_ACTIVE)
		camera->curr_binning_info = info;

	return 0;
}

static int avt3_set_crop(struct avt3_dev *camera,
			 struct v4l2_subdev_state *sd_state,
			 struct v4l2_subdev_selection *sel)
{
	const struct v4l2_rect *min = &camera->min_rect;
	const struct v4l2_rect *max = &camera->max_rect;
	struct v4l2_rect *crop_rect;
	struct v4l2_mbus_framefmt *frmfmt;
	const struct avt3_binning_info *info;

	if (sel->which  == V4L2_SUBDEV_FORMAT_TRY) {
		crop_rect = v4l2_subdev_get_try_crop(&camera->sd, sd_state, sel->pad);
		frmfmt = v4l2_subdev_get_try_format(&camera->sd, sd_state, sel->pad);
	} else {
		crop_rect = &camera->curr_rect;
		frmfmt = &camera->mbus_framefmt;
	}

	v4l_bound_align_image(&sel->r.width,min->width, max->width,3,
			      &sel->r.height,min->height,max->height,3,0);

	v4l2_rect_map_inside(&sel->r, max);

	avt3_calc_compose(camera,&sel->r,&frmfmt->width,&frmfmt->height,&info);

	frmfmt->width = max->width;
	frmfmt->height = max->height;

	*crop_rect = sel->r;

	if (sel->target == V4L2_SUBDEV_FORMAT_ACTIVE)
		camera->curr_binning_info = info;


	return 0;
}

int avt3_pad_ops_set_selection(struct v4l2_subdev *sd,
	struct v4l2_subdev_state *sd_state,
	struct v4l2_subdev_selection *sel)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	int ret = -EINVAL;


	if (sensor->is_streaming && sel->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EBUSY;

	if (sel->pad > 0)
		return -EINVAL;

	//No cropping or binning in genicam for csi2 mode
	if (sensor->mbus_framefmt.code == MEDIA_BUS_FMT_CUSTOM)
		return -EINVAL;

	MUTEX_LOCK(&sensor->lock)

	if (sel->target == V4L2_SEL_TGT_CROP)
		ret = avt3_set_crop(sensor,sd_state, sel);
	else if (sel->target == V4L2_SEL_TGT_COMPOSE)
		ret = avt3_set_compose(sensor,sd_state,sel);

	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

int avt3_pad_ops_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
								struct v4l2_mbus_frame_desc *fd)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	dev_info(&client->dev, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);
	return 0;
}

int avt3_pad_ops_set_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
								struct v4l2_mbus_frame_desc *fd)
{

	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	dev_info(&client->dev, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);
	return 0;
}
#ifdef CONFIG_MEDIA_CONTROLLER
int avt3_pad_ops_link_validate(struct v4l2_subdev *sd, struct media_link *link,
							   struct v4l2_subdev_format *source_fmt,
							   struct v4l2_subdev_format *sink_fmt)
{

	v4l2_dbg(2, debug, sd, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);
	return 0;
}
#endif /* CONFIG_MEDIA_CONTROLLER */

static const struct v4l2_subdev_pad_ops avt3_pad_ops = {
	.enum_mbus_code = avt3_pad_ops_enum_mbus_code,
	.enum_frame_size = avt3_pad_ops_enum_frame_size,
	.enum_frame_interval = avt3_pad_ops_enum_frame_interval,
	.get_fmt = avt3_pad_ops_get_fmt,
	.set_fmt = avt3_pad_ops_set_fmt,
	.get_selection = avt3_pad_ops_get_selection,
	.set_selection = avt3_pad_ops_set_selection,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0))
	.g_mbus_config = v4l2_subdev_video_ops_g_mbus_config,
	.s_mbus_config = v4l2_subdev_video_ops_s_mbus_config,
#endif
	.get_frame_desc = avt3_pad_ops_get_frame_desc,
	.set_frame_desc = avt3_pad_ops_set_frame_desc,
#ifdef CONFIG_MEDIA_CONTROLLER
	.link_validate = avt3_pad_ops_link_validate,
#endif /* CONFIG_MEDIA_CONTROLLER */
};
static const struct v4l2_subdev_ops avt3_subdev_ops = {
	.core = &avt3_core_ops,
	.video = &avt3_video_ops,
	.pad = &avt3_pad_ops,
};

static int avt3_meo_link_setup(struct media_entity *entity,
							   const struct media_pad *local,
							   const struct media_pad *remote, u32 flags)
{
	pr_info("%s[%d]", __func__, __LINE__);

	return 0;
}

int avt3_meo_get_fwnode_pad(struct fwnode_endpoint *endpoint)
{
	pr_info("%s[%d]", __func__, __LINE__);

	return 0;
}

int avt3_meo_link_validate(struct media_link *link)
{
	pr_info("%s[%d]", __func__, __LINE__);

	return 0;
}

static const struct media_entity_operations avt3_sd_media_ops = {
	.link_setup = avt3_meo_link_setup,
};


static int avt3_get_sensor_capabilities(struct v4l2_subdev *sd)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	int ret = 0;
	uint64_t value64;
	uint32_t avt_supported_lane_mask = 0;
	uint32_t avt_current_clk = 0;
	uint32_t clk;
	uint8_t bcm_mode = 0;
	u32 temp;

	/* reading the Feature inquiry register */
	ret = regmap_bulk_read(sensor->regmap64,
						   sensor->cci_reg.reg.bcrm_addr + BCRM_FEATURE_INQUIRY_64R,
						   &sensor->feature_inquiry_reg.value, 1);

	if (ret < 0)
	{
		avt_err(sd, "regmap_bulk_read BCRM_FEATURE_INQUIRY_64R failed (%d)\n", ret);
		return ret;
	}
	avt_dbg(sd, "BCRM_FEATURE_INQUIRY_64R %llu\n", sensor->feature_inquiry_reg.value);

	/* Check if requested number of lanes is supported */
	ret = regmap_read(sensor->regmap8,
					  sensor->cci_reg.reg.bcrm_addr + BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R,
					  &avt_supported_lane_mask);

	if (ret < 0)
	{
		avt_dbg(sd, "regmap_read failed (%d)\n", ret);
		return ret;
	}

	sensor->lane_capabilities.value = avt_supported_lane_mask;

	avt_dbg(sd, "supported lane config: %x", (uint32_t)avt_supported_lane_mask);

	if (!(test_bit(sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes - 1, (const long *)(&avt_supported_lane_mask))))
	{
		avt_err(sd, "requested number of lanes (%u) not supported by this camera!\n",
				sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes);
		return -EINVAL;
	}

	avt_dbg(sd, "request %u lanes.\n", sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes);

	/* Set number of lanes */
	ret = bcrm_regmap_write(sensor, sensor->regmap8,
		sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_LANE_COUNT_8RW,
		sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes);

	if (ret < 0)
	{
		avt_err(sd, "bcrm_regmap_write failed (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(sensor->regmap32,
		sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_CLOCK_MIN_32R,
		&sensor->avt_min_clk);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(sensor->regmap32,
					  sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_CLOCK_MAX_32R,
					  &sensor->avt_max_clk);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
		return ret;
	}

	avt_info(sd, "csi clocks\n"
				 "   camera range:           %9d:%9d Hz\n"
				 "   dts nr_of_link_frequencies %d\n"
				 "   dts link_frequencies[0] %9lld Hz\n",
			 sensor->avt_min_clk, sensor->avt_max_clk,
			 sensor->v4l2_fwnode_ep.nr_of_link_frequencies,
			 sensor->v4l2_fwnode_ep.link_frequencies[0]);

	if (sensor->v4l2_fwnode_ep.link_frequencies[0] < sensor->avt_min_clk ||
		sensor->v4l2_fwnode_ep.link_frequencies[0] > sensor->avt_max_clk)
	{

		avt_err(sd, "unsupported csi clock frequency (%lld Hz, range: %d:%d Hz)!\n",
				sensor->v4l2_fwnode_ep.link_frequencies[0],
				sensor->avt_min_clk,
				sensor->avt_max_clk);
		return -EINVAL;
	}

	clk = sensor->v4l2_fwnode_ep.link_frequencies[0];

	ret = bcrm_regmap_write(sensor, sensor->regmap32,
							sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_CLOCK_32RW,
							clk);
	if (ret < 0)
	{
		avt_err(sd, "regmap_write BCRM_CSI2_CLOCK_32RW failed (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(sensor->regmap32,
					  sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_CLOCK_32RW,
					  &avt_current_clk);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read BCRM_CSI2_CLOCK_32RW failed (%d)\n", ret);
		return ret;
	}

	avt_dbg(sd, "csi clock frequency (req: %lld Hz, cur: %d Hz, range: %d:%d Hz)!\n",
			sensor->v4l2_fwnode_ep.link_frequencies[0],
			avt_current_clk,
			sensor->avt_min_clk,
			sensor->avt_max_clk);

	avt_info(sd, "csi clock read from camera: %u Hz\n", avt_current_clk);

	sensor->min_rect.left = sensor->min_rect.top = 0;

	avt_info(sd, "get minimal and maximal resolutions");

	ret = regmap_read(sensor->regmap32,
					  sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_WIDTH_MIN_32R,
					  &sensor->min_rect.width);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_IMG_WIDTH_MIN_32R %u", sensor->min_rect.width);

	ret = regmap_read(sensor->regmap32,
					  sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_WIDTH_MAX_32R,
					  &sensor->max_rect.width);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_IMG_WIDTH_MAX_32R %u", sensor->max_rect.width);

	sensor->max_rect.left = sensor->max_rect.top = 0;

	ret = regmap_read(sensor->regmap32,
					  sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_HEIGHT_MIN_32R,
					  &sensor->min_rect.height);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_IMG_HEIGHT_MIN_32R %u", sensor->min_rect.height);

	ret = regmap_read(sensor->regmap32,
			  sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_HEIGHT_MAX_32R,
			  &sensor->max_rect.height);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)", ret);
		// goto err_out;
	}

	ret = device_property_read_u32(&sensor->i2c_client->dev,"avt,max-width",
				 &temp);

	if (ret == 0)
	{
		if (sensor->max_rect.width > temp)
			sensor->max_rect.width = temp;
	}

	ret = device_property_read_u32(&sensor->i2c_client->dev,"avt,max-height",
				 &temp);
	if (ret == 0)
	{
		if (sensor->max_rect.height > temp)
			sensor->max_rect.height = temp;
	}

	avt_dbg(sd, "BCRM_IMG_HEIGHT_MAX_32R %u", sensor->max_rect.height);

	ret = regmap_bulk_read(sensor->regmap64,
			       sensor->cci_reg.reg.bcrm_addr + BCRM_GAIN_MIN_64R,
			       &value64, 1);

	if (ret < 0)
	{
		dev_err(&client->dev, "regmap_read failed (%d)\n", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_GAIN_MIN_64R %llu", value64);

	ret = regmap_bulk_read(sensor->regmap64,
			       sensor->cci_reg.reg.bcrm_addr + BCRM_GAIN_MAX_64R,
			       &value64, 1);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)", ret);
		// goto err_out;
	}
	avt_dbg(sd, "BCRM_GAIN_MAX_64R %llu", value64);

	ret = bcrm_read32(sensor,BCRM_SENSOR_WIDTH_32R,
			  &sensor->sensor_rect.width);

	if (ret < 0)
		return ret;

	ret = bcrm_read32(sensor,BCRM_SENSOR_HEIGHT_32R,
			  &sensor->sensor_rect.height);

	if (ret < 0)
		return ret;

	sensor->sensor_rect.left = 0;
	sensor->sensor_rect.top = 0;

	sensor->curr_rect = sensor->max_rect;
	sensor->curr_rect.left = 0;
	sensor->curr_rect.top = 0;

	ret = regmap_write(sensor->regmap8,GENCP_CHANGEMODE_8W,bcm_mode);

	if (ret < 0)
	{
		avt_err(sd, "Failed to set BCM mode: i2c write failed (%d)\n", ret);
		return ret;
	}
	sensor->mode = AVT_BCRM_MODE;

	return 0;
}

static int avt_csi2_check_mipicfg(struct avt3_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = -EINVAL;
	int i;

	sensor->v4l2_fwnode_ep.bus_type = V4L2_MBUS_CSI2_DPHY;

	sensor->endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
	if (!sensor->endpoint)
	{
		dev_err(&client->dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(sensor->endpoint, &sensor->v4l2_fwnode_ep))
	{
		dev_err(&client->dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes > 4)
	{
		dev_err(&client->dev, "%s[%d]: more than 4 data lanes are currently not supported\n",
				__func__, __LINE__);
		goto error_out;
	}

	dev_info(&client->dev, "%s[%d]: ep_cfg.bus.mipi_csi2.num_data_lanes % d\n",
			 __func__, __LINE__, sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes);
	dev_info(&client->dev, "%s[%d]: v4l2_fwnode_ep.nr_of_link_frequencies %d",
			 __func__, __LINE__,
			 sensor->v4l2_fwnode_ep.nr_of_link_frequencies);

	for (i = 0; i < sensor->v4l2_fwnode_ep.nr_of_link_frequencies; i++)
		dev_info(&client->dev, "%s[%d]: v4l2_fwnode_ep.link-frequencies %u value %llu\n", __func__, __LINE__, i,
				 sensor->v4l2_fwnode_ep.link_frequencies[i]);

	/* Check the link frequency set in device tree */
	if (1 > sensor->v4l2_fwnode_ep.nr_of_link_frequencies)
	{
		dev_err(&client->dev, "%s[%d]: link-frequency property not found in DT\n", __func__, __LINE__);
		goto error_out;
	}

	ret = 0;
	return ret;

error_out:
	dev_err(&client->dev, "%s[%d]: sensor->v4l2_fwnode_ep invalid from now on!!", __func__, __LINE__);
	v4l2_fwnode_endpoint_free(&sensor->v4l2_fwnode_ep);
	fwnode_handle_put(sensor->endpoint);

	return ret;
}


static int avt3_query_binning(struct avt3_dev *camera)
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
			sizeof(struct avt3_binning_info),GFP_KERNEL);
	}

	memset(type_idx,0,sizeof(type_idx[0]) * AVT_BINNING_TYPE_CNT);
	for (i = 0;i < avt_binning_setting_cnt;i++) {
		const struct avt_binning_setting *setting = &avt_binning_settings[i];
		if (setting->inq == -1 || binning_inq & (1<<setting->inq)) {
			struct avt3_binning_info info = {0};

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


#ifdef BCRM_HS_THREAD
int avt3_streamon_thread(void *data)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)data;
	struct avt3_dev *sensor = to_avt3_dev(sd);
	int ret = 0;

	long jiffies = msecs_to_jiffies(5000);

	avt_info(sd, "+");

	do
	{
		ret = down_timeout(&sensor->streamon_sem, jiffies);

		if (0 == ret)
		{
			if (sensor->is_streaming && sensor->phyreset_on_streamon)
			{
				usleep_range(sensor->dphyreset_delay, sensor->dphyreset_delay * 2);
				avt3_dphy_reset(sensor, true);
				avt3_dphy_reset(sensor, false);
				avt_info(sd, "trigger alvium phy reset, sensor->dphyreset_delay %u ret %d",
						 sensor->dphyreset_delay, ret);
			}
		}
	} while (!kthread_should_stop());
	avt_info(sd, "-");
	return 0;
}

static int avt3_streamon_thread_enable(struct v4l2_subdev *sd)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	struct task_struct *task;

	dev_info(&client->dev, "%s[%d]+", __func__, __LINE__);

	task = kthread_create(avt3_streamon_thread, (void *)sd,
						  "%s", client->name);

	if (IS_ERR(task))
		return PTR_ERR(task);

	get_task_struct(task);
	wake_up_process(task);
	sensor->streamon_task = task;

	dev_info(&client->dev, "%s[%d]-", __func__, __LINE__);
	return 0;
}

static int avt3_streamon_thread_disable(struct v4l2_subdev *sd)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	dev_info(&client->dev, "%s[%d]+", __func__, __LINE__);

	if (sensor->streamon_task)
	{
		up(&sensor->streamon_sem);
		kthread_stop(sensor->streamon_task);
		put_task_struct(sensor->streamon_task);
		sensor->streamon_task = NULL;
	}
	dev_info(&client->dev, "%s[%d]-", __func__, __LINE__);
	return 0;
}
#endif

static const struct regmap_config alvium_reg8_config = {
	.reg_bits = AV_CAM_REG_SIZE * 8,
	.val_bits = AV_CAM_DATA_SIZE_8 * 8,
	.reg_stride = AV_CAM_DATA_SIZE_8,
	.max_register = ALVIUM_MAX_REG_ADDR,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.name = "alvium_regmap8",
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config alvium_reg16_config = {
	.reg_bits = AV_CAM_REG_SIZE * 8,
	.val_bits = AV_CAM_DATA_SIZE_16 * 8,
	.reg_stride = AV_CAM_DATA_SIZE_16,
	.max_register = ALVIUM_MAX_REG_ADDR,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.name = "alvium_regmap16",
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config alvium_reg32_config = {
	.reg_bits = AV_CAM_REG_SIZE * 8,
	.val_bits = AV_CAM_DATA_SIZE_32 * 8,
	.reg_stride = AV_CAM_DATA_SIZE_32,
	.max_register = ALVIUM_MAX_REG_ADDR,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.name = "alvium_regmap32",
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config alvium_reg64_config = {
	.reg_bits = AV_CAM_REG_SIZE * 8,
	.val_bits = AV_CAM_DATA_SIZE_64 * 8,
	.reg_stride = AV_CAM_DATA_SIZE_64,
	.max_register = ALVIUM_MAX_REG_ADDR,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.name = "alvium_regmap64",
	.cache_type = REGCACHE_NONE,
};

static int prepare_write_handshake(struct avt3_dev *camera)
{
	int ret;
	u8 handshake_val;
	u16 const handshake_addr =
		camera->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW;

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
	ret = regmap_write(camera->regmap8,handshake_addr,
			   handshake_val & ~BCRM_HANDSHAKE_STATUS_MASK);

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

static int wait_for_write_handshake(struct avt3_dev *camera)
{
	ulong ret;

	ret = wait_for_completion_timeout(&camera->bcrm_wrhs_completion,
					  msecs_to_jiffies(camera->bcrm_handshake_timeout_ms / 5));

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

static int bcrm_regmap_write64(struct avt3_dev *sensor,struct regmap *map,
			       unsigned int reg,unsigned long long val)
{
	int ret;


	ret = prepare_write_handshake(sensor);

	if (ret < 0)
		return ret;

	ret = regmap_bulk_write(map, reg, &val, 1);

	if (ret < 0)
	{
		dev_err(&sensor->i2c_client->dev,"%s[%d]: Writing value failed with: %d\n",__func__, __LINE__,ret);
		return ret;
	}

	if (!sensor->bcrm_write_handshake)
	{
		dev_info(&sensor->i2c_client->dev,
				 "%s[%d]: bcrm_write_handshake not supported. Use msleep(%u) at as fallback.",
				 __func__, __LINE__, sensor->bcrm_handshake_timeout_ms);
		/* Handshake not supported. Use static sleep at least once as fallback */
		msleep(sensor->bcrm_handshake_timeout_ms);
	}

	return wait_for_write_handshake(sensor);
}

static int bcrm_regmap_write(struct avt3_dev *sensor,
							 struct regmap *map,
							 unsigned int reg,
							 unsigned int val)
{

	int ret;

	ret = prepare_write_handshake(sensor);

	if (ret < 0)
		return ret;

	ret = regmap_write(map, reg, val);

	if (ret < 0)
	{
		dev_err(&sensor->i2c_client->dev,"%s[%d]: Writing value failed with: %d\n",__func__, __LINE__,ret);
		return ret;
	}

	msleep(10);

	if (!sensor->bcrm_write_handshake)
	{
		dev_info(&sensor->i2c_client->dev,
				 "%s[%d]: bcrm_write_handshake not supported. Use msleep(%u) at as fallback.",
				 __func__, __LINE__, sensor->bcrm_handshake_timeout_ms);
		/* Handshake not supported. Use static sleep at least once as fallback */
		msleep(sensor->bcrm_handshake_timeout_ms);

		return ret;
	}

	return wait_for_write_handshake(sensor);
}

static void bcrm_wrhs_work_func(struct work_struct *work)
{
	u32 handshake_val = 0;
	static const int poll_interval_ms = 5;
	int ret = 0;
	int i = 0;

	struct avt3_dev *sensor =
		container_of(work, struct avt3_dev, bcrm_wrhs_work);

	atomic_set(&sensor->bcrm_wrhs_enabled,1);

	do
	{
		//TODO: Must we check the return value here ?
		ret = regmap_read(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW, &handshake_val);

		if (handshake_val & BCRM_HANDSHAKE_STATUS_MASK)
		{
			//TODO: Must we check the return value here ?
			ret = regmap_write(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW, handshake_val & ~BCRM_HANDSHAKE_STATUS_MASK); /* reset only handshake status */

			complete(&sensor->bcrm_wrhs_completion);

			dev_dbg(&sensor->i2c_client->dev, "%s[%d]: Handshake ok\n",
				__func__, __LINE__);

			break;
		}
		msleep(poll_interval_ms);
		i++;
	} while (atomic_read(&sensor->bcrm_wrhs_enabled) != 0);

	if (i == 300)
		dev_info(&sensor->i2c_client->dev, "%s[%d]: 0x%08llx current->pid 0x%08x %d\n",
				 __func__, __LINE__, (u64)work, current->pid, i);
}


static int avt3_detect(struct i2c_client *client)
{
	const u16 address = 0x0;
	u32 value = 0;
	int ret;
	struct i2c_msg msgs[2] = {
			{
				.addr = client->addr,
				.flags = 0,
				.buf = (__u8*)&address,
				.len = 2,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.buf = (__u8*)&value,
				.len = 4,
			},
	};


	ret = i2c_transfer(client->adapter, msgs, 2);

	if (ret < 0)
	{
		return ret;
	}

	if (value == 0)
	{
		return -1;
	}

	return 0;
}


static int avt3_probe(struct i2c_client *client)
{

	struct device *dev = &client->dev;
	struct avt3_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	dev_info(&client->dev, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);

	if (avt3_detect(client) < 0)
	{
		dev_warn(&client->dev,"No camera detected!");
		return -ENODEV;
	}

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
	{
		return -ENOMEM;
	}

	sensor->i2c_client = client;

	sensor->streamon_delay = 0;

	ret = fwnode_property_read_u32(dev_fwnode(&client->dev),
								   "streamon_delay",
								   &sensor->streamon_delay);
	if (sensor->streamon_delay)
	{
		dev_info(dev, "%s[%d]: use sensor->streamon_delay of %u us\n", __func__, __LINE__, sensor->streamon_delay);
	}

	sensor->force_reset_on_init = fwnode_property_present(dev_fwnode(&client->dev), "force_reset_on_init");
	dev_dbg(dev, "%s[%d]: force_reset_on_init %d\n", __func__, __LINE__, sensor->force_reset_on_init);

	/* request optional power down pin */
	sensor->pwdn_gpio = devm_gpiod_get_optional(dev, "powerdown", sensor->force_reset_on_init ? GPIOD_OUT_HIGH : GPIOD_ASIS);

	if (NULL == sensor->pwdn_gpio || IS_ERR(sensor->pwdn_gpio))
	{
		dev_warn(&client->dev, "%s[%d]: no powerdown-gpios defined", __func__, __LINE__);
	}
	else
	{
		dev_info(dev, "%s[%d]: devm_gpiod_get_optional(dev, \"powerdown-gpios\" succeeded\n", __func__, __LINE__);
		gpiod_set_value_cansleep(sensor->pwdn_gpio, GPIOD_OUT_LOW);
	}

	/* request optional reset pin, only the first one will be used at the moment */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	// GPIOD_OUT_LOW);
	if (NULL == sensor->reset_gpio || IS_ERR(sensor->reset_gpio))
	{
		dev_warn(&client->dev, "%s[%d]: no reset-gpios defined", __func__, __LINE__);
	}
	else
	{
		dev_info(dev, "%s[%d]: devm_gpiod_get_optional(dev, \"reset-gpios\" succeeded\n", __func__, __LINE__);
		gpiod_set_value_cansleep(sensor->reset_gpio, GPIOD_OUT_LOW);
	}

	if (fwnode_property_present(dev_fwnode(&client->dev), "mipi_csi"))
		dev_info(dev, "%s[%d]: fwnode_property_present mipi_csi\n", __func__, __LINE__);
	else
		dev_info(dev, "%s[%d]: fwnode_property_present failed to find mipi_csi\n", __func__, __LINE__);

	sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_8_leg_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-yuv420_8_leg_avail");
	dev_info(dev, "%s[%d]: avt-ignore-yuv420_8_leg_avail   %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_8_leg_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_8_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-yuv420_8_avail");
	dev_info(dev, "%s[%d]: avt-ignore-yuv420_8_avail       %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_8_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_10_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-yuv420_10_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-yuv420_10_avail      %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_10_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_8_csps_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-yuv420_8_csps_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-yuv420_8_csps_avail  %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_8_csps_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_10_csps_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-yuv420_10_csps_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-yuv420_10_csps_avail %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.yuv420_10_csps_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_8_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-yuv422_8_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-yuv422_8_avail       %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_8_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_10_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-yuv422_10_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-yuv422_10_avail      %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_10_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.rgb888_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-rgb888_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-rgb888_avail         %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.rgb888_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.rgb666_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-rgb666_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-rgb666_avail         %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.rgb666_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.rgb565_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-rgb565_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-rgb565_avail         %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.rgb565_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.rgb555_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-rgb555_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-rgb555_avail         %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.rgb555_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.rgb444_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-rgb444_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-rgb444_avail         %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.rgb444_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.raw6_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-raw6_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-raw6_avail           %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.raw6_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.raw7_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-raw7_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-raw7_avail           %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.raw7_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.raw8_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-raw8_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-raw8_avail           %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.raw8_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.raw10_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-raw10_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-raw10_avail          %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.raw10_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-raw12_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-raw12_avail          %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail);

	sensor->ignore_avail_mipi_reg.avail_mipi.raw14_avail =
		fwnode_property_present(dev_fwnode(&client->dev), "avt-ignore-raw14_avail") ? 1 : 0;
	dev_info(dev, "%s[%d]: avt-ignore-raw14_avail          %d",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.avail_mipi.raw14_avail);

	dev_info(dev, "%s[%d]: sensor->ignore_avail_mipi_reg.avail_mipi 0x%08llx",
			 __func__, __LINE__, sensor->ignore_avail_mipi_reg.value);

	sensor->regmap8 = devm_regmap_init_i2c(client, &alvium_reg8_config);
	if (IS_ERR(sensor->regmap16))
	{
		dev_err(dev, "%s[%d]: regmap8 init failed: %ld\n", __func__, __LINE__,
				PTR_ERR(sensor->regmap8));
		ret = -ENODEV;
		goto err_exit;
	}

	sensor->regmap16 = devm_regmap_init_i2c(client, &alvium_reg16_config);
	if (IS_ERR(sensor->regmap16))
	{
		dev_err(dev, "%s[%d]: regmap16 init failed: %ld\n", __func__, __LINE__,
				PTR_ERR(sensor->regmap16));
		ret = -ENODEV;
		goto err_exit;
	}

	sensor->regmap32 = devm_regmap_init_i2c(client, &alvium_reg32_config);
	if (IS_ERR(sensor->regmap32))
	{
		dev_err(dev, "%s[%d]: regmap32 init failed: %ld\n", __func__, __LINE__,
				PTR_ERR(sensor->regmap32));
		ret = -ENODEV;
		goto err_exit;
	}

	sensor->regmap64 = devm_regmap_init_i2c(client, &alvium_reg64_config);
	if (IS_ERR(sensor->regmap64))
	{
		dev_err(dev, "%s[%d]: regmap64 init failed: %ld\n", __func__, __LINE__,
				PTR_ERR(sensor->regmap64));
		ret = -ENODEV;
		goto err_exit;
	}

	ret = fwnode_property_read_u32(dev_fwnode(&client->dev),
								   "bcrm_wait_timeout",
								   &sensor->bcrm_handshake_timeout_ms);

	if (ret)
	{
		dev_warn(dev, "%s[%d]: bcrm_wait_timeout not found, use default value\n", __func__, __LINE__);
		sensor->bcrm_handshake_timeout_ms = BCRM_WAIT_HANDSHAKE_TIMEOUT_MS;
	}
	dev_info(dev, "%s[%d]: bcrm_wait_timeout set to %dms\n", __func__, __LINE__, sensor->bcrm_handshake_timeout_ms);

	ret = avt_csi2_check_mipicfg(sensor);
	if (ret)
	{
		dev_err(dev, "%s[%d]: failed to parse endpoint\n", __func__, __LINE__);
		ret = -EINVAL;
		goto err_exit;
	}

	switch (sensor->v4l2_fwnode_ep.bus_type)
	{
	case V4L2_MBUS_CSI2_DPHY:
		dev_info(dev, "%s[%d]: endpoint->bus_type V4L2_MBUS_CSI2_DPHY", __func__, __LINE__);
		break;

	case V4L2_MBUS_UNKNOWN:
		dev_err(dev, "%s[%d]: endpoint->bus_type V4L2_MBUS_UNKNOWN", __func__, __LINE__);
		ret = -EINVAL;
		goto fwnode_cleanup;

	case V4L2_MBUS_PARALLEL:
		dev_err(dev, "%s[%d]: endpoint->bus_type V4L2_MBUS_PARALLEL", __func__, __LINE__);
		ret = -EINVAL;
		goto fwnode_cleanup;

	case V4L2_MBUS_BT656:
		dev_err(dev, "%s[%d]: endpoint->bus_type V4L2_MBUS_BT656", __func__, __LINE__);
		ret = -EINVAL;
		goto fwnode_cleanup;

	case V4L2_MBUS_CSI1:
		dev_err(dev, "%s[%d]: endpoint->bus_type V4L2_MBUS_CSI1", __func__, __LINE__);
		ret = -EINVAL;
		goto fwnode_cleanup;

	case V4L2_MBUS_CCP2:
		dev_err(dev, "%s[%d]: endpoint->bus_type V4L2_MBUS_CCP2", __func__, __LINE__);
		ret = -EINVAL;
		goto fwnode_cleanup;

	case V4L2_MBUS_CSI2_CPHY:
		dev_err(dev, "%s[%d]: endpoint->bus_type V4L2_MBUS_CSI2_CPHY", __func__, __LINE__);
		ret = -EINVAL;
		goto fwnode_cleanup;

	default:
		dev_err(dev, "%s[%d]: endpoint->bus_type unknown 0x%04X",
				__func__, __LINE__, sensor->v4l2_fwnode_ep.bus_type);
		ret = -EINVAL;
		goto fwnode_cleanup;
	}

	/* request optional power down pin */
	sensor->pwdn_gpio = devm_gpiod_get_optional(dev, "powerdown",
												GPIOD_OUT_HIGH);
	if (NULL == sensor->pwdn_gpio || IS_ERR(sensor->pwdn_gpio))
	{
		dev_warn(&client->dev, "%s[%d]: no powerdown-gpios powerdown defined",
				 __func__, __LINE__);
		sensor->pwdn_gpio = NULL;
	}
	else
	{
		dev_info(dev, "%s[%d]: devm_gpiod_get_optional(dev, \"powerdown-gpios\" succeeded\n", __func__, __LINE__);
		gpiod_set_value(sensor->pwdn_gpio, 0);
	}

	/* request optional reset pin, only the first one will be used at the moment */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
												 GPIOD_OUT_HIGH);
	if (NULL == sensor->reset_gpio || IS_ERR(sensor->reset_gpio))
	{
		dev_warn(&client->dev, "%s[%d]: no reset-gpios defined",
				 __func__, __LINE__);
		sensor->reset_gpio = NULL;
	}
	else
	{
		dev_info(dev, "%s[%d]: devm_gpiod_get_optional(dev, \"reset-gpios\" succeeded\n", __func__, __LINE__);
		gpiod_set_value(sensor->reset_gpio, 0);
	}

	/* now create the subdevice on i2c*/
	v4l2_i2c_subdev_init(&sensor->sd, client, &avt3_subdev_ops);
	sensor->sd.dev = &client->dev;
	sensor->sd.internal_ops = &avt3_subdev_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &avt3_sd_media_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret < 0)
		goto fwnode_cleanup;

	mutex_init(&sensor->lock);

	{
		enum avt_reset_type const reset_type = sensor->force_reset_on_init ? RESET_TYPE_HARD : RESET_TYPE_SOFT;
		if(reset_type == RESET_TYPE_HARD) {
			avt_info(&sensor->sd, "Hard reset requested by device tree");
		}

		ret = avt3_reset(sensor, reset_type);
		if(ret < 0) {
			avt_err(&sensor->sd, "Camera reset failed");
			goto fwnode_cleanup;
		}
	}

	ret = read_cci_registers(client);

	if (ret < 0)
	{
		dev_err(dev, "%s[%d]: read_cci_registers failed: %d\n",
				__func__, __LINE__, ret);
		goto entity_cleanup;
	}
	dev_info(dev, "%s[%d]: read_cci_registers succeeded\n", __func__, __LINE__);

	ret = cci_version_check(client);
	if (ret < 0)
	{
		dev_err(&client->dev, "%s[%d]: cci version mismatch - %d !\n",
				__func__, __LINE__, ret);
		goto entity_cleanup;
	}

	ret = bcrm_version_check(client);
	if (ret < 0)
	{
		dev_err(&client->dev, "%s[%d]: bcrm version mismatch - %d !\n",
				__func__, __LINE__, ret);
		goto entity_cleanup;
	}
	dev_info(dev, "%s[%d]: bcrm_version_check succeeded\n", __func__, __LINE__);

	sensor->bcrm_write_handshake =
		bcrm_get_write_handshake_availibility(client);


	dev_info(dev,"Found camera %s %s",sensor->cci_reg.reg.family_name,
		 sensor->cci_reg.reg.model_name);

	/* reading the Firmware Version register */
	ret = bcrm_read64(sensor,BCRM_DEVICE_FIRMWARE_VERSION_64R,
			  &sensor->cam_firmware_version.value);

	dev_info(&client->dev, "%s[%d]: Firmware version: %u.%u.%u.%x ret = %d\n",
			 __func__, __LINE__,
			 sensor->cam_firmware_version.device_firmware.special_version,
			 sensor->cam_firmware_version.device_firmware.major_version,
			 sensor->cam_firmware_version.device_firmware.minor_version,
			 sensor->cam_firmware_version.device_firmware.patch_version,
			 ret);

	if (sensor->cci_reg.reg.device_capabilities.caps.gencp)
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

		dev_info(&client->dev, "correct gcprm version\n");
	}

	init_completion(&sensor->bcrm_wrhs_completion);

	sensor->bcrm_wrhs_queue = create_singlethread_workqueue(sensor->sd.name);
	if (!sensor->bcrm_wrhs_queue)
	{
		dev_err(&client->dev, "%s[%d]: Could not create work queue\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto fwnode_cleanup;
	}

	dev_info(&client->dev, "%s[%d]: INIT_WORK(&sensor->bcrm_wrhs_work, bcrm_wrhs_work_func);\n", __func__, __LINE__);
	INIT_WORK(&sensor->bcrm_wrhs_work, bcrm_wrhs_work_func);
	atomic_set(&sensor->bcrm_wrhs_enabled,0);

	CLEAR(sensor->max_rect);
	CLEAR(sensor->min_rect);
	CLEAR(sensor->curr_rect);

	ret = avt3_get_sensor_capabilities(&sensor->sd);
	if (ret)
		goto entity_cleanup;

	ret = avt3_query_binning(sensor);
	if (ret)
		goto entity_cleanup;

	ret = avt3_get_fmt_available(client);

	ret = avt3_init_avail_formats(&sensor->sd);
	if (ret < 0)
	{
		dev_err(dev, "%s[%d]: avt3_init_avail_formats failed with %d\n",
				__func__, __LINE__, ret);
		goto entity_cleanup;
	}

	sensor->sd.ctrl_handler = &sensor->v4l2_ctrl_hdl;
	sensor->framerate_auto = true;
	sensor->gain = AVT3_DEFAULT_GAIN;
	sensor->exposure_time = AVT3_DEFAULT_EXPOSURETIME;
	sensor->exposure_mode = EMODE_MANUAL;

	fmt = &sensor->mbus_framefmt;
	fmt->code = sensor->available_fmts[0].mbus_code;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	fmt->width = sensor->max_rect.width;
	fmt->height = sensor->max_rect.height;
	fmt->field = V4L2_FIELD_NONE;

	sensor->streamcap.capability = V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = V4L2_MODE_HIGHQUALITY;

	// Init controls before registering the device, because the control handler must be fully initialized before
	// the subdevice is registered.
	ret = avt3_init_controls(sensor);
	if (ret)
	{
		dev_err(dev, "%s[%d]: avt3_init_controls failed with (%d)\n", __func__, __LINE__, ret);
		goto entity_cleanup;
	}

	ret = v4l2_async_register_subdev(&sensor->sd);

	if (ret)
	{
		dev_err(dev, "%s[%d]: v4l2_async_register_subdev_sensor_common failed with (%d)\n", __func__, __LINE__, ret);
		goto free_ctrls;
	}
	dev_info(&client->dev, "sensor %s registered\n", sensor->sd.name);

	ret = sysfs_create_group(&dev->kobj, &avt3_attr_grp);
	dev_info(dev, " -> %s[%d]: sysfs group created! (%d)\n", __func__, __LINE__, ret);
	if (ret)
	{
		dev_err(dev, "%s[%d]: Failed to create sysfs group (%d)\n", __func__, __LINE__, ret);
		goto free_ctrls;
	}



#ifdef DPHY_RESET_WORKAROUND
	sema_init(&sensor->streamon_sem, 0);
	avt3_streamon_thread_enable(&sensor->sd);
#endif

	ret = bcrm_regmap_write(sensor, sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_STREAM_ON_DELAY_32RW, sensor->streamon_delay);

	dev_info(&client->dev, "%s[%d]: probe success !\n", __func__, __LINE__);

	return 0;

free_ctrls:

	v4l2_ctrl_handler_free(&sensor->v4l2_ctrl_hdl);

entity_cleanup:
	media_entity_cleanup(&sensor->sd.entity);

fwnode_cleanup:
	if (sensor->bcrm_wrhs_queue)
		destroy_workqueue(sensor->bcrm_wrhs_queue);
	v4l2_fwnode_endpoint_free(&sensor->v4l2_fwnode_ep);
	fwnode_handle_put(sensor->endpoint);

err_exit:
	if (sensor->pwdn_gpio)
		devm_gpiod_put(dev, sensor->pwdn_gpio);
	if (sensor->reset_gpio)
		devm_gpiod_put(dev, sensor->reset_gpio);

	mutex_destroy(&sensor->lock);
	return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
static int avt3_remove(struct i2c_client *client)
#else
static void avt3_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct avt3_dev *sensor = to_avt3_dev(sd);

	avt_dbg(sd, "+");

	v4l2_fwnode_endpoint_free(&sensor->v4l2_fwnode_ep);
	fwnode_handle_put(sensor->endpoint);

	sysfs_remove_group(&client->dev.kobj, &avt3_attr_grp);
	media_entity_cleanup(&sensor->sd.entity);

	v4l2_ctrl_handler_free(&sensor->v4l2_ctrl_hdl);

#ifdef DPHY_RESET_WORKAROUND
	avt3_streamon_thread_disable(sd);
#endif

	if (sensor->bcrm_wrhs_queue)
		destroy_workqueue(sensor->bcrm_wrhs_queue);

	if (sensor->pwdn_gpio)
		devm_gpiod_put(&client->dev, sensor->pwdn_gpio);

	if (sensor->reset_gpio)
		devm_gpiod_put(&client->dev, sensor->reset_gpio);

	mutex_destroy(&sensor->lock);

	v4l2_async_unregister_subdev(&sensor->sd);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0))
	return 0;
#endif
}

static const struct i2c_device_id avt3_id[] = {
	{AVT3_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, avt3_id);

static const struct of_device_id avt3_dt_ids[] = {
	{.compatible = "alliedvision,avt3"},
	{}};
MODULE_DEVICE_TABLE(of, avt3_dt_ids);

static struct i2c_driver avt3_i2c_driver = {
	.driver = {
		.name = AVT3_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = avt3_dt_ids,
	},
	.id_table = avt3_id,
	.probe_new = avt3_probe,
	.remove = avt3_remove,
};

module_i2c_driver(avt3_i2c_driver);

MODULE_DESCRIPTION("Allied Vision's MIPI-CSI2 Camera Driver");
MODULE_AUTHOR("Allied Vision Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");

#ifdef DPHY_RESET_WORKAROUND
if (sensor->phyreset_on_streamon)
{
	up(&sensor->streamon_sem);
	dev_info(&client->dev, "%s[%d]: up(&sensor->streamon_sem) returnd", __func__, __LINE__);
}
#endif

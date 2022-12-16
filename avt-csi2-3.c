// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 Avnet EMG GmbH 
 * Copyright (C) 2022 Allied Vision Technologies
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
#include <linux/lcm.h>
#include <linux/crc32.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>

// only for dma_get_cache_alignment();
#include <linux/dma-mapping.h>

#include "avt-csi2.h"
#include <uapi/linux/libcsi_ioctl.h>

static int debug = 0;
// module_param(debug, int, 0600);/* S_IRUGO */
module_param(debug, int, 0644); /* S_IRUGO */
MODULE_PARM_DESC(debug, "Debug level (0-2)");

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

struct avt3_mode_info
{
	enum avt3_mode_id id;
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
	//	const struct reg_value *reg_data;
	//	u32 reg_data_size;
};
//
//	[0]: 'RGBP' (16-bit RGB 5-6-5)
//	[1]: 'RGB3' (24-bit RGB 8-8-8)
//	[2]: 'BGR3' (24-bit BGR 8-8-8)
//	[3]: 'YUYV' (YUYV 4:2:2)
//	[4]: 'YUV4' (32-bit A/XYUV 8-8-8-8)
//	[5]: 'NV12' (Y/CbCr 4:2:0)
//	[6]: 'YM24' (Planar YUV 4:4:4 (N-C))
//	[7]: 'XR24' (32-bit BGRX 8-8-8-8)
//	[8]: 'AR24' (32-bit BGRA 8-8-8-8)
//
/* that table is for testing only */
static const struct avt3_mode_info avt3_mode_data[AVT3_NUM_MODES] = {
	{AVT3_MODE_FF_32_16, 32, 32, 16, 16},
	//	{AVT3_MODE_FF_32_32, 32, 32, 32, 32},
	{AVT3_MODE_FF_64_32, 64, 64, 32, 32},
	//	{AVT3_MODE_FF_64_64, 64, 64, 64, 64},
	//	{AVT3_MODE_FF_96_32, 96, 96, 32, 32},
	{AVT3_MODE_FF_128_32, 128, 128, 32, 32},
	{AVT3_MODE_FF_128_64, 128, 128, 64, 64},
	{AVT3_MODE_FF_176_64, 176, 176, 64, 64},
	{AVT3_MODE_QCIF_176_144, 176, 176, 144, 144},
	{AVT3_MODE_QVGA_320_240, 320, 320, 240, 240},
	{AVT3_MODE_VGA_640_480, 640, 640, 480, 480},
	{AVT3_MODE_NTSC_720_480, 720, 720, 480, 480},
	{AVT3_MODE_PAL_720_576, 720, 720, 576, 576},
	//	{AVT3_MODE_XGA_1024_768, 1024, 1024, 768, 576},
	{AVT3_MODE_720P_1280_720, 1280, 1892, 720, 720},
	{AVT3_MODE_1080P_1920_1080, 1920, 2500, 1080, 1080},
	{AVT3_MODE_QSXGA_2592_1944, 2592, 2592, 1944, 1944},
	{AVT3_MODE_UHD_3840_2160, 3840, 3840, 2160, 2160},
	{AVT3_MODE_UHD2_4096_3672, 4096, 4096, 3672, 3672},
	{AVT3_MODE_5376_3672, 5376, 5376, 3672, 3672},
};

/* that table is for testing only */

static const int avt3_framerates[] = {
    [AVT3_05_FPS] = 5,
	[AVT3_08_FPS] = 8,
	[AVT3_10_FPS] = 10,
	[AVT3_15_FPS] = 15,
	[AVT3_20_FPS] = 20,
	[AVT3_24_FPS] = 24,
	[AVT3_25_FPS] = 25,
	[AVT3_30_FPS] = 30,
	[AVT3_50_FPS] = 50,
	[AVT3_60_FPS] = 60,
	[AVT3_75_FPS] = 75,
	[AVT3_90_FPS] = 90,
	[AVT3_100_FPS] = 100,
	[AVT3_120_FPS] = 120,
	[AVT3_150_FPS] = 150,
	[AVT3_200_FPS] = 200,
	[AVT3_250_FPS] = 250,
	[AVT3_300_FPS] = 300,
	[AVT3_500_FPS] = 500,
	[AVT3_750_FPS] = 750,
	[AVT3_1000_FPS] = 1000,
	[AVT3_1200_FPS] = 1200,
	[AVT3_1400_FPS] = 1400,
	[AVT3_1500_FPS] = 1500,
	[AVT3_1600_FPS] = 1600,
	[AVT3_1650_FPS] = 1650,
	[AVT3_1750_FPS] = 1750,
	[AVT3_1800_FPS] = 1800,
	[AVT3_2000_FPS] = 2000,
	[AVT3_NUM_FRAMERATES] = -1, // dummy for framerate set by set_parm
};

static int bcrm_regmap_write64(struct avt3_dev *sensor,
							   struct regmap *map,
							   unsigned int reg,
							   unsigned long long val);

static int bcrm_regmap_write(struct avt3_dev *sensor,
							 struct regmap *map,
							 unsigned int reg,
							 unsigned int val);

static void avt3_soft_reset(struct avt3_dev *sensor);
static void avt3_hard_reset(struct avt3_dev *sensor);
static void avt3_dphy_reset(struct avt3_dev *sensor, bool bResetPhy);

static int avt3_ctrl_send(struct i2c_client *client,
						  struct avt_ctrl *vc);
static inline struct avt3_dev *to_avt3_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct avt3_dev, sd);
}

static struct avt3_dev *client_to_avt3_dev(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct avt3_dev, sd);
}

static uint32_t i2c_read(struct i2c_client *client, uint32_t reg,
						 uint32_t size, uint32_t count, char *buf)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	int ret = 0;

#ifdef I2C_READ_COMPATIBLE_MODE
	/* read byte per byte regardless the endianess
	 * that means ignore parameter size and read count
	 * number of bytes */

	ret = regmap_bulk_read(sensor->regmap8, reg, buf, count);
#else
	/* Read count pakets of size bytes per packet and correct endianess
	 * should work independently on the endianess of the host cross_update.
	 * Take care on the absolute number of bytes what is size * count.  */

	switch (size)
	{
	case AV_CAM_DATA_SIZE_8:
		ret = regmap_bulk_read(sensor->regmap8, reg, buf, count);
		break;
	case AV_CAM_DATA_SIZE_16:
		ret = regmap_bulk_read(sensor->regmap16, reg, buf, count);
		break;
	case AV_CAM_DATA_SIZE_32:
		ret = regmap_bulk_read(sensor->regmap32, reg, buf, count);
		break;
	case AV_CAM_DATA_SIZE_64:
		ret = regmap_bulk_read(sensor->regmap64, reg, buf, count);
		break;
	default:
		avt_err(&sensor->sd, "wrong value of parameter size %d", size);
		ret = -EINVAL;
	}
#endif
	return ret;
}

// static int avt3_set_mipi_clock(struct v4l2_subdev *sd);

#define DUMP_BCRM_REG8(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_8)
#define DUMP_BCRM_REG32(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_32)
#define DUMP_BCRM_REG64(CLIENT, BCRM_REG) dump_bcrm_reg(CLIENT, (BCRM_REG), (#BCRM_REG), AV_CAM_DATA_SIZE_64)

static void dump_bcrm_reg(struct i2c_client *client, u16 nOffset, const char *pRegName, int regsize);

// static void bcrm_dump(struct i2c_client *client, char *szdump2buffer, int max_len)
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

	regmap_bulk_read(sensor->regmap16,
					 sensor->cci_reg.reg.gcprm_address + 0x0000,
					 (char *)&sensor->gencp_reg,
					 sizeof(sensor->gencp_reg) / AV_CAM_REG_SIZE);

	crc = crc32(U32_MAX, &sensor->gencp_reg, crc_byte_count);

	if (ret < 0)
	{
		avt_err(&sensor->sd, "regmap_read failed, ret %d", ret);
		goto err_out;
	}

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

static int avt3_set_bcrm(struct i2c_client *client)
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

	if (AVT_BCRM_MODE == current_mode && current_mode != sensor->mode)
		avt_dbg(&sensor->sd, "Sensor is already in BCRM mode but driver has inconsitent mode value.");

	if (AVT_BCRM_MODE == current_mode)
	{
		avt_warn(&sensor->sd, "Sensor is already in BCRM mode. Ignore request to set BCRM mode.");
		sensor->mode = AVT_BCRM_MODE;
		return ret;
	}

	mode = AVT_BCRM_MODE;
	ret = bcrm_regmap_write(sensor, sensor->regmap8,
							GENCP_CHANGEMODE_8W, mode);

	if (ret < 0)
	{
		avt_err(&sensor->sd, "Failed to set BCRM mode: i2c write failed (%d)", ret);
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
	ret = regmap_read(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_VERSION_32R, &value);

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
	// ret = sprintf(buf, "%d\n", sensor->is_streaming ? 0 : 1);
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

		/* Set number of lanes */
		//		ret = bcrm_regmap_write(sensor, sensor->regmap8,
		//				sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_LANE_COUNT_8RW,
		//				sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes);

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

	//	MUTEX_LOCK(&sensor->lock);

	ret = sprintf(buf, "%d\n", sensor->pending_softreset_request);

	//	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static ssize_t softreset_store(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret;

	//	MUTEX_LOCK(&sensor->lock);
	ret = kstrtoint(buf, 10, &sensor->pending_softreset_request);
	if (ret < 0)
	{
		MUTEX_UNLOCK(&sensor->lock);
		return ret;
	}

	if (sensor->pending_softreset_request > 0)
		avt3_soft_reset(sensor);
	//	MUTEX_UNLOCK(&sensor->lock);
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

	if (!sensor->is_streaming /*||
		!sensor->avt_trigger_status.trigger_mode_enabled ||
		sensor->avt_trigger_status.trigger_source != V4L2_TRIGGER_SOURCE_SOFTWARE */
	)
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

	// MUTEX_LOCK(&sensor->lock);
	ret = kstrtoint(buf, 10, &sensor->pending_hardtreset_request);
	if (ret < 0)
	{
		// MUTEX_UNLOCK(&sensor->lock);
		return ret;
	}

	if (sensor->pending_hardtreset_request > 0)
		avt3_hard_reset(sensor);

	sensor->pending_hardtreset_request = 0;
	// MUTEX_UNLOCK(&sensor->lock);
	return count;
}

static ssize_t bcrm_dump_show(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	// struct avt3_dev *sensor = client_to_avt3_dev(to_i2c_client(dev));
	ssize_t ret = 0;

	bcrm_dump(to_i2c_client(dev));

	return ret;
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
					  u32 fourcc, /* v4l2 format id */
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

	// sensor->available_fmts = kmalloc(sizeof(avt_mbus_formats[0]) * ARRAY_SIZE(avt_mbus_formats), GFP_KERNEL);
	sensor->available_fmts = kmalloc(sizeof(sensor->available_fmts[0]) * AVT3_MAX_FORMAT_ENTRIES, GFP_KERNEL);

	if (!sensor->available_fmts)
	{
		dev_err(&client->dev,
				"%s[%d]: not enough memory to store list of available formats",
				__func__, __LINE__);
		return -ENOMEM;
	}

	/* imx8mp csi-sam:
			.code = MEDIA_BUS_FMT_YUYV8_2X8, IPI_CSIS_ISPCFG_FMT_YCBCR422_8BIT,
			.code = MEDIA_BUS_FMT_RGB888_1X24, IPI_CSIS_ISPCFG_FMT_RGB888,
			.code = MEDIA_BUS_FMT_UYVY8_2X8, IPI_CSIS_ISPCFG_FMT_YCBCR422_8BIT,
			.code = MEDIA_BUS_FMT_VYUY8_2X8, IPI_CSIS_ISPCFG_FMT_YCBCR422_8BIT,
			.code = MEDIA_BUS_FMT_SBGGR8_1X8, IPI_CSIS_ISPCFG_FMT_RAW8,
			
			.code = MEDIA_BUS_FMT_SBGGR10_1X10, IPI_CSIS_ISPCFG_FMT_RAW10,
			.code = MEDIA_BUS_FMT_SGBRG10_1X10, IPI_CSIS_ISPCFG_FMT_RAW10,
			.code = MEDIA_BUS_FMT_SGRBG10_1X10, IPI_CSIS_ISPCFG_FMT_RAW10,
			.code = MEDIA_BUS_FMT_SRGGB10_1X10, IPI_CSIS_ISPCFG_FMT_RAW10,
			.code = MEDIA_BUS_FMT_SBGGR12_1X12, IPI_CSIS_ISPCFG_FMT_RAW12,
			.code = MEDIA_BUS_FMT_SGBRG12_1X12, IPI_CSIS_ISPCFG_FMT_RAW12,
			.code = MEDIA_BUS_FMT_SGRBG12_1X12, IPI_CSIS_ISPCFG_FMT_RAW12,
			.code = MEDIA_BUS_FMT_SRGGB12_1X12, IPI_CSIS_ISPCFG_FMT_RAW12,

	*/
	pfmt = sensor->available_fmts;
	if (sensor->avail_mipi_reg.avail_mipi.yuv422_8_avail && !sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_8_avail)
	{

		adev_info(&client->dev, "add MEDIA_BUS_FMT_UYVY8_2X8/V4L2_PIX_FMT_UYVY/MIPI_CSI2_DT_YUV422_8B to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.yuv422_8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_8_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_UYVY8_2X8, MIPI_CSI2_DT_YUV422_8B,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_UYVY, bayer_ignore, "MEDIA_BUS_FMT_UYVY8_2X8");
		sensor->available_fmts_cnt++;
		pfmt++;

		adev_info(&client->dev, "add MEDIA_BUS_FMT_UYVY8_1X16/V4L2_PIX_FMT_UYVY/MIPI_CSI2_DT_YUV422_8B to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.yuv422_8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_8_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_UYVY8_1X16, MIPI_CSI2_DT_YUV422_8B,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_UYVY, bayer_ignore, "MEDIA_BUS_FMT_UYVY8_1X16");
		sensor->available_fmts_cnt++;
		pfmt++;

		adev_info(&client->dev, "add MEDIA_BUS_FMT_YUYV8_1X16/V4L2_PIX_FMT_YUV422P/MIPI_CSI2_DT_YUV422_8B to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.yuv422_8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_8_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_YUYV8_1X16, MIPI_CSI2_DT_YUV422_8B,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_YUV422P, bayer_ignore, "MEDIA_BUS_FMT_YUYV8_1X16");
		sensor->available_fmts_cnt++;
		pfmt++;

		// NXP MEDIA_BUS_FMT_YUYV8_2X8, MEDIA_BUS_FMT_UYVY8_2X8, MEDIA_BUS_FMT_VYUY8_2X8
		adev_info(&client->dev, "add MEDIA_BUS_FMT_YUYV8_2X8/V4L2_PIX_FMT_YUYV/MIPI_CSI2_DT_YUV422_8B to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.yuv422_8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_8_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_YUYV8_2X8, MIPI_CSI2_DT_YUV422_8B,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_YUYV, bayer_ignore, "MEDIA_BUS_FMT_YUYV8_2X8");
		sensor->available_fmts_cnt++;
		pfmt++;

		adev_info(&client->dev, "add MEDIA_BUS_FMT_VYUY8_2X8/V4L2_PIX_FMT_YUYV/MIPI_CSI2_DT_YUV422_8B to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.yuv422_8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_8_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_VYUY8_2X8, MIPI_CSI2_DT_YUV422_8B,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_VYUY, bayer_ignore, "MEDIA_BUS_FMT_VYUY8_2X8");

		sensor->available_fmts_cnt++;
		pfmt++;
	}
	if (sensor->avail_mipi_reg.avail_mipi.yuv422_10_avail && !sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_10_avail)
	{

		adev_info(&client->dev, "add MEDIA_BUS_FMT_YUYV10_1X20/V4L2_PIX_FMT_YUV410/MIPI_CSI2_DT_YUV422_8B to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.yuv422_10_avail, sensor->ignore_avail_mipi_reg.avail_mipi.yuv422_10_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_YUYV10_1X20, MIPI_CSI2_DT_YUV422_8B,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_YUV410, bayer_ignore, "MEDIA_BUS_FMT_YUYV10_1X20/V4L2_PIX_FMT_YUV410/MIPI_CSI2_DT_YUV422_8B");
		sensor->available_fmts_cnt++;
		pfmt++;
	}

	if (sensor->avail_mipi_reg.avail_mipi.rgb888_avail && !sensor->ignore_avail_mipi_reg.avail_mipi.rgb888_avail)
	{

		adev_info(&client->dev, "add MEDIA_BUS_FMT_RGB888_1X24/V4L2_PIX_FMT_RGB24/MIPI_CSI2_DT_RGB888 to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.rgb888_avail, sensor->ignore_avail_mipi_reg.avail_mipi.rgb888_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_RGB888_1X24, MIPI_CSI2_DT_RGB888,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_RGB24, bayer_ignore, "MEDIA_BUS_FMT_RGB888_1X24");
		sensor->available_fmts_cnt++;
		pfmt++;

		adev_info(&client->dev, "add MEDIA_BUS_FMT_RBG888_1X24/V4L2_PIX_FMT_RGB24/MIPI_CSI2_DT_RGB888 to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.rgb888_avail, sensor->ignore_avail_mipi_reg.avail_mipi.rgb888_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_RBG888_1X24, MIPI_CSI2_DT_RGB888,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_RGB24, bayer_ignore, "MEDIA_BUS_FMT_RBG888_1X24");
		sensor->available_fmts_cnt++;
		pfmt++;

		adev_info(&client->dev, "add MEDIA_BUS_FMT_BGR888_1X24/V4L2_PIX_FMT_RGB24/MIPI_CSI2_DT_RGB888 to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.rgb888_avail, sensor->ignore_avail_mipi_reg.avail_mipi.rgb888_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_BGR888_1X24, MIPI_CSI2_DT_RGB888,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_RGB24, bayer_ignore, "MEDIA_BUS_FMT_BGR888_1X24");
		sensor->available_fmts_cnt++;
		pfmt++;

		adev_info(&client->dev, "add MEDIA_BUS_FMT_RGB888_3X8/V4L2_PIX_FMT_RGB24/MIPI_CSI2_DT_RGB888 to list of available formats bayer %d - %d:%d", 
				bayer_ignore,
				  sensor->avail_mipi_reg.avail_mipi.rgb888_avail, sensor->ignore_avail_mipi_reg.avail_mipi.rgb888_avail);
		set_mode_mapping(pfmt, MEDIA_BUS_FMT_RGB888_3X8, MIPI_CSI2_DT_RGB888,
						 V4L2_COLORSPACE_SRGB, V4L2_PIX_FMT_RGB24, bayer_ignore, "MEDIA_BUS_FMT_RGB888_3X8");
		sensor->available_fmts_cnt++;
		pfmt++;
	}

	if (sensor->avail_mipi_reg.avail_mipi.raw8_avail && !sensor->ignore_avail_mipi_reg.avail_mipi.raw8_avail)
	{

		if (sensor->bayer_inquiry_reg.bayer_pattern.monochrome_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_Y8_1X8/V4L2_PIX_FMT_GREY/MIPI_CSI2_DT_RAW8 to list of available formats bayer %d - %d:%d", 
				monochrome,
					  sensor->avail_mipi_reg.avail_mipi.raw8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw8_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_Y8_1X8, MIPI_CSI2_DT_RAW8,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_GREY, monochrome, "MEDIA_BUS_FMT_Y8_1X8");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SGRBG8_1X8/V4L2_PIX_FMT_SGRBG8/MIPI_CSI2_DT_RAW8 to list of available formats bayer %d - %d:%d", 
				bayer_bg,
					  sensor->avail_mipi_reg.avail_mipi.raw8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw8_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SGRBG8_1X8, MIPI_CSI2_DT_RAW8,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SGRBG8, bayer_gr, "MEDIA_BUS_FMT_SGRBG8_1X8");
			sensor->available_fmts_cnt++;
			pfmt++;
		}
		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SRGGB8_1X8/V4L2_PIX_FMT_SRGGB8/MIPI_CSI2_DT_RAW8 to list of available formats bayer %d - %d:%d", 
				bayer_gr,
					  sensor->avail_mipi_reg.avail_mipi.raw8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw8_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SRGGB8_1X8, MIPI_CSI2_DT_RAW8,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SRGGB8, bayer_rg, "MEDIA_BUS_FMT_SRGGB8_1X8");
			sensor->available_fmts_cnt++;
			pfmt++;
		}
		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SBGGR8_1X8/V4L2_PIX_FMT_SBGGR8/MIPI_CSI2_DT_RAW8 to list of available formats bayer %d - %d:%d", 
				bayer_gr,
					  sensor->avail_mipi_reg.avail_mipi.raw8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw8_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SBGGR8_1X8, MIPI_CSI2_DT_RAW8,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SBGGR8, bayer_bg, "MEDIA_BUS_FMT_SBGGR8_1X8");
			sensor->available_fmts_cnt++;
			pfmt++;
		}
		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SGBRG8_1X8/V4L2_PIX_FMT_SBGGR8/MIPI_CSI2_DT_RAW8 to list of available formats bayer %d - %d:%d", 
				bayer_rg,
					  sensor->avail_mipi_reg.avail_mipi.raw8_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw8_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SGBRG8_1X8, MIPI_CSI2_DT_RAW8,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SGBRG8, bayer_rg, "MEDIA_BUS_FMT_SGBRG8_1X8");
			sensor->available_fmts_cnt++;
			pfmt++;
		}
		//'XR24' (32-bit BGRX 8-8-8-8)
	}

	if (sensor->avail_mipi_reg.avail_mipi.raw10_avail && !sensor->ignore_avail_mipi_reg.avail_mipi.raw10_avail)
	{
		if (sensor->bayer_inquiry_reg.bayer_pattern.monochrome_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_Y10_1X10/V4L2_PIX_FMT_Y10/MIPI_CSI2_DT_RAW10 to list of available formats bayer %d - %d:%d", 
				monochrome,
					  sensor->avail_mipi_reg.avail_mipi.raw10_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw10_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_Y10_1X10, MIPI_CSI2_DT_RAW10,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_Y10, monochrome, "MEDIA_BUS_FMT_Y10_1X10");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SGRBG10_1X10/V4L2_PIX_FMT_SRGGB10/MIPI_CSI2_DT_RAW10 to list of available formats bayer %d - %d:%d", 
				bayer_bg,
					  sensor->avail_mipi_reg.avail_mipi.raw10_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw10_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SGRBG10_1X10, MIPI_CSI2_DT_RAW10,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SGRBG10, bayer_gr, "MEDIA_BUS_FMT_SGRBG10_1X10");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SRGGB10_1X10/V4L2_PIX_FMT_SRGGB10/MIPI_CSI2_DT_RAW10 to list of available formats bayer %d - %d:%d", 
				bayer_gb,
					  sensor->avail_mipi_reg.avail_mipi.raw10_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw10_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SRGGB10_1X10, MIPI_CSI2_DT_RAW10,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SRGGB10, bayer_rg, "MEDIA_BUS_FMT_SRGGB10_1X10");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SBGGR10_1X10/V4L2_PIX_FMT_SGRBG10/MIPI_CSI2_DT_RAW10 to list of available formats bayer %d - %d:%d", 
				bayer_gr,
					  sensor->avail_mipi_reg.avail_mipi.raw10_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw10_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SBGGR10_1X10, MIPI_CSI2_DT_RAW10,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SGRBG10, bayer_bg, "MEDIA_BUS_FMT_SBGGR10_1X10");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail)
		{

			adev_info(&client->dev, "add MEDIA_BUS_FMT_SGBRG10_1X10/V4L2_PIX_FMT_SGBRG10/MIPI_CSI2_DT_RAW10 to list of available formats bayer %d - %d:%d", 
				bayer_rg,
					  sensor->avail_mipi_reg.avail_mipi.raw10_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw10_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SGBRG10_1X10, MIPI_CSI2_DT_RAW10,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SGBRG10, bayer_gb, "MEDIA_BUS_FMT_SGBRG10_1X10");
			sensor->available_fmts_cnt++;
			pfmt++;
		}
	}

	if (sensor->avail_mipi_reg.avail_mipi.raw12_avail && !sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail)
	{
		if (sensor->bayer_inquiry_reg.bayer_pattern.monochrome_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_Y12_1X12/V4L2_PIX_FMT_SRGGB10/MIPI_CSI2_DT_RAW12 to list of available formats bayer %d - %d:%d", 
				monochrome,
					  sensor->avail_mipi_reg.avail_mipi.raw12_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_Y12_1X12, MIPI_CSI2_DT_RAW12,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_Y12, monochrome, "MEDIA_BUS_FMT_Y12_1X12");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SGRBG12_1X12/V4L2_PIX_FMT_SGRBG12/MIPI_CSI2_DT_RAW12 to list of available formats bayer %d - %d:%d", 
				bayer_bg,
					  sensor->avail_mipi_reg.avail_mipi.raw12_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SGRBG12_1X12, MIPI_CSI2_DT_RAW12,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SRGGB12, bayer_gr, "MEDIA_BUS_FMT_SGRBG12_1X12");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SRGGB12_1X12/V4L2_PIX_FMT_SGRBG12/MIPI_CSI2_DT_RAW12 to list of available formats bayer %d - %d:%d", 
				bayer_gb,
					  sensor->avail_mipi_reg.avail_mipi.raw12_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SRGGB12_1X12, MIPI_CSI2_DT_RAW12,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SRGGB12, bayer_rg, "MEDIA_BUS_FMT_SRGGB12_1X12");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SBGGR12_1X12/V4L2_PIX_FMT_SRGGB10/MIPI_CSI2_DT_RAW12 to list of available formats bayer %d - %d:%d", 
				bayer_gr,
					  sensor->avail_mipi_reg.avail_mipi.raw12_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SBGGR12_1X12, MIPI_CSI2_DT_RAW12,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SBGGR12, bayer_bg, "MEDIA_BUS_FMT_SBGGR12_1X12");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SGBRG12_1X12/V4L2_PIX_FMT_SGRBG12/MIPI_CSI2_DT_RAW12 to list of available formats bayer %d - %d:%d", 
				bayer_rg,
					  sensor->avail_mipi_reg.avail_mipi.raw12_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SGBRG12_1X12, MIPI_CSI2_DT_RAW12,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SBGGR12, bayer_gb, "MEDIA_BUS_FMT_SGBRG12_1X12");
			sensor->available_fmts_cnt++;
			pfmt++;
		}
	}

	if (sensor->avail_mipi_reg.avail_mipi.raw14_avail && !sensor->ignore_avail_mipi_reg.avail_mipi.raw14_avail)
	{
		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_GR_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SBGGR14_1X14/V4L2_PIX_FMT_SBGGR14/MIPI_CSI2_DT_RAW14 to list of available formats bayer %d - %d:%d", 
				bayer_gr,
					  sensor->avail_mipi_reg.avail_mipi.raw12_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw12_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SBGGR14_1X14, MIPI_CSI2_DT_RAW14,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SBGGR14, bayer_gr, "MEDIA_BUS_FMT_SBGGR14_1X14");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_RG_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SGBRG14_1X14/V4L2_PIX_FMT_SGRBG14/MIPI_CSI2_DT_RAW14 to list of available formats bayer %d - %d:%d", 
				bayer_rg,
					  sensor->avail_mipi_reg.avail_mipi.raw14_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw14_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SGBRG14_1X14, MIPI_CSI2_DT_RAW14,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SGBRG14, bayer_rg, "MEDIA_BUS_FMT_SGBRG14_1X14");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_BG_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SGRBG14_1X14/V4L2_PIX_FMT_SGRBG14/MIPI_CSI2_DT_RAW14 to list of available formats bayer %d - %d:%d", 
				bayer_bg,
					  sensor->avail_mipi_reg.avail_mipi.raw14_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw14_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SGRBG14_1X14, MIPI_CSI2_DT_RAW14,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SRGGB14, bayer_bg, "MEDIA_BUS_FMT_SGRBG14_1X14");
			sensor->available_fmts_cnt++;
			pfmt++;
		}

		if (sensor->bayer_inquiry_reg.bayer_pattern.bayer_GB_avail)
		{
			adev_info(&client->dev, "add MEDIA_BUS_FMT_SRGGB14_1X14/V4L2_PIX_FMT_SRGGB14/MIPI_CSI2_DT_RAW14 to list of available formats bayer %d - %d:%d", 
				bayer_gb,
					  sensor->avail_mipi_reg.avail_mipi.raw14_avail, sensor->ignore_avail_mipi_reg.avail_mipi.raw14_avail);
			set_mode_mapping(pfmt, MEDIA_BUS_FMT_SRGGB14_1X14, MIPI_CSI2_DT_RAW14,
						 V4L2_COLORSPACE_RAW, V4L2_PIX_FMT_SRGGB14, bayer_gb, "MEDIA_BUS_FMT_SRGGB14_1X14");
			sensor->available_fmts_cnt++;
			pfmt++;
		}
	}

	pfmt->mbus_code = -EINVAL;

	avt_dbg(&sensor->sd, "available_fmts_cnt %d", sensor->available_fmts_cnt);

	return sensor->available_fmts_cnt;
}

static const struct avt3_mode_info *
avt3_find_mode(struct avt3_dev *sensor, enum avt3_frame_rate fr,
			   int width, int height, bool nearest)
{
	const struct avt3_mode_info *mode;
	// int	i;

	avt_dbg(&sensor->sd, "width %d, height %d, framerate[%d] %d [ToDo: replace that code]",
			width, height, fr, avt3_framerates[fr]);
// ToDo: instead of using the table it should be checked if it fits to the camera capabilities
// than variable mode and the avt3_mode_data can be removed

	mode = v4l2_find_nearest_size(avt3_mode_data,
								  ARRAY_SIZE(avt3_mode_data),
								  hact, vact,
								  width, height);

	if (!mode)
		return NULL;

	return mode;
}

/* hard reset depends on gpio-pins, needs to be completed on
   suitable board instead of imx8mp-evk */
static void avt3_hard_reset(struct avt3_dev *sensor)
{
	dev_info(&sensor->i2c_client->dev, "%s[%d]",
			 __func__, __LINE__);

	MUTEX_LOCK(&sensor->lock);

	if (!sensor->reset_gpio)
	{
		dev_info(&sensor->i2c_client->dev, "%s[%d]: - ignore reset request because missing reset gpio",
				 __func__, __LINE__);
		sensor->pending_hardtreset_request = 0;

		return;
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

	sensor->pending_hardtreset_request = 0;

	// ToDo: setup MIPI-Lanes and Clock again!

	MUTEX_UNLOCK(&sensor->lock);
}

static void avt3_soft_reset(struct avt3_dev *sensor)
{
	// todo: needs to be adapted on sequencing and heartbeet GenCP

	struct i2c_client *client = sensor->i2c_client;
	int ret;

	dev_info(&client->dev, "%s[%d]",
			 __func__, __LINE__);

	MUTEX_LOCK(&sensor->lock);

	ret = regmap_write(sensor->regmap8, cci_cmd_tbl[SOFT_RESET].address, 1);

	if (ret < 0)
	{
		dev_err(&client->dev, "%s[%d]: avt3_soft_reset request by calling regmap_write failed (%d)\n",
				__func__, __LINE__, ret);
		goto out;
	}

	sensor->pending_softreset_request = 0;

	// ToDo: setup MIPI-Lanes and Clock again!

out:
	MUTEX_UNLOCK(&sensor->lock);
}

static void avt3_dphy_reset(struct avt3_dev *sensor, bool bResetPhy)
{
	// struct v4l2_ext_control vc;
	struct i2c_client *client = sensor->i2c_client;
	int ret;
	int ival = bResetPhy;

	dev_info(&client->dev, "%s[%d]",
			 __func__, __LINE__);

	//	MUTEX_LOCK(&sensor->lock);

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
	//	MUTEX_UNLOCK(&sensor->lock);
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
//	dump_stack();
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

static int avt3_try_fmt_internal(struct v4l2_subdev *sd,
								 struct v4l2_mbus_framefmt *fmt,
								 enum avt3_frame_rate fr,
								 const struct avt3_mode_info **new_mode)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	const struct avt3_mode_info *mode;
	int i;

	dev_info(&sensor->i2c_client->dev, "%s[%d]",
			 __func__, __LINE__);
	adev_info(&sensor->i2c_client->dev, "fmt->width %d, fmt->height %d, fmt->code 0x%04X, fr %d, "
		"sensor->available_fmts_cnt %d, sensor->mbus_framefmt.code 0x%04X",
			  fmt->width, fmt->height, fmt->code, fr, 
			  sensor->available_fmts_cnt,
			  sensor->mbus_framefmt.code);

	v4l_bound_align_image(&fmt->width,sensor->min_rect.width,sensor->max_rect.width,1,
                          &fmt->height,sensor->min_rect.height,sensor->max_rect.height,1,1);


	mode = avt3_find_mode(sensor, fr, fmt->width, fmt->height, true);
	if (!mode)
	{
		adev_info(&sensor->i2c_client->dev, "avt3_find_mode failed: fmt->width %d, fmt->height %d, fr %d",
				  fmt->width, fmt->height, fr);
		return -EINVAL;
	}
	adev_info(&sensor->i2c_client->dev, "avt3_find_mode returned: , mode->hact %d, mode->vact %d, fr %d, fmt->width %d, fmt->height %d",
			  mode->hact, mode->vact, fr, fmt->width, fmt->height);


	if (new_mode)
		*new_mode = mode;

	for (i = 0; i < sensor->available_fmts_cnt; i++)
	{
		adev_info(&sensor->i2c_client->dev, "loop %d: fmt->width %d, fmt->height %d, fr %d, "
		 	"sensor->mbus_framefmt.code 0x%04X, "
			"sensor->available_fmts[%d].mbus_code 0x%04X, "
			"fmt->code 0x%04X",
				  i, fmt->width, fmt->height, fr,
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
		avt_err(sd, "format fmt->code 0x%04X not found in available formats [ToDo: error handling incomplete]", fmt->code);
		fmt->code = sensor->mbus_framefmt.code;
		//return -EINVAL;
	}

//	fmt->width = mode->hact;
//	fmt->height = mode->vact;
	memset(fmt->reserved, 0, sizeof(fmt->reserved));

	fmt->colorspace = sensor->available_fmts[i].colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	return 0;
}

static int avt3_pad_ops_set_fmt(struct v4l2_subdev *sd,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
								struct v4l2_subdev_state *sd_state,
#else
								struct v4l2_subdev_pad_config *cfg,
#endif
								struct v4l2_subdev_format *format)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	const struct avt3_mode_info *new_mode;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	dev_info(&sensor->i2c_client->dev, "%s[%d]",
			 __func__, __LINE__);
	adev_info(&sensor->i2c_client->dev, "%d x %d, format.code 0x%04X, format.pad %d", 
			format->format.width, format->format.height, format->format.code, format->pad);

	avt_dbg(sd, "%d x %d, format.code 0x%04X, format.pad %d",
			format->format.width, format->format.height, format->format.code, format->pad);

	if (format->pad != 0)
		return -EINVAL;

	MUTEX_LOCK(&sensor->lock);
	if (sensor->is_streaming)
	{
		ret = -EBUSY;
		goto out;
	}

	ret = avt3_try_fmt_internal(sd, mbus_fmt,
								sensor->current_fr, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		adev_info(&sensor->i2c_client->dev, "format->which == V4L2_SUBDEV_FORMAT_TRY");
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
		fmt = v4l2_subdev_get_try_format(sd, sd_state, 0);
#else
		fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
#endif
	} else {
		adev_info(&sensor->i2c_client->dev, "format->which != V4L2_SUBDEV_FORMAT_TRY");
		fmt = &sensor->mbus_framefmt;
	}

	*fmt = *mbus_fmt;

	if (new_mode != sensor->current_mode)
	{
		sensor->current_mode = new_mode;
		sensor->pending_mode_change = true;
	}
	if (mbus_fmt->code != sensor->mbus_framefmt.code)
		sensor->pending_fmt_change = true;

	if (sensor->pending_mode_change || sensor->pending_fmt_change)
		sensor->mbus_framefmt = *mbus_fmt;

out:
	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

static int avt3_ctrl_send(struct i2c_client *client,
						  struct avt_ctrl *vc)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);
	int ret = 0;
	unsigned int reg = 0;
	int length = 0;
	int fmtidx;

	int r_wn = 0; /* write -> r_wn = 0, read -> r_wn = 1 */
	__u8 bayer_temp = 0;

	avt_dbg(&sensor->sd, "switch (vc->id) %x ", vc->id);

	switch (vc->id)
	{

	case V4L2_AV_CSI2_STREAMON_W: {
		unsigned int	acquisition_state;

		ret = regmap_read(sensor->regmap8,
						BCRM_ACQUISITION_STOP_8RW, &acquisition_state);
		if (0 != acquisition_state) {
			adev_info(&client->dev, 
				"V4L2_AV_CSI2_STREAMON_W called but cam is streaming already. acquisition_state %d, sensor->is_streaming %d",
				acquisition_state, sensor->is_streaming);
			dump_stack();
			return -EINVAL;
		}
	}

		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_STREAMON_W %d", vc->value0);
		reg = BCRM_ACQUISITION_START_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_AV_CSI2_STREAMOFF_W:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_STREAMOFF_W %d", vc->value0);
		reg = BCRM_ACQUISITION_STOP_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_AV_CSI2_ABORT_W:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_ABORT_W ");
		reg = BCRM_ACQUISITION_ABORT_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_AV_CSI2_WIDTH_W:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_WIDTH_W %d", vc->value0);
		reg = BCRM_IMG_WIDTH_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_HEIGHT_W:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_HEIGHT_W %d", vc->value0);
		reg = BCRM_IMG_HEIGHT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 0;
		break;
	case V4L2_AV_CSI2_OFFSET_X_W:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_OFFSET_X_W %d", vc->value0);
		reg = BCRM_IMG_OFFSET_X_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 0;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_W:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_OFFSET_Y_W %d", vc->value0);
		reg = BCRM_IMG_OFFSET_Y_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 0;
		break;
	case V4L2_AV_CSI2_HFLIP_W:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_HFLIP_W %d", vc->value0);
		reg = BCRM_IMG_REVERSE_X_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 0;
		break;
	case V4L2_AV_CSI2_VFLIP_W:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_VFLIP_W %d", vc->value0);
		reg = BCRM_IMG_REVERSE_Y_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 0;
		break;

	case V4L2_AV_CSI2_PIXELFORMAT_W:
		avt_dbg(&sensor->sd, "V4L2_AV_CSI2_PIXELFORMAT_W %d 0x%04X", vc->value0, vc->value0);
		reg = BCRM_IMG_MIPI_DATA_FORMAT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 0;
		avt_dbg(&sensor->sd, "switch (vc->id) %d V4L2_AV_CSI2_PIXELFORMAT_W %d 0x%04X",
				vc->id, vc->value0, vc->value0);

		fmtidx = lockup_media_bus_fmt(sensor, vc->value0);

		if (fmtidx == -EINVAL || fmtidx >= sensor->available_fmts_cnt)
		{
			adev_info(&client->dev, "not supported by the host, lockup_media_bus_fmt returned fmtidx %d for V4L2_AV_CSI2_PIXELFORMAT_W %d 0x%04X",
					  fmtidx,
					  vc->value0, vc->value0);
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

		vc->value0 = sensor->available_fmts[fmtidx].mipi_fmt;
		bayer_temp = sensor->available_fmts[fmtidx].bayer_pattern;
		break;

	case V4L2_AV_CSI2_WIDTH_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_WIDTH_R ", __func__, __LINE__);
		reg = BCRM_IMG_WIDTH_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_WIDTH_MINVAL_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_WIDTH_MINVAL_R ", __func__, __LINE__);
		reg = BCRM_IMG_WIDTH_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_WIDTH_MAXVAL_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_WIDTH_MAXVAL_R ", __func__, __LINE__);
		reg = BCRM_IMG_WIDTH_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_WIDTH_INCVAL_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_WIDTH_INCVAL_R ", __func__, __LINE__);
		reg = BCRM_IMG_WIDTH_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_HEIGHT_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_HEIGHT_R ", __func__, __LINE__);
		reg = BCRM_IMG_HEIGHT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_HEIGHT_MINVAL_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_HEIGHT_MINVAL_R ", __func__, __LINE__);
		reg = BCRM_IMG_HEIGHT_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_HEIGHT_MAXVAL_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_HEIGHT_MAXVAL_R ", __func__, __LINE__);
		reg = BCRM_IMG_HEIGHT_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_HEIGHT_INCVAL_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_HEIGHT_INCVAL_R ", __func__, __LINE__);
		reg = BCRM_IMG_HEIGHT_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_OFFSET_X_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_OFFSET_X_R ", __func__, __LINE__);
		reg = BCRM_IMG_OFFSET_X_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_OFFSET_X_MIN_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_OFFSET_X_MIN_R ", __func__, __LINE__);
		reg = BCRM_IMG_OFFSET_X_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_OFFSET_X_MAX_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_OFFSET_X_MAX_R ", __func__, __LINE__);
		reg = BCRM_IMG_OFFSET_X_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_OFFSET_X_INC_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_OFFSET_X_INC_R ", __func__, __LINE__);
		reg = BCRM_IMG_OFFSET_X_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_OFFSET_Y_R ", __func__, __LINE__);
		reg = BCRM_IMG_OFFSET_Y_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_MIN_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_OFFSET_Y_MIN_R ", __func__, __LINE__);
		reg = BCRM_IMG_OFFSET_Y_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_MAX_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_OFFSET_Y_MAX_R ", __func__, __LINE__);
		reg = BCRM_IMG_OFFSET_Y_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_INC_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_OFFSET_Y_INC_R ", __func__, __LINE__);
		reg = BCRM_IMG_OFFSET_Y_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_SENSOR_WIDTH_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_SENSOR_WIDTH_R ", __func__, __LINE__);
		reg = BCRM_SENSOR_WIDTH_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_SENSOR_HEIGHT_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_SENSOR_HEIGHT_R ", __func__, __LINE__);
		reg = BCRM_SENSOR_HEIGHT_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_MAX_WIDTH_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_MAX_WIDTH_R ", __func__, __LINE__);
		reg = BCRM_WIDTH_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_MAX_HEIGHT_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_MAX_HEIGHT_R ", __func__, __LINE__);
		reg = BCRM_HEIGHT_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_PIXELFORMAT_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_PIXELFORMAT_R ", __func__, __LINE__);
		reg = BCRM_IMG_MIPI_DATA_FORMAT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_PAYLOADSIZE_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_PAYLOADSIZE_R ", __func__, __LINE__);
		reg = BCRM_BUFFER_SIZE_32R;
		length = AV_CAM_DATA_SIZE_32;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_ACQ_STATUS_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_ACQ_STATUS_R ", __func__, __LINE__);
		reg = BCRM_ACQUISITION_STATUS_8R;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_HFLIP_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_HFLIP_R ", __func__, __LINE__);
		reg = BCRM_IMG_REVERSE_X_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_VFLIP_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_VFLIP_R ", __func__, __LINE__);
		reg = BCRM_IMG_REVERSE_Y_8RW;
		length = AV_CAM_DATA_SIZE_8;
		r_wn = 1;
		break;
	case V4L2_AV_CSI2_CURRENTMODE_R:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_CURRENTMODE_R ", __func__, __LINE__);
		ret = regmap_read(sensor->regmap8,
						  GENCP_CURRENTMODE_8R, &vc->value0);
		if (ret < 0)
		{
			dev_err(&client->dev, "%s[%d]: Failed to get mode: regmap_read on GENCP_CURRENTMODE_8R failed (%d)\n",
					__func__, __LINE__, ret);
			return ret;
		}
		return ret;

	case V4L2_AV_CSI2_CHANGEMODE_W:
		dev_info(&client->dev, "%s[%d]: V4L2_AV_CSI2_CHANGEMODE_W ", __func__, __LINE__);

		if (vc->value0 == 1)
		{
			ret = avt3_set_gencp(client);
		}
		else
		{
			ret = avt3_set_bcrm(client);
		}
		return ret;

	default:
		dev_err(&client->dev, "%s[%d]: unknown ctrl 0x%x\n", __func__, __LINE__, vc->id);
		return -EINVAL;
	}

	if (r_wn)
	{ /* read (r_wn=1) */

		switch (length)
		{
		case AV_CAM_DATA_SIZE_8:
			ret = regmap_read(sensor->regmap8,
							  sensor->cci_reg.reg.bcrm_addr + reg, &vc->value0);
			break;
		case AV_CAM_DATA_SIZE_16:
			ret = regmap_read(sensor->regmap16,
							  sensor->cci_reg.reg.bcrm_addr + reg, &vc->value0);
			break;
		case AV_CAM_DATA_SIZE_32:
			ret = regmap_read(sensor->regmap32,
							  sensor->cci_reg.reg.bcrm_addr + reg, &vc->value0);
			break;
		case AV_CAM_DATA_SIZE_64:
			ret = regmap_bulk_read(sensor->regmap64,
								   sensor->cci_reg.reg.bcrm_addr + reg, &vc->value0, 1);
			break;
		default:
			dev_err(&client->dev, "%s[%d]: unknown length %d\n", __func__, __LINE__, length);
		}

		if (ret < 0)
		{
			dev_err(&client->dev, "%s[%d]: regmap_read at 0x%04x failed with %d\n",
					__func__, __LINE__, sensor->cci_reg.reg.bcrm_addr + reg, ret);
			return ret;
		}

		if (vc->id == V4L2_AV_CSI2_PIXELFORMAT_R)
		{
			// TODO: Check correct mbus_format_value
			vc->value0 = sensor->mbus_fmt_code;
			dev_info(&client->dev,
					 "%s[%d]: V4L2_AV_CSI2_PIXELFORMAT_R TODO: Check correct mbus_format_value "
					 "vc->value0 0x%04X == sensor->mbus_fmt_code 0x%04X\n",
					 __func__, __LINE__, vc->value0, sensor->mbus_fmt_code);
		}
		return 0;
	}
	else
	{ /* write (r_wn=0) */
		avt_dbg(&sensor->sd, "reg %x, length %d, vc->value0 0x%x\n", reg, length, vc->value0);

		switch (length)
		{
		case AV_CAM_DATA_SIZE_8:
			ret = bcrm_regmap_write(sensor, sensor->regmap8,
									sensor->cci_reg.reg.bcrm_addr + reg, vc->value0);
			break;
		case AV_CAM_DATA_SIZE_16:
			ret = bcrm_regmap_write(sensor, sensor->regmap16,
									sensor->cci_reg.reg.bcrm_addr + reg, vc->value0);
			break;
		case AV_CAM_DATA_SIZE_32:
			ret = bcrm_regmap_write(sensor, sensor->regmap32,
									sensor->cci_reg.reg.bcrm_addr + reg, vc->value0);
			break;
		case AV_CAM_DATA_SIZE_64:
			ret = bcrm_regmap_write64(sensor, sensor->regmap64,
									  sensor->cci_reg.reg.bcrm_addr + reg, vc->value64);
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
		if (vc->id == V4L2_AV_CSI2_PIXELFORMAT_W && bayer_temp != bayer_ignore)
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
}

// V4L2_EXPOSURE_AUTO = 0,
// V4L2_EXPOSURE_MANUAL = 1,
// V4L2_EXPOSURE_SHUTTER_PRIORITY = 2,
// V4L2_EXPOSURE_APERTURE_PRIORITY = 3

//#define V4L2_CID_EXPOSURE		(V4L2_CID_BASE+17)
//#define V4L2_CID_EXPOSURE_AUTO			(V4L2_CID_CAMERA_CLASS_BASE+1)
//#define V4L2_CID_EXPOSURE_ABSOLUTE		(V4L2_CID_CAMERA_CLASS_BASE+2)
//#define V4L2_CID_EXPOSURE_AUTO_PRIORITY		(V4L2_CID_CAMERA_CLASS_BASE+3)
//#define V4L2_CID_AUTO_EXPOSURE_BIAS

static int avt3_queryctrl(struct v4l2_subdev *sd,
						  //		struct v4l2_queryctrl *qctrl,
						  struct v4l2_query_ext_ctrl *qctrl)
{
	//	struct i2c_client *client = v4l2_get_subdevdata(sd);
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
#if 0
		ret = regmap_read(sensor->regmap32,
			sensor->cci_reg.reg.bcrm_addr + BCRM_BLACK_LEVEL_32RW,
			&qctrl->default_value);
#else
		qctrl->default_value = qctrl->minimum + (qctrl->maximum - qctrl->minimum) / 2;
#endif
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

		// #define V4L2_LOCK_EXPOSURE			(1 << 0)
		// #define V4L2_LOCK_WHITE_BALANCE		(1 << 1)
		// #define V4L2_LOCK_FOCUS				(1 << 2)

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

static int avt3_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{

	struct avt3_dev *sensor = container_of(ctrl->handler, struct avt3_dev, v4l2_ctrl_hdl);
	struct v4l2_control c;
	int ret = 0;

	avt_dbg(&sensor->sd, "ctrl->id %d", ctrl->id);

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
		const struct v4l2_ctrl * ctrl = camera->avt3_ctrls[i];

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

static int avt3_v4l2_ctrl_ops_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avt3_dev *sensor = container_of(ctrl->handler, struct avt3_dev, v4l2_ctrl_hdl);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	struct avt_val64 val64;

	CLEAR(val64);

	//				regmap_read(sensor->regmap8, sensor->cci_reg.bcrm_addr + nOffset, &val64.u32[0]);
	//				regmap_read(sensor->regmap16, sensor->cci_reg.bcrm_addr + nOffset, &val64.u32[0]);
	//				regmap_read(sensor->regmap32, sensor->cci_reg.bcrm_addr + nOffset, &val64.u32[0]);
	//				regmap_bulk_read(sensor->regmap64, sensor->cci_reg.bcrm_addr + nOffset, &val64, 1);
	// struct avt_ctrl ct;
	//	struct v4l2_ext_control c;

	/* v4l2_ctrl_lock() locks our own mutex */

	/*
	 * If the device is not powered up by the host driver do
	 * not apply any controls to H/W at this time. Instead
	 * the controls will be restored right after power-up.
	 */

	//	dev_info(&sensor->i2c_client->dev, "%s[%d]: ctrl->id 0x%08X, sensor->power_count %d",
	//			 __func__, __LINE__, ctrl->id, sensor->power_count);

	/* ignore if sensor is in sleep mode */
	if (sensor->power_count == 0)
	{
		avt_info(&sensor->sd, "ToDo: Sensor is in sleep mode. Maybe it is better to ignore ctrl->id 0x%08X, sensor->power_count %d",
				 ctrl->id, sensor->power_count);
		// return -EINVAL;
	}

	if (sensor->power_count > 1)
	{
		avt_info(&sensor->sd, "ctrl->id 0x%08X, sensor->power_count %d", ctrl->id, sensor->power_count);
	}

	switch (ctrl->id)
	{

		/* BLACK LEVEL is deprecated and thus we use Brightness */
	case V4L2_CID_BRIGHTNESS:
		if (!sensor->feature_inquiry_reg.feature_inq.black_level_avail)
		{
			avt_info(&sensor->sd, "V4L2_CID_BRIGHTNESS not supported by current camera model and firmware!");
			return -ENOTSUPP;
		}
		avt_dbg(&sensor->sd, "V4L2_CID_BRIGHTNESS %d", ctrl->val);
		ret = bcrm_regmap_write(sensor, sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_BLACK_LEVEL_32RW, ctrl->val);
		break;

	case V4L2_CID_HFLIP:
		avt_info(&sensor->sd, "V4L2_CID_HFLIP %d", ctrl->val);
		ret = bcrm_regmap_write(sensor, sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_REVERSE_X_8RW, ctrl->val);
		sensor->hflip = ctrl->val;
		break;

	case V4L2_CID_VFLIP:
		avt_dbg(&sensor->sd, "V4L2_CID_VFLIP %d\n", ctrl->val);
		ret = bcrm_regmap_write(sensor, sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_REVERSE_Y_8RW, ctrl->val);
		sensor->vflip = ctrl->val;
		break;

	case V4L2_CID_GAMMA:
		if (!sensor->feature_inquiry_reg.feature_inq.gamma_avail)
		{
			avt_info(&sensor->sd, "V4L2_CID_GAMMA not supported by current camera model and firmware!");
			return -ENOTSUPP;
		}
		avt_dbg(&sensor->sd, "V4L2_CID_GAMMA %d", ctrl->val);
		val64.s64 = ctrl->val;
		ret = regmap_bulk_write(sensor->regmap64, sensor->cci_reg.reg.bcrm_addr + BCRM_GAMMA_64RW, &val64, 1);
		break;

	case V4L2_CID_CONTRAST:
		if (!sensor->feature_inquiry_reg.feature_inq.contrast_avail)
		{
			dev_warn(&client->dev, "%s[%d]: V4L2_CID_CONTRAST not supported by current camera model and firmware!\n", __func__, __LINE__);
			return -ENOTSUPP;
		}
		dev_info(&client->dev, "%s[%d]: V4L2_CID_CONTRAST %d\n", __func__, __LINE__, ctrl->val);
		val64.s64 = ctrl->val;
		ret = bcrm_regmap_write(sensor, sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_CONTRAST_VALUE_32RW, ctrl->val);
		break;

	case V4L2_CID_DO_WHITE_BALANCE:
		dev_info(&client->dev, "%s[%d]: V4L2_CID_DO_WHITE_BALANCE %d\n", __func__, __LINE__, ctrl->val);
		if (!sensor->feature_inquiry_reg.feature_inq.white_balance_auto_avail)
		{
			dev_warn(&client->dev, "%s[%d]: V4L2_CID_DO_WHITE_BALANCE %d but sensor->feature_inquiry_reg.feature_inq.white_balance_auto_avail == 0 \n", __func__, __LINE__, ctrl->val);
			return -ENOTSUPP;
		}
		ret = bcrm_regmap_write(sensor, sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WHITE_BALANCE_AUTO_8RW, ctrl->val > 0 ? 1 : 0);
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		if (!sensor->feature_inquiry_reg.feature_inq.white_balance_auto_avail)
		{
			dev_warn(&client->dev, "%s[%d]: V4L2_CID_AUTO_WHITE_BALANCE\n", __func__, __LINE__);
			return -ENOTSUPP;
		}
		dev_info(&client->dev, "%s[%d]: V4L2_CID_AUTO_WHITE_BALANCE %d\n", __func__, __LINE__, ctrl->val);

		ret = bcrm_regmap_write(sensor, sensor->regmap8,
								sensor->cci_reg.reg.bcrm_addr + BCRM_WHITE_BALANCE_AUTO_8RW,
								ctrl->val > 0 ? 2 : 0);
		break;

	case V4L2_CID_SATURATION:
		if (!sensor->feature_inquiry_reg.feature_inq.saturation_avail)
		{
			dev_warn(&client->dev, "%s[%d]: V4L2_CID_SATURATION not supported by current camera model and firmware!\n", __func__, __LINE__);
			return -ENOTSUPP;
		}
		dev_info(&client->dev, "%s[%d]: V4L2_CID_SATURATION %d\n", __func__, __LINE__, ctrl->val);
		ret = bcrm_regmap_write(sensor, sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_SATURATION_32RW, ctrl->val);
		break;

	case V4L2_CID_HUE:
		if (!sensor->feature_inquiry_reg.feature_inq.hue_avail)
		{
			dev_warn(&client->dev, "%s[%d]: V4L2_CID_HUE not supported by current camera model and firmware!\n", __func__, __LINE__);
			return -ENOTSUPP;
		}
		dev_info(&client->dev, "%s[%d]: V4L2_CID_HUE %d\n", __func__, __LINE__, ctrl->val);
		ret = bcrm_regmap_write(sensor, sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_HUE_32RW, ctrl->val);
		break;

	case V4L2_CID_RED_BALANCE:
		dev_info(&client->dev, "%s[%d]: V4L2_CID_RED_BALANCE %d\n", __func__, __LINE__, ctrl->val);
		ret = regmap_bulk_write(sensor->regmap64, sensor->cci_reg.reg.bcrm_addr + BCRM_RED_BALANCE_RATIO_64RW, &val64, 1);
		break;

	case V4L2_CID_BLUE_BALANCE:
		dev_info(&client->dev, "%s[%d]: V4L2_CID_BLUE_BALANCE %d\n", __func__, __LINE__, ctrl->val);
		ret = regmap_bulk_write(sensor->regmap64, sensor->cci_reg.reg.bcrm_addr + BCRM_BLUE_BALANCE_RATIO_64RW, &val64, 1);
		break;

	case V4L2_CID_EXPOSURE:
		dev_info(&client->dev, "%s[%d]: V4L2_CID_EXPOSURE %d\n", __func__, __LINE__, ctrl->val);
		sensor->exposure_time = val64.s64 = ctrl->val;
		ret = regmap_bulk_write(sensor->regmap64, sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_64RW, &val64, 1);
		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		// dev_info(&client->dev, "%s[%d]: V4L2_CID_EXPOSURE_ABSOLUTE %d\n", __func__, __LINE__, ctrl->val);
		val64.s64 = ctrl->val;
		sensor->exposure_time = val64.s64 * 100;
		dev_info(&client->dev, "%s[%d]: V4L2_CID_EXPOSURE_ABSOLUTE %d --> %lld ns\n", __func__, __LINE__, ctrl->val, sensor->exposure_time);
		ret = regmap_bulk_write(sensor->regmap64, sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_64RW, &val64, 1);
		break;

	case V4L2_CID_GAIN:
		if (!sensor->feature_inquiry_reg.feature_inq.gain_avail)
		{
			dev_warn(&client->dev, "%s[%d]: V4L2_CID_GAIN not supported by current camera model and firmware!\n", __func__, __LINE__);
			return -ENOTSUPP;
		}
		dev_info(&client->dev, "%s[%d]: V4L2_CID_GAIN %d\n", __func__, __LINE__, ctrl->val);
		sensor->gain = val64.s64 = ctrl->val;
		ret = regmap_bulk_write(sensor->regmap64, sensor->cci_reg.reg.bcrm_addr + BCRM_GAIN_64RW, &val64, 1);
		break;

	case V4L2_CID_AUTOGAIN:
		if (!sensor->feature_inquiry_reg.feature_inq.gain_auto_avail)
		{
			dev_warn(&client->dev, "%s[%d]: V4L2_CID_GAIN not supported by current camera model and firmware!\n", __func__, __LINE__);
			return -ENOTSUPP;
		}
		dev_info(&client->dev, "%s[%d]: V4L2_CID_AUTOGAIN %d\n", __func__, __LINE__, ctrl->val);
		ret = bcrm_regmap_write(sensor, sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_GAIN_AUTO_8RW, ctrl->val);
		break;

	case V4L2_CID_SHARPNESS:
		if (!sensor->feature_inquiry_reg.feature_inq.sharpness_avail)
		{
			dev_warn(&client->dev, "%s[%d]: V4L2_CID_SHARPNESS not supported by current camera model and firmware!\n",
					 __func__, __LINE__);
			return -ENOTSUPP;
		}
		dev_info(&client->dev, "%s[%d]: V4L2_CID_SHARPNESS\n", __func__, __LINE__);
		// reg = SHARPNESS_32RW;
		//  ToDo: implement sharpness settings
		break;

	default:
	{
		if (ctrl->priv != NULL)
		{
			const struct avt_ctrl_mapping * const ctrl_mapping = ctrl->priv;
			const struct regmap * ctrl_regmap = avt3_get_regmap_by_size(sensor,ctrl_mapping->data_size);

			dev_info(&client->dev, "%s[%d]: Write custom ctrl %s (%x)\n", __func__, __LINE__, ctrl_mapping->attr.name, ctrl->id);



			if (ctrl_mapping->data_size == AV_CAM_DATA_SIZE_64)
			{
				if (ctrl_mapping->type == V4L2_CTRL_TYPE_BUTTON)
				{
					bcrm_regmap_write64(sensor, ctrl_regmap,
  									sensor->cci_reg.reg.bcrm_addr + ctrl_mapping->reg_offset, 1);
				}
				else
				{

					bcrm_regmap_write64(sensor, ctrl_regmap,
  									sensor->cci_reg.reg.bcrm_addr + ctrl_mapping->reg_offset, *ctrl->p_new.p_s64);
				}
			}
			else
			{
				if (ctrl_mapping->type == V4L2_CTRL_TYPE_BUTTON)
				{
					bcrm_regmap_write(sensor, ctrl_regmap,
  									sensor->cci_reg.reg.bcrm_addr + ctrl_mapping->reg_offset, 1);
				}
				else
				{

					bcrm_regmap_write(sensor, ctrl_regmap,
  									sensor->cci_reg.reg.bcrm_addr + ctrl_mapping->reg_offset, ctrl->val);
				}
			}

			switch (ctrl->id)
			{
				case V4L2_CID_TRIGGER_MODE:
					sensor->avt_trigger_status.trigger_mode_enabled = ctrl->val;
					avt3_update_sw_ctrl_state(sensor);
					break;
				case V4L2_CID_TRIGGER_SOURCE:
					sensor->avt_trigger_status.trigger_source = ctrl->val;
					avt3_update_sw_ctrl_state(sensor);
					break;
				case V4L2_CID_TRIGGER_ACTIVATION:
					sensor->avt_trigger_status.trigger_activation = ctrl->val;
					break;
				default:
					break;
			}
		}
		else
		{
			dev_err(&sensor->i2c_client->dev, "%s[%d]: case default or not supported id %d, val %d\n",
					__func__, __LINE__, ctrl->id, ctrl->val);
			return -EINVAL;
		}
	}

	}

	//		ret = regmap_bulk_write(sensor->regmap64, sensor->cci_reg.reg.bcrm_addr + BCRM_RED_BALANCE_RATIO_64RW, &val64, 1);
	return ret;
}

static const struct v4l2_ctrl_ops avt3_ctrl_ops = {
	.g_volatile_ctrl = avt3_g_volatile_ctrl,
	.s_ctrl = avt3_v4l2_ctrl_ops_s_ctrl,
};

static int avt3_initctrl(struct v4l2_subdev *sd,
						 const struct avt_ctrl_mapping *avt_ctrl_mapping,
						 struct v4l2_query_ext_ctrl *qectrl)
{
	//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct avt3_dev *sensor = to_avt3_dev(sd);
	int ret = 0;
	s32 s32tmp;

	qectrl->type = avt_ctrl_mapping->type;

	if (qectrl->type == V4L2_CTRL_TYPE_INTEGER ||
		qectrl->type == V4L2_CTRL_TYPE_INTEGER64)
		qectrl->flags |= V4L2_CTRL_FLAG_SLIDER;

	memset(qectrl->name, 0, sizeof(qectrl->name));
	strncpy(qectrl->name, avt_ctrl_mapping->attr.name, sizeof(qectrl->name) - 1);

	switch (avt_ctrl_mapping->data_size)
	{

	case AV_CAM_DATA_SIZE_8:
		ret = regmap_read(sensor->regmap8,
						  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->reg_offset, &s32tmp);
		qectrl->default_value = s32tmp;
		if (V4L2_CTRL_TYPE_INTEGER == avt_ctrl_mapping->type ||
			V4L2_CTRL_TYPE_INTEGER64 == avt_ctrl_mapping->type)
		{
			ret = regmap_read(sensor->regmap8,
							  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->max_offset, &s32tmp);
			qectrl->maximum = s32tmp;
			ret = regmap_read(sensor->regmap8,
							  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->min_offset, &s32tmp);
			qectrl->minimum = s32tmp;
			ret = regmap_read(sensor->regmap8,
							  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->step_offset, &s32tmp);
			qectrl->step = s32tmp;
		}
		break;

	case AV_CAM_DATA_SIZE_16:
		ret = regmap_read(sensor->regmap16,
						  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->reg_offset, &s32tmp);
		qectrl->default_value = s32tmp;
		if (V4L2_CTRL_TYPE_INTEGER == avt_ctrl_mapping->type ||
			V4L2_CTRL_TYPE_INTEGER64 == avt_ctrl_mapping->type)
		{
			ret = regmap_read(sensor->regmap16,
							  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->max_offset, &s32tmp);
			qectrl->maximum = s32tmp;
			ret = regmap_read(sensor->regmap16,
							  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->min_offset, &s32tmp);
			qectrl->minimum = s32tmp;
			ret = regmap_read(sensor->regmap16,
							  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->step_offset, &s32tmp);
			qectrl->step = s32tmp;
		}
		break;

	case AV_CAM_DATA_SIZE_32:
		ret = regmap_read(sensor->regmap32,
						  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->reg_offset, &s32tmp);
		qectrl->default_value = s32tmp;
		if (V4L2_CTRL_TYPE_INTEGER == avt_ctrl_mapping->type ||
			V4L2_CTRL_TYPE_INTEGER64 == avt_ctrl_mapping->type)
		{
			ret = regmap_read(sensor->regmap32,
							  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->max_offset, &s32tmp);
			qectrl->maximum = s32tmp;
			ret = regmap_read(sensor->regmap32,
							  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->min_offset, &s32tmp);
			qectrl->minimum = s32tmp;
			ret = regmap_read(sensor->regmap32,
							  sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->step_offset, &s32tmp);
			qectrl->step = s32tmp;

			qectrl->default_value = qectrl->default_value - qectrl->default_value % qectrl->step;
		}
		break;

	case AV_CAM_DATA_SIZE_64:
		ret = regmap_bulk_read(sensor->regmap64,
							   sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->reg_offset,
							   &qectrl->default_value, 1);
		if (V4L2_CTRL_TYPE_INTEGER == avt_ctrl_mapping->type ||
			V4L2_CTRL_TYPE_INTEGER64 == avt_ctrl_mapping->type)
		{
			ret = regmap_bulk_read(sensor->regmap64,
								   sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->min_offset,
								   &qectrl->minimum, 1);
			ret = regmap_bulk_read(sensor->regmap64,
								   sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->max_offset,
								   &qectrl->maximum, 1);
			ret = regmap_bulk_read(sensor->regmap64,
								   sensor->cci_reg.reg.bcrm_addr + avt_ctrl_mapping->step_offset,
								   &qectrl->step, 1);
			qectrl->default_value = qectrl->default_value - qectrl->default_value % qectrl->step;
		}
		break;

	default:
		ret = -EINVAL;
		goto err_out;
	}
	avt_dbg(sd, "prepared control qectrl->id 0x%08X, size %d, qectrl->type %d, qectrl->name %s, [%lld, %lld], %lld, s %lld\n",
			qectrl->id, avt_ctrl_mapping->data_size,
			qectrl->type, qectrl->name, qectrl->minimum, qectrl->maximum, qectrl->default_value, qectrl->step);
	return ret;

err_out:
	avt_err(sd, "failed with %d to prepare control qectrl->id 0x%08X, size %d, qectrl->type %d, qectrl->name %s, [%lld, %lld], %lld, s %lld\n",
			ret,
			qectrl->id, avt_ctrl_mapping->data_size,
			qectrl->type, qectrl->name,
			qectrl->minimum, qectrl->maximum,
			qectrl->default_value, qectrl->step);
	return ret;
}

static int avt3_init_controls(struct avt3_dev *sensor)
{
	// struct i2c_client *client = sensor->i2c_client;
	// struct v4l2_queryctrl qectrl;
	struct v4l2_query_ext_ctrl qectrl;
	struct v4l2_ctrl *ctrl;
	int ret;
	int i, j;

	avt_dbg(&sensor->sd, "code uses now v4l2_ctrl_new_std and v4l2_query_ext_ctrl (VIDIOC_QUERY_EXT_CTRL / s64) ");

	//	MUTEX_LOCK(&sensor->lock);

	//	dev_info(&sensor->i2c_client->dev, "%s[%d]: try to call v4l2_ctrl_handler_init", __func__, __LINE__);

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
		switch (avt_ctrl_mappings[j].id)
		{
		case V4L2_CID_CONTRAST:
			if (!sensor->feature_inquiry_reg.feature_inq.contrast_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_CONTRAST %d",
						 sensor->feature_inquiry_reg.feature_inq.contrast_avail);
				continue;
			}
			break;
		case V4L2_CID_SATURATION:
			if (!sensor->feature_inquiry_reg.feature_inq.saturation_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_SATURATION %d",
						 sensor->feature_inquiry_reg.feature_inq.saturation_avail);
				continue;
			}
			break;
		case V4L2_CID_HUE:
			if (!sensor->feature_inquiry_reg.feature_inq.hue_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_HUE %d",
						 sensor->feature_inquiry_reg.feature_inq.hue_avail);
				continue;
			}
			break;
		case V4L2_CID_AUTO_WHITE_BALANCE:
			if (!sensor->feature_inquiry_reg.feature_inq.white_balance_auto_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_AUTO_WHITE_BALANCE %d",
						 sensor->feature_inquiry_reg.feature_inq.white_balance_auto_avail);
				continue;
			}
			break;
		case V4L2_CID_DO_WHITE_BALANCE:
			if (!sensor->feature_inquiry_reg.feature_inq.white_balance_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_DO_WHITE_BALANCE %d",
						 sensor->feature_inquiry_reg.feature_inq.white_balance_avail);
				continue;
			}
			break;
		case V4L2_CID_GAMMA:
			if (!sensor->feature_inquiry_reg.feature_inq.gamma_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_GAMMA %d",
						 sensor->feature_inquiry_reg.feature_inq.gamma_avail);
				continue;
			}
			break;
		case V4L2_CID_AUTOGAIN:
			if (!sensor->feature_inquiry_reg.feature_inq.gain_auto_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_AUTOGAIN %d",
						 sensor->feature_inquiry_reg.feature_inq.gain_auto_avail);
				continue;
			}
			break;
		case V4L2_CID_GAIN:
			if (!sensor->feature_inquiry_reg.feature_inq.gain_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_GAIN %d",
						 sensor->feature_inquiry_reg.feature_inq.gain_avail);
				continue;
			}
			break;
		case V4L2_CID_HFLIP:
			if (!sensor->feature_inquiry_reg.feature_inq.reverse_x_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_HFLIP %d",
						 sensor->feature_inquiry_reg.feature_inq.reverse_x_avail);
				continue;
			}
			break;
		case V4L2_CID_VFLIP:
			if (!sensor->feature_inquiry_reg.feature_inq.reverse_y_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_VFLIP %d",
						 sensor->feature_inquiry_reg.feature_inq.reverse_y_avail);
				continue;
			}
			break;
		case V4L2_CID_SHARPNESS:
			if (!sensor->feature_inquiry_reg.feature_inq.sharpness_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_SHARPNESS %d",
						 sensor->feature_inquiry_reg.feature_inq.sharpness_avail);
				continue;
			}
			break;
		case V4L2_CID_EXPOSURE_AUTO:
			if (!sensor->feature_inquiry_reg.feature_inq.exposure_auto_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_EXPOSURE_AUTO %d",
						 sensor->feature_inquiry_reg.feature_inq.exposure_auto_avail);
				continue;
			}
			break;
		case V4L2_CID_BRIGHTNESS:
			if (!sensor->feature_inquiry_reg.feature_inq.black_level_avail)
			{
				avt_info(&sensor->sd, "skip V4L2_CID_BRIGHTNESS %d",
						 sensor->feature_inquiry_reg.feature_inq.black_level_avail);
				continue;
			}
			break;
		case V4L2_CID_TRIGGER_MODE:
		case V4L2_CID_TRIGGER_ACTIVATION:
		case V4L2_CID_TRIGGER_SOURCE:
		case V4L2_CID_TRIGGER_SOFTWARE:
			if (!sensor->feature_inquiry_reg.feature_inq.frame_trigger)
			{
				avt_info(&sensor->sd, "skip trigger ctrl %d %d",
 						avt_ctrl_mappings[j].id,sensor->feature_inquiry_reg.feature_inq.frame_trigger);
				continue;
			}
			break;
		/* let's asume that this features are available on all cameras */
		case V4L2_CID_EXPOSURE:
		case V4L2_CID_EXPOSURE_ABSOLUTE:
		case V4L2_CID_RED_BALANCE:
		case V4L2_CID_BLUE_BALANCE:
			break;
		default:
			continue;
		}

		CLEAR(qectrl);
		CLEAR(sensor->avt3_ctrl_cfg[i]);
		qectrl.id = avt_ctrl_mappings[j].id;

		avt_dbg(&sensor->sd, "avt_ctrl_mappings[%d]: %s - Regs: [0x%04x, 0x%04x]: 0x%04x d: 0x%04x", j,
				avt_ctrl_mappings[j].attr.name,
				avt_ctrl_mappings[j].min_offset,
				avt_ctrl_mappings[j].max_offset,
				avt_ctrl_mappings[j].step_offset,
				avt_ctrl_mappings[j].reg_offset);

		ret = avt3_initctrl(&sensor->sd, &avt_ctrl_mappings[j], &qectrl);

		if (ret < 0)
		{
			avt_err(&sensor->sd, "[%d] avt3_initctrl failed i %d, j %d\n",
					avt_ctrl_mappings[j].id, i, j);
			continue;
		}

		avt_dbg(&sensor->sd, "[%d] Checking caps: %s - [%lld, %lld]: %lld d: %lld - %sabled", j,
				avt_ctrl_mappings[j].attr.name,
				qectrl.minimum,
				qectrl.maximum,
				qectrl.step,
				qectrl.default_value,
				(qectrl.flags & V4L2_CTRL_FLAG_DISABLED) ? "dis" : "en");

		if (qectrl.flags & V4L2_CTRL_FLAG_DISABLED /*|| (qectrl.minimum == qectrl.maximum)*/)
			continue;

		//		qectrl.type = sensor->avt3_ctrl_cfg[i].type

		if (qectrl.type == V4L2_CTRL_TYPE_INTEGER ||
			qectrl.type == V4L2_CTRL_TYPE_INTEGER64)
			sensor->avt3_ctrl_cfg[i].flags |= V4L2_CTRL_FLAG_SLIDER;

		sensor->avt3_ctrl_cfg[i].ops = &avt3_ctrl_ops;
		sensor->avt3_ctrl_cfg[i].name = avt_ctrl_mappings[j].attr.name;
		sensor->avt3_ctrl_cfg[i].id = avt_ctrl_mappings[j].id;

		sensor->avt3_ctrl_cfg[i].min = qectrl.minimum;
		sensor->avt3_ctrl_cfg[i].max = qectrl.maximum;
		sensor->avt3_ctrl_cfg[i].def = qectrl.default_value;
		sensor->avt3_ctrl_cfg[i].step = qectrl.step;

		avt_dbg(&sensor->sd, "v4l2_ctrl_new_std %s ctrl %d [%lld, %lld] s %lld d %lld\n",
				sensor->avt3_ctrl_cfg[i].name,
				sensor->avt3_ctrl_cfg[i].id,
				sensor->avt3_ctrl_cfg[i].min,
				sensor->avt3_ctrl_cfg[i].max,
				sensor->avt3_ctrl_cfg[i].step,
				sensor->avt3_ctrl_cfg[i].def);

		if (avt_ctrl_mappings[j].custom)
		{
			struct v4l2_ctrl_config config;
			const struct avt_ctrl_mapping * const ctrl_mapping = &avt_ctrl_mappings[j];
			CLEAR(config);

			avt_info(&sensor->sd, "Init custom ctrl %s (%x)\n", ctrl_mapping->attr.name,ctrl_mapping->id);

			config.ops = &avt3_ctrl_ops;
			config.id = ctrl_mapping->id;
			config.name = ctrl_mapping->attr.name;
			config.type = ctrl_mapping->type;
			config.flags = ctrl_mapping->flags;

			if (ctrl_mapping->avt_flags & AVT_CTRL_STREAM_ENABLED)
			{
				config.flags |= V4L2_CTRL_FLAG_GRABBED;
			}


			switch (ctrl_mapping->type)
			{
				case V4L2_CTRL_TYPE_MENU:
					config.min = ctrl_mapping->min_value;
					config.menu_skip_mask = 0;
					config.max = ctrl_mapping->max_value;
					config.qmenu = ctrl_mapping->qmenu;
					config.def = qectrl.default_value;
					break;
				case V4L2_CTRL_TYPE_BOOLEAN:
					config.min = 0;
					config.max = 1;
					config.step = 1;
					break;
				default:
					break;
			}


			sensor->avt3_ctrl_cfg[i] = config;

			ctrl = v4l2_ctrl_new_custom(&sensor->v4l2_ctrl_hdl,&config,ctrl_mapping);

			if (ctrl != NULL)
			{
				switch (ctrl->id)
				{
					case V4L2_CID_TRIGGER_MODE:
						sensor->avt_trigger_status.trigger_mode_enabled = ctrl->val;
						avt3_update_sw_ctrl_state(sensor);
						break;
					case V4L2_CID_TRIGGER_SOURCE:
						sensor->avt_trigger_status.trigger_source = ctrl->val;
						avt3_update_sw_ctrl_state(sensor);
						break;
					case V4L2_CID_TRIGGER_ACTIVATION:
						sensor->avt_trigger_status.trigger_activation = ctrl->val;
						break;
					case V4L2_CID_TRIGGER_SOFTWARE:
						avt3_update_sw_ctrl_state(sensor);
					default:
						break;
				}
			}
		}
		else
		{
			ctrl = v4l2_ctrl_new_std(&sensor->v4l2_ctrl_hdl,
                                     &avt3_ctrl_ops,
 									sensor->avt3_ctrl_cfg[i].id,
 									sensor->avt3_ctrl_cfg[i].min,
 									sensor->avt3_ctrl_cfg[i].max,
 									sensor->avt3_ctrl_cfg[i].step,
 									sensor->avt3_ctrl_cfg[i].def);
		}
		//		v4l2_ctrl_new_std(&priv->hdl, &ov6550_ctrl_ops,
		//						V4L2_CID_VFLIP, 0, 1, 1, 0);
		//		v4l2_ctrl_new_std(&priv->hdl, &ov6550_ctrl_ops,
		//						V4L2_CID_HFLIP, 0, 1, 1, 0);
		//		priv->autogain = v4l2_ctrl_new_std(&priv->hdl, &ov6550_ctrl_ops,
		//						V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
		//		priv->gain = v4l2_ctrl_new_std(&priv->hdl, &ov6550_ctrl_ops,
		//						V4L2_CID_GAIN, 0, 0x3f, 1, DEF_GAIN);
		//		priv->autowb = v4l2_ctrl_new_std(&priv->hdl, &ov6550_ctrl_ops,
		//						V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);

		//	}
		if (ctrl == NULL)
		{
			avt_err(&sensor->sd, "Failed to init %s ctrl %d 0x%08x\n",
					sensor->avt3_ctrl_cfg[i].name,
					sensor->v4l2_ctrl_hdl.error, sensor->v4l2_ctrl_hdl.error);

            //Clear error
			sensor->v4l2_ctrl_hdl.error = 0;
			continue;
		}

		sensor->avt3_ctrls[i] = ctrl;
		i++;
	}

	return ret;
free_ctrls:
	v4l2_ctrl_handler_free(&sensor->v4l2_ctrl_hdl);
	return ret;
}

/* Implementierung von imx8-media-cap:
static int capture_enum_framesizes(struct file *file, void *fh,
				   struct v4l2_frmsizeenum *fsize)
{
	struct capture_priv *priv = video_drvdata(file);
	const struct imx_media_pixfmt *cc;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.pad = priv->src_sd_pad,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	cc = imx_media_find_pixel_format(fsize->pixel_format, PIXFMT_SEL_ANY);
	if (!cc)
		return -EINVAL;

	fse.code = cc->codes ? cc->codes[0] : 0;

	ret = v4l2_subdev_call(priv->src_sd, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return ret;

	if (fse.min_width == fse.max_width &&
		fse.min_height == fse.max_height) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = fse.min_width;
		fsize->discrete.height = fse.min_height;
	} else {
		fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
		fsize->stepwise.min_width = fse.min_width;
		fsize->stepwise.max_width = fse.max_width;
		fsize->stepwise.min_height = fse.min_height;
		fsize->stepwise.max_height = fse.max_height;
		fsize->stepwise.step_width = 1;
		fsize->stepwise.step_height = 1;
	}

	return 0;
}

ToDo: Min und Max verschiedne zurueckgeben!!
*/
static int avt3_pad_ops_enum_frame_size(struct v4l2_subdev *sd,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
										struct v4l2_subdev_state *sd_state,
#else
										struct v4l2_subdev_pad_config *cfg,
#endif
										struct v4l2_subdev_frame_size_enum *fse)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	//	struct i2c_client *client = sensor->i2c_client;

	if (fse->which == V4L2_SUBDEV_FORMAT_TRY)
		avt_dbg(sd, "fse->index %d, fse->which %s", fse->index,
				fse->which == V4L2_SUBDEV_FORMAT_TRY ? "V4L2_SUBDEV_FORMAT_TRY" : "V4L2_SUBDEV_FORMAT_ACTIVE");

	if (fse->pad == 0 && fse->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		avt_info(sd, "se->which == V4L2_SUBDEV_FORMAT_TRY and no pad availble.");
		return -EINVAL;
	}

#ifndef ENABLE_STEPWISE_IMAGE_SIZE
	if (fse->index >= AVT3_NUM_MODES)
	{
		dev_warn(&client->dev, "%s[%d]: fse->index(%d) >= AVT3_NUM_MODES(%d).",
				 __func__, __LINE__, fse->index, AVT3_NUM_MODES);
		return -EINVAL;
	}

	if (avt3_mode_data[fse->index].hact > sensor->max_rect.width ||
		avt3_mode_data[fse->index].vact > sensor->max_rect.height)
	{
		return -EINVAL;
	}

	fse->min_width = avt3_mode_data[0].vact;
	fse->max_width = avt3_mode_data[fse->index].hact;
	fse->min_height = avt3_mode_data[0].vact;
	fse->max_height = avt3_mode_data[fse->index].vact;

//	v4l2_dbg(2, debug, sd, "%s[%d]: fse->index %d, fse->min_width %d, fse->max_width %d, fse->min_height %d, fse->max_height %d",
//			 __func__, __LINE__, fse->index,
//		 fse->min_width, fse->max_width, fse->min_height, fse->max_height);

//		dev_info(&client->dev, "%s[%d]: fse->index %d, fse->min_width %d, fse->max_width %d, fse->min_height %d, fse->max_height %d",
//			__func__, __LINE__, fse->index,
//			fse->min_width, fse->max_width, fse->min_height, fse->max_height);
#else
	if (fse->index >= 1)
	{
		avt_info(&sensor->sd, "fse->index(%d) >= 1.", fse->index);
		return -EINVAL;
	}
	fse->min_width = sensor->min_rect.width;   // avt3_mode_data[0].vact;
	fse->max_width = sensor->max_rect.width;   // avt3_mode_data[fse->index].hact;
	fse->min_height = sensor->min_rect.height; // avt3_mode_data[0].vact;
	fse->max_height = sensor->max_rect.height; // avt3_mode_data[fse->index].vact;

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
	//	struct i2c_client *client = sensor->i2c_client;
	int i; //, j, count;
	int f,idx;
		   //	int ret = 0;

	if (fie->pad != 0)
	{
		avt_err(sd, "no pad availble. fie->index %d, fie->pad %d, fie->code 0x%04X, fie->width %d, fie->height %d",
				fie->index, fie->pad, fie->code, fie->width, fie->height);
		return -EINVAL;
	}

	if (fie->index >= AVT3_NUM_FRAMERATES)
	{
		avt_info(sd, "fie->index >= AVT3_NUM_FRAMERATES fie->index %d, AVT3_NUM_FRAMERATES %d, fie->pad %d, fie->code 0x%04X, fie->width %d, fie->height %d",
				 fie->index, AVT3_NUM_FRAMERATES, fie->pad, fie->code, fie->width, fie->height);
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

#if 0
	if (fie->width == 0 || fie->height == 0 || fie->code == 0)
	{
		dev_warn(&client->dev, "%s[%d]: Please assign pixel format, width and height.", __func__, __LINE__);
		return -EINVAL;
	}
#endif
	// TODO: set valid values for current mode
	idx = 0;
	for (f = 0;f < AVT3_NUM_FRAMERATES;f++)
	{
		if (avt3_framerates[f] != 0)
		{
			if (idx == fie->index)
			{
				fie->interval.numerator = 1000;
				fie->interval.denominator = 1000 * avt3_framerates[f];
				return 0;
			}
			idx++;
		}
	}

	return -EINVAL;
}

static int avt3_video_ops_g_frame_interval(struct v4l2_subdev *sd,
										   struct v4l2_subdev_frame_interval *fi)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	//	struct i2c_client *client = sensor->i2c_client;

	fi->interval = sensor->frame_interval;
	avt_dbg(sd, "sensor->frame_interval.denom %u, sensor->frame_interval.num %u, sensor->current_fr %d (%d), fi->num %d fi->denom %u",
			sensor->frame_interval.denominator, sensor->frame_interval.numerator,
			sensor->current_fr, AVT3_NUM_FRAMERATES,
			fi->interval.numerator, fi->interval.denominator);
	return 0;
}

static int avt3_video_ops_s_frame_interval(struct v4l2_subdev *sd,
										   struct v4l2_subdev_frame_interval *fi)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	int fps_idx;
	int ret = 0;
	const u64 factor = 1000000L;
	u64 framerate_req,framerate_min,framerate_max;


	avt_dbg(sd, "fie->num %d fie->denom %d",
			fi->interval.numerator, fi->interval.denominator);

	MUTEX_LOCK(&sensor->lock);
	if (sensor->is_streaming)
	{
		ret = -EBUSY;
		goto out;
	}


	ret = regmap_bulk_read(sensor->regmap64,
   						sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_MIN_64R,
                           &framerate_min, 1);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
        // goto err_out;
	}

	ret = regmap_bulk_read(sensor->regmap64,
   						sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_MAX_64R,
                           &framerate_max, 1);

	if (ret < 0)
	{
		avt_err(sd, "regmap_read failed (%d)\n", ret);
        // goto err_out;
	}

	if (fi->interval.numerator == 0)
		fi->interval.numerator = 1;

	framerate_req = (fi->interval.denominator * factor) / fi->interval.numerator;
	framerate_req = clamp(framerate_req,framerate_min,framerate_max);

	fi->interval.denominator = (framerate_req * fi->interval.numerator) / factor;

    // If the denominator and minimal framerate is not zero, try to increase the numerator by 1000
	while (fi->interval.denominator == 0 && framerate_min > 0 && fi->interval.numerator < factor)
	{
		fi->interval.numerator *= 1000;
		fi->interval.denominator = (framerate_req * fi->interval.numerator) / factor;
	}

	sensor->current_fr = AVT3_NUM_FRAMERATES;
	sensor->frame_interval = fi->interval;

	avt_dbg(sd, "set fie->num %d fie->denom %d",
			fi->interval.numerator, fi->interval.denominator);

out:
	MUTEX_UNLOCK(&sensor->lock);

	avt_dbg(&sensor->sd, "- fie->num %d fie->denom %d --> idx sensor->current_fr %d",
			fi->interval.numerator, fi->interval.denominator, sensor->current_fr);
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
		//	return -EINVAL;
	}
#else
	if (NULL == cfg)
	{
		dev_warn(&client->dev, "%s[%d]: cfg == NULL", __func__, __LINE__);
		//	return -EINVAL;
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

#if 0
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 6, 0))
static int v4l2_subdev_video_ops_g_mbus_config(struct v4l2_subdev *sd,
												unsigned int pad,
											   struct v4l2_mbus_config *cfg)
{

//	struct avt3_dev *sensor = to_avt3_dev(sd);
//	struct i2c_client *client = sensor->i2c_client;

	v4l2_dbg(2, debug, sd, "%s[%d]",
			 __func__, __LINE__);

	cfg->type = V4L2_MBUS_CSI2_DPHY;

	//ToDo MH: check correct clock modes for alvium cam
	cfg->flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK; //V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK; //
	cfg->flags |= V4L2_MBUS_CSI2_LANES;

	v4l2_dbg(2, debug, sd, "%s[%d]: mbus type code %d, mbus flags 0x%02x",
			 __func__, __LINE__, (int)cfg->type, (int)cfg->flags);
//	dev_info(&client->dev, "%s[%d]: mbus type code %d, mbus flags 0x%02x",
//			 __func__, __LINE__, (int)cfg->type, (int)cfg->flags);

	return 0;
}
#endif
#endif

static void avt3_controls_stream_grab(struct avt3_dev *camera,bool grabbed)
{
	int i;

	for (i = 0;i < AVT_MAX_CTRLS;i++)
	{
		struct v4l2_ctrl *ctrl = camera->avt3_ctrls[i];

		if (ctrl && ctrl->priv)
		{
			const struct avt_ctrl_mapping * const ctrl_mapping = ctrl->priv;

			if (ctrl_mapping->avt_flags & AVT_CTRL_STREAM_DISABLED)
			{
				__v4l2_ctrl_grab(ctrl,grabbed);
			}
			else if (ctrl_mapping->avt_flags & AVT_CTRL_STREAM_ENABLED)
			{
				__v4l2_ctrl_grab(ctrl,!grabbed);
			}
		}
	}
}

static int avt3_video_ops_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	struct avt_ctrl ct;
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

	MUTEX_LOCK(&sensor->lock);

	CLEAR(ct);
	if (!enable && sensor->is_streaming)
	{
		ct.id = V4L2_AV_CSI2_STREAMOFF_W;
		ct.value0 = 1;
		ret = avt3_ctrl_send(sensor->i2c_client, &ct);
		sensor->is_streaming = false;

		// ToDo: eventually wait until cam has stopped streaming
	}

	if (enable && !sensor->is_streaming)
	{
		struct v4l2_ext_control vc;
		struct v4l2_ctrl *trigger_mode_ctrl,*trigger_selection_ctrl,*trigger_activation_ctrl;

		u64 u64FrMin = 0;
		u64 u64FrMax = 0;

		u32 xoffs;
		u32 yoffs;

		xoffs = sensor->curr_rect.left;
		yoffs = sensor->curr_rect.top;

		dev_err(&client->dev, "%s[%d]: active area +%d:+%d %d x %d (max %d x %d, min %d x %d), code 0x%04X, HFLIP_W %d, VFLIP_W %d",
				__func__, __LINE__,
				xoffs, yoffs,
				sensor->mbus_framefmt.width, sensor->mbus_framefmt.height,
				sensor->max_rect.width, sensor->max_rect.height,
				sensor->min_rect.width, sensor->min_rect.height,
				sensor->mbus_framefmt.code,
				sensor->hflip, sensor->vflip);

		ct.id = V4L2_AV_CSI2_PIXELFORMAT_W;
		ct.value0 = sensor->mbus_framefmt.code;
		ret = avt3_ctrl_send(sensor->i2c_client, &ct);

		ct.id = V4L2_AV_CSI2_OFFSET_X_W;
		ct.value0 = xoffs;
		ret = avt3_ctrl_send(sensor->i2c_client, &ct);

		ct.id = V4L2_AV_CSI2_WIDTH_W;
		ct.value0 = sensor->mbus_framefmt.width;
		ret = avt3_ctrl_send(sensor->i2c_client, &ct);

		ct.id = V4L2_AV_CSI2_OFFSET_Y_W;
		ct.value0 = yoffs;
		ret = avt3_ctrl_send(sensor->i2c_client, &ct);

		ct.id = V4L2_AV_CSI2_HEIGHT_W;
		ct.value0 = sensor->mbus_framefmt.height;
		ret = avt3_ctrl_send(sensor->i2c_client, &ct);

		ct.id = V4L2_AV_CSI2_VFLIP_W;
		ct.value0 = sensor->vflip;
		ret = avt3_ctrl_send(sensor->i2c_client, &ct);

		ct.id = V4L2_AV_CSI2_HFLIP_W;
		ct.value0 = sensor->hflip;
		ret = avt3_ctrl_send(sensor->i2c_client, &ct);

		/* 0 == manual, 2 == set continous auto exposure */
		ret = bcrm_regmap_write(sensor, sensor->regmap8,
								sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_AUTO_8RW, sensor->exposure_mode);

		if (ret < 0)
		{
			dev_err(&client->dev, "%s[%d]: BCRM_EXPOSURE_AUTO_8RW: bcrm_regmap_write failed (%d)\n",
					__func__, __LINE__,
					ret);
			goto out;
		}

		ret = bcrm_regmap_write64(sensor, sensor->regmap64,
								  sensor->cci_reg.reg.bcrm_addr + BCRM_EXPOSURE_TIME_64RW, sensor->exposure_time);

		if (ret < 0)
		{
			dev_err(&client->dev, "%s[%d]: BCRM_EXPOSURE_TIME_64RW: i2c write failed (%d)\n",
					__func__, __LINE__,
					ret);
			goto out;
		}

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

			/* Enable manual frame rate */
			ret = bcrm_regmap_write(sensor, sensor->regmap8,
									sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW, 1);
			if (ret < 0)
			{
				dev_err(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW: bcrm_regmap_write failed (%d)\n",
						__func__, __LINE__,
						ret);
				goto out;
			}
		}

		dev_info(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE: sensor->frame_interval.numerator %u, sensor->frame_interval.denominator %u\n",
				 __func__, __LINE__,
				 sensor->frame_interval.numerator, sensor->frame_interval.denominator);

		/* Save new frame rate to camera register */
		vc.value64 = (sensor->frame_interval.denominator * 1000000) / sensor->frame_interval.numerator;

		dev_info(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_64RW (min: %llu req: %llu max: %llu) uHz\n",
				 __func__, __LINE__,
				 u64FrMin, vc.value64, u64FrMax);

#ifdef AUTO_ACQUISITION_FRAME_RATE
		if (u64FrMin > vc.value64 || u64FrMax < vc.value64)
		{
			dev_err(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_64RW out of bounds (min: %llu req: %llu max: %llu) uHz\n",
					__func__, __LINE__,
					u64FrMin, vc.value64, u64FrMax);

			ret = -EINVAL;
			goto out;
		}
#else
		if (u64FrMin > vc.value64)
		{
			dev_err(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_64RW out of bounds (min: %llu req: %llu max: %llu) uHz\n",
					__func__, __LINE__,
					u64FrMin, vc.value64, u64FrMax);

			vc.value64 = u64FrMin;
			//ret = -EINVAL;
			//goto out;
		}

		if (u64FrMax < vc.value64)
		{
			dev_err(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_64RW out of bounds (min: %llu req: %llu max: %llu) uHz\n",
					__func__, __LINE__,
					u64FrMin, vc.value64, u64FrMax);

			vc.value64 = u64FrMax;
			//ret = -EINVAL;
			//goto out;
		}
#endif
		ret = bcrm_regmap_write64(sensor, sensor->regmap64,
								  sensor->cci_reg.reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_64RW,
								  vc.value64);
		if (ret < 0)
		{
			dev_err(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_64RW: i2c write failed (%d)\n",
					__func__, __LINE__,
					ret);
			goto out;
			//} else {
			// dev_info(&client->dev, "%s[%d]: BCRM_ACQUISITION_FRAME_RATE_64RW set to %12llu\n", __func__, __LINE__, vc.value64);
		}

		if (debug >= 2)
			bcrm_dump(client);

		/* start streaming */
		ct.id = V4L2_AV_CSI2_STREAMON_W;
		ct.value0 = 1;
		ret = avt3_ctrl_send(client, &ct);

		// ToDo: probably it's better to check the status here. but this conflicts with the workaround for imx8mp delayed start
		if (!ret)
			sensor->is_streaming = enable;
	}

	avt3_controls_stream_grab(sensor,enable);

out:
	MUTEX_UNLOCK(&sensor->lock);

	return ret;
}

#if 0

int avt3_core_ops_g_chip_ident(struct v4l2_subdev *sd,
									  struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//	chip->match.name.type.__u32 ident;       /* chip identifier as specified in <media/v4l2-chip-ident.h> */
	//	__u32 revision;
	chip->ident = 0x0815;
	chip->revision = 0x55aa;

	dev_info(&client->dev, "%s[%d]+ %s", __func__, __LINE__, __FILE__);
	return 0;
}
#endif

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
	//__u64 val64 = 0;

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

	// v4l2_dbg(2, debug, sd, "%s[%d]: %s", __func__, __LINE__, __FILE__);
	dev_info(&client->dev, "%s[%d]: reg 0x%04llX, size %u",
			 __func__, __LINE__, reg->reg, reg->size);

	return 0;
}

// long avt3_core_ops_command(struct v4l2_subdev *sd, unsigned int cmd, void *arg) {
//
////	int ret = -ENOTTY;
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
////	struct avt3_dev *sensor = to_avt3_dev(sd);
////	struct v4l2_capability *cap = arg;
//
//	dev_info(&client->dev,  "%s[%d]:  cmd 0x%08x %d %s",
//		__func__, __LINE__, cmd, cmd & 0xff, __FILE__);
//
//	dump_stack();
//
//	return 0;
//}

static void set_channel_pending_trigger(struct v4l2_subdev *sd)
{
	//	struct tegra_channel *tch;
	struct media_pad *pad_csi, *pad_vi;
	struct v4l2_subdev *sd_csi, *sd_vi;
	struct video_device *vdev_vi;

	if (!sd->entity.pads)
		return;

	pad_csi = media_entity_remote_pad(&sd->entity.pads[0]);
	sd_csi = media_entity_to_v4l2_subdev(pad_csi->entity);
	pad_vi = media_entity_remote_pad(&sd_csi->entity.pads[1]);
	sd_vi = media_entity_to_v4l2_subdev(pad_vi->entity);
	vdev_vi = media_entity_to_video_device(pad_vi->entity);
	//	tch = video_get_drvdata(vdev_vi);

	//	tch->pending_trigger = true;
}

#if 1
long avt3_core_ops_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = -ENOTTY;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct v4l2_capability *cap = arg;

	//	struct avt_csi2_priv *priv = avt_get_priv(sd);
	struct v4l2_i2c *i2c_reg;
	struct v4l2_csi_driver_info *info;
	struct v4l2_csi_config *config;
	//	struct v4l2_queryctrl 		*queryctrl;
	//	struct v4l2_query_ext_ctrl 	*queryctrlext;
	//	struct v4l2_querymenu 		*querymenu;
	//	struct v4l2_selection		*v4l2_sel;
	//	struct v4l2_fmtdesc			*v4l2fmtdesc;
	//	struct v4l2_ext_controls	*v4l2_xctl;
	struct v4l2_streamparm *v4l2_srmparm = arg;
	//	struct v4l2_control			*v4l2_ctl;
	//	struct v4l2_subdev_capability 		*v4l2_sd_capability;
	//	struct v4l2_subdev_format 			*v4l2_sd_format;
	//	struct v4l2_subdev_frame_interval 	*v4l2_sd_frame_interval;
	//	struct v4l2_subdev_mbus_code_enum 	*v4l2_sd_mbus_code_enum;
	//	struct v4l2_subdev_frame_size_enum 	*v4l2_sd_frame_size_enum;
	//	struct v4l2_subdev_crop 			*v4l2_sd_crop;
	//	struct v4l2_subdev_selection 		*v4l2_sd_selection;
	//	struct v4l2_subdev_frame_interval_enum 	*v4l2_sd_frame_interval_enum;

	//	struct v4l2_dbg_register
	//	struct v4l2_frmsizeenum
	//	struct v4l2_frmivalenum

	//	uint32_t clk;
	char *i2c_reg_buf;
	//	struct v4l2_gencp_buffer_sizes *gencp_buf_sz;

	avt_dbg(sd, "cmd 0x%08x %d %s", cmd, cmd & 0xff, __FILE__);

	//	dump_stack();

	switch (cmd)
	{
#if 1
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
#endif
	case VIDIOC_G_PARM:
		// V4L2Viewer landet wirklich hier
		if (V4L2_TYPE_IS_CAPTURE(v4l2_srmparm->type))
		{
			v4l2_srmparm->parm.capture.timeperframe = sensor->frame_interval;
			dev_info(&client->dev, "%s[%d] cmd VIDIOC_G_PARM type %d, timeperframe %u/%u", __func__, __LINE__,
					 v4l2_srmparm->type,
					 v4l2_srmparm->parm.capture.timeperframe.denominator, v4l2_srmparm->parm.capture.timeperframe.numerator);
		}
		ret = 0;
		break;

	case VIDIOC_S_PARM:
		// V4L2Viewer landet ggf. hier
		dev_info(&client->dev, "%s[%d] cmd VIDIOC_S_PARM type %d, timeperframe %u/%u", __func__, __LINE__,
				 v4l2_srmparm->type,
				 v4l2_srmparm->parm.capture.timeperframe.denominator, v4l2_srmparm->parm.capture.timeperframe.numerator);

		if (V4L2_TYPE_IS_CAPTURE(v4l2_srmparm->type))
		{
			int fps_idx = 0;
			sensor->frame_interval = v4l2_srmparm->parm.capture.timeperframe;

			do
			{
				if ((v4l2_srmparm->parm.capture.timeperframe.numerator == 1) && (v4l2_srmparm->parm.capture.timeperframe.denominator == avt3_framerates[fps_idx]))
				{
					sensor->frame_interval = v4l2_srmparm->parm.capture.timeperframe;
					sensor->current_fr = fps_idx;
					ret = 0;
					break;
				}
				if ((v4l2_srmparm->parm.capture.timeperframe.numerator == 1000) && (v4l2_srmparm->parm.capture.timeperframe.denominator == avt3_framerates[fps_idx] * 1000))
				{
					// sensor->frame_interval.numerator = fi->interval.numerator/1000;
					// sensor->frame_interval.denominator = fi->interval.denominator/1000;
					sensor->frame_interval = v4l2_srmparm->parm.capture.timeperframe;
					sensor->current_fr = fps_idx;
					ret = 0;
					break;
				}
				fps_idx++;
			} while (fps_idx < AVT3_NUM_FRAMERATES);
			if (fps_idx == AVT3_NUM_FRAMERATES)
			{
				// ToDo: set correct sensor->current_fr
				sensor->current_fr = fps_idx;
				dev_err(&client->dev, "%s[%d] fps_idx == AVT3_NUM_FRAMERATES: set correct sensor->current_fr", __func__, __LINE__);
			}
		}

		ret = 0;
		break;

	case VIDIOC_G_EXT_CTRLS:
		dev_info(&client->dev, "%s[%d] cmd VIDIOC_G_EXT_CTRLS - not implemented here", __func__, __LINE__);
		ret = 0;
		break;

	case VIDIOC_S_EXT_CTRLS:
		dev_warn(&client->dev, "%s[%d] cmd VIDIOC_S_EXT_CTRLS - not implemented here", __func__, __LINE__);
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

	case VIDIOC_CROPCAP:
	{
		struct v4l2_cropcap *cropcap = (struct v4l2_cropcap *)arg;
		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_CROPCAP type %d",
				 __func__, __LINE__, cropcap->type);
		if (!V4L2_TYPE_IS_CAPTURE(cropcap->type))
		{
			//				 != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
			//				cropcap->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			//				dev_err(&client->dev,  "%s[%d]: cmd VIDIOC_CROPCAP - wrong type %d",
			//					__func__, __LINE__, cropcap->type);
			ret = -EINVAL;
			break;
		}
		cropcap->bounds = sensor->curr_rect;
		cropcap->defrect = sensor->max_rect;
		ret = 0;
		break;
	}
	case VIDIOC_G_CROP:
	{
		struct v4l2_crop *crop = (struct v4l2_crop *)arg;
		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_G_CROP type %d, %d,%d x %d,%d",
				 __func__, __LINE__,
				 crop->type, crop->c.left, crop->c.top, crop->c.width, crop->c.height);
		if (!V4L2_TYPE_IS_CAPTURE(crop->type))
		{
			ret = -EINVAL;
			break;
		}
		crop->c = sensor->curr_rect;
		ret = 0;
		break;
	}

	case VIDIOC_S_CROP:
	{
		struct v4l2_crop *crop = (struct v4l2_crop *)arg;

		if (sensor->is_streaming)
		{
			ret = -EBUSY;
			break;
		}

		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_S_CROP type %d, %d,%d - %d,%d", __func__, __LINE__,
				 crop->type, crop->c.left, crop->c.top, crop->c.width, crop->c.height);

		if (!V4L2_TYPE_IS_CAPTURE(crop->type))
		{
			ret = -EINVAL;
			break;
		}

		if (sensor->max_rect.width < crop->c.left + crop->c.width || sensor->max_rect.height < crop->c.top + crop->c.height)
		{
			dev_err(&client->dev, "%s[%d]: cmd VIDIOC_S_CROP - invalid crop rectangle max: %d,%d x %d %d crop->c: %d,%d x %d %d",
					__func__, __LINE__,
					sensor->max_rect.left, sensor->max_rect.top,
					sensor->max_rect.width, sensor->max_rect.height,
					crop->c.left, crop->c.top,
					crop->c.width, crop->c.height);
			ret = -EINVAL;
			break;
		}

		sensor->curr_rect = crop->c;

		ret = regmap_write(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_OFFSET_X_32RW, sensor->curr_rect.left);
		ret = regmap_write(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_OFFSET_Y_32RW, sensor->curr_rect.top);

		ret = regmap_write(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_WIDTH_32RW, sensor->curr_rect.width);
		ret = regmap_write(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_IMG_HEIGHT_32RW, sensor->curr_rect.height);

		ret = 0;
		break;
	}

	case VIDIOC_G_SELECTION:
	{
		struct v4l2_selection *v4l2_sel = (struct v4l2_selection *)arg;
		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_G_SELECTION arg 0x%016llx", __func__, __LINE__, (u64)v4l2_sel);

		if (!V4L2_TYPE_IS_CAPTURE(v4l2_sel->type))
		{
			dev_err(&client->dev, "%s[%d]: cmd VIDIOC_G_SELECTION - wrong type %d", __func__, __LINE__, v4l2_sel->type);
			ret = -EINVAL;
			break;
		}

		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_G_SELECTION type %d, target %d",
				 __func__, __LINE__,
				 v4l2_sel->type,
				 v4l2_sel->target);

		ret = 0;
		break;
	}

	case VIDIOC_S_SELECTION:
	{
		struct v4l2_selection *v4l2_sel = (struct v4l2_selection *)arg;
		if (!V4L2_TYPE_IS_CAPTURE(v4l2_sel->type))
		{
			dev_err(&client->dev, "%s[%d]: cmd VIDIOC_G_SELECTION - wrong type %d", __func__, __LINE__, v4l2_sel->type);
			ret = -EINVAL;
			break;
		}
		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_S_SELECTION type %d, target %d",
				 __func__, __LINE__,
				 v4l2_sel->type,
				 v4l2_sel->target);

		switch (v4l2_sel->target)
		{
		case V4L2_SEL_TGT_CROP_BOUNDS:
		case V4L2_SEL_TGT_CROP_DEFAULT:
			sensor->curr_rect = v4l2_sel->r;
			break;
		case V4L2_SEL_TGT_CROP:
			sensor->curr_rect = v4l2_sel->r;
			break;
		case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		case V4L2_SEL_TGT_COMPOSE_DEFAULT:
			sensor->curr_rect = v4l2_sel->r;
			break;
		default:
			ret = -EINVAL;
			break;
		}

		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_S_SELECTION type %d, target %d, %d,%d - %d,%d",
				 __func__, __LINE__,
				 v4l2_sel->type,
				 v4l2_sel->target,
				 v4l2_sel->r.left, v4l2_sel->r.top, v4l2_sel->r.width, v4l2_sel->r.height);

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
//		ret = i2c_read(client, i2c_reg->register_address, i2c_reg->register_size,
//					   i2c_reg->num_bytes, i2c_reg_buf);

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
//		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_W_I2C", __func__, __LINE__);
		i2c_reg = (struct v4l2_i2c *)arg;

		i2c_reg_buf = kzalloc(i2c_reg->num_bytes, GFP_KERNEL);
		if (!i2c_reg_buf)
			return -ENOMEM;

		ret = copy_from_user(i2c_reg_buf, (char *)i2c_reg->ptr_buffer, i2c_reg->num_bytes);

//		dev_info(&client->dev, "%s[%d]: cmd VIDIOC_W_I2C i2c_reg->reg 0x%04x, i2c_reg->register_size %d, i2c_reg->num_bytes %d",
//				 __func__, __LINE__,
//				 i2c_reg->register_address, i2c_reg->register_size, i2c_reg->num_bytes);

		/* TODO: check count, size and endianess!! */
		ret = regmap_bulk_write(sensor->regmap8, i2c_reg->register_address, i2c_reg_buf, i2c_reg->num_bytes);

		if (ret < 0)
		{
			dev_err(&client->dev, "%s[%d]: i2c write failed (%d), bytes written = %d\n",
					__func__, __LINE__, ret, i2c_reg->num_bytes);
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
#endif

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
	//.g_chip_ident = avt3_core_ops_g_chip_ident,
	.s_power = avt3_core_ops_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.ioctl = avt3_core_ops_ioctl,
	//	.command = avt3_core_ops_command,
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
	// struct avt3_dev *sensor = to_avt3_dev(sd);
	// struct i2c_client *client = sensor->i2c_client;

	v4l2_dbg(2, debug, sd, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);
	return 0;
}

int v4l2_subdev_video_ops_s_mbus_config(struct v4l2_subdev *sd,
										const struct v4l2_mbus_config *cfg)
{
	// struct avt3_dev *sensor = to_avt3_dev(sd);
	// struct i2c_client *client = sensor->i2c_client;
	v4l2_dbg(2, debug, sd, "%s[%d]: %s", __func__, __LINE__, __FILE__);
	return 0;
}

#ifndef ZYNQMP

//#if !defined(CONFIG_ARCH_ZYNQMP)
// struct v4l2_captureparm {
//	__u32		   capability;	  /*  Supported modes */
//	__u32		   capturemode;	  /*  Current mode */
//	struct v4l2_fract  timeperframe;  /*  Time per frame in seconds */
//	__u32		   extendedmode;  /*  Driver-specific extensions */
//	__u32  			readbuffers;   /*  # of buffers for read */
//	__u32		   reserved[4];
//};
//#endif

int avt3_video_ops_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parm)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	// struct i2c_client *client = sensor->i2c_client;
	dev_info(&sensor->i2c_client->dev, "%s[%d]: %s", __func__, __LINE__, __FILE__);

	if (!parm)
		return -EINVAL;

	v4l2_dbg(2, debug, sd, "%s[%d]: parm->type %d", __func__, __LINE__, parm->type);

	if (!V4L2_TYPE_IS_CAPTURE(parm->type))
	{
		return -EINVAL;
	}

	memcpy(&parm->parm.capture, &sensor->streamcap, sizeof(struct v4l2_captureparm));

	//	parm->parm.capture.readbuffers = 1;
	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME | V4L2_MODE_HIGHQUALITY;
	parm->parm.capture.timeperframe = sensor->frame_interval;
	/* return latest format as has been set by avt3_video_ops_g_parm */

	return 0;
}

int avt3_video_ops_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parm)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	// struct i2c_client *client = sensor->i2c_client;
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
#endif

static const struct v4l2_subdev_video_ops avt3_video_ops = {
	.g_frame_interval = avt3_video_ops_g_frame_interval,
	.s_frame_interval = avt3_video_ops_s_frame_interval,
	.s_stream = avt3_video_ops_s_stream,
	.querystd = avt3_video_ops_querystd,
#if !defined(CONFIG_ARCH_ZYNQMP)
	.g_parm = avt3_video_ops_g_parm,
	.s_parm = avt3_video_ops_s_parm,
#endif
#if ((LINUX_VERSION_CODE) < (KERNEL_VERSION(5, 6, 0)))
	.g_mbus_config = v4l2_subdev_video_ops_g_mbus_config, // avt3_g_mbus_config,
	.s_mbus_config = v4l2_subdev_video_ops_s_mbus_config,
#endif
};

int avt3_pad_ops_get_selection(struct v4l2_subdev *sd,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
							   struct v4l2_subdev_state *sd_state,
#else
							   struct v4l2_subdev_pad_config *cfg,
#endif
							   struct v4l2_subdev_selection *sel)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = -EINVAL;

	dev_info(&client->dev, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);

	switch (sel->target)
	{
	/* Current cropping area */
	case V4L2_SEL_TGT_CROP:

	/* Cropping bounds */
	case V4L2_SEL_TGT_CROP_BOUNDS:

	/* Composing bounds */
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:

	/* Default composing area */
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:

	/* Current composing area plus all padding pixels */
	case V4L2_SEL_TGT_COMPOSE_PADDED:

	/* Current composing area */
	case V4L2_SEL_TGT_COMPOSE:
		sel->r = sensor->curr_rect;
		ret = 0;
		break;

	/* Default cropping area */
	case V4L2_SEL_TGT_CROP_DEFAULT:
	/* Native frame size */
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = sensor->max_rect;
		ret = 0;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return 0;
}

int avt3_pad_ops_set_selection(struct v4l2_subdev *sd,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 14, 0))
							   struct v4l2_subdev_state *sd_state,
#else
							   struct v4l2_subdev_pad_config *cfg,
#endif
							   struct v4l2_subdev_selection *sel)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = -EINVAL;

	if (V4L2_SUBDEV_FORMAT_ACTIVE == sel->which && true == sensor->is_streaming)
		return -EBUSY;

	dev_info(&client->dev, "%s[%d]: pad %d, target %s, which %s, flags %d, %d.%d - %d.%d",
			 __func__, __LINE__,
			 sel->pad,
			 (sel->target == V4L2_SEL_TGT_CROP) ? "V4L2_SEL_TGT_CROP" : ((sel->target == V4L2_SEL_TGT_CROP_DEFAULT) ? "V4L2_SEL_TGT_CROP_DEFAULT" : ((sel->target == V4L2_SEL_TGT_CROP_BOUNDS) ? "V4L2_SEL_TGT_CROP_BOUNDS" : ((sel->target == V4L2_SEL_TGT_NATIVE_SIZE) ? "V4L2_SEL_TGT_NATIVE_SIZE" : ((sel->target == V4L2_SEL_TGT_COMPOSE) ? "V4L2_SEL_TGT_COMPOSE" : ((sel->target == V4L2_SEL_TGT_COMPOSE_DEFAULT) ? "V4L2_SEL_TGT_COMPOSE_DEFAULT" : ((sel->target == V4L2_SEL_TGT_COMPOSE_BOUNDS) ? "V4L2_SEL_TGT_COMPOSE_BOUNDS" : ((sel->target == V4L2_SEL_TGT_COMPOSE_PADDED) ? "V4L2_SEL_TGT_COMPOSE_PADDED" : " unknown V4L2_SEL_TGT_xxx"))))))),
			 (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE) ? "V4L2_SUBDEV_FORMAT_ACTIVE" : "V4L2_SUBDEV_FORMAT_TRY",
			 sel->flags,
			 sel->r.left, sel->r.top,
			 sel->r.width, sel->r.height);

	switch (sel->target)
	{
	/* Current cropping area */
	case V4L2_SEL_TGT_CROP:
	/* Cropping bounds */
	case V4L2_SEL_TGT_CROP_BOUNDS:
	/* Composing bounds */
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	/* Current composing area plus all padding pixels */
	case V4L2_SEL_TGT_COMPOSE_PADDED:
	/* Current composing area */
	case V4L2_SEL_TGT_COMPOSE:
	/* Native frame size - eventually not applicable for a sensor */
	case V4L2_SEL_TGT_NATIVE_SIZE:
	/* Default composing area */
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	/* Default cropping area */
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sensor->curr_rect = sel->r;
		ret = 0;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int avt3_pad_ops_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
								struct v4l2_mbus_frame_desc *fd)
{
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	// int	ret = 0;

	dev_info(&client->dev, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);
	return 0;
}

int avt3_pad_ops_set_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
								struct v4l2_mbus_frame_desc *fd)
{
	// struct avt3_dev *sensor = to_avt3_dev(sd);
	// struct i2c_client *client = sensor->i2c_client;

	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	// int	ret = 0;

	dev_info(&client->dev, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);
	return 0;
}
#ifdef CONFIG_MEDIA_CONTROLLER
int avt3_pad_ops_link_validate(struct v4l2_subdev *sd, struct media_link *link,
							   struct v4l2_subdev_format *source_fmt,
							   struct v4l2_subdev_format *sink_fmt)
{
	// struct avt3_dev *sensor = to_avt3_dev(sd);
	// struct i2c_client *client = sensor->i2c_client;

	v4l2_dbg(2, debug, sd, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);
	return 0;
}
#endif /* CONFIG_MEDIA_CONTROLLER */

static const struct v4l2_subdev_pad_ops avt3_pad_ops = {
	//	.init_cfg = avt3_pad_ops_init_cfg,
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
	//	.get_fwnode_pad = avt3_meo_get_fwnode_pad,
	//	.link_validate = avt3_meo_link_validate,
};

#if 0
static int avt3_set_mipi_clock(struct v4l2_subdev *sd) {
	struct avt3_dev *sensor = to_avt3_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int	ret;

	uint32_t avt_current_clk = 0;
		
	/* Set number of lanes */
	ret = bcrm_regmap_write(sensor, sensor->regmap8,
			sensor->cci_reg.reg.bcrm_addr + BCRM_CSI2_LANE_COUNT_8RW,
			sensor->v4l2_fwnode_ep.bus.mipi_csi2.num_data_lanes);

	ret = bcrm_regmap_write(sensor, sensor->regmap32, sensor->cci_reg.reg.bcrm_addr +
			BCRM_CSI2_CLOCK_32RW, sensor->v4l2_fwnode_ep.link_frequencies[0]);

	ret = regmap_read(sensor->regmap32, sensor->cci_reg.reg.bcrm_addr +
			BCRM_CSI2_CLOCK_32RW, &avt_current_clk);

	dev_info(&client->dev, "%s[%d]: requested csi clock frequency %llu Hz, got %u Hz)\n",
			__func__, __LINE__, sensor->v4l2_fwnode_ep.link_frequencies[0], avt_current_clk);

	if (0 < avt_current_clk)
		sensor->v4l2_fwnode_ep.link_frequencies[0] = avt_current_clk;

	return ret;
}
#endif

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

	//	struct v4l2_subdev_selection sel;

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

	sensor->curr_rect = sensor->max_rect;
	sensor->curr_rect.left = 0;
	sensor->curr_rect.top = 0;

	ret = regmap_write(sensor->regmap8,
					   GENCP_CHANGEMODE_8W,
					   bcm_mode);

	if (ret < 0)
	{
		avt_err(sd, "Failed to set BCM mode: i2c write failed (%d)\n", ret);
		return ret;
	}
	sensor->mode = AVT_BCRM_MODE;

	return 0;
}

static int avt_csi2_check_mipicfg(struct avt3_dev *sensor /*, struct device *dev*/)
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

#ifdef BCRM_HS_THREAD
int avt3_streamon_thread(void *data)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)data;
	struct avt3_dev *sensor = to_avt3_dev(sd);
	// struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	// int sampling_us = sensor->bcrm_handshake_timeout_ms*1000;
	// struct timespec64 next, now, delta;
	//	s64 delay_us;
	//	long	loop_counter = 0;
	//	struct avt_ctrl ct;
	long jiffies = msecs_to_jiffies(5000);
	;

	// v4l2_dbg(2, debug, sd, "%s[%d]+\n", __func__, __LINE__);
	avt_info(sd, "+");

	// ktime_get_ts64(&next);

	do
	{

		ret = down_timeout(&sensor->streamon_sem, jiffies);

		if (0 == ret)
		{
			if (sensor->is_streaming && sensor->phyreset_on_streamon)
			{
				usleep_range(sensor->dphyreset_delay, sensor->dphyreset_delay * 2);
				// if (sensor->phyreset_on_streamon) {
				avt3_dphy_reset(sensor, true);
				avt3_dphy_reset(sensor, false);
				avt_info(sd, "trigger alvium phy reset, sensor->dphyreset_delay %u ret %d",
						 sensor->dphyreset_delay, ret);

				//			avt3_set_mipi_clock(sd);
				//			dev_warn(&client->dev, "%s[%d]: release alvium phy reset", __func__, __LINE__);
			}
			// complete(&sensor->streamon_completion);
			//	continue;
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

	// init_completion(&sensor->streamon_completion);

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

static int bcrm_regmap_write64(struct avt3_dev *sensor,
							   struct regmap *map,
							   unsigned int reg,
							   unsigned long long val)
{

	int ret;

	u32 handshake_val = 0;

	ret = regmap_read(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW, &handshake_val);

	if (ret < 0)
	{
		dev_err(&sensor->i2c_client->dev,"%s[%d]: Reading handshake value failed with: %d\n",__func__, __LINE__,ret);
		return ret;
	}

	ret = regmap_write(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW, handshake_val & ~BCRM_HANDSHAKE_STATUS_MASK); /* reset only handshake status */

	if (ret < 0)
	{
		dev_err(&sensor->i2c_client->dev,"%s[%d]: Clearing handshake status failed with: %d\n",__func__, __LINE__,ret);
		return ret;
	}

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
		// duration_ms = (uint64_t)default_wait_time_ms;
	}

	/* wait for bcrm handshake */
	reinit_completion(&sensor->bcrm_wrhs_completion);

	queue_work(sensor->bcrm_wrhs_queue, &sensor->bcrm_wrhs_work);
	//	dev_info(&sensor->i2c_client->dev, "%s[%d]: queue_work done\n", __func__, __LINE__);

	ret = wait_for_completion_timeout(&sensor->bcrm_wrhs_completion,
									  msecs_to_jiffies(sensor->bcrm_handshake_timeout_ms / 5));

	// If wait_for_completion_timeout returns a positive value, then the handshake was successfully
	// and ret contains the remaining time before the timeout would occur
	if (ret > 0)
	{
		return 0;
	}

	dev_err(&sensor->i2c_client->dev,"%s[%d]: Write handshake timeout\n",__func__, __LINE__);

	return -EIO;
}

static int bcrm_regmap_write(struct avt3_dev *sensor,
							 struct regmap *map,
							 unsigned int reg,
							 unsigned int val)
{

	int ret;
	u32 handshake_val = 0;

	ret = regmap_read(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW, &handshake_val);

	if (ret < 0)
	{
		dev_err(&sensor->i2c_client->dev,"%s[%d]: Reading handshake value failed with: %d\n",__func__, __LINE__,ret);
		return ret;
	}

	ret = regmap_write(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW, handshake_val & ~BCRM_HANDSHAKE_STATUS_MASK); /* reset only handshake status */

	if (ret < 0)
	{
		dev_err(&sensor->i2c_client->dev,"%s[%d]: Clearing handshake status failed with: %d\n",__func__, __LINE__,ret);
		return ret;
	}

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
		// duration_ms = (uint64_t)default_wait_time_ms;

		return ret;
	}

	/* wait for bcrm handshake */
	reinit_completion(&sensor->bcrm_wrhs_completion);

	queue_work(sensor->bcrm_wrhs_queue, &sensor->bcrm_wrhs_work);
	//	dev_info(&sensor->i2c_client->dev, "%s[%d]: queue_work done\n", __func__, __LINE__);

	ret = wait_for_completion_timeout(&sensor->bcrm_wrhs_completion,
									  msecs_to_jiffies(sensor->bcrm_handshake_timeout_ms / 5));

	// If wait_for_completion_timeout returns a positive value, then the handshake was successfully
	// and ret contains the remaining time before the timeout would occur
	if (ret > 0)
	{
		return 0;
	}

	dev_err(&sensor->i2c_client->dev,"%s[%d]: Write handshake timeout\n",__func__, __LINE__);

	return -EIO;
}

static void bcrm_wrhs_work_func(struct work_struct *work)
{
	u32 handshake_val = 0;
	static const int poll_interval_ms = 5;
	int ret = 0;
	int i = 0;

	struct avt3_dev *sensor =
		container_of(work, struct avt3_dev, bcrm_wrhs_work);

	//	dev_info(&sensor->i2c_client->dev, "%s[%d]: workqueue_test: 0x%08X current->pid 0x%08x\n",
	//  	__func__, __LINE__, (u32)work, current->pid );

	do
	{
		ret = regmap_read(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW, &handshake_val);

		if (handshake_val & BCRM_HANDSHAKE_STATUS_MASK)
		{
			ret = regmap_write(sensor->regmap8, sensor->cci_reg.reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW, handshake_val & ~BCRM_HANDSHAKE_STATUS_MASK); /* reset only handshake status */
			break;
		}
		msleep(poll_interval_ms);
		i++;
	} while (i < 300 /*duration_ms <= timeout_ms*/);

	if (i == 300)
		dev_info(&sensor->i2c_client->dev, "%s[%d]: 0x%08llx current->pid 0x%08x %d\n",
				 __func__, __LINE__, (u64)work, current->pid, i);
#if 0
static uint64_t wait_for_bcrm_write_handshake(struct i2c_client *client, uint64_t timeout_ms)
{
	struct avt3_dev *sensor = client_to_avt3_dev(client);

	static const int poll_interval_ms = 10;
	static const int default_wait_time_ms = 50;
	int ret = 0;
	u8 buffer[3] = {0};
	u32 handshake_val = 0;
	bool handshake_valid = false;
	struct timespec64 ts64start, ts64end;

	uint64_t start_time_ms = 0;
	uint64_t duration_ms = 0;

	if (sensor->bcrm_write_handshake)
	{
		ktime_get_real_ts64(&ts64start);
		start_time_ms = (ts64start.tv_sec * (uint64_t)1000) + (ts64start.tv_nsec / 1000000);

		/* We need to poll the handshake register and wait until the camera has processed the data */
		dev_dbg(&client->dev, "%s[%d]:  Wait for 'write done' bit (0x81) ...", __func__, __LINE__);
		do
		{
			msleep(poll_interval_ms);
			/* Read handshake register */
			ret = regmap_read(sensor->regmap8, sensor->cci_reg.bcrm_addr +
				BCRM_WRITE_HANDSHAKE_8RW, &handshake_val);

			ktime_get_real_ts64(&ts64end);
			duration_ms = ((ts64end.tv_sec * (uint64_t)1000) + (ts64end.tv_nsec / 1000000)) - start_time_ms;

			if (ret >= 0)
			{
				/* Check, if handshake bit is set */
				if ((handshake_val & 0x01) == 1)
				{
					/* Handshake set by camera. We should to reset it */
					buffer[0] = (sensor->cci_reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW) >> 8;
					buffer[1] = (sensor->cci_reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW) & 0xff;
					buffer[2] = (handshake_val & 0xFE); /* Reset LSB (handshake bit)*/
					ret = i2c_master_send(client, buffer, sizeof(buffer));

					/* Since the camera needs a few ms for every write access to finish, we need to poll here too */
					dev_dbg(&client->dev, "%s[%d]: Wait for reset of 'write done' bit (0x80) ...",
					__func__, __LINE__);
					do
					{
						msleep(poll_interval_ms);
						/* We need to wait again until the bit is reset */
						ret = regmap_read(sensor->regmap8, sensor->cci_reg.bcrm_addr +
								BCRM_WRITE_HANDSHAKE_8RW, &handshake_val);

						ktime_get_real_ts64(&ts64end);
						duration_ms = ((ts64end.tv_sec * (uint64_t)1000) + (ts64end.tv_nsec / 1000000)) - start_time_ms;

						if (ret >= 0)
						{
							if ((handshake_val & 0x01) == 0) /* Verify write */
							{
								handshake_valid = true;
								dev_dbg(&client->dev, "%s[%d]: write_done bit reset after about %llu ms",
									__func__, __LINE__, duration_ms);
								break;
							}
						}
						else
						{
							dev_err(&client->dev, "%s[%d]: Error while reading BCRM_WRITE_HANDSHAKE_8RW register.",
							__func__, __LINE__);
							break;
						}
					} while (duration_ms <= timeout_ms);

					break;
				}
			}
			else
			{
				dev_err(&client->dev, "%s[%d]: Error while reading BCRM_WRITE_HANDSHAKE_8RW register.",
				__func__, __LINE__);
				break;
			}
		} while (duration_ms <= timeout_ms);

		if (!handshake_valid)
		{
			dev_warn(&client->dev, "%s[%d]: Write handshake timeout!", __func__, __LINE__);
		}
	}
	else
	{
		dev_info(&client->dev, "%s[%d]: Handshake not supported. Use static sleep at least once as fallback %lld", __func__, __LINE__, timeout_ms);
		/* Handshake not supported. Use static sleep at least once as fallback */
		msleep(default_wait_time_ms);
		duration_ms = (uint64_t)default_wait_time_ms;
	}

	return duration_ms;
#endif

	complete(&sensor->bcrm_wrhs_completion);

	return;
}

/*******************************************************************
 *  avt_csi2_probe                                                 *
 *******************************************************************
 *                                                                 *
 *******************************************************************
 *                                                                 *
 *******************************************************************
 *                                                                 *
 *******************************************************************
 *                                                                 *
 ******************************************************************/

static int avt3_probe(struct i2c_client *client)
{

	struct device *dev = &client->dev;
	struct avt3_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	dev_info(&client->dev, "%s[%d]: %s",
			 __func__, __LINE__, __FILE__);

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
	{
		ret = -ENOMEM;
		goto err_exit;
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
		// return PTR_ERR(sensor->pwdn_gpio);
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
		// return PTR_ERR(sensor->reset_gpio);
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

	// devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);

	/* request optional power down pin */
	sensor->pwdn_gpio = devm_gpiod_get_optional(dev, "powerdown",
												GPIOD_OUT_HIGH);
	if (NULL == sensor->pwdn_gpio || IS_ERR(sensor->pwdn_gpio))
	{
		// return PTR_ERR(sensor->pwdn_gpio);
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
		// return PTR_ERR(sensor->reset_gpio);
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

	/////////////////////////////////////////////////////
	/* TODO: check for compatible HW */

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

	/* reading the Firmware Version register */
	ret = regmap_bulk_read(sensor->regmap64,
						   sensor->cci_reg.reg.bcrm_addr + BCRM_DEVICE_FIRMWARE_VERSION_64R,
						   &sensor->cam_firmware_version.value, 1);

	dev_info(&client->dev, "%s[%d]: Firmware version: %u.%u.%u.%u ret = %d\n",
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

	CLEAR(sensor->max_rect);
	CLEAR(sensor->min_rect);
	CLEAR(sensor->curr_rect);

	ret = avt3_get_sensor_capabilities(&sensor->sd);
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

	sensor->current_fr = AVT3_DEFAULT_FPS;
	sensor->frame_interval.numerator = 1000;
	sensor->frame_interval.denominator = 1000 * avt3_framerates[sensor->current_fr];
	sensor->current_mode =
		&avt3_mode_data[AVT3_DEFAULT_MODE];
	sensor->last_mode = sensor->current_mode;
	sensor->gain = AVT3_DEFAULT_GAIN;
	sensor->exposure_time = AVT3_DEFAULT_EXPOSURETIME;
	sensor->exposure_mode = EMODE_MANUAL;

	fmt = &sensor->mbus_framefmt;
	fmt->code = sensor->available_fmts[0].mbus_code;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = avt3_mode_data[AVT3_DEFAULT_MODE].hact;
	fmt->height = avt3_mode_data[AVT3_DEFAULT_MODE].vact;
	fmt->field = V4L2_FIELD_NONE;

	sensor->streamcap.capability = V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = V4L2_MODE_HIGHQUALITY;

#if 1
	ret = v4l2_async_register_subdev(&sensor->sd);
#else
	/* registers a sensor sub-device to the asynchronous sub-device framework
	   and parse set up common sensor related devices */
	ret = v4l2_async_register_subdev_sensor_common(&sensor->sd);
#endif
	if (ret)
	{
		dev_err(dev, "%s[%d]: v4l2_async_register_subdev_sensor_common failed with (%d)\n", __func__, __LINE__, ret);
		goto free_ctrls;
	}
	dev_info(&client->dev, "sensor %s registered\n", sensor->sd.name);

	// INIT_LIST_HEAD(&sensor->queue_list);

	ret = sysfs_create_group(&dev->kobj, &avt3_attr_grp);
	dev_info(dev, " -> %s[%d]: sysfs group created! (%d)\n", __func__, __LINE__, ret);
	if (ret)
	{
		dev_err(dev, "%s[%d]: Failed to create sysfs group (%d)\n", __func__, __LINE__, ret);
		goto free_ctrls;
	}

	ret = avt3_init_controls(sensor);
	if (ret)
	{
		dev_err(dev, "%s[%d]: avt3_init_controls failed with (%d)\n", __func__, __LINE__, ret);
		goto entity_cleanup;
	}

#ifdef DPHY_RESET_WORKAROUND
	sema_init(&sensor->streamon_sem, 0);
	// init_completion(&sensor->streamon_completion);
	avt3_streamon_thread_enable(&sensor->sd);
#endif

	ret = bcrm_regmap_write(sensor, sensor->regmap32, sensor->cci_reg.reg.bcrm_addr + BCRM_STREAM_ON_DELAY_32RW, sensor->streamon_delay);

	dev_info(&client->dev, "%s[%d]: probe success !\n", __func__, __LINE__);

	return 0;

	//##################

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

static int avt3_remove(struct i2c_client *client)
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

	return 0;
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
MODULE_VERSION("2023-1-beta");

#ifdef DPHY_RESET_WORKAROUND
if (sensor->phyreset_on_streamon)
{
	up(&sensor->streamon_sem);
	dev_info(&client->dev, "%s[%d]: up(&sensor->streamon_sem) returnd", __func__, __LINE__);
}
#endif

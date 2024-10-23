// SPDX-License-Identifier: GPL-2.0-or-later
/* 
 * Allied Vision Alvium register definitons
 * 
 * Copyright (C) 2022 - 2024 Allied Vision Technologies GmbH
 */

#ifndef __AVT_CSI2_REGS_H__
#define __AVT_CSI2_REGS_H__

// BCRM register offsets
#define BCRM_VERSION_32R 				0x0000
#define BCRM_FEATURE_INQUIRY_64R 			0x0008
#define BCRM_DEVICE_FIRMWARE_VERSION_64R 		0x0010
#define BCRM_WRITE_HANDSHAKE_8RW 			0x0018

/* Streaming Control Registers */
#define BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R 		0x0040
#define BCRM_CSI2_LANE_COUNT_8RW 			0x0044
#define BCRM_CSI2_CLOCK_MIN_32R 			0x0048
#define BCRM_CSI2_CLOCK_MAX_32R 			0x004C
#define BCRM_CSI2_CLOCK_32RW 				0x0050
#define BCRM_BUFFER_SIZE_32R 				0x0054

#define BCRM_IPU_X_MIN_32W 				0x0058
#define BCRM_IPU_X_MAX_32W 				0x005C
#define BCRM_IPU_X_INC_32W 				0x0060
#define BCRM_IPU_Y_MIN_32W 				0x0064
#define BCRM_IPU_Y_MAX_32W 				0x0068
#define BCRM_IPU_Y_INC_32W 				0x006C
#define BCRM_IPU_X_32R 					0x0070
#define BCRM_IPU_Y_32R 					0x0074

#define BCRM_PHY_RESET_8RW 				0x0078
#define BCRM_STREAM_ON_DELAY_32RW 			0x007C

/* Acquisition Control Registers */
#define BCRM_ACQUISITION_START_8RW 			0x0080
#define BCRM_ACQUISITION_STOP_8RW 			0x0084
#define BCRM_ACQUISITION_ABORT_8RW 			0x0088
#define BCRM_ACQUISITION_STATUS_8R 			0x008C
#define BCRM_ACQUISITION_FRAME_RATE_64RW 		0x0090
#define BCRM_ACQUISITION_FRAME_RATE_MIN_64R 		0x0098
#define BCRM_ACQUISITION_FRAME_RATE_MAX_64R 		0x00A0
#define BCRM_ACQUISITION_FRAME_RATE_INC_64R 		0x00A8
#define BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW 		0x00B0

#define BCRM_FRAME_START_TRIGGER_MODE_8RW 		0x00B4
#define BCRM_FRAME_START_TRIGGER_SOURCE_8RW 		0x00B8
#define BCRM_FRAME_START_TRIGGER_ACTIVATION_8RW 	0x00BC
#define BCRM_FRAME_START_TRIGGER_SOFTWARE_8W 		0x00C0
#define BCRM_FRAME_START_TRIGGER_DELAY_32RW 		0x00C4
#define BCRM_EXPOSURE_ACTIVE_LINE_MODE_8RW 		0x00C8
#define BCRM_EXPOSURE_ACTIVE_LINE_SELECTOR_8RW 		0x00CC
#define BCRM_LINE_CONFIGURATION_32RW 			0x00D0

#define BCRM_IMG_WIDTH_32RW 				0x0100
#define BCRM_IMG_WIDTH_MIN_32R 				0x0104
#define BCRM_IMG_WIDTH_MAX_32R 				0x0108
#define BCRM_IMG_WIDTH_INC_32R 				0x010C

#define BCRM_IMG_HEIGHT_32RW 				0x0110
#define BCRM_IMG_HEIGHT_MIN_32R 			0x0114
#define BCRM_IMG_HEIGHT_MAX_32R 			0x0118
#define BCRM_IMG_HEIGHT_INC_32R 			0x011C

#define BCRM_IMG_OFFSET_X_32RW 				0x0120
#define BCRM_IMG_OFFSET_X_MIN_32R 			0x0124
#define BCRM_IMG_OFFSET_X_MAX_32R 			0x0128
#define BCRM_IMG_OFFSET_X_INC_32R 			0x012C

#define BCRM_IMG_OFFSET_Y_32RW 				0x0130
#define BCRM_IMG_OFFSET_Y_MIN_32R 			0x0134
#define BCRM_IMG_OFFSET_Y_MAX_32R 			0x0138
#define BCRM_IMG_OFFSET_Y_INC_32R 			0x013C

#define BCRM_IMG_MIPI_DATA_FORMAT_32RW 			0x0140
#define BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R 	0x0148
#define BCRM_IMG_BAYER_PATTERN_INQUIRY_8R 		0x0150
#define BCRM_IMG_BAYER_PATTERN_8RW 			0x0154
#define BCRM_IMG_REVERSE_X_8RW 				0x0158
#define BCRM_IMG_REVERSE_Y_8RW				0x015C

#define BCRM_SENSOR_WIDTH_32R 				0x0160
#define BCRM_SENSOR_HEIGHT_32R 				0x0164
#define BCRM_WIDTH_MAX_32R 				0x0168
#define BCRM_HEIGHT_MAX_32R 				0x016C

#define BCRM_BINNING_INQ_16R 				0x0170
#define BCRM_BINNING_SETTING_8RW 			0x0174
#define BCRM_BINNING_MODE_8RW 				0x0178

#define BCRM_EXPOSURE_TIME_64RW 			0x0180
#define BCRM_EXPOSURE_TIME_MIN_64R 			0x0188
#define BCRM_EXPOSURE_TIME_MAX_64R 			0x0190
#define BCRM_EXPOSURE_TIME_INC_64R 			0x0198
#define BCRM_EXPOSURE_AUTO_8RW 				0x01A0

#define BCRM_INTENSITY_AUTO_PRECEDENCE_8RW 		0x01A4
#define BCRM_INTENSITY_AUTO_PRECEDENCE_VALUE_32RW	0x01A8
#define BCRM_INTENSITY_AUTO_PRECEDENCE_MIN_32R 		0x01AC
#define BCRM_INTENSITY_AUTO_PRECEDENCE_MAX_32R 		0x01B0
#define BCRM_INTENSITY_AUTO_PRECEDENCE_INC_32R 		0x01B4

#define BCRM_BLACK_LEVEL_32RW 				0x01B8
#define BCRM_BLACK_LEVEL_MIN_32R 			0x01BC
#define BCRM_BLACK_LEVEL_MAX_32R 			0x01C0
#define BCRM_BLACK_LEVEL_INC_32R 			0x01C4

#define BCRM_GAIN_64RW 					0x01C8
#define BCRM_GAIN_MIN_64R 				0x01D0
#define BCRM_GAIN_MAX_64R 				0x01D8
#define BCRM_GAIN_INC_64R 				0x01E0
#define BCRM_GAIN_AUTO_8RW 				0x01E8

#define BCRM_GAMMA_64RW 				0x01F0
#define BCRM_GAMMA_MIN_64R 				0x01F8
#define BCRM_GAMMA_MAX_64R 				0x0200
#define BCRM_GAMMA_INC_64R 				0x0208

#define BCRM_CONTRAST_VALUE_32RW 			0x0214
#define BCRM_CONTRAST_VALUE_MIN_32R 			0x0218
#define BCRM_CONTRAST_VALUE_MAX_32R 			0x021C
#define BCRM_CONTRAST_VALUE_INC_32R 			0x0220

#define BCRM_SATURATION_32RW 				0x0240
#define BCRM_SATURATION_MIN_32R 			0x0244
#define BCRM_SATURATION_MAX_32R 			0x0248
#define BCRM_SATURATION_INC_32R 			0x024C

#define BCRM_HUE_32RW 					0x0250
#define BCRM_HUE_MIN_32R 				0x0254
#define BCRM_HUE_MAX_32R 				0x0258
#define BCRM_HUE_INC_32R 				0x025C

#define BCRM_ALL_BALANCE_RATIO_64RW 			0x0260
#define BCRM_ALL_BALANCE_RATIO_MIN_64R 			0x0268
#define BCRM_ALL_BALANCE_RATIO_MAX_64R 			0x0270
#define BCRM_ALL_BALANCE_RATIO_INC_64R 			0x0278

#define BCRM_RED_BALANCE_RATIO_64RW 			0x0280
#define BCRM_RED_BALANCE_RATIO_MIN_64R 			0x0288
#define BCRM_RED_BALANCE_RATIO_MAX_64R 			0x0290
#define BCRM_RED_BALANCE_RATIO_INC_64R 			0x0298

#define BCRM_GREEN_BALANCE_RATIO_64RW 			0x02A0
#define BCRM_GREEN_BALANCE_RATIO_MIN_64R 		0x02A8
#define BCRM_GREEN_BALANCE_RATIO_MAX_64R 		0x02B0
#define BCRM_GREEN_BALANCE_RATIO_INC_64R 		0x02B8

#define BCRM_BLUE_BALANCE_RATIO_64RW 			0x02C0
#define BCRM_BLUE_BALANCE_RATIO_MIN_64R 		0x02C8
#define BCRM_BLUE_BALANCE_RATIO_MAX_64R 		0x02D0
#define BCRM_BLUE_BALANCE_RATIO_INC_64R 		0x02D8

#define BCRM_WHITE_BALANCE_AUTO_8RW 			0x02E0
#define BCRM_SHARPNESS_32RW 				0x0300
#define BCRM_SHARPNESS_MIN_32R 				0x0304
#define BCRM_SHARPNESS_MAX_32R 				0x0308
#define BCRM_SHARPNESS_INC_32R 				0x030C

#define BCRM_DEVICE_TEMPERATURE_32R 			0x0310
#define BCRM_EXPOSURE_AUTO_MIN_64RW 			0x0330
#define BCRM_EXPOSURE_AUTO_MAX_64RW 			0x0338
#define BCRM_GAIN_AUTO_MIN_64RW 			0x0340
#define BCRM_GAIN_AUTO_MAX_64RW 			0x0348

#define _BCRM_LAST_ADDR 				BCRM_GAIN_AUTO_MAX_64RW

/************************************************/

/* GenCP Registers */
#define GENCP_CHANGEMODE_8W				0x021C
#define GENCP_CURRENTMODE_8R				0x021D

#define GENCP_OUT_HANDSHAKE_8RW				0x0018
#define GENCP_IN_HANDSHAKE_8RW				0x001c
#define GENCP_OUT_SIZE_16W				0x0020
#define GENCP_IN_SIZE_16R				0x0024

#define BCRM_HANDSHAKE_STATUS_MASK			0x01
#define BCRM_HANDSHAKE_AVAILABLE_MASK			0x80

#if !defined(V4L2_MBUS_CSI2_MAX_DATA_LANES) && defined(V4L2_FWNODE_CSI2_MAX_DATA_LANES)
#define V4L2_MBUS_CSI2_MAX_DATA_LANES V4L2_FWNODE_CSI2_MAX_DATA_LANES
#endif

enum CCI_REG_INFO {
	CCI_REGISTER_LAYOUT_VERSION = 0,
	DEVICE_CAPABILITIES,
	GCPRM_ADDRESS,
	BCRM_ADDRESS,
	DEVICE_GUID,
	MANUFACTURER_NAME,
	MODEL_NAME,
	FAMILY_NAME,
	DEVICE_VERSION,
	MANUFACTURER_INFO,
	SERIAL_NUMBER,
	USER_DEFINED_NAME,
	CHECKSUM,
	CHANGE_MODE,
	CURRENT_MODE,
	SOFT_RESET,
	HEARTBEAT,
	CAM_I2C_ADDRESS,
};

struct cci_cmd {
	__u8 command_index; /* diagnostc test name */
	const __u32 address; /* NULL for no alias name */
	__u32 byte_count;
};

static struct cci_cmd cci_cmd_tbl[] = {
	[CCI_REGISTER_LAYOUT_VERSION] = { CCI_REGISTER_LAYOUT_VERSION, 0x0000,  4 },
	[DEVICE_CAPABILITIES]         = { DEVICE_CAPABILITIES,         0x0008,  8 },
	[GCPRM_ADDRESS]               = { GCPRM_ADDRESS,               0x0010,  2 },
	[BCRM_ADDRESS]                = { BCRM_ADDRESS,                0x0014,  2 },
	[DEVICE_GUID]                 = { DEVICE_GUID,                 0x0018, 64 },
	[MANUFACTURER_NAME]           = { MANUFACTURER_NAME,           0x0058, 64 },
	[MODEL_NAME]                  = { MODEL_NAME,                  0x0098, 64 },
	[FAMILY_NAME]                 = { FAMILY_NAME,                 0x00D8, 64 },
	[DEVICE_VERSION]              = { DEVICE_VERSION,              0x0118, 64 },
	[MANUFACTURER_INFO]           = { MANUFACTURER_INFO,           0x0158, 64 },
	[SERIAL_NUMBER]               = { SERIAL_NUMBER,               0x0198, 64 },
	[USER_DEFINED_NAME]           = { USER_DEFINED_NAME,           0x01D8, 64 },
	[CHECKSUM]                    = { CHECKSUM,                    0x0218,  4 },
	[CHANGE_MODE]                 = { CHANGE_MODE,                 0x021C,  1 },
	[CURRENT_MODE]                = { CURRENT_MODE,                0x021D,  1 },
	[SOFT_RESET]                  = { SOFT_RESET,                  0x021E,  1 },
	[HEARTBEAT]                   = { HEARTBEAT,                   0x021F,  1 },
	[CAM_I2C_ADDRESS]             = { CAM_I2C_ADDRESS,             0x0220,  1 },
};

enum CCI_CAPS_STRING_ENCODING {
	CCI_CAPS_SE_ASCII = 0,
	CCI_CAPS_SE_UTF8,
	CCI_CAPS_SE_UTF16,
	CCI_CAPS_SE_MAX
};

union cci_device_caps_reg {
	struct {
		__u64 user_name:1;
		__u64 bcrm:1;
		__u64 gencp:1;
		__u64 reserved:1;
		__u64 string_encoding:4;
		__u64 family_name:1;
		__u64 reserved2:55;
	} caps;
	__u64 value;
};

/*struct*/ union __attribute__((__packed__)) cci_reg {
	struct __attribute__((__packed__)) {
	__u32   layout_version;				// +   0x000	r
	__u32   reserved_4bytes;			// +   0x004	r
	union cci_device_caps_reg device_capabilities; 	// +   0x008	r
	__u16   gcprm_address;				// +   0x010	r
	__u16   reserved_2bytes;			// +   0x012	r
	__u16   bcrm_addr;				// +   0x014	r
	__u16   reserved_2bytes2;			// +   0x016	r
	char    device_guid[64];			// +   0x018	r
	char    manufacturer_name[64];			// +   0x058	r
	char    model_name[64];				// +   0x098	r
	char    family_name[64];			// +   0x0d8	r
	char    device_version[64];			// +   0x118	r
	char    manufacturer_info[64];			// +   0x158	r
	char    serial_number[64];			// +   0x198	r
	char    user_defined_name[64];			// +   0x1d8	r
	__u32   checksum;				// +   0x218	r
	__u8    change_mode;				// +   0x21c	w
	__u8    current_mode;				// +   0x21d	r
	__u8    soft_reset;				// +   0x21e	rw
	__u8    heartbeat;				// +   0x21f	rw
	__u8    cam_i2c_address;			// +   0x220	rw
	} reg;
	__u8    buf[0x220];			
};

struct __attribute__((__packed__)) gencp_reg {
	__u32   gcprm_layout_version;
	__u16   gencp_out_buffer_address;
	__u16   reserved_2byte;
	__u16   gencp_out_buffer_size;
	__u16   reserved_2byte_1;
	__u16	gencp_in_buffer_address;
	__u16   reserved_2byte_2;
	__u16   gencp_in_buffer_size;
	__u16   reserved_2byte_3;
	__u32   checksum;
};


union bcrm_feature_inquiry_reg {
	struct {
		unsigned long long reverse_x_avail:1;
		unsigned long long reverse_y_avail:1;
		unsigned long long intensity_auto_precedence_avail:1;
		unsigned long long black_level_avail:1;
		unsigned long long gain_avail:1;
		unsigned long long gamma_avail:1;
		unsigned long long contrast_avail:1;
		unsigned long long saturation_avail:1;
		unsigned long long hue_avail:1;
		unsigned long long white_balance_avail:1;
		unsigned long long sharpness_avail:1;
		unsigned long long exposure_auto_avail:1;
		unsigned long long gain_auto_avail:1;
		unsigned long long white_balance_auto_avail:1;
		unsigned long long device_temperature_avail:1;
		unsigned long long acquisition_abort:1;
		unsigned long long acquisition_frame_rate:1;
		unsigned long long frame_trigger:1;
		unsigned long long exposure_active_line_available:1;
		unsigned long long reserved:45;
	} feature_inq;

	unsigned long long value;
};

union device_firmware_version_reg {
	struct {
		unsigned long long special_version:8;
		unsigned long long major_version:8;
		unsigned long long minor_version:16;
		unsigned long long patch_version:32;
	} device_firmware;

	unsigned long long value;
};

union bcrm_avail_mipi_reg {
	struct {
		unsigned long long yuv420_8_leg_avail:1;
		unsigned long long yuv420_8_avail:1;
		unsigned long long yuv420_10_avail:1;
		unsigned long long yuv420_8_csps_avail:1;
		unsigned long long yuv420_10_csps_avail:1;
		unsigned long long yuv422_8_avail:1;
		unsigned long long yuv422_10_avail:1;
		unsigned long long rgb888_avail:1;
		unsigned long long rgb666_avail:1;
		unsigned long long rgb565_avail:1;
		unsigned long long rgb555_avail:1;
		unsigned long long rgb444_avail:1;
		unsigned long long raw6_avail:1;
		unsigned long long raw7_avail:1;
		unsigned long long raw8_avail:1;
		unsigned long long raw10_avail:1;
		unsigned long long raw12_avail:1;
		unsigned long long raw14_avail:1;
		unsigned long long jpeg_avail:1;
		unsigned long long reserved:45;
	} avail_mipi;

	unsigned long long value;
};

union bcrm_bayer_inquiry_reg {
	struct {
		unsigned char monochrome_avail:1;
		unsigned char bayer_GR_avail:1;
		unsigned char bayer_RG_avail:1;
		unsigned char bayer_GB_avail:1;
		unsigned char bayer_BG_avail:1;
		unsigned char reserved:3;
	} bayer_pattern;

	unsigned char value;
};

union bcrm_supported_lanecount_reg {
	struct {
		unsigned char one_lane_avail:1;
		unsigned char two_lane_avail:1;
		unsigned char three_lane_avail:1;
		unsigned char four_lane_avail:1;
		unsigned char reserved:4;
	} lane_mask;

	unsigned char value;
};

/**
 * struct avt_csi_information - sensor specific csi2 bus configuration.
 * TODO: The cid to query this structure is V4L2_CID_AVT_CSI_INFORMATION.
 *
 * @max_lanefrequency  Max available CSI frequency per lane in Hertz.
 * @lanecount          Available CSI lane count.
 * @laneassignment     describes the physical CSI-2 connection between host and camera module.
 *                     The index starts with 0 as CSI lane 1 of the host.
 *                     The value starts with 1 as the CSI lane 1 of the sensor.
 */
struct avt_csi_information {
	__u32 csi_clk_freq; //max_lane_frequency;
	__u8 lane_count;
	__u8 lane_assignment[V4L2_MBUS_CSI2_MAX_DATA_LANES];
	__u8 clk_lane;
};

#define AV_CAM_REG_SIZE		2
#define AV_CAM_DATA_SIZE_8	1
#define AV_CAM_DATA_SIZE_16	2
#define AV_CAM_DATA_SIZE_32	4
#define AV_CAM_DATA_SIZE_64	8
#define ALVIUM_MAX_REG_ADDR	0x0fff

//enum AV_BCRM_FRAME_TRIGGER_SOURCE {
//	BCRM_TS_CAM_LINE_0 = 0,
//	BCRM_TS_CAM_LINE_1 = 1,
//	BCRM_TS_CAM_LINE_2 = 2,
//	BCRM_TS_CAM_LINE_3 = 3,
//	BCRM_TS_CAM_SOFTWARE = 4
//};

#endif

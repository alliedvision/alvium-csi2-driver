// SPDX-License-Identifier: GPL-2.0-only
/* Allied Vision Alvium CSI2 Driver
 *
 * Copyright (C) 2024 Allied Vision Technologies Gmbh
 * 
 */

#ifndef _ALVIUM_DEFAULT_MODE_H
#define _ALVIUM_DEFAULT_MODE_H

#define ALVIUM_DEFAULT_MODE \
    discontinuous_clk = "no"; \
    cil_settletime = "0"; \
    embedded_metadata_height = "0"; \
    \
    mclk_khz = "24000"; \
    phy_mode = "DPHY"; \
    dpcm_enable = "false"; \
    \
    csi_pixel_bit_depth = "4"; \
    \
    active_w = "5488"; \
    active_h = "4112"; \
    pixel_t = "bayer_bggr"; \
    readout_orientation = "0"; \
    line_length = "5488"; \
    inherent_gain = "1"; \
    mclk_multiplier = "31.25"; \
    pix_clk_hz = "750000000"; \
    \
    gain_factor = "16"; \
    framerate_factor = "1000000"; \
    exposure_factor = "1000000"; \
    min_gain_val = "16"; /* 1.0 */ \
    max_gain_val = "256"; /* 16.0 */ \
    step_gain_val = "1"; /* 0.125 */ \
    min_hdr_ratio = "1"; \
    max_hdr_ratio = "64"; \
    min_framerate = "1500000"; /* 1.5 */ \
    max_framerate = "30000000"; /* 30 */ \
    step_framerate = "1"; \
    min_exp_time = "34"; /* us */ \
    max_exp_time = "550385"; /* us */ \
    step_exp_time = "1"; 

#endif // #ifndef _ALVIUM_DEFAULT_MODE_H

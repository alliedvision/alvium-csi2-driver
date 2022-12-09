# Alvium CSI2 driver

## Overview
This repository contains the source code of the linux Alvium CSI2 driver module. 

## Compatibility
This release is compatible with:
- i.MX 8M Plus EVK with i.MX Yocto Project BSP Rev. 5.15.52-2.1.0
- Alvium MIPI CSI-2 cameras with firmware 11.1 or higher

You can use 2 Alvium cameras at the same time if resolution of each camera is less than 4 MP. This is a restriction of the SOM's hardware.

## Getting started

For getting started use the Allied Vision Alvium Yocto Layer: [meta-alvium-avt](https://github.com/alliedvision/meta-alvium-avt)

## Known issues
-  Auto exposure is not available.
-  Triggering via v4l2 controls is not available.
-  When triggering with the I/O controls, the first frame is missing.
-  On startup, the image size is set to 720 x 480.
-  Pixel formats not supported by the camera are listed and selectable.
-  Maximum width of an image: 4096 pixels when only port 1 is used. Maximum width with both ports: 2048 pixels.
-  V4L2 Viewer: 10-bit and 12-bit pixel formats are not correctly displayed.
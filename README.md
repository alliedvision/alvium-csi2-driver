# Alvium CSI2 driver

## Overview
This repository contains the source code of the linux Alvium CSI2 driver module release 1.0.0. 

THE SOFTWARE IS PRELIMINARY AND STILL IN TESTING AND VERIFICATION PHASE AND IS PROVIDED ON AN “AS IS” AND “AS AVAILABLE” BASIS AND IS BELIEVED TO CONTAIN DEFECTS. A PRIMARY PURPOSE OF THIS EARLY ACCESS IS TO OBTAIN FEEDBACK ON PERFORMANCE AND THE IDENTIFICATION OF DEFECT SOFTWARE, HARDWARE AND DOCUMENTATION.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Compatibility
This release is compatible with:
- i.MX 8M Plus EVK with i.MX Yocto Project BSP Rev. 5.15.71-2.2.0
- Xilinx ZCU106 with Xilinx Yocto 2022.2
- Alvium MIPI CSI-2 cameras with firmware 11.1 or higher

## Getting started

For getting started use the Allied Vision Alvium Yocto Layer: [meta-alvium-avt](https://github.com/alliedvision/meta-alvium-avt)

## Known issues
-  i.MX 8M Plus: Maximum width of an image: 4096 pixels when only port 1 is used. Maximum width with both ports: 2048 pixels.
-  Xilinx ZCU106: The prebuild image does only support streaming with YUV422.
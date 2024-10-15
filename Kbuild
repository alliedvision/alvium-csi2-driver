obj-m := avt-csi2.o

ccflags-y += -Werror -I $(M)/include/uapi

ccflags-$(CONFIG_TEGRA_OOT_MODULE) += -DNVIDIA -I $(srctree.nvidia-oot)/include

dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-p3768-alvium-dual.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3737-camera-dual-alvium-overlay.dtbo
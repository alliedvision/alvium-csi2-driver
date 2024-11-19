obj-m := avt-csi2.o

ccflags-y += -Werror -I $(M)/include/uapi

ccflags-$(CONFIG_TEGRA_OOT_MODULE) += -DNVIDIA -I $(srctree.nvidia-oot)/include

dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-p3768-alvium-dual-19623-1x2-1x4.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3737-camera-dual-alvium-19616-2x4-overlay.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-forecr-ornx-alvium-dual-19623-2x4.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-forecr-ornx-alvium-dual-19623+19558-2x4.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-forecr-ornx-alvium-dual-19623+19502-2x4.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3737-camera-dual-alvium-19616+19502-2x4-overlay.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3737-camera-dual-alvium-19616+19558-2x4-overlay.dtbo

obj-m := avt-csi2.o

ccflags-y += -Werror -I $(M)/include/uapi

ifdef srctree.nvidia-oot
NVIDIA_KERNEL_OOT_INCLUDE=$(srctree.nvidia-oot)/include
endif

ccflags-$(CONFIG_TEGRA_OOT_MODULE) += -DNVIDIA -I $(NVIDIA_KERNEL_OOT_INCLUDE)

dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-p3768-alvium-dual-19623-1x2-1x4.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3737-camera-dual-alvium-19616-2x4-overlay.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-forecr-ornx-alvium-dual-19623-2x4.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-forecr-ornx-alvium-dual-19623+19558-2x4.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-forecr-ornx-alvium-dual-19623+19502-2x4.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3737-camera-dual-alvium-19616+19502-2x4-overlay.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3737-camera-dual-alvium-19616+19558-2x4-overlay.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3737-camera-dual-alvium-19616+19558-2x4-single-deser-overlay.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-forecr-ornx-alvium-dual-19623-dual-14384-4x2.dtbo
dtb-$(CONFIG_TEGRA_OOT_MODULE) += overlay/tegra234-p3767-camera-forecr-ornx-alvium-dual-19623-single-14384-1x4-2x2.dtbo


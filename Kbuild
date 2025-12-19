obj-m := avt-csi2.o

ccflags-y += -Werror -I $(M)/include/uapi

ifdef srctree.nvidia-oot
NVIDIA_KERNEL_OOT_INCLUDE=$(srctree.nvidia-oot)/include
endif

ccflags-$(CONFIG_TEGRA_OOT_MODULE) += -DNVIDIA -I $(NVIDIA_KERNEL_OOT_INCLUDE)

subdir-y += overlay
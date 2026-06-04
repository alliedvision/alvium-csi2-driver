obj-m := avt-csi2.o

ccflags-y += -Werror -I $(M)/include/uapi

ifdef srctree.nvidia-oot
NVIDIA_KERNEL_OOT_INCLUDE=$(srctree.nvidia-oot)/include
endif

ifdef srctree.nvconftest
NVIDIA_KERNEL_CONFTEST_INCLUDE=$(srctree.nvconftest)
endif


ccflags-$(CONFIG_TEGRA_OOT_MODULE) += -DNVIDIA -I $(NVIDIA_KERNEL_OOT_INCLUDE) -I $(NVIDIA_KERNEL_CONFTEST_INCLUDE)

subdir-y += overlay
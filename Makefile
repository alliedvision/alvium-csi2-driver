SRC := $(shell pwd)

#CFLAGS += -DISP8000NANO_V1802
all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

overlay_install:
	install -d $(INSTALL_MOD_PATH)/boot/
	install -m 0644 $(SRC)/overlay/tegra234-p3767-camera-p3768-alvium-dual.dtbo $(INSTALL_MOD_PATH)/boot/
	install -m 0644 $(SRC)/overlay/tegra234-p3737-camera-dual-alvium-overlay.dtbo $(INSTALL_MOD_PATH)/boot/

header_install:
	install -d $(INSTALL_MOD_PATH)/usr/include/linux
	install -m 0644 $(SRC)/include/uapi/linux/avt-csi2.h $(INSTALL_MOD_PATH)/usr/include/linux/

install: modules_install overlay_install header_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

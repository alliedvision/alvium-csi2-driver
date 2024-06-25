SRC := $(shell pwd)

#CFLAGS += -DISP8000NANO_V1802
all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

overlay_install:
	mkdir $(INSTALL_MOD_PATH)/boot/
	install -m 0644 $(SRC)/overlay/tegra234-p3767-camera-p3768-alvium-dual.dtbo $(INSTALL_MOD_PATH)/boot/

install: modules_install overlay_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

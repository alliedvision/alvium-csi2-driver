SRC := $(shell pwd)

OVERLAY_INSTALL_DIR ?= $(INSTALL_MOD_PATH)/boot/

OVERLAYS := $(wildcard overlay/*/*.dtbo)
OVERLAYS_INSTALL := $(addsuffix .oinst, $(OVERLAYS))

.PHONY: overlay_install

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

overlay_install: overlay_install_prepare $(OVERLAYS_INSTALL)

header_install:
	@install -d $(INSTALL_MOD_PATH)/usr/include/linux
	@echo "  INSTALL include/uapi/linux/avt-csi2.h"
	@install -m 0644 $(SRC)/include/uapi/linux/avt-csi2.h $(INSTALL_MOD_PATH)/usr/include/linux/

install: modules_install overlay_install header_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

overlay_install_prepare:
	@install -d $(OVERLAY_INSTALL_DIR)

%.oinst: %
	@echo "  INSTALL $<"
	@install -m 0644 $< $(OVERLAY_INSTALL_DIR)



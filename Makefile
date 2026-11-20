# Out-of-tree build of the IMX708 VIN sensor driver for Orange Pi (Allwinner A733, 6.6.98-sun60iw2).
# KDIR: Orange Pi kernel tree (branch orange-pi-6.6-sun60iw2) prepared with the running config:
#   cp /boot/config-$(uname -r) .config && make olddefconfig && make modules_prepare LOCALVERSION=-sun60iw2
KDIR ?= $(HOME)/code/a733-kernel/linux
SENSOR_DIR := $(KDIR)/bsp/drivers/vin/modules/sensor

obj-m := imx708.o
ccflags-y += -I$(SENSOR_DIR) -I$(KDIR)/bsp/drivers/vin -I$(KDIR)/bsp/include
ccflags-y += -include $(KDIR)/include/generated/autoconf.h

# The vin helpers live in vin_io.ko / vin_v4l2.ko; their Module.symvers is not shipped.
export KBUILD_MODPOST_WARN := 1

# Kernel's dtc (built by modules_prepare); -@ keeps the symbols the overlay needs
DTC ?= $(KDIR)/scripts/dtc/dtc

.PHONY: all module dtbo install uninstall clean

all: module dtbo

module:
	$(MAKE) -C $(KDIR) M=$(CURDIR) LOCALVERSION=-sun60iw2 modules

dtbo: imx708.dtbo

imx708.dtbo: imx708.dts
	$(DTC) -@ -I dts -O dtb -o $@ $<

# Build as the normal user (KDIR is under $HOME), install with sudo, then reboot
install: all
	sudo ./install.sh

uninstall:
	sudo ./install.sh --remove

# Userspace capture tool that runs the ISP (AE/AWB); see isp_capture.c
isp_capture: isp_capture.c
	gcc -O2 -Wall -rdynamic -o $@ $< -lAWIspApi -lpthread -ldl -lm

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
	rm -f isp_capture imx708.dtbo

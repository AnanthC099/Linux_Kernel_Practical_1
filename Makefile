# Makefile for GPIO LED Driver (Legacy + Platform/DT versions)
#
# Usage:
#   Native build on RPi4:
#     make                    # Build both kernel modules
#     make dtbo               # Compile device tree overlay
#     make app                # Build user-space application
#     make all-targets        # Build everything
#
#   Cross-compile from x86 host:
#     make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
#          KERNEL_DIR=/path/to/rpi-linux
#
#   Clean:
#     make clean

# ---- Both kernel modules ----
obj-m := gpio_led_driver.o gpio_led_platform_driver.o

# Kernel source directory
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build

# DT compiler
DTC ?= dtc

# Current directory
PWD := $(shell pwd)

# Default: build kernel modules
all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

# Build ONLY the legacy driver (no DT needed)
legacy:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules obj-m=gpio_led_driver.o

# Build ONLY the platform driver (needs DT overlay)
platform:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules obj-m=gpio_led_platform_driver.o

# Compile the Device Tree overlay (.dts → .dtbo)
dtbo: gpio_led_overlay.dts
	$(DTC) -@ -I dts -O dtb -o gpio_led_overlay.dtbo gpio_led_overlay.dts
	@echo "Device Tree Overlay compiled: gpio_led_overlay.dtbo"

# Build the user-space application
app: gpio_led_app.c gpio_led_ioctl.h
	$(CC) -Wall -Wextra -O2 -o gpio_led_app gpio_led_app.c

# Build everything
all-targets: all dtbo app
	@echo "All targets built successfully."

# Install the platform driver + DT overlay
install-platform: platform dtbo
	sudo cp gpio_led_overlay.dtbo /boot/overlays/
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install obj-m=gpio_led_platform_driver.o
	depmod -a
	@echo ""
	@echo "=== Installation Complete ==="
	@echo "Add to /boot/config.txt:  dtoverlay=gpio_led_overlay"
	@echo "Then reboot, or load at runtime: sudo dtoverlay gpio_led_overlay"
	@echo "Then: sudo modprobe gpio_led_platform_driver"

# Clean build artifacts
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f gpio_led_app gpio_led_overlay.dtbo

.PHONY: all legacy platform dtbo app all-targets install-platform clean

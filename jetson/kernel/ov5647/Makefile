obj-m := ov5647.o

KDIR  := /usr/src/linux-headers-4.9.337-tegra-ubuntu18.04_aarch64
KVER  := 4.9.337-tegra

EXTRA_CFLAGS := \
	-I$(KDIR)/nvidia/include \
	-I$(KDIR)/kernel-4.9/include

all:
	$(MAKE) -C $(KDIR)/kernel-4.9 \
		M=$(PWD) \
		EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
		modules

clean:
	$(MAKE) -C $(KDIR)/kernel-4.9 M=$(PWD) clean

install:
	cp ov5647.ko /lib/modules/$(KVER)/kernel/drivers/media/i2c/
	depmod -a $(KVER)

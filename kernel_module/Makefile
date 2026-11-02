obj-m += xgo-drv.o

KERNEL_DRIVERS = /lib/modules/$(shell uname -r)/kernel/drivers

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(CURDIR) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(CURDIR) clean

install:
		mkdir -p $(KERNEL_DRIVERS)/xgo
		cp -rv xgo-drv.ko $(KERNEL_DRIVERS)/xgo
		depmod -a

uninstall:
		rm $(KERNEL_DRIVERS)/xgo/xgo-drv.ko
		rmdir $(KERNEL_DRIVERS)/xgo

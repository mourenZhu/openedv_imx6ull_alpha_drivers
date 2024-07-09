KERNELDIR := /home/zhumouren/study/embedded/alientek_linux
CURRENT_PATH := $(shell pwd)

obj-m := keyinput.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
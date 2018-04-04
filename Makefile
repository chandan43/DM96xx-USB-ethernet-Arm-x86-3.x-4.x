# Driver Module Name
DRIVER_KMOD = dm9601

obj-m := $(DRIVER_KMOD).o

#CFLAGS_$(DRIVER_KMOD).o := -DDEBUG

$(DRIVER_KMOD)-y := dm9601_main.o	\
#		    dm9601_usbnet.o     \
		    

KDIR=/lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules 
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean 	 

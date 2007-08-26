
# obj-$(CONFIG_ATH5K) += ath5k.o
obj-m += ath5k.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean

clean-files := Module.symvers

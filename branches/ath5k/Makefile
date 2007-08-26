obj-m += ath5k_pci.o
ath5k_pci-objs := ath5k.o ath5k_hw.o ieee80211_regdomain.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean

clean-files := Module.symvers

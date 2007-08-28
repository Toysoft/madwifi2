obj-m += ath5k_pci.o
ath5k_pci-objs := ath5k.o ath5k_hw.o ath5k_hw_phy.o ath5k_hw_inivals.o ieee80211_regdomain.o

all:
	make -C $(KERNELPATH) M=$(PWD) modules
clean:
	make -C $(KERNELPATH) M=$(PWD) clean

clean-files := Module.symvers

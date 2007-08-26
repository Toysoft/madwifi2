obj-m += ieee80211_regdomains.o iso3166-1.o
all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean

clean-files := Module.symvers

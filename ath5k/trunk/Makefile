MOD:=		ath5k
MODULE:=	$(MOD).ko
KLIB?=		/lib/modules/$(shell uname -r)
KLIB_BUILD?=	$(KLIB)/build
KLIB_UPDATES?=	$(KLIB)/updates
KLIB_ATH5K?=	$(KLIB)/kernel/drivers/net/wireless/$(MOD)/$(MODULE)

$(MOD)-objs		= base.o hw.o regdom.o initvals.o phy.o
obj-m			+= $(MOD).o
	
all:
	make -C $(KLIB_BUILD) M=$(PWD) modules
	@if [ $(shell which size) ]; then \
		echo "Module section size analysis:" ; \
		size $(MODULE) ;\
	fi
clean:
	make -C $(KLIB_BUILD) M=$(PWD) clean
install_kmod:
	@test -d $(KLIB_UPDATES) || mkdir -p $(KLIB_UPDATES)
	@install $(MODULE) $(KLIB_UPDATES)
	@# Note: no need to remove the old module when using updates directory.
	@# This is a nice feature so we don't have to delete the distrubution's
	@# driver. This also allows users to 'uninstall' this driver and still
	@# use their old presetn driver.
	@depmod -ae
install: install_kmod
	@# We split the install up into two sections to be able to run the
	@# test below. Putting them together doesn't work.
	@if [ "$(shell modprobe -l $(MOD))" = \
			"$(KLIB_UPDATES)/$(MODULE)" ]; then \
		echo "$(MOD) installation was successfull! New module:"; \
	else \
		echo modprobe is not picking up $(KLIB_UPDATES)/$(MODULE); \
		echo as the next loadable $(MOD) module. ;\
		echo ;\
		echo You should temporarily move or delete this file module:;\
	fi
	@modprobe -l $(MOD)
uninstall:
	@rm -f $(KLIB_UPDATES)/$(MODULE)
	@depmod -ae
	@test  $(KLIB_ATH5K) && \
		echo "Old $(MOD) module still available:" &&\
		modprobe -l $(MOD)

clean-files := Module.symvers

.PHONY: all clean install install_kmod uninstall

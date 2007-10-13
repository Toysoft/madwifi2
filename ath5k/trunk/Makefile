MOD:=		ath5k
MODULE:=	$(MOD).ko

KMODDIR?=	updates
KMODDIR_ARG:=	"INSTALL_MOD_DIR=$(UPDATE)"
ifneq ($(origin $(KLIB)), undefined)
KMODPATH_ARG:=	"INSTALL_MOD_PATH=$(KLIB)"
else
KLIB:=		/lib/modules/$(shell uname -r)
endif
KSRC?=	$(KLIB)/build
DESTDIR:=	$(KLIB)/$(KMODDIR)
DESTDIR_ATH5K:=	$(KLIB)/kernel/drivers/net/wireless/$(MOD)/$(MODULE)

$(MOD)-objs		= base.o hw.o regdom.o initvals.o phy.o
obj-m			+= $(MOD).o

# Note: no need to remove the old module when using updates directory.
# This is a nice feature so we don't have to delete the distrubution's
# driver. This also allows users to 'uninstall' this driver and still
# use their old present driver.
	
all:
	make -C $(KSRC) M=$(PWD) modules
	@if [ $(shell which size) ]; then \
		echo "Module section size analysis:" ; \
		size $(MODULE) ;\
	fi
clean:
	make -C $(KSRC) M=$(PWD) clean

install_kmod:
	make -C $(KSRC) M=$(PWD) $(KMODDIR_ARG) $(KMODPATH_ARG) modules_install

install: install_kmod
	@# We split the install up into two sections to be able to run the
	@# test below. Putting them together doesn't work.
	@if [ "$(shell modprobe -l $(MOD))" = \
			"$(DESTDIR)/$(MODULE)" ]; then \
		echo ":) $(MOD) installation was successfull! New module:"; \
	else \
		echo ":( modprobe is not picking up";\
		echo "$(DESTDIR)/$(MODULE)"; \
		echo "as the next loadable $(MOD) module.";\
		echo "You should temporarily move or delete this file module:";\
	fi
	@modprobe -l $(MOD)

uninstall:
	@rm -f $(DESTDIR)/$(MODULE)
	@depmod -ae
	@test  $(DESTDIR_ATH5K) && \
		echo "Old $(MOD) module still available:" &&\
		modprobe -l $(MOD)

clean-files := Module.symvers

.PHONY: all clean install install_kmod uninstall

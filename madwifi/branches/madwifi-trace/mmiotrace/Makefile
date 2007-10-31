ifeq ($(obj),)
obj=    .
endif
TOP = $(obj)/..
include $(TOP)/Makefile.inc

INCS += -I$(TOP) -I$(ATH_HAL) -I$(HAL) -I$(WLAN)
EXTRA_CFLAGS+= $(INCS) $(COPTS) -DOPT_AH_H=\"public/$(TARGET).opt_ah.h\"

obj-m := mmio.o
mmio-objs := mmio-mod.o pf_in.o kmmio.o

install:
	test -d $(DESTDIR)/$(KMODPATH) || mkdir -p $(DESTDIR)/$(KMODPATH)
	install mmio.$(KMODSUF) $(DESTDIR)/$(KMODPATH)

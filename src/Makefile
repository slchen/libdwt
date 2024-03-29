include ../common.mk

LIBNAME = libdwt
CFLAGS += -finline-functions -D_POSIX_C_SOURCE=199309L -D_GNU_SOURCE -DPACKAGE_VERSION="$(PACKAGE_VERSION)" -DPACKAGE_NAME="$(PACKAGE_NAME)" -DPACKAGE_STRING="$(PACKAGE_STRING)" -DARCH="$(ARCH)"

.PHONY: all clean version

all: $(LIBNAME).o

clean:
	-$(RM) *.o *.a *.so

# ARCH=microblaze needs firmwares to be compiled
ifeq ($(ARCH),microblaze)
FWDIR = firmware
FIRMWARES = fw_fp01_lift4sa fw_fp01_lift4sb

$(LIBNAME).c: $(addprefix $(FWDIR)/,$(addsuffix .h,$(FIRMWARES)))

$(FWDIR)/%.h: $(FWDIR)/%.c
	$(MAKE) -C $(FWDIR) $(@F)
endif

$(LIBNAME).o: $(LIBNAME).c $(LIBNAME).h

$(LIBNAME).S: $(LIBNAME).c $(LIBNAME).h
	$(CC) $(CFLAGS) -S -Wa,-adhln -g -fverbose-asm $< -o $@

# $(LIBNAME).a: $(LIBNAME).o
# 	$(AR) -rsc $@ $^
# 
# $(LIBNAME).so: $(LIBNAME).o
# 	$(CC) -shared -o $@ $^

LIBNAME_CV=cvdwt

$(LIBNAME_CV).o: $(LIBNAME_CV).cpp $(LIBNAME_CV).h
	$(CXX) `pkg-config --cflags opencv` -c -o $@ $<

# $(LIBNAME_CV).a: $(LIBNAME_CV).o
# 	$(AR) -rsc $@ $^

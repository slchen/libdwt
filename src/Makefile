include ../common.mk

LIBNAME=libdwt

.PHONY: all clean version

all: $(LIBNAME).o

clean:
	-$(RM) *.o *.a *.so

# ARCH=microblaze needs firmware
ifeq ($(ARCH),microblaze)
FWDIR = firmware
FWNAME = fw_fp01_lift97

$(LIBNAME).c: $(FWDIR)/$(FWNAME).h

$(FWDIR)/$(FWNAME).h: $(FWDIR)/$(FWNAME).c
	$(MAKE) -C $(FWDIR) $(FWNAME).h
endif

$(LIBNAME).o: $(LIBNAME).c $(LIBNAME).h

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
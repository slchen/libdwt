include ../common.mk

LIBNAME=libdwt

.PHONY: all clean version

all: $(LIBNAME).o

clean:
	-$(RM) *.o *.a *.so

# ARCH=microblaze needs firmware
ifeq ($(ARCH),microblaze)
# TODO: FWNAMES=name1 name2
FWDIR = firmware
FWNAME1 = fw_fp01_lift4sa
FWNAME2 = fw_fp01_lift4sb

$(LIBNAME).c: $(FWDIR)/$(FWNAME1).h $(FWDIR)/$(FWNAME2).h

$(FWDIR)/$(FWNAME1).h: $(FWDIR)/$(FWNAME1).c
	$(MAKE) -C $(FWDIR) $(FWNAME1).h

$(FWDIR)/$(FWNAME2).h: $(FWDIR)/$(FWNAME2).c
	$(MAKE) -C $(FWDIR) $(FWNAME2).h
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

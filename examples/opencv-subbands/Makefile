include ../../common.mk

LIBPATH = $(ROOT)/src
LIBNAME = libdwt
LIBNAME_CV = cvdwt
CXXFLAGS = `pkg-config --cflags opencv` -std=c++98 -pedantic -Wall -Wextra -O2 -I$(LIBPATH)
LDLIBS += `pkg-config --libs opencv`
BIN = showdwt

.PHONY: all clean

all: $(BIN)

clean:
	$(MAKE) -C $(LIBPATH) $@
	-$(RM) *.o $(BIN)

# link application
$(BIN): $(BIN).o $(LIBPATH)/$(LIBNAME).o $(LIBPATH)/$(LIBNAME_CV).o

# compile application
$(BIN).o: $(BIN).cpp $(LIBPATH)/$(LIBNAME).h $(LIBPATH)/$(LIBNAME_CV).h

# compile dependency
$(LIBPATH)/$(LIBNAME).o: $(LIBPATH)/$(LIBNAME).c $(LIBPATH)/$(LIBNAME).h
	$(MAKE) -C $(LIBPATH) $(LIBNAME).o

# compile dependency
$(LIBPATH)/$(LIBNAME_CV).o: $(LIBPATH)/$(LIBNAME_CV).cpp $(LIBPATH)/$(LIBNAME_CV).h
	$(MAKE) -C $(LIBPATH) $(LIBNAME_CV).o

LIBNAME=libdwt
LIBPATH=../../src
CFLAGS=-std=c99 -pedantic -Wall -Wextra -O2 -I$(LIBPATH) -fopenmp
LDFLAGS=-lrt -fopenmp -lm
BIN=simple

.PHONY: all clean

all: $(BIN)

clean:
	$(MAKE) -C $(LIBPATH) $@
	-$(RM) $(BIN) *.o

$(BIN): $(BIN).o $(LIBPATH)/$(LIBNAME).o

$(BIN).o: $(BIN).c $(LIBPATH)/$(LIBNAME).h

$(LIBPATH)/$(LIBNAME).o: $(LIBPATH)/$(LIBNAME).c $(LIBPATH)/$(LIBNAME).h
	$(MAKE) -C $(LIBPATH) $(LIBNAME).o

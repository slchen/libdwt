LIBPATH=../../src
LIBNAME=libdwt
CXXFLAGS=`pkg-config --cflags opencv` -std=c++98 -pedantic -Wall -Wextra -O2 -I$(LIBPATH)
LDFLAGS=`pkg-config --libs opencv` -fopenmp
BIN=showdwt

.PHONY: all clean

all: $(BIN)

clean:
	$(MAKE) -C $(LIBPATH) $@
	-$(RM) *.o $(BIN)

# link application
$(BIN): $(BIN).o $(LIBPATH)/$(LIBNAME).o

# compile application
$(BIN).o: $(BIN).cpp $(LIBPATH)/$(LIBNAME).h

# compile dependency
$(LIBPATH)/$(LIBNAME).o: $(LIBPATH)/$(LIBNAME).c $(LIBPATH)/$(LIBNAME).h
	$(MAKE) -C $(LIBPATH) $(LIBNAME).o

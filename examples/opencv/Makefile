CXXFLAGS=`pkg-config --cflags opencv` -std=c++98 -pedantic -Wall -Wextra -O2 -I../../src
LDFLAGS=`pkg-config --libs opencv` -fopenmp
BIN=showdwt

.PHONY: all clean

all: $(BIN)

clean:
	-$(RM) *.o $(BIN)

$(BIN): ../../src/libdwt.o $(BIN).o ../../src/libdwt.h

../../src/libdwt.o:
	$(MAKE) -C ../../src libdwt.o

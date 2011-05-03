CFLAGS=-std=c99 -pedantic -Wall -Wextra -O2 -fopenmp -fPIC

.PHONY: all clean

all: libdwt.o

clean:
	-$(RM) *.o *.a *.so

libdwt.o: libdwt.c libdwt.h

libdwt.a: libdwt.o
	$(AR) -rsc $@ $^

libdwt.so: libdwt.o
	$(CC) -shared -o $@ $^

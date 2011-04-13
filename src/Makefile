CFLAGS=-std=c99 -pedantic -Wall -Wextra `pkg-config --cflags opencv`
CXXFLAGS=-std=c++98 -pedantic -Wall -Wextra `pkg-config --cflags opencv`
LDFLAGS=`pkg-config --libs opencv`

.PHONY: clean

showdwt: libdwt.o showdwt.o libdwt.h

libdwt.o: libdwt.c libdwt.h

clean:
	-$(RM) *.o showdwt

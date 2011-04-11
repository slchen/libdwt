CFLAGS=-std=c99 -pedantic -Wall -Wextra `pkg-config --cflags opencv`
CXXFLAGS=-std=c++98 -pedantic -Wall -Wextra `pkg-config --cflags opencv`
LDFLAGS=`pkg-config --libs opencv`

.PHONY: clean

showdwt: libdwt.o showdwt.o

clean:
	-$(RM) *.o showdwt

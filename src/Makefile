CFLAGS_DEFAULT=-std=c99 -pedantic -Wall -Wextra -O2
CFLAGS_OPENCV=`pkg-config --cflags opencv`
CFLAGS=$(CFLAGS_DEFAULT) $(CFLAGS_OPENCV)
CXXFLAGS_DEFAULT=-std=c++98 -pedantic -Wall -Wextra -O2
CXXFLAGS_OPENCV=$(CFLAGS_OPENCV)
CXXFLAGS=$(CXXFLAGS_DEFAULT) $(CXXFLAGS_OPENCV)
LDFLAGS_DEFAULT=
LDFLAGS_OPENCV=`pkg-config --libs opencv`
LDFLAGS=$(LDFLAGS_DEFAULT) $(LDFLAGS_OPENCV)

.PHONY: clean

showdwt: libdwt.o showdwt.o libdwt.h

libdwt.o: libdwt.c libdwt.h
	$(CC) $(CFLAGS_DEFAULT) -c -o $@ $<

clean:
	-$(RM) *.o showdwt

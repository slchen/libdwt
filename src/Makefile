LIBNAME=libdwt
CFLAGS=-std=c99 -pedantic -Wall -Wextra -O2 -fopenmp -fPIC

.PHONY: all clean

all: $(LIBNAME).o

clean:
	-$(RM) *.o *.a *.so

$(LIBNAME).o: $(LIBNAME).c $(LIBNAME).h

$(LIBNAME).a: $(LIBNAME).o
	$(AR) -rsc $@ $^

$(LIBNAME).so: $(LIBNAME).o
	$(CC) -shared -o $@ $^

LIBNAME_CV=cvdwt

$(LIBNAME_CV).o: $(LIBNAME_CV).cpp $(LIBNAME_CV).h
	$(CXX) `pkg-config --cflags opencv` -c -o $@ $<

$(LIBNAME_CV).a: $(LIBNAME_CV).o
	$(AR) -rsc $@ $^

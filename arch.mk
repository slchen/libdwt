ARCH ?= $(shell uname -m)

# common part
CROSS_COMPILE ?= 
CC = $(CROSS_COMPILE)gcc
CFLAGS = -std=c99 -pedantic -Wall -Wextra -O2
LDLIBS = -lm
LDFLAGS = 

# ASVP platform
ifeq ($(ARCH),microblaze)
	CROSS_COMPILE = microblaze-uclinux-
	CFLAGS += -mno-xl-soft-mul -mhard-float -mcpu=v8.00.b -DEMBED -Dlinux -D__linux__ -Dunix -D__uClinux__ -DLINUX -I$(ROOT)/api/22-mb-petalinux/libwal -I$(ROOT)/api/22-mb-petalinux/libbce_config_step4 -Wno-unknown-pragmas
	LDFLAGS += -L$(ROOT)/api/22-mb-petalinux/libwal -L$(ROOT)/api/22-mb-petalinux/libbce_config_step4 # asvp-demo_v0
	LDLIBS += -lwal -lbce_config
endif

# 64-bit x86 platform
ifeq ($(ARCH),x86_64)
	CROSS_COMPILE = 
	CFLAGS += -fopenmp -fPIC
	LDFLAGS += -fopenmp
	LDLIBS += -lrt
endif

# x86-64 without MMX and SSE
ifeq ($(ARCH),x86_64+nosse)
	CROSS_COMPILE = 
	CFLAGS += -fopenmp -fPIC
	CFLAGS += -mno-sse
	LDFLAGS += -fopenmp
	LDLIBS += -lrt
endif

ifeq ($(ARCH),armv6l)
	CROSS_COMPILE = 
	CFLAGS += -fopenmp -fPIC
	LDFLAGS += -fopenmp
	LDLIBS += -lrt                                                                                                                                                              
endif

ifeq ($(BUILD),release)
	CFLAGS += -DNDEBUG
endif

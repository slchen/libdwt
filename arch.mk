ARCH ?= $(shell arch)

# common part
CROSS_COMPILE ?= 
CC = $(CROSS_COMPILE)gcc
CFLAGS = -std=c99 -pedantic -Wall -Wextra -O2
LDLIBS = -lm
LDFLAGS = 

# EdkDSP platform
ifeq ($(ARCH),microblaze-legacy)
	CROSS_COMPILE = microblaze-uclinux-
	CFLAGS += -mno-xl-soft-mul -mhard-float -mcpu=v7.30.b -DLINUX -I$(ROOT)/api/02-mb-petalinux/libwal -I$(ROOT)/api/02-mb-petalinux/libbce_step3 -Ifirmware -Wno-unknown-pragmas
	LDFLAGS += -L$(ROOT)/api/02-mb-petalinux/libwal -L$(ROOT)/api/02-mb-petalinux/libbce_step3
	LDLIBS += -lwal -lbce_fp01_1x1
endif

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

ifeq ($(BUILD),release)
	CFLAGS += -DNDEBUG
endif

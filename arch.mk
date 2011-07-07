ARCH ?= $(shell arch)

# common part
CTARGET ?= 
CROSS_COMPILE ?= $(CTARGET)-
CC = $(CROSS_COMPILE)gcc
CFLAGS = -std=c99 -pedantic -Wall -Wextra -O2 -DPACKAGE_VERSION="$(PACKAGE_VERSION)" -DPACKAGE_NAME="$(PACKAGE_NAME)"
CFLAGS += -finline-functions # FIXME: good performance on microblaze
# CFLAGS += -DNDEBUG # NOTE: faster
LDLIBS = -lm
LDFLAGS = 

# EdkDSP platform
ifeq ($(ARCH),microblaze)
CROSS_COMPILE = microblaze-uclinux-
CFLAGS += -mno-xl-soft-mul -mhard-float -mcpu=v7.30.b -DLINUX -I$(ROOT)/api/02-mb-petalinux/libwal -I$(ROOT)/api/02-mb-petalinux/libbce_step3 -Ifirmware
CFLAGS += -Wno-unused-function -Wno-unknown-pragmas # FIXME
LDFLAGS += -L$(ROOT)/api/02-mb-petalinux/libwal -L$(ROOT)/api/02-mb-petalinux/libbce_step3
LDLIBS += -lwal -lbce_fp01_1x1
endif

# 64-bit x86 platform
ifeq ($(ARCH),x86_64)
CROSS_COMPILE = 
CFLAGS += -fopenmp -fPIC
LDFLAGS += -fopenmp
LDLIBS += -lrt
endif

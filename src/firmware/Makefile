EDKDSP_API_PATH=../../api
CC=pbcc
PSMO=$(EDKDSP_API_PATH)/00-pb-firmware/dfu_fp01_1x1.psmo
OUT=fw_fp01_lift97.h

.PHONY: clean all

all: $(OUT)

clean:
	-$(RM) $(OUT)

%.h: %.c $(PSMO)
	$(CC) $^ -o $@

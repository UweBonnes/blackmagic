TARGET=blackmagic_stlinkv2
SYS = $(shell $(CC) -dumpmachine)
VERS = $(shell git describe --always --dirty)
CFLAGS += -DLIBFTDI  -DSTLINKV2 -DJTAG_HL
CFLAGS +=-I ./target
LDFLAGS += -lusb-1.0
ifneq (, $(findstring mingw, $(SYS)))
LDFLAGS += -lws2_32
CFLAGS += -Wno-cast-function-type
else ifneq (, $(findstring cygwin, $(SYS)))
LDFLAGS += -lws2_32
endif
VPATH += platforms/pc
SRC += 	timing.c stlinkv2.c
SWD_HL = 1
JTAG_HL = 1

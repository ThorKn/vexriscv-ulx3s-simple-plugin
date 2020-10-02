PROJ_NAME=simple_plugin
DEBUG=no
BENCH=no
MULDIV=no

CFLAGS += -fno-inline -fno-common 
LDFLAGS += -lc

CFLAGS +=  -DPREALLOCATE=1 -DHOST_DEBUG=0 -DMSC_CLOCK 

SRCS = 	src/main.c src/crt.S  src/stdlib.c 
LDFLAGS += -lc #Used to get memcpy/strcmp stuff only

INC += -I libs/
LDSCRIPT = linker.ld


include gcc.mk
include subproject.mk


			

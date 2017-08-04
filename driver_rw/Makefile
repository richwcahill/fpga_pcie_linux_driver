## Simple Makefile
## Altera SOC compiler requirements
##export ARCH=arm
##export CC=/opt/linaro/gcc-linaro-4.8-2015.06-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc

CFLAGS=-I.

all:
	 $(CC) -o driver_rw driver_rw.c -I.


.PHONY: clean

clean:
	rm -f driver_rw  


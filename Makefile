# Makefile

export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

ifeq (${KERNELRELEASE},)
    KERNEL_SOURCE := ../linux
    PWD := $(shell pwd)
default:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} modules

clean:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} clean

else
    obj-m := mcr20a.o
endif

remote_install:
	scp mcr20a.ko pi@raspberrypi.local:

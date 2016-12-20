# mcr20a-linux
Linux kernel drivers for NXP MCR20A IEEE 802.15.4 transceiver. This project is aiming at not only enabling all MCR20A feathers to 802.15.4 stack in Linux kernel but also providing setup tutorials and tests.

## Table of Contents

- [Install](#install)
- [Hardware](#hardware)
- [Test](#test)
- [TODO](#todo)
- [Contribute](#contribute)
- [License](#license)

## Install

Gerneral: The Linux kbuild system must be used to build the driver as a kernel module.

The command to build an external module is:
```
$ make -C <path_to_kernel_src> M=$PWD
```
*The kbuild system knows that an external module is being built due to the "M=\<dir\>" option given in the command.*

*To build against the running kernel use:*
```
$ make -C /lib/modules/`uname -r`/build M=$PWD
```
*Then to install the module(s) just built, add the target "modules_install" to the command:*
```
$ make -C /lib/modules/`uname -r`/build M=$PWD modules_install
```
To enable the debugfs interface you must add the CONFIG_IEEE802154_MCR20A_DEBUGFS flag to the module's CFLAGS when building the module, i.e.
```
$ make CFLAGS_mcr20a.o:=-DCONFIG_IEEE802154_MCR20A_DEBUGFS=1 -C /lib/modules/`uname -r`/build M=$PWD
```
The built module can be found at /lib/modules/\`uname -r\`/extra/**mcr20a** and can be loaded with the command  
```
sudo modprobe mcr20a
```
Information about the hardware is specified in the Linux device tree. The device tree file will be platform specific but for any tree a section must be added within the section for the relevant spi bus for the mcr20a. See Docs/devicetree/mcr20a.txt for more info.

### Install in Raspberry Pi 3B

## Hardware

```
```

## Test

## TODO

- [ ] Dual PAN
- [ ] TRX with RX ACK
- [ ] Low-power receive mode (LPPS)
- [ ] Fast diversity antenna  (FDA)

## Contribute

PRs accepted.

## License

GPLv2
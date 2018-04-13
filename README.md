The DecaWave DW1000 is an IEEE 802.15.4 ultra-wideband (UWB) radio
chip, capable of timestamping received and transmitted packets with a
nominal resolution of 64GHz (15ps).  This equates to a distance of
around 0.5cm, allowing the DW1000 to be used to construct a viable
indoor positioning system.  (Note that sub-centimetre resolution is
extremely unlikely to be achieved in practice.)

This repository contains a Linux kernel module for the DecaWave DW1000
radio chip, and a reference design for a Raspberry Pi Hat PCB based
around the DWM1000 module.  The total material cost in single-unit
quantities is around $30-$50 per board (including the manufacture of
the PCB itself).

Build instructions
==================

On your Raspberry Pi, install the most recent kernel along with the
corresponding kernel headers:

    sudo apt install raspberrypi-kernel raspberrypi-kernel-headers

If necessary, reboot to ensure that you are running the correct kernel
version:

    sudo reboot

Build and install the kernel module(s):

    git clone --depth 1 https://github.com/raspberrypi/linux rpi/kernel/linux
    KBUILD=/lib/modules/`uname -r`/build
    make -C ${KBUILD} M=`pwd`/rpi/kernel
    sudo make -C ${KBUILD} M=`pwd`/rpi/kernel modules_install
    sudo depmod -a

Retrigger `udev` to detect the DW1000:

    sudo udevadm trigger

Your kernel log should show a message such as
`dw1000 spi0.0: found model 1.3.0`.

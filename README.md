Overview
========

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

    sudo apt update
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

Install the `udev` and `ifupdown` configuration files:

    sudo make -C rpi/config install

Retrigger `udev` to detect the DW1000:

    sudo udevadm trigger

List your network devices:

    ip addr

You should see two interfaces: `wpan0` and `lowpan0`.  For example:

    wpan0: <BROADCAST,NOARP,UP,LOWER_UP> mtu 123 state UNKNOWN
     link/ieee802.15.4 70:b3:d5:b1:e0:00:00:02 brd ff:ff:ff:ff:ff:ff:ff:ff
    lowpan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1280 state UNKNOWN
     link/6lowpan 70:b3:d5:b1:e0:00:00:02 brd ff:ff:ff:ff:ff:ff:ff:ff
     inet6 fe80::72b3:d5b1:e000:2/64 scope link
        valid_lft forever preferred_lft forever

Congratulations; you now have a working DW1000 network interface!

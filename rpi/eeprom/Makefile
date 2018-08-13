#
# Makefile for RPi Tail DT eeproms
#

XTALT  ?= 15
ANTD16 ?= 0x4020
ANTD64 ?= 0x4020
SPIMAX ?= 20000000

DTC = dtc

EEPROMUTILS = hats/eepromutils
EEPMAKE = $(EEPROMUTILS)/eepmake
EEPDUMP = $(EEPROMUTILS)/eepdump
EEPPROG = sudo ./piprog.sh

.PHONY: all tools program clean

all : tools

program :
	@rm -f pitail.eep
	$(MAKE) pitail.eep
	$(EEPPROG) pitail.eep
	@echo "Programming done."

%.dtbo : pitail.dts Makefile
	$(CPP) -nostdinc -undef -x assembler-with-cpp \
		-DEUI64=0x$(firstword $(EUI64) $*)  -DSPIMAX=$(SPIMAX) \
		-DXTALT=$(XTALT) -DANTD16=$(ANTD16) -DANTD64=$(ANTD64) \
		$< | $(DTC) -Wno-unit_address_vs_reg -@ -I dts -O dtb -o $@

%.eep : %.dtbo pitail.txt $(EEPMAKE)
	$(EEPMAKE) pitail.txt $@ $<

%.dump : %.eep $(EEPDUMP)
	$(EEPDUMP) $< $@

tools : $(EEPMAKE) $(EEPDUMP)

$(EEPMAKE) $(EEPDUMP) :
	$(MAKE) -C $(EEPROMUTILS)

clean :
	$(RM) *.dtbo *.eep *.dump
	$(MAKE) -C $(EEPROMUTILS) clean

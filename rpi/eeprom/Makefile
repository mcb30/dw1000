DTC = dtc
EEPROMUTILS = hats/eepromutils
EEPMAKE = $(EEPROMUTILS)/eepmake
EEPDUMP = $(EEPROMUTILS)/eepdump

EUI64_pitail = 0000000000000000

all : pitail.dtbo pitail.eep

%.dtbo : pitail.dts Makefile
	$(CPP) -nostdinc -undef -x assembler-with-cpp \
		-DEUI64=0x$(firstword $(EUI64_$*) $*)  $< | \
		$(DTC) -Wno-unit_address_vs_reg -@ -I dts -O dtb -o $@

%.eep : %.dtbo pitail.txt $(EEPMAKE)
	$(EEPMAKE) pitail.txt $@ $<

%.dump : %.eep $(EEPDUMP)
	$(EEPDUMP) $< $@

$(EEPMAKE) $(EEPDUMP) :
	$(MAKE) -C $(EEPROMUTILS)

clean :
	$(RM) *.dtbo *.eep *.dump
	$(MAKE) -C $(EEPROMUTILS) clean

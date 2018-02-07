DTC = dtc
EEPROMUTILS = hats/eepromutils
EEPMAKE = $(EEPROMUTILS)/eepmake
EEPDUMP = $(EEPROMUTILS)/eepdump

all : pitail.dtbo pitail.eep

%.dtbo : %.dts Makefile
	$(DTC) -Wno-unit_address_vs_reg -@ -I dts -O dtb -o $@ $<

%.eep : %.txt %.dtbo $(EEPMAKE)
	$(EEPMAKE) $< $@ $*.dtbo

%.dump : %.eep $(EEPDUMP)
	$(EEPDUMP) $< $@

$(EEPMAKE) $(EEPDUMP) :
	$(MAKE) -C $(EEPROMUTILS)

clean :
	$(RM) *.dtbo
	$(MAKE) -C $(EEPROMUTILS) clean

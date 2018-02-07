DTC = dtc

all : pitail.dtbo

%.dtbo : %.dts Makefile
	$(DTC) -Wno-unit_address_vs_reg -@ -I dts -O dtb -o $@ $<

clean :
	rm -f *.dtbo

all :

install :
	install -m 644 interfaces-wpan /etc/network/interfaces.d/wpan
	install -m 644 60-wpan.rules /etc/udev/rules.d/60-wpan.rules
	install -m 644 dw1000.conf /etc/modprobe.d/dw1000.conf
	udevadm control --reload

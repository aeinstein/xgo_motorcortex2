MAKEFILES := $(wildcard */Makefile)

# Treiber und Display-Daemon (alles zusammen: sudo ./deploy/install-all.sh)
# Achtung: "make install" aktiviert st7789_display.service; auf dem RiderPi startet
# rc.local den Daemon, deshalb dort install-all.sh verwenden.
DIRS := kernel_module st7789/project/riderpi

.PHONY: all clean install $(DIRS)

all: $(DIRS)

$(DIRS):
	cd $@ && $(MAKE)

clean:
	for dir in $(DIRS); do \
		$(MAKE) -C $$dir clean; \
	done

install:
	for dir in $(DIRS); do \
		$(MAKE) -C $$dir install; \
	done



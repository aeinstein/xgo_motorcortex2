MAKEFILES := $(wildcard */Makefile)

# Treiber und Display-Daemon (xgonav selbst: deploy/install.sh)
DIRS := kernel_module st7789/project/riderpi

.PHONY: all clean $(DIRS)

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



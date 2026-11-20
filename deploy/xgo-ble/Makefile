# xgo-ble: Gegenstelle des BLE-Tunnels am Pi (docs/ble.md). Auf dem Rider: make -C deploy/xgo-ble
CFLAGS ?= -O2 -Wall -Wextra

xgo-ble: xgo-ble.c
	$(CC) $(CFLAGS) -o $@.tmp $< && mv $@.tmp $@

clean:
	rm -f xgo-ble xgo-ble.tmp

.PHONY: clean

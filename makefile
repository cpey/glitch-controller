.PHONY: debug release test clean

BIN = glitch-controller.elf 
BIN_RELEASE = Release/$(BIN)
BIN_DEBUG = Debug/$(BIN)

debug:
	$(MAKE) -C Debug all
	
release:
	$(MAKE) -C Release all

test:
	@if [ -e $(BIN_RELEASE) ] && [ -e $(BIN_DEBUG) ]; then \
		if [ $(BIN_RELEASE) -nt $(BIN_DEBUG) ]; then \
			BIN_TEST=$(BIN_RELEASE); \
		else \
			BIN_TEST=$(BIN_DEBUG); \
		fi; \
	elif [ -e $(BIN_RELEASE) ] && [ ! -e $(BIN_DEBUG) ]; then \
		BIN_TEST=$(BIN_RELEASE); \
	elif [ ! -e $(BIN_RELEASE) ] && [ -e $(BIN_DEBUG) ]; then \
		BIN_TEST=$(BIN_DEBUG); \
	else \
		BIN_TEST=$(BIN_DEBUG); \
		make debug; \
	fi; \
	echo "Using: $$BIN_TEST"; \
	openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
		-c init -c "reset halt" -c "flash write_image erase $$BIN_TEST" \
		-c reset -c shutdown

gdb: debug test
	@if ! netstat -tunap | grep -q ":3333"; then \
		openocd -f interface/stlink.cfg -f target/stm32f0x.cfg -c "init; reset halt" & \
		OPENOCD_PID=$$!; \
	fi; \
	gdb-multiarch Debug/glitch-controller.elf -ex "target remote :3333"; \
	if [ -n "$$OPENOCD_PID" ]; then \
        kill $$OPENOCD_PID; \
    fi

clean:
	$(MAKE) -C Debug clean
	$(MAKE) -C Release clean

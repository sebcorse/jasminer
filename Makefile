include tools/build.mk

# core/: the miner's code. ethash.c and keccak.c are compiled apart (FAST,
# below); ethash_sizes.c, pool.c and surveillance.c go with the hardware layer
# (HAL), as they are left out of the bridge (development target, at the end).
#
# THE ORDER OF THE FILES IS PART OF THE BINARY: it sets the order of the
# functions in .text, hence the alignment. alu_analysis.c comes BEFORE alu.c;
# the default sort of $(wildcard) would put alu.c first and give a binary
# functionally identical but different to the byte.
CORE   := core/alu_analysis.c $(filter-out core/alu_analysis.c core/main.c core/main_stress.c core/ethash_sizes.c core/ethash.c core/keccak.c core/pool.c core/surveillance.c,$(sort $(wildcard core/*.c)))
HAL    := hal/hal_devmem.c hal/fix_params.c hal/ethash_hash_stub.c core/ethash_sizes.c \
          core/pool.c core/surveillance.c

# core/ethash.c and core/keccak.c are compiled apart, with CFLAGS_FAST (see
# tools/build.mk): the only two files whose run time shows, the building of the
# cache, which is downtime at every startup and every epoch change.
FAST := build/ethash.o build/keccak.o

build/%.o: core/%.c
	@mkdir -p build
	$(CC) $(CFLAGS_FAST) -c -o $@ $<

# librepairarm.so (Sunlune) is not linked: our redundancy allocation is
# core/repair_calc.c (corrected mode by default). "make REPAIR=original" gives
# the full survey computation back to librepairarm.so, for comparison: the
# sysroot holds the stock library, and the machine must have it at run time.
LIBS   :=
ifeq ($(REPAIR),original)
CFLAGS += -DJAS_REPAIR_ORIGINAL
CFLAGS_FAST += -DJAS_REPAIR_ORIGINAL
LIBS   := -lrepairarm
endif

.PHONY: all standalone clean
all: standalone

# STANDALONE target: the miner, through /dev/mem (installed as jasminer.elf).
standalone: build/jasre
build/jasre: $(CORE) $(HAL) $(FAST) core/main.c
	@mkdir -p build
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LIBS)
	@echo "  standalone : $$(stat -c%s $@) B"

# Standalone STRESS bench: same chain as standalone, main_stress.c instead.
# Makes the ASICs hash at full speed on synthetic work, without a pool, and
# measures the silicon reject PER CHIP under load. See core/main_stress.c.
.PHONY: stress
stress: build/jasre_stress
build/jasre_stress: $(CORE) $(HAL) $(FAST) core/main_stress.c
	@mkdir -p build
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LIBS)
	@echo "  stress     : $$(stat -c%s $@) B"

# Test vectors FOR THE TARGET: to copy to the machine and run there.
# `make vectors`, then on the machine: ./test_keccak && ./test_ethash
.PHONY: vectors
vectors: build/test_keccak build/test_ethash
build/test_keccak: tools/test_keccak.c core/keccak.c
	@mkdir -p build
	$(CC) $(CFLAGS_TEST) -o $@ $^ $(LDFLAGS)
build/test_ethash: tools/test_ethash.c core/ethash.c core/keccak.c core/ethash_sizes.c
	@mkdir -p build
	$(CC) $(CFLAGS_TEST) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf build

# BRIDGE AND TRACER: development targets, only where bridge/ exists. The bridge
# is a library injected into the stock jasminer.elf by LD_PRELOAD, whose ethash
# stays the firmware's (bridge/ethash_bridge.c), the reference ours is checked
# against; the tracer a library that exports nothing.
ifneq ($(wildcard bridge/hal_bridge.c),)
BRIDGE := bridge/hal_bridge.c bridge/selftest.c bridge/ethash_bridge.c
.PHONY: bridge tracer
bridge: build/libjasre.so
build/libjasre.so: $(CORE) $(BRIDGE)
	@mkdir -p build
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $^ $(LDFLAGS) $(LIBS)
	@echo "  bridge     : $$($(CROSS)readelf -dW $@ | grep -c NEEDED) dependencies, $$(stat -c%s $@) B"
tracer: build/tracer.so
build/tracer.so: bridge/tracer_seul.c
	@mkdir -p build
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $^ $(LDFLAGS)
	@echo "  tracer     : $$(stat -c%s $@) B, $$($(CROSS)readelf -sW $@ | grep -c 'GLOBAL DEFAULT.*[0-9]') exported symbols"
endif

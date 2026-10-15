# Toolchain. The development machine's gcc targets glibc 2.39 and the miner runs
# 2.26: without --sysroot, the binary asks for symbol versions the target's
# loader cannot provide.
SYSROOT ?= $(CURDIR)/sysroot
CROSS   ?= arm-linux-gnueabihf-
CC       = $(CROSS)gcc
ARCH     = -mthumb -march=armv7-a -mfloat-abi=hard -mfpu=vfpv3-d16
WARN     = -Wall -Wextra -Wno-unused-parameter
# -nostdinc + -isystem: otherwise the compiler takes ITS headers (glibc 2.39)
# instead of the sysroot's, hence symbols such as __nanosleep64@GLIBC_2.34 that
# the target cannot resolve.
GCCINC   = $(shell $(CC) -print-file-name=include)
CFLAGS   = -O2 $(ARCH) $(WARN) -Iinclude -nostdinc \
           -isystem $(SYSROOT)/usr/include -isystem $(GCCINC) \
           --sysroot=$(SYSROOT)
# -B: the startup files (crt1.o) must come from the sysroot as well, otherwise
# __libc_start_main is required in version 2.34.
LDFLAGS  = -pthread --sysroot=$(SYSROOT) -B$(SYSROOT)/usr/lib

# ---------------------------------------------------------------------------
# STRONGER FLAGS, AND ONLY FOR TWO FILES.
#
# core/ethash.c and core/keccak.c do NOT mine (the chips hash, not the
# processor). They build the light cache, which takes about two and a half
# minutes during which a chip cannot resume: at every startup and every epoch
# change. That is downtime, not hashrate.
#
# The Zynq-7020 is a Cortex-A9 with NEON (32 D registers); vfpv3-d16 declares
# only sixteen and forbids any vectorization. -O3 unrolls the Keccak
# permutation and vectorizes the FNV loops over 16 words.
#
# WHAT DOES NOT CHANGE: the semantics. These files only do integer operations,
# no floating point, hence no reassociation question. The result must stay
# identical BIT FOR BIT, which is exactly what tools/test_keccak.c and
# tools/test_ethash.c check. If a vector moves, these flags go: a hash is not
# negotiable.
# ---------------------------------------------------------------------------
#
# -fno-strict-aliasing IS NOT A MATTER OF STYLE.
#
# keccak.c does `uint64_t *a = (uint64_t *)state;` on a `uint8_t a[200]`: type
# punning the standard does not allow, which the compiler may break as soon as
# it believes the two pointers cannot overlap. It holds at -O2; -O3 increases
# exactly the analyses that rely on it. The file comes as is from keccak-tiny,
# the reference the bit-for-bit agreement with the firmware depends on, and is
# not rewritten: the assumption is disarmed rather than the code.
#
ARCH_FAST = -mthumb -mcpu=cortex-a9 -mfloat-abi=hard -mfpu=neon
CFLAGS_FAST = -O3 -ftree-vectorize -funroll-loops -fno-strict-aliasing \
                $(ARCH_FAST) $(WARN) -Iinclude -nostdinc \
                -isystem $(SYSROOT)/usr/include -isystem $(GCCINC) \
                --sysroot=$(SYSROOT)

# The test vectors, compiled FOR THE TARGET and with THE SAME flags. Running
# these two binaries on the machine is the only check that proves the
# bit-for-bit agreement: the host is x86, what it validates is not what ships.
CFLAGS_TEST = -O3 -ftree-vectorize -funroll-loops -fno-strict-aliasing \
              $(ARCH_FAST) -Iinclude -nostdinc \
              -isystem $(SYSROOT)/usr/include -isystem $(GCCINC) \
              --sysroot=$(SYSROOT)

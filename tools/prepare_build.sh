#!/bin/bash
# Prepare the build of the miner from Sunlune's official archive.
#
#   tools/prepare_build.sh <JasMiner-X16-Q-20241014-141748.tar.gz>
#
# Run once from the root of the project, before "make". It produces the
# two things the sources take from the stock firmware, without which nothing
# compiles:
#
#   sysroot/              the target's C library, to compile against;
#   core/dft_tables.c     the DFT block's test patterns, read from the stock
#   include/dft_tables.h  jasminer.elf by tools/gen_dft_tables.py.
#
# THE SYSROOT. The miner runs glibc 2.26; a current cross-compiler brings its
# own, newer headers and startup files, and the binary would then ask the
# target's loader for symbol versions it does not have (see tools/build.mk).
# It holds:
#   usr/include, and the files at the top of lib/ and usr/lib/ (neither the
#       subdirectories nor the links to them): the official firmware's own;
#   usr/lib/crt1.o crti.o crtn.o Mcrt1.o Scrt1.o gcrt1.o libc.a libm.a
#       libpthread.a: Debian's libc6-dev 2.24 for armhf (archive.debian.org).
#       glibc 2.24 is older than the target's, so every symbol they need is
#       there; the firmware's own crt*.o are replaced by them.
#
# Both downloads are checked: the archive by the md5 of its ramdisk, the Debian
# package by its sha256.
#
# Needs: tar, gunzip, dd, debugfs (e2fsprogs), dpkg-deb, curl, md5sum,
# sha256sum, python3, readelf (binutils). No root rights: debugfs reads the
# file system image without mounting it.

set -euo pipefail

ARCHIVE=${1:?usage: tools/prepare_build.sh <JasMiner-X16-Q-20241014-141748.tar.gz>}
if [ ! -f Makefile ] || [ ! -f tools/gen_dft_tables.py ]; then
  echo "run from the root of the project"
  exit 1
fi
if [ -e sysroot ]; then
  echo "sysroot/ already exists: remove it to build it again"
  exit 1
fi

OFFICIAL_URAMDISK_MD5=0f6581e0a80f17ff2df9e41758e07322
DEB=libc6-dev_2.24-11+deb9u4_armhf.deb
DEB_URL=http://archive.debian.org/debian/pool/main/g/glibc/$DEB
DEB_SHA256=8a82224c7fb069c0cbfee092c446a16392c986ea969d8dbcd8ccfced516ffc1a
FROM_DEB="crt1.o crti.o crtn.o Mcrt1.o Scrt1.o gcrt1.o libc.a libm.a libpthread.a"
DEBUGFS=$(command -v debugfs || echo /sbin/debugfs)

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "== 1. the official firmware's file system"
tar xzf "$ARCHIVE" -C "$TMP" uramdisk.image.gz
if [ "$(md5sum < "$TMP/uramdisk.image.gz" | cut -c1-32)" != "$OFFICIAL_URAMDISK_MD5" ]; then
  echo "REFUSING: $(basename "$ARCHIVE") is not Sunlune's archive 20241014-141748."
  exit 1
fi
# A u-boot header (64 bytes), then a gzipped ext2 image.
dd if="$TMP/uramdisk.image.gz" of="$TMP/rd.gz" bs=64 skip=1 status=none
gunzip -c "$TMP/rd.gz" > "$TMP/rd.ext2"
mkdir -p "$TMP/fs/usr/bin"
"$DEBUGFS" -R "rdump /lib $TMP/fs" "$TMP/rd.ext2" 2> "$TMP/debugfs.log"
"$DEBUGFS" -R "rdump /usr/lib $TMP/fs/usr" "$TMP/rd.ext2" 2>> "$TMP/debugfs.log"
"$DEBUGFS" -R "rdump /usr/include $TMP/fs/usr" "$TMP/rd.ext2" 2>> "$TMP/debugfs.log"
"$DEBUGFS" -R "dump /usr/bin/jasminer.elf $TMP/fs/usr/bin/jasminer.elf" \
           "$TMP/rd.ext2" 2>> "$TMP/debugfs.log"
for p in lib usr/lib usr/include usr/bin/jasminer.elf; do
  if [ ! -e "$TMP/fs/$p" ]; then
    echo "not extracted: /$p"
    cat "$TMP/debugfs.log"
    exit 1
  fi
done

echo "== 2. the sysroot"
mkdir -p sysroot/lib sysroot/usr/lib
cp -a "$TMP/fs/usr/include" sysroot/usr/include
for d in lib usr/lib; do
  for e in "$TMP/fs/$d"/* "$TMP/fs/$d"/.[!.]*; do
    [ -e "$e" ] || [ -L "$e" ] || continue   # an unmatched pattern
    [ -d "$e" ] && continue                   # a directory, or a link to one
    cp -a "$e" "sysroot/$d/"
  done
done
curl -fsS -o "$TMP/$DEB" "$DEB_URL"
if [ "$(sha256sum < "$TMP/$DEB" | cut -c1-64)" != "$DEB_SHA256" ]; then
  echo "REFUSING: $DEB does not match its sha256."
  exit 1
fi
dpkg-deb -x "$TMP/$DEB" "$TMP/deb"
for f in $FROM_DEB; do
  cp "$TMP/deb/usr/lib/arm-linux-gnueabihf/$f" "sysroot/usr/lib/$f"
done
echo "   sysroot/: $(find sysroot -type f | wc -l) files, $(find sysroot -type l | wc -l) links"

echo "== 3. the DFT tables"
python3 tools/gen_dft_tables.py "$TMP/fs/usr/bin/jasminer.elf" | tail -n 1
echo
echo "Ready: make standalone"

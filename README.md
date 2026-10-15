# JasRe

**This project lets you use (and, I hope, improve) the jasminer.elf that runs our beloved Jasminer X16-Q.**

jasre is an open-source rewrite, in C, of the Jasminer X16-Q's mining program
(`/usr/bin/jasminer.elf`). This repository holds its sources and the recipe to
build it from your own copy of Sunlune's official firmware: no binary, and no
file of Sunlune's.

- **No fee:** every share goes to your pools.
- **Open source:** GPL-3.0-or-later, with all the sources.
- **Built by you:** from your copy of the official firmware, reproducibly.

> **Use at your own risk.** The miner sets the chips' clock and voltage: a
> wrong setting can damage them. This project is not affiliated with Sunlune
> or Jasminer. It has been tested on the X16-Q only.

## What it changes

- **More memory for the ETHW DAG.** Since epoch 840, the ETHW DAG and the cache
  a chip builds to compute it no longer fit together in the chip's memory: on
  the stock firmware the DAG overwrites the cache as it grows, and 17 to 40% of
  the hashes come out wrong. jasre gives the cache's banks back to the DAG:
  once a chip has built what it can, the processor computes the missing end of
  the DAG and writes it there (32 to 36 MiB per chip at epoch 841). A chip
  that still lacks a few banks keeps mining ETHW as long as its expected error
  rate stays below the ETC switch threshold.
- **Per-chip mining.** Each of the three ASICs belongs to one of two pool
  groups, so one machine can mine two pools, or two coins (ETHW and ETC), at
  the same time. Each group has a main pool and a backup: after 120 s without
  news from the main pool the miner switches to the backup, and regularly gives
  the main pool another chance.
- **Per-chip clock and voltage.** Every chip has its own regulator: each can
  get its own clock and core voltage, or all follow a work mode.
- **ETC switch.** When a chip's memory can no longer hold the whole ETHW DAG,
  some of its hashes are wrong. From an error rate you choose, the chip joins
  the group whose pool address contains "etc", where the DAG fits, until the
  next start.
- **Memory repair without Sunlune's library.** The allocation of the memory
  redundancies is rewritten (`core/repair_calc.c`), and you choose which
  repaired banks are kept.
- **Epoch changes.** The DAG is rebuilt in place, chip by chip.
- **Faster starts.** The ethash cache can come from a server on your network
  in about 4 s, instead of about 150 s of computation on the miner (see
  below). Best start measured: the three chips mining ETHW 3 min 13 s after
  the miner started.
- **API** on port 4028, answering in JSON: `summary`, `pools`, `boards`,
  `groups`, `groupctrl`, `all`, `standby`, `resume`, `standby_state`.
- **Logs** in English, in `/var/log/run.log`.

## What you need

- A Linux computer. Tested on Ubuntu 24.04:

  ```sh
  sudo apt install gcc-arm-linux-gnueabihf make python3 binutils e2fsprogs dpkg curl
  ```

- **Sunlune's official firmware, version 20241014-141748**, from Jasminer's
  support pages. The build checks it and refuses any other file:

  | | |
  |---|---|
  | File | `JasMiner-X16-Q-20241014-141748.tar.gz` |
  | Size | 39,126,254 bytes |
  | MD5 | `aea7d05169812b4743bd20ecba56184e` |
  | SHA-256 | `cdb83ae09da92b40375589c15e4cac162bb19f38f6272ac026a8f34076b23806` |

## Build your own jasminer.elf

```sh
git clone https://github.com/sebcorse/jasminer.git
cd jasminer

# 1. Once: what the build takes from the official firmware.
tools/prepare_build.sh ~/Downloads/JasMiner-X16-Q-20241014-141748.tar.gz

# 2. The miner.
make                       # -> build/jasre
```

**Step 1** reads the official archive, without root rights, and writes:

- `sysroot/`: the target's C library (glibc 2.26), to compile against. It holds
  the firmware's own headers and libraries, plus the startup files and static
  libraries of Debian's libc6-dev 2.24 for armhf, downloaded from
  archive.debian.org and checked by their SHA-256.
- `core/dft_tables.c` and `include/dft_tables.h`: the memory test patterns,
  read from the official `jasminer.elf`.

Both stay out of git.

**Step 2** gives `build/jasre`: your jasminer.elf.

The build is reproducible: with Ubuntu 24.04's `arm-linux-gnueabihf-gcc` 13.3,
the same sources give the same binary, byte for byte. Another compiler gives a
different binary, which should behave the same.

Other targets:

| Command | Result |
|---|---|
| `make stress` | `build/jasre_stress`: makes the chips hash at full speed on synthetic work, without a pool, and measures each chip's error rate under load (see `core/main_stress.c`). |
| `make vectors` | `build/test_keccak` and `build/test_ethash`: to run on the miner, they check Keccak and ethash bit for bit, built with the miner's own flags. |

## Configuration

Like the stock program, jasre reads the file given with `-c`,
`/media/configs/jasminer.json` on the X16-Q. The keys of the stock firmware keep
their meaning; the others are optional, and take their default when missing.

| Key | Meaning | Default |
|---|---|---|
| `pools` | Group 1: main pool, then backup, each with `url`, `user` and `pass`. In `user`, what follows the last dot is the worker's name (`jasre` without a dot). | none |
| `pools2` | Group 2, same form. | none |
| `asic_pool_group` | The group of each chip, for instance `"0,1,0"`: 0 for group 1, 1 for group 2. | `"0,0,0"` |
| `work_pattern` | Work mode: 1 Efficiency (500 MHz, 800 mV), 2 Balance (510 MHz, 860 mV), 3 Performance (520 MHz, 880 mV), 4 Performance Ultra (540 MHz, 900 mV). Its clock and voltage apply when `freq` or `asic_rail0` is missing. | 2 |
| `asic_enabled_mask` | The chips in use, one bit per chip, 1 to 7. | 7 |
| `freq`, `asicN_freq` | Clock in MHz, for all chips, and for chip N (0, 1, 2; 270 to 635 accepted). | the work mode's |
| `asic_rail0`, `asicN_rail0` | Core voltage in mV, for all chips, and for chip N (700 to 1000 accepted; the stock pages offer 800 to 900). | the work mode's |
| `asic_rail1`, `asic_rail2` | The board's two other rails, in mV: leave them as they are. | 1150, 1500 |
| `etc_switch_threshold` | Expected error rate, in %, from which a chip whose memory cannot hold the whole ETHW DAG moves to ETC. | 70 |
| `memory_bank_accept` | `tolerant` also keeps the banks left with a few imperfect rows; `original` keeps only the perfect ones, as the stock firmware does. | `tolerant` |
| `cache_server` | The address of an ethash cache server, `[http://]host[:port][/path]`; port 8090 and path `/ethw` by default. Empty or missing: the miner computes the cache. | none |

## The ethash cache server (optional)

For each epoch, the miner asks the server set in `cache_server` for:

- `<epoch>.json`: a record with `cache_size` (in bytes), `seed` (`0x` and 64
  hexadecimal digits) and `crc32` (hexadecimal);
- `<epoch>.cache`: the raw light cache, byte for byte the one geth keeps in its
  ethash directory, without its 8-byte header;
- optionally `<epoch>.tail`, announced in the record by `dag_size`,
  `total_nodes` and `tail_start_node`: the last 4,194,304 nodes of the DAG,
  which the miner reads with range requests (`Range: bytes=N-`) instead of
  computing them.

The size and the seed must match, the cache its CRC32, and the miner
recomputes one tail node in 65,536. At the slightest difference, it computes
everything itself.

## Repository

| Path | Content |
|---|---|
| `core/` | The miner: pools, work, DAG, memory test and repair, chips, API. |
| `hal/` | Access to the hardware through `/dev/mem`. |
| `include/` | The headers. |
| `Makefile`, `tools/build.mk` | The build. |
| `tools/prepare_build.sh`, `tools/gen_dft_tables.py` | What the build takes from the official firmware. |
| `tools/test_keccak.c`, `tools/test_ethash.c`, `tools/vecteurs_ethash.h`, `tools/ref_ethash.py` | The Keccak and ethash test vectors. |

## License

GPL-3.0-or-later, see [LICENSE](LICENSE). Copyright (C) 2026 sebcorse.

`core/ethash.c` comes from libethash (GPL-3.0-or-later), `core/keccak.c` from
keccak-tiny (CC0).

No file of Sunlune's firmware is in this repository: the build extracts what
it needs from your own copy of the official archive. A binary built from it
carries data extracted from that archive (the memory test patterns): think
twice before distributing one. Jasminer and Sunlune are trademarks of their
owners.

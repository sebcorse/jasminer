/*
 * The processor cache: the one that JUDGES, not the one that mines.
 *
 * The chips compute, but nothing says what they return is right. The ethash
 * cache lets the processor REDO the computation: dram_check_dag uses it to
 * check the DAG written into the silicon, ethash_nonce_check to check each
 * nonce before submitting it. No step of the mining itself depends on it.
 */

#define _GNU_SOURCE     /* strcasestr, for the HTTP header */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>

#include "jas_hal.h"
#include "jas_http.h"
#include "jas_types.h"
#include "jas_ethash.h"
#include "jas_coin.h"

/* ====================================================================== *
 * THE CACHE REGISTRY, so that EACH CHIP finds its own.
 *
 * The key is NOT the coin but the pair (block, seed), which alone
 * determines the cache: two chips on the same coin and epoch share the same
 * one without having to say so, and two chips on different coins each find
 * theirs (a single cache chosen by *jas_coin_type would break dual mining).
 *
 * Two slots, like the firmware (cpu_cache_zil, cpu_cache_etc), and the
 * memory imposes it: three ~126 MB caches on a 509 MB machine with a 128 MB
 * RAM disk do not fit.
 *
 * The registry is the SOLE owner of the caches and jas_cache_request the
 * only construction path: a global pointer co-owning a cache led to a third
 * cache in memory and to entries freed while still in use.
 * ====================================================================== */

#define CACHES_MAX 2

static struct {
    uint64_t        block;
    ethash_h256_t   seed;
    ethash_light_t  light;
    int             busy;
    int             in_progress;   /* reserved: a thread is already building it */
} registry[CACHES_MAX];
static pthread_mutex_t registry_mutex = PTHREAD_MUTEX_INITIALIZER;

static int same_cache(int k, uint64_t block, const ethash_h256_t *seed)
{
    return registry[k].busy && registry[k].block == block &&
           memcmp(registry[k].seed.b, seed->b, 32) == 0;
}

/* The cache of this (block, seed), or NULL if not built yet. */
ethash_light_t jas_cache_find(uint64_t block, const ethash_h256_t *seed)
{
    int k;

    for (k = 0; k < CACHES_MAX; k++)
        if (same_cache(k, block, seed))
            return registry[k].light;
    return NULL;
}

/*
 * Drop a cache in. If both slots are taken, the oldest one is replaced — and
 * its memory freed, otherwise we would hold three caches for a moment, which
 * the machine cannot afford.
 */
void jas_cache_save(uint64_t block, const ethash_h256_t *seed,
                           ethash_light_t light)
{
    static int next;
    int k;

    for (k = 0; k < CACHES_MAX; k++)
        if (same_cache(k, block, seed)) {
            if (registry[k].light != light) {
                ethash_light_delete(registry[k].light);
                registry[k].light = light;
            }
            return;
        }
    for (k = 0; k < CACHES_MAX; k++)
        if (!registry[k].busy)
            break;
    if (k == CACHES_MAX) {
        k = next;
        next = (next + 1) % CACHES_MAX;
        jas_log("cache: slot %d reused (block %llu replaced)", k,
                (unsigned long long)registry[k].block);
        ethash_light_delete(registry[k].light);
    }
    registry[k].block = block;
    registry[k].seed = *seed;
    registry[k].light = light;
    registry[k].busy = 1;
    registry[k].in_progress = 0;
    jas_log("cache: slot %d = block %llu (epoch %llu)", k,
            (unsigned long long)block, (unsigned long long)(block / 30000));
}

/*
 * Build the cache of a given (block, seed) pair, in its own thread.
 *
 * Dual mining asks for two at once: ETHW epoch 827 (125 MB) and ETC epoch 418
 * (71 MB), i.e. 196 MB — which fits in the machine's 509 MB, RAM disk
 * included. Three would not fit, and that is why the registry has only two
 * slots.
 */
/* ====================================================================== *
 * THE CACHE DOWNLOADED FROM A CACHE SERVER.
 *
 * Computing the cache takes ~149 s on the Cortex-A9 (7.9 M Keccak-512 in
 * sequence). A cache server on the network can serve the
 * caches geth already keeps on disk, identical byte for byte to ours (127 MB
 * in ~2.6 s on a 1 Gbit/s link):
 *     http://<host>:8090/ethw/<epoch>.json   size, seed, crc32...
 *     http://<host>:8090/ethw/<epoch>.cache  the raw buffer
 *
 * Where to ask: the "cache_server" setting of the Miner page (see
 * jas_ethash_server); none set, the cache is computed here.
 *
 * What is checked: the record must announce the expected size AND seed
 * (otherwise it is not our coin: ETC stays computed here), then the content
 * must have the exact length and the announced CRC32. At the slightest
 * error: local computation, exactly as before.
 * ====================================================================== */

static uint32_t crc_table[256];

static void crc32_init(void)
{
    uint32_t i, j, c;

    if (crc_table[1])
        return;
    for (i = 0; i < 256; i++) {
        c = i;
        for (j = 0; j < 8; j++)
            c = (c & 1u) ? 0xedb88320u ^ (c >> 1) : c >> 1;
        crc_table[i] = c;
    }
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t n)
{
    crc = ~crc;
    while (n--)
        crc = crc_table[(crc ^ *p++) & 0xffu] ^ (crc >> 8);
    return ~crc;
}





static ethash_light_t cache_try(const char *host, int port, const char *base,
                                    uint64_t block, const ethash_h256_t *seed)
{
    uint64_t epoch = block / 30000, size = ethash_get_cachesize(block);
    char path[256], rest[4096], record[4096], hex[65];
    size_t n_rest = 0;
    long long length = 0;
    const char *v;
    uint32_t crc_exp, crc;
    ethash_light_t light;
    struct timeval t0, t1;
    int fd, k;

    gettimeofday(&t0, NULL);
    snprintf(path, sizeof(path), "%s/%llu.json", base, (unsigned long long)epoch);
    fd = jas_http_open(host, port, path, NULL, rest, &n_rest, &length);
    if (fd < 0)
        return NULL;
    memset(record, 0, sizeof(record));
    if (length <= 0 || length >= (long long)sizeof(record)
        || jas_http_read(fd, rest, n_rest, length, (uint8_t *)record, (size_t)length) != 0) {
        close(fd);
        return NULL;
    }
    close(fd);
    for (k = 0; k < 32; k++)
        snprintf(hex + 2 * k, 3, "%02x", seed->b[k]);
    v = jas_json_field(record, "\"cache_size\"");
    if (!v || strtoull(v, NULL, 10) != size) {
        jas_log("cache: %s:%d does not announce the expected size for epoch %llu — local computation",
                host, port, (unsigned long long)epoch);
        return NULL;
    }
    v = jas_json_field(record, "\"seed\"");
    if (!v || strncmp(v, "0x", 2) != 0 || strncmp(v + 2, hex, 64) != 0) {
        jas_log("cache: %s:%d — different seed for epoch %llu (another coin?) —"
                " local computation", host, port, (unsigned long long)epoch);
        return NULL;
    }
    v = jas_json_field(record, "\"crc32\"");
    if (!v)
        return NULL;
    crc_exp = (uint32_t)strtoul(v, NULL, 16);

    light = calloc(sizeof(*light), 1);
    if (!light)
        return NULL;
    light->cache = malloc((size_t)size);
    if (!light->cache) {
        free(light);
        return NULL;
    }
    snprintf(path, sizeof(path), "%s/%llu.cache", base, (unsigned long long)epoch);
    fd = jas_http_open(host, port, path, NULL, rest, &n_rest, &length);
    if (fd < 0 || jas_http_read(fd, rest, n_rest, length, light->cache, (size_t)size) != 0) {
        if (fd >= 0)
            close(fd);
        jas_log("cache: download of epoch %llu interrupted (%s:%d) — local computation",
                (unsigned long long)epoch, host, port);
        ethash_light_delete(light);
        return NULL;
    }
    close(fd);
    crc32_init();
    crc = crc32_update(0, light->cache, (size_t)size);
    if (crc != crc_exp) {
        jas_log("cache: epoch %llu downloaded but CRC32 %08x instead of %08x — local computation",
                (unsigned long long)epoch, crc, crc_exp);
        ethash_light_delete(light);
        return NULL;
    }
    light->cache_size = size;
    gettimeofday(&t1, NULL);
    jas_log("cache: epoch %llu DOWNLOADED from %s:%d in %.1f s (%llu B, crc32 %08x)",
            (unsigned long long)epoch, host, port,
            (double)(t1.tv_sec - t0.tv_sec) + (double)(t1.tv_usec - t0.tv_usec) / 1e6,
            (unsigned long long)size, crc);
    return light;
}

static ethash_light_t cache_download(uint64_t block, const ethash_h256_t *seed)
{
    char host[128], path[128];
    int port, k;

    for (k = 0; jas_ethash_server(k, host, sizeof(host), &port, path, sizeof(path)) == 0; k++) {
        ethash_light_t l = cache_try(host, port, path, block, seed);

        if (l)
            return l;
    }
    return NULL;
}

struct request_cache { uint64_t block; ethash_h256_t seed; };

static void *cache_request_thread(void *arg)
{
    struct request_cache *d = (struct request_cache *)arg;
    ethash_light_t light;
    time_t t0 = time(NULL);

    jas_name_thread("cache");
    jas_log("cache: building for epoch %llu...",
            (unsigned long long)(d->block / 30000));
    light = cache_download(d->block, &d->seed);
    if (!light)
        light = ethash_light_new_internal(ethash_get_cachesize(d->block), &d->seed);
    if (light) {
        light->block_number = d->block;
        jas_cache_save(d->block, &d->seed, light);
        jas_web_dag_publish(40);        /* first step, like the original */
        jas_log("cache: epoch %llu ready in %ld s",
                (unsigned long long)(d->block / 30000), (long)(time(NULL) - t0));
    } else {
        jas_log("cache: cannot build for epoch %llu",
                (unsigned long long)(d->block / 30000));
    }
    free(d);
    return NULL;
}

void jas_cache_request(uint64_t block, const ethash_h256_t *seed)
{
    struct request_cache *d;
    pthread_t thread;
    int k, free_slot = -1;

    /*
     * RESERVE THE SLOT BEFORE BUILDING: otherwise two chips on the same epoch
     * ask for the same cache and two threads build it in parallel, wasting
     * memory, and the second frees the one the first has just registered,
     * possibly in use.
     */
    pthread_mutex_lock(&registry_mutex);
    for (k = 0; k < CACHES_MAX; k++) {
        if (same_cache(k, block, seed) ||
            (registry[k].in_progress && registry[k].block == block &&
             memcmp(registry[k].seed.b, seed->b, 32) == 0)) {
            pthread_mutex_unlock(&registry_mutex);
            return;                 /* already there, or already being built */
        }
        if (!registry[k].busy && !registry[k].in_progress && free_slot < 0)
            free_slot = k;
    }
    /*
     * ANTI-OOM: FREE THE OLD CACHE BEFORE BUILDING THE NEW ONE. With no free
     * slot, building the new cache while the two old ones are still held
     * means three caches (~241 MB) on the machine's 509 MB, and the kernel
     * kills the miner. So a slot is freed RIGHT AWAY, under the same lock.
     * Victim = the slot of the SAME coin (epoch closest to the new one), so
     * as never to throw away the other coin's cache that still serves
     * another chip. A slot being built is never chosen.
     */
    if (free_slot < 0) {
        uint64_t e_new = block / 30000;
        uint64_t best_gap = (uint64_t)-1;
        int victim = -1;

        for (k = 0; k < CACHES_MAX; k++) {
            uint64_t e, gap;

            if (registry[k].in_progress)
                continue;
            e = registry[k].block / 30000;
            gap = e > e_new ? e - e_new : e_new - e;
            if (gap < best_gap) {
                best_gap = gap;
                victim = k;
            }
        }
        if (victim >= 0) {
            jas_log("cache: freeing epoch %llu before epoch %llu (anti-OOM)",
                    (unsigned long long)(registry[victim].block / 30000),
                    (unsigned long long)e_new);
            ethash_light_delete(registry[victim].light);
            registry[victim].light = NULL;
            registry[victim].busy = 0;
            free_slot = victim;
        }
    }
    if (free_slot >= 0) {
        registry[free_slot].block = block;
        registry[free_slot].seed = *seed;
        registry[free_slot].in_progress = 1;
        registry[free_slot].busy = 0;
    }
    pthread_mutex_unlock(&registry_mutex);
    d = malloc(sizeof(*d));
    if (!d)
        return;
    d->block = block;
    d->seed = *seed;
    if (pthread_create(&thread, NULL, cache_request_thread, d) != 0)
        free(d);
    else
        pthread_detach(thread);
}

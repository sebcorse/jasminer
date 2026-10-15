/*
 * Cache reclaim — the processor writes the end of the DAG. See cache_reclaim.h.
 */
#include "cache_reclaim.h"
#include "jas_hal.h"
#include "jas_state.h"
#include "jas_ethash.h"
#include "sl_transfer.h"
#include "jas_http.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

/* A node is 64 bytes; it is also the address unit of the manual write
 * (REG_ADDR 0x800). The manual read, on the other hand, addresses by block of
 * 128 B = 2 nodes (REG_R_ADDR 0x702): hence i>>1 and the half i&1. */
#define NODE_BYTES   64u
#define READ_WORDS      0x40u    /* READ_WORDS of single_manual_read, 2 nodes */

static uint64_t now_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ull + (uint64_t)tv.tv_usec;
}

/*
 * DAG LARGER THAN THE MEMORY, tolerated while the expected reject stays under
 * the ETC switch threshold (70 % by default, dag_calc.c). A chip carries 0xff6
 * rows per bank; beyond 0xff6 × valid banks × 8 nodes there is no memory left,
 * and writing further would wrap around to the start of the DAG and overwrite
 * it. dag_calc sets here, per chip, the last storable node; writing and
 * checking the tail stop there. The nodes beyond are missing: the hashes
 * reading them are wrong and rejected by the nonce check (the expected rate
 * dag_calc announces). 0 = no bound (the DAG fits).
 */
static uint32_t reclaim_ends[3][3];

void reclaim_set_end(int board, unsigned asic, uint32_t end)
{
    if (board >= 0 && board < 3 && asic < 3)
        reclaim_ends[board][asic] = end;
}

static uint32_t end_of(int board, unsigned asic, uint32_t total)
{
    uint32_t f = (board >= 0 && board < 3 && asic < 3) ? reclaim_ends[board][asic] : 0;

    return (f && f < total) ? f : total;
}

uint32_t reclaim_front_node(unsigned valid, unsigned cache_start_row,
                            const jas_coin_ctx *coin)
{
    uint32_t total = (uint32_t)(coin->dag_size / NODE_BYTES);
    uint64_t front;

    if (valid == 0)
        return total;                         /* nothing healthy: no reclaim */

    /* front_node = cache_start_row × valid banks × 8 (nodes of a row). */
    front = (uint64_t)cache_start_row * valid * 8ull;
    if (front >= total)
        return total;                         /* the DAG fits: nothing to write */
    return (uint32_t)front;
}

int reclaim_write_tail(int board, unsigned asic, const jas_coin_ctx *coin,
                        uint32_t front_node, struct reclaim_summary *summary)
{
    ethash_light_t light = jas_cache_find(coin->block_number, &coin->seed);
    uint32_t total = (uint32_t)(coin->dag_size / NODE_BYTES);
    uint8_t  asic_en = (uint8_t)(1u << asic);
    uint64_t t_calc = 0, t_write = 0, t0, t1, t2;
    uint32_t i;

    if (summary) {
        summary->nodes = 0;
        summary->front_node = front_node;
        summary->total_node = total;
        summary->us_compute = 0;
        summary->us_write = 0;
    }

    if (light == NULL) {
        jas_log("board %d asic %u: reclaim IMPOSSIBLE — no processor cache"
                " (epoch %llu)", board, asic,
                (unsigned long long)(coin->block_number / 30000));
        return -1;
    }
    if (front_node >= total) {
        jas_log("board %d asic %u: reclaim not needed (the DAG fits, %u nodes)",
                board, asic, total);
        return 0;
    }

    jas_log("board %d asic %u: reclaim of %u nodes (%u..%u, %u MB)",
            board, asic, total - front_node, front_node, total,
            (unsigned)(((uint64_t)(total - front_node) * NODE_BYTES) >> 20));

    for (i = front_node; i < total; i++) {
        node nd;

        t0 = now_us();
        ethash_calculate_dag_item(&nd, i, light);
        t1 = now_us();
        single_manual_write(asic_en, i, (uint16_t *)nd.words, board);
        t2 = now_us();
        t_calc += t1 - t0;
        t_write  += t2 - t1;

        /* Every 16384 writes (~1 MB): a trace line that tells "slow" from
         * "stuck", and the web page's progress bar. */
        if (((i - front_node) & 0x3fffu) == 0u) {
            unsigned done = i - front_node, all = total - front_node;
            jas_trace("board %d asic %u: reclaim %u/%u nodes", board, asic,
                      done, all);
            jas_web_dag_publish((uint8_t)(90u + (done * 10u) / (all ? all : 1u)));
        }
    }

    if (summary) {
        summary->nodes = total - front_node;
        summary->us_compute = t_calc;
        summary->us_write = t_write;
    }
    jas_log("board %d asic %u: reclaim written, %u nodes (compute %llu ms,"
            " write %llu ms)", board, asic, total - front_node,
            (unsigned long long)(t_calc / 1000), (unsigned long long)(t_write / 1000));
    return 0;
}

/*
 * GROUPED write: DAG node i only depends on the block and the seed, so it is
 * identical on all chips of the same coin. It is computed once and written at
 * once to all the chips of the mask that need it (front[p] <= i), through the
 * CBSC fill path, which already broadcasts to the three chips
 * (single_manual_write waits for each one's "ready"): one pass for the three
 * instead of ~10 min per chip.
 */
/*
 * Trial settings of the reclaim write:
 *   JASRE_RECLAIM_DELAY_US     pause after validating a node (default 0;
 *                                1000 = the original);
 *   JASRE_RECLAIM_PROBE_BEFORE  1 = wait for "ready" before each node
 *                                (default 0, see below);
 *   JASRE_RECLAIM_NO_THREAD     1 = compute and write in sequence (reference
 *                                measurement); default: compute in a thread
 *                                ahead, write in the other (the Zynq has two
 *                                cores).
 */
static unsigned reclaim_delay_us(void)
{
    const char *e = getenv("JASRE_RECLAIM_DELAY_US");
    return e ? (unsigned)strtoul(e, NULL, 0) : 0u;     /* default 0 */
}
/*
 * NO POLL BEFORE EACH NODE BY DEFAULT: single_manual_write_param already
 * waits, AFTER each write, for the "ready" of each target chip, so they are
 * ready at the next node, and three reads BEFORE would cost three of the nine
 * exchanges of each node, on the startup's longest phase. Back to the poll:
 * JASRE_RECLAIM_PROBE_BEFORE=1, or the marker file jasre_reclaim_probe in
 * /tmp (trial) or /media/configs (lasting).
 */
static int reclaim_probe_before(void)
{
    const char *e = getenv("JASRE_RECLAIM_PROBE_BEFORE");

    if (e != NULL && *e)
        return *e != '0';
    return jas_marker("jasre_reclaim_probe");
}

/* Samples read back in the written end (16 by default): JASRE_RECLAIM_CHECK
 * or the marker file /tmp/jasre_reclaim_check (a number), for trials. */
unsigned reclaim_samples_check(void)
{
    const char *e = getenv("JASRE_RECLAIM_CHECK");
    unsigned n = 16;
    char v[24];
    FILE *f;

    if (e && *e) {
        n = (unsigned)strtoul(e, NULL, 0);
    } else if ((f = fopen("/tmp/jasre_reclaim_check", "r")) != NULL) {
        if (fgets(v, sizeof(v), f))
            n = (unsigned)strtoul(v, NULL, 0);
        fclose(f);
    }
    if (n == 0)
        n = 16;
    if (n > 65536)
        n = 65536;
    return n;
}
static int reclaim_no_thread(void)
{
    const char *e = getenv("JASRE_RECLAIM_NO_THREAD");
    return e != NULL && *e == '1';
}

/* Producer (computation) / consumer (write) ring. */
#define RING_NODES  8192u          /* 512 Ko */
/*
 * THE DAG TAIL, DOWNLOADED.
 *
 * Computing the tail nodes is the reclaim's bottleneck (~0.23 ms per node on
 * one core) while the write alone takes ~0.17 ms. These nodes only depend on
 * the epoch: a server (see jas_http.h) publishes the last 4,194,304 of them
 * (<epoch>.tail, with an <epoch>.json record: dag_size, total_nodes,
 * tail_start_node...), and the miner reads, through a range request, the span
 * [first node to write, end]. The producer thread reads the stream instead of
 * computing; the first node and one node in 65,536 are recomputed here and
 * compared. At the slightest difference, or if the stream breaks, local
 * computation resumes from the current node: never worse than without it.
 */
#define TAIL_CHECK_STEP 65536u

struct tail_stream {
    int      fd;                        /* -1: no stream, we compute */
    uint8_t  buffer[65536];
    size_t   n, pos;
    uint32_t downloaded, checks, gaps;
};

static int tail_stream_open(struct tail_stream *f, const jas_coin_ctx *coin, uint32_t imin,
                           uint32_t total)
{
    char host[128], path[256], base[128], record[4096], rest[4096], range[64], hex[65];
    size_t n_rest = 0;
    long long length = 0;
    const char *v;
    int port, k, fd, j;
    uint64_t epoch = coin->block_number / 30000;

    f->fd = -1;
    for (j = 0; j < 32; j++)
        snprintf(hex + 2 * j, 3, "%02x", coin->seed.b[j]);
    for (k = 0; jas_ethash_server(k, host, sizeof(host), &port, base, sizeof(base)) == 0; k++) {
        uint32_t start;

        snprintf(path, sizeof(path), "%s/%llu.json", base, (unsigned long long)epoch);
        fd = jas_http_open(host, port, path, NULL, rest, &n_rest, &length);
        memset(record, 0, sizeof(record));
        if (fd < 0)
            continue;
        if (length <= 0 || length >= (long long)sizeof(record)
            || jas_http_read(fd, rest, n_rest, length, (uint8_t *)record, (size_t)length) != 0) {
            close(fd);
            continue;
        }
        close(fd);
        v = jas_json_field(record, "\"dag_size\"");
        if (!v || strtoull(v, NULL, 10) != coin->dag_size)
            continue;
        v = jas_json_field(record, "\"seed\"");
        if (!v || strncmp(v, "0x", 2) != 0 || strncmp(v + 2, hex, 64) != 0)
            continue;
        v = jas_json_field(record, "\"total_nodes\"");
        if (!v || strtoul(v, NULL, 10) != total)
            continue;
        v = jas_json_field(record, "\"tail_start_node\"");
        if (!v)
            continue;
        start = (uint32_t)strtoul(v, NULL, 10);
        if (imin < start) {
            jas_log("reclaim: DAG tail served from node %u, %u needed — local computation",
                    start, imin);
            continue;
        }
        snprintf(path, sizeof(path), "%s/%llu.tail", base, (unsigned long long)epoch);
        snprintf(range, sizeof(range), "bytes=%llu-", (unsigned long long)(imin - start) * 64u);
        fd = jas_http_open(host, port, path, range, (char *)f->buffer, &f->n, &length);
        if (fd < 0)
            continue;
        if (length != (long long)(total - imin) * 64) {
            close(fd);
            continue;
        }
        f->fd = fd;
        f->pos = 0;
        jas_log("reclaim: DAG tail DOWNLOADED from %s:%d (nodes %u..%u, %llu MB)",
                host, port, imin, total, (unsigned long long)((uint64_t)(total - imin) * 64u >> 20));
        return 0;
    }
    return -1;
}

/* The next node of the stream into `dst`. 0 if read, -1 if the stream ended/broke. */
static int tail_stream_node(struct tail_stream *f, node *dst)
{
    size_t copy = 0;

    while (copy < sizeof(node)) {
        if (f->pos == f->n) {
            ssize_t r = recv(f->fd, f->buffer, sizeof(f->buffer), 0);

            if (r <= 0)
                return -1;
            f->n = (size_t)r;
            f->pos = 0;
        }
        {
            size_t k = f->n - f->pos;

            if (k > sizeof(node) - copy)
                k = sizeof(node) - copy;
            memcpy((uint8_t *)dst + copy, f->buffer + f->pos, k);
            f->pos += k;
            copy += k;
        }
    }
    return 0;
}

static void tail_stream_close(struct tail_stream *f, const char *reason, uint32_t i)
{
    if (f->fd < 0)
        return;
    close(f->fd);
    f->fd = -1;
    if (reason)
        jas_log("reclaim: DAG tail stream dropped at node %u (%s) — local"
                " computation for the rest", i, reason);
}

struct ring {
    node           *buf;
    uint32_t        imin, total;
    volatile uint32_t prod, cons;     /* absolute node counters */
    ethash_light_t  light;
    pthread_mutex_t m;
    pthread_cond_t  c_full, c_empty;
    uint64_t        us_compute;
    struct tail_stream *stream;              /* NULL or fd < 0: local computation */
};

static void *compute_thread(void *arg)
{
    struct ring *a = arg;
    uint32_t i;

    for (i = a->imin; i < a->total; i++) {
        uint64_t t0, t1;

        pthread_mutex_lock(&a->m);
        while (a->prod - a->cons >= RING_NODES)
            pthread_cond_wait(&a->c_full, &a->m);
        pthread_mutex_unlock(&a->m);

        t0 = now_us();
        if (a->stream && a->stream->fd >= 0) {
            node *dst = &a->buf[i % RING_NODES];

            if (tail_stream_node(a->stream, dst) != 0) {
                tail_stream_close(a->stream, "stream interrupted", i);
                ethash_calculate_dag_item(dst, i, a->light);
            } else {
                a->stream->downloaded++;
                if (i == a->imin || (i - a->imin) % TAIL_CHECK_STEP == 0) {
                    node ref;

                    ethash_calculate_dag_item(&ref, i, a->light);
                    a->stream->checks++;
                    if (memcmp(&ref, dst, sizeof(node)) != 0) {
                        a->stream->gaps++;
                        *dst = ref;
                        tail_stream_close(a->stream, "downloaded node differs from the computation", i);
                    }
                }
            }
        } else
            ethash_calculate_dag_item(&a->buf[i % RING_NODES], i, a->light);
        t1 = now_us();
        a->us_compute += t1 - t0;

        pthread_mutex_lock(&a->m);
        a->prod = i + 1u;
        pthread_cond_signal(&a->c_empty);
        pthread_mutex_unlock(&a->m);
    }
    return NULL;
}

int reclaim_write_tail_group(int board, const jas_coin_ctx *coin,
                               const uint32_t front[3], uint8_t mask,
                               struct reclaim_summary *summary)
{
    ethash_light_t light = jas_cache_find(coin->block_number, &coin->seed);
    uint32_t total = (uint32_t)(coin->dag_size / NODE_BYTES);
    uint32_t imin = total;
    uint64_t t_calc = 0, t_write = 0, t_start, t0, t1;
    uint32_t i;
    unsigned p, n_chips = 0;
    unsigned delay = reclaim_delay_us();
    int probe = reclaim_probe_before(), no_thread = reclaim_no_thread();
    struct ring a;
    struct tail_stream *stream = NULL;
    pthread_t thread;
    int thread_started = 0;

    uint32_t end_max = 0, ends[3];

    mask &= 7u;
    for (p = 0; p < 3; p++) {
        ends[p] = end_of(board, p, total);
        if (mask >> p & 1) {
            n_chips++;
            if (front[p] < imin)
                imin = front[p];
            if (ends[p] > end_max)
                end_max = ends[p];
        }
    }
    if (end_max < total)
        jas_log("board %d: DAG tail BOUNDED at node %u — %u nodes (%u MB)"
                " beyond the chips' memory, left missing", board, end_max, total - end_max,
                (unsigned)(((uint64_t)(total - end_max) * NODE_BYTES) >> 20));
    if (summary) {
        summary->nodes = 0;
        summary->front_node = imin;
        summary->total_node = total;
        summary->us_compute = 0;
        summary->us_write = 0;
    }
    if (n_chips == 0 || imin >= end_max)
        return 0;
    if (light == NULL) {
        jas_log("board %d: grouped reclaim IMPOSSIBLE — no processor cache"
                " (epoch %llu)", board,
                (unsigned long long)(coin->block_number / 30000));
        return -1;
    }

    jas_log("board %d: GROUPED reclaim, chips 0x%x, %u nodes (%u..%u, %u MB)"
            " computed once and written to the %u chips — delay %u us, poll"
            " before %d, compute %s", board, mask, total - imin, imin, total,
            (unsigned)(((uint64_t)(total - imin) * NODE_BYTES) >> 20), n_chips,
            delay, probe, no_thread ? "in sequence" : "in a thread ahead");

    memset(&a, 0, sizeof(a));
    /* The producer stops at the same bound as the write (the stream is
     * opened with the real size, for its check, and closed earlier). */
    a.imin = imin; a.total = end_max; a.prod = imin; a.cons = imin; a.light = light;
    stream = calloc(1, sizeof(*stream));
    if (stream) {
        stream->fd = -1;
        if (!no_thread && tail_stream_open(stream, coin, imin, total) == 0)
            a.stream = stream;
    }
    if (!no_thread) {
        a.buf = malloc((size_t)RING_NODES * sizeof(node));
        if (a.buf) {
            pthread_mutex_init(&a.m, NULL);
            pthread_cond_init(&a.c_full, NULL);
            pthread_cond_init(&a.c_empty, NULL);
            if (pthread_create(&thread, NULL, compute_thread, &a) == 0)
                thread_started = 1;
            else
                jas_log("board %d: compute thread impossible, computing in sequence", board);
        } else
            jas_log("board %d: ring of %u nodes impossible, computing in sequence",
                    board, RING_NODES);
    }

    t_start = now_us();
    for (i = imin; i < end_max; i++) {
        node nd;
        node *src;
        uint8_t m = 0;

        for (p = 0; p < 3; p++)
            if ((mask >> p & 1) && front[p] <= i && i < ends[p])
                m |= (uint8_t)(1u << p);

        if (thread_started) {
            pthread_mutex_lock(&a.m);
            while (a.prod <= i)
                pthread_cond_wait(&a.c_empty, &a.m);
            pthread_mutex_unlock(&a.m);
            src = &a.buf[i % RING_NODES];
        } else {
            t0 = now_us();
            ethash_calculate_dag_item(&nd, i, light);
            t_calc += now_us() - t0;
            src = &nd;
        }

        t0 = now_us();
        single_manual_write_param(m, i, (uint16_t *)src->words, board, delay, probe);
        t1 = now_us();
        t_write += t1 - t0;

        if (thread_started) {
            pthread_mutex_lock(&a.m);
            a.cons = i + 1u;
            pthread_cond_signal(&a.c_full);
            pthread_mutex_unlock(&a.m);
        }

        if (((i - imin) & 0x3fffu) == 0u) {
            unsigned done = i - imin, all = end_max - imin;
            jas_trace("board %d: grouped reclaim %u/%u nodes", board, done, all);
            jas_web_dag_publish((uint8_t)(90u + (done * 10u) / (all ? all : 1u)));
        }
    }

    if (thread_started) {
        pthread_join(thread, NULL);
        t_calc = a.us_compute;
        pthread_mutex_destroy(&a.m);
        pthread_cond_destroy(&a.c_full);
        pthread_cond_destroy(&a.c_empty);
    }
    free(a.buf);
    if (stream) {
        if (a.stream)
            jas_log("reclaim: DAG tail — %u nodes downloaded, %u checked, %u gap(s)%s",
                    stream->downloaded, stream->checks, stream->gaps,
                    stream->fd >= 0 ? "" : " (stream closed before the end)");
        tail_stream_close(stream, NULL, 0);
        free(stream);
    }

    if (summary) {
        summary->nodes = end_max - imin;
        summary->us_compute = t_calc;
        summary->us_write = t_write;
    }
    jas_log("board %d: grouped reclaim written, %u nodes for %u chips (%s %llu"
            " ms%s, write %llu ms, total %llu ms, %.2f ms/node)", board, end_max - imin,
            n_chips, a.stream ? "read+check" : "compute", (unsigned long long)(t_calc / 1000),
            thread_started ? " recouvert" : "", (unsigned long long)(t_write / 1000),
            (unsigned long long)((now_us() - t_start) / 1000),
            (double)(now_us() - t_start) / 1000.0 / (double)(end_max - imin));
    return 0;
}

/* ------------------------------------------------------------------------
 * Function 3 — DAG check and repair (the check of the reclaim's tail is a
 * special case of it: reclaim_check_tail).
 * ---------------------------------------------------------------------- */
int dag_check_repair(int board, unsigned asic, const jas_coin_ctx *coin,
                         uint32_t start, uint32_t end, unsigned samples,
                         int repair, unsigned *repaired)
{
    ethash_light_t light = jas_cache_find(coin->block_number, &coin->seed);
    uint32_t total = (uint32_t)(coin->dag_size / NODE_BYTES);
    uint8_t  asic_en = (uint8_t)(1u << asic);
    uint32_t span;
    unsigned k, wrong = 0, rep = 0, shown = 0;

    if (repaired)
        *repaired = 0;
    if (light == NULL)
        return -1;
    if (end > total)
        end = total;
    if (start >= end)
        return 0;
    if (samples == 0)
        samples = 1;

    span = end - start;
    for (k = 0; k < samples; k++) {
        node exp, loaded;
        uint16_t block[READ_WORDS];
        const uint16_t *half;
        uint32_t i = start + (uint32_t)((uint64_t)span * k / samples);

        if (i >= end)
            i = end - 1;

        ethash_calculate_dag_item(&exp, i, light);

        /* Read by block of 2 nodes (ALU 0); the node is the half i&1. */
        if (single_manual_read(asic_en, 0, i >> 1, block, board) < 0) {
            jas_log("board %d asic %u: dag check, silent read at node %u",
                    board, asic, i);
            wrong++;
            continue;
        }
        half = block + ((i & 1u) ? 0x20u : 0u);
        if (memcmp(half, exp.bytes, NODE_BYTES) == 0)
            continue;

        wrong++;
        memcpy(loaded.bytes, half, NODE_BYTES);
        if (shown++ < 8)
            jas_log("board %d asic %u: dag check, node %u WRONG | read %08x"
                    " %08x | exp %08x %08x%s",
                    board, asic, i, loaded.words[0], loaded.words[1],
                    exp.words[0], exp.words[1], repair ? " -> reecriture" : "");
        if (!repair)
            continue;

        /* Repair: rewrite the right value, then read back to judge. */
        single_manual_write(asic_en, i, (uint16_t *)exp.words, board);
        if (single_manual_read(asic_en, 0, i >> 1, block, board) == 0
            && memcmp(block + ((i & 1u) ? 0x20u : 0u), exp.bytes, NODE_BYTES) == 0) {
            rep++;
        } else if (shown <= 8) {
            jas_log("board %d asic %u: node %u REWRITTEN WITHOUT EFFECT (faulty"
                    " bank? block %u)", board, asic, i, i >> 1);
        }
    }

    if (repaired)
        *repaired = rep;
    jas_log("board %d asic %u: dag check [%u..%u) %u samples: %u wrong%s",
            board, asic, start, end, samples, wrong,
            repair ? "" : "");
    if (repair && wrong)
        jas_log("board %d asic %u: dag repair: %u/%u nodes rewritten successfully",
                board, asic, rep, wrong);
    return (int)wrong;
}

int reclaim_check_tail(int board, unsigned asic, const jas_coin_ctx *coin,
                          uint32_t front_node, unsigned samples)
{
    uint32_t total = end_of(board, asic, (uint32_t)(coin->dag_size / NODE_BYTES));
    int r;

    if (front_node >= total)
        return 0;
    r = dag_check_repair(board, asic, coin, front_node, total, samples,
                             0, NULL);
    if (r == 0)
        jas_log("board %d asic %u: reclaim check OK (%u samples)",
                board, asic, samples ? samples : 1u);
    return r;
}

/* ------------------------------------------------------------------------
 * Function 2: diagnosis of the cache INSIDE the chip.
 *
 * The chip's cache is, by construction, the processor's ethash cache
 * (jas_cache_find): it can be read back block by block and compared.
 * dram_read_last_cache_node reads the EVEN node of block
 * `base_addr + cache_size>>7` through ALU `manual_sel`: for block k of the
 * cache we pass base0 + k, with base0 = (cache_init_addr>>7) − (cache>>7),
 * exactly the path of dag_calc's node 0 check.
 *
 * Verdict per wrong node, reading again through an ALU of ANOTHER mesh:
 *   - both ALUs read the same wrong value      → DATA (faulty bank);
 *   - the second reads the right value          → read PATH of the 1st;
 *   - otherwise                                 → inconsistent (bus noise?).
 * ---------------------------------------------------------------------- */
#define ALU_MAIN   1u     /* the one of the node 0 check (dag_calc) */
#define ALU_ALTERNATIVE  31u    /* mesh 1, alu 1: another mesh */

int cache_diagnose(int board, unsigned asic, const jas_coin_ctx *coin,
                        unsigned samples, uint32_t *last_block_wrong)
{
    ethash_light_t light = jas_cache_find(coin->block_number, &coin->seed);
    unsigned valid = jas_good_bank_num_cbsc[board * 3 + asic] & 0xffffu;
    uint8_t  asic_en = (uint8_t)(1u << asic);
    uint32_t base, base0, blocks;
    unsigned k, wrong = 0, data = 0, path = 0, other = 0, silent = 0, shown = 0;

    if (last_block_wrong)
        *last_block_wrong = 0xffffffffu;
    if (light == NULL || valid == 0)
        return -1;
    if (samples == 0)
        samples = 1;

    base  = (uint32_t)(dag_cache_init_addr(valid, coin->cache_size) >> 7);
    base0 = base - (uint32_t)(coin->cache_size >> 7);
    blocks = (uint32_t)(coin->cache_size >> 7);
    if (blocks == 0)
        return -1;

    for (k = 0; k < samples; k++) {
        uint32_t b = (uint32_t)((uint64_t)blocks * k / samples);
        const uint8_t *exp = (const uint8_t *)light->cache
                           + (uint64_t)(2u * b) * NODE_BYTES;   /* even node */
        uint16_t n1[0x20], n2[0x20];
        node loaded;

        if (b >= blocks)
            b = blocks - 1;

        if (dram_read_last_cache_node(asic_en, ALU_MAIN, base0 + b,
                                      coin->cache_size, board, n1) != 0) {
            silent++;
            continue;
        }
        if (memcmp(n1, exp, NODE_BYTES) == 0)
            continue;

        wrong++;
        memcpy(loaded.bytes, n1, NODE_BYTES);
        if (dram_read_last_cache_node(asic_en, ALU_ALTERNATIVE, base0 + b,
                                      coin->cache_size, board, n2) != 0) {
            other++;
            if (shown++ < 6)
                jas_log("board %d asic %u: cache block %u WRONG (ALU %u), ALU"
                        " %u silent | read %08x %08x", board, asic, b, ALU_MAIN,
                        ALU_ALTERNATIVE, loaded.words[0], loaded.words[1]);
            continue;
        }
        if (memcmp(n2, n1, NODE_BYTES) == 0) {
            data++;
            if (last_block_wrong)
                *last_block_wrong = base + b;
            if (shown++ < 6)
                jas_log("board %d asic %u: cache block %u WRONG = DATA in"
                        " memory (ALU %u and %u agree) | read %08x %08x | exp"
                        " %08x %08x",
                        board, asic, b, ALU_MAIN, ALU_ALTERNATIVE,
                        loaded.words[0], loaded.words[1],
                        ((const uint32_t *)exp)[0], ((const uint32_t *)exp)[1]);
        } else if (memcmp(n2, exp, NODE_BYTES) == 0) {
            path++;
            if (shown++ < 6)
                jas_log("board %d asic %u: cache block %u WRONG = read PATH of"
                        " ALU %u (ALU %u reads right)", board, asic, b,
                        ALU_MAIN, ALU_ALTERNATIVE);
        } else {
            other++;
            if (shown++ < 6)
                jas_log("board %d asic %u: cache block %u WRONG, two different"
                        " wrong reads (noise?)", board, asic, b);
        }
    }

    jas_log("board %d asic %u: cache diagnosis, %u blocks out of %u: %u wrong"
            " (data %u, path %u, inconsistent %u), %u silent reads",
            board, asic, samples, blocks, wrong, data, path, other, silent);
    return (int)wrong;
}

/* ------------------------------------------------------------------------
 * Function 1: hot removal of a bank + cache/DAG rebuild.
 *
 * Block → bank mapping: the bench's scrub one (main_stress:
 * `bank = (A / 4) % bank_num`, A in blocks of 128 B). It is APPROXIMATE; what
 * validates it is what follows: cache and DAG rebuilt and checked on the new
 * map. It avoids replaying the full self-test (DFT ~5 min + CBSC ~1.5 min)
 * when a chip fails its cache: marking, remapping, soft reset, cache (~5 s)
 * and DAG (~50 s).
 *
 * TODO: dag_calc_asic builds the FULL DAG; this rebuild should go through the
 * reclaim path as dag_calc does (native prefix + tail written by the
 * processor). Not tried on silicon yet.
 * ---------------------------------------------------------------------- */
#define BANKS_PER_CHIP  4080u
#define BANKS_PER_MESH  1020u

int bank_remove_hot(int board, unsigned asic, const jas_coin_ctx *coin,
                           uint32_t block_wrong)
{
    unsigned rank = (unsigned)board * 3u + asic;
    unsigned valid = jas_good_bank_num_cbsc[rank] & 0xffffu;
    unsigned bank, idx, mesh;
    int rc;

    if (valid == 0 || asic >= 3)
        return -1;

    bank = (unsigned)((block_wrong / 4u) % valid);
    idx    = asic * BANKS_PER_CHIP + bank;
    mesh   = bank / BANKS_PER_MESH;

    if (!jas_bank_good[board][idx] && !jas_bank_partial[board][idx]) {
        jas_log("board %d asic %u: bank %u (block %u) already out of service",
                board, asic, bank, block_wrong);
        return -2;
    }

    jas_bank_good[board][idx]    = 0;
    jas_bank_partial[board][idx] = 0;
    jas_bank_bad[board][idx]     = 1;
    jas_good_bank_num_cbsc[rank] = (uint16_t)(valid - 1u);
    if (jas_good_bank_num_cbsc_mesh[rank * 4u + mesh] > 0)
        jas_good_bank_num_cbsc_mesh[rank * 4u + mesh]--;

    jas_log("board %d asic %u: bank %u (mesh %u, block %u) REMOVED while"
            " running: %u -> %u banks; remap then cache + DAG, no self-test",
            board, asic, bank, mesh, block_wrong, valid, valid - 1u);

    config_remap_info(board, (uint8_t)asic, 1);
    rc = dag_calc_asic(board, asic, coin);
    if (rc != 0) {
        jas_log("board %d asic %u: rebuild after removing bank %u FAILED (%d)", board, asic, bank, rc);
        return -1;
    }
    jas_log("board %d asic %u: cache + DAG rebuilt on %u banks", board, asic,
            valid - 1u);
    return (int)bank;
}

/* ------------------------------------------------------------------------
 * CACHE DIAGNOSIS ON FAILURE (on by default, JASRE_CACHE_DIAG=0 turns it
 * off): reads only, plus a processor write round trip on node 0's block
 * alone. The map of wrong blocks gives the extent of the damage (one block,
 * one bank row, the whole cache); the round trip tells a DRAM row that does
 * not hold (it reads back an imprint, whatever was written) from a write the
 * chip's engine lost.
 * ---------------------------------------------------------------------- */
int jas_cache_diag_active(void)
{
    static int loaded = 0, active = 0;

    if (!loaded) {
        const char *e = getenv("JASRE_CACHE_DIAG");

        active = !(e != NULL && *e == '0');   /* on by default; =0 turns it off */
        loaded = 1;
        if (!active)
            jas_log("cache diagnosis: OFF (JASRE_CACHE_DIAG=0)");
    }
    return active;
}

int cache_read_block(int board, unsigned asic, const jas_coin_ctx *coin,
                    uint32_t block, uint16_t node_buf[0x20])
{
    /* dram_read_last_cache_node reads at base_addr + (cache>>7): step back. */
    return dram_read_last_cache_node((uint8_t)(1u << asic), ALU_MAIN,
                                     block - (uint32_t)(coin->cache_size >> 7),
                                     coin->cache_size, board, node_buf);
}

int cache_map(int board, unsigned asic, const jas_coin_ctx *coin)
{
    return cache_map_v(board, asic, coin, NULL);
}

int cache_map_v(int board, unsigned asic, const jas_coin_ctx *coin,
                          struct map_verdict *v)
{
    ethash_light_t light = jas_cache_find(coin->block_number, &coin->seed);
    unsigned valid = jas_good_bank_num_cbsc[board * 3 + asic] & 0xffffu;
    uint8_t  asic_en = (uint8_t)(1u << asic);
    uint32_t base, blocks, offs[10], block_data = 0xffffffffu;
    unsigned k, n = 0, wrong_elsewhere = 0, read = 0, wrong = 0, data = 0, path = 0, others = 0;

    if (v)
        memset(v, 0, sizeof(*v));
    if (light == NULL || valid == 0)
        return -1;
    base  = (uint32_t)(dag_cache_init_addr(valid, coin->cache_size) >> 7);
    blocks = (uint32_t)(coin->cache_size >> 7);
    if (blocks < 8)
        return -1;

    offs[n++] = 0; offs[n++] = 1; offs[n++] = 2; offs[n++] = 3;   /* node 0's bank row */
    offs[n++] = 4;                                                /* next bank, same row */
    if (4u * valid < blocks) offs[n++] = 4u * valid;           /* same bank, next row */
    offs[n++] = blocks / 4; offs[n++] = blocks / 2; offs[n++] = (3u * blocks) / 4;
    offs[n++] = blocks - 1;

    jas_log("board %d asic %u: cache MAP (start block %u, %u banks, %u blocks)"
            " — bank row = 4 blocks", board, asic, base, valid, blocks);
    for (k = 0; k < n; k++) {
        uint32_t off = offs[k], b = base + off;
        const uint32_t *exp = (const uint32_t *)((const uint8_t *)light->cache
                              + (uint64_t)(2u * off) * NODE_BYTES);
        uint16_t n1[0x20], n2[0x20];
        const uint32_t *l1 = (const uint32_t *)n1, *l2 = (const uint32_t *)n2;
        const char *verdict;

        if (dram_read_last_cache_node(asic_en, ALU_MAIN,
                                      b - (uint32_t)(coin->cache_size >> 7),
                                      coin->cache_size, board, n1) != 0) {
            jas_log("board %d asic %u: map +%u (bank~%u row+%u): SILENT",
                    board, asic, off, (unsigned)((b / 4u) % valid),
                    (unsigned)(off / (4u * valid)));
            continue;
        }
        read++;
        if (memcmp(n1, exp, NODE_BYTES) == 0) {
            jas_log("board %d asic %u: map +%u (bank~%u row+%u): OK",
                    board, asic, off, (unsigned)((b / 4u) % valid),
                    (unsigned)(off / (4u * valid)));
            continue;
        }
        wrong++;
        if (off != 0)
            wrong_elsewhere++;
        if (dram_read_last_cache_node(asic_en, ALU_ALTERNATIVE,
                                      b - (uint32_t)(coin->cache_size >> 7),
                                      coin->cache_size, board, n2) != 0) {
            verdict = "WRONG, 2nd ALU silent";
            others++;
        } else if (memcmp(n2, n1, NODE_BYTES) == 0) {
            verdict = "WRONG = DATA (2 ALUs agree)";
            data++;
            block_data = b;
        } else if (memcmp(n2, exp, NODE_BYTES) == 0) {
            verdict = "WRONG = PATH of ALU 1 (the other reads right)";
            path++;
        } else {
            verdict = "WRONG, 2 different wrong reads";
            others++;
        }
        jas_log("board %d asic %u: map +%u (bank~%u row+%u): %s | read %08x"
                " %08x %08x %08x | exp %08x %08x %08x %08x",
                board, asic, off, (unsigned)((b / 4u) % valid),
                (unsigned)(off / (4u * valid)), verdict,
                l1[0], l1[1], l1[2], l1[3], exp[0], exp[1], exp[2], exp[3]);
        (void)l2;
    }
    jas_log("board %d asic %u: cache map: %u probes read, %u wrong outside"
            " block 0 (data %u, path %u, other %u) — %s", board, asic, read,
            wrong_elsewhere, data, path, others,
            read == 0 ? "unusable" :
            wrong_elsewhere == 0 ? "defect LOCALIZED in node 0's block (or none)" :
            wrong_elsewhere >= read - 1 ? "the WHOLE cache is wrong (propagation)" :
            "PARTIAL defect");
    if (v) {
        v->read = read; v->wrong = wrong; v->wrong_outside0 = wrong_elsewhere;
        v->data = data; v->path = path; v->others = others;
        v->block_data = block_data;
    }
    return read ? (int)wrong_elsewhere : -1;
}

int cache_cpu_write_attempt(int board, unsigned asic, const jas_coin_ctx *coin)
{
    ethash_light_t light = jas_cache_find(coin->block_number, &coin->seed);
    unsigned valid = jas_good_bank_num_cbsc[board * 3 + asic] & 0xffffu;
    uint8_t  asic_en = (uint8_t)(1u << asic);
    uint32_t base;
    uint16_t readback[0x20];
    const uint32_t *l = (const uint32_t *)readback, *exp;

    if (light == NULL || valid == 0)
        return -1;
    base = (uint32_t)(dag_cache_init_addr(valid, coin->cache_size) >> 7);
    exp  = (const uint32_t *)light->cache;

    /* Node 0 -> node address 2*base, node 1 -> 2*base+1 (= block base). */
    single_manual_write(asic_en, 2u * base, (uint16_t *)light->cache, board);
    single_manual_write(asic_en, 2u * base + 1u,
                        (uint16_t *)((uint8_t *)light->cache + NODE_BYTES), board);
    usleep(2000);
    if (cache_read_block(board, asic, coin, base, readback) != 0) {
        jas_log("board %d asic %u: processor round trip at block %u: SILENT"
                " read-back", board, asic, base);
        return -1;
    }
    if (memcmp(readback, light->cache, NODE_BYTES) == 0) {
        jas_log("board %d asic %u: processor round trip at block %u: IDENTICAL"
                " — the memory holds the data, the chip's engine had lost the"
                " write; block REPAIRED", board, asic, base);
        return 1;
    }
    jas_log("board %d asic %u: processor round trip at block %u: DIFFERENT —"
            " faulty memory at this spot | read %08x %08x %08x %08x | written"
            " %08x %08x %08x %08x", board, asic, base,
            l[0], l[1], l[2], l[3], exp[0], exp[1], exp[2], exp[3]);
    return 0;
}

int jas_cache_cpu_active(void)
{
    static int loaded = 0, active = 0;

    if (!loaded) {
        const char *e = getenv("JASRE_CACHE_CPU");

        active = !(e != NULL && *e == '0');   /* on by default; =0 turns it off */
        loaded = 1;
        if (!active)
            jas_log("processor-written cache: OFF (JASRE_CACHE_CPU=0)");
    }
    return active;
}

int cache_write_by_cpu(int board, unsigned asic, const jas_coin_ctx *coin,
                         unsigned samples)
{
    ethash_light_t light = jas_cache_find(coin->block_number, &coin->seed);
    unsigned valid = jas_good_bank_num_cbsc[board * 3 + asic] & 0xffffu;
    uint8_t  asic_en = (uint8_t)(1u << asic);
    uint32_t base, n, j, blocks;
    uint64_t t0;
    unsigned k, wrong = 0, silent = 0;

    if (light == NULL || valid == 0)
        return -1;
    base  = (uint32_t)(dag_cache_init_addr(valid, coin->cache_size) >> 7);
    n     = (uint32_t)(coin->cache_size / NODE_BYTES);
    blocks = (uint32_t)(coin->cache_size >> 7);
    if (n == 0 || blocks == 0)
        return -1;

    jas_log("board %d asic %u: CACHE WRITTEN BY THE PROCESSOR — %u nodes"
            " (%u MB) from node address %u", board, asic, n,
            (unsigned)(coin->cache_size >> 20), 2u * base);
    t0 = now_us();
    for (j = 0; j < n; j++) {
        single_manual_write_param(asic_en, 2u * base + j,
                                  (uint16_t *)((uint8_t *)light->cache
                                               + (uint64_t)j * NODE_BYTES),
                                  board, 0u, 1);
        if ((j & 0x3ffffu) == 0u && j)
            jas_trace("board %d asic %u: CPU cache %u/%u nodes", board, asic, j, n);
    }
    jas_log("board %d asic %u: cache written by the processor in %llu ms (%.2f ms/node)",
            board, asic, (unsigned long long)((now_us() - t0) / 1000),
            (double)(now_us() - t0) / 1000.0 / (double)n);

    if (samples == 0)
        samples = 1;
    for (k = 0; k < samples; k++) {
        uint32_t b = (uint32_t)((uint64_t)blocks * k / samples);
        const uint8_t *exp = (const uint8_t *)light->cache + (uint64_t)(2u * b) * NODE_BYTES;
        uint16_t loaded[0x20];

        if (b >= blocks)
            b = blocks - 1;
        if (cache_read_block(board, asic, coin, base + b, loaded) != 0) {
            silent++;
            continue;
        }
        if (memcmp(loaded, exp, NODE_BYTES) != 0) {
            uint32_t l2[2], a2[2];

            memcpy(l2, loaded, sizeof(l2));
            memcpy(a2, exp, sizeof(a2));
            wrong++;
            if (wrong <= 4)
                jas_log("board %d asic %u: CPU cache block +%u WRONG | read %08x %08x | exp %08x %08x",
                        board, asic, b, l2[0], l2[1], a2[0], a2[1]);
        }
    }
    jas_log("board %d asic %u: CPU cache checked, %u samples: %u wrong, %u silent — %s",
            board, asic, samples, wrong, silent,
            (wrong == 0 && silent == 0) ? "ALL GOOD, the memory holds a correct cache"
                                      : "DEFECT");
    return (int)wrong;
}

/* ------------------------------------------------------------------------
 * The enable switch, ON by default: an environment variable, like the other
 * trial settings (JASRE_SKIP_DFT, JASRE_MRR_ACCEPT...), settable from
 * jasminer.sh or the bench's command line without touching the flash.
 * TODO: the `cache_reclaim` key of jasminer.json.
 * ---------------------------------------------------------------------- */
int jas_cache_reclaim_active(void)
{
    static int loaded = 0, active = 0;

    if (!loaded) {
        const char *e = getenv("JASRE_CACHE_RECLAIM");

        /* ON BY DEFAULT (full DAG on all chips, a few % of rejects instead of
         * ~20 % with a truncated one); JASRE_CACHE_RECLAIM=0 restores the
         * stock behaviour (truncated DAG). */
        active = !(e != NULL && *e == '0');
        loaded = 1;
        jas_log("cache reclaim: %s — the chip builds the DAG prefix, the"
                " processor writes the tail", active ? "ON" : "OFF (JASRE_CACHE_RECLAIM=0)");
    }
    return active;
}

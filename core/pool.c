/*
 * The pool, in getwork (ethproxy).
 *
 * The firmware speaks getwork, Stratum V1 and ethproxy, but the machines
 * are configured for "getwork", which is ethproxy (see pool_login below).
 * Two methods are enough:
 *
 *     -> {"id":N,"method":"eth_getWork","params":[]}
 *     <- {"id":N,"result":["0x<header>","0x<seed>","0x<target>", height]}
 *
 *     -> {"id":N,"method":"eth_submitWork","params":["0x<nonce>","0x<header>","0x<mix>"]}
 *     <- {"id":N,"result":true}
 *
 * Some pools add the height as a fourth value, others do not; without it,
 * ethash_get_blocknum deduces the block number from the seed.
 *
 * No TLS: not needed for a plain pool.
 */

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_coin.h"
#include "jas_ethash.h"
#include <pthread.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define BUFFER      8192
#define POLL_SEC    1        /* pool polling rate */

/*
 * One jas_pool per slot: four slots, two groups (one per coin) of a main
 * pool and a backup. Everything that describes a CONNECTION lives here:
 * socket, leftover buffer p->rest, JSON-RPC id counter, host, user. Each
 * network thread owns one and touches only it. The cache registry and the
 * chip contexts stay global; a chip finds its pool with jas_pool_of_chip().
 */
typedef struct {
    int      sock;
    char     rest[BUFFER];
    unsigned rest_n;
    unsigned next_id;
    /*
     * Per-pool counters, as the original keeps them (parse_submit_result,
     * 0x3c555): the web page's statistics come from them.
     */
    unsigned accepts;
    unsigned rejects;
    /*
     * Works RECEIVED from this pool: a quantity of its own, not accepts +
     * rejects (the original reports them separately).
     */
    unsigned works;
    unsigned lines_pass;          /* lines absorbed without yielding */
    struct timeval pass_t0;
    unsigned trace_received;
    unsigned beat_rounds;
    time_t   beat_t0;
    char     host[128];
    int      port;
    char     user[192];
    char     name[64];      /* the part after the dot: the rig name */
    char     pass[64];
    time_t   last_receive;
    time_t   last_request;
    time_t   last_dns_failed;   /* log brake when the host does not resolve */
    /* The dropped-off work, with its lock and flag: pool_task_t. */
    pthread_mutex_t mutex;
    pthread_mutex_t write;   /* a single writer on the socket */
    jas_work     task;
    int             task_valid;
    int             task_flag;
    int             index;
} jas_pool;

/*
 * TWO GROUPS OF TWO POOLS.
 *
 * The factory firmware keeps three slots with ONE active, and switching
 * (pool_watchdog_rotate_and_restart) restarts the whole miner. Here two
 * pools work AT THE SAME TIME (one chip can mine ETHW while two mine ETC),
 * so there is one GROUP per coin, each with a main pool and a backup.
 * JASRE_MAP and "asic_pool_group" assign a chip to a GROUP; the group
 * chooses which of its two pools serves.
 *
 * The backup stays in reserve, without a socket, until its group switches
 * to it. The switch costs a connection and a login, never a restart: the
 * DAG and the bank map stay in place.
 */
#define N_GROUPS    2
#define PER_GROUP   2
#define N_POOLS      (N_GROUPS * PER_GROUP)

static jas_pool pools[N_POOLS];

/* Which pool of the group is serving right now. */
static int rank_active[N_GROUPS];

/*
 * SWITCH TO THE BACKUP.
 *
 * Triggered by silence (SWITCH_SILENCE), not by the socket: a pool can
 * vanish without closing its connection. Never during the first
 * SWITCH_GRACE seconds (the chips do not mine yet), and only to a
 * configured backup. The main pool gets its chance back every
 * BACK_TO_MAIN seconds: it is the one the user chose.
 */
#define SWITCH_SILENCE 120   /* s with nothing received = silent pool */
#define SWITCH_GRACE   300
#define BACK_TO_MAIN 600    /* retry the main pool every 10 min */
/*
 * All arbitration timers use a monotonic clock: the machine boots with its
 * clock in 2021, then ntpdate jumps it years forward, which made every pool
 * look silent at once with time(NULL).
 */
static time_t jas_mono(void)
{
    struct timespec t;

    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0)
        return time(NULL);          /* for lack of anything better */
    return (time_t)t.tv_sec;
}

/*
 * Per group, not shared: otherwise a switch in one group resets the other
 * group's countdown back to its main pool.
 */
static time_t switch_done[N_GROUPS];

static time_t startup_pools[N_GROUPS];

static int pool_alive(const jas_pool *p, time_t now)
{
    return p->host[0] && p->sock >= 0 && p->last_receive &&
           now - p->last_receive < SWITCH_SILENCE;
}

/*
 * A backup is a candidate as soon as it is configured, without being
 * connected, as in the original firmware (a single socket, to the active
 * pool; the backup shows "Alive" in reserve).
 */
static int pool_candidate(const jas_pool *p)
{
    return p->host[0] != 0;
}

/*
 * How long has this pool been silent? Returns -1 if it has NEVER spoken:
 * last_receive is zero until then, and subtracting it from the clock
 * gave a bogus 56-year silence and endless switching.
 */
static long pool_silence(const jas_pool *p, time_t now)
{
    if (!p->last_receive)
        return -1;
    return (long)(now - p->last_receive);
}

static void arbitrate_group(int group)
{
    int base = group * PER_GROUP;
    int cur = rank_active[group];
    time_t now = jas_mono();
    int k;

    if (!startup_pools[group])
        startup_pools[group] = now;
    if (now - startup_pools[group] < SWITCH_GRACE)
        return;

    /*
     * Hand back to the main pool. It is not connected while the backup
     * serves, so it cannot "speak again": it is retried at regular
     * intervals, and if it still does not answer, the next round switches
     * back to the backup.
     */
    if (cur != 0 && now - switch_done[group] >= BACK_TO_MAIN) {
        rank_active[group] = 0;
        switch_done[group] = now;
        jas_hashrate_rt_reset();   /* active pool changed: new difficulty, clear the rt window */
        jas_log("pool: group %d, giving the primary another chance %s:%d",
                group, pools[base].host, pools[base].port);
        return;
    }
    if (cur != 0 && pool_alive(&pools[base], now)) {
        rank_active[group] = 0;
        jas_hashrate_rt_reset();   /* back to the main pool: new difficulty, clear the rt window */
        jas_log("pool: group %d, back to primary %s:%d",
                group, pools[base].host, pools[base].port);
        return;
    }
    if (pool_alive(&pools[base + cur], now))
        return;

    /*
     * Give the newly chosen pool its chance: it has received nothing yet, so
     * it would look silent and be switched away at once. It gets
     * SWITCH_SILENCE, counted from the switch.
     */
    if (switch_done[group] && now - switch_done[group] < SWITCH_SILENCE)
        return;

    for (k = 1; k < PER_GROUP; k++) {
        int r = (cur + k) % PER_GROUP;

        /* pool_candidate, not pool_alive: a backup in reserve is not
         * connected, so it cannot be judged by its reception. If it does not
         * answer, the next arbitration goes elsewhere. */
        if (r == cur || !pool_candidate(&pools[base + r]))
            continue;
        {
            long silence = pool_silence(&pools[base + cur], now);

            if (silence < 0)
                jas_log("pool: group %d never answered, switching %s:%d -> %s:%d",
                        group, pools[base + cur].host, pools[base + cur].port,
                        pools[base + r].host, pools[base + r].port);
            else
                jas_log("pool: group %d silent for %lds, switching %s:%d -> %s:%d",
                        group, silence,
                        pools[base + cur].host, pools[base + cur].port,
                        pools[base + r].host, pools[base + r].port);
        }
        rank_active[group] = r;
        switch_done[group] = now;
        jas_hashrate_rt_reset();   /* switch to the backup: new difficulty, clear the rt window */
        return;
    }
}

/*
 * A GROUP WITHOUT ANY CHIP needs no connection: its pools would never
 * serve anyone. They stay in permanent reserve: no socket, no jobs.
 * chip_groups[a] holds the group of chip a (0..2).
 */
static int group_no_chip(int group)
{
    unsigned a;

    for (a = 0; a < 3; a++)
        if (jas_pool_of_chip(a) == group)
            return 0;               /* at least one chip here */
    return 1;                       /* none */
}

static jas_pool *pool_active(int group)
{
    if (group < 0 || group >= N_GROUPS)
        group = 0;
    return &pools[group * PER_GROUP + rank_active[group]];
}


/*
 * Read this slot's pool in jasminer.json.
 *
 * A minimal extractor, not a JSON parser (the firmware uses cJSON,
 * load_json_config_cjson): it finds the fields of this slot's entry, see
 * below.
 */
static int read_config(jas_pool *p)
{
    FILE *f;
    char line[512], url[192] = "";
    char *c, *q;

    /*
     * Trial override: JASRE_POOL=host:port bypasses the file, so no
     * development address is ever written to a machine's flash.
     */
    c = getenv("JASRE_POOL");

    f = fopen(jas_config_file(), "r");

    if (!f) {
        jas_log("pool: /media/configs/jasminer.json unreadable");
        return -1;
    }
    /*
     * WHICH ARRAY, AND WHICH RANK IN IT.
     *
     * Group 0 reads "pools", group 1 reads "pools2": same shape, same order.
     * In each array, rank 0 is the main pool and rank 1 the backup. Empty
     * entries are skipped, not counted (a configuration may carry an empty
     * third slot).
     */
    {
        const char *table = (p->index / PER_GROUP) ? "\"pools2\"" : "\"pools\"";
        int rank_wanted = p->index % PER_GROUP;
        int in_table = 0, rank = -1, taken = 0;
        char u[192] = "", us[192] = "", pw[64] = "";

        while (fgets(line, sizeof(line), f)) {
            if (strstr(line, table)) { in_table = 1; continue; }
            if (!in_table)
                continue;
            /* Another array starts: ours is over. */
            if (strstr(line, "\"pools") && !strstr(line, table))
                break;

            if ((c = strstr(line, "\"url\""))) {
                u[0] = us[0] = pw[0] = 0;
                if ((c = strchr(c + 5, '"')) && (q = strchr(c + 1, '"')) && q > c + 1) {
                    size_t n = (size_t)(q - c - 1);
                    if (n < sizeof(u)) { memcpy(u, c + 1, n); u[n] = 0; }
                }
                if (u[0]) rank++;          /* one more usable entry */
                continue;
            }
            if (!u[0])
                continue;
            if ((c = strstr(line, "\"user\""))) {
                if ((c = strchr(c + 6, '"')) && (q = strchr(c + 1, '"'))) {
                    size_t n = (size_t)(q - c - 1);
                    if (n < sizeof(us)) { memcpy(us, c + 1, n); us[n] = 0; }
                }
                continue;
            }
            if ((c = strstr(line, "\"pass\""))) {
                if ((c = strchr(c + 6, '"')) && (q = strchr(c + 1, '"'))) {
                    size_t n = (size_t)(q - c - 1);
                    if (n < sizeof(pw)) { memcpy(pw, c + 1, n); pw[n] = 0; }
                }
                if (rank == rank_wanted) {
                    snprintf(url, sizeof(url), "%s", u);
                    snprintf(p->user, sizeof(p->user), "%s", us);
                    snprintf(p->pass, sizeof(p->pass), "%s", pw);
                    taken = 1;
                    break;
                }
                u[0] = 0;
            }
        }
        (void)taken;
    }
    fclose(f);

    /*
     * The override first, the check afterwards: a configuration without
     * "pools2" leaves the url empty, and the function must not give up
     * before looking at JASRE_POOL2.
     */
    /*
     * The trial override replaces ONLY the address: user and password stay
     * those of the file, so the trial exercises the real login.
     */
    {
        /* Environment overrides: no trial identity in /media/configs. */
        /*
         * One override per slot, to try a backup without touching the flash:
         *
         *     JASRE_POOL   / JASRE_POOL_S    group 0: main, backup
         *     JASRE_POOL2  / JASRE_POOL2_S   group 1: main, backup
         */
        static const char *keys[N_POOLS] = {
            "JASRE_POOL", "JASRE_POOL_S", "JASRE_POOL2", "JASRE_POOL2_S"
        };
        const char *override = getenv(keys[p->index & (N_POOLS - 1)]);

        if (override && *override)
            snprintf(url, sizeof(url), "%s", override);
    }

    /*
     * JASRE_USER: the trial identifier, without touching the flash. An ETC
     * pool such as k1pool expects an account name rather than a 0x address;
     * the format stays "<account>.<worker>", and open_session cuts at the
     * last dot.
     */
    {
        static const char *keys_u[N_POOLS] = {
            "JASRE_USER", "JASRE_USER_S", "JASRE_USER2", "JASRE_USER2_S"
        };
        const char *user_override = getenv(keys_u[p->index & (N_POOLS - 1)]);
        if (user_override && *user_override)
            snprintf(p->user, sizeof(p->user), "%s", user_override);
    }

    if (!url[0]) {
        jas_log("pool: no url in configuration");
        return -1;
    }
    /*
     * Strip a "stratum+tcp://" scheme: pools often write their address that
     * way, and getaddrinfo cannot resolve it.
     */
    char *start;

    {
        char *schema = strstr(url, "://");

        start = schema ? schema + 3 : url;
    }
    c = strrchr(start, ':');
    if (!c) {
        jas_log("pool: url \"%s\" without port", start);
        return -1;
    }
    *c = 0;
    p->port = atoi(c + 1);
    snprintf(p->host, sizeof(p->host), "%s", start);
    jas_log("pool[%d]: %s:%d, worker %s", p->index, p->host, p->port, p->user);
    return 0;
}

static int pool_connect(jas_pool *p)
{
    struct addrinfo index, *res = NULL, *a;
    char service[16];
    int one = 1;

    memset(&index, 0, sizeof(index));
    index.ai_family = AF_INET;
    index.ai_socktype = SOCK_STREAM;
    snprintf(service, sizeof(service), "%d", p->port);

    if (getaddrinfo(p->host, service, &index, &res) != 0 || !res) {
        /*
         * An unreachable pool is retried every 5 s: say it the first time,
         * then once a minute, so the log is not flooded.
         */
        time_t now = time(NULL);

        if (!p->last_dns_failed || now - p->last_dns_failed >= 60) {
            p->last_dns_failed = now;
            jas_log("pool: %s not found (DNS resolution)", p->host);
        }
        return -1;
    }
    p->last_dns_failed = 0;
    for (a = res; a; a = a->ai_next) {
        p->sock = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (p->sock < 0)
            continue;
        if (connect(p->sock, a->ai_addr, a->ai_addrlen) == 0)
            break;
        close(p->sock);
        p->sock = -1;
    }
    freeaddrinfo(res);

    if (p->sock < 0) {
        jas_log("pool: connect to %s:%d refused (%s)",
                p->host, p->port, strerror(errno));
        return -1;
    }
    /* Without TCP_NODELAY, Nagle delays our submissions by tens of
     * milliseconds; on a share, that is sometimes the difference. */
    setsockopt(p->sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    /*
     * SO_KEEPALIVE and its timers, as in socket_init (0x387b9):
     *
     *   setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE,  1)
     *   setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  ...)
     *   setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, ...)
     *   setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   1)
     *
     * Without them, a pool that vanishes without closing leaves the socket
     * open and silent until the 120 s net fires.
     */
    setsockopt(p->sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    {
        int idle = 30, intvl = 10, cnt = 3;

        setsockopt(p->sock, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,  sizeof(idle));
        setsockopt(p->sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(p->sock, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof(cnt));
    }
    /*
     * Read timeout: a pool that goes quiet must not block the thread
     * forever. Ten seconds are ample for a getwork answer.
     */
    {
        struct timeval tv = { 10, 0 };
        setsockopt(p->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }
    p->rest_n = 0;
    jas_log("pool[%d]: connected to %s:%d", p->index, p->host, p->port);
    return 0;
}

static void pool_close(jas_pool *p)
{
    if (p->sock >= 0)
        close(p->sock);
    p->sock = -1;
    p->rest_n = 0;
}

/*
 * SEND WITHOUT EVER FREEZING THE MINING THREAD.
 *
 * jas_submit runs in the mining loop: a blocking write on a full send
 * buffer would stop harvesting and work changes. The original sets
 * O_NONBLOCK in socket_init. Here: a non-blocking send, a brief retry,
 * then give up SAYING SO: a lost submission beats a frozen miner, but it
 * must not be lost silently.
 */
static int pool_send(jas_pool *p, const char *line)
{
    size_t n = strlen(line), written = 0;
    unsigned attempts = 0;
    int r = 0;

    if (p->sock < 0)
        return -1;
    /*
     * A SINGLE WRITER AT A TIME ON THE SOCKET.
     *
     * Two threads write to it: mining (jas_submit) and the network thread
     * (open_session, request_work). With non-blocking writes resumed
     * after a partial write, two messages could interleave in the middle of a
     * line; the pool silently ignores the invalid JSON and every submission
     * goes unanswered.
     */
    pthread_mutex_lock(&p->write);
    while (written < n) {
        /*
         * MSG_NOSIGNAL IS VITAL. Without it, writing to a socket the pool has
         * just closed raises SIGPIPE, which KILLS the process without a trace
         * in the log. A pool that accepts the connection and then hangs up
         * would kill the miner at each restart, before the backup switch
         * (SWITCH_GRACE) ever gets a chance. main.c also ignores SIGPIPE.
         */
        ssize_t k = send(p->sock, line + written, n - written,
                         MSG_DONTWAIT | MSG_NOSIGNAL);

        if (k > 0) { written += (size_t)k; attempts = 0; continue; }
        if (k < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (++attempts > 20) {          /* 20 x 10 ms = 200 ms at most */
                jas_log("pool[%d]: send buffer full, %zu/%zu bytes sent,"
                        " message abandoned", p->index, written, n);
                /* A cut line would leave the pool on truncated JSON: better to
                 * close and start again clean. */
                pool_close(p);
                r = -1;
                break;
            }
            usleep(10000);
            continue;
        }
        jas_log("pool: send interrupted (%s)", strerror(errno));
        pool_close(p);
        r = -1;
        break;
    }
    pthread_mutex_unlock(&p->write);
    return r;
}

#define REPLIES_MAX  8      /* unused */


/*
 * Extract the k-th hexadecimal element of the "result" array.
 *
 * Not a JSON parser: it looks for the "result" key, then the quoted
 * strings that follow. Enough for eth_getWork's flat array of strings;
 * any richer answer would defeat it.
 */
static int field_hex(const char *line, int k, uint8_t *dst, size_t n)
{
    const char *p = strstr(line, "\"result\"");
    int found = 0;
    size_t i;

    if (!p)
        return -1;
    p += 8;

    /* Advance to the k-th quoted string after "result". */
    while (*p) {
        const char *start, *end;

        if (*p != '"') { p++; continue; }
        start = ++p;
        while (*p && *p != '"') p++;
        if (!*p) return -1;
        end = p++;

        if (found++ != k)
            continue;

        if (end - start >= 2 && start[0] == '0' && (start[1] == 'x' || start[1] == 'X'))
            start += 2;
        if ((size_t)(end - start) < n * 2)
            return -1;
        for (i = 0; i < n; i++) {
            unsigned v;
            if (sscanf(start + i * 2, "%2x", &v) != 1)
                return -1;
            dst[i] = (uint8_t)v;
        }
        return 0;
    }
    return -1;
}

/* ---------------------------------------------------------------------- */

#define REREQUEST_SEC   60      /* unused: the safety net is the 120 s check */

/* Read a line if it has already arrived, never waiting; leftover: p->rest. */
static int receive_nonblocking(jas_pool *p, char *line, size_t size)
{
    for (;;) {
        char *end = memchr(p->rest, '\n', p->rest_n);
        ssize_t n;

        if (end) {
            size_t len = (size_t)(end - p->rest);

            if (len >= size)
                len = size - 1;
            memcpy(line, p->rest, len);
            line[len] = 0;
            p->rest_n -= (unsigned)(end - p->rest + 1);
            memmove(p->rest, end + 1, p->rest_n);
            return 1;
        }
        if (p->sock < 0)
            return 0;
        /*
         * FULL BUFFER WITHOUT A NEWLINE: a mandatory way out. Returning 0
         * here would hold at every following call: the socket would never be
         * read again and the pool would look silent while it answers. A line
         * longer than the buffer is dropped, with a trace.
         */
        if (p->rest_n >= sizeof(p->rest)) {
            char preview[121];

            memcpy(preview, p->rest, sizeof(preview) - 1);
            preview[sizeof(preview) - 1] = 0;
            jas_log("pool[%d]: line longer than %u bytes, dropped. Starts: %s",
                    p->index, (unsigned)sizeof(p->rest), preview);
            p->rest_n = 0;
            continue;
        }
        n = recv(p->sock, p->rest + p->rest_n, sizeof(p->rest) - p->rest_n, MSG_DONTWAIT);
        if (n > 0) {
            p->rest_n += (unsigned)n;
            continue;
        }
        if (n == 0) {
            jas_log("pool: connection closed by the pool");
            pool_close(p);
            return 0;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return 0;                       /* nothing for now */
        jas_log("pool: read interrupted (%s)", strerror(errno));
        pool_close(p);
        return 0;
    }
}

/*
 * The announced HEIGHT, fourth element of the array; 0 if absent. Only
 * used to flag an inconsistent coin detection (see name_says_etchash).
 */
static uint64_t field_height(const char *line)
{
    const char *p = strstr(line, "\"result\"");
    int quotes = 0;

    if (!p)
        return 0;
    p += 8;
    /* Skip the three strings (header, seed, target), i.e. six quotes. */
    while (*p && quotes < 6) {
        if (*p == '"')
            quotes++;
        p++;
    }
    while (*p == ' ' || *p == ',')
        p++;
    /*
     * The height can be a bare number OR a quoted string (k1pool sends
     * "0x017f3273"): both are accepted.
     */
    if (*p == '"')
        p++;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X'))
        return strtoull(p + 2, NULL, 16);
    if (*p >= '0' && *p <= '9')
        return strtoull(p, NULL, 10);
    return 0;
}

/*
 * ETHASH OR ETCHASH? THE POOL'S NAME SETTLES IT.
 *
 * ECIP-1099 doubled ETC's epoch length (30,000 -> 60,000 blocks, from
 * block 11,700,000) but the seeds kept the old pace: the seed of etchash
 * epoch N is keccak^(2N)(0). The seed thus gives 2N while the DAG must be
 * that of epoch N.
 *
 * The height cannot decide it: during the first half of an etchash epoch,
 * height/30000 and (height/60000)*2 give the same index, and a DAG twice
 * too large gets built (100 % rejects, "Low difficulty share").
 *
 * Rule: "etc" anywhere in the pool's host -> etchash (epoch = index/2);
 * otherwise ethash (epoch = index). The height only flags a badly named
 * pool.
 */
static int name_says_etchash(const char *host)
{
    int i;

    if (!host)
        return 0;
    for (i = 0; host[i] && host[i + 1] && host[i + 2]; i++)
        if ((host[i]     == 'e' || host[i]     == 'E') &&
            (host[i + 1] == 't' || host[i + 1] == 'T') &&
            (host[i + 2] == 'c' || host[i + 2] == 'C'))
            return 1;
    return 0;
}

static uint64_t effective_epoch(uint64_t seed_index, uint64_t height,
                                 int is_etchash)
{
    static int announce;                 /* bit 1 = ethash, bit 2 = etchash */
    uint64_t epoch = is_etchash ? seed_index / 2ull : seed_index;
    int bit = is_etchash ? 2 : 1;

    if (!(announce & bit)) {
        int coherent = (height == 0) || (is_etchash
            ? (height / 60000ull) * 2ull == seed_index
            : height / 30000ull == seed_index);

        announce |= bit;
        if (coherent)
            jas_log("pool: %s (pool name) — seed index %llu, epoch %llu",
                    is_etchash ? "etchash" : "ethash",
                    (unsigned long long)seed_index,
                    (unsigned long long)epoch);
        else
            jas_log("pool: %s (pool name) but height %llu inconsistent with"
                    " seed %llu — following the name (epoch %llu)."
                    " Misnamed pool?",
                    is_etchash ? "etchash" : "ethash",
                    (unsigned long long)height,
                    (unsigned long long)seed_index,
                    (unsigned long long)epoch);
    }
    return epoch;
}

/* Does a line carry usable work? */
static int read_work(const char *line, jas_work *out, const jas_pool *p)
{
    ethash_h256_t header, seed, target;

    if (field_hex(line, 0, header.b, 32) < 0 ||
        field_hex(line, 1, seed.b, 32) < 0 ||
        field_hex(line, 2, target.b, 32) < 0)
        return 0;

    out->header = header;
    out->seed   = seed;
    out->target = target;
    {
        uint64_t index = ethash_get_blocknum(&seed) / 30000ull;
        uint64_t height = field_height(line);
        int is_etchash = p && name_says_etchash(p->host);

        out->block_number = effective_epoch(index, height, is_etchash)
                           * 30000ull;
    }
    snprintf(out->work_id, sizeof(out->work_id), "%02x%02x%02x%02x",
             header.b[0], header.b[1], header.b[2], header.b[3]);
    return 1;
}

/*
 * LOG IN BEFORE ASKING.
 *
 * The firmware's protocol 0 is called "getwork" but it is ethproxy:
 * pool_login first sends
 *
 *     {"id":N,"worker":"<name>","method":"eth_submitLogin","params":["<user>"]}
 *
 * and this session opening is what puts the pool into notification mode.
 * Without it, the pool answers each eth_getWork but never pushes
 * anything. The user field of jasminer.json is "<wallet>.<name>"; the two
 * halves are used separately.
 */
static void request_work(jas_pool *p);

static void open_session(jas_pool *p)
{
    char outgoing[384], wallet[192], name[64];
    char *point;

    snprintf(wallet, sizeof(wallet), "%s", p->user);
    point = strrchr(wallet, '.');
    if (point) {
        *point = 0;
        snprintf(name, sizeof(name), "%s", point + 1);
    } else {
        snprintf(name, sizeof(name), "jasre");
    }

    snprintf(p->name, sizeof(p->name), "%s", name);
    snprintf(outgoing, sizeof(outgoing),
             "{\"id\": %u,\"worker\": \"%s\",\"method\": \"eth_submitLogin\","
             "\"params\": [\"%s\"]}\n",
             p->next_id++, name, wallet);
    if (pool_send(p, outgoing) == 0)
        jas_log("pool: session opened, wallet %s, worker %s", wallet, name);

}

static void request_work(jas_pool *p)
{
    char outgoing[128];

    snprintf(outgoing, sizeof(outgoing),
             "{\"id\": %u,\"method\": \"eth_getWork\",\"params\": []}\n",
             p->next_id++);
    if (pool_send(p, outgoing) == 0)
        p->last_request = jas_mono();
}

/* ====================================================================== *
 * THE NETWORK THREAD: the firmware's architecture.
 *
 * The original strictly separates the network from mining:
 * pthread_recv_ether (@0x3cb9c) owns the socket, and sl_wrapper_work never
 * touches it. The handover goes through a global structure and a flag
 * (struct pool_task_t, fields from the DWARF: task_type, work_id,
 * headhash, seedhash, target, task_flag, extranonce, zil_dag_flg,
 * sl_task_mutex).
 *
 * It is a FLAG, not a queue: the network thread OVERWRITES the previous
 * work, since a stale work has no value. And the mining loop's rate no
 * longer affects the freshness of the work: when the loop read the socket
 * itself, it missed most pushed jobs and a third of the shares came back
 * stale.
 *
 * Departure: the original also has a send thread (pthread_send_ether).
 * We send from the mining loop (a write does not block), but the ANSWER
 * is read by the network thread, sole reader of the socket: hence the
 * small table of pending submissions below.
 * ====================================================================== */

#define PENDING_MAX 32

/* The work state lives in jas_pool: one per connection. */

static struct {
    unsigned id;
    char     nonce[24];
    unsigned asic;          /* to attribute the verdict without guessing */
    int      index;        /* which pool */
    int      busy;
    struct timeval sent_at; /* for the round trip, like JsonRpcIdStu */
} pending[PENDING_MAX];
static pthread_mutex_t pending_mutex = PTHREAD_MUTEX_INITIALIZER;

static void pending_set(unsigned id, const char *nonce,
                          unsigned asic, int index)
{
    int k;

    pthread_mutex_lock(&pending_mutex);
    for (k = 0; k < PENDING_MAX; k++)
        if (!pending[k].busy) {
            pending[k].id = id;
            snprintf(pending[k].nonce, sizeof(pending[k].nonce), "%s", nonce);
            pending[k].asic = asic;
            pending[k].index = index;
            pending[k].busy = 1;
            gettimeofday(&pending[k].sent_at, NULL);
            break;
        }
    pthread_mutex_unlock(&pending_mutex);
    /*
     * And say so when the table overflows: an untracked submission still
     * goes out, but its verdict is never logged. The original logs the same
     * situation ("jsonrpc_id_item is full, clear jsonrpc_id_item").
     */
    if (k == PENDING_MAX) {
        /*
         * OVERFLOW: CLEAR THE TABLE, as the original does (insert_jsonrpc_id,
         * 0x39c98, calls init_jsonrpc_id()). Dropping the NEW entry instead
         * would leak a slot per unanswered submission until verdict tracking
         * stops for good.
         */
        pthread_mutex_lock(&pending_mutex);
        for (k = 0; k < PENDING_MAX; k++)
            pending[k].busy = 0;
        pending[0].id = id;
        snprintf(pending[0].nonce, sizeof(pending[0].nonce), "%s", nonce);
        pending[0].asic = asic;
        pending[0].index = index;
        pending[0].busy = 1;
        gettimeofday(&pending[0].sent_at, NULL);
        pthread_mutex_unlock(&pending_mutex);
        jas_log("pool[%d]: pending table full, cleared", index);
    }
}

/* Does an answer carry the id of a pending submission? */
static int pending_verdict(const char *line, char *nonce, size_t size,
                           unsigned *asic, int *index, long *round_trip_us)
{
    char no_space[BUFFER];
    const char *src;
    char *dst;
    int k, found = 0;
    unsigned received_id = 0;
    int received_id_valid = 0;

    for (src = line, dst = no_space;
         *src && dst < no_space + sizeof(no_space) - 1; src++)
        if (*src != ' ')
            *dst++ = *src;
    *dst = 0;

    /*
     * THE ID IS COMPARED AS AN INTEGER, not as a substring, as the original
     * does (check_jsonrpc_id, 0x39dbc). A strstr on `"id":3` also matches
     * `"id":30`, `"id":31`... and sends the verdict to the wrong nonce.
     */
    {
        const char *q = strstr(no_space, "\"id\":");

        if (q) {
            char *end = NULL;
            unsigned long v = strtoul(q + 5, &end, 10);

            if (end && end != q + 5) { received_id = (unsigned)v; received_id_valid = 1; }
        }
    }

    pthread_mutex_lock(&pending_mutex);
    for (k = 0; k < PENDING_MAX; k++) {
        char pattern[32];

        if (!pending[k].busy)
            continue;
        (void)pattern;
        if (received_id_valid && pending[k].id == received_id) {
            snprintf(nonce, size, "%s", pending[k].nonce);
            if (asic) *asic = pending[k].asic;
            if (index) *index = pending[k].index;
            if (round_trip_us) {
                struct timeval t_now;

                gettimeofday(&t_now, NULL);
                *round_trip_us =
                    (t_now.tv_sec  - pending[k].sent_at.tv_sec)  * 1000000L +
                    (t_now.tv_usec - pending[k].sent_at.tv_usec);
            }
            pending[k].busy = 0;
            found = 1;
            break;
        }
    }
    pthread_mutex_unlock(&pending_mutex);
    return found;
}

/*
 * The thread. It owns the socket: nobody else reads from it.
 */
void *jas_pool_thread(void *arg)
{
    jas_pool *p = (jas_pool *)arg;
    char received[BUFFER];

    /*
     * The index comes from the place in the array (not a two-slot test), so
     * each of the four threads reads its own configuration.
     */
    p->index = (int)(p - pools);
    if (p->sock == 0) p->sock = -1;      /* calloc leaves 0 */
    pthread_mutex_init(&p->mutex, NULL);
    pthread_mutex_init(&p->write, NULL);
    p->next_id = 1 + (unsigned)p->index * 100000;   /* distinct ids */
    char name[16];

    snprintf(name, sizeof(name), "pool%d", p->index);
    jas_name_thread(name);
    jas_log("pool[%d]: network thread started", p->index);

    for (;;) {
        time_t now;

        /*
         * IN RESERVE: NO SOCKET. As in the original firmware, only the active
         * pool of each group is connected; the others keep their configuration
         * in memory and talk to nobody.
         */
        if (!p->host[0])
            read_config(p);
        if (p->host[0] &&
            (group_no_chip(p->index / PER_GROUP) ||
             (p->index % PER_GROUP) != rank_active[p->index / PER_GROUP])) {
            if (p->sock >= 0) {
                jas_log("pool[%d]: moved to standby, connection closed", p->index);
                pool_close(p);
            }
            sleep(1);
            continue;
        }

        if (p->sock < 0) {
            /*
             * ARBITRATE EVEN WHEN WE CANNOT CONNECT: these "continue" must not
             * skip the arbitration below, or a group whose active pool refuses
             * connections would never try its main pool again.
             */
            arbitrate_group(p->index / PER_GROUP);
            if (!p->host[0] && read_config(p) < 0) {
                sleep(5);
                continue;
            }
            if (pool_connect(p) < 0) {
                sleep(5);
                continue;
            }
            p->last_receive = 0;
            /*
             * INTRODUCE OURSELVES, THEN ONE SINGLE REQUEST, as pool_login does
             * (0x3acd1): login, read the answer, then one eth_getWork
             * (0x3b212). The original never polls the pool afterwards: it lives
             * on pushes.
             */
            open_session(p);
            request_work(p);
        }

        /* Everything that arrived since the last pass. */
        p->lines_pass = 0;
        gettimeofday(&p->pass_t0, NULL);
        while (receive_nonblocking(p, received, sizeof(received))) {
            jas_work t;
            char nonce[24];

            /*
             * Measured from inside this loop (the thread may never leave it):
             * how many lines per second it absorbs, against what the pool
             * pushes. A permanent lag would make our work stale.
             */
            if (jas_trace_active() && ++p->lines_pass % 25u == 0) {
                struct timeval m;
                long ms;

                gettimeofday(&m, NULL);
                ms = (m.tv_sec - p->pass_t0.tv_sec) * 1000L +
                     (m.tv_usec - p->pass_t0.tv_usec) / 1000L;
                jas_log("pool[%d]: %u lines without yielding, %ld ms"
                        " (%.1f ms/line)", p->index, p->lines_pass, ms,
                        p->lines_pass ? (double)ms / p->lines_pass : 0.0);
            }

            /*
             * Trace only what is not a job push: the answers carrying
             * "result" without an array, i.e. the submission verdicts.
             */
            if (!strstr(received, "\"result\":[") && !strstr(received, "\"result\": [")) {
                if (jas_trace_active())
                    jas_log("pool[%d] REPLY: %.180s", p->index, received);
            }

            if (read_work(received, &t, p)) {
                pthread_mutex_lock(&p->mutex);
                if (!p->task_valid || memcmp(t.header.b, p->task.header.b, 32) != 0) {
                    p->works++;
                    if (jas_trace_active())
                        jas_log("pool[%d]: new work %s, block %llu", p->index,
                            t.work_id, (unsigned long long)t.block_number);
                    p->task_flag = 1;
                }
                p->task = t;
                p->task_valid = 1;
                pthread_mutex_unlock(&p->mutex);
                p->last_receive = jas_mono();
                continue;
            }
            /* Otherwise: the answer to one of our submissions? */
            {
                unsigned a = 9; int ip = p->index;
                long rtt = -1;

                if (pending_verdict(received, nonce, sizeof(nonce), &a, &ip, &rtt)) {
                    /*
                     * Chip and pool on the verdict line itself: deducing them
                     * from the previous line is wrong as soon as two nonces
                     * follow each other.
                     */
                    /*
                     * N_POOLS, NOT 2: a two-pool bound sent the verdicts of
                     * pools 2 and 3 to pools[0]. Any constant 2 in this file
                     * is to be reread as a possibly forgotten N_POOLS.
                     */
                    jas_pool *pv = &pools[(ip >= 0 && ip < N_POOLS) ? ip : 0];

                    if (strstr(received, "true")) {
                        pv->accepts++;
                        /*
                         * ONE ACCEPTED SHARE IN FIFTY IS LOGGED, NOT ALL.
                         *
                         * messagebak.sh (a factory script) archives then
                         * DELETES run.log beyond 100 KB, and share lines would
                         * fill it within hours, taking the startup lines with
                         * them. Like the original, log EVENTS: the first share
                         * of each session (it proves the pool answers), then
                         * one in fifty. The "(%u accepted)" counter stays
                         * exact; REJECTS are ALL logged. JASRE_TRACE gives
                         * the full detail.
                         */
                        if (jas_trace_active() || pv->accepts == 1
                            || (pv->accepts % 50) == 0)
                            jas_log("pool[%d] asic %u: share %s -> ACCEPTED (%u"
                                    " accepted, %ld us)",
                                    ip, a, nonce, pv->accepts, rtt);
                    } else {
                        pv->rejects++;
                        jas_log("pool[%d] asic %u: share %s -> REJECTED (%u"
                                " rejected, %ld us), %.160s",
                                ip, a, nonce, pv->rejects, rtt, received);
                    }
                    continue;
                }
            }

            /*
             * EVERYTHING ELSE IS LOGGED: a line we cannot read may be the
             * pool imposing a difficulty, reporting an error or asking for a
             * reconnection, and dropping it silently hides the cause.
             */
            jas_log("pool[%d]: unhandled line: %.200s", p->index, received);
        }

        /*
         * Safety net: ask again after 120 s of silence (k1pool can leave 66 s
         * between two jobs, so 60 s would fire for nothing).
         */
        now = jas_mono();
        if (p->sock >= 0 && p->last_receive &&
            now - p->last_receive > 120 &&
            now - p->last_request >= 120) {
            jas_log("pool: nothing received for %ld s, asking again",
                    (long)(now - p->last_receive));
            request_work(p);
        }

        /*
         * Network thread heartbeat: proves this thread is up to date before
         * any delay it measures (verdict latency, lag) is trusted.
         */
        /* Each thread arbitrates its own group: two threads per group go
         * through it, the operation is idempotent. */
        arbitrate_group(p->index / PER_GROUP);

        {
            /*
             * One counter per pool, not a shared static: both threads run
             * this function.
             */
            time_t t_now = jas_mono();

            if (!p->beat_t0) p->beat_t0 = t_now;
            p->beat_rounds++;
            if (t_now - p->beat_t0 >= 10) {
                if (jas_trace_active())
                jas_log("pool[%d] thread: %u loops in %ld s (%.1f ms/loop)",
                        p->index, p->beat_rounds,
                        (long)(t_now - p->beat_t0),
                        1000.0 * (double)(t_now - p->beat_t0) /
                        (double)p->beat_rounds);
                p->beat_rounds = 0; p->beat_t0 = t_now;
            }
        }

        /*
         * WAIT ON THE SOCKET, NOT ON THE CLOCK: poll() returns as soon as a
         * byte arrives (a fixed usleep added up to 20 ms before a pushed job
         * was even read), and falls back on the same 20 ms when there is
         * nothing, so the periodic tasks of this loop (arbitration, restart,
         * keepalive) keep their rate. Without a socket (reserve, lost
         * connection), plain wait.
         */
        if (p->sock >= 0) {
            struct pollfd pfd;

            pfd.fd      = p->sock;
            pfd.events  = POLLIN;
            pfd.revents = 0;
            /* EINTR: a signal during the wait is not an error; the next turn
             * will redo the work anyway. */
            (void)poll(&pfd, 1, 20);
        } else {
            usleep(20000);
        }
    }
    return NULL;
}

/*
 * The consumer, called by the mining loop. It does not touch the network:
 * it takes the last work dropped off and lowers the flag.
 */
/*
 * WHICH GROUP FOR WHICH CHIP.
 *
 * By default everyone in group 0: the single-coin behavior.
 * JASRE_MAP="1,0,1" puts chip 0 in group 1, chip 1 in group 0, chip 2
 * in group 1. (The firmware's pool_task_t carries a zil_dag_flg field: it
 * had room for a second coin, but switches the whole machine.)
 */
static int chip_groups[3] = { 0, 0, 0 };

void jas_pool_assign(const char *groups_txt)
{
    unsigned a;

    if (!groups_txt || !*groups_txt)
        return;
    for (a = 0; a < 3 && *groups_txt; a++) {
        if (*groups_txt >= '0' && *groups_txt <= '1')
            chip_groups[a] = *groups_txt - '0';
        groups_txt++;
        if (*groups_txt == ',')
            groups_txt++;
    }
    jas_log("pool: asic assignment -> %d %d %d",
            chip_groups[0], chip_groups[1], chip_groups[2]);
}

int jas_pool_of_chip(unsigned asic)
{
    return (asic < 3) ? chip_groups[asic] : 0;
}

/*
 * The network thread wants one per socket: four in all, group by group,
 * in the order [g0 main, g0 backup, g1 main, g1 backup].
 */
void *jas_pool_n(int idx)
{
    return &pools[(idx < 0 || idx >= N_POOLS) ? 0 : idx];
}

int jas_pool_count(void) { return N_POOLS; }
/* The host of a pool slot, if known: used to ask OUR pool for the ethash
 * cache (cpu_cache.c). NULL otherwise. */
const char *jas_pool_host(int idx)
{
    if (idx < 0 || idx >= N_POOLS || !pools[idx].host[0])
        return NULL;
    return pools[idx].host;
}

/*
 * Does this slot have an address? Answers without opening a socket:
 * main.c uses it not to start a thread on a missing backup. The
 * environment override counts as much as the file.
 */
/*
 * THE LIVE ASSIGNMENT, the one really in use: it can come from the
 * environment or from an automatic switch, so the web page must show this
 * rather than the configuration file.
 */
int jas_chip_group(unsigned asic) { return jas_pool_of_chip(asic); }
int jas_group_active(int group)
{
    return (group >= 0 && group < N_GROUPS) ? rank_active[group] : 0;
}

int jas_pool_configure(int idx)
{
    static const char *keys[N_POOLS] = {
        "JASRE_POOL", "JASRE_POOL_S", "JASRE_POOL2", "JASRE_POOL2_S"
    };
    const char *e;
    jas_pool probe;

    if (idx < 0 || idx >= N_POOLS)
        return 0;
    e = getenv(keys[idx]);
    if (e && *e)
        return 1;
    memset(&probe, 0, sizeof(probe));
    probe.index = idx;
    probe.sock = -1;
    return read_config(&probe) == 0;
}

/*
 * THE CHIPS' ASSIGNMENT, from the configuration.
 *
 * "asic_pool_group": "1,0,1" puts chip 0 in group 1, chip 1 in group 0,
 * chip 2 in group 1. Same form as JASRE_MAP, which keeps priority for
 * trials. A chip depends on a COIN (a group), not on an address.
 */
void read_public_assignment_config(void)
{
    FILE *f = fopen(jas_config_file(), "r");
    char line[512];

    if (!f)
        return;
    while (fgets(line, sizeof(line), f)) {
        char *c = strstr(line, "\"asic_pool_group\"");

        if (!c) continue;
        if ((c = strchr(c, ':')) && (c = strchr(c, '"'))) {
            char *q = strchr(c + 1, '"');

            if (q && q > c + 1) {
                char v[32];
                size_t n = (size_t)(q - c - 1);

                if (n < sizeof(v)) {
                    memcpy(v, c + 1, n); v[n] = 0;
                    jas_pool_assign(v);
                }
            }
        }
        break;
    }
    fclose(f);
}

/* The work of the pool this chip depends on. */
int jas_work_of_chip(unsigned asic, jas_work *out)
{
    return jas_work_of_group(jas_pool_of_chip(asic), out);
}

/* The work of a group's active pool. */
int jas_work_of_group(int group, jas_work *out)
{
    jas_pool *p = pool_active(group);
    int valid;

    pthread_mutex_lock(&p->mutex);
    valid = p->task_valid;
    if (valid)
        *out = p->task;
    pthread_mutex_unlock(&p->mutex);
    return valid;
}

/*
 * THE ETC GROUP, if there is one: a chip whose expected ETHW reject
 * reaches the switch threshold moves to ETC (dag_calc.c, switch_to_etc;
 * "etc_switch_threshold", 70 % by default). The active pool's name carries
 * the coin, as for the epochs (name_says_etchash). -1 if no configured group
 * is ETC.
 */
int jas_group_etchash(void)
{
    int g;

    for (g = 0; g < N_GROUPS; g++) {
        const jas_pool *p = pool_active(g);

        if (p->host[0] && name_says_etchash(p->host))
            return g;
    }
    return -1;
}

/*
 * Change a chip's group while running. The file is not touched (the
 * settings page keeps showing the configured choice). A group that
 * receives its first chip leaves its reserve and connects (jas_pool_thread,
 * group_no_chip); the one that loses its last closes its socket.
 */
void jas_pool_switch_chip(unsigned asic, int group)
{
    if (asic >= 3 || group < 0 || group >= N_GROUPS)
        return;
    chip_groups[asic] = group;
    jas_log("pool: asic %u -> group %d (switch while running); assignment %d %d %d",
            asic, group + 1, chip_groups[0], chip_groups[1], chip_groups[2]);
}

int jas_work_current(jas_work *out)
{
    /*
     * NEVER HARD-CODE pools[0]: with every chip in group 1 (the page's
     * "Group 2", ETC) and none in group 0, pools[0] is silent and the
     * startup would wait for it in vain. Take the first pool REALLY
     * assigned to a chip that carries a valid work (pools[0] in the usual
     * ETHW or mixed setups).
     */
    unsigned asic;
    int found = 0;

    for (asic = 0; asic < 3 && !found; asic++) {
        jas_pool *p = pool_active(jas_pool_of_chip(asic));

        pthread_mutex_lock(&p->mutex);
        if (p->task_valid) {
            *out = p->task;
            jas_work_g = p->task;
            p->task_flag = 0;
            found = 1;
        }
        pthread_mutex_unlock(&p->mutex);
    }
    return found;
}

/*
 * Submit. We only WRITE: the answer is read by the network thread, sole
 * reader of the socket. The id goes into the pending table; the thread
 * will match the answer and log the verdict.
 */
/*
 * Describe a pool for the API: url, user, and whether it answers.
 *
 * "Alive" = an open socket AND something received less than 120 s ago:
 * an open socket alone proves nothing, a pool can vanish without closing.
 */
int jas_pool_describe(int idx, char *url, size_t nurl,
                     char *user, size_t nuser, int *live)
{
    const jas_pool *p;

    if (idx < 0 || idx >= N_POOLS)
        return 0;
    p = &pools[idx];
    if (!p->host[0])
        return 0;
    if (url)  snprintf(url, nurl, "%s:%d", p->host, p->port);
    if (user) snprintf(user, nuser, "%s", p->user);
    if (live) {
        /*
         * Three states, named as in the original:
         *   2 "In use"  the active pool of its group, the one receiving;
         *   1 "Alive"   ready to take over (a backup in reserve, see below);
         *   0 "Dead"    configured but silent.
         */
        int alive = pool_alive(p, jas_mono());
        int active = (idx % PER_GROUP) == rank_active[idx / PER_GROUP];
        int is_main = (idx % PER_GROUP) == 0;

        /*
         * A backup IN RESERVE has no socket: it is trusted on its
         * configuration alone (pool_candidate), as the original shows it
         * "Alive". The main pool we LEFT is judged by its real reachability
         * (pool_alive): it was left for being silent, so it shows "Dead"
         * until the failback brings it back.
         */
        /*
         * A GROUP WITHOUT A CHIP serves nothing: all its pools are "Dead",
         * main AND backup.
         */
        if (group_no_chip(idx / PER_GROUP))
            *live = 0;                            /* Dead for the whole group */
        else if (active)
            *live = alive ? 2 : 0;                  /* In use / Dead    */
        else if (is_main)
            *live = alive ? 1 : 0;                  /* main left: Dead if silent */
        else
            *live = pool_candidate(p) ? 1 : 0;    /* backup in reserve: Alive */
    }
    return 1;
}

int jas_pool_counters(int idx, unsigned *accepts, unsigned *rejects,
                       unsigned *works)
{
    const jas_pool *p;

    if (idx < 0 || idx >= N_POOLS)
        return 0;
    p = &pools[idx];
    if (!p->host[0])
        return 0;
    if (accepts) *accepts = p->accepts;
    if (rejects) *rejects = p->rejects;
    if (works)   *works   = p->works;
    return 1;
}

void jas_submit(const struct gooddata_detail *d)
{
    jas_pool *p = pool_active(jas_pool_of_chip(d->asic_id));
    char outgoing[512];
    char nonce[24], header[80], mix[80];
    unsigned k, id;

    if (p->sock < 0)
        return;

    snprintf(nonce, sizeof(nonce), "0x%016llx",
             (unsigned long long)d->good_data.nonce);
    /*
     * THE CHIP'S HEADER, not the machine's: in dual mining, an ETC share
     * submitted with the ETHW header would be refused.
     */
    {
        const jas_coin_ctx *cc = jas_coin_of((int)d->board_id, d->asic_id);
        const uint8_t *src = cc ? cc->header.b : (const uint8_t *)jas_fpga_header;

        strcpy(header, "0x");
        strcpy(mix, "0x");
        for (k = 0; k < 32; k++) {
            snprintf(header + 2 + k * 2, 3, "%02x", src[k]);
            snprintf(mix + 2 + k * 2, 3, "%02x", d->good_data.mixhash.b[k]);
        }
    }
    if (0) for (k = 0; k < 32; k++) {
        snprintf(header + 2 + k * 2, 3, "%02x", jas_fpga_header[k]);
        snprintf(mix + 2 + k * 2, 3, "%02x", d->good_data.mixhash.b[k]);
    }

    id = p->next_id++;
    pending_set(id, nonce, d->asic_id, p->index);
    /*
     * THE MOMENT OF SENDING, logged apart from the verdict: the verdict line
     * is written when the ANSWER arrives, so it mixes our delay with the
     * pool's.
     */
    {
        const jas_coin_ctx *cc = jas_coin_of((int)d->board_id, d->asic_id);

        if (jas_trace_active())
            jas_log("pool[%d] asic %u: sending %s work %s", p->index, d->asic_id,
                nonce, (cc && cc->work_id[0]) ? cc->work_id : "?");
    }
    /*
     * THE "worker" FIELD, as in the original ELF's strings:
     *
     *   {"id": %d, "worker": "%s", "method": "eth_submitWork","params": [...]}
     *
     * Without it, a pool cannot tell which rig a share comes from, and may
     * file it as "Stale share" (k1pool refused 88 % of our shares that way).
     */
    snprintf(outgoing, sizeof(outgoing),
             /*
              * ALIGNED ON THE ORIGINAL ELF, spaces included:
              *
              *   {"id": %d, "worker": "%s", "method": "eth_submitWork","params": ["0x%s", "0x%s", "0x%s"]}
              *
              * A compliant JSON parser sees no difference, but faced with
              * refusals, remove every gap with the reference first.
              */
             "{\"id\": %u, \"worker\": \"%s\", \"method\": \"eth_submitWork\","
             "\"params\": [\"%s\", \"%s\", \"%s\"]}\n",
             id, p->name, nonce, header, mix);
    /*
     * THE REAL BYTES of the first three submissions of each pool are logged:
     * when a pool answers nothing at all, the message itself is suspect, and
     * looking at what really goes out beats reasoning about the code.
     */
    {
        static unsigned trace[4];
        int g = p->index & 3;

        if (trace[g] < 3) {
            trace[g]++;
            jas_log("pool[%d] OUTGOING: %s", p->index, outgoing);
        }
    }
    pool_send(p, outgoing);
}

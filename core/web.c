/*
 * The miner's API: port 4028, JSON answers.
 *
 * Not a web server: the page is served by lighttpd from /www, and its CGI
 * scripts query this API (read in minerStatus.cgi):
 *
 *     browser -> lighttpd -> minerStatus.cgi -> jasminer_api -o groupctrl
 *                                            -> 127.0.0.1:4028 -> us
 *
 * THE PORT comes from `sprintf(port_s, "%d", 0xfbc)` in pthread_web: 4028,
 * cgminer's, from which this firmware descends (jasminer_api still carries
 * the symbols SEPARATOR, COMMA and ONLY of api.c).
 *
 * THE CONTRACT IS READ ON A MACHINE RUNNING THE STOCK FIRMWARE, not deduced
 * from the decompiled code:
 *
 *   - `summary` alone returns the BARE object; it is wrapped under the key
 *     "summary" only in `groupctrl`;
 *   - `boardstats`, `coin` and `allinfos` answer NOTHING: they are internal
 *     function names, not commands;
 *   - `uptime` is a float, and `fan_percentN` a STRING while `fanN` is a
 *     number;
 *   - the answer ends with a null byte followed by a newline.
 */

#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "jas_hal.h"
#include "jas_state.h"
#include "jas_coin.h"

#define PORT_API      4028
#define OUTPUT_BUFFER 65536
#define INPUT_BUFFER 512
#define N_BOARDS      3
#define N_CHIPS       3

extern uint32_t jas_hw_err[9];
extern uint32_t jas_accept_num[9];

/* ====================================================================== *
 * What the monitoring drops off here: the per-chip throughput and the rail
 * voltages it already reads at each round. Publishing them spares the web
 * thread from reading the silicon, which would interleave with the mining
 * transfers.
 * ====================================================================== */

static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
static double   hashrate_chip[N_BOARDS][N_CHIPS];
static unsigned alus_chip[N_BOARDS][N_CHIPS];
static unsigned rail_chip[N_BOARDS][N_CHIPS][3];
static double   hashrate_average;

/*
 * AGE IS MEASURED ON A CLOCK THAT DOES NOT JUMP: the machine boots with its
 * clock at the file system's build date (2021), then ntpdate throws it
 * years forward, so time(NULL) - start would return years. CLOCK_MONOTONIC
 * counts from boot and ignores time corrections: the only valid source for
 * a duration.
 */
static double instant_monotone(void)
{
    struct timespec t;

    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0)
        return 0.0;
    return (double)t.tv_sec + (double)t.tv_nsec / 1e9;
}

static double start;

void jas_web_hashrate_publish(int board, unsigned chip, double mh, unsigned alus)
{
    if (board < 0 || board >= N_BOARDS || chip >= N_CHIPS)
        return;
    pthread_mutex_lock(&state_mutex);
    hashrate_chip[board][chip] = mh;
    alus_chip[board][chip]  = alus;
    pthread_mutex_unlock(&state_mutex);
}

void jas_web_rails_publish(int board, unsigned chip, unsigned r0, unsigned r1, unsigned r2)
{
    if (board < 0 || board >= N_BOARDS || chip >= N_CHIPS)
        return;
    pthread_mutex_lock(&state_mutex);
    rail_chip[board][chip][0] = r0;
    rail_chip[board][chip][1] = r1;
    rail_chip[board][chip][2] = r2;
    pthread_mutex_unlock(&state_mutex);
}

/* ====================================================================== *
 * THE HISTORY, for the three charts of the home page, served by "all"
 * (minerStatus_all.cgi asks for it; it is groupctrl plus these series).
 *
 * Format, as the stock firmware returns it:
 *
 *     "hashrate_line": "1786350197,1600.79;1786350137,1600.79;…"
 *
 * Unix timestamp, comma, value, semicolon: from the MOST RECENT to the
 * oldest, one point every 60 s, over about eight hours.
 * ====================================================================== */

#define HIST_N 480          /* 480 x 60 s = 8 h, like the original */

static struct {
    time_t t;
    double rt, avg, refusal;
} hist[HIST_N];
static unsigned hist_n, hist_head;
static time_t   hist_last;

/*
 * Called by the monitoring; keeps only one point per minute.
 * The sampling must not depend on the page's requests — otherwise an open
 * browser would change the shape of the curves.
 */
void jas_web_sample(double rt, double avg, double refusal)
{
    time_t now = time(NULL);

    /*
     * A CLOCK JUMP INVALIDATES THE WHOLE HISTORY: the curves need real dates,
     * which the monotonic clock cannot give, and the clock jumps years forward
     * when ntpdate first answers. At one point per minute, two samples cannot
     * be more than an hour apart; beyond that the clock has jumped, and the
     * history starts again from zero (otherwise the graph spans years and the
     * recent points squash into one column).
     */
    if (hist_n && now - hist_last > 3600) {
        jas_log("history: clock jumped %ld s, %u samples dropped",
                (long)(now - hist_last), hist_n);
        pthread_mutex_lock(&state_mutex);
        hist_n = 0;
        hist_head = 0;
        pthread_mutex_unlock(&state_mutex);
        hist_last = 0;
    }

    if (hist_last && now - hist_last < 60)
        return;
    hist_last = now;
    pthread_mutex_lock(&state_mutex);
    hist[hist_head].t     = now;
    hist[hist_head].rt    = rt;
    hist[hist_head].avg   = avg;
    hist[hist_head].refusal = refusal;
    hist_head = (hist_head + 1) % HIST_N;
    if (hist_n < HIST_N)
        hist_n++;
    pthread_mutex_unlock(&state_mutex);
}

/*
 * THE "DAG loading progress" BAR of the home page: index.html reads
 * summary.dag_time as a PERCENTAGE (dag_sch.style.width = dag + "%") and
 * hides the bar once the miner runs. The original splits the progress into
 * steps (40 %, 90 %, 100 %; dag_progress_bar @ 0x472cc); we keep that shape
 * with our real stages: caches ready at 40, filling from 40 to 90, DAG
 * checked at 100.
 */
static unsigned dag_percent;

void jas_web_dag_publish(unsigned percent)
{
    if (percent > 100)
        percent = 100;
    pthread_mutex_lock(&state_mutex);
    /*
     * Ratchet, EXCEPT for an explicit reset: the bar never goes backwards
     * during a build, but seed_update_operation can REPLAY the startup, and
     * the pool watchdog uses the bar to know whether the machine is still
     * working (jasminer.sh, pool_watchdog_get_dag). No caller publishes 0 by
     * accident: the steps are 40, 40..90 and 100.
     */
    if (percent == 0 || percent > dag_percent)
        dag_percent = percent;
    pthread_mutex_unlock(&state_mutex);
}

double jas_hashrate_total_mh(void)
{
    double t = 0;
    int c, p;

    pthread_mutex_lock(&state_mutex);
    for (c = 0; c < N_BOARDS; c++)
        for (p = 0; p < N_CHIPS; p++)
            t += hashrate_chip[c][p];
    pthread_mutex_unlock(&state_mutex);
    return t;
}

static double hashrate_total(void)
{
    double t = 0;
    int c, p;

    pthread_mutex_lock(&state_mutex);
    for (c = 0; c < N_BOARDS; c++)
        for (p = 0; p < N_CHIPS; p++)
            t += hashrate_chip[c][p];
    /*
     * The average, smoothed: the original tells "rt" (instant) from "avg",
     * and they clearly differ; a moving average is enough (avg = rt would
     * show the same measurement twice under two names).
     */
    hashrate_average = hashrate_average > 0 ? hashrate_average * 0.9 + t * 0.1 : t;
    pthread_mutex_unlock(&state_mutex);
    return t;
}

/* ====================================================================== *
 * A tiny JSON emitter: cJSON would be disproportionate for a flat document.
 * What matters is escaping correctly: a pool URL contains "/" and sometimes
 * '"', and a malformed document makes the page show its fallback without
 * any message.
 * ====================================================================== */

typedef struct { char *b; size_t n, max; int truncated; } jbuf;

static void jput(jbuf *j, const char *fmt, ...)
{
    va_list ap;
    int k;

    if (j->truncated || j->n >= j->max)
        return;
    va_start(ap, fmt);
    k = vsnprintf(j->b + j->n, j->max - j->n, fmt, ap);
    va_end(ap);
    if (k < 0 || (size_t)k >= j->max - j->n) { j->truncated = 1; return; }
    j->n += (size_t)k;
}

static void jstring(jbuf *j, const char *key, const char *val)
{
    const unsigned char *s = (const unsigned char *)(val ? val : "");

    jput(j, "\"%s\": \"", key);
    for (; *s; s++) {
        if (*s == '"' || *s == '\\')  jput(j, "\\%c", *s);
        else if (*s == '\n')          jput(j, "\\n");
        else if (*s == '\r')          jput(j, "\\r");
        else if (*s == '\t')          jput(j, "\\t");
        else if (*s < 0x20)           jput(j, "\\u%04x", *s);
        else                          jput(j, "%c", *s);
    }
    jput(j, "\"");
}

static void jnumber(jbuf *j, const char *key, long v) { jput(j, "\"%s\": %ld", key, v); }

/* A series, from the most recent to the oldest. */
static void web_series(jbuf *j, const char *envelope, const char *key, int what)
{
    unsigned k;

    jput(j, "\"%s\": {\"%s\": \"", envelope, key);
    pthread_mutex_lock(&state_mutex);
    for (k = 0; k < hist_n; k++) {
        /* hist_head points to the NEXT free slot: walk back from it. */
        unsigned i = (hist_head + HIST_N - 1 - k) % HIST_N;
        double v = (what == 0) ? hist[i].rt
                 : (what == 1) ? hist[i].avg
                               : hist[i].refusal;

        /*
         * SEPARATOR BEFORE, never after: a trailing ";" gives the page an empty
         * item, which becomes a ghost point at epoch 0 (1970), stretching the
         * time axis and squashing every real measurement into one column.
         */
        jput(j, "%s%ld,%.2f", k ? ";" : "", (long)hist[i].t, v);
    }
    pthread_mutex_unlock(&state_mutex);
    jput(j, "\"}");
}

static void web_lines(jbuf *j)
{
    web_series(j, "hashline",     "hashrate_line",     0);  jput(j, ", ");
    web_series(j, "avg_hashline", "avg_hashrate_line", 1);  jput(j, ", ");
    web_series(j, "rejectline",   "reject_rate_line",  2);
}


/* ====================================================================== *
 * summary, key by key and type by type as the stock firmware returns it:
 *
 *   {"miner": "JASMINER X16-Q", "version": "...", "uptime": 167995.0,
 *    "machine_sn": "...", "ctrlboard_sn": "...", "pw_sn": "...",
 *    "work_mode": 2, "coin_type": 0, "hashboard_sn": {"sn_0": "...", "sn_1": ""},
 *    "rt": "1657.96 MH/s", "avg": "1969.70 MH/s", "network": "connected",
 *    "rejectRate": "0.00", "temp_max": 53, "temp_min": 51,
 *    "dag_time": 100, "eth0_ip": 1}
 *
 * The serial numbers come from core/serial.c (decrypted from
 * /media/configs); they are empty when the machine has none, which
 * minerStatus.cgi's fallback also accepts.
 * ====================================================================== */
/*
 * summary's "version" = the build date, line 1 of /etc/machine (e.g. "Mon
 * Oct 14 14:17:39 CST 2024"), which faire_firmware.sh writes and the page
 * shows.
 */
static const char *machine_version(void)
{
    static char v[64];
    static int loaded;
    FILE *f;

    if (!loaded) {
        loaded = 1;
        v[0] = 0;
        f = fopen("/etc/machine", "r");
        if (f) {
            if (fgets(v, sizeof(v), f)) {
                size_t n = strlen(v);
                while (n && (v[n - 1] == '\n' || v[n - 1] == '\r'))
                    v[--n] = 0;
            }
            fclose(f);
        }
        if (!v[0])
            snprintf(v, sizeof(v), "JASMINER X16-Q");
    }
    return v;
}


static void web_summary(jbuf *j)
{
    unsigned seen = 0, submitted = 0, rej = 0, per = 0;
    double rt = hashrate_total(), avg;
    double rate = 0;
    double high = start > 0.0 ? instant_monotone() - start : 0.0;
    int network = 0, k;

    jas_mining_counters(&seen, &submitted, &rej, &per);

    /*
     * THE REJECT RATE IS THE POOLS', as in the original, not that of our local
     * checks: those include the nonces nonce_check discards before sending,
     * which the pool never sees.
     */
    {
        unsigned ta = 0, tr = 0;

        for (k = 0; k < jas_pool_count(); k++) {
            unsigned a = 0, r = 0;

            if (jas_pool_counters(k, &a, &r, NULL)) { network = 1; ta += a; tr += r; }
        }
        if (ta + tr)
            rate = 100.0 * (double)tr / (double)(ta + tr);
    }

    pthread_mutex_lock(&state_mutex);
    avg = hashrate_average;
    pthread_mutex_unlock(&state_mutex);

    /*
     * REAL THROUGHPUT, measured on the stream of ACCEPTED nonces — replaces
     * the nominal one (get_eff, frozen). It is the judge of the true
     * efficiency on the pool. The nominal one is still computed above for its
     * side effects (hashrate_average, the per-chip throughput of the boards page).
     * See core/hashrate_rt.c.
     */
    jas_hashrate_rt_get(&rt, &avg);

    jput(j, "{");
    jstring(j, "miner", "JASMINER X16-Q");                              jput(j, ", ");
    jstring(j, "version", machine_version());                     jput(j, ", ");
    /* WHOLE seconds: the monotonic clock returns a float (tv_nsec), and
     * uptime_cont % 60 on a non-integer value prints "7m5.1999999s". */
    jput(j, "\"uptime\": %.0f, ", high);
    jstring(j, "machine_sn", jas_sn_machine());                         jput(j, ", ");
    jstring(j, "ctrlboard_sn", jas_sn_ctrlboard());                     jput(j, ", ");
    jstring(j, "pw_sn", jas_sn_pw());                                   jput(j, ", ");
    jnumber(j, "work_mode", 2);                                         jput(j, ", ");
    jnumber(j, "coin_type", jas_coin_type_current());                   jput(j, ", ");
    jput(j, "\"hashboard_sn\": {");
    jstring(j, "sn_0", jas_sn_hashboard(0));  jput(j, ", ");
    jstring(j, "sn_1", jas_sn_hashboard(1));
    jput(j, "}, ");
    jput(j, "\"rt\": \"%.2f MH/s\", ", rt);
    jput(j, "\"avg\": \"%.2f MH/s\", ", avg);
    jstring(j, "network", network ? "connected" : "disconnected");       jput(j, ", ");
    jput(j, "\"rejectRate\": \"%.2f\", ", rate);
    jnumber(j, "temp_max", jas_temp_max);                               jput(j, ", ");
    jnumber(j, "temp_min", jas_temp_min);                               jput(j, ", ");
    jnumber(j, "dag_time", (long)dag_percent);                         jput(j, ", ");
    jnumber(j, "eth0_ip", 1);
    jput(j, "}");
}

/* ====================================================================== *
 * pools: {"pool": [ {...}, ... ]}, one entry per slot (four here, three in
 * the stock firmware), an unconfigured slot as {}. States: "In use" (the
 * active pool of its group), "Alive" (ready to take over), "Dead"
 * (configured but silent); the binary also names "Switch", which we do not
 * produce.
 * ====================================================================== */

static void web_pools(jbuf *j)
{
    int k;

    jput(j, "{\"pool\": [");
    for (k = 0; k < jas_pool_count(); k++) {
        unsigned a = 0, r = 0, w = 0;
        char url[192] = "", user[192] = "";
        int live = 0;

        if (k) jput(j, ", ");
        if (!jas_pool_counters(k, &a, &r, &w)) { jput(j, "{}"); continue; }
        jas_pool_describe(k, url, sizeof(url), user, sizeof(user), &live);
        jput(j, "{");
        jnumber(j, "pool", k);                               jput(j, ", ");
        jstring(j, "url", url);                              jput(j, ", ");
        jstring(j, "user", user);                            jput(j, ", ");
        jstring(j, "status", live == 2 ? "In use" :
                             live == 1 ? "Alive" : "Dead");   jput(j, ", ");
        /* The works RECEIVED, not accept + reject (the original reports
         * them separately). */
        jnumber(j, "works", (long)w);                        jput(j, ", ");
        jnumber(j, "accept", (long)a);                       jput(j, ", ");
        jnumber(j, "reject", (long)r);
        jput(j, "}");
    }
    jput(j, "]}");
}

/* ====================================================================== *
 * boards: board_num, fan_num, fanN (number) and fan_percentN (STRING) for N
 * from 1 to 4, then a "board" array whose entries each carry plug, asics,
 * rate, hw, freq, board_temp_min/max, asicN_temp, asicN_rail0..2,
 * asicN_code, asic_sn{asic0..2} and asic_stat ("OOO" on a healthy machine:
 * one character per chip).
 *
 * Not filled yet, hence zero or empty: freq (the PLL is not read back),
 * asicN_code and the chips' serial numbers; reads to be written, not
 * values to invent.
 * ====================================================================== */

static void web_boards(jbuf *j)
{
    int speeds[4] = { 0, 0, 0, 0 };
    int pwm = jas_pwm_current();
    unsigned board, chip;
    int first = 1, k;

    fan_speed_read(speeds);

    jput(j, "{");
    {
        /*
         * THE NUMBER OF ACTIVE BOARDS, not the size of the array: BOARD_NUM is
         * 3 on the X16-Q (the most the firmware handles) while the machine
         * carries ONE board, and the original returns "board_num: 1".
         */
        unsigned c, actives = 0;

        for (c = 0; c < BOARD_NUM && c < N_BOARDS; c++)
            if (BOARD_EN >> c & 1)
                actives++;
        jnumber(j, "board_num", (long)actives);  jput(j, ", ");
    }
    jnumber(j, "fan_num", 4);                  jput(j, ", ");
    for (k = 0; k < 4; k++) {
        char key[16];

        jput(j, "\"fan%d\": %d, ", k + 1, speeds[k]);
        snprintf(key, sizeof(key), "fan_percent%d", k + 1);
        {
            char v[8];

            snprintf(v, sizeof(v), "%d", speeds[k] ? pwm : 0);
            jstring(j, key, v);
        }
        jput(j, ", ");
    }
    jput(j, "\"board\": [");
    for (board = 0; board < BOARD_NUM && board < N_BOARDS; board++) {
        double rate = 0;
        unsigned tmin = 0xffff, tmax = 0;
        unsigned long hw = 0;

        if (!(BOARD_EN >> board & 1))
            continue;
        if (!first) jput(j, ", ");
        first = 0;

        pthread_mutex_lock(&state_mutex);
        for (chip = 0; chip < ASIC_NUM && chip < N_CHIPS; chip++)
            rate += hashrate_chip[board][chip];
        pthread_mutex_unlock(&state_mutex);

        for (chip = 0; chip < ASIC_NUM && chip < N_CHIPS; chip++) {
            unsigned t = jas_asic_temp[board][chip];

            if (t && t < tmin) tmin = t;
            if (t > tmax)      tmax = t;
            hw += jas_hw_err[board * 3 + chip];
        }
        if (tmin == 0xffff) tmin = 0;

        jput(j, "{");
        jnumber(j, "plug", 0);                       jput(j, ", ");
        jnumber(j, "asics", (long)ASIC_NUM);         jput(j, ", ");
        jput(j, "\"rate\": \"%.2f MH/s\", ", rate);
        jnumber(j, "hw", (long)hw);                  jput(j, ", ");
        jnumber(j, "freq", 0);                       jput(j, ", ");
        jnumber(j, "board_temp_min", (long)tmin);    jput(j, ", ");
        jnumber(j, "board_temp_max", (long)tmax);    jput(j, ", ");

        for (chip = 0; chip < ASIC_NUM && chip < N_CHIPS; chip++)
            jput(j, "\"asic%u_temp\": %u, ", chip, jas_asic_temp[board][chip]);
        for (chip = 0; chip < ASIC_NUM && chip < N_CHIPS; chip++) {
            unsigned r[3];

            pthread_mutex_lock(&state_mutex);
            r[0] = rail_chip[board][chip][0];
            r[1] = rail_chip[board][chip][1];
            r[2] = rail_chip[board][chip][2];
            pthread_mutex_unlock(&state_mutex);
            jput(j, "\"asic%u_rail0\": %u, \"asic%u_rail1\": %u,"
                    " \"asic%u_rail2\": %u, ", chip, r[0], chip, r[1], chip, r[2]);
        }
        for (chip = 0; chip < ASIC_NUM && chip < N_CHIPS; chip++)
            jput(j, "\"asic%u_code\": 0, ", chip);

        jput(j, "\"asic_sn\": {");
        for (chip = 0; chip < ASIC_NUM && chip < N_CHIPS; chip++) {
            char key[12];

            snprintf(key, sizeof(key), "asic%u", chip);
            if (chip) jput(j, ", ");
            jstring(j, key, "");
        }
        jput(j, "}, ");

        {
            char stat[N_CHIPS + 1];

            for (chip = 0; chip < ASIC_NUM && chip < N_CHIPS; chip++)
                stat[chip] = (ASIC_EN((int)board) >> chip & 1) ? 'O' : 'X';
            stat[chip] = 0;
            jstring(j, "asic_stat", stat);
        }
        jput(j, "}");
    }
    jput(j, "]}");
}

/* ====================================================================== *
 * The dispatcher. As in the stock firmware, `summary`, `pools` and
 * `groupctrl` (with `all`, its variant with the history) answer;
 * `boardstats`, `coin` and `allinfos` return an EMPTY string. We
 * additionally answer `boards` (diagnosis), `groups` and the standby
 * commands `standby`, `resume` and `standby_state`.
 * ====================================================================== */

static void web_reply(const char *cmd, jbuf *j)
{
    if (!strcmp(cmd, "summary")) {
        web_summary(j);
    } else if (!strcmp(cmd, "pools")) {
        web_pools(j);
    } else if (!strcmp(cmd, "boards")) {
        web_boards(j);
    } else if (!strcmp(cmd, "groups")) {
        /*
         * OUR OWN COMMAND, absent from the stock firmware: the LIVE state, i.e.
         * which group feeds each chip and which rank of each group serves right
         * now. The web pages use it (get_groups.cgi, the home page's Asic
         * column) to show what runs, not what the file asks for: the two
         * differ after an environment override, a switch to a backup, or a
         * chip moved to ETC. Kept separate so that summary, pools and
         * groupctrl stay identical to the original, byte for byte.
         */
        unsigned k;

        jput(j, "{\"asic_pool_group\": \"");
        for (k = 0; k < 3; k++)
            jput(j, "%s%d", k ? "," : "", jas_chip_group(k));
        jput(j, "\", \"active\": [%d, %d]",
             jas_group_active(0), jas_group_active(1));
        jput(j, ", \"pool\": [");
        for (k = 0; k < (unsigned)jas_pool_count(); k++) {
            char url[192] = "", user[192] = "";
            int live = 0;

            if (k) jput(j, ", ");
            if (!jas_pool_describe((int)k, url, sizeof(url), user, sizeof(user),
                                  &live)) { jput(j, "{}"); continue; }
            jput(j, "{");
            jnumber(j, "pool", (long)k);                jput(j, ", ");
            jnumber(j, "group", (long)(k / 2));        jput(j, ", ");
            jnumber(j, "rank", (long)(k % 2));          jput(j, ", ");
            jstring(j, "url", url);                     jput(j, ", ");
            jstring(j, "user", user);                   jput(j, ", ");
            jstring(j, "status", live == 2 ? "In use" :
                                 live == 1 ? "Alive" : "Dead");
            jput(j, "}");
        }
        jput(j, "]}");
    } else if (!strcmp(cmd, "standby") || !strcmp(cmd, "resume")) {
        /* OUR COMMAND: standby without losing the DAG, or resume. The mining
         * loop applies the request at its next round. */
        jas_standby_requested = !strcmp(cmd, "standby");
        jput(j, "{\"standby_requested\": %d, \"in_standby\": %d}",
             jas_standby_requested, jas_in_standby);
    } else if (!strcmp(cmd, "standby_state")) {
        jput(j, "{\"standby_requested\": %d, \"in_standby\": %d}",
             jas_standby_requested, jas_in_standby);
    } else if (!strcmp(cmd, "groupctrl") || !strcmp(cmd, "all")) {
        int series = !strcmp(cmd, "all");

        jput(j, "{\"summary\": ");  web_summary(j);
        jput(j, ", \"pools\": ");   web_pools(j);
        jput(j, ", \"boards\": ");  web_boards(j);
        /* "all" adds the three series; groupctrl stops there, like the
         * original — 1,539 bytes against 27,573. */
        if (series) { jput(j, ", "); web_lines(j); }
        jput(j, "}");
    }
    /* Otherwise: nothing, like the original. An unknown command gets no
     * error message — jasminer_api does not expect one, and minerStatus.cgi
     * simply tests for the word "summary" in the answer. */
}

/* ====================================================================== *
 * The thread.
 *
 * SO_REUSEADDR: without it, a restart within the minute fails with
 * "Address already in use" and the page stays silent.
 *
 * One connection, one command, one answer, close. The answer ends with a
 * NUL byte followed by a newline: jasminer_api uses it to know it has read
 * everything.
 * ====================================================================== */

void *jas_web_thread(void *arg)
{
    int listen_fd;
    struct sockaddr_in a;
    int one = 1;

    (void)arg;
    if (start <= 0.0)
        start = instant_monotone();

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        jas_log("api: socket failed (%s)", strerror(errno));
        return NULL;
    }
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    memset(&a, 0, sizeof(a));
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port        = htons(PORT_API);

    if (bind(listen_fd, (struct sockaddr *)&a, sizeof(a)) < 0) {
        jas_log("api: port %d unavailable (%s) - the web page will stay silent",
                PORT_API, strerror(errno));
        close(listen_fd);
        return NULL;
    }
    if (listen(listen_fd, 100) < 0) {
        jas_log("api: listen failed (%s)", strerror(errno));
        close(listen_fd);
        return NULL;
    }
    jas_name_thread("api");
    jas_log("api: listening on port %d", PORT_API);

    for (;;) {
        struct sockaddr_in cli;
        socklen_t size = sizeof(cli);
        int fd = accept(listen_fd, (struct sockaddr *)&cli, &size);
        char input[INPUT_BUFFER];
        static char output[OUTPUT_BUFFER];
        ssize_t n;
        jbuf j;
        size_t written;

        if (fd < 0) {
            if (errno == EINTR)
                continue;
            jas_log("api: accept failed (%s)", strerror(errno));
            break;
        }

        n = recv(fd, input, sizeof(input) - 1, 0);
        if (n <= 0) { close(fd); continue; }
        input[n] = 0;
        while (n > 0 && (input[n - 1] == '\n' || input[n - 1] == '\r' ||
                         input[n - 1] == ' ' || input[n - 1] == '\0'))
            input[--n] = 0;

        j.b = output; j.n = 0; j.max = sizeof(output) - 2; j.truncated = 0;
        web_reply(input, &j);
        if (j.truncated) {
            /* Say so rather than send a truncated document: the page
             * would show its fallback without anyone knowing why. */
            jas_log("api: reply to \"%s\" truncated", input);
            j.n = 0;
        }
        /* The original's termination: NUL then newline. */
        output[j.n++] = '\0';
        output[j.n++] = '\n';

        for (written = 0; written < j.n; ) {
            ssize_t k = send(fd, output + written, j.n - written, MSG_NOSIGNAL);

            if (k <= 0) break;
            written += (size_t)k;
        }
        close(fd);
    }
    close(listen_fd);
    return NULL;
}

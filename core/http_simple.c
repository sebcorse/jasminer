/*
 * Small HTTP/1.0 client for the ethash files of the cache server (cache,
 * end of the DAG), shared by cpu_cache.c and the reclaim. Connection
 * bounded to 2 s, each read to 5 s.
 */
#define _GNU_SOURCE     /* strcasestr */
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include "jas_http.h"
#include "jas_hal.h"

#define HTTP_CONNECTION_S    2
#define HTTP_READ_S      5

/* Opens a GET request; returns the socket positioned on the body, the body
 * bytes already read in `rest`, and the announced length. -1 on failure. */
int jas_http_open(const char *host, int port, const char *path, const char *range,
                    char *rest, size_t *n_rest, long long *length)
{
    struct addrinfo indices, *res = NULL;
    struct timeval to = { HTTP_READ_S, 0 };
    struct pollfd pf;
    char sport[8], head[4096], req[512];
    size_t n = 0;
    int fd = -1, fl, err = 0;
    socklen_t lerr = sizeof(err);
    char *end, *cl;

    memset(&indices, 0, sizeof(indices));
    indices.ai_family = AF_INET;
    indices.ai_socktype = SOCK_STREAM;
    snprintf(sport, sizeof(sport), "%d", port);
    if (getaddrinfo(host, sport, &indices, &res) != 0 || !res)
        return -1;
    fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0)
        goto failure;
    fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    if (connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
        if (errno != EINPROGRESS)
            goto failure;
        pf.fd = fd;
        pf.events = POLLOUT;
        if (poll(&pf, 1, HTTP_CONNECTION_S * 1000) != 1
            || getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &lerr) != 0 || err != 0)
            goto failure;
    }
    fcntl(fd, F_SETFL, fl);
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof(to));
    freeaddrinfo(res);
    res = NULL;

    snprintf(req, sizeof(req), "GET %s HTTP/1.0\r\nHost: %s\r\n%s%s%sConnection: close\r\n\r\n",
             path, host, range ? "Range: " : "", range ? range : "", range ? "\r\n" : "");
    if (send(fd, req, strlen(req), 0) != (ssize_t)strlen(req))
        goto failure;
    for (;;) {
        ssize_t r = recv(fd, head + n, sizeof(head) - 1 - n, 0);

        if (r <= 0)
            goto failure;
        n += (size_t)r;
        head[n] = 0;
        if ((end = strstr(head, "\r\n\r\n")) != NULL)
            break;
        if (n >= sizeof(head) - 1)
            goto failure;
    }
    if (strncmp(head, "HTTP/1.", 7) != 0
        || (strncmp(head + 9, "200", 3) != 0 && !(range && strncmp(head + 9, "206", 3) == 0)))
        goto failure;
    cl = strcasestr(head, "\r\nContent-Length:");
    if (!cl || cl > end)
        goto failure;
    *length = strtoll(cl + 17, NULL, 10);
    end += 4;
    *n_rest = n - (size_t)(end - head);
    memcpy(rest, end, *n_rest);
    return fd;
failure:
    if (res)
        freeaddrinfo(res);
    if (fd >= 0)
        close(fd);
    return -1;
}

/* Reads exactly `size` bytes of the body into `dst`. 0 if all is there. */
int jas_http_read(int fd, const char *rest, size_t n_rest, long long length,
                  uint8_t *dst, size_t size)
{
    size_t n = n_rest;

    if (length != (long long)size || n_rest > size)
        return -1;
    memcpy(dst, rest, n_rest);
    while (n < size) {
        ssize_t r = recv(fd, dst + n, size - n, 0);

        if (r <= 0)
            return -1;
        n += (size_t)r;
    }
    return 0;
}

const char *jas_json_field(const char *j, const char *key)
{
    const char *p = strstr(j, key);

    if (!p)
        return NULL;
    p += strlen(key);
    while (*p == ' ' || *p == ':' || *p == '"')
        p++;
    return p;
}

/*
 * THE ETHASH FILE SERVER: the "cache_server" setting of
 * the Miner page, in jasminer.json; JASRE_CACHE_URL wins over it, for a
 * trial. Accepted: [http://]host[:port][/path], port 8090 and path /ethw by
 * default, as the server serves them. Empty or "0": no server, the miner
 * computes the cache itself. Read once: the page restarts the miner after a
 * save.
 */
static char server_host[128];
static char server_path[128] = "/ethw";
static int server_port = 8090;
static int server_set;
static pthread_once_t server_once = PTHREAD_ONCE_INIT;

/* The setting's value: what follows "cache_server" in the file, unquoted. */
static void server_setting(char *value, size_t n)
{
    FILE *f = fopen(jas_config_file(), "r");
    char line[512];

    value[0] = '\0';
    if (!f)
        return;
    while (fgets(line, sizeof(line), f)) {
        char *p = strstr(line, "\"cache_server\""), *q;

        if (p && (p = strchr(p + 14, ':')) && (p = strchr(p, '"'))) {
            snprintf(value, n, "%s", p + 1);
            if ((q = strchr(value, '"')))
                *q = '\0';
        }
    }
    fclose(f);
}

/* [http://]host[:port][/path] into server_*. 0 if the address is valid. */
static int server_parse(const char *v)
{
    size_t n;

    if (!strncmp(v, "http://", 7))
        v += 7;
    n = strcspn(v, ":/");
    if (n == 0 || n >= sizeof(server_host))
        return -1;
    memcpy(server_host, v, n);
    server_host[n] = '\0';
    v += n;
    if (*v == ':') {
        char *end;
        long p = strtol(v + 1, &end, 10);

        if (end == v + 1 || p < 1 || p > 65535)
            return -1;
        server_port = (int)p;
        v = end;
    }
    if (*v == '/') {
        if (strlen(v) >= sizeof(server_path))
            return -1;
        snprintf(server_path, sizeof(server_path), "%s", v);
        n = strlen(server_path);
        while (n > 0 && server_path[n - 1] == '/')
            server_path[--n] = '\0';      /* the file names add their own */
    } else if (*v != '\0') {
        return -1;
    }
    return 0;
}

static void server_read(void)
{
    const char *e = getenv("JASRE_CACHE_URL");
    const char *source = "settings page";
    char value[200];

    if (e && *e) {
        snprintf(value, sizeof(value), "%s", e);
        source = "JASRE_CACHE_URL";
    } else {
        server_setting(value, sizeof(value));
    }
    if (!value[0] || !strcmp(value, "0")) {
        jas_log("cache server: none, the miner computes the cache");
        return;
    }
    if (server_parse(value) != 0) {
        jas_log("cache server: \"%s\" (%s) is not an address, ignored: the"
                " miner computes the cache", value, source);
        return;
    }
    server_set = 1;
    jas_log("cache server: http://%s:%d%s (%s)", server_host, server_port,
            server_path, source);
}

int jas_ethash_server(int k, char *host, size_t nh, int *port, char *path, size_t nc)
{
    pthread_once(&server_once, server_read);
    if (!server_set || k != 0)
        return -1;
    snprintf(host, nh, "%s", server_host);
    *port = server_port;
    snprintf(path, nc, "%s", server_path);
    return 0;
}

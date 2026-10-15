#ifndef JAS_HTTP_H
#define JAS_HTTP_H
#include <stddef.h>
#include <stdint.h>

/* Opens a GET (range = "bytes=N-" or NULL); returns the socket positioned on
 * the body, the body bytes already read in `rest` (4096 B), the announced
 * length. -1 on failure. */
int jas_http_open(const char *host, int port, const char *path, const char *range,
                    char *rest, size_t *n_rest, long long *length);
/* Reads exactly `size` bytes of the body into `dst`. 0 if all is there. */
int jas_http_read(int fd, const char *rest, size_t n_rest, long long length,
                  uint8_t *dst, size_t size);
/* Raw value after '"key":' in a flat JSON record, NULL if absent. */
const char *jas_json_field(const char *j, const char *key);
/* The ethash file server: the "cache_server" setting of the Miner page, or
 * JASRE_CACHE_URL. 0 if there is one (k = 0 only), -1 otherwise. */
int jas_ethash_server(int k, char *host, size_t nh, int *port, char *path, size_t nc);

#endif

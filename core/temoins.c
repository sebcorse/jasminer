/*
 * temoins.c — switches driven by marker files.
 *
 * The miner is started by jasminer.sh, which passes no environment variable:
 * to turn a new feature off on one machine, you create a file. Two places:
 *   /tmp/<name>            RAM disk, wiped at the next boot of the machine —
 *                          for a trial;
 *   /media/configs/<name>  flash, survives reboots — to turn a feature off on
 *                          a flashed machine without reflashing.
 */
#include <stdio.h>
#include <unistd.h>

#include "jas_hal.h"

int jas_marker(const char *name)
{
    char path[160];

    snprintf(path, sizeof(path), "/tmp/%s", name);
    if (access(path, F_OK) == 0)
        return 1;
    snprintf(path, sizeof(path), "/media/configs/%s", name);
    return access(path, F_OK) == 0;
}

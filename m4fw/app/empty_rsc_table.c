/*
 * Empty resource table — required by remoteproc to parse the ELF.
 * A minimal resource table with no entries tells remoteproc that
 * this firmware has no resource requirements (no vdev, no trace, etc.).
 */

#include <stdint.h>

/* Resource table header expected by the Linux remoteproc framework */
struct resource_table {
    uint32_t ver;
    uint32_t num;
    uint32_t reserved[2];
};

/* Placed in a dedicated section so the ELF linker script can position it */
__attribute__((section(".resource_table"), used))
const struct resource_table rsc_table = {
    .ver      = 1,
    .num      = 0,
    .reserved = {0, 0},
};

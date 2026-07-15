/* Copyright 2020 HPS/SAFARI Research Groups (MIT license, see shim_import.h) */

/* SPP (Signature Path Prefetcher, vendor: spp.l2c + spp.h, byte-for-byte).
 * Enable by listing TYPE_SPP in one of --pref_{dcache,mlc,l1}_prefetchers with
 * its DEST_* token; also generates the ul1_cache_fill hook so SPP's
 * eviction-based training works when trained at the LLC. */

#define SHIM_NAME spp
#define SHIM_VENDOR "prefetcher/champsim/csenv/spp.l2c.inc"
#define SHIM_API_L2C 1
#define SHIM_GEN_UL1_CACHE_FILL 1
#include "prefetcher/champsim/shim_import.h"

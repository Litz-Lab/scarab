/* Copyright 2020 HPS/SAFARI Research Groups (MIT license, see shim_import.h) */

/* ChampSim's classic IP-stride prefetcher (DPC2 baseline, vendor:
 * ip_stride.l2c, byte-for-byte). Enable by listing TYPE_IPSTRIDE in one of
 * --pref_{dcache,mlc,l1}_prefetchers with its DEST_* token. The vendor tags
 * prefetches FILL_L2/FILL_LLC adaptively by MSHR occupancy; shim_op() (in
 * shim_import.h) wires real MSHR occupancy for every template-based import by
 * default, so no extra-op glue is needed here for that. --pref_ipstride_route_by_fill 1
 * routes each prefetch by the vendor's own fill-level choice instead of the
 * instance's fixed DEST_* level. */

#include "prefetcher/champsim/pref_ipstride.param.h"

#define SHIM_NAME ipstride
#define SHIM_VENDOR "prefetcher/champsim/csenv/ip_stride.l2c.inc"
#define SHIM_API_L2C 1
#define SHIM_EXTRA_INIT                                \
  g_shim._route_by_fill = PREF_IPSTRIDE_ROUTE_BY_FILL; \
  g_shim._max_pf_per_op = PREF_IPSTRIDE_MAX_PF_PER_OP;
#include "prefetcher/champsim/shim_import.h"

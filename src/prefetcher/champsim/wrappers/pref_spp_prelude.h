/* Copyright 2020 HPS/SAFARI Research Groups (MIT license, see shim_import.h) */

/* SPP vendor-constant override, included by shim_import.h INSIDE the import
 * namespace, before the vendor .inc. Pre-including spp.h here makes the .inc's
 * own `#include "spp.h"` a no-op (SPP_H guard), so its tunables can be
 * redefined without editing either vendor file. Comparison-only constants
 * become param reads; array extents must stay compile-time. */

#include "prefetcher/champsim/csenv/spp.h"

#undef PF_THRESHOLD
#define PF_THRESHOLD ((uint32_t)::PREF_SPP_PF_THRESH)
#undef FILL_THRESHOLD
#define FILL_THRESHOLD ((uint32_t)::PREF_SPP_FILL_THRESH)

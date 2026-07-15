/* Copyright 2020 HPS/SAFARI Research Groups */
/***************************************************************************************
 * File         : prefetcher/champsim/wrappers/pref_ipcp.cc
 * Description  : ChampSim-shim wrapper for the self-contained IPCP (vendor:
 *                ipcp_isca2020.l1d — it computes its own strides/streams, so it works
 *                standalone, unlike the L2 variant which needs an L1 metadata feed).
 *
 *                ONE instance, trained at exactly one Scarab cache level at a time by
 *                listing TYPE_IPCP in one of --pref_{dcache,mlc,l1}_prefetchers with
 *                its DEST_* token (dcache=L1D/DL0, mlc=L2/UMLC, l1=LLC/UL1). The
 *                single pref_table entry carries hooks for all three levels; the
 *                dispatchers fire only the listed training level's hooks, and
 *                prefetch_line routing follows the instance's DEST_* destination. No
 *                data-structure duplication (vendor tables live once in cs_ipcp).
 ***************************************************************************************/

#include <bits/stdc++.h>

extern "C" {
#include "globals/assert.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "core.param.h"
#include "general.param.h"
#include "memory/memory.param.h"
#include "prefetcher/pref.param.h"

#include "prefetcher/pref_common.h"

#include "statistics.h"
}

#include "prefetcher/champsim/champsim_shim.h"
#include "prefetcher/champsim/csenv/champsim.h"
#include "prefetcher/champsim/csenv/memory_class.h"
#include "prefetcher/champsim/csenv/ooo_cpu.h"

/* === unmodified vendor source, isolated in a private namespace === */
namespace cs_ipcp {
#include "prefetcher/champsim/csenv/ipcp_isca2020.l1d.inc"
}  // namespace cs_ipcp

static cs_ipcp::CACHE g_ipcp;
static bool g_ipcp_inited = false;

static inline uint64_t cs_ipcp_addr(Addr lineAddr) {
  return (uint64_t)(lineAddr & ((1ULL << 58) - 1ULL));
}

static inline champsim::Level cs_dest(HWP_Type d) {
  return d == PREF_TO_DCACHE ? champsim::Level::DCACHE
         : d == PREF_TO_UMLC ? champsim::Level::UMLC
                             : champsim::Level::UL1;
}

extern "C" void pref_ipcp_init(HWP* hwp) {
  if (!pref_hwp_enabled(hwp))
    return;
  int n = (pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_DCACHE) ? 1 : 0) +
          (pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_UMLC) ? 1 : 0) +
          (pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_UL1) ? 1 : 0);
  ASSERTM(0, n == 1, "champsim IPCP: list TYPE_IPCP at exactly ONE training level (single component)\n");
  Pref_Train_Level lvl = pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_DCACHE) ? PREF_TRAIN_LEVEL_DCACHE
                         : pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_UMLC) ? PREF_TRAIN_LEVEL_UMLC
                                                                                 : PREF_TRAIN_LEVEL_UL1;
  hwp->hwp_info->enabled = TRUE;
  champsim::init_globals();
  g_ipcp = cs_ipcp::CACHE();
  g_ipcp.cpu = 0;
  g_ipcp.NAME = "champsim_ipcp";
  g_ipcp._level = cs_dest(pref_hwp_instance_dest(hwp, lvl));
  g_ipcp._hwp_id = hwp->hwp_info->id;
  g_ipcp._stat_issued = PREF_SHIM_ISSUED;
  g_ipcp._stat_qfull = PREF_SHIM_QFULL;
  g_ipcp._stat_useful = PREF_SHIM_USEFUL;
  g_ipcp._stat_fill = PREF_SHIM_FILL;
  g_ipcp.l1d_prefetcher_initialize();
  g_ipcp_inited = true;
}

extern "C" void pref_ipcp_per_core_done(uns /*proc_id*/) {
}

static inline void ipcp_op(uns8 proc_id, Addr lineAddr, Addr loadPC, uint8_t cache_hit) {
  if (!g_ipcp_inited || proc_id != 0)
    return; /* single-core shim */
  STAT_EVENT(proc_id, PREF_SHIM_OPERATE);
  champsim::tick(proc_id);
  g_ipcp.cpu = proc_id;
  g_ipcp.l1d_prefetcher_operate(cs_ipcp_addr(lineAddr), (uint64_t)loadPC, cache_hit, (uint8_t)LOAD,
                                /*critical_ip_flag=*/1);
}

/* Useful-prefetch feedback: a demand hit on a line IPCP prefetched. The access's
 * own operate already ran via the regular *_hit hook, so here we ONLY credit the
 * prefetch (pref_useful[class]++), which drives the vendor's accuracy/degree throttle.
 * Fires once per prefetched line (Scarab guards on !seen_prefetch). */
static inline void ipcp_note_useful(uns8 proc_id, Addr lineAddr) {
  if (!g_ipcp_inited || proc_id != 0)
    return;
  g_ipcp.cpu = proc_id;
  g_ipcp.note_pref_hit(cs_ipcp_addr(lineAddr));
}

/* DCACHE (DL0 / L1D) hooks: signature (lineAddr, loadPC); proc_id from bit 58. */
extern "C" void pref_ipcp_dl0_miss(Addr a, Addr pc) {
  ipcp_op((uns8)(a >> 58), a, pc, 0);
}
extern "C" void pref_ipcp_dl0_hit(Addr a, Addr pc) {
  ipcp_op((uns8)(a >> 58), a, pc, 1);
}
extern "C" void pref_ipcp_dl0_pref_hit(Addr a, Addr /*pc*/) {
  ipcp_note_useful((uns8)(a >> 58), a);
}

/* MLC (UMLC / L2) hooks. */
extern "C" void pref_ipcp_mlc_miss(uns8 p, Addr a, Addr pc, uns32) {
  ipcp_op(p, a, pc, 0);
}
extern "C" void pref_ipcp_mlc_hit(uns8 p, Addr a, Addr pc, uns32) {
  ipcp_op(p, a, pc, 1);
}
extern "C" void pref_ipcp_mlc_pref_hit(uns8 p, Addr a, Addr /*pc*/, uns32) {
  ipcp_note_useful(p, a);
}

/* L1 (UL1 / LLC) hooks. */
extern "C" void pref_ipcp_l1_miss(uns8 p, Addr a, Addr pc, uns32) {
  ipcp_op(p, a, pc, 0);
}
extern "C" void pref_ipcp_l1_hit(uns8 p, Addr a, Addr pc, uns32) {
  ipcp_op(p, a, pc, 1);
}
extern "C" void pref_ipcp_l1_pref_hit(uns8 p, Addr a, Addr /*pc*/, uns32) {
  ipcp_note_useful(p, a);
}

/* MLC fill feedback: a prefetch IPCP issued filled the L2. Count it under its class
 * (pref_filled[class]++) so acc = useful/filled reflects true accuracy. Counting at
 * fill (not issue) avoids over-counting redundant/dropped issues. */
extern "C" void pref_ipcp_umlc_cache_fill(uns8 p, Addr fill_addr, Flag prefetch, Addr /*evicted*/, uns32 /*meta*/) {
  if (g_ipcp_inited && g_ipcp._level == champsim::Level::UMLC && prefetch && p == 0) {
    g_ipcp.cpu = p;
    g_ipcp.note_pref_fill(cs_ipcp_addr(fill_addr));
  }
}

/* Copyright 2020 HPS/SAFARI Research Groups */
/***************************************************************************************
 * File         : prefetcher/champsim/wrappers/pref_mlop.cc
 * Description  : ChampSim-shim wrapper for MLOP (DPC3'19). ONE instance; enable by
 *                listing TYPE_MLOP in exactly one of --pref_{dcache,mlc,l1}_prefetchers
 *                with its DEST_* token (the list selects the training level). MLOP is
 *                multi-fill-level (issues FILL_L1 and FILL_L2): with
 *                --pref_mlop_route_by_fill 1 each prefetch is routed by its own fill
 *                level (FILL_L1->L1D/dcache, FILL_L2->L2/mlc); by default (FALSE) all
 *                prefetches collapse onto the instance DEST_* level, which measures
 *                best in Scarab. Native placement is dcache (it is an L1D prefetcher).
 ***************************************************************************************/

#include <bits/stdc++.h>

#include "prefetcher/champsim/pref_mlop.param.h"

extern "C" {
#include "globals/assert.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "core.param.h"
#include "general.param.h"
#include "memory/memory.param.h"
#include "prefetcher/pref.param.h"

#include "memory/memory.h"  // mem, mem_get_req_count, uncores[].num_outstanding_l1_misses
#include "prefetcher/pref_common.h"

#include "statistics.h"
}

#include "prefetcher/champsim/champsim_shim.h"
#include "prefetcher/champsim/csenv/champsim.h"
#include "prefetcher/champsim/csenv/memory_class.h"

/* === unmodified vendor source, isolated in a private namespace === */
namespace cs_mlop {
#include "prefetcher/champsim/csenv/mlop_dpc3.l1d.inc"
}  // namespace cs_mlop

static cs_mlop::CACHE g_mlop;
static bool g_mlop_inited = false;

static inline uint64_t cs_mlop_addr(Addr lineAddr) {
  return (uint64_t)(lineAddr & ((1ULL << 58) - 1ULL));
}

static inline champsim::Level cs_dest(HWP_Type d) {
  return d == PREF_TO_DCACHE ? champsim::Level::DCACHE
         : d == PREF_TO_UMLC ? champsim::Level::UMLC
                             : champsim::Level::UL1;
}

extern "C" void pref_mlop_init(HWP* hwp) {
  if (!pref_hwp_enabled(hwp))
    return;
  int n = (pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_DCACHE) ? 1 : 0) +
          (pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_UMLC) ? 1 : 0) +
          (pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_UL1) ? 1 : 0);
  ASSERTM(0, n == 1, "champsim MLOP: list TYPE_MLOP at exactly ONE training level (single component)\n");
  Pref_Train_Level lvl = pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_DCACHE) ? PREF_TRAIN_LEVEL_DCACHE
                         : pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_UMLC) ? PREF_TRAIN_LEVEL_UMLC
                                                                                 : PREF_TRAIN_LEVEL_UL1;
  hwp->hwp_info->enabled = TRUE;
  champsim::init_globals();
  g_mlop = cs_mlop::CACHE();
  g_mlop.cpu = 0;
  g_mlop.NAME = "champsim_mlop";
  g_mlop._level = cs_dest(pref_hwp_instance_dest(hwp, lvl));
  /* Default: collapse all fills onto the drive level (best IPC in Scarab's tiny
   * L1D). Set --pref_mlop_route_by_fill 1 for faithful multi-fill (FILL_L1->L1D,
   * FILL_L2->L2; needs --pref_dl0_fill_dcache for real L1D fills). */
  g_mlop._route_by_fill = PREF_MLOP_ROUTE_BY_FILL;
  g_mlop._hwp_id = hwp->hwp_info->id;
  g_mlop._stat_issued = PREF_SHIM_ISSUED;
  g_mlop._stat_qfull = PREF_SHIM_QFULL;
  /* Cache geometry MLOP uses only to size its access-map table. */
  g_mlop.NUM_SET = 1024;
  g_mlop.NUM_WAY = 16;
  g_mlop.l1d_prefetcher_initialize();
  g_mlop_inited = true;
}

extern "C" void pref_mlop_per_core_done(uns /*proc_id*/) {
}

static inline void mlop_op(uns8 proc_id, Addr lineAddr, Addr loadPC, uint8_t cache_hit, uint8_t pref_hit) {
  if (!g_mlop_inited || proc_id != 0)
    return; /* single-core shim */
  STAT_EVENT(proc_id, PREF_SHIM_OPERATE);
  champsim::tick(proc_id);
  g_mlop.cpu = proc_id;
  g_mlop.begin_operate(); /* reset per-operate dedup set */
  /* Real backpressure (READ-ONLY Scarab state) so MLOP's built-in PQ/MSHR throttle
   * self-limits like ChampSim (MEM_REQ_BUFFER_ENTRIES==32 matches ChampSim MSHR). */
  g_mlop.MSHR.SIZE = MEM_REQ_BUFFER_ENTRIES;
  g_mlop.MSHR.occupancy = mem_get_req_count(proc_id);
  g_mlop.PQ.SIZE = PREF_UMLC_REQ_QUEUE_SIZE;
  g_mlop.PQ.occupancy = mem->uncores[proc_id].num_outstanding_l1_misses;
  /* MLOP detects a prefetch-hit via block[set][way].prefetch when cache_hit==1. */
  g_mlop._blk.prefetch = pref_hit;
  g_mlop._blk.valid = 1;
  g_mlop.l1d_prefetcher_operate(cs_mlop_addr(lineAddr), (uint64_t)loadPC, cache_hit, (uint8_t)LOAD,
                                /*critical_ip_flag=*/1);
}

/* DCACHE (DL0 / L1D) hooks: (lineAddr, loadPC); proc_id from bit 58. Native level. */
extern "C" void pref_mlop_dl0_miss(Addr a, Addr pc) {
  mlop_op((uns8)(a >> 58), a, pc, 0, 0);
}
extern "C" void pref_mlop_dl0_hit(Addr a, Addr pc) {
  mlop_op((uns8)(a >> 58), a, pc, 1, 0);
}
extern "C" void pref_mlop_dl0_pref_hit(Addr a, Addr pc) {
  mlop_op((uns8)(a >> 58), a, pc, 1, 1);
}

/* MLC (UMLC / L2) hooks. */
extern "C" void pref_mlop_mlc_miss(uns8 p, Addr a, Addr pc, uns32) {
  mlop_op(p, a, pc, 0, 0);
}
extern "C" void pref_mlop_mlc_hit(uns8 p, Addr a, Addr pc, uns32) {
  mlop_op(p, a, pc, 1, 0);
}
extern "C" void pref_mlop_mlc_pref_hit(uns8 p, Addr a, Addr pc, uns32) {
  mlop_op(p, a, pc, 1, 1);
}

/* L1 (UL1 / LLC) hooks. */
extern "C" void pref_mlop_l1_miss(uns8 p, Addr a, Addr pc, uns32) {
  mlop_op(p, a, pc, 0, 0);
}
extern "C" void pref_mlop_l1_hit(uns8 p, Addr a, Addr pc, uns32) {
  mlop_op(p, a, pc, 1, 0);
}
extern "C" void pref_mlop_l1_pref_hit(uns8 p, Addr a, Addr pc, uns32) {
  mlop_op(p, a, pc, 1, 1);
}

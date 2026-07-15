/* Copyright 2020 HPS/SAFARI Research Groups */
/***************************************************************************************
 * File         : prefetcher/champsim/champsim_shim.cc
 * Description  : Implementation of the ChampSim shim routing/glue. Translates a
 *                ChampSim prefetch_line() byte address into a Scarab cmp line index
 *                and enqueues it at the right cache level. Owns the fake ChampSim
 *                globals warmup_complete[] / current_core_cycle[].
 ***************************************************************************************/

#include "prefetcher/champsim/champsim_shim.h"

/* Fake ChampSim globals the vendor sources reference (sized to NUM_CPUS). */
#include "prefetcher/champsim/csenv/champsim.h"
#include "prefetcher/champsim/csenv/ooo_cpu.h"

extern "C" {
#include "globals/assert.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "core.param.h"
#include "general.param.h"
#include "memory/memory.param.h"

#include "prefetcher/pref_common.h"

#include "statistics.h"
}

/* Definitions of the fake ChampSim globals declared extern in the csenv headers. */
uint8_t warmup_complete[NUM_CPUS] = {0};
uint64_t current_core_cycle[NUM_CPUS] = {0};
OOO_CPU ooo_cpu[NUM_CPUS] = {};

namespace champsim {

uint64_t strip_proc(uint64_t cmp_addr) {
  return cmp_addr & ((1ULL << 58) - 1ULL);
}

void init_globals() {
  /* The shim fakes a fixed compile-time NUM_CPUS; refuse to run silently wrong
   * if Scarab has more cores than that (vendor per-core arrays would overflow). */
  ASSERTM(0, NUM_CORES <= (uns)NUM_CPUS,
          "champsim shim built for NUM_CPUS=%d but NUM_CORES=%d; bump NUM_CPUS in "
          "prefetcher/champsim/csenv/champsim.h\n",
          (int)NUM_CPUS, (int)NUM_CORES);
  for (int i = 0; i < NUM_CPUS; i++) {
    warmup_complete[i] = 1; /* treat as past warmup from the start */
    current_core_cycle[i] = 0;
  }
}

void tick(uint32_t proc_id) {
  if ((int)proc_id < NUM_CPUS) {
    warmup_complete[proc_id] = 1;
    current_core_cycle[proc_id] = (uint64_t)cycle_count;
    /* Map ChampSim's per-core retired-instruction count to Scarab's inst_count[]
     * (sim.c: "retired per core"). Used by IPCP-L1 for its MPKI throttle. */
    ooo_cpu[proc_id].num_retired = inst_count ? (uint64_t)inst_count[proc_id] : 0;
    ooo_cpu[proc_id].warmup_instructions = 0;
  }
}

int shim_issue(Level level, uint32_t proc_id, int hwp_id, uint64_t pf_byte_addr, int stat_issued, int stat_qfull) {
  uint64_t byte = strip_proc(pf_byte_addr);
  Addr cmp = convert_to_cmp_addr((uns8)proc_id, (Addr)byte);
  Addr line_index = cmp >> LOG2(DCACHE_LINE_SIZE);

  HWP_Type dest = level == Level::DCACHE ? PREF_TO_DCACHE : level == Level::UMLC ? PREF_TO_UMLC : PREF_TO_UL1;
  Flag ok = pref_addto_dest_req_queue((uns8)proc_id, dest, line_index, (uns8)hwp_id);
  if (ok) {
    if (stat_issued >= 0)
      STAT_EVENT(proc_id, stat_issued);
    return 1;
  }
  if (stat_qfull >= 0)
    STAT_EVENT(proc_id, stat_qfull);
  return 0;
}

void shim_stat(uint32_t proc_id, int stat_id) {
  if (stat_id >= 0)
    STAT_EVENT(proc_id, stat_id);
}

}  // namespace champsim

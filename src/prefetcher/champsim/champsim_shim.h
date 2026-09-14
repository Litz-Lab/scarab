/* Copyright 2020 HPS/SAFARI Research Groups */
/***************************************************************************************
 * File         : prefetcher/champsim/champsim_shim.h
 * Description  : Scarab-side core of the universal ChampSim prefetcher compatibility
 *                shim. Provides the routing/address glue that the per-prefetcher
 *                wrapper's fake CACHE object delegates to. See README in this dir.
 *
 *                MUST be included at GLOBAL scope (never inside a wrapper namespace):
 *                it declares `namespace champsim`, and the per-wrapper csenv/cache.h
 *                relies on that being the single global ::champsim.
 ***************************************************************************************/
#ifndef __CHAMPSIM_SHIM_H__
#define __CHAMPSIM_SHIM_H__

#include <cstdint>

namespace champsim {

/* Scarab destination cache level a shim instance issues prefetches into.
 * Mirrors the framework's HWP_Type; a wrapper sets it from its instance's
 * DEST_* token in the pref_{dcache,mlc,l1}_prefetchers list. */
enum class Level {
  DCACHE = 0,
  UMLC = 1,
  UL1 = 2
};

/* Map a ChampSim fill level (FILL_L1=1, FILL_L2=2, FILL_LLC=4) onto the Scarab
 * cache level it should land in. Lets a multi-fill-level prefetcher (e.g. MLOP)
 * issue to any level via the vendor's prefetch_line(fill_level) argument, instead
 * of being collapsed onto one fixed level. Unknown values default to UMLC (L2). */
inline Level level_from_fill(int fill_level) {
  switch (fill_level) {
    case 1:
      return Level::DCACHE; /* FILL_L1  -> L1D  */
    case 2:
      return Level::UMLC; /* FILL_L2  -> L2   */
    case 4:
      return Level::UL1; /* FILL_LLC -> LLC  */
    default:
      return Level::UMLC;
  }
}

/* Issue a prefetch for ChampSim byte address `pf_byte_addr` (proc bits already
 * absent or ignored) on behalf of core `proc_id` at the given Scarab level.
 * Converts byte addr -> Scarab cmp line index and routes through the framework's
 * pref_addto_dest_req_queue. Returns 1 if queued, 0 if the queue was full.
 * Increments stat id `stat_issued` (on success) or `stat_qfull` (on full);
 * pass a negative id to skip the stat. */
int shim_issue(Level level, uint32_t proc_id, int hwp_id, uint64_t pf_byte_addr, int stat_issued, int stat_qfull);

/* Real capacity of the Scarab request queue a shim instance at `level` drains
 * into (dl0req/umlc_req/ul1req -- 32/64/128 by default, pref.param.def). Lets a
 * wrapper wire its vendor's own PQ.SIZE (e.g. MLOP's occupancy-based throttle,
 * mlop_dpc3.l1d.inc:652) to the queue it actually issues into. A wrapper must
 * NOT hardcode one level's size (e.g. always PREF_UMLC_REQ_QUEUE_SIZE): the
 * instance's DEST_* token (`_level`) can place it at any of the three queues,
 * each a different size, so a fixed constant silently mis-sizes the throttle
 * whenever the instance isn't at that one hardcoded level. */
uint32_t shim_queue_size(Level level);

/* Single shared place backpressure is wired: fills the four out-params (PQ
 * size, PQ occupancy, MSHR size, MSHR occupancy) with real Scarab state for a
 * shim instance at `level`, instead of each wrapper duplicating the same four
 * assignments (or skipping them, which leaves the fake CACHE's ctor default
 * in effect -- see csenv/cache.h). Call once per operate(), before the
 * vendor's own prefetcher-operate entry point, so every import gets
 * backpressure by default; only imports whose vendor code actually reads
 * PQ/MSHR (currently MLOP, IP-stride) are affected behaviorally -- IPCP/SPP/
 * next-line never reference these fields, so wiring them is a harmless no-op
 * there. */
void shim_wire_backpressure(uint32_t proc_id, Level level, uint32_t* pq_size, uint32_t* pq_occupancy,
                            uint32_t* mshr_size, uint32_t* mshr_occupancy);

/* Fire Scarab stat id `stat_id` for core `proc_id`, or no-op if `stat_id` < 0.
 * Universal hook so any shim component (cache.h's note_pref_hit/note_pref_fill,
 * etc.) can surface an internal feedback event as a stat WITHOUT including the C
 * statistics.h inside a wrapper namespace. Each import opts in by setting the
 * matching _stat_* id on its fake CACHE; imports that don't leave it at -1. */
void shim_stat(uint32_t proc_id, int stat_id);

/* Initialize the fake ChampSim globals (warmup_complete[]/current_core_cycle[]).
 * Idempotent; safe to call from each prefetcher's init. Aborts if Scarab is
 * configured with more cores than the shim's fake NUM_CPUS (see csenv/champsim.h). */
void init_globals();

/* Refresh fake ChampSim per-core globals for `proc_id` (current cycle, warmup). */
void tick(uint32_t proc_id);

/* Strip Scarab proc-id bits (>=58) from a cmp address, yielding a clean byte addr. */
uint64_t strip_proc(uint64_t cmp_addr);

}  // namespace champsim

#endif  // __CHAMPSIM_SHIM_H__

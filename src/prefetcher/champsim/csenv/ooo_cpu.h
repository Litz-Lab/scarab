/* Universal ChampSim shim: minimal fake of ChampSim's inc/ooo_cpu.h.
 * Only the per-core retired-instruction fields a prefetcher reads (e.g. IPCP-L1
 * computes MPKI = misses*1000/(num_retired - warmup_instructions)).
 * Defined in champsim_shim.cc; num_retired is refreshed from Scarab's per-core
 * retired counter inst_count[] via champsim::tick(). Included at GLOBAL scope by
 * the wrapper so `ooo_cpu` resolves to the single global instance. */
#ifndef OOO_CPU_H
#define OOO_CPU_H

#include "champsim.h"

struct OOO_CPU {
  uint64_t num_retired;
  uint64_t warmup_instructions;
};

extern OOO_CPU ooo_cpu[NUM_CPUS];

#endif  // OOO_CPU_H

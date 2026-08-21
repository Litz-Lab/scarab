/* Universal ChampSim shim: minimal fake of ChampSim's inc/champsim.h.
 * Provides only the constants/globals that DPC3 prefetcher sources reference.
 * Included at GLOBAL scope by each wrapper (and by champsim_shim.cc) so the
 * NUM_CPUS macro, FILL_* constants, and warmup_complete[]/current_core_cycle[]
 * externs live in the global namespace, not inside a wrapper's namespace. */
#ifndef CHAMPSIM_H
#define CHAMPSIM_H

#include <cstdint>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <deque>
#include <queue>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <functional>

using namespace std;

/* CPU: the shim emulates a single-core ChampSim (matches Scarab single-core
 * experiments and the hand-port behaviour, incl. IPCP's NUM_CPUS==1 degree=3
 * branch). Bump this (and rebuild) to support multi-core; init_globals() asserts
 * NUM_CORES <= NUM_CPUS. */
#define NUM_CPUS 1

#define PAGE_SIZE 4096
#define LOG2_PAGE_SIZE 12
#define BLOCK_SIZE 64
#define LOG2_BLOCK_SIZE 6

/* ChampSim cache sizing constants some prefetchers use for table sizing.
 *
 * Authoritative definition: capacity of SPP's own lookahead confidence/delta
 * queues (spp.l2c.inc, byte-identical to upstream):
 *   confidence_q[SPP_LOOKAHEAD_QUEUE_SIZE], delta_q[SPP_LOOKAHEAD_QUEUE_SIZE]
 * filled by a pf_q_tail++ loop (PATTERN_TABLE::read_pattern) with NO bounds
 * check anywhere in the vendor file -- it stays safe only because lookahead
 * confidence empirically decays below PF_THRESHOLD well before the queue
 * could fill. This is a fixed vendor constant, NOT a model of real MSHR
 * occupancy, and it must stay a compile-time constant: do not wire it to
 * Scarab's real MSHR size (--mem_req_buffer_entries, memory.param.def) --
 * that param is runtime-configurable (e.g. `--mem_req_buffer_entries 8`),
 * which would turn these fixed-size C arrays into a stack buffer overflow the
 * first time SPP's lookahead runs deeper than the configured value. */
#define SPP_LOOKAHEAD_QUEUE_SIZE 32

/* L2C_MSHR_SIZE is the exact token spp.l2c.inc references (upstream ChampSim's
 * own, equally misleading name for this constant); kept as an alias, never
 * given its own value, so the byte-identical vendor file needs no edits while
 * every other use of the *real* constant goes through the honestly-named
 * SPP_LOOKAHEAD_QUEUE_SIZE above. */
#define L2C_MSHR_SIZE SPP_LOOKAHEAD_QUEUE_SIZE

/* Fill levels (bitmask form, as in ChampSim). The shim ignores these for queue
 * selection — routing is by the wrapper's registered Scarab Level. */
#define FILL_L1   1
#define FILL_L2   2
#define FILL_LLC  4
#define FILL_DRAM 16

/* Owned/defined in champsim_shim.cc; updated via champsim::tick()/init_globals(). */
extern uint8_t  warmup_complete[NUM_CPUS];

/* ChampSim's debug-print macro; compiled out in the shim. */
#ifndef DP
#define DP(x)
#endif
extern uint64_t current_core_cycle[NUM_CPUS];

#endif  // CHAMPSIM_H

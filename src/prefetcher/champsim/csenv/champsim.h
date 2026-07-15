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

/* ChampSim cache sizing constants some prefetchers use for table sizing. */
#define L2C_MSHR_SIZE 32

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

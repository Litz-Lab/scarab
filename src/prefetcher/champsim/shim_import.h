/* Copyright 2020 HPS/SAFARI Research Groups
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Generic ChampSim import template: expands one UNMODIFIED vendor prefetcher
 * into a complete shim wrapper (registry-gated init, operate glue, and the
 * nine per-level hook trampolines). A simple import is a ~6-line stub:
 *
 *   // wrappers/pref_nextline.cc  (complete file)
 *   #define SHIM_NAME    nextline                                  // token: names pref_<name>_* and TYPE_<NAME>
 *   #define SHIM_VENDOR  "prefetcher/champsim/csenv/next_line.l2c.inc"
 *   #define SHIM_API_L2C 1                                         // or SHIM_API_L1D: vendor entry-point family
 *   #include "prefetcher/champsim/shim_import.h"
 *
 * plus a pref_table.def row. Events count into the shared PREF_SHIM_* stats
 * (pref_champsim.stat.def).
 *
 * Optional knobs, defined before the include:
 *   SHIM_EXTRA_INIT          statements pasted into init (e.g. set _route_by_fill from a param)
 *   SHIM_EXTRA_OP            statements pasted before each operate (e.g. MSHR backpressure wiring)
 *   SHIM_GEN_UL1_CACHE_FILL  also generate pref_<name>_ul1_cache_fill -> vendor l2c_prefetcher_cache_fill
 *
 * Each import must be its OWN translation unit: the vendor file carries
 * file-scope state and CACHE method definitions, so the template wraps it in
 * a private namespace (cs_<name>) whose fake cache.h is included once per TU.
 * Imports needing custom glue beyond these knobs (ipcp's accuracy feedback,
 * mlop's fill-routing) keep hand-written wrappers instead. */

#if !defined(SHIM_NAME) || !defined(SHIM_VENDOR)
#error "define SHIM_NAME and SHIM_VENDOR before including shim_import.h"
#endif
#if !defined(SHIM_API_L1D) && !defined(SHIM_API_L2C)
#error "define SHIM_API_L1D or SHIM_API_L2C (the vendor's entry-point family)"
#endif

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

#include "memory/memory.h"
#include "prefetcher/pref_common.h"

#include "statistics.h"
}

#include "prefetcher/champsim/champsim_shim.h"
#include "prefetcher/champsim/csenv/champsim.h"
#include "prefetcher/champsim/csenv/memory_class.h"
#include "prefetcher/champsim/csenv/ooo_cpu.h"

#define SHIM_CAT2(a, b) a##b
#define SHIM_CAT(a, b) SHIM_CAT2(a, b)
#define SHIM_FN(suffix) SHIM_CAT(SHIM_CAT(pref_, SHIM_NAME), suffix)
#define SHIM_STR2(x) #x
#define SHIM_STR(x) SHIM_STR2(x)

namespace SHIM_CAT(cs_, SHIM_NAME) {
#include SHIM_VENDOR
}  // namespace SHIM_CAT(cs_,SHIM_NAME)

static SHIM_CAT(cs_, SHIM_NAME)::CACHE g_shim;
static bool g_shim_inited = false;

static inline uint64_t shim_line_addr(Addr lineAddr) {
  return (uint64_t)(lineAddr & ((1ULL << 58) - 1ULL));
}

static inline champsim::Level shim_dest(HWP_Type d) {
  return d == PREF_TO_DCACHE ? champsim::Level::DCACHE
         : d == PREF_TO_UMLC ? champsim::Level::UMLC
                             : champsim::Level::UL1;
}

extern "C" void SHIM_FN(_init)(HWP* hwp) {
  if (!pref_hwp_enabled(hwp))
    return;
  int n = (pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_DCACHE) ? 1 : 0) +
          (pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_UMLC) ? 1 : 0) +
          (pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_UL1) ? 1 : 0);
  ASSERTM(0, n == 1, "champsim %s: enable at exactly ONE training level (single component)\n", SHIM_STR(SHIM_NAME));
  Pref_Train_Level lvl = pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_DCACHE) ? PREF_TRAIN_LEVEL_DCACHE
                         : pref_hwp_instance_enabled(hwp, PREF_TRAIN_LEVEL_UMLC) ? PREF_TRAIN_LEVEL_UMLC
                                                                                 : PREF_TRAIN_LEVEL_UL1;
  hwp->hwp_info->enabled = TRUE;
  champsim::init_globals();
  g_shim = SHIM_CAT(cs_, SHIM_NAME)::CACHE();
  g_shim.cpu = 0;
  g_shim.NAME = "champsim_" SHIM_STR(SHIM_NAME);
  g_shim._level = shim_dest(pref_hwp_instance_dest(hwp, lvl));
  g_shim._hwp_id = hwp->hwp_info->id;
  g_shim._stat_issued = PREF_SHIM_ISSUED;
  g_shim._stat_qfull = PREF_SHIM_QFULL;
#ifdef SHIM_EXTRA_INIT
  SHIM_EXTRA_INIT
#endif
#ifdef SHIM_API_L1D
  g_shim.l1d_prefetcher_initialize();
#else
  g_shim.l2c_prefetcher_initialize();
#endif
  g_shim_inited = true;
}

extern "C" void SHIM_FN(_per_core_done)(uns /*proc_id*/) {
}

static inline void shim_op(uns8 proc_id, Addr lineAddr, Addr loadPC, uint8_t cache_hit) {
  if (!g_shim_inited || proc_id != 0)
    return; /* single-core shim */
  STAT_EVENT(proc_id, PREF_SHIM_OPERATE);
  champsim::tick(proc_id);
  g_shim.cpu = proc_id;
#ifdef SHIM_EXTRA_OP
  SHIM_EXTRA_OP
#endif
#ifdef SHIM_API_L1D
  g_shim.l1d_prefetcher_operate(shim_line_addr(lineAddr), (uint64_t)loadPC, cache_hit, (uint8_t)LOAD,
                                /*critical_ip_flag=*/1);
#else
  g_shim.l2c_prefetcher_operate(shim_line_addr(lineAddr), (uint64_t)loadPC, cache_hit, (uint8_t)LOAD,
                                /*metadata_in=*/0, /*critical_ip_flag=*/1);
#endif
}

/* DCACHE (DL0 / L1D) hooks: (lineAddr, loadPC); proc_id from bit 58. */
extern "C" void SHIM_FN(_dl0_miss)(Addr a, Addr pc) {
  shim_op((uns8)(a >> 58), a, pc, 0);
}
extern "C" void SHIM_FN(_dl0_hit)(Addr a, Addr pc) {
  shim_op((uns8)(a >> 58), a, pc, 1);
}
extern "C" void SHIM_FN(_dl0_pref_hit)(Addr a, Addr pc) {
  shim_op((uns8)(a >> 58), a, pc, 1);
}

/* MLC (UMLC / L2) hooks. */
extern "C" void SHIM_FN(_mlc_miss)(uns8 p, Addr a, Addr pc, uns32) {
  shim_op(p, a, pc, 0);
}
extern "C" void SHIM_FN(_mlc_hit)(uns8 p, Addr a, Addr pc, uns32) {
  shim_op(p, a, pc, 1);
}
extern "C" void SHIM_FN(_mlc_pref_hit)(uns8 p, Addr a, Addr pc, uns32) {
  shim_op(p, a, pc, 1);
}

/* L1 (UL1 / LLC) hooks. */
extern "C" void SHIM_FN(_l1_miss)(uns8 p, Addr a, Addr pc, uns32) {
  shim_op(p, a, pc, 0);
}
extern "C" void SHIM_FN(_l1_hit)(uns8 p, Addr a, Addr pc, uns32) {
  shim_op(p, a, pc, 1);
}
extern "C" void SHIM_FN(_l1_pref_hit)(uns8 p, Addr a, Addr pc, uns32) {
  shim_op(p, a, pc, 1);
}

#ifdef SHIM_GEN_UL1_CACHE_FILL
/* UL1 (LLC) fill/eviction training hook -> vendor l2c_prefetcher_cache_fill. */
extern "C" void SHIM_FN(_ul1_cache_fill)(uns8 proc_id, Addr fill_addr, Flag prefetch, Addr evicted_addr,
                                         uns32 metadata) {
  if (!g_shim_inited || proc_id != 0)
    return;
  STAT_EVENT(proc_id, PREF_SHIM_CACHE_FILL);
  g_shim.cpu = proc_id;
  g_shim.l2c_prefetcher_cache_fill(shim_line_addr(fill_addr), /*set=*/0, /*match=*/0, (uint8_t)(prefetch ? 1 : 0),
                                   evicted_addr ? shim_line_addr(evicted_addr) : 0, metadata);
}
#endif

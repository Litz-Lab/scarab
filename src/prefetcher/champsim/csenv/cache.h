/* Universal ChampSim shim: fake of ChampSim's inc/cache.h.
 *
 * This header is included by each VENDOR prefetcher source via its bare
 * `#include "cache.h"`, which the compiler resolves to THIS file via
 * current-file-directory lookup (the vendor .inc lives in this same csenv/ dir).
 * Because the vendor source is wrapped in `namespace cs_<name> { ... }`, this
 * CACHE class becomes `cs_<name>::CACHE` — a DISTINCT class per prefetcher, so
 * `CACHE::l2c_prefetcher_operate` definitions in different vendor files never
 * collide. (champsim.h / memory_class.h / champsim_shim.h are pre-included at
 * GLOBAL scope by the wrapper, so their include guards make the includes below
 * no-ops here — keeping NUM_CPUS, FILL_*, and ::champsim global.)
 */
#ifndef CHAMPSIM_COMPAT_CACHE_H
#define CHAMPSIM_COMPAT_CACHE_H

#include "champsim.h"
#include "memory_class.h"
#include "prefetcher/champsim/champsim_shim.h"

/* Minimal per-block metadata a prefetcher may inspect (e.g. MLOP reads
 * block[set][way].prefetch to detect a prefetch hit). */
struct CS_BLOCK {
  uint8_t prefetch;
  uint8_t valid;
};

/* Minimal queue model for bandwidth-aware prefetchers (PQ.occupancy/SIZE,
 * MSHR.occupancy/SIZE). Defaults make "queue never full" so prefetchers that
 * gate on occupancy behave like the hand-ports (no PQ throttle). */
struct CS_QUEUE {
  uint32_t occupancy;
  uint32_t SIZE;
};

class CACHE {
 public:
  /* ---- ChampSim-visible members vendor sources touch ---- */
  uint32_t cpu;
  const char* NAME;
  uint32_t NUM_SET;
  uint32_t NUM_WAY;
  CS_QUEUE PQ;
  CS_QUEUE MSHR;

  /* ---- Scarab-side glue (not part of the ChampSim contract) ---- */
  champsim::Level _level;   /* destination level (instance's DEST_* token) when !_route_by_fill */
  bool _route_by_fill;      /* if true, route each prefetch by its fill_level arg
                               (FILL_L1->DL0, FILL_L2->UMLC, FILL_LLC->UL1) so a
                               multi-fill-level prefetcher hits all levels as designed */
  bool _dedup_per_op;       /* if true, collapse duplicate blocks issued within one
                               operate() to a single prefetch (e.g. MLOP re-issues a
                               block when upgrading FILL_L2->FILL_L1; on a single
                               Scarab level those are redundant). */
  std::unordered_set<uint64_t> _op_issued;  /* blocks issued in the current operate */
  /* Inter-level metadata channel: when set, every prefetch_line() this component
   * issues also forwards (pf_addr, ip, metadata) downstream. Used to feed a
   * cooperating next-level component (e.g. IPCP L1 -> L2), reproducing ChampSim's
   * pf_metadata handoff where an L1 prefetch arriving at L2 triggers l2c operate. */
  std::function<void(uint64_t /*pf_addr*/, uint64_t /*ip*/, uint32_t /*metadata*/)> _downstream_op;
  int _hwp_id;
  int _stat_issued;
  int _stat_qfull;
  /* Optional observability for the accuracy feedback loop (acc = useful/filled).
   * Set by a wrapper to its prefetcher's stat ids; -1 = don't count. Lets us read
   * the otherwise-internal pref_useful/pref_filled counters from memory.stat. */
  int _stat_useful;
  int _stat_fill;
  uint8_t _cur_pref_hit;   /* set by wrapper before each operate; surfaced via block[][].prefetch */
  CS_BLOCK _blk;           /* the single fake block get_set/get_way always resolve to */

  /* Prefetch-accuracy accounting some prefetchers (IPCP-L1) read for degree
   * adaptation. ChampSim's cache.cc updates these on fill/hit/MSHR-merge; the shim
   * approximates that by attributing each successfully issued prefetch to a class
   * (pf_metadata bits 8-11, ChampSim convention), then bumping pref_filled[class]
   * at FILL (note_pref_fill, from the umlc_cache_fill hook) and pref_useful[class]
   * on a demand hit to a prefetched line (note_pref_hit, from the pref-hit hooks).
   * This restores IPCP's accuracy-based degree throttle; without it acc sticks at
   * the 60% default and IPCP over-prefetches on irregular workloads. */
  uint64_t pref_filled[NUM_CPUS][6] = {};
  uint64_t pref_useful[NUM_CPUS][6] = {};
  uint64_t pref_late[NUM_CPUS][6] = {};
  uint64_t pf_fill = 0;
  uint64_t pf_useful = 0;
  /* block-addr -> class table so a later demand pref-hit can be attributed to the
   * class that issued it (Scarab cache lines don't carry pf_metadata). Direct-mapped;
   * collisions just misattribute a hit between classes (benign for the ratio). */
  static const uint32_t CLASS_TBL_MASK = (1u << 16) - 1u;
  uint8_t _pf_class_tbl[1u << 16] = {};
  void note_pref_hit(uint64_t pf_addr) {
    uint8_t cls = _pf_class_tbl[(pf_addr >> 6) & CLASS_TBL_MASK];
    if (cls < 6)
      pref_useful[cpu][cls]++;
    champsim::shim_stat(cpu, _stat_useful);
  }
  /* A prefetch actually filled the cache: count it under its issuing class. acc =
   * useful/filled then reflects true prefetch accuracy (fills that got demand-hit). */
  void note_pref_fill(uint64_t fill_addr) {
    uint8_t cls = _pf_class_tbl[(fill_addr >> 6) & CLASS_TBL_MASK];
    if (cls < 6)
      pref_filled[cpu][cls]++;
    champsim::shim_stat(cpu, _stat_fill);
  }

  CACHE()
      : cpu(0), NAME("champsim"), NUM_SET(1024), NUM_WAY(16),
        PQ{0u, (1u << 20)}, MSHR{0u, (1u << 20)},
        _level(champsim::Level::UMLC), _route_by_fill(false), _dedup_per_op(false),
        _hwp_id(0), _stat_issued(-1), _stat_qfull(-1),
        _stat_useful(-1), _stat_fill(-1), _cur_pref_hit(0) {
    _blk.prefetch = 0;
    _blk.valid = 1;
    block.b = &_blk;
  }

  /* ---- ChampSim callbacks INTO the cache ---- */
  int prefetch_line(uint64_t ip, uint64_t /*base_addr*/, uint64_t pf_addr,
                    int fill_level, uint32_t metadata) {
    if (_dedup_per_op) {
      uint64_t blk = (pf_addr & ((1ULL << 58) - 1ULL)) >> 6;  /* LOG2_BLOCK_SIZE */
      if (!_op_issued.insert(blk).second)
        return 1;  /* already issued this operate; suppress redundant re-issue */
    }
    champsim::Level tgt = _route_by_fill ? champsim::level_from_fill(fill_level) : _level;
    int ok = champsim::shim_issue(tgt, cpu, _hwp_id, pf_addr, _stat_issued, _stat_qfull);
    if (ok) {
      /* Remember block->class (from pf_metadata bits 8-11) so the eventual fill and
       * demand pref-hit can be attributed to the issuing class. pref_filled is bumped
       * at FILL (note_pref_fill), not here -- counting every issue overcounts (lookahead
       * re-issues, drops, redundant hits), deflating acc and over-throttling. */
      _pf_class_tbl[(pf_addr >> 6) & CLASS_TBL_MASK] = (uint8_t)((metadata >> 8) & 0x7u);
    }
    /* Feed the cooperating downstream component this prefetch's metadata (ChampSim:
     * an L1-issued prefetch reaching L2 triggers l2c_prefetcher_operate). */
    if (_downstream_op)
      _downstream_op(pf_addr, ip, metadata);
    return ok;
  }
  void begin_operate() { if (_dedup_per_op) _op_issued.clear(); }
  uint32_t get_occupancy(uint8_t /*queue_type*/, uint64_t /*addr*/) { return 0; }
  uint32_t get_size(uint8_t /*queue_type*/, uint64_t /*addr*/) { return PQ.SIZE; }

  /* Tag-array access faked to a single block reflecting the current access's
   * prefetch-hit-ness (Scarab tells us which hook fired). get_set/get_way
   * always resolve to that one block. */
  uint32_t get_set(uint64_t /*addr*/) { return 0; }
  uint32_t get_way(uint64_t /*addr*/, uint32_t /*set*/) { return 0; }

  struct BlockRow {
    CS_BLOCK* b;
    CS_BLOCK& operator[](uint32_t) { return *b; }
  };
  struct BlockArr {
    CS_BLOCK* b;
    BlockRow operator[](uint32_t) { return BlockRow{b}; }
  } block;   /* vendor accesses as block[set][way] */

  /* ---- Prefetcher module entry points (vendor defines whichever family it uses) ---- */
  /* L2C family */
  void l2c_prefetcher_initialize();
  uint32_t l2c_prefetcher_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit,
                                  uint8_t type, uint32_t metadata_in,
                                  uint8_t critical_ip_flag);
  uint32_t l2c_prefetcher_cache_fill(uint64_t addr, uint32_t set, uint32_t way,
                                     uint8_t prefetch, uint64_t evicted_addr,
                                     uint32_t metadata_in);
  void l2c_prefetcher_final_stats();

  /* L1D family */
  void l1d_prefetcher_initialize();
  void l1d_prefetcher_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit,
                              uint8_t type, uint8_t critical_ip_flag);
  void l1d_prefetcher_cache_fill(uint64_t v_addr, uint64_t addr, uint32_t set,
                                 uint32_t way, uint8_t prefetch,
                                 uint64_t v_evicted_addr, uint64_t evicted_addr,
                                 uint32_t metadata_in);
  void l1d_prefetcher_final_stats();

  /* LLC family */
  void llc_prefetcher_initialize();
  uint32_t llc_prefetcher_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit,
                                  uint8_t type, uint32_t metadata_in);
  uint32_t llc_prefetcher_cache_fill(uint64_t addr, uint32_t set, uint32_t way,
                                     uint8_t prefetch, uint64_t evicted_addr,
                                     uint32_t metadata_in);
  void llc_prefetcher_final_stats();
};

#endif  // CHAMPSIM_COMPAT_CACHE_H

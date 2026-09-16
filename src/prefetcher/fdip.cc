#include "prefetcher/fdip.h"

#include "frontend/pt_memtrace/trace_fe.h"

#include "decoupled_frontend.h"
#include "sim.h"

#include "prefetcher/udp.hpp"
#include "prefetcher/uftq.hpp"

extern "C" {
#include "memory/memory.param.h"
#include "prefetcher/pref.param.h"

#include "memory/memory.h"
#include "prefetcher/eip.h"

#include "op.h"
}

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_FDIP, ##args)
#define FDIP_PREF_STAT_COUNT 12

using namespace std;

int per_cyc_ipref = 0;

template <typename A, typename B>
pair<B, A> flip_pair(const pair<A, B>& p) {
  return pair<B, A>(p.second, p.first);
}

template <typename A, typename B>
multimap<B, A> flip_map(const map<A, B>& src) {
  multimap<B, A> dst;
  transform(src.begin(), src.end(), inserter(dst, dst.begin()), flip_pair<A, B>);
  return dst;
}

typedef enum FDIP_BREAK_enum {
  BR_REACH_FTQ_END,
  BR_FTQ_EMPTY,
  BR_MAX_FTQ_ENTRY_CYC,
  BR_FULL_MEM_REQ_BUF,
  BR_BREAK_END_INVALID,
} FDIP_Break;

typedef enum UTILITY_PREF_POLICY_enum {
  PREF_CONV_FROM_USEFUL_SET,
  PREF_OPT_FROM_UNUSEFUL_SET,
  PREF_CONV_FROM_THROTTLE_CNT,
  PREF_OPT_FROM_THROTTLE_CNT,
  PREF_POL_END,  // add a new policy above this line
} Utility_Pref_Policy;

class FDIP_Stat {
 public:
  FDIP_Stat(uns _proc_id, uns _bp_id)
      : proc_id(_proc_id),
        bp_id(_bp_id),
        last_imiss_reason(Imiss_Reason::IMISS_NOT_PREFETCHED),
        last_break_reason(BR_REACH_FTQ_END),
        last_recover_cycle(0),
        cur_line_delay(0),
        ftq_occupancy_ops(0),
        ftq_occupancy_blocks(0) {}
  void inc_prefetched_cls(Addr line_addr, Flag on_path, uns success);
  void not_prefetch(Addr line_addr);
  void print_cl_info(Icache_Stage* ic_ref);
  void inc_cnt_useful(Addr line_addr, Flag pref_miss);
  void inc_cnt_useful_signed(Addr line_addr);
  void dec_cnt_useful_signed(Addr line_addr);
  void inc_icache_miss(Addr line_addr);
  void probe_prefetched_cls(Addr line_addr);
  void inc_icache_hit(Addr line_addr);
  void inc_cnt_unuseful(Addr line_addr);

 private:
  uns proc_id;
  uns bp_id;
  /* global variables for utility study and stats */
  // for icache miss stats
  uns last_imiss_reason;
  // for assertions
  uns last_break_reason;
  Counter last_recover_cycle;
  // <CL address, # of first demand load on-path hits of cache lines, flag for learning from a true miss> - useful count
  unordered_map<Addr, pair<Counter, Flag>> cnt_useful;
  // <CL address, # of first demand load on-path hits of cache lines, flag for learning from a true miss> - useful count
  // after warm-up
  unordered_map<Addr, pair<Counter, Flag>> cnt_useful_aw;
  // <CL address, # of evictions w/o hit of cache lines> - unuseful count
  unordered_map<Addr, Counter> cnt_unuseful;
  // <CL address, # of evictions w/o hit of cache lines> - unuseful count after warm-up
  unordered_map<Addr, Counter> cnt_unuseful_aw;
  // Increment if useful by UDP_WEIGHT_USEFUL, decrement if unuseful by UDP_WEIGHT_UNUSEFUL
  // <CL address, counter for on/off-path unuseful/useful> init by UDP_USEFUL_THRESHOLD
  // OPTIMISTIC POLICY : do not prefetch if < USEFUL_THRESHOLD, otherwise, prefetch (do not prefetch only when it was
  // unuseful at least once) CONSERVATIVE POLICY : prefetch if > USEFUL_THRESHOLD, otherwise, do not prefetch (prefetch
  // only when it was useful at least once)
  unordered_map<Addr, int32_t> cnt_useful_signed;
  // <CL addresses, retirement count> - on-path retired cache line count
  unordered_map<Addr, Counter> cnt_useful_ret;
  // <CL addresses, icache miss count>
  map<Addr, Counter> icache_miss;
  // <CL addresses, icache miss count> after warm-up
  map<Addr, Counter> icache_miss_aw;
  // <CL addresses, icache hit count>
  map<Addr, Counter> icache_hit;
  // <CL addresses, icache hit count> after warm-up
  map<Addr, Counter> icache_hit_aw;
  // <CL addresses, fetched_cycle on the off-path>
  map<Addr, Counter> off_fetched_cls;
  ;
  // <CL addresses, prefetched count>
  map<Addr, Counter> prefetched_cls;
  // <CL addresses, prefetched count> after warm-up
  map<Addr, Counter> prefetched_cls_aw;
  // <CL addresses, new_prefetched count>
  map<Addr, Counter> new_prefetched_cls;
  // <CL addresses, new_prefetched count> after warm-up
  map<Addr, Counter> new_prefetched_cls_aw;
  // <CL address, cyc_access_by_fdip, conf_on/off-path, cyc_evicted_from_l1_by_demand_load, cyc_evicted_from_l1_by_FDIP>
  // - prefetched and access time information for timeliness analysis
  unordered_map<Addr, pair<pair<Counter, Flag>, pair<Counter, Counter>>> prefetched_cls_info;
  // Per-cache-line usefulness history emitted by print_cl_info(). The vectors
  // are populated only when FDIP_PRINT_CL_INFO is enabled.
  unordered_map<Addr, vector<uns8>> usefulness_history;
  // Tracks first recorded I-cache access independently of optional history output.
  unordered_set<Addr> accessed_cachelines;
  // Per-cache-line hit/miss history emitted by print_cl_info(). Allocate and
  // populate entries only when FDIP_PRINT_CL_INFO is enabled.
  unordered_map<Addr, vector<uns8>> icache_access_history;
  // Consumers only need to know whether each cache-line event type occurred;
  // a bitmask avoids retaining duplicate events, ordering, and cycle timestamps.
  enum CachelineEvent : uint8_t {
    CACHELINE_EVENT_NOT_PREFETCHED = 1u << 0,
    CACHELINE_EVENT_PREFETCHED = 1u << 1,
    CACHELINE_EVENT_ICACHE_MISS = 1u << 2,
    CACHELINE_EVENT_ICACHE_HIT = 1u << 3,
    CACHELINE_EVENT_UNUSEFUL = 1u << 4,
    CACHELINE_EVENT_USEFUL = 1u << 5,
    CACHELINE_EVENT_EVICTED = 1u << 6,
  };
  unordered_map<Addr, uint8_t> cacheline_events;

  // Complete per-cache-line event history emitted by print_cl_info(), including
  // event order, duplicates, and cycle timestamps. Populate it only when
  // FDIP_PRINT_CL_INFO is enabled; otherwise this history
  // is never consumed and retaining it causes unnecessary memory growth.
  unordered_map<Addr, vector<pair<char, Counter>>> cacheline_event_history;

  // <CL address, total miss delay>
  map<Addr, Counter> per_line_delay_aw;
  Counter cur_line_delay;
  // accumulated FTQ occupancy every cycle
  uint64_t ftq_occupancy_ops;
  uint64_t ftq_occupancy_blocks;
  friend class FDIP;
};

class FDIP {
 public:
  FDIP()
      : ic_ref(nullptr),
        cur_op(nullptr),
        last_line_addr(0),
        warmed_up(FALSE),
        fdip_stat(nullptr),
        ftq_idx(0),
        dfe(nullptr),
        udp(nullptr),
        uftq(nullptr) {}
  ~FDIP();
  void init(uns proc_id, uns bp_id, Icache_Stage* _ic);
  void recover();
  void update();
  void insert_pref_candidate_to_seniority_ftq(Addr line_addr, uint64_t ghist);
  void set_ic_ref(Icache_Stage* ic) { ic_ref = ic; }
  Icache_Stage* get_ic_ref() { return ic_ref; }
  Flag is_off_path();
  Flag is_conf_off_path();
  Counter get_last_recover_cycle() { return fdip_stat->last_recover_cycle; }
  Op* get_cur_op() { return cur_op; }
  void update_unuseful_lines_uc(Addr line_addr);
  void update_useful_lines_uc(Addr line_addr);
  void update_useful_lines_bloom_filter(Addr line_addr);
  Flag get_warmed_up() { return warmed_up; }
  void print_cl_info() { fdip_stat->print_cl_info(ic_ref); }
  uns get_proc_id() { return proc_id; }
  uns get_bp_id() { return bp_id; }
  void inc_cnt_useful(Addr line_addr, Flag pref_miss) { fdip_stat->inc_cnt_useful(line_addr, pref_miss); }
  void inc_cnt_unuseful(Addr line_addr);
  void inc_cnt_useful_signed(Addr line_addr) { fdip_stat->inc_cnt_useful_signed(line_addr); }
  void dec_cnt_useful_signed(Addr line_addr) { fdip_stat->dec_cnt_useful_signed(line_addr); }
  void inc_icache_miss(Addr line_addr) { fdip_stat->inc_icache_miss(line_addr); }
  void evict_prefetched_cls(Addr line_addr, Flag by_fdip);
  uns get_miss_reason(Addr line_addr);
  uns get_last_miss_reason();
  void set_last_miss_reason(uns reason) { fdip_stat->last_imiss_reason = reason; }
  void inc_utility_info(Flag useful);
  void inc_timeliness_info(Flag mshr_hit);
  Flag search_pref_candidate(Addr addr);
  void dec_useful_lines_uc(Addr line_addr);
  void inc_useful_lines_uc(Addr line_addr);
  void assert_break_reason(Addr line_addr);
  void inc_icache_hit(Addr line_addr) { fdip_stat->inc_icache_hit(line_addr); }
  void add_evict_seq(Addr line_addr);
  void inc_off_fetched_cls(Addr line_addr);
  uint64_t get_ftq_occupancy_ops();
  uint64_t get_ftq_occupancy();
  Flag determine_usefulness(Addr line_addr, Op* op, uint64_t ghist);
  void determine_usefulness_by_inf_hash(Addr line_addr, Flag* emit_new_prefetch, Op* op, uint64_t ghist);
  void determine_usefulness_by_utility_cache(Addr line_addr, Flag* emit_new_prefetch, Op* op, uint64_t ghist);
  void determine_usefulness_by_bloom_filter(Addr line_addr, Flag* emit_new_prefetch, Op* op, uint64_t ghist);
  void inc_prefetched_cls(Addr line_addr, uns success, Flag conf_off_path);
  void log_stats_path_conf_per_pref_candidate();
  void log_stats_path_conf_emitted();
  FDIP_Stat* get_fdip_stat() { return fdip_stat; }
  UDP* get_udp() { return udp; }
  UFTQ* get_uftq() { return uftq; }

 protected:
  uns proc_id = 0;
  uns bp_id = 0;
  Icache_Stage* ic_ref;
  Op* cur_op;
  Addr last_line_addr;
  Flag warmed_up;
  FDIP_Stat* fdip_stat;
  uns ftq_idx;
  Decoupled_FE* dfe;
  UDP* udp;
  UFTQ* uftq;
};

/* Global Variables */
FDIP* g_fdip = NULL;

// Per core FDIP
vector<vector<unique_ptr<FDIP>>> per_core_fdip;

/* Wrapper functions */
void alloc_mem_fdip(uns numCores, uns numBPs) {
  per_core_fdip.reserve(numCores);
  for (uns i = 0; i < numCores; ++i) {
    vector<unique_ptr<FDIP>> fdip_vec;
    fdip_vec.reserve(numBPs);
    for (uns j = 0; j < numBPs; ++j)
      fdip_vec.emplace_back(make_unique<FDIP>());
    per_core_fdip.emplace_back(std::move(fdip_vec));
  }

  if (FDIP_UTILITY_HASH_ENABLE)
    ASSERT(0, FDIP_UTILITY_PREF_POLICY >= Utility_Pref_Policy::PREF_CONV_FROM_USEFUL_SET &&
                  FDIP_UTILITY_PREF_POLICY < Utility_Pref_Policy::PREF_POL_END);
}

void init_fdip(uns proc_id, uns bp_id, Icache_Stage* ic) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(proc_id, WP_COLLECT_STATS);
  per_core_fdip[proc_id][bp_id]->init(proc_id, bp_id, ic);
}

void set_fdip(uns proc_id, uns bp_id) {
  g_fdip = per_core_fdip[proc_id][bp_id].get();
}

void recover_fdip(uns proc_id, uns bp_id) {
  if (!FDIP_ENABLE)
    return;
  g_fdip = per_core_fdip[proc_id][bp_id].get();
  ASSERT(proc_id, g_fdip);
  ASSERT(proc_id, g_fdip->get_proc_id() == proc_id);
  ASSERT(proc_id, g_fdip->get_bp_id() == bp_id);
  per_core_fdip[proc_id][bp_id]->recover();
}

void update_fdip(uns proc_id, uns bp_id) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(proc_id, g_fdip->get_proc_id() == proc_id);
  ASSERT(proc_id, g_fdip->get_bp_id() == bp_id);
  per_core_fdip[proc_id][bp_id]->update();
}

uns64 fdip_hash_addr_ghist(uint64_t addr, uint64_t ghist) {
  // ghist is 32 bit, most recent branch outcome at bit 31
  return addr ^ ((ghist >> (32 - FDIP_GHIST_BITS)) << (64 - FDIP_GHIST_BITS));
}

Flag fdip_off_path(uns proc_id, uns bp_id) {
  ASSERT(proc_id, g_fdip->get_proc_id() == proc_id);
  return per_core_fdip[proc_id][bp_id]->is_off_path();
}

Flag fdip_conf_off_path() {
  return g_fdip->is_conf_off_path();
}

void print_cl_info(uns proc_id) {
  if (!FDIP_ENABLE)
    return;
  per_core_fdip[proc_id][0]->print_cl_info();
}

uint64_t get_fdip_ftq_occupancy_ops(uns proc_id, uns bp_id) {
  return per_core_fdip[proc_id][bp_id]->get_ftq_occupancy_ops();
}

uint64_t get_fdip_ftq_occupancy(uns proc_id, uns bp_id) {
  return per_core_fdip[proc_id][bp_id]->get_ftq_occupancy();
}

void fdip_stats(uns proc_id) {
  if (!FDIP_ENABLE)
    return;
  if (FDIP_PRINT_CL_INFO)
    print_cl_info(proc_id);
  for (uns bp_id = 0; bp_id < NUM_BPS; bp_id++) {
    INC_STAT_EVENT(proc_id, FDIP_AVG_FTQ_OCCUPANCY_OPS + 2 * bp_id, get_fdip_ftq_occupancy_ops(proc_id, bp_id));
    INC_STAT_EVENT(proc_id, FDIP_AVG_FTQ_OCCUPANCY + 2 * bp_id, get_fdip_ftq_occupancy(proc_id, bp_id));
  }
}

void inc_cnt_useful(uns proc_id, Addr line_addr, Flag pref_miss) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(proc_id, g_fdip->get_proc_id() == proc_id);
  ASSERT(proc_id, g_fdip->get_bp_id() == 0);
  g_fdip->inc_cnt_useful(line_addr, pref_miss);
}

void inc_cnt_unuseful(uns proc_id, Addr line_addr) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(proc_id, g_fdip->get_proc_id() == proc_id);
  ASSERT(proc_id, g_fdip->get_bp_id() == 0);
  g_fdip->inc_cnt_unuseful(line_addr);
}

void inc_cnt_useful_signed(Addr line_addr) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->inc_cnt_useful_signed(line_addr);
}

void dec_cnt_useful_signed(Addr line_addr) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->dec_cnt_useful_signed(line_addr);
}

void inc_icache_miss(Addr line_addr) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->inc_icache_miss(line_addr);
}

void inc_off_fetched_cls(Addr line_addr) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->inc_off_fetched_cls(line_addr);
}

void evict_prefetched_cls(Addr line_addr, Flag by_fdip) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->evict_prefetched_cls(line_addr, by_fdip);
}

uns get_miss_reason(Addr line_addr) {
  if (!FDIP_ENABLE)
    return IMISS_NOT_PREFETCHED;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  return g_fdip->get_miss_reason(line_addr);
}

uns get_last_miss_reason() {
  if (!FDIP_ENABLE)
    return IMISS_NOT_PREFETCHED;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  return g_fdip->get_last_miss_reason();
}

void set_last_miss_reason(uns reason) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->set_last_miss_reason(reason);
}

void update_unuseful_lines_uc(Addr line_addr) {
  if (!FDIP_UC_SIZE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->update_unuseful_lines_uc(line_addr);
}

void update_useful_lines_uc(Addr line_addr) {
  if (!FDIP_UC_SIZE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->update_useful_lines_uc(line_addr);
}

void update_useful_lines_bloom_filter(Addr line_addr) {
  if (!FDIP_BLOOM_FILTER)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->update_useful_lines_bloom_filter(line_addr);
}

void inc_utility_info(Flag useful) {
  if (FDIP_ADJUSTABLE_FTQ != 1 && FDIP_ADJUSTABLE_FTQ != 3)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->inc_utility_info(useful);
}

void inc_timeliness_info(Flag mshr_hit) {
  if (FDIP_ADJUSTABLE_FTQ != 2 && FDIP_ADJUSTABLE_FTQ != 3)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  g_fdip->inc_timeliness_info(mshr_hit);
}

Flag fdip_search_pref_candidate(Addr addr) {
  if (!FDIP_ENABLE)
    return TRUE;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  if (!FDIP_UTILITY_HASH_ENABLE && !FDIP_UC_SIZE && !FDIP_BLOOM_FILTER)
    return TRUE;
  return g_fdip->search_pref_candidate(addr);
}

void inc_useful_lines_uc(Addr line_addr) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  if (!FDIP_UC_SIZE)
    return;
  g_fdip->inc_useful_lines_uc(line_addr);
}

void dec_useful_lines_uc(Addr line_addr) {
  if (!FDIP_ENABLE)
    return;
  ASSERT(0, g_fdip->get_bp_id() == 0);
  if (!FDIP_UC_SIZE)
    return;
  g_fdip->dec_useful_lines_uc(line_addr);
}

void assert_fdip_break_reason(Addr line_addr) {
  if (!FDIP_UTILITY_HASH_ENABLE || (FDIP_UTILITY_PREF_POLICY != PREF_CONV_FROM_USEFUL_SET) ||
      (FULL_WARMUP && !warmup_dump_done[g_fdip->get_proc_id()]) || !PERFECT_CONFIDENCE)
    return;
  g_fdip->assert_break_reason(line_addr);
}

void inc_icache_hit(Addr line_addr) {
  g_fdip->inc_icache_hit(line_addr);
}

void add_evict_seq(Addr line_addr) {
  g_fdip->add_evict_seq(line_addr);
}

Op* fdip_get_cur_op() {
  return g_fdip->get_cur_op();
}

Counter fdip_get_last_recover_cycle() {
  return g_fdip->get_last_recover_cycle();
}

/* FDIP_Stat member functions */
void FDIP_Stat::print_cl_info(Icache_Stage* ic_ref) {
  DEBUG(proc_id,
        "icache miss cache lines (UNIQUE_MISSED_LINES) size: %lu, icache hit cache lines (UNIQUE_MISSED_LINES): %lu\n",
        icache_miss.size(), icache_hit.size());
  INC_STAT_EVENT(proc_id, ICACHE_UNIQUE_MISSED_LINES, icache_miss.size());
  INC_STAT_EVENT(proc_id, ICACHE_UNIQUE_HIT_LINES, icache_hit.size());
  multimap<Counter, Addr> icache_miss_sorted = flip_map(icache_miss);
  for (multimap<Counter, Addr>::const_iterator it = icache_miss_sorted.begin(); it != icache_miss_sorted.end(); ++it) {
    DEBUG(proc_id, "[set %u] 0x%llx missed %llu times\n",
          (uns)(it->second >> ic_ref->icache.shift_bits & ic_ref->icache.set_mask), it->second, it->first);
  }
  DEBUG(proc_id, "unique prefetched lines (UNIQUE_PREFETCHED_LINES) size: %lu\n", prefetched_cls.size());
  multimap<Counter, Addr> prefetched_cls_sorted = flip_map(prefetched_cls);
  for (multimap<Counter, Addr>::const_iterator it = prefetched_cls_sorted.begin(); it != prefetched_cls_sorted.end();
       ++it) {
    auto cnt_useful_iter = cnt_useful.find(it->second);
    if (cnt_useful_iter == cnt_useful.end()) {
      DEBUG(proc_id, "Unuseful 0x%llx prefetched %llu times\n", it->second, it->first);
    }
  }

  unordered_map<Addr, int32_t>* cnt_learned_cl = &cnt_useful_signed;
  FILE* fp = fopen("per_line_icache_line_info.csv", "w");
  fprintf(fp, "cl_addr,useful_cnt,unuseful_cnt,prefetch_cnt,new_prefetch_cnt,icache_hit,icache_miss\n");
  for (auto it = cnt_learned_cl->begin(); it != cnt_learned_cl->end(); ++it) {
    auto cnt_useful_iter = cnt_useful.find(it->first);
    auto cnt_unuseful_iter = cnt_unuseful.find(it->first);
    auto cnt_prefetch_iter = prefetched_cls.find(it->first);
    auto cnt_new_prefetch_iter = new_prefetched_cls.find(it->first);
    auto hit_iter = icache_hit.find(it->first);
    auto miss_iter = icache_miss.find(it->first);
    Counter _cnt_useful = (cnt_useful_iter != cnt_useful.end()) ? cnt_useful_iter->second.first : 0;
    Counter _cnt_unuseful = (cnt_unuseful_iter != cnt_unuseful.end()) ? cnt_unuseful_iter->second : 0;
    Counter cnt_prefetch = (cnt_prefetch_iter != prefetched_cls.end()) ? cnt_prefetch_iter->second : 0;
    Counter cnt_new_prefetch = (cnt_new_prefetch_iter != new_prefetched_cls.end()) ? cnt_new_prefetch_iter->second : 0;
    Counter num_hit = (hit_iter != icache_hit.end()) ? hit_iter->second : 0;
    Counter num_miss = (miss_iter != icache_miss.end()) ? miss_iter->second : 0;
    fprintf(fp, "%llx,%llu,%llu,%llu,%llu,%llu,%llu\n", it->first, _cnt_useful, _cnt_unuseful, cnt_prefetch,
            cnt_new_prefetch, num_hit, num_miss);
    ASSERT(proc_id, (cnt_useful_iter != cnt_useful.end()) || (cnt_unuseful_iter != cnt_unuseful.end()));
  }
  fclose(fp);

  fp = fopen("per_line_icache_line_info_after_warmup.csv", "w");
  fprintf(fp, "cl_addr,useful_cnt,unuseful_cnt,prefetch_cnt,new_prefetch_cnt,icache_hit,icache_miss\n");
  for (auto it = cnt_useful_signed.begin(); it != cnt_useful_signed.end(); ++it) {
    auto cnt_useful_iter = cnt_useful_aw.find(it->first);
    auto cnt_unuseful_iter = cnt_unuseful_aw.find(it->first);
    auto cnt_prefetch_iter = prefetched_cls_aw.find(it->first);
    auto cnt_new_prefetch_iter = new_prefetched_cls_aw.find(it->first);
    auto hit_iter = icache_hit_aw.find(it->first);
    auto miss_iter = icache_miss_aw.find(it->first);
    Counter cnt_useful = (cnt_useful_iter != cnt_useful_aw.end()) ? cnt_useful_iter->second.first : 0;
    Counter cnt_unuseful = (cnt_unuseful_iter != cnt_unuseful_aw.end()) ? cnt_unuseful_iter->second : 0;
    Counter cnt_prefetch = (cnt_prefetch_iter != prefetched_cls_aw.end()) ? cnt_prefetch_iter->second : 0;
    Counter cnt_new_prefetch =
        (cnt_new_prefetch_iter != new_prefetched_cls_aw.end()) ? cnt_new_prefetch_iter->second : 0;
    Counter num_hit = (hit_iter != icache_hit_aw.end()) ? hit_iter->second : 0;
    Counter num_miss = (miss_iter != icache_miss_aw.end()) ? miss_iter->second : 0;
    if (cnt_useful != 0 || cnt_unuseful != 0)
      fprintf(fp, "%llx,%llu,%llu,%llu,%llu,%llu,%llu\n", it->first, cnt_useful, cnt_unuseful, cnt_prefetch,
              cnt_new_prefetch, num_hit, num_miss);
  }
  fclose(fp);

  fp = fopen("per_line_useful_seq.csv", "w");
  fprintf(fp, "cl_addr,seq\n");
  for (auto it = usefulness_history.begin(); it != usefulness_history.end(); ++it) {
    fprintf(fp, "%llx", it->first);
    for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
      fprintf(fp, ",%u", *it2);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);

  fp = fopen("per_line_icache_seq.csv", "w");
  fprintf(fp, "cl_addr,seq\n");
  for (auto it = icache_access_history.begin(); it != icache_access_history.end(); ++it) {
    fprintf(fp, "%llx", it->first);
    for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
      fprintf(fp, ",%u", *it2);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);

  fp = fopen("per_line_seq_aw.csv", "w");
  fprintf(fp, "cl_addr,seq\n");
  for (auto it = cacheline_event_history.begin(); it != cacheline_event_history.end(); ++it) {
    fprintf(fp, "%llx", it->first);
    if (it->second.size() == 2) {
      auto it2 = it->second.begin();
      if (it2++->first == 'P' && it2->first == 'u')
        STAT_EVENT(proc_id, FDIP_PREFETCH_EVICT_NO_HIT_ONLY_ONCE);
    }
    for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
      fprintf(fp, ",%c", it2->first);
    }
    fprintf(fp, "\n");
    for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
      fprintf(fp, ",%lld", it2->second);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);

  multimap<Counter, Addr> per_line_delay_sorted = flip_map(per_line_delay_aw);
  fp = fopen("per_line_delay.csv", "w");
  fprintf(fp, "cl_addr,delay\n");
  for (multimap<Counter, Addr>::const_iterator it = per_line_delay_sorted.begin(); it != per_line_delay_sorted.end();
       ++it) {
    fprintf(fp, "%llx,%lld\n", it->second, it->first);
  }
  fclose(fp);
}

void FDIP_Stat::inc_cnt_useful_signed(Addr line_addr) {
  auto it = cnt_useful_signed.find(line_addr);
  if (it == cnt_useful_signed.end())
    cnt_useful_signed.insert(pair<Addr, int64_t>(line_addr, UDP_USEFUL_THRESHOLD + UDP_WEIGHT_USEFUL));
  else if (it->second + UDP_WEIGHT_USEFUL <= UDP_WEIGHT_POSITIVE_SATURATION)
    it->second += UDP_WEIGHT_USEFUL;

  if (FDIP_PRINT_CL_INFO) {
    const uns8 useful_value = g_fdip->get_warmed_up() ? 3 : 1;
    usefulness_history[line_addr].push_back(useful_value);
  }
}

void FDIP_Stat::inc_cnt_unuseful(Addr line_addr) {
  auto unuseful_iter = cnt_unuseful.find(line_addr);
  if (unuseful_iter == cnt_unuseful.end()) {
    STAT_EVENT(proc_id, ICACHE_UNUSEFUL_FETCHES);
    cnt_unuseful.insert(make_pair(std::move(line_addr), 1));
  } else {
    unuseful_iter->second++;
  }

  if (g_fdip->get_warmed_up()) {
    auto it = cnt_unuseful_aw.find(line_addr);
    if (it == cnt_unuseful_aw.end())
      cnt_unuseful_aw.insert(make_pair(std::move(line_addr), 1));
    else
      it->second++;

    if (FDIP_PRINT_CL_INFO)
      cacheline_event_history[line_addr].push_back(make_pair('u', cycle_count));
  } else {
    cacheline_events[line_addr] |= CACHELINE_EVENT_UNUSEFUL;
  }
}

void FDIP_Stat::inc_cnt_useful(Addr line_addr, Flag pref_miss) {
  auto useful_iter = cnt_useful.find(line_addr);
  DEBUG(proc_id, "cnt_useful size %ld\n", cnt_useful.size());
  if (useful_iter == cnt_useful.end()) {
    DEBUG(proc_id, "%llx useful line new insert\n", line_addr);
    STAT_EVENT(proc_id, ICACHE_USEFUL_FETCHES);
    cnt_useful.insert(make_pair(std::move(line_addr), make_pair(1, pref_miss)));
  } else {
    useful_iter->second.first++;
    useful_iter->second.second = pref_miss;
  }
  DEBUG(proc_id, "cnt_useful size after inserted %ld\n", cnt_useful.size());

  if (g_fdip->get_warmed_up()) {
    auto it = cnt_useful_aw.find(line_addr);
    if (it == cnt_useful_aw.end())
      cnt_useful_aw.insert(make_pair(std::move(line_addr), make_pair(1, pref_miss)));
    else {
      it->second.first++;
      it->second.second = pref_miss;
    }

    if (FDIP_PRINT_CL_INFO) {
      cacheline_event_history[line_addr].push_back(make_pair('U', cycle_count));
    }
  } else {
    cacheline_events[line_addr] |= CACHELINE_EVENT_USEFUL;
  }
}

void FDIP_Stat::probe_prefetched_cls(Addr line_addr) {
  auto cl_iter = prefetched_cls_info.find(line_addr);
  if (cl_iter != prefetched_cls_info.end())
    cl_iter->second.first.first = cycle_count;
}

void FDIP_Stat::not_prefetch(Addr line_addr) {
  if (g_fdip->get_warmed_up()) {
    if (FDIP_PRINT_CL_INFO) {
      const Counter onoff_cycle_count = fdip_off_path(proc_id, bp_id) ? -cycle_count : cycle_count;
      cacheline_event_history[line_addr].push_back(make_pair('p', onoff_cycle_count));
    }
  } else {
    cacheline_events[line_addr] |= CACHELINE_EVENT_NOT_PREFETCHED;
  }
}

void FDIP_Stat::inc_icache_miss(Addr line_addr) {
  auto cl_iter = icache_miss.find(line_addr);
  if (cl_iter == icache_miss.end()) {
    STAT_EVENT(proc_id, UNIQUE_MISSED_LINES);
    icache_miss.insert(pair<Addr, Counter>(line_addr, 1));
  } else
    cl_iter->second++;

  if (g_fdip->get_warmed_up()) {
    auto it = icache_miss_aw.find(line_addr);
    if (it == icache_miss_aw.end())
      icache_miss_aw.insert(pair<Addr, Counter>(line_addr, 1));
    else
      it->second++;

    if (FDIP_PRINT_CL_INFO) {
      cacheline_event_history[line_addr].push_back(make_pair('m', cycle_count));
    }

    cur_line_delay = cycle_count;
  } else {
    cacheline_events[line_addr] |= CACHELINE_EVENT_ICACHE_MISS;
  }

  uns icache_val = g_fdip->get_warmed_up() ? 2 : 0;
  const bool first_access = accessed_cachelines.insert(line_addr).second;
  if (FDIP_PRINT_CL_INFO)
    icache_access_history[line_addr].push_back(icache_val);
  if (first_access) {
    if (icache_val == 2) {
      auto it2 = cacheline_events.find(line_addr);
      if (it2 != cacheline_events.end()) {
        STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_SEEN_DURING_WARMUP);
        const uint8_t state = it2->second;
        const bool no_pref = state & CACHELINE_EVENT_NOT_PREFETCHED;
        // Preserve the legacy interpretation: the events produced by
        // inc_cnt_unuseful() and inc_cnt_useful() drive the opposite-named
        // first-miss predicates. Changing it would alter existing FDIP statistics.
        const bool useful = state & CACHELINE_EVENT_UNUSEFUL;
        const bool unuseful = state & CACHELINE_EVENT_USEFUL;
        if (no_pref && !unuseful && !useful)
          STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_NO_PREF_DURING_WARMUP);
        if (!no_pref && unuseful && !useful)
          STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_TRAINED_UNUSEFUL_DURING_WARMUP);
        if (!no_pref && !unuseful && useful)
          STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_TRAINED_USEFUL_DURING_WARMUP);
      } else
        STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_NOT_SEEN_DURING_WARMUP);
    }
  }
}

void FDIP_Stat::inc_prefetched_cls(Addr line_addr, Flag on_path, uns success) {
  auto cl_iter = prefetched_cls.find(line_addr);
  if (cl_iter == prefetched_cls.end()) {
    prefetched_cls.insert(pair<Addr, Counter>(line_addr, 1));
    prefetched_cls_info.insert(
        make_pair(std::move(line_addr), make_pair(make_pair(std::move(cycle_count), on_path), make_pair(0, 0))));
    DEBUG(proc_id, "%llx inserted into prefetched_cls at %llu\n", line_addr, cycle_count);
  } else {
    cl_iter->second++;
    auto cl_info_iter = prefetched_cls_info.find(line_addr);
    ASSERT(proc_id, cl_info_iter != prefetched_cls_info.end());
    cl_info_iter->second.first.first = cycle_count;
    cl_info_iter->second.first.second = on_path;
    DEBUG(proc_id, "%llx updated with cnt %llu in prefetched_cls at cyc %llu\n", line_addr, cl_iter->second,
          cycle_count);
  }

  if (success == Mem_Queue_Req_Result::SUCCESS_NEW) {
    auto cl_new_iter = new_prefetched_cls.find(line_addr);
    if (cl_new_iter == new_prefetched_cls.end())
      new_prefetched_cls.insert(pair<Addr, Counter>(line_addr, 1));
    else
      cl_new_iter->second++;
  }

  if (g_fdip->get_warmed_up()) {
    auto it = prefetched_cls_aw.find(line_addr);
    if (it == prefetched_cls_aw.end())
      prefetched_cls_aw.insert(pair<Addr, Counter>(line_addr, 1));
    else
      it->second++;

    if (success == Mem_Queue_Req_Result::SUCCESS_NEW) {
      it = new_prefetched_cls_aw.find(line_addr);
      if (it == new_prefetched_cls_aw.end())
        new_prefetched_cls_aw.insert(pair<Addr, Counter>(line_addr, 1));
      else
        it->second++;
    }

    if (FDIP_PRINT_CL_INFO) {
      const Counter onoff_cycle_count = fdip_off_path(proc_id, bp_id) ? -cycle_count : cycle_count;
      cacheline_event_history[line_addr].push_back(make_pair('P', onoff_cycle_count));
    }
  } else {
    cacheline_events[line_addr] |= CACHELINE_EVENT_PREFETCHED;
  }
}

void FDIP_Stat::dec_cnt_useful_signed(Addr line_addr) {
  auto it = cnt_useful_signed.find(line_addr);
  if (it == cnt_useful_signed.end())
    cnt_useful_signed.insert(pair<Addr, int64_t>(line_addr, UDP_USEFUL_THRESHOLD - UDP_WEIGHT_UNUSEFUL));
  else
    it->second -= UDP_WEIGHT_UNUSEFUL;

  if (FDIP_PRINT_CL_INFO) {
    const uns8 unuseful_value = g_fdip->get_warmed_up() ? 2 : 0;
    usefulness_history[line_addr].push_back(unuseful_value);
  }
}

void FDIP_Stat::inc_icache_hit(Addr line_addr) {
  auto cl_iter = icache_hit.find(line_addr);
  if (cl_iter == icache_hit.end()) {
    STAT_EVENT(proc_id, UNIQUE_HIT_LINES);
    icache_hit.insert(pair<Addr, Counter>(line_addr, 1));
  } else
    cl_iter->second++;

  if (g_fdip->get_warmed_up()) {
    auto it = icache_hit_aw.find(line_addr);
    if (it == icache_hit_aw.end())
      icache_hit_aw.insert(pair<Addr, Counter>(line_addr, 1));
    else
      it->second++;

    if (FDIP_PRINT_CL_INFO)
      cacheline_event_history[line_addr].push_back(make_pair('h', cycle_count));

    if (cur_line_delay) {
      auto it3 = per_line_delay_aw.find(line_addr);
      if (it3 == per_line_delay_aw.end()) {
        per_line_delay_aw.insert(make_pair(line_addr, cycle_count - cur_line_delay));
      } else {
        it3->second += cycle_count - cur_line_delay;
      }
    }
    cur_line_delay = 0;
  } else {
    cacheline_events[line_addr] |= CACHELINE_EVENT_ICACHE_HIT;
  }

  uns icache_val = g_fdip->get_warmed_up() ? 3 : 1;
  accessed_cachelines.insert(line_addr);
  if (FDIP_PRINT_CL_INFO)
    icache_access_history[line_addr].push_back(icache_val);
}

/* FDIP member functions */
FDIP::~FDIP() {
  if (bp_id)
    return;

  delete fdip_stat;
  delete uftq;
}

void FDIP::init(uns _proc_id, uns _bp_id, Icache_Stage* _ic) {
  proc_id = _proc_id;
  bp_id = _bp_id;
  ic_ref = _ic;
  dfe = decoupled_fe_new_ftq_iter(proc_id, bp_id, &ftq_idx);
  ASSERT(proc_id, dfe);
  if (bp_id) {
    fdip_stat = per_core_fdip[proc_id][0]->get_fdip_stat();
    udp = per_core_fdip[proc_id][0]->get_udp();
    uftq = per_core_fdip[proc_id][0]->get_uftq();
  } else {
    fdip_stat = new FDIP_Stat(proc_id, bp_id);
    if (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER)
      udp = new UDP(_proc_id);
    if (FDIP_ADJUSTABLE_FTQ)
      uftq = new UFTQ(_proc_id);
  }
}

void FDIP::recover() {
  last_line_addr = 0;
  const bool is_early_recovery = bp_l0_enabled() &&
                                 bp_recovery_info->recovery_op->bp_pred_l0.recovery_point == RECOVER_AT_FE &&
                                 bp_recovery_info->recovery_op->bp_pred_main.recovery_point != RECOVER_AT_EXEC &&
                                 bp_recovery_info->recovery_op->bp_pred_main.recovery_point != RECOVER_AT_DECODE;
  DEBUG(proc_id, "[FDIP%u] recover cycle from %llu", bp_id, fdip_stat->last_recover_cycle);
  if (!is_early_recovery) {
    fdip_stat->last_recover_cycle = cycle_count;
    DEBUG(proc_id, " to %llu\n", fdip_stat->last_recover_cycle);
  } else {
    DEBUG(proc_id, " unchanged (early recovery)\n");
  }

  if (!bp_id && FDIP_ADJUSTABLE_FTQ)
    uftq->set_ftq_ft_num();
}

void FDIP::update() {
  if (FULL_WARMUP && warmup_dump_done[proc_id] && !warmed_up)
    warmed_up = TRUE;

  if (!bp_id && (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER))
    udp->cyc_reset();

  uint32_t ops_per_cycle = 0;
  uint32_t ftq_entry_per_cycle = 0;
  FDIP_Break break_reason = BR_REACH_FTQ_END;
  bool end_of_block;
  per_cyc_ipref = 0;
  Op* op = NULL;
  ASSERT(proc_id, dfe);
  for (op = decoupled_fe_ftq_iter_get(dfe, ftq_idx, &end_of_block); op != NULL;
       op = decoupled_fe_ftq_iter_get_next(dfe, ftq_idx, &end_of_block), ops_per_cycle++) {
    cur_op = op;
    Flag emit_new_prefetch = FALSE;
    if (ftq_entry_per_cycle >= IPRF_MAX_FTQ_ENTRY_CYC && ops_per_cycle >= IC_ISSUE_WIDTH) {
      DEBUG(proc_id, "[FDIP%u] Break due to max FTQ entries per cycle\n", bp_id);
      break_reason = BR_MAX_FTQ_ENTRY_CYC;
      break;
    }
    if (!bp_id && (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER)) {
      udp->clear_old_seniority_ftq();
      udp->set_last_bbl_start_addr(op->inst->addr);
    }

    uint64_t pc_addr = op->inst->addr;
    Addr line_addr = op->inst->addr & ~0x3F;
    uint64_t ft_id = (op->parent_FT) ? op->parent_FT->get_ft_info().dynamic_info.FT_id : 0;
    DEBUG(proc_id,
          "[FDIP%u] ft_id: %llu, op_num: %llu, op->inst->addr: %llx, line_addr: %llx, last_line_addr: %llx, "
          "off-path: %d\n",
          bp_id, (unsigned long long)ft_id, op->op_num, op->inst->addr, line_addr, last_line_addr, op->off_path);
    UNUSED(ft_id);
    if (line_addr != last_line_addr) {
      STAT_EVENT(proc_id, FDIP_ATTEMPTED_PREF_ONPATH0 + FDIP_PREF_STAT_COUNT * bp_id + op->off_path);
      DEBUG(proc_id, "[FDIP%u] fdip off path: %d, conf off path: %d\n", bp_id, op->off_path, op->conf_off_path);
      if (!bp_id)
        log_stats_path_conf_per_pref_candidate();
      emit_new_prefetch = determine_usefulness(line_addr, op, op->bp_pred_info->pred_global_hist);

      Flag demand_hit_prefetch = FALSE;
      Flag demand_hit_writeback = FALSE;
      Mem_Queue_Entry* queue_entry = NULL;
      Flag ramulator_match = FALSE;
      Addr dummy_addr = 0;
      bool line = false;
      Mem_Req* mem_req = NULL;
      if (emit_new_prefetch) {
        line = (Inst_Info**)cache_access(&(ic_ref->icache), pc_addr, &line_addr, TRUE);
        // icache_line_info cache should be accessed same times with icache for a consistant line information
        if (WP_COLLECT_STATS) {
          bool line_info = (Icache_Data*)cache_access(&(ic_ref->icache_line_info), pc_addr, &dummy_addr, TRUE);
          UNUSED(line_info);
        }
        bool mlc_line = (Inst_Info**)cache_access(&mem->uncores[proc_id].mlc->cache, pc_addr, &dummy_addr, FALSE);
        bool l1_line = (Inst_Info**)cache_access(&mem->uncores[proc_id].l1->cache, pc_addr, &dummy_addr, FALSE);
        UNUSED(dummy_addr);
        uns pref_from = line ? 0 : (mlc_line ? 1 : (l1_line ? 2 : 3));
        STAT_EVENT(proc_id, FDIP_PREFETCH_HIT_ICACHE0 + 4 * bp_id + pref_from);
        mem_req = mem_search_reqbuf_wrapper(
            proc_id, line_addr, MRT_FDIPPRFON, ICACHE_LINE_SIZE, &demand_hit_prefetch, &demand_hit_writeback,
            QUEUE_MLC | QUEUE_L1 | QUEUE_MEM | QUEUE_L1FILL | QUEUE_MLC_FILL, &queue_entry, &ramulator_match);

        if (!mem_req) {
          mem_req = mem_search_reqbuf_wrapper(
              proc_id, line_addr, MRT_FDIPPRFOFF, ICACHE_LINE_SIZE, &demand_hit_prefetch, &demand_hit_writeback,
              QUEUE_MLC | QUEUE_L1 | QUEUE_MEM | QUEUE_L1FILL | QUEUE_MLC_FILL, &queue_entry, &ramulator_match);
        }

        if (line) {
          DEBUG(proc_id, "[FDIP%u] probe hit for %llx\n", bp_id, line_addr);
          fdip_stat->probe_prefetched_cls(line_addr);
          STAT_EVENT(proc_id, FDIP_PREF_ICACHE_PROBE_HIT_ONPATH0 + FDIP_PREF_STAT_COUNT * bp_id + op->off_path);
        }
      }

      Mem_Req_Type mem_type = is_conf_off_path() ? MRT_FDIPPRFOFF : MRT_FDIPPRFON;
      if (!emit_new_prefetch && !line && !mem_req)  // TODO: whether to insert the candidate for the subsidiary
        insert_pref_candidate_to_seniority_ftq(line_addr, op->bp_pred_info->pred_global_hist);
      if (!bp_id && (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER))
        INC_STAT_EVENT(proc_id, FDIP_SENIORITY_FTQ_ACCUMULATED, udp->seniority_ftq.size());
      if (emit_new_prefetch)
        STAT_EVENT(proc_id, FDIP_DECIDE_PREF_ONPATH0 + FDIP_PREF_STAT_COUNT * bp_id + op->off_path);

      if (!line && emit_new_prefetch) {  // create a mem request only if line doesn't exist. If the corresponding
                                         // mem_req exists, it will merge.
        uns success = Mem_Queue_Req_Result::FAILED;
        if (FDIP_PREF_NO_LATENCY) {
          Mem_Req req;
          req.off_path = op ? op->off_path : FALSE;
          req.conf_off_path = op ? op->conf_off_path : FALSE;
          req.off_path_confirmed = FALSE;
          req.type = mem_type;
          mem_req_set_types(&req, mem_type);
          req.proc_id = proc_id;
          req.addr = line_addr;
          req.oldest_op_unique_num = (Counter)0;
          req.oldest_op_op_num = (Counter)0;
          req.oldest_op_addr = (Addr)0;
          req.dirty_l0 = op && op->uop->mem_type == MEM_ST && !op->off_path;
          req.fdip_pref_off_path = op->off_path;
          req.demand_icache_emitted_cycle = 0;
          req.fdip_emitted_cycle = cycle_count;
          req.ghist = g_bp_data->global_hist;
          if (icache_fill_line(&req)) {
            STAT_EVENT(proc_id, FDIP_NEW_PREFETCHES_ONPATH0 + FDIP_PREF_STAT_COUNT * bp_id + op->off_path);
            if (FDIP_BLOOM_FILTER)
              udp->bloom_inc_new_prefs();  // TODO: still update for the secondary?
          } else
            ASSERT(proc_id, false);
          success = Mem_Queue_Req_Result::SUCCESS_NEW;
        } else {
          success =
              new_mem_req(mem_type, proc_id, line_addr, ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count, 0);
          // ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count++, 0); // bug?
          if (success == Mem_Queue_Req_Result::SUCCESS_NEW) {
            STAT_EVENT(proc_id, FDIP_NEW_PREFETCHES_ONPATH0 + FDIP_PREF_STAT_COUNT * bp_id + op->off_path);
            DEBUG(proc_id, "[FDIP%u] Success to emit a new prefetch for %llx\n", bp_id, line_addr);
            per_cyc_ipref++;
            if (!bp_id && FDIP_BLOOM_FILTER)  // TODO: still update for the secondary?
              udp->bloom_inc_new_prefs();
          } else if (success == Mem_Queue_Req_Result::SUCCESS_MERGED) {
            STAT_EVENT(proc_id, FDIP_PREF_MSHR_PROBE_HIT_ONPATH0 + FDIP_PREF_STAT_COUNT * bp_id + op->off_path);
            DEBUG(proc_id, "[FDIP%u] Success to merge a prefetch for %llx\n", bp_id, line_addr);
          } else if (success == Mem_Queue_Req_Result::FAILED) {
            // The target queue is full; the request buffer cannot run dry.
            STAT_EVENT(proc_id, FDIP_PREF_FAILED_QUEUE_FULL);
            STAT_EVENT(proc_id, FDIP_PREF_FAILED_ONPATH0 + FDIP_PREF_STAT_COUNT * bp_id + op->off_path);
            DEBUG(proc_id, "[FDIP%u] Failed to emit a prefetch for %llx\n", bp_id, line_addr);
            if (FDIP_FREEZE_AT_MEM_BUF_LIMIT && decoupled_fe_ftq_iter_ft_offset(dfe, ftq_idx) > 1) {
              break_reason = BR_FULL_MEM_REQ_BUF;
              break;
            }
          }
        }
        if (!bp_id)
          log_stats_path_conf_emitted();
        inc_prefetched_cls(line_addr, success, op->conf_off_path);
      } else {
        fdip_stat->not_prefetch(line_addr);
      }
      last_line_addr = line_addr;
    }
    if (end_of_block) {
      ftq_entry_per_cycle++;
      DEBUG(proc_id, "[FDIP%u] End of block - ftq_entry_per_cycle: %d\n", bp_id, ftq_entry_per_cycle);
    }
  }
  if (!op) {
    if (!decoupled_fe_ftq_iter_ft_offset(dfe, ftq_idx)) {
      DEBUG(proc_id, "[FDIP%u] Break due to FTQ Empty\n", bp_id);
      break_reason = BR_FTQ_EMPTY;
    } else {
      break_reason = BR_REACH_FTQ_END;
      DEBUG(proc_id, "[FDIP%u] Break due to reaching FTQ end\n", bp_id);
    }
  }

  STAT_EVENT(proc_id, FDIP_BREAK_REACH_FTQ_END0 + BR_BREAK_END_INVALID * bp_id + break_reason);
  DEBUG(proc_id, "[FDIP%u] FTQ size : %lu, FDIP prefetch FT offset : %lu\n", bp_id, decoupled_fe_ftq_num_fts(dfe),
        decoupled_fe_ftq_iter_ft_offset(dfe, ftq_idx));
  if (!bp_id && FDIP_ADJUSTABLE_FTQ)
    uftq->cyc_reset();

  fdip_stat->ftq_occupancy_ops += decoupled_fe_ftq_iter_offset(dfe, ftq_idx);
  INC_STAT_EVENT(proc_id, FDIP_FTQ_OCCUPANCY_OPS_ACCUMULATED0 + 2 * bp_id, decoupled_fe_ftq_iter_offset(dfe, ftq_idx));
  if (break_reason == BR_REACH_FTQ_END) {
    fdip_stat->ftq_occupancy_blocks += decoupled_fe_ftq_iter_ft_offset(dfe, ftq_idx);
    INC_STAT_EVENT(proc_id, FDIP_FTQ_OCCUPANCY_BLOCKS_ACCUMULATED0 + 2 * bp_id, decoupled_fe_ftq_num_fts(dfe));
  }
  fdip_stat->last_break_reason = break_reason;
}

Flag FDIP::is_off_path() {
  ASSERT(proc_id, cur_op);
  return cur_op->off_path;
}

Flag FDIP::is_conf_off_path() {
  if (CONFIDENCE_ENABLE)
    return cur_op->conf_off_path;
  return FALSE;
}

void FDIP::inc_cnt_unuseful(Addr line_addr) {
  if (FDIP_BLOOM_FILTER)
    udp->bloom_inc_cnt_unuseful();
  fdip_stat->inc_cnt_unuseful(line_addr);
}

void FDIP::inc_prefetched_cls(Addr line_addr, uns success, Flag conf_off_path) {
  if (success == Mem_Queue_Req_Result::FAILED)
    return;
  if (bp_id)
    ASSERT(proc_id, conf_off_path);
  Flag on_path = FALSE;
  if (CONFIDENCE_ENABLE && !conf_off_path)
    on_path = TRUE;
  if (!CONFIDENCE_ENABLE && !cur_op->off_path)
    on_path = TRUE;

  fdip_stat->inc_prefetched_cls(line_addr, on_path, success);
}

void FDIP::inc_off_fetched_cls(Addr line_addr) {
  auto cl_iter = fdip_stat->off_fetched_cls.find(line_addr);
  if (cl_iter == fdip_stat->off_fetched_cls.end()) {
    fdip_stat->off_fetched_cls.insert(pair<Addr, Counter>(line_addr, cycle_count));
    DEBUG(proc_id, "%llx inserted into off_fetched_cls at %llu\n", line_addr, cycle_count);
  } else {
    cl_iter->second = cycle_count;
    DEBUG(proc_id, "%llx in off_fetched_cls updated at %llu\n", line_addr, cycle_count);
  }
}

void FDIP::evict_prefetched_cls(Addr line_addr, Flag by_fdip) {
  DEBUG(proc_id, "%llx evicted by %s\n", line_addr, by_fdip ? "FDIP" : "IFETCH");
  auto cl_iter = fdip_stat->prefetched_cls_info.find(line_addr);
  if (cl_iter != fdip_stat->prefetched_cls_info.end()) {
    if (by_fdip) {
      cl_iter->second.second.first = 0;
      cl_iter->second.second.second = cycle_count;
    } else {
      cl_iter->second.second.first = cycle_count;
      cl_iter->second.second.second = 0;
    }
  }
}

uns FDIP::get_miss_reason(Addr line_addr) {
  auto cl_iter = fdip_stat->prefetched_cls_info.find(line_addr);
  if (cl_iter == fdip_stat->prefetched_cls_info.end()) {
    auto tmp_iter = fdip_stat->prefetched_cls.find(line_addr);
    DEBUG(proc_id, "%llx misses due to 'not prefetched ever'\n", line_addr);
    ASSERT(proc_id, tmp_iter == fdip_stat->prefetched_cls.end());
    return Imiss_Reason::IMISS_NOT_PREFETCHED;
  }
  if (cl_iter->second.first.first < fdip_stat->last_recover_cycle) {
    DEBUG(proc_id, "%llx misses due to 'not prefetched after last recover cycle'\n", line_addr);
    return Imiss_Reason::IMISS_NOT_PREFETCHED;
  }

  if (cl_iter->second.first.first >= fdip_stat->last_recover_cycle) {
    if (cl_iter->second.second.first > cl_iter->second.first.first) {
      DEBUG(proc_id, "%llx misses due to 'prefetched but evicted by a demand load'\n", line_addr);
      return Imiss_Reason::IMISS_TOO_EARLY_EVICTED_BY_IFETCH;
    } else if (cl_iter->second.second.second > cl_iter->second.first.first) {
      DEBUG(proc_id, "%llx misses due to 'prefetched but evicted by FDIP'\n", line_addr);
      return Imiss_Reason::IMISS_TOO_EARLY_EVICTED_BY_FDIP;
    }
  }

  if (cl_iter->second.first.second) {
    DEBUG(proc_id, "%llx misses due to 'MSHR hit prefetched on path'\n", line_addr);
    return Imiss_Reason::IMISS_MSHR_HIT_PREFETCHED_ONPATH;
  }

  DEBUG(proc_id, "%llx misses due to 'MSHR hit prefetched off path'\n", line_addr);
  return Imiss_Reason::IMISS_MSHR_HIT_PREFETCHED_OFFPATH;
}

uns FDIP::get_last_miss_reason() {
  DEBUG(proc_id, "get last miss reason %u\n", fdip_stat->last_imiss_reason);
  return fdip_stat->last_imiss_reason;
}

uint64_t FDIP::get_ftq_occupancy_ops() {
  return (uint64_t)(fdip_stat->ftq_occupancy_ops) / cycle_count;
}

uint64_t FDIP::get_ftq_occupancy() {
  return (uint64_t)(fdip_stat->ftq_occupancy_blocks) / cycle_count;
}

void FDIP::determine_usefulness_by_inf_hash(Addr line_addr, Flag* emit_new_prefetch, Op* op, uint64_t ghist) {
  uint64_t hashed_line_addr = line_addr;
  if (FDIP_GHIST_HASHING)
    hashed_line_addr = fdip_hash_addr_ghist(line_addr, ghist);

  if (CONFIDENCE_ENABLE && !op->conf_off_path) {
    ASSERT(proc_id, !bp_id);
    DEBUG(proc_id, "emit_new_prefetch confidence on-path fdip_off_path: %d\n", op->off_path);
    *emit_new_prefetch = TRUE;
  } else {
    switch (FDIP_UTILITY_PREF_POLICY) {
      case Utility_Pref_Policy::PREF_CONV_FROM_USEFUL_SET: {
        unordered_map<Addr, pair<Counter, Flag>>* cnt_useful = &(fdip_stat->cnt_useful);
        auto iter = cnt_useful->find(hashed_line_addr);
        if (iter == cnt_useful->end()) {
          *emit_new_prefetch = FALSE;
        } else {
          *emit_new_prefetch = TRUE;
        }
        break;
      }
      case Utility_Pref_Policy::PREF_OPT_FROM_UNUSEFUL_SET: {
        unordered_map<Addr, Counter>* cnt_unuseful = &(fdip_stat->cnt_unuseful);
        auto iter = cnt_unuseful->find(hashed_line_addr);
        if (iter == cnt_unuseful->end())
          *emit_new_prefetch = TRUE;
        else {
          *emit_new_prefetch = FALSE;
        }
        break;
      }
      case Utility_Pref_Policy::PREF_CONV_FROM_THROTTLE_CNT: {
        unordered_map<Addr, int32_t>* cnt = &(fdip_stat->cnt_useful_signed);
        auto iter = cnt->find(hashed_line_addr);
        if (iter != cnt->end() && iter->second > UDP_USEFUL_THRESHOLD)
          *emit_new_prefetch = TRUE;
        else {
          *emit_new_prefetch = FALSE;
        }
        break;
      }
      case Utility_Pref_Policy::PREF_OPT_FROM_THROTTLE_CNT: {
        unordered_map<Addr, int32_t>* cnt = &(fdip_stat->cnt_useful_signed);
        auto iter = cnt->find(hashed_line_addr);
        if (iter != cnt->end() && iter->second < UDP_USEFUL_THRESHOLD) {
          *emit_new_prefetch = FALSE;
        } else
          *emit_new_prefetch = TRUE;
        break;
      }
    }
  }
  if (*emit_new_prefetch) {
    DEBUG(proc_id, "emit a new prefetch for cl 0x%llx\n", line_addr);
  } else {
    DEBUG(proc_id, "do not emit a new prefetch for cl 0x%llx\n", line_addr);
    udp->set_last_cl_unuseful(line_addr);
  }
}

void FDIP::determine_usefulness_by_utility_cache(Addr line_addr, Flag* emit_new_prefetch, Op* op, uint64_t ghist) {
  Addr uc_line_addr = 0;
  uint64_t hashed_line_addr = line_addr;
  if (FDIP_GHIST_HASHING)
    hashed_line_addr = fdip_hash_addr_ghist(line_addr, ghist);

  if (CONFIDENCE_ENABLE && !op->conf_off_path) {
    ASSERT(proc_id, !bp_id);
    DEBUG(proc_id, "emit_new_prefetch confidence on-path, fdip_off_path: %d\n", op->off_path);
    *emit_new_prefetch = TRUE;
  } else {
    switch (FDIP_UTILITY_PREF_POLICY) {
      case Utility_Pref_Policy::PREF_CONV_FROM_USEFUL_SET: {
        void* useful = (void*)cache_access(udp->get_fdip_uc(), hashed_line_addr, &uc_line_addr, TRUE);
        if (useful) {
          STAT_EVENT(proc_id, FDIP_UC_HIT);
          *emit_new_prefetch = TRUE;
        } else {
          STAT_EVENT(proc_id, FDIP_UC_MISS);
          *emit_new_prefetch = FALSE;
        }
        break;
      }
      case Utility_Pref_Policy::PREF_OPT_FROM_UNUSEFUL_SET: {
        void* unuseful = (void*)cache_access(udp->get_fdip_uc_unuseful(), hashed_line_addr, &uc_line_addr, TRUE);
        if (unuseful) {
          STAT_EVENT(proc_id, FDIP_UC_HIT);
          *emit_new_prefetch = FALSE;
        } else {
          STAT_EVENT(proc_id, FDIP_UC_MISS);
          *emit_new_prefetch = TRUE;
        }
        break;
      }
      case Utility_Pref_Policy::PREF_CONV_FROM_THROTTLE_CNT: {
        int32_t* useful = (int32_t*)cache_access(udp->get_fdip_uc_signed(), hashed_line_addr, &uc_line_addr, TRUE);
        if (useful) {
          STAT_EVENT(proc_id, FDIP_UC_HIT);
          if (*useful > UDP_USEFUL_THRESHOLD)
            *emit_new_prefetch = TRUE;
          else
            *emit_new_prefetch = FALSE;
        } else {
          STAT_EVENT(proc_id, FDIP_UC_MISS);
          *emit_new_prefetch = FALSE;
        }
        break;
      }
      case Utility_Pref_Policy::PREF_OPT_FROM_THROTTLE_CNT: {
        int32_t* useful = (int32_t*)cache_access(udp->get_fdip_uc_signed(), hashed_line_addr, &uc_line_addr, TRUE);
        if (useful) {
          STAT_EVENT(proc_id, FDIP_UC_HIT);
          if (*useful < UDP_USEFUL_THRESHOLD) {
            *emit_new_prefetch = FALSE;
          } else
            *emit_new_prefetch = TRUE;
        } else {
          STAT_EVENT(proc_id, FDIP_UC_MISS);
          *emit_new_prefetch = TRUE;
        }
        break;
      }
    }
  }
  if (*emit_new_prefetch) {
    DEBUG(proc_id, "emit a new prefetch for cl 0x%llx\n", line_addr);
  } else {
    DEBUG(proc_id, "do not emit a new prefetch for cl 0x%llx\n", line_addr);
    udp->set_last_cl_unuseful(line_addr);
  }
}

void FDIP::determine_usefulness_by_bloom_filter(Addr line_addr, Flag* emit_new_prefetch, Op* op, uint64_t ghist) {
  uint64_t hashed_line_addr = line_addr;
  if (FDIP_GHIST_HASHING)
    hashed_line_addr = fdip_hash_addr_ghist(line_addr, ghist);

  if (CONFIDENCE_ENABLE && !op->conf_off_path) {
    ASSERT(proc_id, !bp_id);
    DEBUG(proc_id, "emit_new_prefetch confidence on-path fdip_off_path: %d\n", op->off_path);
    *emit_new_prefetch = TRUE;
  } else {
    void* useful = udp->bloom_lookup(hashed_line_addr);
    if (useful) {
      STAT_EVENT(proc_id, FDIP_BLOOM_HIT);
      *emit_new_prefetch = TRUE;
      DEBUG(proc_id, "bloom : emit a new prefetch for cl 0x%llx off_path: %u", line_addr, cur_op->off_path ? 1 : 0);
    } else {
      STAT_EVENT(proc_id, FDIP_BLOOM_MISS);
      DEBUG(proc_id, "bloom : do not emit a new prefetch for cl 0x%llx", line_addr);
      *emit_new_prefetch = FALSE;
      udp->set_last_cl_unuseful(line_addr);
    }
  }
}

Flag FDIP::determine_usefulness(Addr line_addr, Op* op, uint64_t ghist) {
  if (!FDIP_UTILITY_HASH_ENABLE && !FDIP_UC_SIZE && !FDIP_BLOOM_FILTER && !FDIP_PERFECT_PREFETCH)
    return TRUE;

  Flag emit_new_prefetch = FALSE;
  if (FDIP_PERFECT_PREFETCH) {
    if (!cur_op->off_path)
      emit_new_prefetch = TRUE;
    else {
      emit_new_prefetch = buf_map_find(proc_id, line_addr);
      if (emit_new_prefetch)
        STAT_EVENT(proc_id, FDIP_MEM_BUF_FOUND0 + 2 * bp_id);
      else
        STAT_EVENT(proc_id, FDIP_MEM_BUF_MISS0 + 2 * bp_id);
    }
  } else if (udp->get_last_cl_unuseful() != line_addr) {
    if (FDIP_UTILITY_HASH_ENABLE)
      determine_usefulness_by_inf_hash(line_addr, &emit_new_prefetch, op, ghist);
    else if (FDIP_UC_SIZE)
      determine_usefulness_by_utility_cache(line_addr, &emit_new_prefetch, op, ghist);
    else if (FDIP_BLOOM_FILTER)
      determine_usefulness_by_bloom_filter(line_addr, &emit_new_prefetch, op, ghist);
  }
  return emit_new_prefetch;
}

void FDIP::update_unuseful_lines_uc(Addr line_addr) {
  Cache* fdip_uc_unuseful = udp->get_fdip_uc_unuseful();
  Addr uc_line_addr = 0;
  Addr repl_uc_line_addr = 0;
  void* cnt = (void*)cache_access(fdip_uc_unuseful, line_addr, &uc_line_addr, TRUE);
  if (!cnt)
    cache_insert_replpos(fdip_uc_unuseful, proc_id, line_addr, &uc_line_addr, &repl_uc_line_addr,
                         (Cache_Insert_Repl)FDIP_UC_INSERT_REPLPOL, FALSE);
  UNUSED(uc_line_addr);
  if (repl_uc_line_addr)
    STAT_EVENT(proc_id, FDIP_UC_REPLACEMENT);
}

void FDIP::update_useful_lines_uc(Addr line_addr) {
  Cache* fdip_uc = udp->get_fdip_uc();
  Addr uc_line_addr = 0;
  Addr repl_uc_line_addr = 0;
  void* cnt = (void*)cache_access(fdip_uc, line_addr, &uc_line_addr, TRUE);
  if (!cnt)
    cache_insert_replpos(fdip_uc, proc_id, line_addr, &uc_line_addr, &repl_uc_line_addr,
                         (Cache_Insert_Repl)FDIP_UC_INSERT_REPLPOL, FALSE);
  UNUSED(uc_line_addr);
  if (repl_uc_line_addr)
    STAT_EVENT(proc_id, FDIP_UC_REPLACEMENT);
}

void FDIP::update_useful_lines_bloom_filter(Addr line_addr) {
  void* cnt = udp->bloom_lookup(line_addr);
  if (!cnt)
    udp->detect_stream(line_addr);
}

void FDIP::inc_utility_info(Flag useful) {
  if (useful)
    uftq->inc_useful_prefetches();
  else
    uftq->inc_unuseful_prefetches();
}

void FDIP::inc_timeliness_info(Flag mshr_hit) {
  if (mshr_hit)
    uftq->inc_mshr_prefetch_hits();
  else
    uftq->inc_icache_prefetch_hits();
}

Flag FDIP::search_pref_candidate(Addr addr) {
  auto seniority_ftq = &(udp->seniority_ftq);
  DEBUG(proc_id, "Search a pref candidate for %llx. seniority_ftq.size(): %ld\n", addr, seniority_ftq->size());
  for (auto it = seniority_ftq->begin(); it != seniority_ftq->end(); ++it) {
    if (get<0>(*it) == addr && (FDIP_UTILITY_ONLY_TRAIN_OFF_PATH ? get<2>(*it) == FALSE : TRUE)) {
      DEBUG(proc_id, "Hit seniority-FTQ for addr: %llx cyc: %lld, on-path: %u\n", get<0>(*it), get<1>(*it),
            get<2>(*it));
      STAT_EVENT(proc_id, FDIP_SENIORITY_FTQ_HIT);
      return TRUE;
    }
  }
  STAT_EVENT(proc_id, FDIP_SENIORITY_FTQ_MISS);
  return FALSE;
}

void FDIP::insert_pref_candidate_to_seniority_ftq(Addr line_addr, uint64_t ghist) {
  if (!FDIP_UTILITY_HASH_ENABLE && !FDIP_UC_SIZE && !FDIP_BLOOM_FILTER)
    return;
  uint64_t hashed_line_addr = line_addr;
  if (FDIP_GHIST_HASHING)
    hashed_line_addr = fdip_hash_addr_ghist(line_addr, ghist);
  udp->seniority_ftq.push_back(make_tuple(hashed_line_addr, cycle_count, !is_conf_off_path()));
  DEBUG(proc_id, "Insert %llx (hashed %lx) to seniority FTQ at cyc %llu seniority_ftq.size() : %ld\n", line_addr,
        hashed_line_addr, cycle_count, udp->seniority_ftq.size());
}

void FDIP::inc_useful_lines_uc(Addr line_addr) {
  Addr uc_line_addr = 0;
  Addr repl_uc_line_addr = 0;
  int32_t* cnt = (int32_t*)cache_access(udp->get_fdip_uc_signed(), line_addr, &uc_line_addr, TRUE);
  if (!cnt) {
    cnt = (int32_t*)cache_insert_replpos(udp->get_fdip_uc_signed(), proc_id, line_addr, &uc_line_addr,
                                         &repl_uc_line_addr, (Cache_Insert_Repl)FDIP_UC_INSERT_REPLPOL, FALSE);
    *cnt = UDP_USEFUL_THRESHOLD + UDP_WEIGHT_USEFUL;
    DEBUG(proc_id, "Insert uc cnt with value %d\n", *cnt);
  } else if (*cnt + UDP_WEIGHT_USEFUL <= UDP_WEIGHT_POSITIVE_SATURATION) {
    DEBUG(proc_id, "Increment uc cnt from %d to ", *cnt);
    *cnt += UDP_WEIGHT_USEFUL;
    DEBUG(proc_id, "%d\n", *cnt);
  }
  UNUSED(uc_line_addr);
  if (repl_uc_line_addr)
    STAT_EVENT(proc_id, FDIP_UC_REPLACEMENT);
}

void FDIP::dec_useful_lines_uc(Addr line_addr) {
  Addr uc_line_addr = 0;
  Addr repl_uc_line_addr = 0;
  int32_t* cnt = (int32_t*)cache_access(udp->get_fdip_uc_signed(), line_addr, &uc_line_addr, TRUE);
  if (!cnt) {
    cnt = (int32_t*)cache_insert_replpos(udp->get_fdip_uc_signed(), proc_id, line_addr, &uc_line_addr,
                                         &repl_uc_line_addr, (Cache_Insert_Repl)FDIP_UC_INSERT_REPLPOL, FALSE);
    *cnt = UDP_USEFUL_THRESHOLD - UDP_WEIGHT_UNUSEFUL;
    DEBUG(proc_id, "Insert uc cnt with value %d\n", *cnt);
  } else {
    DEBUG(proc_id, "Decrement uc cnt from %d to ", *cnt);
    *cnt -= UDP_WEIGHT_UNUSEFUL;
    DEBUG(proc_id, "%d\n", *cnt);
  }
  UNUSED(uc_line_addr);
  if (repl_uc_line_addr)
    STAT_EVENT(proc_id, FDIP_UC_REPLACEMENT);
}

void FDIP::assert_break_reason(Addr line_addr) {
  auto useful_iter = fdip_stat->cnt_useful.find(line_addr);
  if (useful_iter != fdip_stat->cnt_useful.end() && !useful_iter->second.second) {  // learned from a seniority-FTQ hit
    ASSERT(proc_id, fdip_stat->last_break_reason == BR_FULL_MEM_REQ_BUF);
  }
}

void FDIP::add_evict_seq(Addr line_addr) {
  if (warmed_up) {
    if (FDIP_PRINT_CL_INFO)
      fdip_stat->cacheline_event_history[line_addr].push_back(make_pair('e', cycle_count));
  } else {
    fdip_stat->cacheline_events[line_addr] |= FDIP_Stat::CACHELINE_EVENT_EVICTED;
  }
}

void FDIP::log_stats_path_conf_per_pref_candidate() {
  if (!CONFIDENCE_ENABLE)
    return;

  if (!is_conf_off_path()) {
    if (is_off_path()) {
      STAT_EVENT(proc_id, FDIP_OFF_CONF_ON_PREF_CANDIDATES);
      STAT_EVENT(proc_id, FDIP_OFF_CONF_ON_NOT_IDENTIFIED_PREF_CANDIDATES + decoupled_fe_get_off_path_reason());
    } else
      STAT_EVENT(proc_id, FDIP_ON_CONF_ON_PREF_CANDIDATES);
  } else {
    if (is_off_path())
      STAT_EVENT(proc_id, FDIP_OFF_CONF_OFF_PREF_CANDIDATES);
    else {
      STAT_EVENT(proc_id, FDIP_ON_CONF_OFF_PREF_CANDIDATES);
      STAT_EVENT(proc_id,
                 FDIP_ON_CONF_OFF_IBTB_MISS_BP_TAKEN_PREF_CANDIDATES + decoupled_fe_get_conf_off_path_reason());
    }
  }
}

void FDIP::log_stats_path_conf_emitted() {
  if (!CONFIDENCE_ENABLE)
    return;

  // conf on
  if (!is_conf_off_path()) {
    // actually off
    if (is_off_path()) {
      STAT_EVENT(proc_id, FDIP_OFF_CONF_ON_EMITTED);
      STAT_EVENT(proc_id, FDIP_OFF_CONF_ON_NOT_IDENTIFIED_EMITTED + decoupled_fe_get_off_path_reason());
    } else {
      STAT_EVENT(proc_id, FDIP_ON_CONF_ON_EMITTED);
    }
  } else {
    if (is_off_path()) {
      STAT_EVENT(proc_id, FDIP_OFF_CONF_OFF_EMITTED);
    } else {
      STAT_EVENT(proc_id, FDIP_ON_CONF_OFF_EMITTED);
      STAT_EVENT(proc_id, FDIP_ON_CONF_OFF_IBTB_MISS_BP_TAKEN_EMITTED + decoupled_fe_get_conf_off_path_reason());
    }
  }
}

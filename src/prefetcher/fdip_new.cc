#include "decoupled_frontend.h"
#include "prefetcher/fdip_new.h"
#include "libs/bloom_filter.hpp"
#include "sim.h"
#include "frontend/pt_memtrace/memtrace_fe.h"

extern "C" {
#include "op.h"
#include "prefetcher/pref.param.h"
#include "prefetcher/eip.h"
#include "memory/memory.h"
#include "memory/memory.param.h"
}

#include <iostream>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <deque>
#include <tuple>
#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_FDIP, ##args)

decoupled_fe_iter* iter;
extern const int MAX_FTQ_ENTRY_CYC = 2;
int fdip_proc_id;
Icache_Stage *ic_ref;
std::vector<Op*> per_core_cur_op;
std::vector<decoupled_fe_iter*> per_core_ftq_iter;
std::vector<Addr> per_core_last_line_addr;
std::vector<Flag> per_core_warmed_up;
int per_cyc_ipref = 0;

typedef enum FDIP_BREAK_enum {
  BR_REACH_FTQ_END,
  BR_FTQ_EMPTY,
  BR_MAX_FTQ_ENTRY_CYC,
  BR_FULL_MEM_REQ_BUF,
} FDIP_Break;

typedef enum UTILITY_PREF_POLICY_enum {
  PREF_CONV_FROM_USEFUL_SET,
  PREF_OPT_FROM_UNUSEFUL_SET,
  PREF_CONV_FROM_THROTTLE_CNT,
  PREF_OPT_FROM_THROTTLE_CNT,
  PREF_POL_END, // add a new policy above this line
} Utility_Pref_Policy;

/* Seniority-FTQ */
// <Cl address, cycle count, on/off-path>
std::vector<std::deque<std::tuple<uns64, Counter, Flag>>> per_core_seniority_ftq;

/* global variables for dynamic FTQ adjustment based on utility/timeliness study */
std::vector<Utility_Timeliness_Info> per_core_utility_timeliness_info;

/* global variables for BTB miss-based BP confidence */
std::vector<Counter> per_core_cnt_btb_miss;
std::vector<double> per_core_btb_miss_rate;

/* global variables for utility study and stats */
// for icache miss stats
std::vector<uns> per_core_last_imiss_reason;
// for assertions
std::vector<uns> per_core_last_break_reason;
std::vector<Counter> per_core_last_recover_cycle;
// <CL address, # of first demand load on-path hits of cache lines, flag for learning from a true miss> - useful count
std::vector<std::unordered_map<Addr, std::pair<Counter, Flag>>> per_core_cnt_useful;
// <CL address, # of first demand load on-path hits of cache lines, flag for learning from a true miss> - useful count after warm-up
std::vector<std::unordered_map<Addr, std::pair<Counter, Flag>>> per_core_cnt_useful_aw;
// <CL address, # of evictions w/o hit of cache lines> - unuseful count
std::vector<std::unordered_map<Addr, Counter>> per_core_cnt_unuseful;
// <CL address, # of evictions w/o hit of cache lines> - unuseful count after warm-up
std::vector<std::unordered_map<Addr, Counter>> per_core_cnt_unuseful_aw;
// Increment if useful by UDP_WEIGHT_USEFUL, decrement if unuseful by UDP_WEIGHT_UNUSEFUL
// <CL address, counter for on/off-path unuseful/useful> init by UDP_USEFUL_THRESHOLD
// OPTIMISTIC POLICY : do not prefetch if < USEFUL_THRESHOLD, otherwise, prefetch (do not prefetch only when it was unuseful at least once)
// CONSERVATIVE POLICY : prefetch if > USEFUL_THRESHOLD, otherwise, do not prefetch (prefetch only when it was useful at least once)
std::vector<std::unordered_map<Addr, int32_t>> per_core_cnt_useful_signed;
// <CL addresses, retirement count> - on-path retired cache line count
std::vector<std::unordered_map<Addr, Counter>> per_core_cnt_useful_ret;
// <CL addresses, icache miss count>
std::vector<std::map<Addr, Counter>> per_core_icache_miss;
// <CL addresses, icache miss count> after warm-up
std::vector<std::map<Addr, Counter>> per_core_icache_miss_aw;
// <CL addresses, icache hit count>
std::vector<std::map<Addr, Counter>> per_core_icache_hit;
// <CL addresses, icache hit count> after warm-up
std::vector<std::map<Addr, Counter>> per_core_icache_hit_aw;
// <CL addresses, fetched_cycle on the off-path>
std::vector<std::map<Addr, Counter>> per_core_off_fetched_cls;;
// <CL addresses, prefetched count>
std::vector<std::map<Addr, Counter>> per_core_prefetched_cls;
// <CL addresses, prefetched count> after warm-up
std::vector<std::map<Addr, Counter>> per_core_prefetched_cls_aw;
// <CL addresses, new_prefetched count>
std::vector<std::map<Addr, Counter>> per_core_new_prefetched_cls;
// <CL addresses, new_prefetched count> after warm-up
std::vector<std::map<Addr, Counter>> per_core_new_prefetched_cls_aw;
// <CL address, cyc_access_by_fdip, conf_on/off-path, cyc_evicted_from_l1_by_demand_load, cyc_evicted_from_l1_by_FDIP> - prefetched and access time information for timeliness analysis
std::vector<std::unordered_map<Addr, std::pair<std::pair<Counter, Flag>, std::pair<Counter, Counter>>>> per_core_prefetched_cls_info;
// <CL address, sequence of useful/unuseful>
std::vector<std::unordered_map<Addr, std::vector<uns8>>> per_core_useful_sequence;
// <CL address, sequence of hit/miss>
std::vector<std::unordered_map<Addr, std::vector<uns8>>> per_core_icache_sequence;
// <CL address, all sequence> char - P: prefetch, p: not prefetch, m: icache miss, h: icache hit, U: useful, u: unuseful (Counter - cycle count)
std::vector<std::unordered_map<Addr, std::vector<std::pair<char,Counter>>>> per_core_sequence_bw;
// <CL address, all sequence> char - P: prefetch, p: not prefetch, m: icache miss, h: icache hit, U: useful, u: unuseful (Counter - cycle count)
std::vector<std::unordered_map<Addr, std::vector<std::pair<char,Counter>>>> per_core_sequence_aw;
// <CL address, total miss delay>
std::vector<std::map<Addr, Counter>> per_core_per_line_delay_aw;
std::vector<Counter> per_core_cur_line_delay;
// accumulated FTQ occupancy every cycle
std::vector<uint64_t> per_core_fdip_ftq_occupancy_ops;
std::vector<uint64_t> per_core_fdip_ftq_occupancy_blocks;
std::vector<Addr> per_core_last_cl_unuseful;
std::vector<Addr> per_core_last_bbl_start_addr;
// Utility cache
std::vector<Cache> per_core_fdip_uc;
std::vector<Cache> per_core_fdip_uc_unuseful;
std::vector<Cache> per_core_fdip_uc_signed;
// bloom filters
typedef struct Bloom_Filter_struct {
  bloom_filter *bloom;
  bloom_filter *bloom2;
  bloom_filter *bloom4;
  Addr last_prefetch_candidate;
  uint32_t last_prefetch_candidate_counter;
  Counter last_clear_cycle_count;
  Counter new_prefs;
  Counter cnt_unuseful;
  Counter cnt_insert_bloom;
  Counter cnt_insert_bloom2;
  Counter cnt_insert_bloom4;
} Bloom_Filter;
std::vector<Bloom_Filter> per_core_bloom_filter;

//confidence counter
std::vector<uns> per_core_low_confidence_cnt;
std::vector<double> per_core_cf_op_distance;
//confidence stats
std::vector<FDIP_Confidence_Info> per_core_conf_info;

void alloc_mem_fdip(uns numCores) {
  per_core_cur_op.resize(numCores);
  per_core_ftq_iter.resize(numCores);
  per_core_last_line_addr.resize(numCores);
  per_core_warmed_up.resize(numCores);
  if (FDIP_ADJUSTABLE_FTQ)
    per_core_utility_timeliness_info.resize(numCores);
  if (FDIP_BP_CONFIDENCE) {
    per_core_cnt_btb_miss.resize(numCores);
    per_core_btb_miss_rate.resize(numCores);
    per_core_low_confidence_cnt.resize(numCores);
    per_core_cf_op_distance.resize(numCores);
    per_core_conf_info.resize(numCores);
  }
  per_core_last_imiss_reason.resize(numCores);
  per_core_last_break_reason.resize(numCores);
  per_core_last_recover_cycle.resize(numCores);
  per_core_cnt_useful.resize(numCores);
  per_core_cnt_useful_aw.resize(numCores);
  per_core_cnt_unuseful.resize(numCores);
  per_core_cnt_unuseful_aw.resize(numCores);
  per_core_cnt_useful_signed.resize(numCores);
  per_core_cnt_useful_ret.resize(numCores);
  per_core_icache_miss.resize(numCores);
  per_core_icache_miss_aw.resize(numCores);
  per_core_icache_hit.resize(numCores);
  per_core_icache_hit_aw.resize(numCores);
  per_core_off_fetched_cls.resize(numCores);
  per_core_prefetched_cls.resize(numCores);
  per_core_prefetched_cls_aw.resize(numCores);
  per_core_new_prefetched_cls.resize(numCores);
  per_core_new_prefetched_cls_aw.resize(numCores);
  per_core_prefetched_cls_info.resize(numCores);
  per_core_useful_sequence.resize(numCores);
  per_core_icache_sequence.resize(numCores);
  per_core_sequence_bw.resize(numCores);
  per_core_sequence_aw.resize(numCores);
  per_core_per_line_delay_aw.resize(numCores);
  per_core_cur_line_delay.resize(numCores);
  per_core_fdip_ftq_occupancy_ops.resize(numCores);
  per_core_fdip_ftq_occupancy_blocks.resize(numCores);

  if (FDIP_UTILITY_HASH_ENABLE)
    ASSERT(fdip_proc_id, FDIP_UTILITY_PREF_POLICY >= Utility_Pref_Policy::PREF_CONV_FROM_USEFUL_SET &&
        FDIP_UTILITY_PREF_POLICY < Utility_Pref_Policy::PREF_POL_END);

  if (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER) {
    per_core_last_cl_unuseful.resize(numCores);
    per_core_last_bbl_start_addr.resize(numCores);
    per_core_seniority_ftq.resize(numCores);
  }
  if (FDIP_UC_SIZE) {
    per_core_fdip_uc.resize(numCores);
    per_core_fdip_uc_unuseful.resize(numCores);
    per_core_fdip_uc_signed.resize(numCores);
  }
  if (FDIP_BLOOM_FILTER)
    per_core_bloom_filter.resize(numCores);
}

void init_fdip(uns proc_id) {
  per_core_ftq_iter[proc_id] = decoupled_fe_new_ftq_iter();
  per_core_last_line_addr[proc_id] = 0;
  per_core_warmed_up[proc_id] = FALSE;
  if (FDIP_ADJUSTABLE_FTQ)
    per_core_utility_timeliness_info[proc_id] = {0, 0, 0.0, 0, 0, 0.0, FALSE, 0, 0};
  if (FDIP_BP_CONFIDENCE) {
    per_core_cnt_btb_miss[proc_id] = 0;
    per_core_btb_miss_rate[proc_id] = 0.0;

    //per_core_conf_info[proc_id].cur_op = nullptr;
    per_core_conf_info[proc_id].prev_op = nullptr;

    per_core_conf_info[proc_id].fdip_on_conf_off_event = false;

    per_core_conf_info[proc_id].num_conf_0_branches = 0;
    per_core_conf_info[proc_id].num_conf_1_branches = 0;
    per_core_conf_info[proc_id].num_conf_2_branches = 0;
    per_core_conf_info[proc_id].num_conf_3_branches = 0;

    per_core_conf_info[proc_id].num_cf_br = 0;
    per_core_conf_info[proc_id].num_cf_cbr = 0;
    per_core_conf_info[proc_id].num_cf_call = 0;
    per_core_conf_info[proc_id].num_cf_ibr = 0;
    per_core_conf_info[proc_id].num_cf_icall = 0;
    per_core_conf_info[proc_id].num_cf_ico = 0;
    per_core_conf_info[proc_id].num_cf_ret = 0;
    per_core_conf_info[proc_id].num_cf_sys = 0;

    per_core_conf_info[proc_id].num_BTB_misses = 0;
    per_core_conf_info[proc_id].num_op_dist_incs = 0;

    per_core_conf_info[proc_id].fdip_off_path_event = false;
    per_core_conf_info[proc_id].fdip_off_conf_on_event = false;
  }
  per_core_last_imiss_reason[proc_id] = Imiss_Reason::IMISS_NOT_PREFETCHED;
  per_core_last_break_reason[proc_id] = BR_REACH_FTQ_END;
  per_core_last_recover_cycle[proc_id] = 0;
  per_core_fdip_ftq_occupancy_ops[proc_id] = 0;
  per_core_fdip_ftq_occupancy_blocks[proc_id] = 0;
  per_core_cur_line_delay[proc_id] = 0;
  if (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER) {
    per_core_last_cl_unuseful[proc_id] = 0;
    per_core_last_bbl_start_addr[proc_id] = 0;
  }
  if (FDIP_UC_SIZE) {
    ASSERT(fdip_proc_id, !FDIP_BLOOM_FILTER);
    init_cache(&per_core_fdip_uc[proc_id], "FDIP_USEFULNESS_CACHE", FDIP_UC_SIZE, FDIP_UC_ASSOC, ICACHE_LINE_SIZE,
               0, REPL_TRUE_LRU); //Data size = 2 byte
    init_cache(&per_core_fdip_uc_unuseful[proc_id], "FDIP_USEFULNESS_CACHE_UNUSEFUL", FDIP_UC_SIZE, FDIP_UC_ASSOC, ICACHE_LINE_SIZE,
               0, REPL_TRUE_LRU); //Data size = 2 byte
    init_cache(&per_core_fdip_uc_signed[proc_id], "FDIP_USEFULNESS_CACHE_SIGNED", FDIP_UC_SIZE, FDIP_UC_ASSOC, ICACHE_LINE_SIZE,
               sizeof(int32_t), REPL_TRUE_LRU);
  }
  if (FDIP_BLOOM_FILTER) {
    ASSERT(fdip_proc_id, !FDIP_UC_SIZE && !FDIP_UTILITY_HASH_ENABLE);
    bloom_parameters bloom1_parameters;
    bloom1_parameters.projected_element_count = FDIP_BLOOM_ENTRIES;
    bloom1_parameters.false_positive_probability = 0.005;
    bloom1_parameters.compute_optimal_parameters();
    per_core_bloom_filter[proc_id].bloom = new bloom_filter(bloom1_parameters);

    bloom_parameters bloom2_parameters;
    bloom2_parameters.projected_element_count = FDIP_BLOOM2_ENTRIES;
    bloom2_parameters.false_positive_probability = 0.005;
    bloom2_parameters.compute_optimal_parameters();
    per_core_bloom_filter[proc_id].bloom2 = new bloom_filter(bloom2_parameters);

    bloom_parameters bloom4_parameters;
    bloom4_parameters.projected_element_count = FDIP_BLOOM4_ENTRIES;
    bloom4_parameters.false_positive_probability = 0.005;
    bloom4_parameters.compute_optimal_parameters();
    per_core_bloom_filter[proc_id].bloom4 = new bloom_filter(bloom4_parameters);

    per_core_bloom_filter[proc_id].last_prefetch_candidate_counter = 0;
    per_core_bloom_filter[proc_id].last_clear_cycle_count = 0;
    per_core_bloom_filter[proc_id].new_prefs = 0;
    per_core_bloom_filter[proc_id].cnt_unuseful = 0;
    per_core_bloom_filter[proc_id].cnt_insert_bloom = 0;
    per_core_bloom_filter[proc_id].cnt_insert_bloom2 = 0;
    per_core_bloom_filter[proc_id].cnt_insert_bloom4 = 0;
  }
}

void set_fdip(int _proc_id, Icache_Stage *_ic) {
  fdip_proc_id = _proc_id;
  ic_ref = _ic;
  iter = per_core_ftq_iter[_proc_id];
}

void recover_fdip() {
  per_core_last_line_addr[fdip_proc_id] = 0;
  per_core_last_recover_cycle[fdip_proc_id] = cycle_count;

  if(FDIP_BP_CONFIDENCE){
    per_core_low_confidence_cnt[fdip_proc_id] = 0;
    per_core_cf_op_distance[fdip_proc_id] = 0.0;
    // set previous reset previous instruction
    //per_core_conf_info[fdip_proc_id].cur_op = nullptr;
    per_core_conf_info[fdip_proc_id].prev_op = nullptr;
    // reset counters and event flags
    per_core_conf_info[fdip_proc_id].fdip_on_conf_off_event = false;

    per_core_conf_info[fdip_proc_id].num_conf_0_branches = 0;
    per_core_conf_info[fdip_proc_id].num_conf_1_branches = 0;
    per_core_conf_info[fdip_proc_id].num_conf_2_branches = 0;
    per_core_conf_info[fdip_proc_id].num_conf_3_branches = 0;

    per_core_conf_info[fdip_proc_id].num_cf_br = 0;
    per_core_conf_info[fdip_proc_id].num_cf_cbr = 0;
    per_core_conf_info[fdip_proc_id].num_cf_call = 0;
    per_core_conf_info[fdip_proc_id].num_cf_ibr = 0;
    per_core_conf_info[fdip_proc_id].num_cf_icall = 0;
    per_core_conf_info[fdip_proc_id].num_cf_ico = 0;
    per_core_conf_info[fdip_proc_id].num_cf_ret = 0;
    per_core_conf_info[fdip_proc_id].num_cf_sys = 0;

    per_core_conf_info[fdip_proc_id].num_BTB_misses = 0;
    per_core_conf_info[fdip_proc_id].num_op_dist_incs = 0;

    per_core_conf_info[fdip_proc_id].fdip_off_conf_on_event = false;
    per_core_conf_info[fdip_proc_id].fdip_off_path_event = false;
  }

  if (FDIP_ADJUSTABLE_FTQ)
    uftq_set_ftq_ft_num(fdip_proc_id);
}

static inline void bloom_clear(uns proc_id) {
  Bloom_Filter* bf = &per_core_bloom_filter[proc_id];
  bf->bloom->clear();
  bf->bloom2->clear();
  bf->bloom4->clear();
  STAT_EVENT(proc_id, FDIP_BLOOM_CLEAR1);
  STAT_EVENT(proc_id, FDIP_BLOOM_CLEAR2);
  STAT_EVENT(proc_id, FDIP_BLOOM_CLEAR4);
}

void update_fdip() {
  if (!FDIP_ENABLE)
    return;
  if (FULL_WARMUP && warmup_dump_done[fdip_proc_id] && !per_core_warmed_up[fdip_proc_id])
    per_core_warmed_up[fdip_proc_id] = TRUE;

  uint32_t ops_per_cycle = 0;
  int ftq_entry_per_cycle = 0;
  FDIP_Break break_reason = BR_REACH_FTQ_END;
  bool end_of_block;
  per_cyc_ipref = 0;
  if (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER) {
    per_core_last_cl_unuseful[fdip_proc_id] = 0;
    if (FDIP_BLOOM_FILTER && (cycle_count - per_core_bloom_filter[fdip_proc_id].last_clear_cycle_count > FDIP_BLOOM_CLEAR_CYC_PERIOD)) {
      Bloom_Filter* bf = &per_core_bloom_filter[fdip_proc_id];
      bf->last_clear_cycle_count = cycle_count;
      bf->new_prefs = 0;
      bf->cnt_unuseful = 0;
    }
  }

  if (FDIP_BP_CONFIDENCE && (cycle_count % FDIP_BTB_MISS_SAMPLE_RATE == 0)) {
    per_core_btb_miss_rate[fdip_proc_id] = (double)per_core_cnt_btb_miss[fdip_proc_id] / (double)FDIP_BTB_MISS_SAMPLE_RATE;
    per_core_cnt_btb_miss[fdip_proc_id] = 0;
  }

  Op *op = NULL;
  for (op = decoupled_fe_ftq_iter_get(iter, &end_of_block); op != NULL; op = decoupled_fe_ftq_iter_get_next(iter, &end_of_block), ops_per_cycle++) {
    //set previous op, only if on path or first off path instruction
    if(FDIP_BP_CONFIDENCE &&
        per_core_cur_op[fdip_proc_id] &&
	(per_core_cur_op[fdip_proc_id]->op_num != op->op_num) &&
        (!(op->off_path) || !(per_core_conf_info[fdip_proc_id].fdip_off_path_event))){
      per_core_conf_info[fdip_proc_id].prev_op = per_core_cur_op[fdip_proc_id];
      DEBUG(fdip_proc_id, "Set prev_op off_path:%i, op_num:%llu, cf_type:%i\n", per_core_conf_info[fdip_proc_id].prev_op->off_path, per_core_conf_info[fdip_proc_id].prev_op->op_num, per_core_conf_info[fdip_proc_id].prev_op->table_info->cf_type);
      if(op->off_path) {
        per_core_conf_info[fdip_proc_id].fdip_off_path_event = true;
      }
    }
    per_core_cur_op[fdip_proc_id] = op;
    DEBUG(fdip_proc_id, "Set per_core_cur_op off_path:%i, op_num:%llu, cf_type:%i\n", per_core_cur_op[fdip_proc_id]->off_path, per_core_cur_op[fdip_proc_id]->op_num, per_core_cur_op[fdip_proc_id]->table_info->cf_type);

    Addr last_line_addr = per_core_last_line_addr[fdip_proc_id];
    Flag emit_new_prefetch = FALSE;
    if (ftq_entry_per_cycle >= MAX_FTQ_ENTRY_CYC && ops_per_cycle >= IC_ISSUE_WIDTH) {
      DEBUG(fdip_proc_id, "Break due to max FTQ entries per cycle\n");
      break_reason = BR_MAX_FTQ_ENTRY_CYC;
      break;
    }
    if (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER) {
      clear_old_seniority_ftq();
      if (!per_core_last_bbl_start_addr[fdip_proc_id]) {
        per_core_last_bbl_start_addr[fdip_proc_id] = op->inst_info->addr;
        DEBUG(fdip_proc_id, "init last_bbl_start_addr: %llx\n", per_core_last_bbl_start_addr[fdip_proc_id]);
      }
    }
    //update confidence
    if (FDIP_BP_CONFIDENCE) {
      if (FDIP_BP_PERFECT_CONFIDENCE) {
        if (fdip_off_path(fdip_proc_id))
          per_core_low_confidence_cnt[fdip_proc_id] = ~0U;
        if(per_core_low_confidence_cnt[fdip_proc_id] == ~0U)
          ASSERT(0,fdip_off_path(fdip_proc_id));
        per_core_cf_op_distance[fdip_proc_id] = 0.0;
      } else if (FDIP_BTB_MISS_BP_TAKEN_CONF) {
        btb_miss_bp_taken_conf_update(op);
      } else {
        default_conf_update(op);
      }
    }

    //log conf stats

    //if it is a cf with bp conf
    if((op)->table_info->cf_type == CF_CBR || 
      (op)->table_info->cf_type == CF_IBR || 
      (op)->table_info->cf_type == CF_ICALL){
      if(op->oracle_info.mispred) {
        //reorder stats
        STAT_EVENT(fdip_proc_id, FDIP_BP_CONF_0_MISPRED + op->bp_confidence);
      } else {
        STAT_EVENT(fdip_proc_id, FDIP_BP_CONF_0_CORRECT + op->bp_confidence);
      }
    }

    uint64_t pc_addr = op->inst_info->addr;
    Addr line_addr = op->inst_info->addr & ~0x3F;
    DEBUG(fdip_proc_id, "op_num: %llu, op->inst_info->addr: %llx, line_addr: %llx, last_line_addr: %llx, off-path: %d\n", op->op_num, op->inst_info->addr, line_addr, last_line_addr, fdip_off_path(fdip_proc_id));
    if (line_addr != last_line_addr) {
      STAT_EVENT(ic_ref->proc_id, FDIP_ATTEMPTED_PREF_ONPATH + op->off_path);
      DEBUG(fdip_proc_id, "fdip off path: %d, conf off path: %d\n", fdip_off_path(fdip_proc_id), fdip_conf_off_path(fdip_proc_id));
      if (FDIP_BP_CONFIDENCE)
        log_stats_bp_conf();
      if (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER || FDIP_PERFECT_PREFETCH)
        emit_new_prefetch = determine_usefulness(line_addr, op);
      else
        emit_new_prefetch = TRUE;

      Flag demand_hit_prefetch = FALSE;
      Flag demand_hit_writeback = FALSE;
      Mem_Queue_Entry* queue_entry = NULL;
      Flag ramulator_match = FALSE;
      Addr dummy_addr = 0;
      bool line = false;
      Mem_Req* mem_req = NULL;
      if (emit_new_prefetch) {
        line = (Inst_Info**)cache_access(&ic_ref->icache, pc_addr, &line_addr, TRUE);
        // icache_line_info cache should be accessed same times with icache for a consistant line information
        if (WP_COLLECT_STATS) {
          bool line_info = (Icache_Data*)cache_access(&ic_ref->icache_line_info, pc_addr, &dummy_addr, TRUE);
          UNUSED(line_info);
        }
        bool mlc_line = (Inst_Info**)cache_access(&mem->uncores[ic_ref->proc_id].mlc->cache, pc_addr, &dummy_addr, FALSE);
        bool l1_line = (Inst_Info**)cache_access(&mem->uncores[ic_ref->proc_id].l1->cache, pc_addr, &dummy_addr, FALSE);
        UNUSED(dummy_addr);
        uns pref_from = line ? 0 : (mlc_line ? 1 : (l1_line ? 2 : 3));
        STAT_EVENT(ic_ref->proc_id, FDIP_PREFETCH_HIT_ICACHE + pref_from);
        mem_req = mem_search_reqbuf_wrapper(ic_ref->proc_id, line_addr,
            MRT_FDIPPRFON, ICACHE_LINE_SIZE, &demand_hit_prefetch, &demand_hit_writeback,
            QUEUE_MLC | QUEUE_L1 | QUEUE_BUS_OUT |
            QUEUE_MEM | QUEUE_L1FILL | QUEUE_MLC_FILL,
            &queue_entry, &ramulator_match);

        if (!mem_req) {
          mem_req = mem_search_reqbuf_wrapper(ic_ref->proc_id, line_addr,
              MRT_FDIPPRFOFF, ICACHE_LINE_SIZE, &demand_hit_prefetch, &demand_hit_writeback,
              QUEUE_MLC | QUEUE_L1 | QUEUE_BUS_OUT |
              QUEUE_MEM | QUEUE_L1FILL | QUEUE_MLC_FILL,
              &queue_entry, &ramulator_match);
        }

        if (line) {
          DEBUG(fdip_proc_id, "probe hit for %llx\n", line_addr);
          probe_prefetched_cls(line_addr);
          STAT_EVENT(ic_ref->proc_id, FDIP_PREF_ICACHE_PROBE_HIT_ONPATH + op->off_path);
        }
      }

      Mem_Req_Type mem_type = fdip_conf_off_path(ic_ref->proc_id)? MRT_FDIPPRFOFF : MRT_FDIPPRFON;
      if (!emit_new_prefetch && !line && !mem_req)
        insert_pref_candidate_to_seniority_ftq(line_addr);
      if (FDIP_UTILITY_HASH_ENABLE || FDIP_UC_SIZE || FDIP_BLOOM_FILTER)
        INC_STAT_EVENT(fdip_proc_id, FDIP_SENIORITY_FTQ_ACCUMULATED, per_core_seniority_ftq[fdip_proc_id].size());
      Flag mem_req_buf_full = FALSE;
      if (emit_new_prefetch && !line && !mem_req && !mem_can_allocate_req_buffer(fdip_proc_id, mem_type, FALSE)) {
        mem_req_buf_full = TRUE;
        // should keep running ahead without breaking the loop by failing to emit a prefetch when FDIP is only one FTQ entry ahead where the backend fetches the FT soon
        // freeze FDIP when mem_req buffer hits the limit. This should rarely happends if mem_req_buffer_entries and ramulator_readq_entries are big enough.
        if (FDIP_FREEZE_AT_MEM_BUF_LIMIT && decoupled_fe_ftq_iter_ft_offset(iter) > 1) {
          DEBUG(fdip_proc_id, "Break due to full mem_req buf\n");
          break_reason = BR_FULL_MEM_REQ_BUF;
          break;
        }
      }

      if (emit_new_prefetch)
        STAT_EVENT(ic_ref->proc_id, FDIP_DECIDE_PREF_ONPATH + op->off_path);

      if (!line && emit_new_prefetch) { // create a mem request only if line doesn't exist. If the corresponding mem_req exists, it will merge.
        uns success = Mem_Queue_Req_Result::FAILED;
        if (FDIP_PREF_NO_LATENCY) {
          Mem_Req req;
          req.off_path           = op ? op->off_path : FALSE;
          req.off_path_confirmed = FALSE;
          req.type               = mem_type;
          mem_req_set_types(&req, mem_type);
          req.proc_id            = fdip_proc_id;
          req.addr               = line_addr;
          req.oldest_op_unique_num = (Counter)0;
          req.oldest_op_op_num = (Counter)0;
          req.oldest_op_addr = (Addr)0;
          req.dirty_l0 = op && op->table_info->mem_type == MEM_ST && !op->off_path;
          req.fdip_pref_off_path = op->off_path;
          req.demand_icache_emitted_cycle = 0;
          req.fdip_emitted_cycle = cycle_count;
	  req.ghist = g_bp_data->global_hist;
          if (icache_fill_line(&req)) {
            STAT_EVENT(fdip_proc_id, FDIP_NEW_PREFETCHES_ONPATH + op->off_path);
            if (FDIP_BLOOM_FILTER)
              per_core_bloom_filter[fdip_proc_id].new_prefs++;
          } else
            ASSERT(fdip_proc_id, false);
          success = Mem_Queue_Req_Result::SUCCESS_NEW;
        } else {
          success = new_mem_req(mem_type, fdip_proc_id, line_addr,
              ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count, 0);
              //ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count++, 0); // bug?
          // A buffer entry should be available since it is checked by mem_can_allocate_req_buffer for a new prefetch
          if (success == Mem_Queue_Req_Result::SUCCESS_NEW) {
            STAT_EVENT(ic_ref->proc_id, FDIP_NEW_PREFETCHES_ONPATH + op->off_path);
            DEBUG(fdip_proc_id, "Success to emit a new prefetch for %llx\n", line_addr);
            per_cyc_ipref++;
            if (FDIP_BLOOM_FILTER)
              per_core_bloom_filter[fdip_proc_id].new_prefs++;
          } else if (success == Mem_Queue_Req_Result::SUCCESS_MERGED) {
            STAT_EVENT(ic_ref->proc_id, FDIP_PREF_MSHR_PROBE_HIT_ONPATH + op->off_path);
            DEBUG(fdip_proc_id, "Success to merge a prefetch for %llx\n", line_addr);
          } else if (success == Mem_Queue_Req_Result::FAILED) {
            ASSERT(ic_ref->proc_id, mem_req_buf_full);
            STAT_EVENT(ic_ref->proc_id, FDIP_PREF_FAILED_ONPATH + op->off_path);
            DEBUG(fdip_proc_id, "Failed to emit a prefetch for %llx\n", line_addr);
          }
        }
        if (FDIP_BP_CONFIDENCE)
          log_stats_bp_conf_emitted();
        inc_prefetched_cls(line_addr, success);
      } else {
        not_prefetch(line_addr);
      }
      per_core_last_line_addr[fdip_proc_id] = line_addr;
    }
    if (end_of_block) {
      ftq_entry_per_cycle++;
      DEBUG(fdip_proc_id, "End of block - ftq_entry_per_cycle: %d\n", ftq_entry_per_cycle);
    }
  }
  if (!op) {
    if (!decoupled_fe_ftq_iter_ft_offset(iter)) {
      DEBUG(fdip_proc_id, "Break due to FTQ Empty\n");
      break_reason = BR_FTQ_EMPTY;
    } else {
      break_reason = BR_REACH_FTQ_END;
      DEBUG(fdip_proc_id, "Break due to reaching FTQ end\n");
    }
  }

  STAT_EVENT(ic_ref->proc_id, FDIP_BREAK_REACH_FTQ_END + break_reason);
  DEBUG(fdip_proc_id, "FTQ size : %lu, FDIP prefetch FT offset : %lu\n", decoupled_fe_ftq_num_fts(), decoupled_fe_ftq_iter_ft_offset(iter));
  if (FDIP_ADJUSTABLE_FTQ && cycle_count % FDIP_ADJUSTABLE_FTQ_CYC == 0) {
    if (per_core_utility_timeliness_info[ic_ref->proc_id].unuseful_prefetches) {
      per_core_utility_timeliness_info[ic_ref->proc_id].utility_ratio = (double)per_core_utility_timeliness_info[ic_ref->proc_id].useful_prefetches/((double)per_core_utility_timeliness_info[ic_ref->proc_id].useful_prefetches+(double)per_core_utility_timeliness_info[ic_ref->proc_id].unuseful_prefetches);
      per_core_utility_timeliness_info[ic_ref->proc_id].adjust = TRUE;
    }
    per_core_utility_timeliness_info[ic_ref->proc_id].useful_prefetches = 0;
    per_core_utility_timeliness_info[ic_ref->proc_id].unuseful_prefetches = 0;
    DEBUG(fdip_proc_id, "Update utility ratio : %lf\n", per_core_utility_timeliness_info[ic_ref->proc_id].utility_ratio);
    DEBUG(fdip_proc_id, "mshr_prefetch_hits : %llu, icache_prefetch_hits : %llu\n", per_core_utility_timeliness_info[ic_ref->proc_id].mshr_prefetch_hits, per_core_utility_timeliness_info[ic_ref->proc_id].icache_prefetch_hits);
    if (per_core_utility_timeliness_info[ic_ref->proc_id].icache_prefetch_hits &&
        (per_core_utility_timeliness_info[ic_ref->proc_id].icache_prefetch_hits + per_core_utility_timeliness_info[ic_ref->proc_id].mshr_prefetch_hits > 4)) { // do not adjust if there are too few samples?
      per_core_utility_timeliness_info[ic_ref->proc_id].timeliness_ratio = (double)per_core_utility_timeliness_info[ic_ref->proc_id].mshr_prefetch_hits/((double)per_core_utility_timeliness_info[ic_ref->proc_id].mshr_prefetch_hits+(double)per_core_utility_timeliness_info[ic_ref->proc_id].icache_prefetch_hits);
      per_core_utility_timeliness_info[ic_ref->proc_id].adjust = TRUE;
    }
    per_core_utility_timeliness_info[ic_ref->proc_id].mshr_prefetch_hits = 0;
    per_core_utility_timeliness_info[ic_ref->proc_id].icache_prefetch_hits = 0;
    DEBUG(fdip_proc_id, "Update timeliness ratio : %lf\n", per_core_utility_timeliness_info[ic_ref->proc_id].timeliness_ratio);
  }

  per_core_fdip_ftq_occupancy_ops[ic_ref->proc_id] += decoupled_fe_ftq_iter_offset(iter);
  INC_STAT_EVENT(fdip_proc_id, FDIP_FTQ_OCCUPANCY_OPS_ACCUMULATED, decoupled_fe_ftq_iter_offset(iter));
  if (break_reason == BR_REACH_FTQ_END) {
    per_core_fdip_ftq_occupancy_blocks[ic_ref->proc_id] += decoupled_fe_ftq_iter_ft_offset(iter);
    INC_STAT_EVENT(fdip_proc_id, FDIP_FTQ_OCCUPANCY_BLOCKS_ACCUMULATED, decoupled_fe_ftq_num_fts());
  }
  per_core_last_break_reason[ic_ref->proc_id] = break_reason;
}

uns64 fdip_get_ghist() {
  return g_bp_data->global_hist;
}

uns64 fdip_hash_addr_ghist(uint64_t addr, uint64_t ghist) {
  //ghist is 32 bit, most recent branch outcome at bit 31
  return addr ^ ((ghist >> (32-FDIP_GHIST_BITS))<< (64-FDIP_GHIST_BITS));
}

Flag fdip_off_path(uns proc_id) {
  ASSERT(proc_id, per_core_cur_op[proc_id]);
  return per_core_cur_op[proc_id]->off_path;
}

Flag fdip_conf_off_path(uns proc_id) {
  if (FDIP_BP_CONFIDENCE) {
    if (per_core_low_confidence_cnt[fdip_proc_id] < FDIP_OFF_PATH_THRESHOLD)
      return FALSE;
    else
      return TRUE;
  }
  return fdip_off_path(proc_id);
}

template<typename A, typename B>
std::pair<B,A> flip_pair(const std::pair<A,B> &p)
{
  return std::pair<B,A>(p.second, p.first);
}

template<typename A, typename B>
std::multimap<B,A> flip_map(const std::map<A,B> &src)
{
  std::multimap<B,A> dst;
  std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()), flip_pair<A,B>);
  return dst;
}

void print_cl_info(uns proc_id) {
  if (!FDIP_ENABLE)
    return;
  std::unordered_map<Addr, Counter>* cnt_useful_ret = &per_core_cnt_useful_ret[proc_id];
  std::map<Addr, Counter>* prefetched_cls = &per_core_prefetched_cls[proc_id];
  std::map<Addr, Counter>* icache_miss = &per_core_icache_miss[proc_id];
  std::map<Addr, Counter>* icache_hit = &per_core_icache_hit[proc_id];

  DEBUG(proc_id, "icache miss cache lines (UNIQUE_MISSED_LINES) size: %lu, icache hit cache lines (UNIQUE_MISSED_LINES): %lu\n", icache_miss->size(), icache_hit->size());
  INC_STAT_EVENT(proc_id, ICACHE_UNIQUE_MISSED_LINES, icache_miss->size());
  INC_STAT_EVENT(proc_id, ICACHE_UNIQUE_HIT_LINES, icache_hit->size());
  std::multimap<Counter, Addr> icache_miss_sorted = flip_map(*icache_miss);
  for(std::multimap<Counter, Addr>::const_iterator it = icache_miss_sorted.begin();
      it != icache_miss_sorted.end(); ++it) {
    DEBUG(proc_id, "[set %u] 0x%llx missed %llu times\n", (uns)(it->second >> ic_ref->icache.shift_bits & ic_ref->icache.set_mask), it->second, it->first);
  }
  DEBUG(proc_id, "unique prefetched lines (UNIQUE_PREFETCHED_LINES) size: %lu\n", prefetched_cls->size());
  std::multimap<Counter, Addr> prefetched_cls_sorted = flip_map(*prefetched_cls);
  for(std::multimap<Counter, Addr>::const_iterator it = prefetched_cls_sorted.begin();
      it != prefetched_cls_sorted.end(); ++it) {
    auto useful_ret_iter = cnt_useful_ret->find(it->second);
    if (useful_ret_iter == cnt_useful_ret->end()) {
      DEBUG(proc_id, "Unuseful 0x%llx prefetched %llu times\n", it->second, it->first);
    }
  }

  std::unordered_map<Addr, int32_t>* cnt_learned_cl = &per_core_cnt_useful_signed[proc_id];
  FILE* fp = fopen("per_line_icache_line_info.csv", "w");
  fprintf(fp, "cl_addr,useful_cnt,unuseful_cnt,prefetch_cnt,new_prefetch_cnt,icache_hit,icache_miss\n");
  for(auto it = cnt_learned_cl->begin(); it != cnt_learned_cl->end(); ++it) {
    auto cnt_useful_iter = per_core_cnt_useful[proc_id].find(it->first);
    auto cnt_unuseful_iter = per_core_cnt_unuseful[proc_id].find(it->first);
    auto cnt_prefetch_iter = per_core_prefetched_cls[proc_id].find(it->first);
    auto cnt_new_prefetch_iter = per_core_new_prefetched_cls[proc_id].find(it->first);
    auto hit_iter = per_core_icache_hit[proc_id].find(it->first);
    auto miss_iter = per_core_icache_miss[proc_id].find(it->first);
    Counter cnt_useful = (cnt_useful_iter != per_core_cnt_useful[proc_id].end())? cnt_useful_iter->second.first : 0;
    Counter cnt_unuseful = (cnt_unuseful_iter != per_core_cnt_unuseful[proc_id].end())? cnt_unuseful_iter->second : 0;
    Counter cnt_prefetch = (cnt_prefetch_iter != per_core_prefetched_cls[proc_id].end())? cnt_prefetch_iter->second : 0;
    Counter cnt_new_prefetch = (cnt_new_prefetch_iter != per_core_new_prefetched_cls[proc_id].end())? cnt_new_prefetch_iter->second : 0;
    Counter num_hit = (hit_iter != per_core_icache_hit[proc_id].end())? hit_iter->second : 0;
    Counter num_miss = (miss_iter != per_core_icache_miss[proc_id].end())? miss_iter->second : 0;
    fprintf(fp, "%llx,%llu,%llu,%llu,%llu,%llu,%llu\n", it->first, cnt_useful, cnt_unuseful, cnt_prefetch, cnt_new_prefetch, num_hit, num_miss);
    ASSERT(proc_id, (cnt_useful_iter != per_core_cnt_useful[proc_id].end()) || (cnt_unuseful_iter != per_core_cnt_unuseful[proc_id].end()));
  }
  fclose(fp);

  cnt_learned_cl = &per_core_cnt_useful_signed[proc_id];
  fp = fopen("per_line_icache_line_info_after_warmup.csv", "w");
  fprintf(fp, "cl_addr,useful_cnt,unuseful_cnt,prefetch_cnt,new_prefetch_cnt,icache_hit,icache_miss\n");
  for(auto it = cnt_learned_cl->begin(); it != cnt_learned_cl->end(); ++it) {
    auto cnt_useful_iter = per_core_cnt_useful_aw[proc_id].find(it->first);
    auto cnt_unuseful_iter = per_core_cnt_unuseful_aw[proc_id].find(it->first);
    auto cnt_prefetch_iter = per_core_prefetched_cls_aw[proc_id].find(it->first);
    auto cnt_new_prefetch_iter = per_core_new_prefetched_cls_aw[proc_id].find(it->first);
    auto hit_iter = per_core_icache_hit_aw[proc_id].find(it->first);
    auto miss_iter = per_core_icache_miss_aw[proc_id].find(it->first);
    Counter cnt_useful = (cnt_useful_iter != per_core_cnt_useful_aw[proc_id].end())? cnt_useful_iter->second.first : 0;
    Counter cnt_unuseful = (cnt_unuseful_iter != per_core_cnt_unuseful_aw[proc_id].end())? cnt_unuseful_iter->second : 0;
    Counter cnt_prefetch = (cnt_prefetch_iter != per_core_prefetched_cls_aw[proc_id].end())? cnt_prefetch_iter->second : 0;
    Counter cnt_new_prefetch = (cnt_new_prefetch_iter != per_core_new_prefetched_cls_aw[proc_id].end())? cnt_new_prefetch_iter->second : 0;
    Counter num_hit = (hit_iter != per_core_icache_hit_aw[proc_id].end())? hit_iter->second : 0;
    Counter num_miss = (miss_iter != per_core_icache_miss_aw[proc_id].end())? miss_iter->second : 0;
    if (cnt_useful != 0 || cnt_unuseful != 0)
      fprintf(fp, "%llx,%llu,%llu,%llu,%llu,%llu,%llu\n", it->first, cnt_useful, cnt_unuseful, cnt_prefetch, cnt_new_prefetch, num_hit, num_miss);
  }
  fclose(fp);

  std::unordered_map<Addr, std::vector<uns8>>* useful_seq = &per_core_useful_sequence[proc_id];
  fp = fopen("per_line_useful_seq.csv", "w");
  fprintf(fp, "cl_addr,seq\n");
  for(auto it = useful_seq->begin(); it != useful_seq->end(); ++it) {
    fprintf(fp, "%llx", it->first);
    for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
      fprintf(fp, ",%u", *it2);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);

  std::unordered_map<Addr, std::vector<uns8>>* icache_seq = &per_core_icache_sequence[proc_id];
  fp = fopen("per_line_icache_seq.csv", "w");
  fprintf(fp, "cl_addr,seq\n");
  for(auto it = icache_seq->begin(); it != icache_seq->end(); ++it) {
    fprintf(fp, "%llx", it->first);
    for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
      fprintf(fp, ",%u", *it2);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);

  std::unordered_map<Addr, std::vector<std::pair<char,Counter>>>* seq = &per_core_sequence_aw[proc_id];
  fp = fopen("per_line_seq_aw.csv", "w");
  fprintf(fp, "cl_addr,seq\n");
  for(auto it = seq->begin(); it != seq->end(); ++it) {
    fprintf(fp, "%llx", it->first);
    if (it->second.size() == 2) {
      auto it2 = it->second.begin();
      if (it2++->first == 'P' && it2->first == 'u')
        STAT_EVENT(proc_id, FDIP_PREFETCH_EVICT_NO_HIT_ONLY_ONCE);
    }
    for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
      fprintf(fp, ",%c", it2->first);
    }
    fprintf(fp, "\n");
    for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
      fprintf(fp, ",%lld", it2->second);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);

  std::map<Addr, Counter>* per_line_delay = &per_core_per_line_delay_aw[proc_id];
  std::multimap<Counter, Addr> per_line_delay_sorted = flip_map(*per_line_delay);
  fp = fopen("per_line_delay.csv", "w");
  fprintf(fp, "cl_addr,delay\n");
  for(std::multimap<Counter, Addr>::const_iterator it = per_line_delay_sorted.begin();
      it != per_line_delay_sorted.end(); ++it) {
    fprintf(fp, "%llx,%lld\n", it->second, it->first);
  }
  fclose(fp);
}

void inc_cnt_useful(uns proc_id, Addr line_addr, Flag pref_miss) {
  auto useful_iter = per_core_cnt_useful[proc_id].find(line_addr);
  DEBUG(proc_id, "cnt_useful size %ld\n", per_core_cnt_useful[proc_id].size());
  if (useful_iter == per_core_cnt_useful[proc_id].end()) {
    DEBUG(proc_id, "%llx useful line new insert\n", line_addr);
    STAT_EVENT(proc_id, ICACHE_USEFUL_FETCHES);
    per_core_cnt_useful[proc_id].insert(std::make_pair(std::move(line_addr), std::make_pair(1, pref_miss)));
  } else {
    useful_iter->second.first++;
    useful_iter->second.second = pref_miss;
  }
  DEBUG(proc_id, "cnt_useful size after inserted %ld\n", per_core_cnt_useful[proc_id].size());

  if (per_core_warmed_up[proc_id]) {
    auto it = per_core_cnt_useful_aw[proc_id].find(line_addr);
    if (it == per_core_cnt_useful_aw[proc_id].end())
      per_core_cnt_useful_aw[proc_id].insert(std::make_pair(std::move(line_addr), std::make_pair(1, pref_miss)));
    else {
      it->second.first++;
      it->second.second = pref_miss;
    }

    auto it2 = per_core_sequence_aw[proc_id].find(line_addr);
    if (it2 == per_core_sequence_aw[proc_id].end()) {
      per_core_sequence_aw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_aw[proc_id][line_addr].push_back(std::make_pair('U',cycle_count));
    } else {
      it2->second.push_back(std::make_pair('U',cycle_count));
    }
  } else {
    auto iter = per_core_sequence_bw[proc_id].find(line_addr);
    if (iter == per_core_sequence_bw[proc_id].end()) {
      per_core_sequence_bw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_bw[proc_id][line_addr].push_back(std::make_pair('U',cycle_count));
    } else {
      iter->second.push_back(std::make_pair('U',cycle_count));
    }
  }
}

void inc_cnt_unuseful(uns proc_id, Addr line_addr) {
  if (FDIP_BLOOM_FILTER)
    per_core_bloom_filter[proc_id].cnt_unuseful++;
  auto unuseful_iter = per_core_cnt_unuseful[proc_id].find(line_addr);
  if (unuseful_iter == per_core_cnt_unuseful[proc_id].end()) {
    STAT_EVENT(proc_id, ICACHE_UNUSEFUL_FETCHES);
    per_core_cnt_unuseful[proc_id].insert(std::make_pair(std::move(line_addr), 1));
  } else {
    unuseful_iter->second++;
  }

  if (per_core_warmed_up[proc_id]) {
    auto it = per_core_cnt_unuseful_aw[proc_id].find(line_addr);
    if (it == per_core_cnt_unuseful_aw[proc_id].end())
      per_core_cnt_unuseful_aw[proc_id].insert(std::make_pair(std::move(line_addr), 1));
    else
      it->second++;

    auto it2 = per_core_sequence_aw[proc_id].find(line_addr);
    if (it2 == per_core_sequence_aw[proc_id].end()) {
      per_core_sequence_aw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_aw[proc_id][line_addr].push_back(std::make_pair('u',cycle_count));
    } else {
      it2->second.push_back(std::make_pair('u',cycle_count));
    }
  } else {
    auto it2 = per_core_sequence_bw[proc_id].find(line_addr);
    if (it2 == per_core_sequence_bw[proc_id].end()) {
      per_core_sequence_bw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_bw[proc_id][line_addr].push_back(std::make_pair('u',cycle_count));
    } else {
      it2->second.push_back(std::make_pair('u',cycle_count));
    }
  }
}

void inc_cnt_useful_signed(uns proc_id, Addr line_addr) {
  auto it = per_core_cnt_useful_signed[proc_id].find(line_addr);
  if (it == per_core_cnt_useful_signed[proc_id].end())
    per_core_cnt_useful_signed[proc_id].insert(std::pair<Addr, int64_t>(line_addr, UDP_USEFUL_THRESHOLD+UDP_WEIGHT_USEFUL));
  else if (it->second + UDP_WEIGHT_USEFUL <= UDP_WEIGHT_POSITIVE_SATURATION)
    it->second += UDP_WEIGHT_USEFUL;

  uns8 useful_value = per_core_warmed_up[proc_id]? 3 : 1;
  auto it2 = per_core_useful_sequence[proc_id].find(line_addr);
  if (it2 == per_core_useful_sequence[proc_id].end()) {
    per_core_useful_sequence[proc_id].insert(std::make_pair(line_addr, std::vector<uns8>()));
    per_core_useful_sequence[proc_id][line_addr].push_back(useful_value);
  } else {
    it2->second.push_back(useful_value);
  }
}

void dec_cnt_useful_signed(uns proc_id, Addr line_addr) {
  auto it = per_core_cnt_useful_signed[proc_id].find(line_addr);
  if (it == per_core_cnt_useful_signed[proc_id].end())
    per_core_cnt_useful_signed[proc_id].insert(std::pair<Addr, int64_t>(line_addr, UDP_USEFUL_THRESHOLD-UDP_WEIGHT_UNUSEFUL));
  else
    it->second -= UDP_WEIGHT_UNUSEFUL;

  uns8 unuseful_value = per_core_warmed_up[proc_id]? 2 : 0;
  auto it2 = per_core_useful_sequence[proc_id].find(line_addr);
  if (it2 == per_core_useful_sequence[proc_id].end()) {
    per_core_useful_sequence[proc_id].insert(std::make_pair(line_addr, std::vector<uns8>()));
    per_core_useful_sequence[proc_id][line_addr].push_back(unuseful_value);
  } else {
    it2->second.push_back(unuseful_value);
  }
}

void inc_cnt_useful_ret(uns proc_id, Addr line_addr) {
  auto useful_iter = per_core_cnt_useful_ret[proc_id].find(line_addr);
  if (useful_iter == per_core_cnt_useful_ret[proc_id].end()) {
    STAT_EVENT(proc_id, USEFUL_CACHELINES_RETIRED);
    per_core_cnt_useful_ret[proc_id].insert(std::pair<Addr, Counter>(line_addr, 1));
  }
  else
    useful_iter->second++;
}

void inc_icache_miss(uns proc_id, Addr line_addr) {

  auto cl_iter = per_core_icache_miss[proc_id].find(line_addr);
  if (cl_iter == per_core_icache_miss[proc_id].end()) {
    STAT_EVENT(proc_id, UNIQUE_MISSED_LINES);
    per_core_icache_miss[proc_id].insert(std::pair<Addr, Counter>(line_addr, 1));
  }
  else
    cl_iter->second++;

  if (per_core_warmed_up[proc_id]) {
    auto it = per_core_icache_miss_aw[proc_id].find(line_addr);
    if (it == per_core_icache_miss_aw[proc_id].end())
      per_core_icache_miss_aw[proc_id].insert(std::pair<Addr, Counter>(line_addr, 1));
    else
      it->second++;

    auto it2 = per_core_sequence_aw[proc_id].find(line_addr);
    if (it2 == per_core_sequence_aw[proc_id].end()) {
      per_core_sequence_aw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_aw[proc_id][line_addr].push_back(std::make_pair('m',cycle_count));
    } else {
      it2->second.push_back(std::make_pair('m',cycle_count));
    }

    per_core_cur_line_delay[proc_id] = cycle_count;
  } else {
    auto it2 = per_core_sequence_bw[proc_id].find(line_addr);
    if (it2 == per_core_sequence_bw[proc_id].end()) {
      per_core_sequence_bw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_bw[proc_id][line_addr].push_back(std::make_pair('m',cycle_count));
    } else {
      it2->second.push_back(std::make_pair('m',cycle_count));
    }
  }

  uns icache_val = per_core_warmed_up[proc_id]? 2 : 0;
  auto it = per_core_icache_sequence[proc_id].find(line_addr);
  if (it == per_core_icache_sequence[proc_id].end()) {
    per_core_icache_sequence[proc_id].insert(std::make_pair(line_addr, std::vector<uns8>()));
    per_core_icache_sequence[proc_id][line_addr].push_back(icache_val);
    if (icache_val == 2) {
      auto it2 = per_core_sequence_bw[proc_id].find(line_addr);
      if (it2 != per_core_sequence_bw[proc_id].end()) {
        STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_SEEN_DURING_WARMUP);
        Counter no_pref = 0;
        Counter unuseful = 0;
        Counter useful = 0;
        auto it3 = it2->second.begin();
        while(it3 != it2->second.end()) {
          if (it3->first == 'p')
            no_pref++;
          else if (it3->first == 'u')
            useful++;
          else if (it3->first == 'U')
            unuseful++;
          ++it3;
        }
        if (no_pref && !unuseful && !useful)
          STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_NO_PREF_DURING_WARMUP);
        if (!no_pref && unuseful && !useful)
          STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_TRAINED_UNUSEFUL_DURING_WARMUP);
        if (!no_pref && !unuseful && useful)
          STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_TRAINED_USEFUL_DURING_WARMUP);
      } else
        STAT_EVENT(proc_id, ICACHE_FIRST_MISS_AFTER_WARMUP_NOT_SEEN_DURING_WARMUP);
    }
  } else {
    it->second.push_back(icache_val);
  }
}

void inc_prefetched_cls(Addr line_addr, uns success) {
  if (success == Mem_Queue_Req_Result::FAILED)
    return;
  Flag on_path = FALSE;
  if (FDIP_BP_CONFIDENCE && per_core_low_confidence_cnt[fdip_proc_id] < FDIP_OFF_PATH_THRESHOLD)
    on_path = TRUE;
  if (!FDIP_BP_CONFIDENCE && !fdip_off_path(fdip_proc_id))
    on_path = TRUE;

  auto cl_iter = per_core_prefetched_cls[fdip_proc_id].find(line_addr);
  if (cl_iter == per_core_prefetched_cls[fdip_proc_id].end()) {
    per_core_prefetched_cls[fdip_proc_id].insert(std::pair<Addr, Counter>(line_addr, 1));
    per_core_prefetched_cls_info[fdip_proc_id].insert(std::make_pair(std::move(line_addr), std::make_pair(std::make_pair(std::move(cycle_count), on_path), std::make_pair(0, 0))));
    DEBUG(fdip_proc_id, "%llx inserted into prefetched_cls at %llu\n", line_addr, cycle_count);
  } else {
    cl_iter->second++;
    auto cl_info_iter = per_core_prefetched_cls_info[fdip_proc_id].find(line_addr);
    ASSERT(fdip_proc_id, cl_info_iter != per_core_prefetched_cls_info[fdip_proc_id].end());
    cl_info_iter->second.first.first = cycle_count;
    cl_info_iter->second.first.second = on_path;
    DEBUG(fdip_proc_id, "%llx updated with cnt %llu in prefetched_cls at cyc %llu\n", line_addr, cl_iter->second, cycle_count);
  }

  if (success == Mem_Queue_Req_Result::SUCCESS_NEW) {
    auto cl_new_iter = per_core_new_prefetched_cls[fdip_proc_id].find(line_addr);
    if (cl_new_iter == per_core_new_prefetched_cls[fdip_proc_id].end())
      per_core_new_prefetched_cls[fdip_proc_id].insert(std::pair<Addr, Counter>(line_addr, 1));
    else
      cl_new_iter->second++;
  }

  if (per_core_warmed_up[fdip_proc_id]) {
    auto it = per_core_prefetched_cls_aw[fdip_proc_id].find(line_addr);
    if (it == per_core_prefetched_cls_aw[fdip_proc_id].end())
      per_core_prefetched_cls_aw[fdip_proc_id].insert(std::pair<Addr, Counter>(line_addr, 1));
    else
      it->second++;

    if (success == Mem_Queue_Req_Result::SUCCESS_NEW) {
      it = per_core_new_prefetched_cls_aw[fdip_proc_id].find(line_addr);
      if (it == per_core_new_prefetched_cls_aw[fdip_proc_id].end())
        per_core_new_prefetched_cls_aw[fdip_proc_id].insert(std::pair<Addr, Counter>(line_addr, 1));
      else
        it->second++;
    }

    auto it2 = per_core_sequence_aw[fdip_proc_id].find(line_addr);
    Counter onoff_cycle_count = fdip_off_path(fdip_proc_id)? -cycle_count : cycle_count;
    if (it2 == per_core_sequence_aw[fdip_proc_id].end()) {
      per_core_sequence_aw[fdip_proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_aw[fdip_proc_id][line_addr].push_back(std::make_pair('P',onoff_cycle_count));
    } else {
      it2->second.push_back(std::make_pair('P',onoff_cycle_count));
    }
  } else {
    auto it2 = per_core_sequence_bw[fdip_proc_id].find(line_addr);
    Counter onoff_cycle_count = fdip_off_path(fdip_proc_id)? -cycle_count : cycle_count;
    if (it2 == per_core_sequence_bw[fdip_proc_id].end()) {
      per_core_sequence_bw[fdip_proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_bw[fdip_proc_id][line_addr].push_back(std::make_pair('P',onoff_cycle_count));
    } else {
      it2->second.push_back(std::make_pair('P',onoff_cycle_count));
    }
  }
}

void not_prefetch(Addr line_addr) {
  if (per_core_warmed_up[fdip_proc_id]) {
    auto it = per_core_sequence_aw[fdip_proc_id].find(line_addr);
    Counter onoff_cycle_count = fdip_off_path(fdip_proc_id)? -cycle_count : cycle_count;
    if (it == per_core_sequence_aw[fdip_proc_id].end()) {
      per_core_sequence_aw[fdip_proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_aw[fdip_proc_id][line_addr].push_back(std::make_pair('p',onoff_cycle_count));
    } else {
      it->second.push_back(std::make_pair('p',onoff_cycle_count));
    }
  } else {
    auto it = per_core_sequence_bw[fdip_proc_id].find(line_addr);
    Counter onoff_cycle_count = fdip_off_path(fdip_proc_id)? -cycle_count : cycle_count;
    if (it == per_core_sequence_bw[fdip_proc_id].end()) {
      per_core_sequence_bw[fdip_proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_bw[fdip_proc_id][line_addr].push_back(std::make_pair('p',onoff_cycle_count));
    } else {
      it->second.push_back(std::make_pair('p',onoff_cycle_count));
    }
  }
}

void inc_off_fetched_cls(Addr line_addr) {
  auto cl_iter = per_core_off_fetched_cls[fdip_proc_id].find(line_addr);
  if (cl_iter == per_core_off_fetched_cls[fdip_proc_id].end()) {
    per_core_off_fetched_cls[fdip_proc_id].insert(std::pair<Addr, Counter>(line_addr, cycle_count));
    DEBUG(fdip_proc_id, "%llx inserted into off_fetched_cls at %llu\n", line_addr, cycle_count);
  } else {
    cl_iter->second = cycle_count;
    DEBUG(fdip_proc_id, "%llx in off_fetched_cls updated at %llu\n", line_addr, cycle_count);
  }
}

void probe_prefetched_cls(Addr line_addr) {
  auto cl_iter = per_core_prefetched_cls_info[fdip_proc_id].find(line_addr);
  if (cl_iter != per_core_prefetched_cls_info[fdip_proc_id].end())
    cl_iter->second.first.first = cycle_count;
}

void evict_prefetched_cls(uns proc_id, Addr line_addr, Flag by_fdip) {
  auto cl_iter = per_core_prefetched_cls_info[proc_id].find(line_addr);
  if (cl_iter != per_core_prefetched_cls_info[proc_id].end()) {
    if (by_fdip) {
      cl_iter->second.second.first = 0;
      cl_iter->second.second.second = cycle_count;
    } else {
      cl_iter->second.second.first = cycle_count;
      cl_iter->second.second.second = 0;
    }
  }
}

uns get_miss_reason(uns proc_id, Addr line_addr) {
  auto cl_iter = per_core_prefetched_cls_info[proc_id].find(line_addr);
  if (cl_iter == per_core_prefetched_cls_info[proc_id].end()) {
    auto tmp_iter = per_core_prefetched_cls[proc_id].find(line_addr);
    DEBUG(proc_id, "%llx misses due to 'not prefetched ever'\n", line_addr);
    ASSERT(proc_id, tmp_iter == per_core_prefetched_cls[proc_id].end());
    return Imiss_Reason::IMISS_NOT_PREFETCHED;
  }
  if (cl_iter->second.first.first < per_core_last_recover_cycle[proc_id]) {
    DEBUG(proc_id, "%llx misses due to 'not prefetched after last recover cycle'\n", line_addr);
    return Imiss_Reason::IMISS_NOT_PREFETCHED;
  }

  if (cl_iter->second.first.first >= per_core_last_recover_cycle[proc_id]) {
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

uns get_last_miss_reason(uns proc_id) {
  DEBUG(proc_id, "get last miss reason %u\n", per_core_last_imiss_reason[proc_id]);
  return per_core_last_imiss_reason[proc_id];
}

void set_last_miss_reason(uns proc_id, uns reason) {
  per_core_last_imiss_reason[proc_id] = reason;
}

uint64_t get_fdip_ftq_occupancy_ops(uns proc_id) {
  return (uint64_t)per_core_fdip_ftq_occupancy_ops[proc_id]/cycle_count;
}

uint64_t get_fdip_ftq_occupancy(uns proc_id) {
  return (uint64_t)per_core_fdip_ftq_occupancy_blocks[proc_id]/cycle_count;
}

static inline void determine_usefulness_by_inf_hash(Addr line_addr, Flag* emit_new_prefetch, Op* op) {
  uint64_t hashed_line_addr = line_addr;
  if (FDIP_GHIST_HASHING)
    hashed_line_addr = fdip_hash_addr_ghist(line_addr, g_bp_data->global_hist);

  if (FDIP_BP_CONFIDENCE && per_core_low_confidence_cnt[fdip_proc_id] < FDIP_OFF_PATH_THRESHOLD) {
    DEBUG(fdip_proc_id, "emit_new_prefetch low_confidence_cnt: %d, fdip_off_path: %d\n", per_core_low_confidence_cnt[fdip_proc_id], fdip_off_path(fdip_proc_id));
    *emit_new_prefetch = TRUE;
  } else {
    switch(FDIP_UTILITY_PREF_POLICY) {
      case Utility_Pref_Policy::PREF_CONV_FROM_USEFUL_SET: {
        std::unordered_map<Addr, std::pair<Counter, Flag>>* cnt_useful = &per_core_cnt_useful[fdip_proc_id];
        auto iter = cnt_useful->find(hashed_line_addr);
        if (iter == cnt_useful->end()) {
          *emit_new_prefetch = FALSE;
	      }
        else {
          *emit_new_prefetch = TRUE;
	      }
        break;
      }
      case Utility_Pref_Policy::PREF_OPT_FROM_UNUSEFUL_SET: {
        std::unordered_map<Addr, Counter>* cnt_unuseful = &per_core_cnt_unuseful[fdip_proc_id];
        auto iter = cnt_unuseful->find(hashed_line_addr);
        if (iter == cnt_unuseful->end())
          *emit_new_prefetch = TRUE;
        else {
          *emit_new_prefetch = FALSE;
	      }
        break;
      }
      case Utility_Pref_Policy::PREF_CONV_FROM_THROTTLE_CNT: {
        std::unordered_map<Addr, int32_t>* cnt = &per_core_cnt_useful_signed[fdip_proc_id];
        auto iter = cnt->find(hashed_line_addr);
        if (iter != cnt->end() && iter->second > UDP_USEFUL_THRESHOLD)
          *emit_new_prefetch = TRUE;
        else {
          *emit_new_prefetch = FALSE;
	      }
        break;
      }
      case Utility_Pref_Policy::PREF_OPT_FROM_THROTTLE_CNT: {
        std::unordered_map<Addr, int32_t>* cnt = &per_core_cnt_useful_signed[fdip_proc_id];
        auto iter = cnt->find(hashed_line_addr);
        if (iter != cnt->end() && iter->second < UDP_USEFUL_THRESHOLD) {
          *emit_new_prefetch = FALSE;
	      }
	      else
          *emit_new_prefetch = TRUE;
        break;
      }
    }
  }
  if (*emit_new_prefetch) {
    DEBUG(fdip_proc_id, "emit a new prefetch for cl 0x%llx\n", line_addr);
  } else {
    DEBUG(fdip_proc_id, "do not emit a new prefetch for cl 0x%llx\n", line_addr);
    per_core_last_cl_unuseful[fdip_proc_id] = line_addr;
  }
}

static inline void determine_usefulness_by_utility_cache(Addr line_addr, Flag* emit_new_prefetch, Op* op) {
  Addr uc_line_addr = 0;
  uint64_t hashed_line_addr = line_addr;
  if (FDIP_GHIST_HASHING)
    hashed_line_addr = fdip_hash_addr_ghist(line_addr, g_bp_data->global_hist);

  if (FDIP_BP_CONFIDENCE && per_core_low_confidence_cnt[fdip_proc_id] < FDIP_OFF_PATH_THRESHOLD) {
    DEBUG(fdip_proc_id, "emit_new_prefetch low_confidence_cnt: %d, fdip_off_path: %d\n", per_core_low_confidence_cnt[fdip_proc_id], fdip_off_path(fdip_proc_id));
    *emit_new_prefetch = TRUE;
  } else {
    switch(FDIP_UTILITY_PREF_POLICY) {
      case Utility_Pref_Policy::PREF_CONV_FROM_USEFUL_SET: {
        void* useful = (void*)cache_access(&per_core_fdip_uc[fdip_proc_id], hashed_line_addr, &uc_line_addr, TRUE);
        if (useful) {
          STAT_EVENT(fdip_proc_id, FDIP_UC_HIT);
          *emit_new_prefetch = TRUE;
	      }
        else {
          STAT_EVENT(fdip_proc_id, FDIP_UC_MISS);
          *emit_new_prefetch = FALSE;
	      }
        break;
      }
      case Utility_Pref_Policy::PREF_OPT_FROM_UNUSEFUL_SET: {
        void* unuseful = (void*)cache_access(&per_core_fdip_uc_unuseful[fdip_proc_id], hashed_line_addr, &uc_line_addr, TRUE);
        if (unuseful) {
          STAT_EVENT(fdip_proc_id, FDIP_UC_HIT);
          *emit_new_prefetch = FALSE;
	      }
        else {
          STAT_EVENT(fdip_proc_id, FDIP_UC_MISS);
          *emit_new_prefetch = TRUE;
	      }
        break;
      }
      case Utility_Pref_Policy::PREF_CONV_FROM_THROTTLE_CNT: {
        int32_t* useful = (int32_t*)cache_access(&per_core_fdip_uc_signed[fdip_proc_id], hashed_line_addr, &uc_line_addr, TRUE);
        if (useful) {
          STAT_EVENT(fdip_proc_id, FDIP_UC_HIT);
          if (*useful > UDP_USEFUL_THRESHOLD)
            *emit_new_prefetch = TRUE;
          else
            *emit_new_prefetch = FALSE;
        }
        else {
          STAT_EVENT(fdip_proc_id, FDIP_UC_MISS);
          *emit_new_prefetch = FALSE;
	      }
        break;
      }
      case Utility_Pref_Policy::PREF_OPT_FROM_THROTTLE_CNT: {
        int32_t* useful = (int32_t*)cache_access(&per_core_fdip_uc_signed[fdip_proc_id], hashed_line_addr, &uc_line_addr, TRUE);
        if (useful) {
          STAT_EVENT(fdip_proc_id, FDIP_UC_HIT);
          if (*useful < UDP_USEFUL_THRESHOLD) {
            *emit_new_prefetch = FALSE;
          } else
            *emit_new_prefetch = TRUE;
	      }
	      else {
          STAT_EVENT(fdip_proc_id, FDIP_UC_MISS);
          *emit_new_prefetch = TRUE;
        }
        break;
      }
    }
  }
  if (*emit_new_prefetch) {
    DEBUG(fdip_proc_id, "emit a new prefetch for cl 0x%llx\n", line_addr);
  } else {
    DEBUG(fdip_proc_id, "do not emit a new prefetch for cl 0x%llx\n", line_addr);
    per_core_last_cl_unuseful[fdip_proc_id] = line_addr;
  }
}

static inline void* bloom_lookup(uns proc_id, Addr uc_line_addr) {
  Bloom_Filter* bf = &per_core_bloom_filter[proc_id];
  Addr line_addr = uc_line_addr >> 6;
  return (void*)(bf->bloom->contains(line_addr) || bf->bloom2->contains(line_addr >> 1) || bf->bloom4->contains(line_addr >> 2));
}

static inline void insert1(Addr line_addr) {
  Bloom_Filter* bf = &per_core_bloom_filter[fdip_proc_id];
  STAT_EVENT(fdip_proc_id, FDIP_BLOOM_1INSERT);
  if (!bf->bloom2->contains(line_addr >> 1) && !bf->bloom4->contains(line_addr >> 2)) {
    if (bf->cnt_insert_bloom >= FDIP_BLOOM_ENTRIES && (bf->new_prefs > FDIP_BLOOM_CLEAR_CYC_PERIOD * 0.01) && ((float)bf->cnt_unuseful/(float)bf->new_prefs > FDIP_BLOOM_CLEAR_UNUSEFUL_RATIO)) {
      bf->bloom->clear();
      bf->cnt_insert_bloom = 0;
      STAT_EVENT(fdip_proc_id, FDIP_BLOOM_CLEAR1);
    }
    bf->bloom->insert(line_addr);
    bf->cnt_insert_bloom++;
  }

}

static inline void insert2(Addr line_addr) {
  Bloom_Filter* bf = &per_core_bloom_filter[fdip_proc_id];
  if((line_addr & 1) == 0) { //2CL aligned
    if (!bf->bloom4->contains(line_addr >> 2) && !bf->bloom2->contains(line_addr >> 1)) {
      if (bf->cnt_insert_bloom2 >= FDIP_BLOOM2_ENTRIES && (bf->new_prefs > FDIP_BLOOM_CLEAR_CYC_PERIOD * 0.01) && ((float)bf->cnt_unuseful/(float)bf->new_prefs > FDIP_BLOOM_CLEAR_UNUSEFUL_RATIO)) {
        bf->bloom2->clear();
        bf->cnt_insert_bloom2 = 0;
        STAT_EVENT(fdip_proc_id, FDIP_BLOOM_CLEAR2);
      }
      bf->bloom2->insert(line_addr >> 1);
      bf->cnt_insert_bloom2++;
    }
    STAT_EVENT(fdip_proc_id, FDIP_BLOOM_2INSERT);
  }
  else {
    insert1(line_addr);
    insert1(line_addr + 1);
  }
}

static inline void insert3(Addr line_addr) {
  if((line_addr & 1) == 0) { //2CL aligned
    insert2(line_addr);
    insert1(line_addr + 2);
  }
  else {
    insert1(line_addr + 1);
    insert2(line_addr);
  }
}

static inline void insert4(Addr line_addr) {
  Bloom_Filter* bf = &per_core_bloom_filter[fdip_proc_id];
  ASSERT(0, (line_addr & 3) == 0);
  if (bf->cnt_insert_bloom4 >= FDIP_BLOOM4_ENTRIES && (bf->new_prefs > FDIP_BLOOM_CLEAR_CYC_PERIOD * 0.01) && ((float)bf->cnt_unuseful/(float)bf->new_prefs > FDIP_BLOOM_CLEAR_UNUSEFUL_RATIO)) {
    bf->bloom4->clear();
    bf->cnt_insert_bloom4 = 0;
    STAT_EVENT(fdip_proc_id, FDIP_BLOOM_CLEAR4);
  }
  bf->bloom4->insert(line_addr >> 2);
  bf->cnt_insert_bloom4++;
  STAT_EVENT(fdip_proc_id, FDIP_BLOOM_4INSERT);
}

static inline void insert_remaining(uint32_t inserted) {
  Bloom_Filter* bf = &per_core_bloom_filter[fdip_proc_id];
  while (inserted + 4 <= bf->last_prefetch_candidate_counter) {
    insert4(bf->last_prefetch_candidate + inserted);
    inserted += 4;
  }
  if (inserted + 3 == bf->last_prefetch_candidate_counter)
    insert3(bf->last_prefetch_candidate + inserted);
  if (inserted + 2 == bf->last_prefetch_candidate_counter)
    insert2(bf->last_prefetch_candidate + inserted);
  if (inserted + 1 == bf->last_prefetch_candidate_counter)
    insert1(bf->last_prefetch_candidate + inserted);

}

static inline void bloom_insert() {
  Bloom_Filter* bf = &per_core_bloom_filter[fdip_proc_id];
  uint32_t inserted = 0;
  if (bf->last_prefetch_candidate_counter < 4) {
    insert_remaining(inserted);
    return;
  }
  if ((bf->last_prefetch_candidate & 3) == 0) {
    //4cl aligned
    insert_remaining(inserted);
  }
  else if ((bf->last_prefetch_candidate & 3) == 2) {
    //2cl algned
    insert2(bf->last_prefetch_candidate);
    inserted += 2;
    insert_remaining(inserted);
  }
  else if ((bf->last_prefetch_candidate & 3) == 1) {
    //cl aligned
    //cl aligned
    insert3(bf->last_prefetch_candidate);
    inserted += 3;
    insert_remaining(inserted);
  }
  else if ((bf->last_prefetch_candidate & 3) == 3) {
    //cl aligned
    insert1(bf->last_prefetch_candidate);
    inserted +=1;
    insert_remaining(inserted);
  }
  return;
}

static inline void detect_stream(uns proc_id, Addr uc_line_addr) {
  Bloom_Filter* bf = &per_core_bloom_filter[proc_id];
  Addr line_addr = uc_line_addr >> 6;

  if(bf->last_prefetch_candidate_counter == 0) {
    bf->last_prefetch_candidate_counter++;
    bf->last_prefetch_candidate = line_addr;
    return;
  }
  STAT_EVENT(fdip_proc_id, FDIP_BLOOM_INSERTED);

  if (line_addr == bf->last_prefetch_candidate + bf->last_prefetch_candidate_counter) {
    bf->last_prefetch_candidate_counter++;
  }
  else {
    bloom_insert();
    bf->last_prefetch_candidate_counter = 1;
    bf->last_prefetch_candidate = line_addr;
  }
}

static inline void determine_usefulness_by_bloom_filter(Addr line_addr, Flag* emit_new_prefetch, Op* op) {
  uint64_t hashed_line_addr = line_addr;
  if (FDIP_GHIST_HASHING)
    hashed_line_addr = fdip_hash_addr_ghist(line_addr, g_bp_data->global_hist);

  if (FDIP_BP_CONFIDENCE && per_core_low_confidence_cnt[fdip_proc_id] < FDIP_OFF_PATH_THRESHOLD) {
    DEBUG(fdip_proc_id, "emit_new_prefetch low_confidence_cnt: %d, fdip_off_path: %d\n", per_core_low_confidence_cnt[fdip_proc_id], fdip_off_path(fdip_proc_id));
    *emit_new_prefetch = TRUE;
  } else {
    void* useful = bloom_lookup(fdip_proc_id, hashed_line_addr);
    if (useful) {
      STAT_EVENT(fdip_proc_id, FDIP_BLOOM_HIT);
      *emit_new_prefetch = TRUE;
      DEBUG(fdip_proc_id, "bloom : emit a new prefetch for cl 0x%llx off_path: %u", line_addr, fdip_off_path(fdip_proc_id) ? 1 : 0);
    } else {
      STAT_EVENT(fdip_proc_id, FDIP_BLOOM_MISS);
      DEBUG(fdip_proc_id, "bloom : do not emit a new prefetch for cl 0x%llx", line_addr);
      *emit_new_prefetch = FALSE;
      per_core_last_cl_unuseful[fdip_proc_id] = line_addr;
    }
  }
}

Flag determine_usefulness(Addr line_addr, Op* op) {
  Flag emit_new_prefetch = FALSE;
  if (FDIP_PERFECT_PREFETCH) {
    if (!fdip_off_path(fdip_proc_id))
      emit_new_prefetch = TRUE;
    else {
      emit_new_prefetch = buf_map_find(line_addr);
      if (emit_new_prefetch)
        STAT_EVENT(fdip_proc_id, FDIP_MEM_BUF_FOUND);
      else
        STAT_EVENT(fdip_proc_id, FDIP_MEM_BUF_MISS);
    }
  //} else if (!per_core_last_cl_unuseful[fdip_proc_id] && per_core_last_cl_unuseful[fdip_proc_id] != line_addr) {
  } else if (per_core_last_cl_unuseful[fdip_proc_id] != line_addr) {
    if (FDIP_UTILITY_HASH_ENABLE)
      determine_usefulness_by_inf_hash(line_addr, &emit_new_prefetch, op);
    else if (FDIP_UC_SIZE)
      determine_usefulness_by_utility_cache(line_addr, &emit_new_prefetch, op);
    else if (FDIP_BLOOM_FILTER)
      determine_usefulness_by_bloom_filter(line_addr, &emit_new_prefetch, op);
  }
  return emit_new_prefetch;
}

void update_useful_lines(uns proc_id, Op* op) {
  if (!FDIP_UTILITY_HASH_ENABLE && !FDIP_UC_SIZE && !FDIP_BLOOM_FILTER)
    return;

  Addr fetch_addr = per_core_last_bbl_start_addr[proc_id];
  Addr useful_cl_addr = per_core_last_bbl_start_addr[proc_id] & ~0x3F;
  Addr last_useful_cl_addr = 0;
  while (fetch_addr <= op->inst_info->addr) {
    if (last_useful_cl_addr != useful_cl_addr) {
      if (FDIP_UTILITY_HASH_ENABLE)
        inc_cnt_useful_ret(proc_id, useful_cl_addr);
      if (FDIP_UC_SIZE) {
        Addr uc_line_addr = 0;
        Addr repl_uc_line_addr = 0;
        void* cnt = (void*)cache_access(&per_core_fdip_uc[proc_id], useful_cl_addr, &uc_line_addr, TRUE);
        if (!cnt)
          cache_insert_replpos(&per_core_fdip_uc[proc_id], fdip_proc_id, useful_cl_addr, &uc_line_addr,
              &repl_uc_line_addr, (Cache_Insert_Repl)FDIP_UC_INSERT_REPLPOL, FALSE);
        UNUSED(uc_line_addr);
        if (repl_uc_line_addr)
          STAT_EVENT(proc_id, FDIP_UC_REPLACEMENT);
      }
      if (FDIP_BLOOM_FILTER) {
        void* cnt = bloom_lookup(proc_id, useful_cl_addr);
        if (!cnt)
          detect_stream(proc_id, useful_cl_addr);
      }
    }

    last_useful_cl_addr = useful_cl_addr;
    useful_cl_addr = ++fetch_addr & ~0x3F;
  }
  per_core_last_bbl_start_addr[proc_id] = op->oracle_info.npc;
  DEBUG(proc_id, "last_bbl_start_addr: %llx\n", per_core_last_bbl_start_addr[proc_id]);
}

void update_unuseful_lines_uc(uns proc_id, Addr line_addr) {
  if (!FDIP_UC_SIZE)
    return;
  Addr uc_line_addr = 0;
  Addr repl_uc_line_addr = 0;
  void* cnt = (void*)cache_access(&per_core_fdip_uc_unuseful[proc_id], line_addr, &uc_line_addr, TRUE);
  if (!cnt)
    cache_insert_replpos(&per_core_fdip_uc_unuseful[proc_id], fdip_proc_id, line_addr, &uc_line_addr,
        &repl_uc_line_addr, (Cache_Insert_Repl)FDIP_UC_INSERT_REPLPOL, FALSE);
  UNUSED(uc_line_addr);
  if (repl_uc_line_addr)
    STAT_EVENT(proc_id, FDIP_UC_REPLACEMENT);
}

void update_useful_lines_uc(uns proc_id, Addr line_addr) {
  if (!FDIP_UC_SIZE)
    return;
  Addr uc_line_addr = 0;
  Addr repl_uc_line_addr = 0;
  void* cnt = (void*)cache_access(&per_core_fdip_uc[proc_id], line_addr, &uc_line_addr, TRUE);
  if (!cnt)
    cache_insert_replpos(&per_core_fdip_uc[proc_id], fdip_proc_id, line_addr, &uc_line_addr,
        &repl_uc_line_addr, (Cache_Insert_Repl)FDIP_UC_INSERT_REPLPOL, FALSE);
  UNUSED(uc_line_addr);
  if (repl_uc_line_addr)
    STAT_EVENT(proc_id, FDIP_UC_REPLACEMENT);
}

void update_useful_lines_bloom_filter(uns proc_id, Addr line_addr) {
  if (!FDIP_BLOOM_FILTER)
    return;
  void* cnt = bloom_lookup(proc_id, line_addr);
  if (!cnt)
    detect_stream(proc_id, line_addr);
}

void inc_utility_info(uns proc_id, Flag useful) {
  if (FDIP_ADJUSTABLE_FTQ != 1 && FDIP_ADJUSTABLE_FTQ != 3)
    return;
  if (useful)
    per_core_utility_timeliness_info[proc_id].useful_prefetches++;
  else
    per_core_utility_timeliness_info[proc_id].unuseful_prefetches++;
}

void inc_timeliness_info(uns proc_id, Flag mshr_hit) {
  if (FDIP_ADJUSTABLE_FTQ != 2 && FDIP_ADJUSTABLE_FTQ != 3)
    return;
  if (mshr_hit)
    per_core_utility_timeliness_info[proc_id].mshr_prefetch_hits++;
  else
    per_core_utility_timeliness_info[proc_id].icache_prefetch_hits++;
}

void fdip_inc_cnt_btb_miss(uns proc_id) {
  per_core_cnt_btb_miss[proc_id]++;
}

Flag fdip_search_pref_candidate(Addr addr) {
  if (!FDIP_UTILITY_HASH_ENABLE && !FDIP_UC_SIZE && !FDIP_BLOOM_FILTER)
    return TRUE;
  auto seniority_ftq = &per_core_seniority_ftq[fdip_proc_id];
  DEBUG(fdip_proc_id, "Search a pref candidate for %llx. seniority_ftq.size(): %ld\n", addr, seniority_ftq->size());
  for (auto it = seniority_ftq->begin();
      it != seniority_ftq->end(); ++it) {
    if (std::get<0>(*it) == addr && (FDIP_UTILITY_ONLY_TRAIN_OFF_PATH ? std::get<2>(*it) == FALSE :TRUE)) {
      DEBUG(fdip_proc_id, "Hit seniority-FTQ for addr: %llx cyc: %lld, on-path: %u\n", std::get<0>(*it), std::get<1>(*it), std::get<2>(*it));
      STAT_EVENT(fdip_proc_id, FDIP_SENIORITY_FTQ_HIT);
      return TRUE;
    }
  }
  STAT_EVENT(fdip_proc_id, FDIP_SENIORITY_FTQ_MISS);
  return FALSE;
}

void insert_pref_candidate_to_seniority_ftq(Addr line_addr) {
  if (!FDIP_UTILITY_HASH_ENABLE && !FDIP_UC_SIZE && !FDIP_BLOOM_FILTER)
    return;
  uint64_t hashed_line_addr = line_addr;
  if (FDIP_GHIST_HASHING)
    hashed_line_addr = fdip_hash_addr_ghist(line_addr, g_bp_data->global_hist);
  Flag on_path = FDIP_BP_CONFIDENCE && (per_core_low_confidence_cnt[fdip_proc_id] < FDIP_OFF_PATH_THRESHOLD);
  per_core_seniority_ftq[fdip_proc_id].push_back(std::make_tuple(hashed_line_addr, cycle_count, on_path));
  DEBUG(fdip_proc_id, "Insert %llx (hashed %lx) to seniority FTQ at cyc %llu seniority_ftq.size() : %ld\n", line_addr, hashed_line_addr, cycle_count, per_core_seniority_ftq[fdip_proc_id].size());
}

void clear_old_seniority_ftq() {
  if (!FDIP_UTILITY_HASH_ENABLE && !FDIP_UC_SIZE && !FDIP_BLOOM_FILTER)
    return;
  auto seniority_ftq = &per_core_seniority_ftq[fdip_proc_id];
  Counter cnt_old = 0;
  for (auto it = seniority_ftq->begin();
       it != seniority_ftq->end(); ++it) {
    if (cycle_count <= FDIP_SENIORITY_FTQ_HOLD_CYC)
      break;
    if ((cycle_count > FDIP_SENIORITY_FTQ_HOLD_CYC) && (std::get<1>(*it) >= cycle_count - FDIP_SENIORITY_FTQ_HOLD_CYC))
      break;
    cnt_old++;
  }
  DEBUG(fdip_proc_id, "Clear %llu entries among %ld at cyc %llu\n", cnt_old, seniority_ftq->size(), cycle_count);
  seniority_ftq->erase(seniority_ftq->begin(), seniority_ftq->begin() + cnt_old);
}

void inc_useful_lines_uc(uns proc_id, Addr line_addr) {
  if (!FDIP_UC_SIZE)
    return;
  Addr uc_line_addr = 0;
  Addr repl_uc_line_addr = 0;
  int32_t* cnt = (int32_t*)cache_access(&per_core_fdip_uc_signed[proc_id], line_addr, &uc_line_addr, TRUE);
  if (!cnt) {
    cnt = (int32_t*)cache_insert_replpos(&per_core_fdip_uc_signed[proc_id], fdip_proc_id, line_addr, &uc_line_addr,
        &repl_uc_line_addr, (Cache_Insert_Repl)FDIP_UC_INSERT_REPLPOL, FALSE);
    *cnt = UDP_USEFUL_THRESHOLD+UDP_WEIGHT_USEFUL;
    DEBUG(proc_id, "Insert uc cnt with value %d\n", *cnt);
  }
  else if (*cnt + UDP_WEIGHT_USEFUL <= UDP_WEIGHT_POSITIVE_SATURATION) {
    DEBUG(proc_id, "Increment uc cnt from %d to ", *cnt);
    *cnt += UDP_WEIGHT_USEFUL;
    DEBUG(proc_id, "%d\n", *cnt);
  }
  UNUSED(uc_line_addr);
  if (repl_uc_line_addr)
    STAT_EVENT(proc_id, FDIP_UC_REPLACEMENT);
}

void dec_useful_lines_uc(uns proc_id, Addr line_addr) {
  if (!FDIP_UC_SIZE)
    return;
  Addr uc_line_addr = 0;
  Addr repl_uc_line_addr = 0;
  int32_t* cnt = (int32_t*)cache_access(&per_core_fdip_uc_signed[proc_id], line_addr, &uc_line_addr, TRUE);
  if (!cnt) {
    cnt = (int32_t*)cache_insert_replpos(&per_core_fdip_uc_signed[proc_id], fdip_proc_id, line_addr, &uc_line_addr,
        &repl_uc_line_addr, (Cache_Insert_Repl)FDIP_UC_INSERT_REPLPOL, FALSE);
    *cnt = UDP_USEFUL_THRESHOLD-UDP_WEIGHT_UNUSEFUL;
    DEBUG(proc_id, "Insert uc cnt with value %d\n", *cnt);
  }
  else {
    DEBUG(proc_id, "Decrement uc cnt from %d to ", *cnt);
    *cnt -= UDP_WEIGHT_UNUSEFUL;
    DEBUG(proc_id, "%d\n", *cnt);
  }
  UNUSED(uc_line_addr);
  if (repl_uc_line_addr)
    STAT_EVENT(proc_id, FDIP_UC_REPLACEMENT);
}

void assert_fdip_break_reason(uns proc_id, Addr line_addr) {
  if (!FDIP_UTILITY_HASH_ENABLE || (FDIP_UTILITY_PREF_POLICY != PREF_CONV_FROM_USEFUL_SET) || (FULL_WARMUP && !warmup_dump_done[proc_id]) || !FDIP_BP_PERFECT_CONFIDENCE)
    return;
  auto useful_iter = per_core_cnt_useful[proc_id].find(line_addr);
  if (useful_iter != per_core_cnt_useful[proc_id].end() && !useful_iter->second.second) { // learned from a seniority-FTQ hit
    ASSERT(proc_id, per_core_last_break_reason[proc_id] == BR_FULL_MEM_REQ_BUF);
  }
}

void inc_icache_hit(uns proc_id, Addr line_addr) {
  auto cl_iter = per_core_icache_hit[proc_id].find(line_addr);
  if (cl_iter == per_core_icache_hit[proc_id].end()) {
    STAT_EVENT(proc_id, UNIQUE_HIT_LINES);
    per_core_icache_hit[proc_id].insert(std::pair<Addr, Counter>(line_addr, 1));
  }
  else
    cl_iter->second++;

  if (per_core_warmed_up[proc_id]) {
    auto it = per_core_icache_hit_aw[proc_id].find(line_addr);
    if (it == per_core_icache_hit_aw[proc_id].end())
      per_core_icache_hit_aw[proc_id].insert(std::pair<Addr, Counter>(line_addr, 1));
    else
      it->second++;

    auto it2 = per_core_sequence_aw[proc_id].find(line_addr);
    if (it2 == per_core_sequence_aw[proc_id].end()) {
      per_core_sequence_aw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_aw[proc_id][line_addr].push_back(std::make_pair('h',cycle_count));
    } else {
      it2->second.push_back(std::make_pair('h',cycle_count));
    }

    if (per_core_cur_line_delay[proc_id]) {
      auto it3 = per_core_per_line_delay_aw[proc_id].find(line_addr);
      if (it3 == per_core_per_line_delay_aw[proc_id].end()) {
        per_core_per_line_delay_aw[proc_id].insert(std::make_pair(line_addr, cycle_count - per_core_cur_line_delay[proc_id]));
      } else {
        it3->second += cycle_count - per_core_cur_line_delay[proc_id];
      }
    }
    per_core_cur_line_delay[proc_id] = 0;
  } else {
    auto it2 = per_core_sequence_bw[proc_id].find(line_addr);
    if (it2 == per_core_sequence_bw[proc_id].end()) {
      per_core_sequence_bw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_bw[proc_id][line_addr].push_back(std::make_pair('h',cycle_count));
    } else {
      it2->second.push_back(std::make_pair('h',cycle_count));
    }
  }

  uns icache_val = per_core_warmed_up[proc_id]? 3 : 1;
  auto it = per_core_icache_sequence[proc_id].find(line_addr);
  if (it == per_core_icache_sequence[proc_id].end()) {
    per_core_icache_sequence[proc_id].insert(std::make_pair(line_addr, std::vector<uns8>()));
    per_core_icache_sequence[proc_id][line_addr].push_back(icache_val);
  } else {
    it->second.push_back(icache_val);
  }
}

void inc_br_conf_counters(int conf){
  switch(conf){
    case 0:
      per_core_conf_info[fdip_proc_id].num_conf_0_branches += 1;
      break;
    case 1:
      per_core_conf_info[fdip_proc_id].num_conf_1_branches += 1;
      break;
    case 2:
      per_core_conf_info[fdip_proc_id].num_conf_2_branches += 1;
      break;
    case 3:
      per_core_conf_info[fdip_proc_id].num_conf_3_branches += 1;
      break;
    default:
      DEBUG(fdip_proc_id, "inc_br_conf_counters: invalid conf value\n");
      break;
  }
}

void inc_cf_type_counters(Cf_Type cf_type){
  switch(cf_type){
    case NOT_CF:
      DEBUG(fdip_proc_id, "inc_cf_type_counters: instruction is not a cf inst.\n");
      break;
    case CF_BR:
      per_core_conf_info[fdip_proc_id].num_cf_br += 1;
      break;
    case CF_CBR:
      per_core_conf_info[fdip_proc_id].num_cf_cbr += 1;
      break;
    case CF_CALL:
      per_core_conf_info[fdip_proc_id].num_cf_call += 1;
      break;
    case CF_IBR:
      per_core_conf_info[fdip_proc_id].num_cf_ibr += 1;
      break;
    case CF_ICALL:
      per_core_conf_info[fdip_proc_id].num_cf_icall += 1;
      break;
    case CF_ICO:
      per_core_conf_info[fdip_proc_id].num_cf_ico += 1;
      break;
    case CF_RET:
      per_core_conf_info[fdip_proc_id].num_cf_ret += 1;
      break;
    case CF_SYS:
      per_core_conf_info[fdip_proc_id].num_cf_sys += 1;
      break;
    default:
      DEBUG(fdip_proc_id, "inc_cf_type_counters: instruction is not a valid cf inst.\n");
      break;
  }
}

void inc_low_conf_ctr_(Op * op){
  per_core_low_confidence_cnt[fdip_proc_id] += 3 - op->bp_confidence + (double)FDIP_BTB_MISS_RATE_WEIGHT*per_core_btb_miss_rate[fdip_proc_id]; //3 is highest bp_confidence
  per_core_cf_op_distance[fdip_proc_id] = 0.0;

  if(op->oracle_info.btb_miss){
    per_core_conf_info[fdip_proc_id].num_BTB_misses += 1;
  }
  inc_br_conf_counters(op->bp_confidence);
  inc_cf_type_counters(op->table_info->cf_type);
  DEBUG(fdip_proc_id, "op->bp_confidence: %d, low_confidence_cnt: %d, off_path: %d\n", op->bp_confidence, per_core_low_confidence_cnt[fdip_proc_id], op->off_path? 1:0);
}

// default conf mechanism
void default_conf_update(Op * op){
  if (!FDIP_BP_CONFIDENCE)
    return;
  DEBUG(fdip_proc_id, "default_conf_update\n");
  //prevent wrap around
  if(per_core_low_confidence_cnt[fdip_proc_id] != ~0U){
    if (op->table_info->cf_type) {
      per_core_low_confidence_cnt[fdip_proc_id] += 3 - op->bp_confidence + (double)FDIP_BTB_MISS_RATE_WEIGHT*per_core_btb_miss_rate[fdip_proc_id]; //3 is highest bp_confidence
      per_core_cf_op_distance[fdip_proc_id] = 0.0;
      //log stats
      if(op->oracle_info.btb_miss){
        per_core_conf_info[fdip_proc_id].num_BTB_misses += 1;
      }
      inc_br_conf_counters(op->bp_confidence);
      inc_cf_type_counters(op->table_info->cf_type);
      DEBUG(fdip_proc_id, "op->bp_confidence: %d, low_confidence_cnt: %d, off_path: %d\n", op->bp_confidence, per_core_low_confidence_cnt[fdip_proc_id], op->off_path? 1:0);
    }
    else if (per_core_cf_op_distance[fdip_proc_id] >= FDIP_OFF_PATH_THRESHOLD) {
      per_core_low_confidence_cnt[fdip_proc_id] += FDIP_OFF_PATH_CONF_INC + (double)FDIP_BTB_MISS_RATE_WEIGHT*per_core_btb_miss_rate[fdip_proc_id];
      per_core_cf_op_distance[fdip_proc_id] = 0.0;
      per_core_conf_info[fdip_proc_id].num_op_dist_incs += 1;
    } else{
      per_core_cf_op_distance[fdip_proc_id] += (1.0+(double)FDIP_BTB_MISS_RATE_WEIGHT*per_core_btb_miss_rate[fdip_proc_id]);
    }
  }
}
//if btb miss and high enough bp confidence set confidence to off path
void btb_miss_bp_taken_conf_update(Op * op){
  if (!FDIP_BP_CONFIDENCE)
    return;
  if(per_core_low_confidence_cnt[fdip_proc_id] != ~0U){
    if (op->table_info->cf_type) {
      if(op->oracle_info.btb_miss && op->oracle_info.pred_orig == TAKEN && (op->bp_confidence >= FDIP_BTB_MISS_BP_TAKEN_CONF_THRESHOLD)){
        per_core_low_confidence_cnt[fdip_proc_id] = ~0U;
        if(!per_core_conf_info[fdip_proc_id].fdip_on_conf_off_event){
          switch(op->bp_confidence){
            case 0:
              per_core_conf_info[fdip_proc_id].conf_off_path_reason = REASON_BTB_MISS_BP_TAKEN_CONF_0;
              break;
            case 1:
              per_core_conf_info[fdip_proc_id].conf_off_path_reason = REASON_BTB_MISS_BP_TAKEN_CONF_1;
              break;
            case 2:
              per_core_conf_info[fdip_proc_id].conf_off_path_reason = REASON_BTB_MISS_BP_TAKEN_CONF_2;
              break;
            case 3:
              per_core_conf_info[fdip_proc_id].conf_off_path_reason = REASON_BTB_MISS_BP_TAKEN_CONF_3;
              break;
            default:
              break;
          }
        }
      } else { // update confidence
        per_core_low_confidence_cnt[fdip_proc_id] += 3 - op->bp_confidence;
        if(!per_core_conf_info[fdip_proc_id].fdip_on_conf_off_event)
          per_core_conf_info[fdip_proc_id].conf_off_path_reason = REASON_INV_CONF_INC;
      }
      //log stats
      if(op->oracle_info.btb_miss){
        per_core_conf_info[fdip_proc_id].num_BTB_misses += 1;
      }
      inc_br_conf_counters(op->bp_confidence);
      inc_cf_type_counters(op->table_info->cf_type);
      DEBUG(fdip_proc_id, "op->bp_confidence: %d, low_confidence_cnt: %d, off_path: %d\n", op->bp_confidence, per_core_low_confidence_cnt[fdip_proc_id], op->off_path? 1:0);
    } else { // update confidence based on number of cycles elapsed and btb miss rate
      //if number of cycles times btb miss rate is greater than 1 we have probably seen a btb miss
      DEBUG(fdip_proc_id, "btb miss rate: %f, cycles since recovery: %llu", per_core_btb_miss_rate[fdip_proc_id], cycle_count - per_core_last_recover_cycle[fdip_proc_id]);
      if((double)((cycle_count - per_core_last_recover_cycle[fdip_proc_id]) * per_core_btb_miss_rate[fdip_proc_id])  >= FDIP_BTB_MISS_RATE_CYCLES_THRESHOLD){
        per_core_low_confidence_cnt[fdip_proc_id] = ~0U;
        STAT_EVENT(fdip_proc_id, FDIP_BTB_NUM_CYCLES_OFF_PATH_EVENT);
        if(!per_core_conf_info[fdip_proc_id].fdip_on_conf_off_event)
          per_core_conf_info[fdip_proc_id].conf_off_path_reason = REASON_BTB_MISS_RATE;
      }
    }
  }
}

void log_stats_bp_conf() {
  if (!FDIP_BP_CONFIDENCE)
    return;

  FDIP_Confidence_Info *conf_info = &(per_core_conf_info.data()[fdip_proc_id]);
  if (per_core_low_confidence_cnt[fdip_proc_id] < FDIP_OFF_PATH_THRESHOLD) {
    if (fdip_off_path(fdip_proc_id)) {
      if (!conf_info->fdip_off_conf_on_event) {
        DEBUG(fdip_proc_id, "prev_op op_num: %llu, cf_type: %i, cur_op op_num: %llu, cf_type: %i\n", conf_info->prev_op->op_num, conf_info->prev_op->table_info->cf_type, per_core_cur_op[fdip_proc_id]->op_num, per_core_cur_op[fdip_proc_id]->table_info->cf_type);
        ASSERT(fdip_proc_id, conf_info->prev_op->table_info->cf_type); // must be a cf as the last on-path op
        conf_info->fdip_off_conf_on_event = true;
        STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_NUM_EVENTS);
        if (conf_info->prev_op->oracle_info.mispred) {
          conf_info->off_path_reason = REASON_MISPRED;
          STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_BP_INCORRECT);
          STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_BP_INCORRECT_0_CONF + conf_info->prev_op->bp_confidence);
        }
        //if off path due to btb miss
        else if (conf_info->prev_op->oracle_info.btb_miss) {
          conf_info->off_path_reason = REASON_BTB_MISS;
          STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_BTB_MISS);
          STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_BTB_MISS_NOT_CF + conf_info->prev_op->table_info->cf_type);
        }
        //if off path due to no target
        else if (conf_info->prev_op->oracle_info.no_target) {
          conf_info->off_path_reason = REASON_NO_TARGET;
          STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_NO_TARGET);
        }
        //if off path due to misfetch
        else if (conf_info->prev_op->oracle_info.misfetch) {
          conf_info->off_path_reason = REASON_MISFETCH;
          STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_MISFETCH);
        }
        //if some other reason (shouldn't happen)
        else{
          DEBUG(fdip_proc_id, "fdip off conf on event, unrecognized off path reason: op type: %u\n", conf_info->prev_op->table_info->op_type);
          //ASSERT(fdip_proc_id, false); //Disable for now
        }
      }
      STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_PREF_CANDIDATES);
      STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_BTB_MISS_PREF_CANDIDATES + conf_info->off_path_reason);
    } else {
      STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_ON_PREF_CANDIDATES);
    }
  } else {
    if (fdip_off_path(fdip_proc_id))
      STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_OFF_PREF_CANDIDATES);
    else {
      STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_PREF_CANDIDATES);
      STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_BTB_MISS_BP_TAKEN_CONF_0_PREF_CANDIDATES + conf_info->conf_off_path_reason);
      if (!conf_info->fdip_on_conf_off_event) {
        conf_info->fdip_on_conf_off_event = true;

        STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_EVENTS);

        STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_BTB_MISS_BP_TAKEN_CONF_0 + conf_info->conf_off_path_reason);

        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CF_BR, conf_info->num_cf_br);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CF_CBR, conf_info->num_cf_cbr);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CF_CALL, conf_info->num_cf_call);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CF_IBR, conf_info->num_cf_ibr);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CF_ICALL, conf_info->num_cf_icall);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CF_ICO, conf_info->num_cf_ico);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CF_RET, conf_info->num_cf_ret);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CF_SYS, conf_info->num_cf_sys);

        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CONF_0_BR, conf_info->num_conf_0_branches);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CONF_1_BR, conf_info->num_conf_1_branches);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CONF_2_BR, conf_info->num_conf_2_branches);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_CONF_3_BR, conf_info->num_conf_3_branches);

        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_BTB_MISS, conf_info->num_BTB_misses);
        INC_STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_NUM_OP_DIST_INC, conf_info->num_op_dist_incs);
      }
    }
  }
}

void log_stats_bp_conf_emitted() {
  if (!FDIP_BP_CONFIDENCE)
    return;
  //conf on
  if(per_core_low_confidence_cnt[fdip_proc_id] < FDIP_OFF_PATH_THRESHOLD){
    //actually off
    if(fdip_off_path(fdip_proc_id)) {
      STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_EMITTED);
      STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_ON_BTB_MISS_EMITTED + per_core_conf_info[fdip_proc_id].off_path_reason);
    } else {
      STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_ON_EMITTED);
    }
  } else {
    if(fdip_off_path(fdip_proc_id)) {
      STAT_EVENT(fdip_proc_id, FDIP_OFF_CONF_OFF_EMITTED);
    } else {
      STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_EMITTED);
      STAT_EVENT(fdip_proc_id, FDIP_ON_CONF_OFF_BTB_MISS_BP_TAKEN_CONF_0_EMITTED + per_core_conf_info[fdip_proc_id].conf_off_path_reason);
    }
  }
}

void add_evict_seq(uns proc_id, Addr line_addr) {
  if (per_core_warmed_up[proc_id]) {
    auto it2 = per_core_sequence_aw[proc_id].find(line_addr);
    if (it2 == per_core_sequence_aw[proc_id].end()) {
      per_core_sequence_aw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_aw[proc_id][line_addr].push_back(std::make_pair('e',cycle_count));
    } else {
      it2->second.push_back(std::make_pair('e',cycle_count));
    }
  } else {
    auto it2 = per_core_sequence_bw[proc_id].find(line_addr);
    if (it2 == per_core_sequence_bw[proc_id].end()) {
      per_core_sequence_bw[proc_id].insert(std::make_pair(line_addr, std::vector<std::pair<char,Counter>>()));
      per_core_sequence_bw[proc_id][line_addr].push_back(std::make_pair('e',cycle_count));
    } else {
      it2->second.push_back(std::make_pair('e',cycle_count));
    }
  }
}

/* Returns a new computed FTQ depth based on the current utility ratio or/and timeliness ratio of
   FDIP prefetches. Three different modes based on FDIP_ADJUSTABLE_FTQ (1: AUR, 2: ATR, 3: AURATR) */
void uftq_set_ftq_ft_num(uns proc_id) {
  uint64_t cur_ftq_ft_num = decoupled_fe_get_ftq_num(proc_id);
  if (!FDIP_ADJUSTABLE_FTQ || !per_core_utility_timeliness_info[proc_id].adjust)
    return;

  uint64_t new_ftq_ft_num = cur_ftq_ft_num;
  double cur_utility_ratio = per_core_utility_timeliness_info[proc_id].utility_ratio;
  double cur_timeliness_ratio = per_core_utility_timeliness_info[proc_id].timeliness_ratio;
  uint64_t qdaur = per_core_utility_timeliness_info[proc_id].qdaur;
  uint64_t qdatr = per_core_utility_timeliness_info[proc_id].qdatr;
  if (FDIP_ADJUSTABLE_FTQ == 1) { // utility-based adjustment UFTQ-AUR
    DEBUG(proc_id, "Current utility ratio : %lf, current FTQ ft num : %lu\n", cur_utility_ratio, cur_ftq_ft_num);
    if (cur_utility_ratio < UTILITY_RATIO_THRESHOLD) {
      new_ftq_ft_num -= std::round(cur_ftq_ft_num * (UTILITY_RATIO_THRESHOLD - cur_utility_ratio));
      if (new_ftq_ft_num < UFTQ_MIN_FTQ_BLOCK_NUM)
        new_ftq_ft_num = UFTQ_MIN_FTQ_BLOCK_NUM;
    } else if (cur_utility_ratio > UTILITY_RATIO_THRESHOLD) {
      new_ftq_ft_num += std::round(cur_ftq_ft_num * (cur_utility_ratio - UTILITY_RATIO_THRESHOLD));
      if (new_ftq_ft_num > UFTQ_MAX_FTQ_BLOCK_NUM)
        new_ftq_ft_num = UFTQ_MAX_FTQ_BLOCK_NUM;
    }
    DEBUG(proc_id, "New FTQ ft num : %lu\n", new_ftq_ft_num);
    per_core_utility_timeliness_info[proc_id].adjust = FALSE;
  } else if (FDIP_ADJUSTABLE_FTQ == 2) { // timeliness-based adjustment UFTQ-ATR
    DEBUG(proc_id, "Current timeliness ratio : %lf, current FTQ ft num : %lu\n", cur_timeliness_ratio, cur_ftq_ft_num);
    if (cur_timeliness_ratio < TIMELINESS_RATIO_THRESHOLD) {
      new_ftq_ft_num += std::round(cur_ftq_ft_num * (TIMELINESS_RATIO_THRESHOLD - cur_timeliness_ratio));
      if (new_ftq_ft_num > UFTQ_MAX_FTQ_BLOCK_NUM)
        new_ftq_ft_num = UFTQ_MAX_FTQ_BLOCK_NUM;
    } else if (cur_timeliness_ratio > TIMELINESS_RATIO_THRESHOLD) {
      new_ftq_ft_num -= std::round(cur_ftq_ft_num * (cur_timeliness_ratio - TIMELINESS_RATIO_THRESHOLD));
      if (new_ftq_ft_num < UFTQ_MIN_FTQ_BLOCK_NUM)
        new_ftq_ft_num = UFTQ_MIN_FTQ_BLOCK_NUM;
    }
    DEBUG(proc_id, "New FTQ ft num : %lu\n", new_ftq_ft_num);
    per_core_utility_timeliness_info[proc_id].adjust = FALSE;
  } else if (FDIP_ADJUSTABLE_FTQ == 3) { // combined method UFTQ-ATR-AUR
    DEBUG(proc_id, "Current utility ratio : %lf, timeliness ratio : %lf, current FTQ ft num : %lu\n", cur_utility_ratio, cur_timeliness_ratio, cur_ftq_ft_num);
    if (!qdaur) { // First, find QDAUR
      if (cur_utility_ratio < UTILITY_RATIO_THRESHOLD) {
        new_ftq_ft_num -= std::round(cur_ftq_ft_num * (UTILITY_RATIO_THRESHOLD - cur_utility_ratio));
        if (new_ftq_ft_num < UFTQ_MIN_FTQ_BLOCK_NUM)
          new_ftq_ft_num = UFTQ_MIN_FTQ_BLOCK_NUM;
      } else if (cur_utility_ratio > UTILITY_RATIO_THRESHOLD) {
        new_ftq_ft_num += std::round(cur_ftq_ft_num * (cur_utility_ratio - UTILITY_RATIO_THRESHOLD));
        if (new_ftq_ft_num > UFTQ_MAX_FTQ_BLOCK_NUM)
          new_ftq_ft_num = UFTQ_MAX_FTQ_BLOCK_NUM;
      }
      if (new_ftq_ft_num == cur_ftq_ft_num) // set qdaur when ftq size plateaus
        per_core_utility_timeliness_info[proc_id].qdaur = new_ftq_ft_num;
    } else if (!qdatr) { // Then find QDATR after QDAUR found
      if (cur_timeliness_ratio < TIMELINESS_RATIO_THRESHOLD) {
        new_ftq_ft_num += std::round(cur_ftq_ft_num * (TIMELINESS_RATIO_THRESHOLD - cur_timeliness_ratio));
        if (new_ftq_ft_num > UFTQ_MAX_FTQ_BLOCK_NUM)
          new_ftq_ft_num = UFTQ_MAX_FTQ_BLOCK_NUM;
      } else if (cur_timeliness_ratio > TIMELINESS_RATIO_THRESHOLD) {
        new_ftq_ft_num -= std::round(cur_ftq_ft_num * (cur_timeliness_ratio - TIMELINESS_RATIO_THRESHOLD));
        if (new_ftq_ft_num < UFTQ_MIN_FTQ_BLOCK_NUM)
          new_ftq_ft_num = UFTQ_MIN_FTQ_BLOCK_NUM;
      }
      if (new_ftq_ft_num == cur_ftq_ft_num) // set qdatr when ftq size plateaus
        per_core_utility_timeliness_info[proc_id].qdatr = new_ftq_ft_num;
    } else {
      new_ftq_ft_num = std::round(-0.34*qdaur - 0.64*qdatr + 0.008*qdaur*qdaur + 0.01*qdatr*qdatr + 0.008*qdaur*qdatr);
      if (new_ftq_ft_num < UFTQ_MIN_FTQ_BLOCK_NUM)
        new_ftq_ft_num = UFTQ_MIN_FTQ_BLOCK_NUM;
      else if (new_ftq_ft_num > UFTQ_MAX_FTQ_BLOCK_NUM)
        new_ftq_ft_num = UFTQ_MAX_FTQ_BLOCK_NUM;

      // update qdaur and qdatr for the future use
      if (cur_utility_ratio < UTILITY_RATIO_THRESHOLD) {
        qdaur -= std::round(qdaur * (UTILITY_RATIO_THRESHOLD - cur_utility_ratio));
        if (qdaur < UFTQ_MIN_FTQ_BLOCK_NUM)
          qdaur = UFTQ_MIN_FTQ_BLOCK_NUM;
      } else if (cur_utility_ratio > UTILITY_RATIO_THRESHOLD) {
        qdaur += std::round(qdaur * (cur_utility_ratio - UTILITY_RATIO_THRESHOLD));
        if (qdaur > UFTQ_MAX_FTQ_BLOCK_NUM)
          qdaur = UFTQ_MAX_FTQ_BLOCK_NUM;
      }
      if (cur_timeliness_ratio < TIMELINESS_RATIO_THRESHOLD) {
        qdatr += std::round(qdatr * (TIMELINESS_RATIO_THRESHOLD - cur_timeliness_ratio));
        if (qdatr > UFTQ_MAX_FTQ_BLOCK_NUM)
          qdatr = UFTQ_MAX_FTQ_BLOCK_NUM;
      } else if (cur_timeliness_ratio > TIMELINESS_RATIO_THRESHOLD) {
        qdatr -= std::round(qdatr * (cur_timeliness_ratio - TIMELINESS_RATIO_THRESHOLD));
        if (qdatr < UFTQ_MIN_FTQ_BLOCK_NUM)
          qdatr = UFTQ_MIN_FTQ_BLOCK_NUM;
      }

      per_core_utility_timeliness_info[proc_id].qdaur = qdaur;
      per_core_utility_timeliness_info[proc_id].qdatr = qdatr;
    }
    DEBUG(proc_id, "New FTQ ft num : %lu\n", new_ftq_ft_num);
    per_core_utility_timeliness_info[proc_id].adjust = FALSE;
  }

  if (cur_ftq_ft_num == new_ftq_ft_num)
    STAT_EVENT(proc_id, FDIP_UFTQ_STAY_SAME_FTQ_NUM);
  else if (cur_ftq_ft_num > new_ftq_ft_num)
    STAT_EVENT(proc_id, FDIP_UFTQ_DEC_FTQ_NUM);
  else
    STAT_EVENT(proc_id, FDIP_UFTQ_INC_FTQ_NUM);
  return decoupled_fe_set_ftq_num(proc_id, new_ftq_ft_num);
}

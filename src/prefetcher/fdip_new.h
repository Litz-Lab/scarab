#ifndef __FDIP_NEW_H__
#define __FDIP_NEW_H__


#ifdef __cplusplus
extern "C" {
#endif

  #include "icache_stage.h"

  void alloc_mem_fdip(uns numProcs);
  void init_fdip(uns proc_id);
  void update_fdip();
  void recover_fdip();
  void set_fdip(int _proc_id, Icache_Stage *_ic);
  Flag fdip_off_path(uns proc_id);
  Flag fdip_conf_off_path(uns proc_id);
  uns64 fdip_get_ghist();
  uns64 fdip_hash_addr_ghist(uint64_t addr, uint64_t ghist);
  void print_cl_info(uns proc_id);
  void inc_cnt_useful(uns proc_id, Addr line_addr, Flag pref_miss);
  void inc_cnt_unuseful(uns proc_id, Addr line_addr);
  void inc_cnt_useful_signed(uns proc_id, Addr line_addr);
  void dec_cnt_useful_signed(uns proc_id, Addr line_addr);
  void inc_cnt_useful_ret(uns proc_id, Addr line_addr);
  void inc_icache_miss(uns proc_id, Addr line_addr);
  void inc_icache_hit(uns proc_id, Addr line_addr);
  void inc_off_fetched_cls(Addr line_addr);
  void inc_prefetched_cls(Addr line_addr, uns success);
  void add_evict_seq(uns proc_id, Addr line_addr);
  void not_prefetch(Addr line_addr);
  void probe_prefetched_cls(Addr line_addr);
  void evict_prefetched_cls(uns proc_id, Addr line_addr, Flag by_fdip);
  uns get_miss_reason(uns proc_id, Addr line_addr);
  uns get_last_miss_reason(uns proc_id);
  void set_last_miss_reason(uns proc_id, uns reason);
  uint64_t get_fdip_ftq_occupancy_ops(uns proc_id);
  uint64_t get_fdip_ftq_occupancy(uns proc_id);
  Flag determine_usefulness(Addr line_addr, Op* op);
  void update_useful_lines(uns proc_id, Op* op);
  void update_useful_lines_uc(uns proc_id, Addr line_addr);
  void update_unuseful_lines_uc(uns proc_id, Addr line_addr);
  void inc_useful_lines_uc(uns proc_id, Addr line_addr);
  void dec_useful_lines_uc(uns proc_id, Addr line_addr);
  void update_useful_lines_bloom_filter(uns proc_id, Addr line_addr);
  void inc_utility_info(uns proc_id, Flag useful);
  void inc_timeliness_info(uns proc_id, Flag mshr_hit);
  void fdip_inc_cnt_btb_miss(uns proc_id);
  Flag fdip_search_pref_candidate(Addr addr);
  void insert_pref_candidate_to_seniority_ftq(Addr line_addr);
  void clear_old_seniority_ftq();
  void assert_fdip_break_reason(uns proc_id, Addr line_addr);
  void inc_br_conf_counters(int conf);
  void inc_cf_type_counters(Cf_Type cf_type);
  void btb_miss_bp_taken_conf_update(Op * op);
  void default_conf_update(Op * op);
  void log_stats_bp_conf();
  void log_stats_bp_conf_emitted();
  void uftq_set_ftq_ft_num(uns proc_id);

  
#ifdef __cplusplus
}
#endif

typedef enum ICACHE_MISS_REASON_enum {
  IMISS_NOT_PREFETCHED,
  IMISS_TOO_EARLY_EVICTED_BY_IFETCH,
  IMISS_TOO_EARLY_EVICTED_BY_FDIP,
  IMISS_MSHR_HIT_PREFETCHED_OFFPATH,
  IMISS_MSHR_HIT_PREFETCHED_ONPATH,
} Imiss_Reason;

typedef enum IC_FETCH_TYPE_enum {
  DEMAND_LOAD,
  FDIP_ONPATH,
  FDIP_OFFPATH,
  FDIP_BOTHPATH,
} IC_Fetch_Type;

typedef enum OFF_PATH_REASON_enum {
  REASON_BTB_MISS,
  REASON_MISPRED,
  REASON_MISFETCH,
  REASON_NO_TARGET,
} Off_Path_Reason;

typedef enum CONF_OFF_PATH_REASON_enum {
  REASON_BTB_MISS_BP_TAKEN_CONF_0,
  REASON_BTB_MISS_BP_TAKEN_CONF_1,
  REASON_BTB_MISS_BP_TAKEN_CONF_2,
  REASON_BTB_MISS_BP_TAKEN_CONF_3,
  REASON_BTB_MISS_RATE,
  REASON_INV_CONF_INC,
} Conf_Off_Path_Reason;

typedef struct Utility_Timeliness_Info_struct {
  // useful prefetch counter per 100,000 cycles
  Counter useful_prefetches;
  // unuseful prefetch counter per 100,000 cycles
  Counter unuseful_prefetches;
  double utility_ratio;
  // MSHR prefetch hit counter per 100,000 cycles
  Counter mshr_prefetch_hits;
  // Icache prefetch hit counter per 100,000 cycles
  Counter icache_prefetch_hits;
  double timeliness_ratio;
  Flag adjust;
  uint64_t qdaur;
  uint64_t qdatr;
} Utility_Timeliness_Info;

//metadata for fdip confidence
typedef struct FDIP_Confidence_Info_struct {
  Op * prev_op;
  Op * cur_op;

  //prbably only need one of these lads
  Flag fdip_off_path_event;
  Flag fdip_on_conf_off_event;

  Off_Path_Reason off_path_reason;
  Conf_Off_Path_Reason  conf_off_path_reason;
  
  Counter num_conf_0_branches;
  Counter num_conf_1_branches;
  Counter num_conf_2_branches;
  Counter num_conf_3_branches;

  Counter num_cf_br;
  Counter num_cf_cbr;
  Counter num_cf_call;
  Counter num_cf_ibr;
  Counter num_cf_icall;
  Counter num_cf_ico;
  Counter num_cf_ret;
  Counter num_cf_sys;

  Counter num_BTB_misses;
  Counter num_op_dist_incs;

  Flag fdip_off_conf_on_event;
} FDIP_Confidence_Info;

#endif

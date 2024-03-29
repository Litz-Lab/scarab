#include "decoupled_frontend.h"
#include "frontend/frontend_intf.h"
#include "op.h"
#include "op_pool.h"
#include "isa/isa_macros.h"
#include "prefetcher/pref.param.h"
#include "prefetcher/fdip_new.h"

#include <deque>
#include <vector>
#include <iostream>
#include <tuple>
#include <cmath>

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_DECOUPLED_FE, ##args)

// a threshold of utility ratio
#define UTILITY_RATIO_THRESHOLD 0.70
// a threshold of timeliness ratio
#define TIMELINESS_RATIO_THRESHOLD 0.77

uint32_t FE_FTQ_BLOCK_SIZE = pow(2, FE_FTQ_BLOCK_SIZE_LOG);

std::vector<std::deque<std::pair<Op*, bool>>> per_core_ftq;
std::vector<int> per_core_off_path;
std::vector<int> per_core_sched_off_path;
std::vector<uint64_t> per_core_op_count;
std::vector<std::vector<decoupled_fe_iter>> per_core_ftq_iterators;
std::vector<uint64_t> per_core_recovery_addr;
std::vector<uint64_t> per_core_redirect_cycle;
std::vector<bool> per_core_stalled;
std::vector<uint64_t> per_core_block_count;
std::vector<uint64_t> per_core_ftq_block_num;
extern std::vector<Utility_Timeliness_Info> per_core_utility_timeliness_info;

//per_core pointers
std::deque<std::pair<Op*,bool>> *df_ftq;
int *off_path;
int *sched_off_path;
int set_proc_id;
std::vector<decoupled_fe_iter> *ftq_iterator;
//need to overwrite op->op_num with decoupeld fe

bool trace_mode;

void alloc_mem_decoupled_fe(uns numCores) {
  per_core_ftq.resize(numCores);
  per_core_off_path.resize(numCores);
  per_core_sched_off_path.resize(numCores);
  per_core_op_count.resize(numCores);
  per_core_ftq_iterators.resize(numCores);
  per_core_recovery_addr.resize(numCores);
  per_core_redirect_cycle.resize(numCores);
  per_core_stalled.resize(numCores);
  per_core_block_count.resize(numCores);
  per_core_ftq_block_num.resize(numCores);
}

void init_decoupled_fe(uns proc_id, const char*) {
  trace_mode = false;

#ifdef ENABLE_PT_MEMTRACE
  trace_mode |= (FRONTEND == FE_PT || FRONTEND == FE_MEMTRACE);
#endif

  per_core_off_path[proc_id] = false;
  per_core_sched_off_path[proc_id] = false;
  per_core_op_count[proc_id] = 1;
  per_core_recovery_addr[proc_id] = 0;
  per_core_redirect_cycle[proc_id] = 0;
  per_core_block_count[proc_id] = 0;
  per_core_ftq_block_num[proc_id] = FE_FTQ_BLOCK_NUM;

}

void set_decoupled_fe(int proc_id) {

  df_ftq = &(per_core_ftq.data()[proc_id]);
  off_path = &(per_core_off_path.data()[proc_id]);
  sched_off_path = &(per_core_sched_off_path.data()[proc_id]);
  ftq_iterator = &(per_core_ftq_iterators.data()[proc_id]);
  set_proc_id = proc_id;
}


void reset_decoupled_fe() {}

void recover_decoupled_fe(int proc_id) {
  per_core_off_path[proc_id] = false;
  per_core_sched_off_path[proc_id] = false;
  per_core_recovery_addr[proc_id] = bp_recovery_info->recovery_fetch_addr;
  per_core_block_count[proc_id] = 0;

  for (auto it = per_core_ftq[proc_id].begin(); it != per_core_ftq[proc_id].end(); it++) {
    free_op(it->first);
  }
  per_core_ftq[proc_id].clear();
  per_core_op_count[proc_id] = bp_recovery_info->recovery_op_num + 1;
  DEBUG(set_proc_id,
        "Recovery signalled fetch_addr0x:%llx\n", bp_recovery_info->recovery_fetch_addr);

  for (auto it = per_core_ftq_iterators[proc_id].begin(); it != per_core_ftq_iterators[proc_id].end(); it++) {
    // When the FTQ flushes, reset all iterators
    it->pos = 0;
  }
  if (FDIP_ADJUSTABLE_FTQ && per_core_utility_timeliness_info[proc_id].adjust) {
    double cur_utility_ratio = per_core_utility_timeliness_info[proc_id].utility_ratio;
    double cur_timeliness_ratio = per_core_utility_timeliness_info[proc_id].timeliness_ratio;
    if (FDIP_ADJUSTABLE_FTQ == 1) { // utility-based adjustment UFTQ-AUR
      DEBUG(set_proc_id, "Current utility ratio : %lf, current FTQ block num : %lu\n", cur_utility_ratio, per_core_ftq_block_num[proc_id]);
      if (cur_utility_ratio < UTILITY_RATIO_THRESHOLD) {
        per_core_ftq_block_num[proc_id] -= std::round(per_core_ftq_block_num[proc_id] * (UTILITY_RATIO_THRESHOLD - cur_utility_ratio));
        if (per_core_ftq_block_num[proc_id] < UFTQ_MIN_FTQ_BLOCK_NUM)
          per_core_ftq_block_num[proc_id] = UFTQ_MIN_FTQ_BLOCK_NUM;
      } else if (cur_utility_ratio > UTILITY_RATIO_THRESHOLD) {
        per_core_ftq_block_num[proc_id] += std::round(per_core_ftq_block_num[proc_id] * (cur_utility_ratio - UTILITY_RATIO_THRESHOLD));
        if (per_core_ftq_block_num[proc_id] > UFTQ_MAX_FTQ_BLOCK_NUM)
          per_core_ftq_block_num[proc_id] = UFTQ_MAX_FTQ_BLOCK_NUM;
      }
      DEBUG(set_proc_id, "New FTQ block num : %lu\n", per_core_ftq_block_num[proc_id]);
      per_core_utility_timeliness_info[proc_id].adjust = FALSE;
    } else if (FDIP_ADJUSTABLE_FTQ == 2) { // timeliness-based adjustment UFTQ-ATR
      DEBUG(set_proc_id, "Current timeliness ratio : %lf, current FTQ block num : %lu\n", cur_timeliness_ratio, per_core_ftq_block_num[proc_id]);
      if (cur_timeliness_ratio < TIMELINESS_RATIO_THRESHOLD) {
        per_core_ftq_block_num[proc_id] -= std::round(per_core_ftq_block_num[proc_id] * (TIMELINESS_RATIO_THRESHOLD - cur_timeliness_ratio));
        if (per_core_ftq_block_num[proc_id] < UFTQ_MIN_FTQ_BLOCK_NUM)
          per_core_ftq_block_num[proc_id] = UFTQ_MIN_FTQ_BLOCK_NUM;
      } else if (cur_timeliness_ratio > TIMELINESS_RATIO_THRESHOLD) {
        per_core_ftq_block_num[proc_id] += std::round(per_core_ftq_block_num[proc_id] * (cur_timeliness_ratio - TIMELINESS_RATIO_THRESHOLD));
        if (per_core_ftq_block_num[proc_id] > UFTQ_MAX_FTQ_BLOCK_NUM)
          per_core_ftq_block_num[proc_id] = UFTQ_MAX_FTQ_BLOCK_NUM;
      }
      DEBUG(set_proc_id, "New FTQ block num : %lu\n", per_core_ftq_block_num[proc_id]);
      per_core_utility_timeliness_info[proc_id].adjust = FALSE;
    } else if (FDIP_ADJUSTABLE_FTQ == 3) { // combined method UFTQ-ATR-AUR
      uint64_t qdaur = per_core_ftq_block_num[proc_id];
      uint64_t qdatr = per_core_ftq_block_num[proc_id];
      if (cur_utility_ratio < UTILITY_RATIO_THRESHOLD)
        qdaur -= std::round(per_core_ftq_block_num[proc_id] * (UTILITY_RATIO_THRESHOLD - cur_utility_ratio));
      else if (cur_utility_ratio > UTILITY_RATIO_THRESHOLD)
        qdaur += std::round(per_core_ftq_block_num[proc_id] * (cur_utility_ratio - UTILITY_RATIO_THRESHOLD));
      if (cur_timeliness_ratio < TIMELINESS_RATIO_THRESHOLD)
        qdatr -= std::round(per_core_ftq_block_num[proc_id] * (TIMELINESS_RATIO_THRESHOLD - cur_timeliness_ratio));
      else if (cur_timeliness_ratio > TIMELINESS_RATIO_THRESHOLD)
        qdatr += std::round(per_core_ftq_block_num[proc_id] * (cur_timeliness_ratio - TIMELINESS_RATIO_THRESHOLD));
      per_core_ftq_block_num[proc_id] = std::round(-2.3*qdaur - 31.2*qdatr + 0.007*qdaur*qdaur + 0.1*qdatr*qdatr + 0.3*qdaur*qdatr);
      if (per_core_ftq_block_num[proc_id] < UFTQ_MIN_FTQ_BLOCK_NUM)
        per_core_ftq_block_num[proc_id] = UFTQ_MIN_FTQ_BLOCK_NUM;
      else if (per_core_ftq_block_num[proc_id] > UFTQ_MAX_FTQ_BLOCK_NUM)
        per_core_ftq_block_num[proc_id] = UFTQ_MAX_FTQ_BLOCK_NUM;
    }
  }

  auto op = bp_recovery_info->recovery_op;

  if(per_core_stalled[set_proc_id]) {
    DEBUG(set_proc_id,
          "Unstalled off-path fetch barrier due to recovery fetch_addr0x:%llx off_path:%i op_num:%llu\n",
          op->inst_info->addr, op->off_path, op->op_num);
    per_core_stalled[set_proc_id] = false;
  }

  if (op->oracle_info.recover_at_decode)
    STAT_EVENT(proc_id, FTQ_RECOVER_DECODE);
  else if (op->oracle_info.recover_at_exec)
    STAT_EVENT(proc_id, FTQ_RECOVER_EXEC);

  uint64_t offpath_cycles = cycle_count - per_core_redirect_cycle[proc_id];
  ASSERT(proc_id, cycle_count > per_core_redirect_cycle[proc_id]);
  INC_STAT_EVENT(proc_id, FTQ_OFFPATH_CYCLES, offpath_cycles);
  per_core_redirect_cycle[proc_id] = 0;

  //FIXME always fetch off path ops? should we get rid of this parameter?
  frontend_recover(proc_id, bp_recovery_info->recovery_inst_uid);
  ASSERTM(proc_id, bp_recovery_info->recovery_fetch_addr == frontend_next_fetch_addr(proc_id),
          "Scarab's recovery addr 0x%llx does not match frontend's recovery "
          "addr 0x%llx\n",
          bp_recovery_info->recovery_fetch_addr, frontend_next_fetch_addr(proc_id));
}

void debug_decoupled_fe() {

}

void update_decoupled_fe() {
  uint fetched_inst_bytes = 0;
  uint taken_cf = 0;
  uint predicted_branches = 0;
  uns cf_num = 0;
  uint64_t block_this_cycle = 0;
  uint64_t bytes_this_cycle = 0;
  static int fwd_progress = 0;
  fwd_progress++;
  if (fwd_progress >= 100000) {
    std::cout << "No forward progress for 1000000 cycles" << std::endl;
    ASSERT(0,0);
  }
  if (*off_path)
    STAT_EVENT(set_proc_id, FTQ_CYCLES_OFFPATH);
  else
    STAT_EVENT(set_proc_id, FTQ_CYCLES_ONPATH);

  while(1) {
    if (per_core_block_count[set_proc_id] > per_core_ftq_block_num[set_proc_id]) {
      DEBUG(set_proc_id, "Break due to full FTQ\n");
      if (*off_path)
        STAT_EVENT(set_proc_id, FTQ_BREAK_FULL_BLOCK_OFFPATH);
      else
        STAT_EVENT(set_proc_id, FTQ_BREAK_FULL_BLOCK_ONPATH);
      break;
    }
    if (block_this_cycle == FE_BLOCKS_PER_CYCLE) {
      DEBUG(set_proc_id, "Break due to max blocks per cycle\n");
      if (*off_path)
        STAT_EVENT(set_proc_id, FTQ_BREAK_MAX_BLOCKS_OFFPATH);
      else
        STAT_EVENT(set_proc_id, FTQ_BREAK_MAX_BLOCKS_ONPATH);
      break;
    }
    if (BP_MECH != MTAGE_BP && !bp_is_predictable(g_bp_data, set_proc_id)) {
      DEBUG(set_proc_id, "Break due to limited branch predictor\n");
      if (*off_path)
        STAT_EVENT(set_proc_id, FTQ_BREAK_PRED_BR_OFFPATH);
      else
        STAT_EVENT(set_proc_id, FTQ_BREAK_PRED_BR_ONPATH);
      break;
    }
    if (per_core_stalled[set_proc_id]) {
      DEBUG(set_proc_id, "Break due to wait for fetch barrier resolved\n");
      if (*off_path)
        STAT_EVENT(set_proc_id, FTQ_BREAK_BAR_FETCH_OFFPATH);
      else
        STAT_EVENT(set_proc_id, FTQ_BREAK_BAR_FETCH_ONPATH);
      break;
    }
    if (!frontend_can_fetch_op(set_proc_id)) {
      std::cout << "Warning could not fetch inst from frontend" << std::endl;
      break;
    }

    fwd_progress = 0;
    uint64_t pred_addr = 3;
    Op* op = alloc_op(set_proc_id);
    frontend_fetch_op(set_proc_id, op);
    op->op_num = per_core_op_count[set_proc_id]++;
    op->off_path = *off_path;

    if(op->table_info->cf_type) {
      ASSERT(set_proc_id, op->eom);
      pred_addr = bp_predict_op(g_bp_data, op, cf_num++, op->inst_info->addr);
      DEBUG(set_proc_id,
            "Predict CF fetch_addr:%llx true_npc:%llx pred_npc:%lx mispred:%i misfetch:%i btb miss:%i taken:%i recover_at_decode:%i recover_at_exec:%i off_path:%i bar_fetch:%i\n",
            op->inst_info->addr, op->oracle_info.npc, pred_addr,
            op->oracle_info.mispred, op->oracle_info.misfetch,
            op->oracle_info.btb_miss, op->oracle_info.pred == TAKEN,
            op->oracle_info.recover_at_decode, op->oracle_info.recover_at_exec,
            *off_path, op->table_info->bar_type & BAR_FETCH);

      /* On fetch barrier stall the frontend. Ignore BTB misses here as the exec frontend cannot
         handle recovery/execution until syscalls retire. This is ok as stalling causes the same
         cycle penalty than recovering from BTB miss. */
      if ((op->table_info->bar_type & BAR_FETCH) || IS_CALLSYS(op->table_info)) {
        op->oracle_info.recover_at_decode = FALSE;
        op->oracle_info.recover_at_exec = FALSE;
        decoupled_fe_stall(op);
      }

      if(op->oracle_info.recover_at_decode || op->oracle_info.recover_at_exec) {
        ASSERT(0, (int)op->oracle_info.recover_at_decode + (int)op->oracle_info.recover_at_exec < 2);
        /* If already on the off-path do not schedule recovery as scarab cannot recover OOO
           (An older op may recover at exec and a younger op may recover at decode)
           This is not accurate but it should not affect the time spend on the off-path */
        if (*off_path) {
          op->oracle_info.recover_at_decode = FALSE;
          op->oracle_info.recover_at_exec = FALSE;
        }
        *off_path = true;
        frontend_redirect(set_proc_id, op->inst_uid, pred_addr);
        per_core_redirect_cycle[set_proc_id] = cycle_count;
      }
      // If we are already on the off-path redirect on all taken branches in TRACE-MODE
      else if (trace_mode && *off_path && op->oracle_info.pred == TAKEN) {
        frontend_redirect(set_proc_id, op->inst_uid, pred_addr);
      }
      taken_cf = (op->oracle_info.pred == TAKEN) ? taken_cf + 1 : taken_cf;
    }
    else {
      ASSERT(0,!(op->oracle_info.recover_at_decode | op->oracle_info.recover_at_exec));
      /* On fetch barrier stall the frontend. */
      if (op->table_info->bar_type & BAR_FETCH) {
        decoupled_fe_stall(op);
      }
    }
    // We start a new block if crossing a block or take a branch depending on packet break conditions
    bool start_new_block = false;
    if (op->eom) {
      bool cross_block = (op->inst_info->addr >> FE_FTQ_BLOCK_SIZE_LOG) != ((op->inst_info->addr + op->inst_info->trace_info.inst_size) >> FE_FTQ_BLOCK_SIZE_LOG);
      bool max_bytes = (bytes_this_cycle + op->inst_info->trace_info.inst_size) > FE_FTQ_BLOCK_SIZE;

      bytes_this_cycle += op->inst_info->trace_info.inst_size;
      start_new_block |= cross_block || (op->table_info->cf_type && op->oracle_info.pred == TAKEN);
      block_this_cycle += (!FETCH_ACROSS_CACHE_LINES && cross_block) || (FETCH_ACROSS_CACHE_LINES && max_bytes) ||
        (FETCH_BREAK_ON_TAKEN && op->table_info->cf_type && op->oracle_info.pred == TAKEN);
    }

    if (op->table_info->cf_type == CF_CBR) {
      predicted_branches++;
    }

    per_core_block_count[set_proc_id] += start_new_block;
    df_ftq->emplace_back(std::pair<Op*, bool>(op, start_new_block));
    fetched_inst_bytes += op->inst_info->trace_info.inst_size;

    if (*off_path) {
      STAT_EVENT(set_proc_id, FTQ_FETCHED_INS_OFFPATH);
    }
    else {
      STAT_EVENT(set_proc_id, FTQ_FETCHED_INS_ONPATH);
    }
      
    DEBUG(set_proc_id,
          "Push new op to FTQ fetch_addr0x:%llx off_path:%i op_num:%llu dis:%s recovery_addr:%lx fetch_bar:%i\n",
          op->inst_info->addr, op->off_path, op->op_num, disasm_op(op, TRUE), per_core_recovery_addr[set_proc_id], op->table_info->bar_type & BAR_FETCH);
    // Recovery sanity check
    if (per_core_recovery_addr[set_proc_id]) {
      ASSERT(set_proc_id, per_core_recovery_addr[set_proc_id] == op->inst_info->addr);
      per_core_recovery_addr[set_proc_id] = 0;
    }
  }
}

bool decoupled_fe_fetch_op(Op** op, int proc_id) {
  ASSERT(proc_id, (uns)proc_id<per_core_ftq.size());
  if (per_core_ftq[proc_id].size()) {
    bool start_new_block;
    std::tie(*op, start_new_block) = per_core_ftq[proc_id].front();
    per_core_block_count[proc_id] -= start_new_block;
    per_core_ftq[proc_id].pop_front();
    DEBUG(set_proc_id,
          "Fetch op from FTQ fetch_addr0x:%llx off_path:%i op_num:%llu\n",
          (*op)->inst_info->addr, (*op)->off_path, (*op)->op_num);
    for (auto it = per_core_ftq_iterators[proc_id].begin(); it != per_core_ftq_iterators[proc_id].end(); it++) {
      // When the icache consumes an op decrement the iter's offset so it points to the same entry as before
      if (FDIP_BP_CONFIDENCE && it->pos == 0) {
        fdip_set_cur_op(proc_id, *op);
      }
      if (it->pos) {
        it->pos--;
      }
    }
    return true;
  }
  return false;
}

bool decoupled_fe_can_fetch_op(int proc_id) {
  return per_core_ftq[proc_id].size() > 0;
}

uint64_t decoupled_fe_next_fetch_addr(int proc_id) {
  //ASSERT(proc_id, per_core_ftq[proc_id].size() > 0);
  if(!per_core_ftq[proc_id].size())
    return frontend_next_fetch_addr(proc_id);
  
  return per_core_ftq[proc_id].front().first->inst_info->addr;
}

void decoupled_fe_return_op(Op *op) {
  df_ftq->emplace_front(std::pair<Op*, bool>(op,false));
  DEBUG(set_proc_id,
        "Return fetched op backt to FTQ fetch_addr0x:%llx off_path:%i\n",
        op->inst_info->addr, *off_path);
  for (auto it = per_core_ftq_iterators[set_proc_id].begin(); it != per_core_ftq_iterators[set_proc_id].end(); it++) {
    // When the icache returns an op increment the iter's offset so it points to the same entry as before
    if (it->pos) {
      it->pos++;
    }
  }
  
}
  
decoupled_fe_iter* decoupled_fe_new_ftq_iter() {
  per_core_ftq_iterators[set_proc_id].push_back(decoupled_fe_iter());
  return &per_core_ftq_iterators[set_proc_id].back();
}

/* Returns the Op at current FTQ iterator position. Returns NULL if the FTQ is empty */ 
Op* decoupled_fe_ftq_iter_get(decoupled_fe_iter* iter, bool *end_of_block) {
  if (iter->pos == df_ftq->size()) {
    if (!df_ftq->size())
      ASSERT(set_proc_id, iter->pos == 0);
    return NULL;
  }
  ASSERT(set_proc_id, iter->pos >= 0);
  ASSERT(set_proc_id, iter->pos < df_ftq->size());
  *end_of_block = df_ftq->at(iter->pos).second;
  return df_ftq->at(iter->pos).first;
}

/* Increments the iterator and returns the Op at FTQ iterator position. Returns NULL if the FTQ is empty */
Op* decoupled_fe_ftq_iter_get_next(decoupled_fe_iter* iter, bool *end_of_block) {
  if (iter->pos == df_ftq->size() || iter->pos + 1 == df_ftq->size()) {
    if (!df_ftq->size())
      ASSERT(set_proc_id, iter->pos == 0);
    return NULL;
  }
  ASSERT(set_proc_id, iter->pos + 1 >= 0);
  ASSERT(set_proc_id, iter->pos + 1 < df_ftq->size());
  iter->pos++;
  *end_of_block = df_ftq->at(iter->pos).second;
  return df_ftq->at(iter->pos).first;
}

/* Returns iter offset from the start of the FTQ, this offset gets incremented
   by advancing the iter and decremented by the icache consuming FTQ entries,
   and reset by flushes */
uint64_t decoupled_fe_ftq_iter_offset(decoupled_fe_iter* iter) {
  return iter->pos;
}

uint64_t decoupled_fe_ftq_num_ops() {
  return df_ftq->size();
}

uint64_t decoupled_fe_ftq_num_blocks() {
  return  per_core_block_count[set_proc_id];
}

void decoupled_fe_stall(Op *op) {
  per_core_stalled[set_proc_id] = true;
  DEBUG(set_proc_id,
        "Decoupled fetch stalled due to barrier fetch_addr0x:%llx off_path:%i op_num:%llu\n",
        op->inst_info->addr, op->off_path, op->op_num);
}

void decoupled_fe_retire(Op *op, int proc_id, uns64 inst_uid) {
  if((op->table_info->bar_type & BAR_FETCH) || IS_CALLSYS(op->table_info)) {
    per_core_stalled[set_proc_id] = false;
    DEBUG(set_proc_id,
          "Decoupled fetch unstalled due to retired barrier fetch_addr0x:%llx off_path:%i op_num:%llu list_count:%i\n",
          op->inst_info->addr, op->off_path, op->op_num, td->seq_op_list.count);
    ASSERT(set_proc_id, td->seq_op_list.count == 1);
  }

  //unblock pin exec driven, trace frontends do not need to block/unblock
  frontend_retire(proc_id, inst_uid);
}

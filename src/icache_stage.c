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

/***************************************************************************************
 * File         : icache_stage.c
 * Author       : HPS Research Group
 * Date         : 12/7/1998
 * Description  :
 ***************************************************************************************/

#include "icache_stage.h"

#include <math.h>

#include "globals/assert.h"
#include "globals/debug_stage.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "debug/debug.param.h"
#include "debug/debug_macros.h"
#include "debug/debug_print.h"

#include "bp/bp.param.h"
#include "core.param.h"
#include "memory/memory.param.h"
#include "prefetcher/pref.param.h"

#include "bp/bp.h"
#include "frontend/frontend.h"
#include "frontend/pin_trace_fe.h"
#include "libs/list_lib.h"
#include "memory/memory.h"
#include "prefetcher/D_JOLT.h"
#include "prefetcher/FNL+MMA.h"
#include "prefetcher/eip.h"
#include "prefetcher/fdip.h"
#include "prefetcher/l2l1pref.h"
#include "prefetcher/stream_pref.h"

#include "cmp_model.h"
#include "decode_stage.h"
#include "ft.h"
#include "ft_op_buffer.h"
#include "map.h"
#include "op_pool.h"
#include "sim.h"
#include "statistics.h"
#include "thread.h"
#include "uop_queue_stage.h"

static inline void icache_demote_victim(Addr fill_addr);

/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_ICACHE_STAGE, ##args)
#define DEBUG_FDIP(proc_id, args...) _DEBUG(proc_id, DEBUG_FDIP, ##args)

/**************************************************************************************/
/* Global Variables */

/**************************************************************************************/

Icache_Stage* ic = NULL;

extern Cmp_Model cmp_model;
extern Memory* mem;
extern Rob_Stall_Reason rob_stall_reason;
extern Rob_Block_Issue_Reason rob_block_issue_reason;

/**************************************************************************************/
/* Local prototypes */

static inline void icache_process_ops(Stage_Data* cur_data, Flag fetched_from_uop_cache, uns start_idx);
static inline Inst_Info** lookup_icache(void);
static inline void prefetcher_update_on_icache_access(Flag icache_hit);
static inline void icache_hit_events(void);
static inline void icache_miss_events(void);
static inline Flag mem_req_on_icache_miss(void);
static inline Inst_Info** ic_pref_cache_access(void);
static inline int32_t inst_lost_get_full_window_reason(void);
static inline void log_stats_ic_miss(void);
static inline void log_stats_ic_hit(void);
static inline void log_stats_mshr_hit(Addr line_addr);

static inline FT_Arbitration_Result ft_arbitration(void);
static inline Icache_State icache_mem_req_actions(Break_Reason*);
static inline Icache_State icache_wait_for_miss_actions(Break_Reason*);
static inline Flag fill_icache_stage_data(int requested, Stage_Data* sd);
static inline void icache_serve_ops(void);
static inline Icache_State icache_serving_actions(Break_Reason*);
static inline void uop_cache_serve_ops(void);
static inline Icache_State uop_cache_serving_actions(Break_Reason*);
static inline void execute_coupled_FSM(void);

static inline void wp_process_icache_evicted(Icache_Data* line, Mem_Req* req, Addr* repl_line_addr);
static inline void wp_process_icache_hit(Icache_Data* line, Addr fetch_addr);
static inline void wp_process_icache_fill(Icache_Data* line, Mem_Req* req);

static inline Flag is_fetch_barrier_op(Op* op) {
  return (op->uop->bar_type & BAR_FETCH);
}

/**************************************************************************************/
/* set_icache_stage: */

void set_icache_stage(Icache_Stage* new_ic) {
  ic = new_ic;
}

/**************************************************************************************/
/* init_icache_stage: */

void init_icache_stage(uns8 proc_id, const char* name) {
  ASSERT(0, ic);
  DEBUG(proc_id, "Initializing %s stage\n", name);

  memset(ic, 0, sizeof(Icache_Stage));

  ic->proc_id = proc_id;
  /* `name` is expected to be long-lived (caller passes string literals). */
  ic->sd.name = (char*)name;

  /* initialize the ops array */
  ASSERT(proc_id, IC_ISSUE_WIDTH);
  ic->sd.max_op_count = IC_ISSUE_WIDTH;
  ic->sd.ops = (Op**)malloc(sizeof(Op*) * IC_ISSUE_WIDTH);
  ft_op_buffer_init(ic);
  ft_op_buffer_reset(ic);

  /* initialize the cache structure */
  init_cache(&ic->icache, "ICACHE", ICACHE_SIZE, ICACHE_ASSOC, ICACHE_LINE_SIZE, 0, REPL_TRUE_LRU);

  /* init icache_line_info struct - this struct keeps data about corresponding
   * icache lines */
  if (WP_COLLECT_STATS) {
    // init_cache(&ic->icache_line_info, "IC LI", ICACHE_SIZE, ICACHE_ASSOC,
    // ICACHE_LINE_SIZE,
    //           sizeof(Icache_Data), REPL_TRUE_LRU);
    /*init_cache(&ic->icache_line_info, "IC LI", ICACHE_SIZE, ICACHE_ASSOC,*/
    /*ICACHE_LINE_SIZE, sizeof(Icache_Data), ICACHE_REPL);*/
    init_cache(&ic->icache_line_info, "IC LI", ICACHE_SIZE, ICACHE_ASSOC, ICACHE_LINE_SIZE, sizeof(Icache_Data),
               REPL_TRUE_LRU);
  }

  // moved the init code from here to reset
  reset_icache_stage();

  if (IC_PREF_CACHE_ENABLE)
    init_cache(&ic->pref_icache, "IC_PREF_CACHE", IC_PREF_CACHE_SIZE, IC_PREF_CACHE_ASSOC, ICACHE_LINE_SIZE, 0,
               REPL_TRUE_LRU);

  memset(ic->rand_wb_state, 0, NUM_ELEMENTS(ic->rand_wb_state));
}

/**************************************************************************************/
/* get_current_stage_data: */

Stage_Data* get_current_stage_data() {
  if (!UOP_CACHE_ENABLE) {
    return &ic->sd;
  } else {
    // two stage data cannot be used at once
    ASSERT(ic->proc_id, ic->sd.op_count == 0 || uc->sd.op_count == 0);
    return uc->sd.op_count ? &uc->sd : &ic->sd;
  }
}

/**************************************************************************************/
/* reset_icache_stage: */

void reset_icache_stage() {
  Stage_Data* cur_data = get_current_stage_data();
  uns ii;
  for (ii = 0; ii < cur_data->max_op_count; ii++)
    cur_data->ops[ii] = NULL;
  cur_data->op_count = 0;

  ic->off_path = FALSE;
  ic->back_on_path = FALSE;
  ic->fetch_barrier_pending = FALSE;
  ic->fetch_barrier_inst_uid = 0;
  ic->fetch_barrier_op_num = 0;
  op_count[ic->proc_id] = 1;
  unique_count_per_core[ic->proc_id] = 1;
}

/**************************************************************************************/
/* reset_all_ops_icache_stage: */
// CMP used for bogus run: may be combined with reset_icache_stage
void reset_all_ops_icache_stage() {
  Stage_Data* cur_data = get_current_stage_data();
  uns ii;
  for (ii = 0; ii < cur_data->max_op_count; ii++)
    cur_data->ops[ii] = NULL;
  cur_data->op_count = 0;

  ic->off_path = FALSE;
  ic->back_on_path = FALSE;
  ic->fetch_barrier_pending = FALSE;
  ic->fetch_barrier_inst_uid = 0;
  ic->fetch_barrier_op_num = 0;
}

/**************************************************************************************/
/* recover_icache_stage: */

void recover_icache_stage() {
  Stage_Data* cur_data = get_current_stage_data();
  uns ii;
  Flag flushed = FALSE;

  ASSERT(ic->proc_id, ic->proc_id == bp_recovery_info->proc_id);
  DEBUG(ic->proc_id, "Icache stage recovery signaled.  recovery_fetch_addr: 0x%s recovery_op_num:%llu\n",
        hexstr64s(bp_recovery_info->recovery_fetch_addr), (unsigned long long)bp_recovery_info->recovery_op_num);

  recover_ft_op_buffer(ic);

  cur_data->op_count = 0;
  for (ii = 0; ii < cur_data->max_op_count; ii++) {
    if (cur_data->ops[ii]) {
      if (IS_FLUSHING_OP(cur_data->ops[ii])) {
        op_select_bp_pred_info(cur_data->ops[ii], BP_PRED_MAIN);
        DEBUG(ic->proc_id, "Recovery op found in %s slot:%u op_num:%llu off_path:%u addr:0x%llx\n", cur_data->name, ii,
              (unsigned long long)cur_data->ops[ii]->op_num, cur_data->ops[ii]->off_path,
              (unsigned long long)cur_data->ops[ii]->inst->addr);
      }
      if (FLUSH_OP(cur_data->ops[ii])) {
        DEBUG(ic->proc_id, "Icache flushing op_num:%llu off_path:%u\n", (unsigned long long)cur_data->ops[ii]->op_num,
              cur_data->ops[ii]->off_path);
        flushed = TRUE;
        ASSERT(ic->proc_id, cur_data->ops[ii]->off_path);
        if (cur_data->ops[ii]->parent_FT)
          ft_free_op(cur_data->ops[ii]);
        cur_data->ops[ii] = NULL;
      } else {
        cur_data->op_count++;
      }
    }
  }

  /* Only assert when cur_data itself had flushes: in that case the youngest
     survivor in cur_data IS the recovery op (IS_FLUSHING_OP). */
  if (cur_data->op_count > 0 && flushed) {
    Op* op = cur_data->ops[cur_data->op_count - 1];
    assert_ft_after_recovery(ic->proc_id, op, bp_recovery_info->recovery_fetch_addr);
  }

  ic->back_on_path = !bp_recovery_info->recovery_force_offpath;

  // Only clear the fetch barrier if the barrier op is off-path (op_num is
  // newer than the recovery point and will be flushed). If the barrier op is
  // on-path (op_num <= recovery_op_num) it will still retire from the ROB, so
  // keep the barrier pending and let icache_resolve_fetch_barrier() handle it.
  if (ic->fetch_barrier_pending && ic->fetch_barrier_op_num > bp_recovery_info->recovery_op_num) {
    ic->fetch_barrier_pending = FALSE;
    ic->fetch_barrier_inst_uid = 0;
    ic->fetch_barrier_op_num = 0;
  }

  if (!ft_op_buffer_can_fetch_op(ic)) {
    op_count[ic->proc_id] = bp_recovery_info->recovery_op_num + 1;
    if (UOP_CACHE_ENABLE)
      uop_cache_clear_lookup_buffer();
  } else if (UOP_CACHE_ENABLE) {
    // ft_op_buffer has surviving on-path ops after a partial flush (mixed-FT
    // recovery: L0 early recovery followed by a main-predictor recovery).
    // The UOP cache lookup buffer was loaded for the original pre-flush FT and
    // now covers more ops than ft_op_buffer. Trim it to the surviving count so
    // that uop_cache_serve_ops() sees end_of_ft exactly when ft_op_buffer is
    // exhausted, preserving the invariant between the two buffers.
    uop_cache_trim_lookup_buffer_to_n_ops(ft_op_buffer_count(ic));
  }

  /* TC'24: squash wrong-path EIP entries on recovery */
  if (EIP_ENABLE)
    eip_recover(ic->proc_id, bp_recovery_info->recovery_op_num);

  // Do not signal resteer if a fetch barrier is still pending: the icache must
  // remain STALLED until icache_resolve_fetch_barrier() is called at ROB
  // retirement.  Setting the signal here would cause execute_coupled_FSM() to
  // override the STALLED state and skip the barrier wait.
  ic->icache_stage_resteer_signaled = !ft_op_buffer_can_fetch_op(ic) && !ic->fetch_barrier_pending;
}

/**************************************************************************************/
/* redirect_icache_stage: */

void redirect_icache_stage() {
  recover_icache_stage();
  return;
}

/**************************************************************************************/
/* debug_icache_stage: */

void debug_icache_stage() {
  Stage_Data* cur_data = get_current_stage_data();
  DPRINTF("# %-10s  op_count:%d ", cur_data->name, cur_data->op_count);
  DPRINTF("fetch_addr:0x%s  path:%s  state:%s  next_state:%s\n", hexstr64s(ic->fetch_addr),
          ic->off_path ? "OFF_PATH" : "ON_PATH ", icache_state_names[ic->state], icache_state_names[ic->next_state]);
  DPRINTF("# %-10s  op_nums:", cur_data->name);
  print_stage_op_nums(GLOBAL_DEBUG_STREAM, cur_data->ops, cur_data->op_count);
  DPRINTF("\n");

  // print icache stage
  DPRINTF("# %-10s  op_count:%d\n", "ICache", cur_data->op_count);
  print_op_array(GLOBAL_DEBUG_STREAM, cur_data->ops, cur_data->max_op_count, cur_data->op_count);
}

/**************************************************************************************/
/* icache_resolve_fetch_barrier: */

void icache_resolve_fetch_barrier(uns8 proc_id, uns64 inst_uid) {
  if (model->id == CMP_MODEL) {
    set_icache_stage(&cmp_model.icache_stage[proc_id]);
  }

  ASSERT(proc_id,
         ic && ic->fetch_barrier_pending && ic->fetch_barrier_inst_uid && ic->fetch_barrier_inst_uid == inst_uid);
  ASSERT(proc_id, ic->state == ICACHE_STALLED);

  ic->fetch_barrier_pending = FALSE;
  ic->fetch_barrier_inst_uid = 0;
  ic->fetch_barrier_op_num = 0;

  // Drop any stale FT pointers; the barrier op may have been the last op and
  // its FT freed at retirement, so force a resteer to pick up a fresh FT.
  if (UOP_CACHE_ENABLE) {
    uop_cache_clear_lookup_buffer();
  }
  ft_op_buffer_reset(ic);

  DEBUG(proc_id, "Fetch barrier resolved by ROB for inst_uid:%llu\n", (uns64)inst_uid);
}

/**************************************************************************************/
/* in_icache: returns whether instr in icache
 *            Used for branch stat collection
 */

Flag in_icache(Addr addr) {
  Addr line_addr;
  return cache_access(&ic->icache, addr, &line_addr, FALSE) != NULL;
}

/**************************************************************************************/
/* lookup_icache: returns instr if found in icache or other structures when enabled
 */
Inst_Info** lookup_icache() {
  STAT_EVENT(ic->proc_id, POWER_ICACHE_ACCESS);
  STAT_EVENT(ic->proc_id, POWER_ITLB_ACCESS);

  Inst_Info** line = NULL;
  line = (Inst_Info**)cache_access(&ic->icache, ic->fetch_addr, &ic->line_addr, TRUE);
  if (PERFECT_ICACHE && !line)
    line = (Inst_Info**)INIT_CACHE_DATA_VALUE;

  // ideal L2 Icache prefetcher
  if (IDEAL_L2_ICACHE_PREFETCHER && !line) {
    Addr dummy_line_addr;
    L1_Data* data;
    Cache* l1_cache = model->mem == MODEL_MEM ? &mem->uncores[ic->proc_id].l1->cache : NULL;
    data = l1_cache ? (L1_Data*)cache_access(l1_cache, ic->fetch_addr, &dummy_line_addr, TRUE) : NULL;
    if (data) {  // second level cache hit
      STAT_EVENT(ic->proc_id, L2_IDEAL_FILL_ICACHE);
      // actually bring it into the L1 icache
      Addr dummy_repl_line_addr;
      icache_demote_victim(ic->fetch_addr);
      line =
          (Inst_Info**)cache_insert(&ic->icache, ic->proc_id, ic->fetch_addr, &dummy_line_addr, &dummy_repl_line_addr);
    } else
      STAT_EVENT(ic->proc_id, L2_IDEAL_MISS_ICACHE);
  }

  if (IC_PREF_CACHE_ENABLE && (!line)) {
    line = ic_pref_cache_access();
  }

  if (WP_COLLECT_STATS) {  // CMP remove?
    Addr dummy_addr;
    Icache_Data* line_info = (Icache_Data*)cache_access(&ic->icache_line_info, ic->fetch_addr, &dummy_addr, TRUE);
    if (line && (line != (Inst_Info**)INIT_CACHE_DATA_VALUE)) {
      ASSERT(ic->proc_id, line_info);
      wp_process_icache_hit(line_info, ic->fetch_addr);
    }
  }

  return line;
}

void prefetcher_update_on_icache_access(Flag icache_hit) {
  if (EIP_ENABLE)
    eip_prefetch(ic->proc_id, ic->fetch_addr, icache_hit, 0, ic->off_path, op_count[ic->proc_id]);
  if (DJOLT_ENABLE)
    djolt_prefetch(ic->proc_id, ic->fetch_addr, icache_hit, 0);
  if (FNLMMA_ENABLE)
    fnlmma_prefetch(ic->proc_id, ic->fetch_addr, icache_hit, 0);
}

void icache_hit_events() {
  DEBUG(ic->proc_id, "Cache hit on op_num:%s @ 0x%s line_addr 0x%s\n", unsstr64(op_count[ic->proc_id]),
        hexstr64s(ic->fetch_addr), hexstr64s(ic->fetch_addr & ~0x3F));

  prefetcher_update_on_icache_access(/*icache_hit*/ TRUE);
  log_stats_ic_hit();
}

void icache_miss_events() {
  DEBUG(ic->proc_id, "Cache miss on op_num:%s @ 0x%s\n", unsstr64(op_count[ic->proc_id]), hexstr64s(ic->fetch_addr));

  prefetcher_update_on_icache_access(/*icache_hit*/ FALSE);
  log_stats_ic_miss();
  log_stats_mshr_hit(ic->line_addr);
}

Flag mem_req_on_icache_miss() {
  /* if the icache is available, wait for a miss */
  /* otherwise, refetch next cycle */
  if (model->mem == MODEL_MEM) {
    if (ic->proc_id)
      ASSERTM(ic->proc_id, ic->line_addr, "ic fetch addr: %llu\n", ic->fetch_addr);
    ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->line_addr)
    Flag success = FALSE;
    success = new_mem_req(MRT_IFETCH, ic->proc_id, ic->line_addr, ICACHE_LINE_SIZE, 0, NULL, instr_fill_line,
                          unique_count, 0);
    if (success) {  // CMP maybe unique_count_per_core[proc_id]?
      DEBUG(ic->proc_id, "from IC_STAGE for cl 0x%llx at cycle %llu\n", ic->line_addr, cycle_count);
      STAT_EVENT(ic->proc_id, ICACHE_STAGE_MISS);

      if (ONE_MORE_CACHE_LINE_ENABLE) {
        Addr one_more_addr;
        Addr extra_line_addr;
        Icache_Data* extra_line;

        one_more_addr = ((ic->line_addr >> LOG2(ICACHE_LINE_SIZE)) & 1)
                            ? ((ic->line_addr >> LOG2(ICACHE_LINE_SIZE)) - 1) << LOG2(ICACHE_LINE_SIZE)
                            : ((ic->line_addr >> LOG2(ICACHE_LINE_SIZE)) + 1) << LOG2(ICACHE_LINE_SIZE);

        extra_line = (Icache_Data*)cache_access(&ic->icache, one_more_addr, &extra_line_addr, FALSE);
        ASSERT(ic->proc_id, one_more_addr == extra_line_addr);
        if (!extra_line) {
          if (new_mem_req(MRT_IFETCH, ic->proc_id, extra_line_addr, ICACHE_LINE_SIZE, 0, NULL, NULL, unique_count, 0))
            STAT_EVENT_ALL(ONE_MORE_SUCESS);
          else
            STAT_EVENT_ALL(ONE_MORE_DISCARDED_MEM_REQ_FULL);
        } else
          STAT_EVENT_ALL(ONE_MORE_DISCARDED_L0CACHE);
      }
    }

    if (!ic->off_path) {
      STAT_EVENT(ic->proc_id, ICACHE_MISS_MEM_REQ_FAIL_ON_PATH + success);
    }

    return success;
  }
  ASSERT(ic->proc_id, 0);
  return TRUE;
}

FT_Arbitration_Result ft_arbitration() {
  ASSERT(ic->proc_id, !ft_op_buffer_can_fetch_op(ic));

  FT* ft = decoupled_fe_pop_ft();
  if (!ft) {
    return FT_UNAVAILABLE;
  } else {
    FT_Info ft_info = ft_get_ft_info(ft);
    // set the current fetch address
    ic->fetch_addr = ft_info.static_info.start;
    ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->fetch_addr);

    // look up uop cache
    Flag ft_in_uop_cache = uop_cache_lookup_ft_and_fill_lookup_buffer(ft_info, ic->off_path);

    // look up icache if uop miss (inlcuding when uop cache disabled) or if requested
    if (!ft_in_uop_cache || ALWAYS_LOOKUP_ICACHE) {
      ic->line = lookup_icache();
      ic->lookups_per_cycle_count++;
      ASSERT(ic->proc_id, ic->lookups_per_cycle_count <= ICACHE_READ_PORTS);
    }

    if (ft_in_uop_cache) {
      // uop cache hit
      if (!ic->off_path) {
        STAT_EVENT(ic->proc_id, FT_UOP_CACHE_HIT_ON_PATH);
      } else {
        STAT_EVENT(ic->proc_id, FT_UOP_CACHE_HIT_OFF_PATH);
      }

      if (ALWAYS_LOOKUP_ICACHE) {
        if (ic->line) {
          icache_hit_events();
        } else {
          icache_miss_events();
          if (IPRF_ON_UOP_CACHE_HIT) {
            // start a memreq to fill icache, but do not cause any stalls.
            // Use for more inclusivity between IC and UC
            new_mem_req(MRT_IPRF, ic->proc_id, ic->line_addr, ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count,
                        0);
          }
        }
      }

      ft_op_buffer_fill_from_ft(ic, ft);

      return FT_HIT_UOP_CACHE;
    } else if (ic->line) {
      // uop cache miss and icache hit
      if (!ic->off_path) {
        STAT_EVENT(ic->proc_id, FT_UOP_CACHE_MISS_ICACHE_HIT_ON_PATH);
      } else {
        STAT_EVENT(ic->proc_id, FT_UOP_CACHE_MISS_ICACHE_HIT_OFF_PATH);
      }
      icache_hit_events();

      ft_op_buffer_fill_from_ft(ic, ft);

      return FT_HIT_ICACHE;
    } else {
      // uop cache miss and icache miss
      if (!ic->off_path) {
        STAT_EVENT(ic->proc_id, FT_UOP_CACHE_MISS_ICACHE_MISS_ON_PATH);
      } else {
        STAT_EVENT(ic->proc_id, FT_UOP_CACHE_MISS_ICACHE_MISS_OFF_PATH);
      }
      icache_miss_events();

      ft_op_buffer_fill_from_ft(ic, ft);

      return FT_MISS_BOTH;
    }
  }
}

Icache_State icache_mem_req_actions(Break_Reason* break_fetch) {
  DEBUG(ic->proc_id, "Icache mem req on op_num:%s @ 0x%s\n", unsstr64(op_count[ic->proc_id]),
        hexstr64s(ic->fetch_addr));
  Flag success = mem_req_on_icache_miss();
  if (success) {
    ic->icache_miss_fulfilled = FALSE;
    *break_fetch = BREAK_ICACHE_MISS_REQ_SUCCESS;
    return ICACHE_WAIT_FOR_MISS;
  } else {
    *break_fetch = BREAK_ICACHE_MISS_REQ_FAILURE;
    return ICACHE_MEM_REQ;
  }
}

Icache_State icache_wait_for_miss_actions(Break_Reason* break_fetch) {
  ASSERT(ic->proc_id, ft_op_buffer_can_fetch_op(ic));
  DEBUG(ic->proc_id, "Ifetch barrier: Waiting for miss \n");
  if (!ic->off_path) {
    INC_STAT_EVENT(ic->proc_id, INST_LOST_WAIT_FOR_ICACHE_MISS_NOT_PREFETCHED + get_last_miss_reason(ic->proc_id),
                   ic->sd.max_op_count);
  }
  *break_fetch = BREAK_ICACHE_WAIT_FOR_MISS;
  if (ic->icache_miss_fulfilled) {
    return ICACHE_SERVING;
  } else {
    return ICACHE_WAIT_FOR_MISS;
  }
}

// fill in the icache stage data with current FT ops buffered in ft_op_buffer
// return if FT has ended
// if true, the requested number of ops might not be fulfilled
Flag fill_icache_stage_data(int requested, Stage_Data* sd) {
  ASSERT(ic->proc_id, requested && requested <= sd->max_op_count - sd->op_count);
  ASSERT(ic->proc_id, ft_op_buffer_can_fetch_op(ic));

  while (requested && ft_op_buffer_can_fetch_op(ic)) {
    sd->ops[sd->op_count] = ft_op_buffer_pop(ic);
    sd->op_count++;
    requested--;
  }
  Flag ft_has_ended = !ft_op_buffer_can_fetch_op(ic);
  return ft_has_ended;
}

void icache_serve_ops() {
  uns op_num_prev_fetch_target = ic->sd.op_count;

  // ic->line should have already been set correctly
  ASSERT(ic->proc_id, ic->line);
  ASSERT(ic->proc_id, ic->line_addr);

  int requested = ic->sd.max_op_count - ic->sd.op_count;
  Flag ft_has_ended = fill_icache_stage_data(requested, &ic->sd);
  ASSERT(ic->proc_id, ic->sd.op_count == ic->sd.max_op_count || ft_has_ended);

  if (ft_has_ended) {
    ASSERT(ic->proc_id, !ft_op_buffer_can_fetch_op(ic));
  } else {
    ASSERT(ic->proc_id, ft_op_buffer_can_fetch_op(ic));
    ASSERT(ic->proc_id, ic->sd.op_count == ic->sd.max_op_count);
  }

  // process the fetched ops
  icache_process_ops(&ic->sd, FALSE, op_num_prev_fetch_target);
}

Icache_State icache_serving_actions(Break_Reason* break_fetch) {
  STAT_EVENT(ic->proc_id, ICACHE_SERVING_CYCLE_ON_PATH + ic->off_path);
  ASSERT(ic->proc_id, !ic->fetch_barrier_pending);
  if (ic->sd.op_count) {
    *break_fetch = BREAK_ICACHE_STALLED;
    return ICACHE_SERVING;
  } else {
    if (uc && uc->sd.op_count) {
      *break_fetch = BREAK_UOP_CACHE_STALLED;
      return ICACHE_SERVING;
    }
  }

  // ft_can_fetch_op denotes if the fetched ft has more uops,
  // or equivalently, if the lookup buffer is occupied;
  // for the legacy design, the buffer should be occupied,
  // either by an ft from a previous cycle or this cycle,
  // otherwise icache_serving_actions should not be called.
  ASSERT(ic->proc_id, ft_op_buffer_can_fetch_op(ic));
  // unless ICACHE_FETCH_ACROSS_FETCH_TARGET is turned on,
  // to determine the availability of read ports,
  // we need to consider if the buffer is occupied by an ft from a previous cycle;
  // it is true if there is no lookup in the current cycle but the buffer is occupied.
  int occupied_lookup_buffer = 0;
  if (!ICACHE_FETCH_ACROSS_FETCH_TARGET && ic->lookups_per_cycle_count == 0 && ft_op_buffer_can_fetch_op(ic)) {
    occupied_lookup_buffer = 1;
  }
  while (ic->sd.op_count < ic->sd.max_op_count) {
    if (ft_op_buffer_can_fetch_op(ic)) {
      icache_serve_ops();
    } else if (ic->fetch_barrier_pending) {
      break;
    } else if (ic->lookups_per_cycle_count + occupied_lookup_buffer < ICACHE_READ_PORTS) {
      FT_Arbitration_Result result = ft_arbitration();
      switch (result) {
        case FT_UNAVAILABLE:
          *break_fetch = BREAK_FT_UNAVAILABLE;
          return ICACHE_STAGE_RESTEER;
        case FT_MISS_BOTH:
          return icache_mem_req_actions(break_fetch);
        case FT_HIT_ICACHE:
          icache_serve_ops();
          break;
        case FT_HIT_UOP_CACHE:
          *break_fetch = BREAK_ICACHE_TO_UOP_CACHE_SWITCH;
          return UOP_CACHE_SERVING;
        default:
          ASSERT(ic->proc_id, 0);
      }
    } else {
      *break_fetch = BREAK_ICACHE_READ_LIMIT;
      return ICACHE_STAGE_RESTEER;
    }
  }

  if (ft_op_buffer_can_fetch_op(ic)) {
    ASSERT(ic->proc_id, ic->sd.op_count == ic->sd.max_op_count);
    *break_fetch = BREAK_ICACHE_ISSUE_WIDTH;
    return ICACHE_SERVING;
  } else if (ic->fetch_barrier_pending) {
    *break_fetch = BREAK_ICACHE_STALLED;
    return ICACHE_STALLED;
  } else {
    ASSERT(ic->proc_id, ic->sd.op_count == ic->sd.max_op_count);
    *break_fetch = BREAK_ICACHE_ISSUE_WIDTH;
    return ICACHE_STAGE_RESTEER;
  }
}

void uop_cache_serve_ops() {
  uns op_num_prev_fetch_target = uc->sd.op_count;

  uns requested = uc->sd.max_op_count - uc->sd.op_count;
  Uop_Cache_Data uop_cache_line = uop_cache_consume_uops_from_lookup_buffer(requested);
  // the line must be valid
  ASSERT(ic->proc_id, uop_cache_line.n_uops);
  ASSERT(ic->proc_id, uop_cache_line.n_uops <= requested);

  Flag ft_has_ended = fill_icache_stage_data(uop_cache_line.n_uops, &uc->sd);
  // the ft should provide exactly the same amount of uops as in the uop cache line
  ASSERT(ic->proc_id, uop_cache_line.n_uops == uc->sd.op_count - op_num_prev_fetch_target);

  if (ft_has_ended) {
    ASSERT(ic->proc_id, !ft_op_buffer_can_fetch_op(ic));
    // sanity check that the uop cache is in sync
    ASSERT(ic->proc_id, uop_cache_line.end_of_ft);
    uop_cache_clear_lookup_buffer();
  } else {
    ASSERT(ic->proc_id, ft_op_buffer_can_fetch_op(ic));
    ASSERT(ic->proc_id, !uop_cache_line.end_of_ft);
  }

  // process the fetched ops
  icache_process_ops(&uc->sd, TRUE, op_num_prev_fetch_target);
  DEBUG(ic->proc_id, "ic_uopc filled added:%u sd_head:%s sd_tail:%s sd_count:%d ft_ended:%u end_of_ft:%u\n",
        uop_cache_line.n_uops, sd_head_opnum_str(&uc->sd), sd_tail_opnum_str(&uc->sd), uc->sd.op_count, ft_has_ended,
        uop_cache_line.end_of_ft);
}

Icache_State uop_cache_serving_actions(Break_Reason* break_fetch) {
  ASSERT(ic->proc_id, UOP_CACHE_ENABLE);
  STAT_EVENT(ic->proc_id, UCACHE_SERVING_CYCLE_ON_PATH + ic->off_path);
  ASSERT(ic->proc_id, !ic->fetch_barrier_pending);

  if (ic->sd.op_count) {
    *break_fetch = BREAK_ICACHE_STALLED;
    return UOP_CACHE_SERVING;
  } else if (uc->sd.op_count) {
    *break_fetch = BREAK_UOP_CACHE_STALLED;
    return UOP_CACHE_SERVING;
  }

  // ft_can_fetch_op denotes if the fetched ft has more uops,
  // or equivalently, if the lookup buffer is occupied;
  // for the legacy design, the buffer should be occupied,
  // either by an ft from a previous cycle or this cycle,
  // otherwise uop_cache_serving_actions should not be called.
  ASSERT(ic->proc_id, ft_op_buffer_can_fetch_op(ic));
  // unless UOP_CACHE_FETCH_ACROSS_FETCH_TARGET is turned on,
  // to determine the availability of read ports,
  // we need to consider if the buffer is occupied by an ft from a previous cycle;
  // it is true if there is no lookup in the current cycle but the buffer is occupied.
  int occupied_lookup_buffer = 0;
  if (!UOP_CACHE_FETCH_ACROSS_FETCH_TARGET && uc->lookups_per_cycle_count == 0 && ft_op_buffer_can_fetch_op(ic)) {
    occupied_lookup_buffer = 1;
  }
  while (uc->sd.op_count < uc->sd.max_op_count) {
    if (ft_op_buffer_can_fetch_op(ic)) {
      uop_cache_serve_ops();
    } else if (ic->fetch_barrier_pending) {
      break;
    } else if (uc->lookups_per_cycle_count + occupied_lookup_buffer < UOP_CACHE_READ_PORTS) {
      FT_Arbitration_Result result = ft_arbitration();
      switch (result) {
        case FT_UNAVAILABLE:
          *break_fetch = BREAK_FT_UNAVAILABLE;
          return ICACHE_STAGE_RESTEER;
        case FT_MISS_BOTH:
          return icache_mem_req_actions(break_fetch);
        case FT_HIT_ICACHE:
          *break_fetch = BREAK_UOP_CACHE_TO_ICACHE_SWITCH;
          return ICACHE_SERVING;
        case FT_HIT_UOP_CACHE:
          uop_cache_serve_ops();
          break;
        default:
          ASSERT(ic->proc_id, 0);
      }
    } else {
      *break_fetch = BREAK_UOP_CACHE_READ_LIMIT;
      return ICACHE_STAGE_RESTEER;
    }
  }

  if (ft_op_buffer_can_fetch_op(ic)) {
    ASSERT(ic->proc_id, uc->sd.op_count == uc->sd.max_op_count);
    *break_fetch = BREAK_UOP_CACHE_ISSUE_WIDTH;
    return UOP_CACHE_SERVING;
  } else if (ic->fetch_barrier_pending) {
    *break_fetch = BREAK_ICACHE_STALLED;
    return ICACHE_STALLED;
  } else {
    ASSERT(ic->proc_id, uc->sd.op_count == uc->sd.max_op_count);
    *break_fetch = BREAK_UOP_CACHE_ISSUE_WIDTH;
    return ICACHE_STAGE_RESTEER;
  }
}

void execute_coupled_FSM() {
  STAT_EVENT(ic->proc_id, ICACHE_CYCLE);
  STAT_EVENT(ic->proc_id, ICACHE_CYCLE_ONPATH + ic->off_path);
  if (ic->off_path)
    STAT_EVENT(ic->proc_id, ICACHE_STAGE_OFF_PATH);

  ic->off_path &= !ic->back_on_path;
  ic->back_on_path = FALSE;
  STAT_EVENT(ic->proc_id, FETCH_ON_PATH + ic->off_path);

  Break_Reason break_fetch = BREAK_DONT;
  ic->state = ic->next_state;
  DEBUG(ic->proc_id, "Icache state: %i\n", ic->state);
  if (ic->icache_stage_resteer_signaled) {
    ic->icache_stage_resteer_signaled = FALSE;
    ic->next_state = ICACHE_STAGE_RESTEER;
    break_fetch = BREAK_ICACHE_STAGE_RESTEER;
  } else if (ic->state == ICACHE_STAGE_RESTEER) {
    ASSERT(ic->proc_id, !ft_op_buffer_can_fetch_op(ic));

    FT_Arbitration_Result result = ft_arbitration();
    switch (result) {
      case FT_UNAVAILABLE:
        ic->next_state = ICACHE_STAGE_RESTEER;
        break_fetch = BREAK_FT_UNAVAILABLE;
        break;
      case FT_MISS_BOTH:
        ic->next_state = icache_mem_req_actions(&break_fetch);
        break;
      case FT_HIT_ICACHE:
        ic->next_state = icache_serving_actions(&break_fetch);
        break;
      case FT_HIT_UOP_CACHE:
        ic->next_state = uop_cache_serving_actions(&break_fetch);
        break;
      default:
        ASSERT(ic->proc_id, 0);
    }
  } else if (ic->state == ICACHE_MEM_REQ) {
    ic->next_state = icache_mem_req_actions(&break_fetch);
  } else if (ic->state == ICACHE_WAIT_FOR_MISS) {
    ic->next_state = icache_wait_for_miss_actions(&break_fetch);
  } else if (ic->state == ICACHE_SERVING) {
    ic->next_state = icache_serving_actions(&break_fetch);
  } else if (ic->state == UOP_CACHE_SERVING) {
    ic->next_state = uop_cache_serving_actions(&break_fetch);
  } else if (ic->state == ICACHE_STALLED) {
    ic->next_state = ic->fetch_barrier_pending ? ICACHE_STALLED : ICACHE_STAGE_RESTEER;
    break_fetch = ic->fetch_barrier_pending ? BREAK_ICACHE_STALLED : BREAK_ICACHE_STAGE_RESTEER;
  } else {
    ASSERT(ic->proc_id, 0);
  }
  ASSERT(ic->proc_id, break_fetch != BREAK_DONT);
  if (UOP_CACHE_ENABLE) {
    ASSERT(ic->proc_id, ic->sd.op_count * uc->sd.op_count == 0);
    DEBUG(ic->proc_id,
          "IC/UOC stage snapshot ic_head:%s ic_tail:%s ic_count:%d uopc_head:%s uopc_tail:%s uopc_count:%d\n",
          sd_head_opnum_str(&ic->sd), sd_tail_opnum_str(&ic->sd), ic->sd.op_count, sd_head_opnum_str(&uc->sd),
          sd_tail_opnum_str(&uc->sd), uc->sd.op_count);
  }

  Stage_Data* cur_data = get_current_stage_data();
  INC_STAT_EVENT(ic->proc_id, INST_LOST_TOTAL, cur_data->max_op_count);
  INC_STAT_EVENT(ic->proc_id, INST_LOST_BREAK_DONT + break_fetch,
                 cur_data->max_op_count > cur_data->op_count ? cur_data->max_op_count - cur_data->op_count : 0);
  STAT_EVENT(ic->proc_id, FETCH_0_OPS + cur_data->op_count);
  STAT_EVENT(ic->proc_id, ST_BREAK_DONT + break_fetch);
  if (!ic->off_path) {
    if (break_fetch == BREAK_UOP_CACHE_STALLED || break_fetch == BREAK_ICACHE_STALLED) {
      INC_STAT_EVENT(ic->proc_id, INST_LOST_FULL_WINDOW + inst_lost_get_full_window_reason(), cur_data->max_op_count);
      STAT_EVENT(ic->proc_id, ICACHE_STAGE_STALLED);
    } else {
      STAT_EVENT(ic->proc_id, ICACHE_STAGE_NOT_STALLED);
    }
    if (!cur_data->op_count) {
      STAT_EVENT(ic->proc_id, ICACHE_STAGE_STARVED);
    } else {
      STAT_EVENT(ic->proc_id, ICACHE_STAGE_NOT_STARVED);
    }
  }
}

/**************************************************************************************/
/* icache_cycle: */

void update_icache_stage() {
  ic->lookups_per_cycle_count = 0;
  if (UOP_CACHE_ENABLE) {
    uc->lookups_per_cycle_count = 0;
  }

  execute_coupled_FSM();
}

/**************************************************************************************/
/* icache_process_ops: process fetched ops.
 * In one cycle, icache_process_ops can be executed multiple times, once per fetch target.
 *
 * For example, a small FT could not fill up the issue width; if there are enough icache/uopc ports,
 * we would fetch more FTs in the same cycle, and call icache_process_ops more than once.
 *
 * start_idx indicates the op index the processing should start at.
 */
static inline void icache_process_ops(Stage_Data* cur_data, Flag fetched_from_uop_cache, uns start_idx) {
  for (uns ii = start_idx; ii < cur_data->op_count; ii++) {
    Op* op = cur_data->ops[ii];

    if (fetched_from_uop_cache) {
      uc->sd.ops[ii]->fetched_from_uop_cache = TRUE;
    }

    ASSERTM(ic->proc_id, ic->off_path == op->off_path, "Inconsistent off-path op PC: %llx ic:%i op:%i\n",
            op->inst->addr, ic->off_path, op->off_path);

    if (!op->off_path) {
      STAT_EVENT(ic->proc_id, UOPS_SERVED_BY_ICACHE_ON_PATH + op->fetched_from_uop_cache);
    } else {
      STAT_EVENT(ic->proc_id, UOPS_SERVED_BY_ICACHE_OFF_PATH + op->fetched_from_uop_cache);
    }

    if (!op->off_path && (op->uop->mem_type == MEM_LD || op->uop->mem_type == MEM_ST) && op->oracle_info.va == 0) {
      // don't care if the va is 0x0 if mem_type is MEM_PF(SW prefetch),
      // MEM_WH(write hint), or MEM_EVICT(cache block eviction hint)
      print_func_op(op);
      FATAL_ERROR(ic->proc_id, "Access to 0x0\n");
    }

    if (DUMP_TRACE && DEBUG_RANGE_COND(ic->proc_id))
      print_func_op(op);

    if (DIE_ON_CALLSYS && !op->off_path) {
      ASSERT(ic->proc_id, op->uop->cf_type != CF_SYS);
    }

    STAT_EVENT(op->proc_id, FETCH_ALL_INST);
    STAT_EVENT(op->proc_id, ORACLE_ON_PATH_INST + op->off_path);
    STAT_EVENT(op->proc_id, ORACLE_ON_PATH_INST_MEM + (op->uop->mem_type == NOT_MEM) + 2 * op->off_path);

    op_set_fetch_cycle(op, cycle_count);

    if (is_fetch_barrier_op(op)) {
      ASSERT(ic->proc_id, !ic->fetch_barrier_pending);
      ASSERT(ic->proc_id, ii == cur_data->op_count - 1);
      ic->fetch_barrier_pending = TRUE;
      ic->fetch_barrier_inst_uid = op->inst_uid;
      ic->fetch_barrier_op_num = op->op_num;
      DEBUG(ic->proc_id, "Fetch barrier encountered op_num:%s inst_uid:%llu\n", unsstr64(op->op_num),
            (uns64)op->inst_uid);
    }

    op_count[ic->proc_id]++; /* increment instruction counters */
    unique_count_per_core[ic->proc_id]++;
    unique_count++;
    /* check trigger */
    if (op->uop->trigger_op_fetched_hook)
      model->op_fetched_hook(op);

    INC_STAT_EVENT(ic->proc_id, INST_LOST_FETCH + ic->off_path, 1);

    DEBUG(ic->proc_id,
          "Fetching op from Icache addr: %s off: %d inst_info: %p ii_addr: %s "
          "dis: %s opnum: (%s:%s)\n",
          hexstr64s(op->inst->addr), op->off_path, (void*)op->inst, hexstr64s(op->inst->addr), disasm_op(op, TRUE),
          unsstr64(op->op_num), unsstr64(op->unique_num));

    if (op->uop->cf_type) {
      // TODO: can we move this prefetch update to decoupled front-end or need it be here?
      if (DJOLT_ENABLE)
        update_djolt(ic->proc_id, op->inst->addr, op->uop->cf_type, op->bp_pred_info->pred_npc);

      ASSERT(ic->proc_id, (((op->bp_pred_info->recovery_point == RECOVER_AT_EXEC) << 2) |
                           ((op->bp_pred_info->recovery_point == RECOVER_AT_DECODE) << 1) |
                           btb_pred_miss(op->btb_pred_info)) <= 0x7);

      ic->off_path = ic->off_path || (op->bp_pred_info->recovery_point == RECOVER_AT_FE) ||
                     op->bp_pred_info->recovery_point == RECOVER_AT_DECODE ||
                     op->bp_pred_info->recovery_point == RECOVER_AT_EXEC;

      // Measuring basic block lengths
      /*static int bbl_len = 0;
      static int bbl_len_dont_end_pred_nt = 0;
      bbl_len++;
      bbl_len_dont_end_pred_nt++;
      if (op->uop->cf_type) {
        STAT_EVENT(ic->proc_id, BBL_LENGTH_1 + bbl_len-1);
        bbl_len = 0;
        if (op->bp_pred_info->pred == TAKEN) {
          STAT_EVENT(ic->proc_id, BBL_DONT_END_PRED_NT_LENGTH_1 + bbl_len_dont_end_pred_nt-1);
          bbl_len_dont_end_pred_nt = 0;
        }
      }*/
    } else {
      // pass the global branch history to all the instructions
      op->bp_pred_l0.pred_global_hist = g_bp_data->global_hist;
      op->bp_pred_main.pred_global_hist = g_bp_data->global_hist;
    }
  }
}

/**************************************************************************************/
/* icache_demote_victim: */
/* Exclusive hierarchy: the line this fill is about to evict moves down a level
   instead of vanishing. Without this, instruction lines -- which skip the MLC and
   LLC on fill (see mlc_fill_line/l1_fill_line) -- have no backing store at all. */

static inline void icache_demote_victim(Addr fill_addr) {
  Addr repl_line_addr;
  Flag repl_line_valid;

  if (!EXCLUSIVE_CACHES)
    return;

  get_next_repl_line(&ic->icache, ic->proc_id, fill_addr, &repl_line_addr, &repl_line_valid);
  if (repl_line_valid)
    mem_demote_to_mlc(get_proc_id_from_cmp_addr(repl_line_addr), repl_line_addr, FALSE, FALSE, FALSE);
}

/**************************************************************************************/
/* icache_fill_line: */

Flag icache_fill_line(Mem_Req* req)  // cmp FIXME maybe needed to be optimized
{
  Addr repl_line_addr;
  Addr repl_line_addr2;
  Inst_Info** line;
  Addr dummy_addr;
  Addr dummy_addr2;
  Icache_Data* line_info = NULL;
  UNUSED(line);

  // cmp
  if (model->id == CMP_MODEL) {
    set_icache_stage(&cmp_model.icache_stage[req->proc_id]);
    set_fdip(req->proc_id, 0);
  }

  ASSERT(ic->proc_id, ic->proc_id == req->proc_id);

  if (req->dirty_l0) {
    STAT_EVENT(ic->proc_id, DIRTY_WRITE_TO_ICACHE);
    printf("fetch_addr:%s line_addr:%s req_addr:%s off:%d\n", hexstr64s(ic->fetch_addr), hexstr64s(ic->line_addr),
           hexstr64s(req->addr), ic->off_path);
  }

  /* get new line in the cache */
  if ((ic->line_addr == req->addr) && ic->next_state == ICACHE_WAIT_FOR_MISS) {
    INC_STAT_EVENT(ic->proc_id, MISS_WAIT_TIME, cycle_count - ic->wait_for_miss_start);
    if (IC_PREF_CACHE_ENABLE &&  // cmp FIXME prefetchers
        (USE_CONFIRMED_OFF ? req->off_path_confirmed : req->off_path)) {
      Addr pref_line_addr;

      line = (Inst_Info**)cache_insert(&ic->pref_icache, ic->proc_id, ic->fetch_addr, &pref_line_addr, &repl_line_addr);
      DEBUG(ic->proc_id, "Insert PREF_ICACHE fetch_addr0x:%s line_addr:%s index:%ld addr:0x%s\n",
            hexstr64(ic->fetch_addr), hexstr64(pref_line_addr), (long int)(req - mem->req_buffer),
            hexstr64s(req->addr));
      STAT_EVENT(ic->proc_id, IC_PREF_CACHE_FILL);
      ic->icache_miss_fulfilled = TRUE;
      return TRUE;
    }

    icache_demote_victim(ic->fetch_addr);
    ic->line = (Inst_Info**)cache_insert(&ic->icache, ic->proc_id, ic->fetch_addr, &ic->line_addr, &repl_line_addr);
    DEBUG(ic->proc_id, "Got line switch into ic fetch %llx\n", ic->line_addr);
    STAT_EVENT(ic->proc_id, ICACHE_FILL);

    if (WP_COLLECT_STATS) {  // cmp IGNORE
      line_info = (Icache_Data*)cache_insert(&ic->icache_line_info, ic->proc_id, ic->fetch_addr, &dummy_addr2,
                                             &repl_line_addr2);
      if (line_info) {
        wp_process_icache_evicted(line_info, req, &repl_line_addr2);
        if (EIP_ENABLE)
          eip_cache_fill(ic->proc_id, req->addr, repl_line_addr2);
        line_info->fetched_by_offpath = USE_CONFIRMED_OFF ? req->off_path_confirmed : req->off_path;
        line_info->offpath_op_addr = req->oldest_op_addr;
        line_info->offpath_op_unique = req->oldest_op_unique_num;
        line_info->fetch_cycle = cycle_count;
        line_info->onpath_use_cycle = req->off_path ? 0 : cycle_count;
        line_info->read_count[0] = req->cyc_hit_by_demand_load ? 1 : 0;
        line_info->read_count[1] = 0;
        line_info->HW_prefetch = req->type == MRT_IPRF;
        line_info->ghist = req->ghist;
        if (mem_req_is_type(req, MRT_FDIPPRFON) || mem_req_is_type(req, MRT_FDIPPRFOFF)) {
          if (req->fdip_pref_off_path == 2)
            line_info->FDIP_prefetch = FDIP_BOTHPATH;
          else if (req->fdip_pref_off_path == 1)
            line_info->FDIP_prefetch = FDIP_OFFPATH;
          else
            line_info->FDIP_prefetch = FDIP_ONPATH;
        } else {
          line_info->FDIP_prefetch = DEMAND_LOAD;
        }
        wp_process_icache_fill(line_info, req);
      }
    }

    STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ);
    ic->icache_miss_fulfilled = TRUE;

    if (req->demand_icache_emitted_cycle) {
      ASSERT(ic->proc_id, !req->fdip_emitted_cycle && (cycle_count - req->demand_icache_emitted_cycle > 0));
      STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_BY_ICACHE_DEMAND);
      INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_CYCLE_DELTA_BY_ICACHE_DEMAND,
                     cycle_count - req->demand_icache_emitted_cycle);
    } else if (req->fdip_emitted_cycle) {
      ASSERT(ic->proc_id, !req->demand_icache_emitted_cycle);
      STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_BY_FDIP);
      INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_CYCLE_DELTA_BY_FDIP, cycle_count - req->fdip_emitted_cycle);
      if (req->cyc_hit_by_demand_load) {
        STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_BY_ON_FDIP_HIT_BY_DEMAND_LOAD + req->fdip_pref_off_path);
        INC_STAT_EVENT(ic->proc_id,
                       ICACHE_FILL_CORRECT_REQ_CYCLE_DELTA_BY_ON_FDIP_HIT_BY_DEMAND_LOAD + req->fdip_pref_off_path,
                       cycle_count - req->cyc_hit_by_demand_load);
      }
    }
    INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_CYCLE_DELTA, cycle_count - req->demand_icache_emitted_cycle);
  } else {
    if (IC_PREF_CACHE_ENABLE &&  // cmp FIXME prefetchers
        (USE_CONFIRMED_OFF ? req->off_path_confirmed : req->off_path)) {
      Addr pref_line_addr;

      line = (Inst_Info**)cache_insert(&ic->pref_icache, ic->proc_id, req->addr, &pref_line_addr, &repl_line_addr);
      DEBUG(ic->proc_id, "Insert PREF_ICACHE fetch_addr0x:%s line_addr:%s index:%ld addr:0x%s\n", hexstr64(req->addr),
            hexstr64(pref_line_addr), (long int)(req - mem->req_buffer), hexstr64s(req->addr));
      STAT_EVENT(ic->proc_id, IC_PREF_CACHE_FILL);

      return TRUE;
    }

    icache_demote_victim(req->addr);
    line = (Inst_Info**)cache_insert(&ic->icache, ic->proc_id, req->addr, &dummy_addr, &repl_line_addr);

    if (WP_COLLECT_STATS) {  // cmp IGNORE
      line_info =
          (Icache_Data*)cache_insert(&ic->icache_line_info, ic->proc_id, req->addr, &dummy_addr2, &repl_line_addr2);
      if (line_info) {
        STAT_EVENT(ic->proc_id, ICACHE_FILL);

        wp_process_icache_evicted(line_info, req, &repl_line_addr2);
        if (EIP_ENABLE)
          eip_cache_fill(ic->proc_id, req->addr, repl_line_addr2);
        line_info->fetched_by_offpath = USE_CONFIRMED_OFF ? req->off_path_confirmed : req->off_path;
        line_info->offpath_op_addr = req->oldest_op_addr;
        line_info->offpath_op_unique = req->oldest_op_unique_num;
        line_info->fetch_cycle = cycle_count;
        line_info->onpath_use_cycle = req->off_path ? 0 : cycle_count;
        line_info->read_count[0] = 0;
        line_info->read_count[1] = 0;
        line_info->HW_prefetch = req->type == MRT_IPRF;
        line_info->ghist = req->ghist;
        if (mem_req_is_type(req, MRT_FDIPPRFON) || mem_req_is_type(req, MRT_FDIPPRFOFF)) {
          if (req->fdip_pref_off_path == 2)
            line_info->FDIP_prefetch = FDIP_BOTHPATH;
          else if (req->fdip_pref_off_path == 1)
            line_info->FDIP_prefetch = FDIP_OFFPATH;
          else
            line_info->FDIP_prefetch = FDIP_ONPATH;
        } else {
          line_info->FDIP_prefetch = DEMAND_LOAD;
        }
        wp_process_icache_fill(line_info, req);
      }
    }

    STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ);
    if (req->demand_icache_emitted_cycle) {
      ASSERT(ic->proc_id, !req->fdip_emitted_cycle);
      STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_BY_ICACHE_DEMAND);
      INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_CYCLE_DELTA_BY_ICACHE_DEMAND,
                     cycle_count - req->demand_icache_emitted_cycle);
    } else if (req->fdip_emitted_cycle) {
      ASSERT(ic->proc_id, !req->demand_icache_emitted_cycle);
      STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_BY_FDIP);
      INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_CYCLE_DELTA_BY_FDIP, cycle_count - req->fdip_emitted_cycle);
      if (req->cyc_hit_by_demand_load) {
        STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_BY_ON_FDIP_HIT_BY_DEMAND_LOAD + req->fdip_pref_off_path);
        INC_STAT_EVENT(ic->proc_id,
                       ICACHE_FILL_INCORRECT_REQ_CYCLE_DELTA_BY_ON_FDIP_HIT_BY_DEMAND_LOAD + req->fdip_pref_off_path,
                       cycle_count - req->cyc_hit_by_demand_load);
      }
    }
    INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_CYCLE_DELTA, cycle_count - req->demand_icache_emitted_cycle);
  }

  return TRUE;
}

/**************************************************************************************/
/* icache_off_path: */

inline Flag icache_off_path(void) {
  return ic->off_path;
}

/*************************************************************************************/
/* ic_pref_cace_access() */

Inst_Info** ic_pref_cache_access(void) {
  Addr repl_line_addr, inval_line_addr;
  Inst_Info** inserted_line = NULL;

  ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->fetch_addr)
  Inst_Info** line = (Inst_Info**)cache_access(&ic->pref_icache, ic->fetch_addr, &ic->line_addr, FALSE);

  if (ic->off_path && !PREFCACHE_MOVE_OFFPATH) {
    if (line) {
      DEBUG(ic->proc_id, "off_path ic_pref cache hit:fetch_addr:0x%s \n", hexstr64(ic->fetch_addr));
      STAT_EVENT(ic->proc_id, IC_PREF_CACHE_HIT_PER_OFFPATH);
      STAT_EVENT(ic->proc_id, IC_PREF_CACHE_HIT_OFFPATH);
    }
    return line;
  }

  if (line) {
    icache_demote_victim(ic->fetch_addr);
    inserted_line =
        (Inst_Info**)cache_insert(&ic->icache, ic->proc_id, ic->fetch_addr, &ic->line_addr, &repl_line_addr);
    DEBUG(ic->proc_id, "ic_pref cache hit:fetch_addr:0x%s \n", hexstr64(ic->fetch_addr));
    STAT_EVENT(ic->proc_id, IC_PREF_MOVE_IC);

    STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ);

    STAT_EVENT(ic->proc_id, IC_PREF_CACHE_HIT_PER + MIN2(ic->off_path, 1));
    STAT_EVENT(ic->proc_id, IC_PREF_CACHE_HIT + MIN2(ic->off_path, 1));
    cache_invalidate(&ic->pref_icache, ic->fetch_addr, &inval_line_addr);

    if (PREF_ICACHE_HIT_FILL_L1) {
      if (model->mem == MODEL_MEM) {
        Addr line_addr;
        Cache* l1_cache = &mem->uncores[ic->proc_id].l1->cache;
        L1_Data* l1_data = (L1_Data*)cache_access(l1_cache, ic->fetch_addr, &line_addr, TRUE);
        if (!l1_data) {
          Mem_Req tmp_req;
          tmp_req.addr = ic->fetch_addr;
          tmp_req.off_path = FALSE;
          tmp_req.op_count = 0;
          FATAL_ERROR(0, "This fill code is wrong. Writebacks may be lost.");
          l1_fill_line(&tmp_req);
          STAT_EVENT(ic->proc_id, IC_PREF_MOVE_L1);
        }
      }
    }
  }

  return inserted_line;
}

/**************************************************************************************/
/* Stat Inline Functions */

static inline void wp_process_icache_hit(Icache_Data* line, Addr fetch_addr) {
  L1_Data* l1_line;

  if (!WP_COLLECT_STATS)
    return;

  if (icache_off_path() == FALSE) {
    inc_icache_hit(ic->line_addr);
    if (line->fetched_by_offpath) {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_ONPATH_SAT_BY_OFFPATH);
      STAT_EVENT(ic->proc_id, ICACHE_USE_OFFPATH);
      STAT_EVENT(ic->proc_id, DIST_ICACHE_FILL_OFFPATH_USED);
      STAT_EVENT(ic->proc_id, DIST_REQBUF_OFFPATH_USED);
      STAT_EVENT(ic->proc_id, DIST2_REQBUF_OFFPATH_USED_FULL);

      l1_line = do_l1_access_addr(fetch_addr);
      if (l1_line) {
        if (l1_line->fetched_by_offpath) {
          STAT_EVENT(ic->proc_id, L1_USE_OFFPATH);
          STAT_EVENT(ic->proc_id, DIST_L1_FILL_OFFPATH_USED);
          STAT_EVENT(ic->proc_id, L1_USE_OFFPATH_IFETCH);
          l1_line->fetched_by_offpath = FALSE;
          l1_line->l0_modified_fetched_by_offpath = TRUE;
        }
      }
    } else {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_ONPATH_SAT_BY_ONPATH);
      STAT_EVENT(ic->proc_id, ICACHE_USE_ONPATH);
    }
  } else {
    if (line->fetched_by_offpath) {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_OFFPATH_SAT_BY_OFFPATH);
    } else {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_OFFPATH_SAT_BY_ONPATH);
    }
  }

  if (!line->read_count[0]) {  // only consider the first hit
    if (!icache_off_path()) {
      uns64 hashed_addr = FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(ic->line_addr, line->ghist) : ic->line_addr;
      if (fdip_search_pref_candidate(ic->line_addr)) {
        inc_cnt_useful(ic->proc_id, hashed_addr, FALSE);
        inc_cnt_useful_signed(hashed_addr);
        inc_useful_lines_uc(hashed_addr);
        update_useful_lines_uc(hashed_addr);
        update_useful_lines_bloom_filter(hashed_addr);
        inc_utility_info(TRUE);
        inc_timeliness_info(FALSE);
      }
      if (line->FDIP_prefetch == FDIP_BOTHPATH || line->FDIP_prefetch == FDIP_ONPATH)
        STAT_EVENT(ic->proc_id, ICACHE_HIT_BY_FDIP_ONPATH);
      else if (line->FDIP_prefetch == FDIP_OFFPATH)
        STAT_EVENT(ic->proc_id, ICACHE_HIT_BY_FDIP_OFFPATH);
      line->read_count[0] += 1;
    }
    if (line->FDIP_prefetch)
      STAT_EVENT(ic->proc_id, ICACHE_HIT_ONPATH_BY_FDIP + icache_off_path());
  }

  if (icache_off_path() == FALSE)
    line->fetched_by_offpath = FALSE;
}

static inline void wp_process_icache_evicted(Icache_Data* line, Mem_Req* req, Addr* repl_line_addr) {
  if (!WP_COLLECT_STATS)
    return;

  if (*repl_line_addr && !line->read_count[0]) {
    DEBUG(ic->proc_id, "%llx is evicted without hit, FDIP pref: %d\n", *repl_line_addr, line->FDIP_prefetch);
    INC_STAT_EVENT(ic->proc_id, ICACHE_UNUSEFUL_CL_CYC, cycle_count - line->fetch_cycle);
    STAT_EVENT(ic->proc_id, ICACHE_UNUSEFUL_CL);
    if (line->FDIP_prefetch && (FDIP_UTILITY_ONLY_TRAIN_OFF_PATH ? line->FDIP_prefetch >= FDIP_OFFPATH : TRUE)) {
      uns64 hashed_addr = FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(*repl_line_addr, line->ghist) : *repl_line_addr;
      inc_cnt_unuseful(ic->proc_id, hashed_addr);
      dec_cnt_useful_signed(hashed_addr);
      dec_useful_lines_uc(hashed_addr);
      update_unuseful_lines_uc(hashed_addr);
      inc_utility_info(FALSE);
      STAT_EVENT(ic->proc_id, ICACHE_EVICT_MISS_ONPATH_BY_FDIP + icache_off_path());
      if (line->FDIP_prefetch == FDIP_BOTHPATH || line->FDIP_prefetch == FDIP_ONPATH)
        STAT_EVENT(ic->proc_id, ICACHE_EVICT_MISS_BY_FDIP_ONPATH);
      else
        STAT_EVENT(ic->proc_id, ICACHE_EVICT_MISS_BY_FDIP_OFFPATH);
    }
  } else if (*repl_line_addr && line->read_count[0]) {
    DEBUG(ic->proc_id, "%llx is evicted with hits, FDIP pref: %d\n", *repl_line_addr, line->FDIP_prefetch);
    if (line->FDIP_prefetch) {
      uns64 hashed_addr = FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(*repl_line_addr, line->ghist) : *repl_line_addr;
      add_evict_seq(hashed_addr);
      STAT_EVENT(ic->proc_id, ICACHE_EVICT_HIT_ONPATH_BY_FDIP + icache_off_path());
      if (line->FDIP_prefetch == FDIP_BOTHPATH || line->FDIP_prefetch == FDIP_ONPATH)
        STAT_EVENT(ic->proc_id, ICACHE_EVICT_HIT_BY_FDIP_ONPATH);
      else
        STAT_EVENT(ic->proc_id, ICACHE_EVICT_HIT_BY_FDIP_OFFPATH);
    }
  }

  if (FDIP_ENABLE && *repl_line_addr)
    evict_prefetched_cls(*repl_line_addr,
                         (mem_req_is_type(req, MRT_FDIPPRFON) || mem_req_is_type(req, MRT_FDIPPRFOFF)) ? TRUE : FALSE);
}

static inline void wp_process_icache_fill(Icache_Data* line, Mem_Req* req) {
  if (!WP_COLLECT_STATS)
    return;

  if ((req->type == MRT_WB) || (req->type == MRT_WB_NODIRTY) ||
      (req->type == MRT_IPRF)) /* for now we don't consider prefetches */
    return;

  if (req->off_path) {
    STAT_EVENT(ic->proc_id, ICACHE_FILL_OFFPATH);
  } else {
    STAT_EVENT(ic->proc_id, ICACHE_FILL_ONPATH);
    if (req->onpath_match_offpath)
      STAT_EVENT(ic->proc_id, DIST_ICACHE_FILL_ONPATH_PARTIAL);
    else
      STAT_EVENT(ic->proc_id, DIST_ICACHE_FILL_ONPATH);
  }
  STAT_EVENT(ic->proc_id, DIST_ICACHE_FILL);
}

static inline int32_t inst_lost_get_full_window_reason() {
  if (rob_stall_reason != ROB_STALL_NONE) {
    return rob_stall_reason;
  }

  if (rob_block_issue_reason != ROB_BLOCK_ISSUE_NONE) {
    return rob_block_issue_reason;
  }

  return 0;
}

static inline void log_stats_ic_miss() {
  STAT_EVENT(ic->proc_id, ICACHE_MISS);
  STAT_EVENT(ic->proc_id, POWER_ICACHE_MISS);
  STAT_EVENT(ic->proc_id, ICACHE_MISS_ONPATH + ic->off_path);
}

static inline void log_stats_ic_hit() {
  STAT_EVENT(ic->proc_id, ICACHE_HIT);
  STAT_EVENT(ic->proc_id, ICACHE_HIT_ONPATH + ic->off_path);
}

void log_stats_mshr_hit(Addr line_addr) {
  Flag demand_hit_prefetch = FALSE;
  Flag demand_hit_writeback = FALSE;
  Mem_Queue_Entry* queue_entry = NULL;
  Flag ramulator_match = FALSE;
  Mem_Req* req = mem_search_reqbuf_wrapper(
      ic->proc_id, line_addr, MRT_FDIPPRFON, ICACHE_LINE_SIZE, &demand_hit_prefetch, &demand_hit_writeback,
      QUEUE_MLC | QUEUE_L1 | QUEUE_BUS_OUT | QUEUE_MEM | QUEUE_L1FILL | QUEUE_MLC_FILL, &queue_entry, &ramulator_match);
  if (!req) {
    req = mem_search_reqbuf_wrapper(ic->proc_id, line_addr, MRT_FDIPPRFOFF, ICACHE_LINE_SIZE, &demand_hit_prefetch,
                                    &demand_hit_writeback,
                                    QUEUE_MLC | QUEUE_L1 | QUEUE_BUS_OUT | QUEUE_MEM | QUEUE_L1FILL | QUEUE_MLC_FILL,
                                    &queue_entry, &ramulator_match);
  }

  if (req && !req->cyc_hit_by_demand_load) {
    uns64 hashed_addr = FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(ic->line_addr, req->ghist) : ic->line_addr;
    if (!icache_off_path() && fdip_search_pref_candidate(ic->line_addr)) {
      inc_cnt_useful(ic->proc_id, hashed_addr, FALSE);
      inc_cnt_useful_signed(hashed_addr);
      inc_useful_lines_uc(hashed_addr);
      update_useful_lines_uc(hashed_addr);
      update_useful_lines_bloom_filter(hashed_addr);
      inc_utility_info(TRUE);
      inc_timeliness_info(TRUE);
    }
    if (mem_req_is_type(req, MRT_FDIPPRFON) || mem_req_is_type(req, MRT_FDIPPRFOFF)) {
      STAT_EVENT(ic->proc_id, ICACHE_MISS_MSHR_HIT_ONPATH_BY_FDIP + icache_off_path());
      if (req->fdip_pref_off_path)
        STAT_EVENT(ic->proc_id, ICACHE_MISS_MSHR_HIT_BY_FDIP_OFFPATH);
      else
        STAT_EVENT(ic->proc_id, ICACHE_MISS_MSHR_HIT_BY_FDIP_ONPATH);
    }
    if (!icache_off_path())
      req->cyc_hit_by_demand_load = cycle_count;
  }
  if (!icache_off_path())
    inc_icache_miss(ic->line_addr);
  Imiss_Reason imiss_reason = get_miss_reason(line_addr);
  DEBUG_FDIP(ic->proc_id, "miss reason: %d, req: %d\n", imiss_reason, req ? 1 : 0);
  if (!req) {
    if (!icache_off_path()) {
      uns64 hashed_addr =
          FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(ic->line_addr, g_bp_data->global_hist) : ic->line_addr;
      DEBUG_FDIP(ic->proc_id, "learn missed line %llx\n", ic->line_addr);
      inc_cnt_useful(ic->proc_id, hashed_addr, TRUE);
      inc_cnt_useful_signed(hashed_addr);
      inc_useful_lines_uc(hashed_addr);
      update_useful_lines_uc(hashed_addr);
      update_useful_lines_bloom_filter(hashed_addr);
      inc_utility_info(TRUE);
      ASSERT(ic->proc_id, imiss_reason <= IMISS_TOO_EARLY_EVICTED_BY_FDIP);
      STAT_EVENT(ic->proc_id, ICACHE_MISS_NOT_PREFETCHED + imiss_reason);
      if (imiss_reason == IMISS_NOT_PREFETCHED)
        assert_fdip_break_reason(hashed_addr);
    } else {
      inc_off_fetched_cls(ic->line_addr);
    }
    if (imiss_reason == IMISS_TOO_EARLY_EVICTED_BY_FDIP || imiss_reason == IMISS_TOO_EARLY_EVICTED_BY_IFETCH)
      STAT_EVENT(ic->proc_id, ICACHE_MISS_TOO_EARLY_ONPATH + icache_off_path());
    else
      STAT_EVENT(ic->proc_id, ICACHE_MISS_NOT_PREFETCHED_ONPATH + icache_off_path());
  } else {
    if (FDIP_ENABLE && !FDIP_UTILITY_HASH_ENABLE && !FDIP_BLOOM_FILTER && !FDIP_UC_SIZE && !EIP_ENABLE &&
        !FDIP_PERFECT_PREFETCH && (NUM_BPS == 1) &&
        (mem_req_is_type(req, MRT_FDIPPRFON) || mem_req_is_type(req, MRT_FDIPPRFOFF)))
      /* FDIP's per-line map is reset at a recovery, so an outstanding prefetch can
         still read as IMISS_NOT_PREFETCHED. Count it rather than assert. */
      if (!(imiss_reason == IMISS_MSHR_HIT_PREFETCHED_OFFPATH || imiss_reason == IMISS_MSHR_HIT_PREFETCHED_ONPATH))
        STAT_EVENT(ic->proc_id, ICACHE_MISS_MSHR_HIT_FDIP_STALE_MAP);
    if (imiss_reason == IMISS_MSHR_HIT_PREFETCHED_ONPATH)
      STAT_EVENT(ic->proc_id, ICACHE_MISS_MSHR_HIT_PREFETCHED_ONPATH);
    else if (imiss_reason == IMISS_MSHR_HIT_PREFETCHED_OFFPATH)
      STAT_EVENT(ic->proc_id, ICACHE_MISS_MSHR_HIT_PREFETCHED_OFFPATH);
    else
      STAT_EVENT(ic->proc_id, ICACHE_MISS_NOT_PREFETCHED);
  }
  DEBUG_FDIP(ic->proc_id, "set last miss reason %u for %llx\n", imiss_reason, ic->line_addr);
  set_last_miss_reason(imiss_reason);
}

// Wrapper callback for any instruction memreq.
// This must always return TRUE so that memreq is satisfied that
// done_func is finished and does not need to be retried.
Flag instr_fill_line(Mem_Req* req) {
  ASSERT(ic->proc_id, req->type == MRT_IPRF || req->type == MRT_FDIPPRFON || req->type == MRT_FDIPPRFOFF ||
                          req->type == MRT_UOCPRF || req->type == MRT_IFETCH);
  if (mem_req_is_type(req, MRT_IFETCH) || mem_req_is_type(req, MRT_IPRF) || mem_req_is_type(req, MRT_FDIPPRFON) ||
      mem_req_is_type(req, MRT_FDIPPRFOFF)) {
    icache_fill_line(req);
    DEBUG(ic->proc_id, "line 0x%llx is filled into icache\n", req->addr);
  }

  return TRUE;
}

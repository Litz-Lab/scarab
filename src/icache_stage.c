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

#include <math.h>
#include "debug/debug_macros.h"
#include "debug/debug_print.h"
#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "bp/bp.h"
#include "icache_stage.h"
#include "map.h"
#include "op_pool.h"
#include "thread.h"
#include "sim.h"

#include "bp/bp.param.h"
#include "cmp_model.h"
#include "core.param.h"
#include "debug/debug.param.h"
#include "frontend/frontend.h"
#include "frontend/pin_trace_fe.h"
#include "memory/memory.h"
#include "memory/memory.param.h"
#include "prefetcher/l2l1pref.h"
#include "prefetcher/stream_pref.h"
#include "statistics.h"
#include "libs/list_lib.h"

/*#include "prefetcher/fdip.h"*/
#include "prefetcher/fdip_new.h"
#include "prefetcher/eip.h"
#include "prefetcher/D_JOLT.h"
#include "prefetcher/FNL+MMA.h"
#include "prefetcher/pref.param.h"
#include "uop_queue_stage.h"
#include "decode_stage.h"
/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_ICACHE_STAGE, ##args)
#define DEBUG_FDIP(proc_id, args...) _DEBUG(proc_id, DEBUG_FDIP, ##args)

/**************************************************************************************/
/* Global Variables */

/**************************************************************************************/

Icache_Stage* ic = NULL;

extern Cmp_Model              cmp_model;
extern Memory*                mem;
extern Rob_Stall_Reason       rob_stall_reason;
extern Rob_Block_Issue_Reason rob_block_issue_reason;

/**************************************************************************************/
/* Local prototypes */

static inline void         icache_process_ops(Stage_Data* cur_data);
static inline Inst_Info**  lookup_cache(void);
static inline void         prefetcher_update_on_icache_access(Flag icache_hit);
static inline void         icache_hit_events(Flag uop_cache_hit);
static inline void         icache_miss_events(Flag uop_cache_hit);
static inline Flag         mem_req_on_icache_miss(void);
static inline void         uop_cache_to_icache_switch_stats(void);
static Inst_Info**         ic_pref_cache_access(void);
int32_t                    inst_lost_get_full_window_reason(void);
static inline void         log_stats_ic_miss(void);
static inline void         log_stats_ic_hit(void);
static inline void         log_stats_mshr_hit(Addr line_addr);
static inline void         update_stats_bf_retired(void);

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
  ic->sd.name = (char*)strdup(name);

  /* initialize the ops array */
  ASSERT(proc_id, IC_ISSUE_WIDTH);
  ic->sd.max_op_count = IC_ISSUE_WIDTH;
  ic->sd.ops          = (Op**)malloc(sizeof(Op*) * IC_ISSUE_WIDTH);

  if (UOP_CACHE_ENABLE) {
    char uopc_sd_name[32];
    snprintf(uopc_sd_name, sizeof(uopc_sd_name), "%s_%s", name, "UOPC");
    ic->uopc_sd.name = (char*)strdup(uopc_sd_name);
    ic->uopc_sd.max_op_count = UOPC_ISSUE_WIDTH;
    ic->uopc_sd.op_count     = 0;
    ic->uopc_sd.ops          = (Op**)calloc(UOPC_ISSUE_WIDTH, sizeof(Op*));
  }

  /* initialize the cache structure */
  init_cache(&ic->icache, "ICACHE", ICACHE_SIZE, ICACHE_ASSOC, ICACHE_LINE_SIZE,
             0, REPL_TRUE_LRU);

  /* init icache_line_info struct - this struct keeps data about corresponding
   * icache lines */
  if(WP_COLLECT_STATS) {
    // init_cache(&ic->icache_line_info, "IC LI", ICACHE_SIZE, ICACHE_ASSOC,
    // ICACHE_LINE_SIZE,
    //           sizeof(Icache_Data), REPL_TRUE_LRU);
    /*init_cache(&ic->icache_line_info, "IC LI", ICACHE_SIZE, ICACHE_ASSOC,*/
               /*ICACHE_LINE_SIZE, sizeof(Icache_Data), ICACHE_REPL);*/
    init_cache(&ic->icache_line_info, "IC LI", ICACHE_SIZE, ICACHE_ASSOC,
               ICACHE_LINE_SIZE, sizeof(Icache_Data), REPL_TRUE_LRU);
  }

  // moved the init code from here to reset
  reset_icache_stage();

  if(IC_PREF_CACHE_ENABLE)
    init_cache(&ic->pref_icache, "IC_PREF_CACHE", IC_PREF_CACHE_SIZE,
               IC_PREF_CACHE_ASSOC, ICACHE_LINE_SIZE, 0, REPL_TRUE_LRU);

  memset(ic->rand_wb_state, 0, NUM_ELEMENTS(ic->rand_wb_state));
}

/**************************************************************************************/
/* get_current_stage_data: */

Stage_Data* get_current_stage_data() {
  if (!UOP_CACHE_ENABLE) {
    return &ic->sd;
  } else {
    // two stage data cannot be used at once
    ASSERT(ic->proc_id, ic->sd.op_count == 0 || ic->uopc_sd.op_count == 0);
    return ic->uopc_sd.op_count? &ic->uopc_sd : &ic->sd;
  }
}

/**************************************************************************************/
/* reset_icache_stage: */

void reset_icache_stage() {
  Stage_Data* cur_data = get_current_stage_data();
  uns ii;
  for(ii = 0; ii < cur_data->max_op_count; ii++)
    cur_data->ops[ii] = NULL;
  cur_data->op_count = 0;

  ic->off_path                       = FALSE;
  ic->back_on_path                   = FALSE;
  op_count[ic->proc_id]              = 1;
  unique_count_per_core[ic->proc_id] = 1;
}

/**************************************************************************************/
/* reset_all_ops_icache_stage: */
// CMP used for bogus run: may be combined with reset_icache_stage
void reset_all_ops_icache_stage() {
  Stage_Data* cur_data = get_current_stage_data();
  uns ii;
  for(ii = 0; ii < cur_data->max_op_count; ii++)
    cur_data->ops[ii] = NULL;
  cur_data->op_count = 0;

  ic->off_path     = FALSE;
  ic->back_on_path = FALSE;
}

/**************************************************************************************/
/* recover_icache_stage: */

void recover_icache_stage() {
  Stage_Data* cur_data = get_current_stage_data();
  uns         ii;

  ASSERT(ic->proc_id, ic->proc_id == bp_recovery_info->proc_id);
  DEBUG(ic->proc_id,
        "Icache stage recovery signaled.  recovery_fetch_addr: 0x%s\n",
        hexstr64s(bp_recovery_info->recovery_fetch_addr));
  for(ii = 0; ii < cur_data->max_op_count; ii++) {
    if(cur_data->ops[ii]) {
      ASSERT(ic->proc_id, FLUSH_OP(cur_data->ops[ii]));
      ASSERT(ic->proc_id, cur_data->ops[ii]->off_path);
      free_op(cur_data->ops[ii]);
      cur_data->ops[ii] = NULL;
    }
  }
  cur_data->op_count = 0;

  ic->back_on_path = !bp_recovery_info->recovery_force_offpath;

  Op* op = bp_recovery_info->recovery_op;
  if(bp_recovery_info->late_bp_recovery && op->oracle_info.btb_miss &&
     !op->oracle_info.btb_miss_resolved) {
    // Late branch predictor recovered before btb miss is resolved (i.e., icache
    // stage should still wait for redirect)
  } else {
    if(ic->next_state != WAIT_FOR_MISS) {
      ic->next_state = SERVING_INIT;
    } else {
      ic->after_waiting_state = SERVING_INIT;
    }
    if(SWITCH_IC_FETCH_ON_RECOVERY && model->id == CMP_MODEL) {
      ic->next_state = SERVING_INIT;
    }
  }
  op_count[ic->proc_id] = bp_recovery_info->recovery_op_num + 1;

  uop_cache_clear_lookup_buffer();
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
  DPRINTF(
    "fetch_addr:0x%s  path:%s  state:%s  next_state:%s\n",
    hexstr64s(ic->fetch_addr),
    ic->off_path ? "OFF_PATH" : "ON_PATH ", icache_state_names[ic->state],
    icache_state_names[ic->next_state]);

  // print icache stage
  DPRINTF("# %-10s  op_count:%d\n", "ICache", cur_data->op_count);
  print_op_array(GLOBAL_DEBUG_STREAM, cur_data->ops, cur_data->max_op_count,
                 cur_data->op_count);
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
/* lookup_cache: returns instr if found in icache or other structures when enabled
 */
Inst_Info** lookup_cache() {
  STAT_EVENT(ic->proc_id, POWER_ICACHE_ACCESS);
  STAT_EVENT(ic->proc_id, POWER_ITLB_ACCESS);

  Inst_Info** line = NULL;
  line = (Inst_Info**)cache_access(&ic->icache, ic->fetch_addr,
                                             &ic->line_addr, TRUE);
  if(PERFECT_ICACHE && !line)
    line = (Inst_Info**)INIT_CACHE_DATA_VALUE;

  // ideal L2 Icache prefetcher
  if(IDEAL_L2_ICACHE_PREFETCHER && !line) {
    Addr     dummy_line_addr;
    L1_Data* data;
    Cache*   l1_cache = model->mem == MODEL_MEM ?
                        &mem->uncores[ic->proc_id].l1->cache :
                        NULL;
    data = l1_cache ? (L1_Data*)cache_access(l1_cache, ic->fetch_addr,
                                              &dummy_line_addr, TRUE) :
                      NULL;
    if(data) {  // second level cache hit
      STAT_EVENT(ic->proc_id, L2_IDEAL_FILL_ICACHE);
      // actually bring it into the L1 icache
      Addr dummy_repl_line_addr;
      line = (Inst_Info**)cache_insert(&ic->icache, ic->proc_id,
                                        ic->fetch_addr, &dummy_line_addr,
                                        &dummy_repl_line_addr);
    } else
      STAT_EVENT(ic->proc_id, L2_IDEAL_MISS_ICACHE);
  }

  if (IC_PREF_CACHE_ENABLE && (!ic->line)) {
    line = ic_pref_cache_access();
  }

  return line;
}

void prefetcher_update_on_icache_access(Flag icache_hit) {
  if(EIP_ENABLE)
    eip_prefetch(ic->proc_id, ic->fetch_addr, icache_hit, 0, ic->off_path);
  if(DJOLT_ENABLE)
    djolt_prefetch(ic->proc_id, ic->fetch_addr, icache_hit, 0);
  if(FNLMMA_ENABLE)
    fnlmma_prefetch(ic->proc_id, ic->fetch_addr, icache_hit, 0);
}

void icache_hit_events(Flag uop_cache_hit) {
  DEBUG(ic->proc_id, "Cache hit on op_num:%s @ 0x%s line_addr 0x%s\n",
        unsstr64(op_count[ic->proc_id]), hexstr64s(ic->fetch_addr), hexstr64s(ic->fetch_addr & ~0x3F));

  if (ALWAYS_LOOKUP_ICACHE) {
    if (!ic->off_path) {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_UOP_CACHE_MISS_ON_PATH + uop_cache_hit);
    } else {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_UOP_CACHE_MISS_OFF_PATH + uop_cache_hit);
    }
  }

  prefetcher_update_on_icache_access(/*icache_hit*/ TRUE);
  log_stats_ic_hit();
}

void icache_miss_events(Flag uop_cache_hit) {
    DEBUG(ic->proc_id, "Cache miss on op_num:%s @ 0x%s\n",
          unsstr64(op_count[ic->proc_id]), hexstr64s(ic->fetch_addr));

    if (ALWAYS_LOOKUP_ICACHE) {
      if (!ic->off_path) {
        STAT_EVENT(ic->proc_id, ICACHE_MISS_UOP_CACHE_MISS_ON_PATH + uop_cache_hit);
      } else {
        STAT_EVENT(ic->proc_id, ICACHE_MISS_UOP_CACHE_MISS_OFF_PATH + uop_cache_hit);
      }
    }

    prefetcher_update_on_icache_access(/*icache_hit*/ FALSE);
    log_stats_ic_miss();
    log_stats_mshr_hit(ic->line_addr);
}

Flag mem_req_on_icache_miss() {
  /* if the icache is available, wait for a miss */
  /* otherwise, refetch next cycle */
  if(model->mem == MODEL_MEM) {
    if(ic->proc_id)
      ASSERTM(ic->proc_id, ic->line_addr, "ic fetch addr: %llu\n",
              ic->fetch_addr);
    ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->line_addr)
    Flag success = FALSE;
    success = new_mem_req(MRT_IFETCH, ic->proc_id, ic->line_addr,
                    ICACHE_LINE_SIZE, 0, NULL, instr_fill_line,
                    unique_count,
                    0);
    if (success) {  // CMP maybe unique_count_per_core[proc_id]?
      DEBUG(ic->proc_id, "from IC_STAGE for cl 0x%llx at cycle %llu\n", ic->line_addr, cycle_count);
      STAT_EVENT(ic->proc_id, ICACHE_STAGE_MISS);

      if(ONE_MORE_CACHE_LINE_ENABLE) {
        Addr         one_more_addr;
        Addr         extra_line_addr;
        Icache_Data* extra_line;

        one_more_addr =
          ((ic->line_addr >> LOG2(ICACHE_LINE_SIZE)) & 1) ?
            ((ic->line_addr >> LOG2(ICACHE_LINE_SIZE)) - 1)
              << LOG2(ICACHE_LINE_SIZE) :
            ((ic->line_addr >> LOG2(ICACHE_LINE_SIZE)) + 1)
              << LOG2(ICACHE_LINE_SIZE);

        extra_line = (Icache_Data*)cache_access(
          &ic->icache, one_more_addr, &extra_line_addr, FALSE);
        ASSERT(ic->proc_id, one_more_addr == extra_line_addr);
        if(!extra_line) {
          if(new_mem_req(MRT_IFETCH, ic->proc_id, extra_line_addr,
                          ICACHE_LINE_SIZE, 0, NULL, NULL, unique_count,
                          0))
            STAT_EVENT_ALL(ONE_MORE_SUCESS);
          else
            STAT_EVENT_ALL(ONE_MORE_DISCARDED_MEM_REQ_FULL);
        } else
          STAT_EVENT_ALL(ONE_MORE_DISCARDED_L0CACHE);
      }
    }
    return success;
  }
  ASSERT(ic->proc_id, 0);
  return TRUE;
}

void uop_cache_to_icache_switch_stats() {
  int uop_queue_length = get_uop_queue_stage_length();
  int decode_stages_filled = get_decode_stages_filled();
  // TODO(peterbraun): Only measure these stats for ON-PATH. Same thing with resteer stats
  const int switch_stat_max_len = 20;  // Stat supports up to 20.
  STAT_EVENT(ic->proc_id, UOP_CACHE_ICACHE_SWITCH_UOP_QUEUE_LENGTH_0 + uop_queue_length + ic->off_path*(switch_stat_max_len+1));
  STAT_EVENT(ic->proc_id, UOP_CACHE_ICACHE_SWITCH_UOP_QUEUE_PLUS_DECODE_LENGTH_0 + uop_queue_length + decode_stages_filled + ic->off_path*(switch_stat_max_len+1));
  ASSERT(ic->proc_id, uop_queue_length + decode_stages_filled <= switch_stat_max_len);
  _DEBUG(ic->proc_id, DEBUG_UOP_CACHE, "uoc->ic switch, uop_queue=%u\n", uop_queue_length);
}
/**************************************************************************************/
/* icache_cycle: */

void update_icache_stage() {
  Icache_Data* line_info = NULL;
  Addr         dummy_addr;

  STAT_EVENT(ic->proc_id, ICACHE_CYCLE);
  STAT_EVENT(ic->proc_id, ICACHE_CYCLE_ONPATH + ic->off_path);

  if (ic->off_path)
    STAT_EVENT(exec->proc_id, ICACHE_STAGE_OFF_PATH);

  if(ic->sd.op_count || (UOP_CACHE_ENABLE && ic->uopc_sd.op_count)) {
    // if uop cache is enabled, use UOPC_ISSUE_WIDTH as the optimal width
    INC_STAT_EVENT(ic->proc_id, INST_LOST_TOTAL, UOP_CACHE_ENABLE ? UOPC_ISSUE_WIDTH : IC_ISSUE_WIDTH);
    STAT_EVENT(ic->proc_id, FETCH_0_OPS);
    INC_STAT_EVENT(ic->proc_id,
                   INST_LOST_FULL_WINDOW + inst_lost_get_full_window_reason(),
                   UOP_CACHE_ENABLE ? UOPC_ISSUE_WIDTH : IC_ISSUE_WIDTH);
    DEBUG(ic->proc_id, "Icache stalled\n");
    if(!ic->off_path) {
      STAT_EVENT(map->proc_id, ICACHE_STAGE_STALLED);
    }
    return;
  }
  else if(!ic->off_path) {
    STAT_EVENT(map->proc_id, ICACHE_STAGE_NOT_STALLED);
  }

  DEBUG(ic->proc_id, "Icache state: %i\n", ic->state);

  Break_Reason break_fetch = BREAK_DONT;

  ic->off_path &= !ic->back_on_path;
  ic->back_on_path = FALSE;

  STAT_EVENT(ic->proc_id, FETCH_ON_PATH + ic->off_path);

  ASSERT(ic->proc_id, ic->next_state != ICACHE_FINISHED_FT_EXPECTING_NEXT);
  ASSERT(ic->proc_id, ic->next_state != ICACHE_LOOKUP_SERVING);
  while(!break_fetch) {
    ic->state = ic->next_state;

    if (!UOP_CACHE_ENABLE) {
      ASSERT(ic->proc_id, ic->state != UOP_CACHE_FINISHED_FT && ic->state != UOP_CACHE_SERVING);
    }
    //TODO: renaming should stall icache via back-pressure,
    // but it seems that icache stage is doing some map stage work.
    // consider moving them to the map stage?
    if (REG_RENAMING_TABLE_ENABLE
        && !rename_table_available()
        && ic->state != WAIT_FOR_MISS
        && ic->state != WAIT_FOR_EMPTY_ROB
        && ic->state != WAIT_FOR_RENAME
        && ic->state != ICACHE_RETRY_MEM_REQ) {
      DEBUG(ic->proc_id, "Renaming Stall\n");
      break_fetch = BREAK_RENAME;
      ic->next_state = WAIT_FOR_RENAME;
      if (ic->state == ICACHE_FINISHED_FT_EXPECTING_NEXT) {
        ic->after_waiting_state = ICACHE_FINISHED_FT;
      } else {
        ic->after_waiting_state = ic->state;
      }
    } else if (ic->state == SERVING_INIT
              || ic->state == ICACHE_FINISHED_FT
              || ic->state == ICACHE_FINISHED_FT_EXPECTING_NEXT
              || ic->state == UOP_CACHE_FINISHED_FT) {
      if (ic->state == ICACHE_FINISHED_FT_EXPECTING_NEXT) {
        ASSERT(ic->proc_id, ic->sd.op_count && !ic->uopc_sd.op_count);
      } else {
        ASSERT(ic->proc_id, !ic->sd.op_count && !ic->uopc_sd.op_count);
      }
      ASSERT(ic->proc_id, !decoupled_fe_current_ft_can_fetch_op(ic->proc_id));
      if (!decoupled_fe_can_fetch_ft(ic->proc_id)) {
        // if this happened, the app exit should have been seen
        ASSERT(ic->proc_id, get_stat(ic->proc_id, "ST_BREAK_APP_EXIT"));
        break_fetch = BREAK_FT_UNAVAILABLE;
        ic->next_state = SERVING_INIT;
      } else {
        ic->current_ft_info = decoupled_fe_fetch_ft(ic->proc_id);

        // set the current fetch address
        ic->fetch_addr = ic->current_ft_info.static_info.start;
        ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->fetch_addr);

        // look up uop cache
        Flag ft_in_uop_cache = uop_cache_lookup_ft_and_fill_lookup_buffer(ic->current_ft_info, ic->off_path);

        // look up icache if uop miss (inlcuding when uop cache disabled) or if requested
        if (!ft_in_uop_cache || ALWAYS_LOOKUP_ICACHE) {
          ic->line = lookup_cache();

          //TODO: power stats are poorly mantained. move this to the decoupled fe?
          // STAT_EVENT(ic->proc_id, POWER_BTB_READ);

          if(WP_COLLECT_STATS) { // CMP remove?
            line_info = (Icache_Data*)cache_access(&ic->icache_line_info, ic->fetch_addr, &dummy_addr, TRUE);
            if (ic->line) {
              ASSERT(ic->proc_id, line_info);
              wp_process_icache_hit(line_info, ic->fetch_addr);
            }
          }
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
              icache_hit_events(/*uop_cache_hit*/ TRUE);
            } else {
              icache_miss_events(/*uop_cache_hit*/ TRUE);
              if (IPRF_ON_UOP_CACHE_HIT) {
                // start a memreq to fill icache, but do not cause any stalls.
                // Use for more inclusivity between IC and UC
                new_mem_req(MRT_IPRF, ic->proc_id, ic->line_addr,
                            ICACHE_LINE_SIZE, 0, NULL, instr_fill_line,
                            unique_count,
                            0);
              }
            }
          }

          if (ic->state == ICACHE_FINISHED_FT_EXPECTING_NEXT) {
            // in one cycle there can be only one source serving
            break_fetch = BREAK_ICACHE_TO_UOP_CACHE_SWITCH;
          }
          ic->next_state = UOP_CACHE_SERVING;
        } else if (ic->line) {
          // uop cache miss and icache hit
          if (!ic->off_path) {
            STAT_EVENT(ic->proc_id, FT_UOP_CACHE_MISS_ICACHE_HIT_ON_PATH);
          } else {
            STAT_EVENT(ic->proc_id, FT_UOP_CACHE_MISS_ICACHE_HIT_OFF_PATH);
          }
          icache_hit_events(/*uop_cache_hit*/ FALSE);

          ic->next_state = ICACHE_LOOKUP_SERVING;
          if (ic->state == UOP_CACHE_FINISHED_FT) {
            uop_cache_to_icache_switch_stats();
          }
        } else {
          // uop cache miss and icache miss
          if (!ic->off_path) {
            STAT_EVENT(ic->proc_id, FT_UOP_CACHE_MISS_ICACHE_MISS_ON_PATH);
          } else {
            STAT_EVENT(ic->proc_id, FT_UOP_CACHE_MISS_ICACHE_MISS_OFF_PATH);
          }
          icache_miss_events(/*uop_cache_hit*/ FALSE);

          STAT_EVENT(ic->proc_id, FETCH_0_OPS);

          Flag success = mem_req_on_icache_miss();
          if (success) {
            break_fetch = BREAK_ICACHE_MISS_REQ_SUCCESS;
            ic->next_state = WAIT_FOR_MISS;
            ic->after_waiting_state = ICACHE_NO_LOOKUP_SERVING;
          } else {
            break_fetch = BREAK_ICACHE_MISS_REQ_FAILURE;
            ic->next_state = ICACHE_RETRY_MEM_REQ;
          }

          if (ic->state == UOP_CACHE_FINISHED_FT) {
            uop_cache_to_icache_switch_stats();
          }
        }
      }
    } else if (ic->state == ICACHE_RETRY_MEM_REQ) {
      DEBUG(ic->proc_id, "Cache retry mem req on op_num:%s @ 0x%s\n",
            unsstr64(op_count[ic->proc_id]), hexstr64s(ic->fetch_addr));

      STAT_EVENT(ic->proc_id, FETCH_0_OPS);

      Flag success = mem_req_on_icache_miss();
      if (success) {
        break_fetch = BREAK_ICACHE_MISS_REQ_SUCCESS;
        ic->next_state = WAIT_FOR_MISS;
        ic->after_waiting_state = ICACHE_NO_LOOKUP_SERVING;
      } else {
        break_fetch = BREAK_ICACHE_MISS_REQ_FAILURE;
        ic->next_state = ICACHE_RETRY_MEM_REQ;
      }
    } else if (ic->state == UOP_CACHE_SERVING) {
      Uop_Cache_Data* uop_cache_line = uop_cache_get_line_from_lookup_buffer();
      // the line must be valid
      ASSERT(ic->proc_id, uop_cache_line->n_uops);

      ASSERT(ic->proc_id, !ic->uopc_sd.op_count);
      ASSERT(ic->proc_id, uop_cache_line->n_uops <= ic->uopc_sd.max_op_count);
      Flag ft_has_ended = decoupled_fe_fill_icache_stage_data(ic->proc_id, uop_cache_line->n_uops, &ic->uopc_sd);
      ASSERT(ic->proc_id, ic->uopc_sd.op_count && ic->uopc_sd.op_count == uop_cache_line->n_uops);

      if (ft_has_ended) {
        ASSERT(ic->proc_id, !decoupled_fe_current_ft_can_fetch_op(ic->proc_id));
        // sanity check that the uop cache is in sync
        ASSERT(ic->proc_id, uop_cache_line->end_of_ft);
        ic->next_state = UOP_CACHE_FINISHED_FT;
        switch(ic->current_ft_info.dynamic_info.ended_by) {
          case FT_ICACHE_LINE_BOUNDARY:
          case FT_TAKEN_BRANCH:
            if (ic->uopc_sd.op_count < ic->uopc_sd.max_op_count) {
              break_fetch = BREAK_UOP_CACHE_READ_LIMIT;
            } else {
              break_fetch = BREAK_UOP_CACHE_READ_LIMIT_AND_ISSUE_WIDTH;
            }
            break;
          case FT_BAR_FETCH:
            break_fetch = BREAK_BARRIER;
            // overwrite the next serving state.
            ic->next_state = WAIT_FOR_EMPTY_ROB;
            ic->after_waiting_state = UOP_CACHE_FINISHED_FT;
            break;
          case FT_APP_EXIT:
            break_fetch = BREAK_APP_EXIT;
            break;
          default:
            ASSERT(ic->proc_id, 0);
        }
        uop_cache_clear_lookup_buffer();
      } else {
        ASSERT(ic->proc_id, decoupled_fe_current_ft_can_fetch_op(ic->proc_id));
        ASSERT(ic->proc_id, !uop_cache_line->end_of_ft);
        // the current uop cache assumes that uop cache line op num equals ic->uopc_sd.max_op_count (UOPC_ISSUE_WIDTH)
        ASSERT(ic->proc_id, ic->uopc_sd.op_count == ic->uopc_sd.max_op_count);
        // next_fetch_addr is usually updated when an FT is fetched.
        // but for uop cache, one FT can span several lines,
        // and uop cache needs to calculate the next line address
        ic->fetch_addr += uop_cache_line->offset;
        ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->fetch_addr);
        break_fetch = BREAK_ISSUE_WIDTH;
        ic->next_state = UOP_CACHE_SERVING;
      }

      // mark all ops as fetched from the uop cache
      for(int ii = 0; ii < ic->uopc_sd.op_count; ii++) {
        ic->uopc_sd.ops[ii]->fetched_from_uop_cache = TRUE;
      }
    } else if (ic->state == ICACHE_LOOKUP_SERVING
            || ic->state == ICACHE_NO_LOOKUP_SERVING) {
      // ic->line should have already been set correctly
      ASSERT(ic->proc_id, ic->line);
      ASSERT(ic->proc_id, ic->line_addr);
      // sanity checks
      Inst_Info** dummy_inst_info;
      Addr dummy_line_addr;
      dummy_inst_info = (Inst_Info**)cache_access(&ic->icache, ic->fetch_addr, &dummy_line_addr, FALSE);
      ASSERT(ic->proc_id, ic->line == dummy_inst_info);
      ASSERT(ic->proc_id, ic->line_addr == dummy_line_addr);

      int requested = ic->sd.max_op_count - ic->sd.op_count;
      Flag ft_has_ended = decoupled_fe_fill_icache_stage_data(ic->proc_id, requested, &ic->sd);
      ASSERT(ic->proc_id, ic->sd.op_count == ic->sd.max_op_count || ft_has_ended);

      if (ft_has_ended) {
        ASSERT(ic->proc_id, !decoupled_fe_current_ft_can_fetch_op(ic->proc_id));
        ic->next_state = ICACHE_FINISHED_FT;
        switch(ic->current_ft_info.dynamic_info.ended_by) {
          case FT_ICACHE_LINE_BOUNDARY:
          case FT_TAKEN_BRANCH:
            // if there is more op slots
            if (ic->sd.op_count < ic->sd.max_op_count) {
              if (ic->state == ICACHE_NO_LOOKUP_SERVING && FETCH_ACROSS_FETCH_TARGET) {
                // haven't access icache this cycle,
                // so the next FT can be processed if possible.
                // overwrite the next serving state.
                ic->next_state = ICACHE_FINISHED_FT_EXPECTING_NEXT;
              } else {
                break_fetch = BREAK_ICACHE_READ_LIMIT;
              }
            } else {
              if (ic->state == ICACHE_LOOKUP_SERVING) {
                break_fetch = BREAK_ICACHE_READ_LIMIT_AND_ISSUE_WIDTH;
              } else {
                break_fetch = BREAK_ISSUE_WIDTH;
              }
            }
            break;
          case FT_BAR_FETCH:
            break_fetch = BREAK_BARRIER;
            // overwrite the next serving state.
            ic->next_state = WAIT_FOR_EMPTY_ROB;
            ic->after_waiting_state = ICACHE_FINISHED_FT;
            break;
          case FT_APP_EXIT:
            break_fetch = BREAK_APP_EXIT;
            break;
          default:
            ASSERT(ic->proc_id, 0);
        }
      } else {
        ASSERT(ic->proc_id, decoupled_fe_current_ft_can_fetch_op(ic->proc_id));
        ASSERT(ic->proc_id, ic->sd.op_count == ic->sd.max_op_count);
        break_fetch = BREAK_ISSUE_WIDTH;
        ic->next_state = ICACHE_NO_LOOKUP_SERVING;
      }
    } else if (ic->state == WAIT_FOR_MISS) {
      ASSERT(ic->proc_id, decoupled_fe_current_ft_can_fetch_op(ic->proc_id));
      DEBUG(ic->proc_id, "Ifetch barrier: Waiting for miss \n");
      STAT_EVENT(ic->proc_id, FETCH_0_OPS);
      if(!ic->off_path) {
        STAT_EVENT(map->proc_id, ICACHE_STAGE_STARVED);
        INC_STAT_EVENT(ic->proc_id, INST_LOST_WAIT_FOR_ICACHE_MISS_NOT_PREFETCHED + get_last_miss_reason(ic->proc_id), ic->sd.max_op_count);
      }
      break_fetch = BREAK_WAIT_FOR_MISS;
      ic->next_state = ic->state;
      // the next state can be overwritten to ic->after_waiting_state by icache_fill_line
    } else if (ic->state == WAIT_FOR_EMPTY_ROB) {
      DEBUG(ic->proc_id, "Ifetch barrier: Waiting for ROB to become empty \n");
      STAT_EVENT(ic->proc_id, FETCH_0_OPS);
      break_fetch = BREAK_WAIT_FOR_EMPTY_ROB;
      if(td->seq_op_list.count == 0) {
        update_stats_bf_retired();
        ic->next_state = ic->after_waiting_state;
      } else {
        ic->next_state = ic->state;
      }
    } else if (ic->state == WAIT_FOR_RENAME) {
      DEBUG(ic->proc_id, "Ifetch barrier: Waiting for renaming register free \n");
      STAT_EVENT(ic->proc_id, FETCH_0_OPS);
      break_fetch = BREAK_RENAME;
      if (rename_table_available()) {
        DEBUG(ic->proc_id, "Renaming Stall Recover\n");
        ic->next_state = ic->after_waiting_state;
      } else {
        ic->next_state = ic->state;
      }
    } else {
      ASSERT(ic->proc_id, 0);
    }
  }

  // fetches of this cycle has all been completed.
  // start processing the ops.
  Stage_Data* cur_data = get_current_stage_data();
  icache_process_ops(cur_data);

  INC_STAT_EVENT(ic->proc_id, INST_LOST_TOTAL, cur_data->max_op_count);
  INC_STAT_EVENT(ic->proc_id, INST_LOST_BREAK_DONT + break_fetch,
                              cur_data->max_op_count > cur_data->op_count ?
                              cur_data->max_op_count - cur_data->op_count : 0);
  STAT_EVENT(ic->proc_id, FETCH_0_OPS + cur_data->op_count);
  STAT_EVENT(ic->proc_id, ST_BREAK_DONT + break_fetch);
  if(!ic->off_path) {
    if (!cur_data->op_count)
      STAT_EVENT(map->proc_id, ICACHE_STAGE_STARVED);
    else
      STAT_EVENT(map->proc_id, ICACHE_STAGE_NOT_STARVED);
  }
}


/**************************************************************************************/
/* update_bf_uoc_stats: */

void update_stats_bf_retired(void) {
  ASSERT(ic->proc_id, !decoupled_fe_current_ft_can_fetch_op(ic->proc_id));
  Flag next_ft_in_uop_cache = FALSE;
  if (decoupled_fe_can_fetch_ft(ic->proc_id)) {
    FT_Info next_ft_info = decoupled_fe_peek_ft(ic->proc_id);
    next_ft_in_uop_cache =
      uop_cache_lookup_line(next_ft_info.static_info.start, next_ft_info, FALSE) != NULL;
  }
  if (!next_ft_in_uop_cache) {
    // The micro-op cache can reduce the time to refill the pipeline after a fetch barrier.
    int uop_queue_length = get_uop_queue_stage_length();
    int decode_stages_filled = get_decode_stages_filled();
    ASSERT(ic->proc_id, uop_queue_length + decode_stages_filled <= 20);  // Stat supports up to 20.
    STAT_EVENT(ic->proc_id, BF_UOP_CACHE_MISS_UOP_QUEUE_LENGTH_0 + uop_queue_length);
    STAT_EVENT(ic->proc_id, BF_UOP_CACHE_MISS_UOP_QUEUE_PLUS_DECODE_LENGTH_0 + uop_queue_length + decode_stages_filled);
  }
}

/**************************************************************************************/
/* icache_process_ops: process all ops fetched in a cycle.*/

static inline void icache_process_ops(Stage_Data* cur_data) {
  static uns last_icache_issue_time = 0; /* for computing fetch break latency */
  uns            fetch_lag;

  ASSERT(ic->proc_id, ic->proc_id == td->proc_id);

  fetch_lag              = cycle_count - last_icache_issue_time;
  last_icache_issue_time = cycle_count;

  for (uns ii = 0; ii < cur_data->op_count; ii++) {
    Op* op = cur_data->ops[ii];

    ASSERTM(ic->proc_id, ic->off_path == op->off_path,
            "Inconsistent off-path op PC: %llx ic:%i op:%i\n", op->inst_info->addr, ic->off_path, op->off_path);

    if (!op->off_path) {
      STAT_EVENT(ic->proc_id, UOPS_SERVED_BY_ICACHE_ON_PATH + op->fetched_from_uop_cache);
    } else {
      STAT_EVENT(ic->proc_id, UOPS_SERVED_BY_ICACHE_OFF_PATH + op->fetched_from_uop_cache);
    }

    if(!op->off_path &&
       (op->table_info->mem_type == MEM_LD ||
        op->table_info->mem_type == MEM_ST) &&
       op->oracle_info.va == 0) {
      // don't care if the va is 0x0 if mem_type is MEM_PF(SW prefetch),
      // MEM_WH(write hint), or MEM_EVICT(cache block eviction hint)
      print_func_op(op);
      FATAL_ERROR(ic->proc_id, "Access to 0x0\n");
    }

    if(DUMP_TRACE && DEBUG_RANGE_COND(ic->proc_id))
      print_func_op(op);

    if(DIE_ON_CALLSYS && !op->off_path) {
      ASSERT(ic->proc_id, op->table_info->cf_type != CF_SYS);
    }

    /* add to sequential op list */
    add_to_seq_op_list(td, op);

    ASSERT(ic->proc_id, td->seq_op_list.count <= op_pool_active_ops);

    /* map the op based on true dependencies & set information in
     * op->oracle_info */
    /* num cycles since last group issued */
    op->fetch_lag = fetch_lag;

    thread_map_op(op);

    STAT_EVENT(op->proc_id, FETCH_ALL_INST);
    STAT_EVENT(op->proc_id, ORACLE_ON_PATH_INST + op->off_path);
    STAT_EVENT(op->proc_id, ORACLE_ON_PATH_INST_MEM +
               (op->table_info->mem_type == NOT_MEM) +
               2 * op->off_path);

    thread_map_mem_dep(op);
    op->fetch_cycle = cycle_count;

    op_count[ic->proc_id]++;          /* increment instruction counters */
    unique_count_per_core[ic->proc_id]++;
    unique_count++;
    /* check trigger */
    if(op->inst_info->trigger_op_fetched_hook)
      model->op_fetched_hook(op);

    INC_STAT_EVENT(ic->proc_id, INST_LOST_FETCH + ic->off_path, 1);

    DEBUG(ic->proc_id,
          "Fetching op from Icache addr: %s off: %d inst_info: %p ii_addr: %s "
          "dis: %s opnum: (%s:%s)\n",
          hexstr64s(op->inst_info->addr), op->off_path, op->inst_info,
          hexstr64s(op->inst_info->addr), disasm_op(op, TRUE),
          unsstr64(op->op_num), unsstr64(op->unique_num));

    if(op->table_info->cf_type) {
      //TODO: can we move this prefetch update to decoupled front-end or need it be here?
      if(DJOLT_ENABLE)
        update_djolt(ic->proc_id, op->inst_info->addr, op->table_info->cf_type, op->oracle_info.pred_npc);

      ASSERT(ic->proc_id,
             (op->oracle_info.mispred << 2 | op->oracle_info.misfetch << 1 |
              op->oracle_info.btb_miss) <= 0x7);

      inc_bstat_fetched(op);

      ic->off_path = ic->off_path || op->oracle_info.recover_at_decode || op->oracle_info.recover_at_exec;

      // Measuring basic block lengths
      /*static int bbl_len = 0;
      static int bbl_len_dont_end_pred_nt = 0;
      bbl_len++;
      bbl_len_dont_end_pred_nt++;
      if (op->table_info->cf_type) {
        STAT_EVENT(ic->proc_id, BBL_LENGTH_1 + bbl_len-1);
        bbl_len = 0;
        if (op->oracle_info.pred == TAKEN) {
          STAT_EVENT(ic->proc_id, BBL_DONT_END_PRED_NT_LENGTH_1 + bbl_len_dont_end_pred_nt-1);
          bbl_len_dont_end_pred_nt = 0;
        }
      }*/
    } else {
      // pass the global branch history to all the instructions
      op->oracle_info.pred_global_hist = g_bp_data->global_hist;
    }
  }
}

/**************************************************************************************/
/* icache_fill_line: */

Flag icache_fill_line(Mem_Req* req)  // cmp FIXME maybe needed to be optimized
{
  Addr         repl_line_addr;
  Addr         repl_line_addr2;
  Inst_Info**  line;
  Addr         dummy_addr;
  Addr         dummy_addr2;
  Icache_Data* line_info = NULL;
  UNUSED(line);

  // cmp
  if(model->id == CMP_MODEL) {
    set_icache_stage(&cmp_model.icache_stage[req->proc_id]);
  }

  ASSERT(ic->proc_id, ic->proc_id == req->proc_id);

  if(req->dirty_l0) {
    STAT_EVENT(ic->proc_id, DIRTY_WRITE_TO_ICACHE);
    printf("fetch_addr:%s line_addr:%s req_addr:%s off:%d\n",
           hexstr64s(ic->fetch_addr), hexstr64s(ic->line_addr),
           hexstr64s(req->addr), ic->off_path);
  }

  /* get new line in the cache */
  if((ic->line_addr == req->addr) && ((ic->state == WAIT_FOR_MISS) ||
                                      (ic->next_state == WAIT_FOR_MISS))) {
    INC_STAT_EVENT(ic->proc_id, MISS_WAIT_TIME, cycle_count - ic->wait_for_miss_start);
    if(IC_PREF_CACHE_ENABLE &&  // cmp FIXME prefetchers
       (USE_CONFIRMED_OFF ? req->off_path_confirmed : req->off_path)) {
      Addr pref_line_addr;

      line = (Inst_Info**)cache_insert(&ic->pref_icache, ic->proc_id,
                                       ic->fetch_addr, &pref_line_addr,
                                       &repl_line_addr);
      DEBUG(
        ic->proc_id,
        "Insert PREF_ICACHE fetch_addr0x:%s line_addr:%s index:%ld addr:0x%s\n",
        hexstr64(ic->fetch_addr), hexstr64(pref_line_addr),
        (long int)(req - mem->req_buffer), hexstr64s(req->addr));
      STAT_EVENT(ic->proc_id, IC_PREF_CACHE_FILL);
      ic->next_state = ic->after_waiting_state;
      return TRUE;
    }

    ic->line = (Inst_Info**)cache_insert(&ic->icache, ic->proc_id,
                                         ic->fetch_addr, &ic->line_addr,
                                         &repl_line_addr);
    DEBUG(ic->proc_id, "Got line switch into ic fetch %llx\n", ic->line_addr);
    STAT_EVENT(ic->proc_id, ICACHE_FILL);

    if(WP_COLLECT_STATS) {  // cmp IGNORE
      line_info = (Icache_Data*)cache_insert(&ic->icache_line_info, ic->proc_id,
                                             ic->fetch_addr, &dummy_addr2,
                                             &repl_line_addr2);
      if (line_info) {
        wp_process_icache_evicted(line_info, req, &repl_line_addr2);
        if(EIP_ENABLE)
          eip_cache_fill(ic->proc_id, req->addr, repl_line_addr2);
        line_info->fetched_by_offpath = USE_CONFIRMED_OFF ?
          req->off_path_confirmed :
          req->off_path;
        line_info->offpath_op_addr   = req->oldest_op_addr;
        line_info->offpath_op_unique = req->oldest_op_unique_num;
        line_info->fetch_cycle       = cycle_count;
        line_info->onpath_use_cycle  = req->off_path ? 0 : cycle_count;
        line_info->read_count[0]     = req->cyc_hit_by_demand_load? 1 : 0;
        line_info->read_count[1]     = 0;
        line_info->HW_prefetch       = req->type == MRT_IPRF;
        line_info->ghist             = req->ghist;
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
    ic->next_state = ic->after_waiting_state;

    if (req->demand_icache_emitted_cycle) {
      ASSERT(ic->proc_id, !req->fdip_emitted_cycle && (cycle_count - req->demand_icache_emitted_cycle > 0));
      STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_BY_ICACHE_DEMAND);
      INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_CYCLE_DELTA_BY_ICACHE_DEMAND, cycle_count - req->demand_icache_emitted_cycle);
    } else if (req->fdip_emitted_cycle) {
      ASSERT(ic->proc_id, !req->demand_icache_emitted_cycle);
      STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_BY_FDIP);
      INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_CYCLE_DELTA_BY_FDIP, cycle_count - req->fdip_emitted_cycle);
      if (req->cyc_hit_by_demand_load) {
        STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_BY_ON_FDIP_HIT_BY_DEMAND_LOAD + req->fdip_pref_off_path);
        INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_CYCLE_DELTA_BY_ON_FDIP_HIT_BY_DEMAND_LOAD + req->fdip_pref_off_path, cycle_count - req->cyc_hit_by_demand_load);
      }
    }
    INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ_CYCLE_DELTA, cycle_count - req->demand_icache_emitted_cycle);
  } else {
    if(IC_PREF_CACHE_ENABLE &&  // cmp FIXME prefetchers
       (USE_CONFIRMED_OFF ? req->off_path_confirmed : req->off_path)) {
      Addr pref_line_addr;

      line = (Inst_Info**)cache_insert(&ic->pref_icache, ic->proc_id, req->addr,
                                       &pref_line_addr, &repl_line_addr);
      DEBUG(
        ic->proc_id,
        "Insert PREF_ICACHE fetch_addr0x:%s line_addr:%s index:%ld addr:0x%s\n",
        hexstr64(req->addr), hexstr64(pref_line_addr),
        (long int)(req - mem->req_buffer), hexstr64s(req->addr));
      STAT_EVENT(ic->proc_id, IC_PREF_CACHE_FILL);

      return TRUE;
    }

    line = (Inst_Info**)cache_insert(&ic->icache, ic->proc_id, req->addr,
                                     &dummy_addr, &repl_line_addr);

    if(WP_COLLECT_STATS) {  // cmp IGNORE
      line_info = (Icache_Data*)cache_insert(&ic->icache_line_info, ic->proc_id,
                                             req->addr, &dummy_addr2,
                                             &repl_line_addr2);
      if (line_info) {
        STAT_EVENT(ic->proc_id, ICACHE_FILL);

        wp_process_icache_evicted(line_info, req, &repl_line_addr2);
        if(EIP_ENABLE)
          eip_cache_fill(ic->proc_id, req->addr, repl_line_addr2);
        line_info->fetched_by_offpath = USE_CONFIRMED_OFF ?
          req->off_path_confirmed :
          req->off_path;
        line_info->offpath_op_addr   = req->oldest_op_addr;
        line_info->offpath_op_unique = req->oldest_op_unique_num;
        line_info->fetch_cycle       = cycle_count;
        line_info->onpath_use_cycle  = req->off_path ? 0 : cycle_count;
        line_info->read_count[0]     = 0;
        line_info->read_count[1]     = 0;
        line_info->HW_prefetch       = req->type == MRT_IPRF;
	line_info->ghist             = req->ghist;
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
      INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_CYCLE_DELTA_BY_ICACHE_DEMAND, cycle_count - req->demand_icache_emitted_cycle);
    } else if (req->fdip_emitted_cycle) {
      ASSERT(ic->proc_id, !req->demand_icache_emitted_cycle);
      STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_BY_FDIP);
      INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_CYCLE_DELTA_BY_FDIP, cycle_count - req->fdip_emitted_cycle);
      if (req->cyc_hit_by_demand_load) {
        STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_BY_ON_FDIP_HIT_BY_DEMAND_LOAD + req->fdip_pref_off_path);
        INC_STAT_EVENT(ic->proc_id, ICACHE_FILL_INCORRECT_REQ_CYCLE_DELTA_BY_ON_FDIP_HIT_BY_DEMAND_LOAD + req->fdip_pref_off_path, cycle_count - req->cyc_hit_by_demand_load);
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
  Addr        repl_line_addr, inval_line_addr;
  Inst_Info** inserted_line = NULL;

  ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->fetch_addr)
  Inst_Info** line = (Inst_Info**)cache_access(&ic->pref_icache, ic->fetch_addr,
                                               &ic->line_addr, FALSE);

  if(ic->off_path && !PREFCACHE_MOVE_OFFPATH) {
    if(line) {
      DEBUG(ic->proc_id, "off_path ic_pref cache hit:fetch_addr:0x%s \n",
            hexstr64(ic->fetch_addr));
      STAT_EVENT(ic->proc_id, IC_PREF_CACHE_HIT_PER_OFFPATH);
      STAT_EVENT(ic->proc_id, IC_PREF_CACHE_HIT_OFFPATH);
    }
    return line;
  }

  if(line) {
    inserted_line = (Inst_Info**)cache_insert(&ic->icache, ic->proc_id,
                                              ic->fetch_addr, &ic->line_addr,
                                              &repl_line_addr);
    DEBUG(ic->proc_id, "ic_pref cache hit:fetch_addr:0x%s \n",
          hexstr64(ic->fetch_addr));
    STAT_EVENT(ic->proc_id, IC_PREF_MOVE_IC);

    STAT_EVENT(ic->proc_id, ICACHE_FILL_CORRECT_REQ);

    STAT_EVENT(ic->proc_id, IC_PREF_CACHE_HIT_PER + MIN2(ic->off_path, 1));
    STAT_EVENT(ic->proc_id, IC_PREF_CACHE_HIT + MIN2(ic->off_path, 1));
    cache_invalidate(&ic->pref_icache, ic->fetch_addr, &inval_line_addr);

    if(PREF_ICACHE_HIT_FILL_L1) {
      if(model->mem == MODEL_MEM) {
        Addr     line_addr;
        Cache*   l1_cache = &mem->uncores[ic->proc_id].l1->cache;
        L1_Data* l1_data  = (L1_Data*)cache_access(l1_cache, ic->fetch_addr,
                                                  &line_addr, TRUE);
        if(!l1_data) {
          Mem_Req tmp_req;
          tmp_req.addr     = ic->fetch_addr;
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
/* wp_process_icache_hit: */

void wp_process_icache_hit(Icache_Data* line, Addr fetch_addr) {
  L1_Data* l1_line;

  if(!WP_COLLECT_STATS)
    return;

  if(icache_off_path() == FALSE) {
    inc_icache_hit(ic->proc_id, ic->line_addr);
    if(line->fetched_by_offpath) {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_ONPATH_SAT_BY_OFFPATH);
      STAT_EVENT(ic->proc_id, ICACHE_USE_OFFPATH);
      STAT_EVENT(ic->proc_id, DIST_ICACHE_FILL_OFFPATH_USED);
      STAT_EVENT(ic->proc_id, DIST_REQBUF_OFFPATH_USED);
      STAT_EVENT(ic->proc_id, DIST2_REQBUF_OFFPATH_USED_FULL);

      l1_line = do_l1_access_addr(fetch_addr);
      if(l1_line) {
        if(l1_line->fetched_by_offpath) {
          STAT_EVENT(ic->proc_id, L1_USE_OFFPATH);
          STAT_EVENT(ic->proc_id, DIST_L1_FILL_OFFPATH_USED);
          STAT_EVENT(ic->proc_id, L1_USE_OFFPATH_IFETCH);
          l1_line->fetched_by_offpath             = FALSE;
          l1_line->l0_modified_fetched_by_offpath = TRUE;
        }
      }
    } else {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_ONPATH_SAT_BY_ONPATH);
      STAT_EVENT(ic->proc_id, ICACHE_USE_ONPATH);
    }
  } else {
    if(line->fetched_by_offpath) {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_OFFPATH_SAT_BY_OFFPATH);
    } else {
      STAT_EVENT(ic->proc_id, ICACHE_HIT_OFFPATH_SAT_BY_ONPATH);
    }
  }

  if(!line->read_count[0]) { // only consider the first hit
    if(!icache_off_path()) {
      uns64 hashed_addr = FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(ic->line_addr, line->ghist) : ic->line_addr;
      if(fdip_search_pref_candidate(ic->line_addr)) {
        inc_cnt_useful(ic->proc_id, hashed_addr, FALSE);
        inc_cnt_useful_signed(ic->proc_id, hashed_addr);
        inc_useful_lines_uc(ic->proc_id, hashed_addr);
        update_useful_lines_uc(ic->proc_id, hashed_addr);
        update_useful_lines_bloom_filter(ic->proc_id, hashed_addr);
        inc_utility_info(ic->proc_id, TRUE);
        inc_timeliness_info(ic->proc_id, FALSE);
      }
      if(line->FDIP_prefetch == FDIP_BOTHPATH || line->FDIP_prefetch == FDIP_ONPATH)
        STAT_EVENT(ic->proc_id, ICACHE_HIT_BY_FDIP_ONPATH);
      else if(line->FDIP_prefetch == FDIP_OFFPATH)
        STAT_EVENT(ic->proc_id, ICACHE_HIT_BY_FDIP_OFFPATH);
      line->read_count[0] += 1;
    }
    if(line->FDIP_prefetch)
      STAT_EVENT(ic->proc_id, ICACHE_HIT_ONPATH_BY_FDIP + icache_off_path());
  }

  if(icache_off_path() == FALSE)
    line->fetched_by_offpath = FALSE;
}

/**************************************************************************************/
/* wp_process_icache_evicted: */

void wp_process_icache_evicted(Icache_Data* line, Mem_Req* req, Addr* repl_line_addr) {
  if(!WP_COLLECT_STATS)
    return;

  if(*repl_line_addr && !line->read_count[0]) {
    DEBUG(ic->proc_id, "%llx is evicted without hit, FDIP pref: %d\n", *repl_line_addr, line->FDIP_prefetch);
    INC_STAT_EVENT(ic->proc_id, ICACHE_UNUSEFUL_CL_CYC, cycle_count - line->fetch_cycle);
    STAT_EVENT(ic->proc_id, ICACHE_UNUSEFUL_CL);
    if(line->FDIP_prefetch && (FDIP_UTILITY_ONLY_TRAIN_OFF_PATH ? line->FDIP_prefetch >= FDIP_OFFPATH : TRUE)) {
      uns64 hashed_addr = FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(*repl_line_addr, line->ghist) : *repl_line_addr;
      inc_cnt_unuseful(ic->proc_id, hashed_addr);
      dec_cnt_useful_signed(ic->proc_id, hashed_addr);
      dec_useful_lines_uc(ic->proc_id, hashed_addr);
      update_unuseful_lines_uc(ic->proc_id, hashed_addr);
      inc_utility_info(ic->proc_id, FALSE);
      STAT_EVENT(ic->proc_id, ICACHE_EVICT_MISS_ONPATH_BY_FDIP + icache_off_path());
      if(line->FDIP_prefetch == FDIP_BOTHPATH || line->FDIP_prefetch == FDIP_ONPATH)
        STAT_EVENT(ic->proc_id, ICACHE_EVICT_MISS_BY_FDIP_ONPATH);
      else
        STAT_EVENT(ic->proc_id, ICACHE_EVICT_MISS_BY_FDIP_OFFPATH);
    }
  } else if(*repl_line_addr && line->read_count[0]) {
    DEBUG(ic->proc_id, "%llx is evicted with hits, FDIP pref: %d\n", *repl_line_addr, line->FDIP_prefetch);
    if (line->FDIP_prefetch) {
      uns64 hashed_addr = FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(*repl_line_addr, line->ghist) : *repl_line_addr;
      add_evict_seq(ic->proc_id, hashed_addr);
      STAT_EVENT(ic->proc_id, ICACHE_EVICT_HIT_ONPATH_BY_FDIP + icache_off_path());
      if(line->FDIP_prefetch == FDIP_BOTHPATH || line->FDIP_prefetch == FDIP_ONPATH)
        STAT_EVENT(ic->proc_id, ICACHE_EVICT_HIT_BY_FDIP_ONPATH);
      else
        STAT_EVENT(ic->proc_id, ICACHE_EVICT_HIT_BY_FDIP_OFFPATH);
    }
  }

  if(FDIP_ENABLE && *repl_line_addr)
    evict_prefetched_cls(ic->proc_id, *repl_line_addr, (mem_req_is_type(req, MRT_FDIPPRFON) || mem_req_is_type(req, MRT_FDIPPRFOFF))? TRUE : FALSE);
}

/**************************************************************************************/
/* wp_process_icache_fill: */

void wp_process_icache_fill(Icache_Data* line, Mem_Req* req) {
  if(!WP_COLLECT_STATS)
    return;

  if((req->type == MRT_WB) || (req->type == MRT_WB_NODIRTY) ||
     (req->type == MRT_IPRF)) /* for now we don't consider prefetches */
    return;

  if(req->off_path) {
    STAT_EVENT(ic->proc_id, ICACHE_FILL_OFFPATH);
  } else {
    STAT_EVENT(ic->proc_id, ICACHE_FILL_ONPATH);
    if(req->onpath_match_offpath)
      STAT_EVENT(ic->proc_id, DIST_ICACHE_FILL_ONPATH_PARTIAL);
    else
      STAT_EVENT(ic->proc_id, DIST_ICACHE_FILL_ONPATH);
  }
  STAT_EVENT(ic->proc_id, DIST_ICACHE_FILL);
}

/**************************************************************************************/
/* inst_lost_get_full_window_reason(): */

int32_t inst_lost_get_full_window_reason() {
  if(rob_stall_reason != ROB_STALL_NONE) {
    return rob_stall_reason;
  }

  if(rob_block_issue_reason != ROB_BLOCK_ISSUE_NONE) {
    return rob_block_issue_reason;
  }

  return 0;
}

void log_stats_ic_miss() {
  STAT_EVENT(ic->proc_id, ICACHE_MISS);
  STAT_EVENT(ic->proc_id, POWER_ICACHE_MISS);
  STAT_EVENT(ic->proc_id, ICACHE_MISS_ONPATH + ic->off_path);
}

void log_stats_ic_hit() {
  STAT_EVENT(ic->proc_id, ICACHE_HIT);
  STAT_EVENT(ic->proc_id, ICACHE_HIT_ONPATH + ic->off_path);
}

void log_stats_mshr_hit(Addr line_addr) {
  Flag demand_hit_prefetch = FALSE;
  Flag demand_hit_writeback = FALSE;
  Mem_Queue_Entry* queue_entry = NULL;
  Flag ramulator_match = FALSE;
  Mem_Req* req = mem_search_reqbuf_wrapper(ic->proc_id, line_addr,
                                           MRT_FDIPPRFON, ICACHE_LINE_SIZE, &demand_hit_prefetch, &demand_hit_writeback,
                                           QUEUE_MLC | QUEUE_L1 | QUEUE_BUS_OUT |
                                           QUEUE_MEM | QUEUE_L1FILL | QUEUE_MLC_FILL,
                                           &queue_entry, &ramulator_match);
  if (!req) {
    req = mem_search_reqbuf_wrapper(ic->proc_id, line_addr,
                                    MRT_FDIPPRFOFF, ICACHE_LINE_SIZE, &demand_hit_prefetch, &demand_hit_writeback,
                                    QUEUE_MLC | QUEUE_L1 | QUEUE_BUS_OUT |
                                    QUEUE_MEM | QUEUE_L1FILL | QUEUE_MLC_FILL,
                                    &queue_entry, &ramulator_match);
  }

  if (req && !req->cyc_hit_by_demand_load) {
    uns64 hashed_addr = FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(ic->line_addr, req->ghist) : ic->line_addr;
    if (!icache_off_path() &&
        fdip_search_pref_candidate(ic->line_addr)) {
      inc_cnt_useful(ic->proc_id, hashed_addr, FALSE);
      inc_cnt_useful_signed(ic->proc_id, hashed_addr);
      inc_useful_lines_uc(ic->proc_id, hashed_addr);
      update_useful_lines_uc(ic->proc_id, hashed_addr);
      update_useful_lines_bloom_filter(ic->proc_id, hashed_addr);
      inc_utility_info(ic->proc_id, TRUE);
      inc_timeliness_info(ic->proc_id, TRUE);
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
    inc_icache_miss(ic->proc_id, ic->line_addr);
  Imiss_Reason imiss_reason = get_miss_reason(ic->proc_id, line_addr);
  DEBUG_FDIP(ic->proc_id, "miss reason: %d, req: %d\n", imiss_reason, req? 1:0);
  if (!req) {
    if (!icache_off_path()) {
      uns64 hashed_addr = FDIP_GHIST_HASHING ? fdip_hash_addr_ghist(ic->line_addr, g_bp_data->global_hist) : ic->line_addr;
      DEBUG_FDIP(ic->proc_id, "learn missed line %llx\n", ic->line_addr);
      inc_cnt_useful(ic->proc_id, hashed_addr, TRUE);
      inc_cnt_useful_signed(ic->proc_id, hashed_addr);
      inc_useful_lines_uc(ic->proc_id, hashed_addr);
      update_useful_lines_uc(ic->proc_id, hashed_addr);
      update_useful_lines_bloom_filter(ic->proc_id, hashed_addr);
      inc_utility_info(ic->proc_id, TRUE);
      if (imiss_reason == IMISS_TOO_EARLY_EVICTED_BY_IFETCH)
        STAT_EVENT(ic->proc_id, ICACHE_MISS_PREFETCHED_AND_EVICTED_BY_IFETCH);
      else if (imiss_reason == IMISS_TOO_EARLY_EVICTED_BY_FDIP)
        STAT_EVENT(ic->proc_id, ICACHE_MISS_PREFETCHED_AND_EVICTED_BY_FDIP);
      else {
        ASSERT(ic->proc_id, imiss_reason == IMISS_NOT_PREFETCHED);
        STAT_EVENT(ic->proc_id, ICACHE_MISS_NOT_PREFETCHED);
        assert_fdip_break_reason(ic->proc_id, hashed_addr);
      }
    } else {
      inc_off_fetched_cls(ic->line_addr);
    }
    if (imiss_reason == IMISS_TOO_EARLY_EVICTED_BY_FDIP || imiss_reason == IMISS_TOO_EARLY_EVICTED_BY_IFETCH)
      STAT_EVENT(ic->proc_id, ICACHE_MISS_TOO_EARLY_ONPATH + icache_off_path());
    else
      STAT_EVENT(ic->proc_id, ICACHE_MISS_NOT_PREFETCHED_ONPATH + icache_off_path());
  } else {
    if (FDIP_ENABLE && !FDIP_UTILITY_HASH_ENABLE && !FDIP_BLOOM_FILTER && !FDIP_UC_SIZE && !EIP_ENABLE && !FDIP_PERFECT_PREFETCH
        && (mem_req_is_type(req, MRT_FDIPPRFON) || mem_req_is_type(req, MRT_FDIPPRFOFF)))
      ASSERT(ic->proc_id, imiss_reason == IMISS_MSHR_HIT_PREFETCHED_OFFPATH || imiss_reason == IMISS_MSHR_HIT_PREFETCHED_ONPATH);
    if (imiss_reason == IMISS_MSHR_HIT_PREFETCHED_ONPATH)
      STAT_EVENT(ic->proc_id, ICACHE_MISS_MSHR_HIT_PREFETCHED_ONPATH);
    else
      STAT_EVENT(ic->proc_id, ICACHE_MISS_MSHR_HIT_PREFETCHED_OFFPATH);
  }
  DEBUG_FDIP(ic->proc_id, "set last miss reason %u for %llx\n", imiss_reason, ic->line_addr);
  set_last_miss_reason(ic->proc_id, imiss_reason);
}

// Wrapper callback for any instruction memreq.
// This must always return TRUE so that memreq is satisfied that
// done_func is finished and does not need to be retried.
Flag instr_fill_line(Mem_Req* req) {
  ASSERT(ic->proc_id, req->type == MRT_IPRF || req->type == MRT_FDIPPRFON || req->type == MRT_FDIPPRFOFF || req->type == MRT_UOCPRF || req->type == MRT_IFETCH);
  if (mem_req_is_type(req, MRT_IFETCH) || mem_req_is_type(req, MRT_IPRF) || mem_req_is_type(req, MRT_FDIPPRFON) || mem_req_is_type(req, MRT_FDIPPRFOFF)) {
    icache_fill_line(req);
    DEBUG(ic->proc_id, "line 0x%llx is filled into icache\n", req->addr);
  }

  return TRUE;
}

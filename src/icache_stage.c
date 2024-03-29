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
#include "packet_build.h"
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
#include "decoupled_frontend.h"

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

#define STAGE_MAX_OP_COUNT  UC_ISSUE_WIDTH
#define DUMMY_ADDR_UC_FETCH 0x1

/**************************************************************************************/
/* Global Variables */

/**************************************************************************************/

Pb_Data* ic_pb_data;  // cmp cne is fine for cmp now assuming homogeneous cmp
// But decided to use array for future use

Icache_Stage* ic = NULL;

extern Cmp_Model              cmp_model;
extern Memory*                mem;
extern Rob_Stall_Reason       rob_stall_reason;
extern Rob_Block_Issue_Reason rob_block_issue_reason;

/**************************************************************************************/
/* Local prototypes */

static inline Icache_State icache_issue_ops(Break_Reason*, uns*,
                                            Inst_Info** line);
static inline Inst_Info**  lookup_cache(void);
static Inst_Info**         ic_pref_cache_access(void);
int32_t                    inst_lost_get_full_window_reason(void);
static inline void         log_stats_ic_miss(void);
static inline void         log_stats_mshr_hit(Addr line_addr);
static inline void         update_stats_bf_retired(void);

/**************************************************************************************/
/* set_icache_stage: */

void set_icache_stage(Icache_Stage* new_ic) {
  ic = new_ic;
}

/**************************************************************************************/
/* set_pb_data: */

void set_pb_data(Pb_Data* new_pb_data) {
  ic_pb_data = new_pb_data;
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
  ic->sd.max_op_count = STAGE_MAX_OP_COUNT;
  ic->sd.ops          = (Op**)malloc(sizeof(Op*) * STAGE_MAX_OP_COUNT);

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

  if(model->id != CMP_MODEL)
    ic_pb_data = (Pb_Data*)malloc(sizeof(Pb_Data));
  ic_pb_data->proc_id = proc_id;
  init_packet_build(ic_pb_data, PB_ICACHE);

  if(IC_PREF_CACHE_ENABLE)
    init_cache(&ic->pref_icache, "IC_PREF_CACHE", IC_PREF_CACHE_SIZE,
               IC_PREF_CACHE_ASSOC, ICACHE_LINE_SIZE, 0, REPL_TRUE_LRU);

  memset(ic->rand_wb_state, 0, NUM_ELEMENTS(ic->rand_wb_state));
}

/**************************************************************************************/
/* icache_init_trace:  */

void init_icache_trace() {
  ic->next_fetch_addr = decoupled_fe_next_fetch_addr(ic->proc_id);
  ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->next_fetch_addr)
}

/**************************************************************************************/
/* reset_icache_stage: */

void reset_icache_stage() {
  uns ii;
  for(ii = 0; ii < STAGE_MAX_OP_COUNT; ii++)
    ic->sd.ops[ii] = NULL;
  ic->sd.op_count = 0;

  /* set up the initial fetch state */
  ic->next_fetch_addr = td->inst_addr;
  ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->next_fetch_addr);
  ic->off_path                       = FALSE;
  ic->back_on_path                   = FALSE;
  op_count[ic->proc_id]              = 1;
  unique_count_per_core[ic->proc_id] = 1;
}

/**************************************************************************************/
/* reset_all_ops_icache_stage: */
// CMP used for bogus run: may be combined with reset_icache_stage
void reset_all_ops_icache_stage() {
  uns ii;
  for(ii = 0; ii < STAGE_MAX_OP_COUNT; ii++)
    ic->sd.ops[ii] = NULL;
  ic->sd.op_count = 0;

  /* set up the initial fetch state */
  ic->next_fetch_addr = td->inst_addr;
  ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->next_fetch_addr);
  ic->off_path     = FALSE;
  ic->back_on_path = FALSE;
}

/**************************************************************************************/
/* recover_icache_stage: */

void recover_icache_stage() {
  Stage_Data* cur = &ic->sd;
  uns         ii;

  ASSERT(ic->proc_id, ic->proc_id == bp_recovery_info->proc_id);
  DEBUG(ic->proc_id,
        "Icache stage recovery signaled.  recovery_fetch_addr: 0x%s\n",
        hexstr64s(bp_recovery_info->recovery_fetch_addr));
  cur->op_count = 0;
  for(ii = 0; ii < ic->sd.max_op_count; ii++) {
    if(cur->ops[ii]) {
      if(FLUSH_OP(cur->ops[ii])) {
        ASSERT(ic->proc_id, cur->ops[ii]->off_path);
        free_op(cur->ops[ii]);
        cur->ops[ii] = NULL;

      } else {
        cur->op_count++;
      }
    }
  }

  ic->back_on_path = !bp_recovery_info->recovery_force_offpath;

  Op* op = bp_recovery_info->recovery_op;
  if(bp_recovery_info->late_bp_recovery && op->oracle_info.btb_miss &&
     !op->oracle_info.btb_miss_resolved) {
    // Late branch predictor recovered before btb miss is resolved (i.e., icache
    // stage should still wait for redirect)
  } else {
    if(ic->next_state != IC_FILL && ic->next_state != IC_WAIT_FOR_MISS) {
      ic->next_state = IC_FETCH;
    }
    if(SWITCH_IC_FETCH_ON_RECOVERY && model->id == CMP_MODEL) {
      ic->next_state = IC_FETCH;
    }
  }
  op_count[ic->proc_id] = bp_recovery_info->recovery_op_num + 1;
  ic->next_fetch_addr   = bp_recovery_info->recovery_fetch_addr;
  ASSERT(ic->proc_id, ic->next_fetch_addr);
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
  DPRINTF("# %-10s  op_count:%d ", ic->sd.name, ic->sd.op_count);
  DPRINTF(
    "fetch_addr:0x%s  next_fetch_addr:0x%s  path:%s  state:%s  next_state:%s\n",
    hexstr64s(ic->fetch_addr), hexstr64s(ic->next_fetch_addr),
    ic->off_path ? "OFF_PATH" : "ON_PATH ", icache_state_names[ic->state],
    icache_state_names[ic->next_state]);

  // print icache stage
  DPRINTF("# %-10s  op_count:%d\n", "ICache", ic->sd.op_count);
  print_op_array(GLOBAL_DEBUG_STREAM, ic->sd.ops, STAGE_MAX_OP_COUNT,
                 ic->sd.op_count);
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
/* lookup_cache: returns instr if found in either uop cache or icache 
 *                If icache miss but UC hit, set ic->line to non-null  
 */
Inst_Info** lookup_cache() {
  Inst_Info** line = NULL;
  line = (Inst_Info**)cache_access(&ic->icache, ic->fetch_addr,
                                             &ic->line_addr, TRUE);
  ASSERT(0, line != (Inst_Info**)DUMMY_ADDR_UC_FETCH);
  if(PERFECT_ICACHE && !line)
    line = (Inst_Info**)INIT_CACHE_DATA_VALUE;

  if (in_uop_cache(ic->fetch_addr, FALSE, ic->off_path) && !line) {
    // Should return Inst_Info, but op hasn't been fetched yet, so just give it any 
    // value. This is not used anyway
    line = (Inst_Info**)DUMMY_ADDR_UC_FETCH;
  }
  return line;
}

/**************************************************************************************/
/* icache_cycle: */

void update_icache_stage() {
  uns          cf_num    = 0;
  Icache_Data* line_info = NULL;
  Addr         dummy_addr;

  STAT_EVENT(ic->proc_id, ICACHE_CYCLE);
  STAT_EVENT(ic->proc_id, ICACHE_CYCLE_ONPATH + ic->off_path);
  INC_STAT_EVENT(ic->proc_id, INST_LOST_TOTAL, IC_ISSUE_WIDTH);

  ic->state = ic->next_state;

  if (ic->off_path)
    STAT_EVENT(exec->proc_id, ICACHE_STAGE_OFF_PATH);

  if(ic->sd.op_count) {
    STAT_EVENT(ic->proc_id, FETCH_0_OPS);
    INC_STAT_EVENT(ic->proc_id,
                   INST_LOST_FULL_WINDOW + inst_lost_get_full_window_reason(),
                   IC_ISSUE_WIDTH);
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

  switch(ic->state) {
    case IC_FETCH: {
      Break_Reason break_fetch = BREAK_DONT;

      ic->off_path &= !ic->back_on_path;
      ic->back_on_path = FALSE;

      STAT_EVENT(ic->proc_id, FETCH_ON_PATH + ic->off_path);

      reset_packet_build(ic_pb_data);  // reset packet build counters

      while(!break_fetch) {

        ic->fetch_addr = ic->next_fetch_addr;
        ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->fetch_addr)

        if(ic->proc_id)
          ASSERTM(ic->proc_id, ic->fetch_addr, "ic fetch addr: %llu\n",
                  ic->fetch_addr);

        ic->line = lookup_cache();

        if(WP_COLLECT_STATS && ic->line != (Inst_Info**)DUMMY_ADDR_UC_FETCH)  // CMP remove?
          line_info = (Icache_Data*)cache_access(
            &ic->icache_line_info, ic->fetch_addr, &dummy_addr, TRUE);

        if(IC_PREF_CACHE_ENABLE && (!ic->line))
          ic->line = ic_pref_cache_access();


        STAT_EVENT(ic->proc_id, POWER_ITLB_ACCESS);
        STAT_EVENT(ic->proc_id, POWER_ICACHE_ACCESS);
        STAT_EVENT(ic->proc_id, POWER_BTB_READ);

        // ideal L2 Icache prefetcher
        if(IDEAL_L2_ICACHE_PREFETCHER) {
          Addr     line_addr;
          L1_Data* data;
          Cache*   l1_cache = model->mem == MODEL_MEM ?
                              &mem->uncores[ic->proc_id].l1->cache :
                              NULL;
          data = l1_cache ? (L1_Data*)cache_access(l1_cache, ic->fetch_addr,
                                                   &line_addr, TRUE) :
                            NULL;
          if(data) {  // second level cache hit
            STAT_EVENT(ic->proc_id, L2_IDEAL_FILL_ICACHE);
          } else
            STAT_EVENT(ic->proc_id, L2_IDEAL_MISS_ICACHE);
        }

        if (ic->line == NULL) { // cache miss
          DEBUG(ic->proc_id, "Cache miss on op_num:%s @ 0x%s\n",
                unsstr64(op_count[ic->proc_id]), hexstr64s(ic->fetch_addr));
          log_stats_mshr_hit(ic->line_addr);
          log_stats_ic_miss();

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
              ic->next_state = IC_WAIT_FOR_MISS;
              DEBUG(ic->proc_id, "from IC_STAGE for cl 0x%llx at cycle %llu\n", ic->line_addr, cycle_count);
              STAT_EVENT(ic->proc_id, ICACHE_STAGE_MISS);
              ic->wait_for_miss_start = cycle_count;

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
          }
          if(EIP_ENABLE)
            eip_prefetch(ic->proc_id, ic->fetch_addr, 0, 0, ic->off_path);
          if(DJOLT_ENABLE)
            djolt_prefetch(ic->proc_id, ic->fetch_addr, 0, 0);
          if(FNLMMA_ENABLE)
            fnlmma_prefetch(ic->proc_id, ic->fetch_addr, 0, 0);
          break_fetch = BREAK_ICACHE_MISS;
        } else if (ic->line == (Inst_Info**) DUMMY_ADDR_UC_FETCH) { // icache miss, uc hit
          log_stats_ic_miss();
          // start a memreq to fill icache, but do not cause any stalls. 
          // Use for more inclusivity between IC and UC
          if (IPRF_ON_UOP_CACHE_HIT)
            new_mem_req(MRT_IPRF, ic->proc_id, ic->line_addr,
                           ICACHE_LINE_SIZE, 0, NULL, instr_fill_line,
                           unique_count,
                           0);
          if(EIP_ENABLE)
            eip_prefetch(ic->proc_id, ic->fetch_addr, 0, 0, ic->off_path);
          if(DJOLT_ENABLE)
            djolt_prefetch(ic->proc_id, ic->fetch_addr, 0, 0);
          if(FNLMMA_ENABLE)
            fnlmma_prefetch(ic->proc_id, ic->fetch_addr, 0, 0);
          ic->next_state = icache_issue_ops(&break_fetch, &cf_num, ic->line);
        } else { /* icache hit. Can be either UC hit or miss */
          DEBUG(ic->proc_id, "Cache hit on op_num:%s @ 0x%s line_addr 0x%s\n",
                unsstr64(op_count[ic->proc_id]), hexstr64s(ic->fetch_addr), hexstr64s(ic->fetch_addr & ~0x3F));
          static uint64_t last_line = 0;
          if ((ic->fetch_addr >> LOG2(ICACHE_LINE_SIZE)) != last_line) {
            STAT_EVENT(ic->proc_id, ICACHE_HIT);
            STAT_EVENT(ic->proc_id, ICACHE_HIT_ONPATH + ic->off_path);
          }
          last_line = (ic->fetch_addr >> LOG2(ICACHE_LINE_SIZE));
          if(WP_COLLECT_STATS) {
            ASSERT(ic->proc_id, line_info);
            wp_process_icache_hit(line_info, ic->fetch_addr);
          }
          if(EIP_ENABLE)
            eip_prefetch(ic->proc_id, ic->fetch_addr, 1, 0, ic->off_path);
          if(DJOLT_ENABLE)
            djolt_prefetch(ic->proc_id, ic->fetch_addr, 1, 0);
          if(FNLMMA_ENABLE)
            fnlmma_prefetch(ic->proc_id, ic->fetch_addr, 1, 0);
          ic->next_state = icache_issue_ops(&break_fetch, &cf_num, ic->line);
        }
      }
      INC_STAT_EVENT(ic->proc_id, INST_LOST_BREAK_DONT + break_fetch,
                     IC_ISSUE_WIDTH > ic->sd.op_count ? IC_ISSUE_WIDTH - ic->sd.op_count
                                                           : 0);
      STAT_EVENT(ic->proc_id, FETCH_0_OPS + ic->sd.op_count);
      STAT_EVENT(ic->proc_id, ST_BREAK_DONT + break_fetch);
      if(!ic->off_path) {
        if (!ic->sd.op_count)
          STAT_EVENT(map->proc_id, ICACHE_STAGE_STARVED);
        else
          STAT_EVENT(map->proc_id, ICACHE_STAGE_NOT_STARVED);
      }
    } break;

    case IC_WAIT_FOR_MISS: {
      DEBUG(ic->proc_id, "Ifetch barrier: Waiting for miss \n");
      INC_STAT_EVENT(ic->proc_id, INST_LOST_WAIT_FOR_ICACHE_MISS, IC_ISSUE_WIDTH - 1);
      STAT_EVENT(ic->proc_id, FETCH_0_OPS);
      if(!ic->off_path) {
        STAT_EVENT(map->proc_id, ICACHE_STAGE_STARVED);
        INC_STAT_EVENT(ic->proc_id, INST_LOST_WAIT_FOR_ICACHE_MISS_NOT_PREFETCHED + get_last_miss_reason(ic->proc_id), IC_ISSUE_WIDTH - 1);
      }
    } break;

    case IC_WAIT_FOR_EMPTY_ROB: {
      DEBUG(ic->proc_id, "Ifetch barrier: Waiting for ROB to become empty \n");
      INC_STAT_EVENT(ic->proc_id, INST_LOST_WAIT_FOR_EMPTY_ROB, IC_ISSUE_WIDTH);
      STAT_EVENT(ic->proc_id, FETCH_0_OPS);
      if(td->seq_op_list.count == 0) {
        update_stats_bf_retired();
        ic->next_state = IC_FETCH;
      }
    } break;

    case IC_WAIT_FOR_TIMER: {
      DEBUG(ic->proc_id, "Ifetch barrier: Waiting for timer \n");
      INC_STAT_EVENT(ic->proc_id, INST_LOST_WAIT_FOR_TIMER, IC_ISSUE_WIDTH);
      STAT_EVENT(ic->proc_id, FETCH_0_OPS);
      if(!ic->off_path) {
        STAT_EVENT(map->proc_id, ICACHE_STAGE_STARVED);
      }
      if(cycle_count >= ic->timer_cycle)
        ic->next_state = IC_FETCH;
    } break;

    case IC_WAIT_FOR_RENAME: {
      DEBUG(ic->proc_id, "Ifetch barrier: Waiting for renaming register free \n");
      INC_STAT_EVENT(ic->proc_id, INST_LOST_WAIT_FOR_RENAME, IC_ISSUE_WIDTH);
      STAT_EVENT(ic->proc_id, FETCH_0_OPS);

      if (rename_table_available()) {
        DEBUG(ic->proc_id, "Renaming Stall Recover\n");
        ic->next_state = IC_FETCH;
      }
    } break;

    default:
      FATAL_ERROR(ic->proc_id, "Invalid icache state.\n");
  }
}


/**************************************************************************************/
/* update_bf_uoc_stats: */

void update_stats_bf_retired(void) {
  Flag next_op_in_uop_cache = in_uop_cache(ic->next_fetch_addr, FALSE, ic->off_path);
  if (!next_op_in_uop_cache) {
    // The micro-op cache can reduce the time to refill the pipeline after a fetch barrier.
    int uop_queue_length = get_uop_queue_stage_length();
    int decode_stages_filled = get_decode_stages_filled();
    ASSERT(ic->proc_id, uop_queue_length + decode_stages_filled <= 20);  // Stat supports up to 20.
    STAT_EVENT(ic->proc_id, BF_UOP_CACHE_MISS_UOP_QUEUE_LENGTH_0 + uop_queue_length);
    STAT_EVENT(ic->proc_id, BF_UOP_CACHE_MISS_UOP_QUEUE_PLUS_DECODE_LENGTH_0 + uop_queue_length + decode_stages_filled);
  }
}


/**************************************************************************************/
/* icache_issue_ops: On a cache hit, select ops to pass down to the decode
   stage.  Each op that gets issued is executed by the oracle. It will only
   process up to the first control flow operation and then return.  If
   FETCH_ACROSS_CACHE_LINES is set, it will be called again until break_fetch
   becomes true. */

static inline Icache_State icache_issue_ops(Break_Reason* break_fetch,
                                            uns* cf_num, Inst_Info** line) {
  Packet_Build_Condition packet_break = PB_BREAK_DONT;
  static uns last_icache_issue_time = 0; /* for computing fetch break latency */
  static Counter issued_real_inst   = 0;
  static Counter issued_uop         = 0;
  uns            fetch_lag;

  ASSERT(ic->proc_id, ic->proc_id == td->proc_id);

  fetch_lag              = cycle_count - last_icache_issue_time;
  last_icache_issue_time = cycle_count;

  while(1) {
    Op*        op = NULL;
    Inst_Info* inst = 0;
    UNUSED(inst);

    if (REG_RENAMING_TABLE_ENABLE && !rename_table_available()) {
      DEBUG(ic->proc_id, "Renaming Stall: %lld\n", op->unique_num);
      *break_fetch = BREAK_RENAME;
      return IC_WAIT_FOR_RENAME;
    }

    if(decoupled_fe_can_fetch_op(ic->proc_id)) {
      decoupled_fe_fetch_op(&op, ic->proc_id);
      ASSERTM(ic->proc_id, ic->next_fetch_addr == op->inst_info->addr,
               "Fetch address 0x%llx does not match op address 0x%llx\n",
               ic->next_fetch_addr, op->inst_info->addr);
      ASSERTM(ic->proc_id, ic->off_path == op->off_path,
              "Inconsistent off-path op PC: %llx ic:%i op:%i\n", op->inst_info->addr, ic->off_path, op->off_path);
      op->fetch_addr = ic->next_fetch_addr;
      ASSERT_PROC_ID_IN_ADDR(ic->proc_id, op->fetch_addr)
      op->off_path  = ic->off_path;
      td->inst_addr = op->inst_info->addr;  // FIXME: BUG 54
      ASSERT_PROC_ID_IN_ADDR(ic->proc_id, td->inst_addr);
      if(!op->off_path) {
        if(op->eom)
          issued_real_inst++;
        issued_uop++;
      }
      inst = op->inst_info;
    } else {
      *break_fetch = BREAK_BARRIER;
      return IC_FETCH;
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

    packet_break = packet_build(ic_pb_data, break_fetch, op);
    if(packet_break == PB_BREAK_BEFORE) {
      decoupled_fe_return_op(op);
      break;
    }

    /* add to sequential op list */
    add_to_seq_op_list(td, op);

    ASSERT(ic->proc_id, td->seq_op_list.count <= op_pool_active_ops);

    // Log uop cache access - must be called exactly once for every op, so it cannot be called when PB_BREAK_BEFORE
    in_uop_cache(op->inst_info->addr, TRUE, ic->off_path);

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

    ic->sd.ops[ic->sd.op_count] = op; /* put op in the exit list */
    op_count[ic->proc_id]++;          /* increment instruction counters */
    unique_count_per_core[ic->proc_id]++;
    unique_count++;
    /* check trigger */
    if(op->inst_info->trigger_op_fetched_hook)
      model->op_fetched_hook(op);

    /* move on to next instruction in the cache line */
    ic->sd.op_count++;
    INC_STAT_EVENT(ic->proc_id, INST_LOST_FETCH + ic->off_path, 1);

    DEBUG(ic->proc_id,
          "Fetching op from Icache addr: %s off: %d inst_info: %p ii_addr: %s "
          "dis: %s opnum: (%s:%s)\n",
          hexstr64s(op->inst_info->addr), op->off_path, op->inst_info,
          hexstr64s(op->inst_info->addr), disasm_op(op, TRUE),
          unsstr64(op->op_num), unsstr64(op->unique_num));

    /* figure out next address after current instruction */
    if(op->table_info->cf_type) {
      // For pipeline gating
      if(op->table_info->cf_type == CF_CBR)
        td->td_info.fetch_br_count++;

      ic->next_fetch_addr       = op->oracle_info.pred_npc;
      if(DJOLT_ENABLE)
        update_djolt(ic->proc_id, op->fetch_addr, op->table_info->cf_type, op->oracle_info.pred_npc);
      ASSERT(ic->proc_id, ic->next_fetch_addr);
      // initially bp_predict_op can return a garbage, for multi core run,
      // addr must follow cmp addr convention
      ic->next_fetch_addr = convert_to_cmp_addr(ic->proc_id,
                                                ic->next_fetch_addr);
      ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->next_fetch_addr);

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

      /* if it's a taken branch, wait for timer */
      if(FETCH_BREAK_ON_TAKEN && op->oracle_info.pred &&
         *break_fetch != BREAK_BARRIER) {
        *break_fetch = BREAK_TAKEN;
        if(FETCH_TAKEN_BUBBLE_CYCLES >= 1) {
          ic->timer_cycle = cycle_count + FETCH_TAKEN_BUBBLE_CYCLES;
          return IC_WAIT_FOR_TIMER;
        } else
          return IC_FETCH;
      }
    } else {
      if(op->eom) {
        ic->next_fetch_addr = ADDR_PLUS_OFFSET(
                                               ic->next_fetch_addr, op->inst_info->trace_info.inst_size);
        ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->next_fetch_addr);
      }
      // pass the global branch history to all the instructions
      op->oracle_info.pred_global_hist = g_bp_data->global_hist;
      ASSERT_PROC_ID_IN_ADDR(ic->proc_id, ic->next_fetch_addr);
    }

    if(packet_break == PB_BREAK_AFTER)
      break;
  }

  if(*break_fetch == BREAK_BARRIER) {
    return IC_WAIT_FOR_EMPTY_ROB;
  }

  return IC_FETCH;
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
  if((ic->line_addr == req->addr) && ((ic->state == IC_WAIT_FOR_MISS) ||
                                      (ic->next_state == IC_WAIT_FOR_MISS))) {
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

      ic->next_state = IC_FETCH;
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
    ic->next_state = IC_FETCH;
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
  // TEMP CHANGE: all FDIPPRF are UOC_PREF as well, except for a corner case where branch target is same line
  if (UOC_PREF && (mem_req_is_type(req, MRT_FDIPPRFON) || mem_req_is_type(req, MRT_FDIPPRFOFF) || mem_req_is_type(req, MRT_UOCPRF))) {
    if (in_uop_cache(req->addr, FALSE, req->off_path)) {
      STAT_EVENT(ic->proc_id, UOP_CACHE_HIT_NO_PREFETCH);
    }
  }

  return TRUE;
}

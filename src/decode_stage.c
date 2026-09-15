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
 * File         : decode_stage.c
 * Author       : HPS Research Group
 * Date         : 2/17/1999
 * Description  : simulates the latency due to decode stage. (actual uop decoding was
                    done in frontend)
 ***************************************************************************************/

#include "decode_stage.h"

#include "globals/assert.h"
#include "globals/debug_stage.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "debug/debug.param.h"
#include "debug/debug_macros.h"
#include "debug/debug_print.h"

#include "core.param.h"
#include "general.param.h"
#include "memory/memory.param.h"
#include "prefetcher/pref.param.h"

#include "bp/bp.h"
#include "isa/isa_macros.h"
#include "prefetcher/branch_misprediction_table.h"

#include "decoupled_frontend.h"
#include "ft.h"
#include "op_pool.h"
#include "statistics.h"
#include "thread.h" /* for td */
#include "uop_cache.h"

/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_DECODE_STAGE, ##args)
#define STAGE_MAX_OP_COUNT DECODE_WIDTH
#define STAGE_MAX_DEPTH (DECODE_CYCLES + ICACHE_LATENCY - 1)

/**************************************************************************************/
/* Global Variables */

Decode_Stage* dec = NULL;

/**************************************************************************************/
/* Local prototypes */

static inline void update_cycles_stats(Stage_Data* src_sd, int empty_stage_idx);

/**************************************************************************************/
/* set_decode_stage: */

void set_decode_stage(Decode_Stage* new_dec) {
  dec = new_dec;
}

/**************************************************************************************/
/* init_decode_stage: */

void init_decode_stage(uns8 proc_id, const char* name) {
  uns ii;
  ASSERT(0, dec);
  ASSERT(0, STAGE_MAX_DEPTH > 0);
  DEBUG(proc_id, "Initializing %s stage\n", name);

  memset(dec, 0, sizeof(Decode_Stage));
  dec->proc_id = proc_id;

  dec->sds = (Stage_Data*)malloc(sizeof(Stage_Data) * STAGE_MAX_DEPTH);
  for (ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &dec->sds[ii];
    /* Stage_Data::name is only used for debugging/printing; reuse the long-lived
     * stage name pointer passed into init_decode_stage().
     */
    cur->name = (char*)name;
    cur->max_op_count = STAGE_MAX_OP_COUNT;
    cur->ops = (Op**)calloc(STAGE_MAX_OP_COUNT, sizeof(Op*));
  }
  dec->last_sd = &dec->sds[0];
  dec->off_path = FALSE;
  reset_decode_stage();
}

/**************************************************************************************/
/* reset_decode_stage: */

void reset_decode_stage() {
  uns ii, jj;
  ASSERT(0, dec);
  for (ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &dec->sds[ii];
    cur->op_count = 0;
    for (jj = 0; jj < STAGE_MAX_OP_COUNT; jj++)
      cur->ops[jj] = NULL;
  }
}

/**************************************************************************************/
/* recover_decode_stage: */

void recover_decode_stage() {
  uns ii, jj;
  dec->off_path = FALSE;
  ASSERT(0, dec);
  for (ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Flag flushed = FALSE;
    Stage_Data* cur = &dec->sds[ii];
    cur->op_count = 0;
    for (jj = 0; jj < STAGE_MAX_OP_COUNT; jj++) {
      if (cur->ops[jj]) {
        if (IS_FLUSHING_OP(cur->ops[jj])) {
          op_select_bp_pred_info(cur->ops[jj], BP_PRED_MAIN);
          DEBUG(dec->proc_id, "Recovery op found in Decode stage:%u slot:%u op_num:%llu off_path:%u addr:0x%llx\n", ii,
                jj, (unsigned long long)cur->ops[jj]->op_num, cur->ops[jj]->off_path,
                (unsigned long long)cur->ops[jj]->inst->addr);
        }
        if (FLUSH_OP(cur->ops[jj])) {
          DEBUG(dec->proc_id, "Decode flushing op_num:%llu off_path:%u\n", (unsigned long long)cur->ops[jj]->op_num,
                cur->ops[jj]->off_path);
          flushed = TRUE;
          ASSERT(cur->ops[jj]->proc_id, cur->ops[jj]->off_path);
          if (cur->ops[jj]->parent_FT)
            ft_free_op(cur->ops[jj]);
          cur->ops[jj] = NULL;
        } else {
          cur->op_count++;
        }
      }
    }

    if (cur->op_count > 0 && flushed) {
      Op* op = cur->ops[cur->op_count - 1];
      assert_ft_after_recovery(dec->proc_id, op, bp_recovery_info->recovery_fetch_addr);
    }
  }
}

/**************************************************************************************/
/* debug_decode_stage: */

void debug_decode_stage() {
  uns ii;
  for (ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &dec->sds[STAGE_MAX_DEPTH - ii - 1];
    DPRINTF("# %-10s  op_count:%d\n", cur->name, cur->op_count);
    DPRINTF("# %-10s  op_nums:", cur->name);
    print_stage_op_nums(GLOBAL_DEBUG_STREAM, cur->ops, cur->op_count);
    DPRINTF("\n");
    print_op_array(GLOBAL_DEBUG_STREAM, cur->ops, STAGE_MAX_OP_COUNT, cur->op_count);
  }
}

/**************************************************************************************/
/* decode_cycle: Movement of cached uops to later stages assumes that with a stalled
 *               pipeline the same (modified src_sd is passed to update_decode_stage
 *               What if there is a branch mispred? are instr properly flushed?
 */

void update_decode_stage(Stage_Data* src_sd) {
  Flag stall = (dec->last_sd->op_count > 0);
  Stage_Data *cur, *prev;
  Op** temp;
  uns ii;

  if (!dec->off_path) {
    if (stall)
      STAT_EVENT(dec->proc_id, DECODE_STAGE_STALLED);
    else
      STAT_EVENT(dec->proc_id, DECODE_STAGE_NOT_STALLED);
    if (!src_sd->op_count)
      STAT_EVENT(dec->proc_id, DECODE_STAGE_STARVED);
    else
      STAT_EVENT(dec->proc_id, DECODE_STAGE_NOT_STARVED);
  } else
    STAT_EVENT(dec->proc_id, DECODE_STAGE_OFF_PATH);

  /* do all the intermediate stages */
  for (ii = 0; ii < STAGE_MAX_DEPTH - 1; ii++) {
    cur = &dec->sds[ii];
    if (cur->op_count)
      continue;
    prev = &dec->sds[ii + 1];
    temp = cur->ops;
    cur->ops = prev->ops;
    prev->ops = temp;
    cur->op_count = prev->op_count;
    prev->op_count = 0;
  }

  /* do the first decode stage */
  /* Ops from the uop cache do not go to the decode stage. */
  cur = &dec->sds[STAGE_MAX_DEPTH - 1];
  if (cur->op_count == 0 && src_sd->op_count) {
    for (int i = 0; i < src_sd->max_op_count; i++) {
      Op* src_op = src_sd->ops[i];
      if (src_op && src_op->off_path)
        dec->off_path = TRUE;
      if (src_op && !src_op->fetched_from_uop_cache) {
        cur->ops[cur->op_count] = src_op;
        src_sd->ops[i] = NULL;
        cur->op_count++;
        src_sd->op_count--;
      }
    }
    //   if (!stall)
    // ASSERT(0, !src_sd->op_count);
  }

  /* if the last decode stage is stalled, don't re-process the ops  */
  if (stall) {
    DEBUG(dec->proc_id, "Decode Stage stalled op_num:%s\n",
          (dec->last_sd->op_count && dec->last_sd->ops[0]) ? unsstr64(dec->last_sd->ops[0]->op_num) : "none");
    return;
  }

  /* now check the ops in the last decode stage for BTB errors */
  for (ii = 0; ii < dec->last_sd->op_count; ii++) {
    Op* op = dec->last_sd->ops[ii];
    ASSERT(dec->proc_id, op != NULL);
    ASSERT(dec->proc_id, !op->fetched_from_uop_cache);
    DEBUG(dec->proc_id, "Decoding op op_num=%llu, addr=%llx off_path:%i\n", op->op_num, op->inst->addr, op->off_path);
    decode_stage_process_op(op);
    uop_cache_insert_op(op);
  }
}

/**************************************************************************************/
/* process_decode_op: This function may also be called by ops from the uop cache.     */

void decode_stage_process_op(Op* op) {
  Cf_Type cf = op->uop->cf_type;
  op_set_decode_cycle(op, cycle_count);

  if (cf) {
    DEBUG(dec->proc_id, "Decode CF instruction bar:%i fetch_addr:%llx op_num:%llu recover:%i\n",
          op->uop->bar_type & BAR_FETCH ? TRUE : FALSE, op->inst->addr, op->op_num,
          op->bp_pred_info->recovery_point == RECOVER_AT_DECODE);
    // it is a direct branch, so the target is now known
    if (cf <= CF_CALL) {
      bp_target_known_op(g_bp_data, op);
    }
    // If the CF was unconditional and direct and taken and there was a BTB miss
    // we can schedule a redirect. If the branch was not taken we are on the on-path.
    // If the branch is condidtional or indirect, we will schedule recovery at exec
    if (op->bp_pred_info->recovery_point == RECOVER_AT_DECODE) {
      DEBUG(dec->proc_id, "Decode schedules recovery for op_num:%llu at cycle:%llu\n", (unsigned long long)op->op_num,
            (unsigned long long)cycle_count);
      bp_stat_main_branch_resolve_latency(op, cycle_count, FALSE);
      bp_sched_recovery(bp_recovery_info, op, cycle_count);

      op->btb_pred_info->btb_miss_resolved = TRUE;
      op->bp_pred_info->pred = op->oracle_info.dir;

      // stats for the reason of resteer
      STAT_EVENT(dec->proc_id, RESTEER_BTB_MISS_NOT_CF + cf);
    }

    if (FDIP_DUAL_PATH_PREF_UOC_ONLINE_ENABLE)
      increment_branch_count(op->inst->addr);
  }
}

// UNUSED, and not kept up to date with uop cache changes.
static inline void update_cycles_stats(Stage_Data* src_sd, int empty_stage_idx) {
  static Op* last_op = NULL;  // The most recent op that has entered the decode stage.

  int decode_stages_empty = STAGE_MAX_DEPTH - empty_stage_idx;
  // Only count stats if the icache is not stalled due to a full decode stage.
  if (decode_stages_empty && src_sd->op_count)
    last_op = src_sd->ops[src_sd->op_count - 1];
}

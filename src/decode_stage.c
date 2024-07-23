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

#include "debug/debug_macros.h"
#include "debug/debug_print.h"
#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"
#include "isa/isa_macros.h"

#include "bp/bp.h"
#include "decode_stage.h"
#include "op_pool.h"

#include "core.param.h"
#include "debug/debug.param.h"
#include "general.param.h"
#include "prefetcher/pref.param.h"
#include "thread.h" /* for td */

#include "uop_cache.h"
#include "prefetcher/branch_misprediction_table.h"
#include "statistics.h"
#include "memory/memory.param.h"
#include "decoupled_frontend.h"

/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_DECODE_STAGE, ##args)
#define STAGE_MAX_OP_COUNT ISSUE_WIDTH - DECODE_PATH_WIDTH_NARROWER
#define STAGE_MAX_DEPTH (DECODE_CYCLES + ICACHE_LATENCY - 1)


/**************************************************************************************/
/* Global Variables */

Decode_Stage* dec = NULL;
bool decode_off_path;

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
  char tmp_name[MAX_STR_LENGTH + 1];
  uns  ii;
  ASSERT(0, dec);
  ASSERT(0, STAGE_MAX_DEPTH > 0);
  DEBUG(proc_id, "Initializing %s stage\n", name);

  memset(dec, 0, sizeof(Decode_Stage));
  dec->proc_id = proc_id;

  dec->sds = (Stage_Data*)malloc(sizeof(Stage_Data) * STAGE_MAX_DEPTH);
  for(ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &dec->sds[ii];
    snprintf(tmp_name, MAX_STR_LENGTH, "%s %d", name, STAGE_MAX_DEPTH - ii - 1);
    cur->name         = (char*)strdup(tmp_name);
    cur->max_op_count = STAGE_MAX_OP_COUNT;
    cur->ops          = (Op**)calloc(STAGE_MAX_OP_COUNT, sizeof(Op*));
  }
  dec->last_sd = &dec->sds[0];
  reset_decode_stage();
}


/**************************************************************************************/
/* reset_decode_stage: */

void reset_decode_stage() {
  uns ii, jj;
  ASSERT(0, dec);
  for(ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &dec->sds[ii];
    cur->op_count   = 0;
    for(jj = 0; jj < STAGE_MAX_OP_COUNT; jj++)
      cur->ops[jj] = NULL;
  }
}


/**************************************************************************************/
/* recover_decode_stage: */

void recover_decode_stage() {
  uns ii, jj;
  decode_off_path = false;
  ASSERT(0, dec);
  for(ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &dec->sds[ii];
    cur->op_count   = 0;
    for(jj = 0; jj < STAGE_MAX_OP_COUNT; jj++) {
      if(cur->ops[jj]) {
        if(FLUSH_OP(cur->ops[jj])) {
          ASSERT(cur->ops[jj]->proc_id, cur->ops[jj]->off_path);
          free_op(cur->ops[jj]);
          cur->ops[jj] = NULL;
        } else {
          cur->op_count++;
        }
      }
    }
  }
}


/**************************************************************************************/
/* debug_decode_stage: */

void debug_decode_stage() {
  uns ii;
  for(ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &dec->sds[STAGE_MAX_DEPTH - ii - 1];
    DPRINTF("# %-10s  op_count:%d\n", cur->name, cur->op_count);
    print_op_array(GLOBAL_DEBUG_STREAM, cur->ops, STAGE_MAX_OP_COUNT,
                   cur->op_count);
  }
}


/**************************************************************************************/
/* decode_cycle: Movement of cached uops to later stages assumes that with a stalled
 *               pipeline the same (modified src_sd is passed to update_decode_stage
 *               What if there is a branch mispred? are instr properly flushed?
 */

void update_decode_stage(Stage_Data* src_sd) {
  Flag        stall = (dec->last_sd->op_count > 0);
  Stage_Data *cur, *prev;
  Op**        temp;
  uns         ii;

  if(!decode_off_path) {
    if(stall)
      STAT_EVENT(dec->proc_id, DECODE_STAGE_STALLED);
    else
      STAT_EVENT(dec->proc_id, DECODE_STAGE_NOT_STALLED);
    if(!src_sd->op_count)
      STAT_EVENT(dec->proc_id, DECODE_STAGE_STARVED);
    else
      STAT_EVENT(dec->proc_id, DECODE_STAGE_NOT_STARVED);
  }
  else
    STAT_EVENT(dec->proc_id, DECODE_STAGE_OFF_PATH);


  /* do all the intermediate stages */
  for(ii = 0; ii < STAGE_MAX_DEPTH - 1; ii++) {
    cur = &dec->sds[ii];
    if(cur->op_count)
      continue;
    prev           = &dec->sds[ii + 1];
    temp           = cur->ops;
    cur->ops       = prev->ops;
    prev->ops      = temp;
    cur->op_count  = prev->op_count;
    prev->op_count = 0;
  }

  /* do the first decode stage */
  /* Ops from the uop cache do not go to the decode stage. */
  cur = &dec->sds[STAGE_MAX_DEPTH - 1];
  if (cur->op_count == 0 && src_sd->op_count) {
    for (int i = 0; i < src_sd->max_op_count; i++) {
      Op* src_op = src_sd->ops[i];
      if (src_op && src_op->off_path)
        decode_off_path = true;
      if (src_op && !src_op->fetched_from_uop_cache) {
        cur->ops[cur->op_count] = src_op;
        src_sd->ops[i] = NULL;
        cur->op_count++;
        src_sd->op_count--;
      }
    }
    //   if (!stall)
    //ASSERT(0, !src_sd->op_count);
  }

  /* if the last decode stage is stalled, don't re-process the ops  */
  if(stall) {
    DEBUG(dec->proc_id, "Decode Stage stalled\n");
    return;
  }

  /* now check the ops in the last decode stage for BTB errors */
  for(ii = 0; ii < dec->last_sd->op_count; ii++) {
    Op* op = dec->last_sd->ops[ii];
    ASSERT(dec->proc_id, op != NULL);
    ASSERT(dec->proc_id, !op->fetched_from_uop_cache);
    DEBUG(dec->proc_id, "Decoding op op_num=%llu, addr=%llx\n", op->op_num, op->inst_info->addr);
    decode_stage_process_op(op);
    accumulate_op(op);
  }
}


/**************************************************************************************/
/* process_decode_op: This function may also be called by ops from the uop cache.     */

void decode_stage_process_op(Op* op) {
  Cf_Type cf = op->table_info->cf_type;
  op->decode_cycle = cycle_count;

  if(cf) {
    DEBUG(dec->proc_id, "Decode CF instruction bar:%i fetch_addr:%llx op_num:%llu recover:%i\n",
          op->table_info->bar_type & BAR_FETCH ? TRUE : FALSE, op->inst_info->addr, op->op_num,
          op->oracle_info.recover_at_decode);
    // it is a direct branch, so the target is now known
    if (cf <= CF_CALL) {
      bp_target_known_op(g_bp_data, op);
    }
    // If the CF was unconditional and direct and taken and there was a BTB miss
    // we can schedule a redirect. If the branch was not taken we are on the on-path.
    // If the branch is condidtional or indirect, we will schedule recovery at exec
    if (op->oracle_info.recover_at_decode) {
      bp_sched_recovery(bp_recovery_info, op, cycle_count,
                        /*late_bp_recovery=*/FALSE, /*force_offpath=*/FALSE);

      // After recovery remove misfetch/mispred/btb_miss flags so it does not trigger flush by exec again
      op->oracle_info.misfetch = FALSE;
      op->oracle_info.btb_miss = FALSE;
      op->oracle_info.pred = op->oracle_info.dir;
      op->oracle_info.mispred = FALSE;

      // stats for the reason of resteer
      STAT_EVENT(dec->proc_id, RESTEER_BTB_MISS_CF_BR + cf);
    }

    if (FDIP_DUAL_PATH_PREF_UOC_ONLINE_ENABLE)
      increment_branch_count(op->inst_info->addr);
  }
}

// UNUSED, and not kept up to date with uop cache changes.
static inline void update_cycles_stats(Stage_Data* src_sd, int empty_stage_idx) {
  static Op*  last_op = NULL;  // The most recent op that has entered the decode stage.

  int decode_stages_empty = STAGE_MAX_DEPTH - empty_stage_idx;
  // Only count stats if the icache is not stalled due to a full decode stage.
  if (decode_stages_empty && src_sd->op_count)
    last_op = src_sd->ops[src_sd->op_count - 1];
}

int get_decode_stages_filled(void) {
  int full_stages = 0;
  for(int ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &dec->sds[ii];
    if (cur->op_count) {
      full_stages++;
    }
  }
  return full_stages;
}

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
 * File         : map_stage.c
 * Author       : HPS Research Group
 * Date         : 2/4/1999
 * Description  :
 ***************************************************************************************/

#include "debug/debug_macros.h"
#include "debug/debug_print.h"
#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"
#include "memory/memory.param.h"
#include "op_pool.h"

#include "bp/bp.h"
#include "map.h"
#include "map_stage.h"
#include "model.h"
#include "thread.h"
#include "uop_cache.h"
#include "decode_stage.h"

#include "core.param.h"
#include "debug/debug.param.h"
#include "statistics.h"

/**************************************************************************************/
/* Macros */
#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_MAP_STAGE, ##args)
#define STAGE_MAX_OP_COUNT ISSUE_WIDTH + UOP_CACHE_ADDITIONAL_ISSUE_BANDWIDTH
#define STAGE_MAX_DEPTH MAP_CYCLES


/**************************************************************************************/
/* Global Variables */

Map_Stage* map = NULL;
// The next op number is used when deciding whether to consume ops from the
// uop cache: i.e. check if any preceding instructions are still in the decoder.
Counter map_stage_next_op_num = 1;
int map_off_path = 0;

/**************************************************************************************/
/* Local prototypes */

static inline void stage_process_op(Op*);
static inline Flag map_fetch_fill_op(Stage_Data* src_sd, int* fetch_idx, Flag src_is_icache);
static inline void shift_ops_to_arr_start(Stage_Data* sd);

/**************************************************************************************/
/* set_map_stage: */

void set_map_stage(Map_Stage* new_map) {
  map = new_map;
}


/**************************************************************************************/
/* init_map_stage: */

void init_map_stage(uns8 proc_id, const char* name) {
  char tmp_name[MAX_STR_LENGTH + 1];
  uns  ii;
  ASSERT(proc_id, map);
  ASSERT(proc_id, STAGE_MAX_DEPTH > 0);
  DEBUG(proc_id, "Initializing %s stage\n", name);

  memset(map, 0, sizeof(Map_Stage));
  map->proc_id = proc_id;

  map->sds = (Stage_Data*)malloc(sizeof(Stage_Data) * STAGE_MAX_DEPTH);
  for(ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &map->sds[ii];
    snprintf(tmp_name, MAX_STR_LENGTH, "%s %d", name, STAGE_MAX_DEPTH - ii - 1);
    cur->name         = (char*)strdup(tmp_name);
    cur->max_op_count = STAGE_MAX_OP_COUNT;
    cur->ops          = (Op**)malloc(sizeof(Op*) * STAGE_MAX_OP_COUNT);
  }
  map->last_sd = &map->sds[0];
  reset_map_stage();
}


/**************************************************************************************/
/* reset_map_stage: */

void reset_map_stage() {
  uns ii, jj;
  ASSERT(0, map);
  for(ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &map->sds[ii];
    cur->op_count   = 0;
    for(jj = 0; jj < STAGE_MAX_OP_COUNT; jj++)
      cur->ops[jj] = NULL;
  }
}


/**************************************************************************************/
/* recover_map_stage: */

void recover_map_stage() {
  uns ii, jj, kk;
  map_off_path = 0;
  ASSERT(0, map);
  for(ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &map->sds[ii];
    cur->op_count   = 0;

    for(jj = 0, kk = 0; jj < STAGE_MAX_OP_COUNT; jj++) {
      if(cur->ops[jj]) {
        if(FLUSH_OP(cur->ops[jj])) {
          free_op(cur->ops[jj]);
          cur->ops[jj] = NULL;
        } else {
          Op* op = cur->ops[jj];
          cur->op_count++;
          cur->ops[jj]   = NULL;  // collapse the ops
          cur->ops[kk++] = op;
        }
      }
    }
  }

  if (map_stage_next_op_num > bp_recovery_info->recovery_op_num) {
    map_stage_next_op_num = bp_recovery_info->recovery_op_num + 1;
    DEBUG(map->proc_id, "Recovering map_stage_next_op_num to %llu\n", map_stage_next_op_num);
  }
}


/**************************************************************************************/
/* debug_map_stage: */

void debug_map_stage() {
  uns ii;
  for(ii = 0; ii < STAGE_MAX_DEPTH; ii++) {
    Stage_Data* cur = &map->sds[STAGE_MAX_DEPTH - ii - 1];
    DPRINTF("# %-10s  op_count:%d\n", cur->name, cur->op_count);
    print_op_array(GLOBAL_DEBUG_STREAM, cur->ops, STAGE_MAX_OP_COUNT,
                   STAGE_MAX_OP_COUNT);
  }
}


/**************************************************************************************/
/* map_cycle: */

// Consume just from one: Select the stage to consume from
// Else if UOC_IC_SWITCH_FRAG_DISABLE, also consume from the second one.
void update_map_stage(Stage_Data* dec_src_sd, Stage_Data* uopq_src_sd, Flag uopq_src_is_icache) {
  Flag        stall = (map->last_sd->op_count > 0);
  Stage_Data* consume_from_sd = NULL;
  Stage_Data* other_sd = NULL;
  Flag consume_from_is_icache = FALSE;
  Flag other_is_icache = FALSE;
  Flag fetch_from_both_srcs = FALSE;
  Stage_Data *cur, *prev;
  Op**        temp;
  uns         ii;

  /* do all the intermediate stages */
  for(ii = 0; ii < STAGE_MAX_DEPTH - 1; ii++) {
    cur = &map->sds[ii];
    if(cur->op_count)
      continue;
    prev           = &map->sds[ii + 1];
    temp           = cur->ops;
    cur->ops       = prev->ops;
    prev->ops      = temp;
    cur->op_count  = prev->op_count;
    prev->op_count = 0;
  }

  /* do the first map stage */
  // Uops can be received from either the decoder or directly from the uop cache
  // via the uop queue.
  // Only consume if older ops have already been consumed by this stage.
  cur = &map->sds[STAGE_MAX_DEPTH - 1];
  if (UOP_CACHE_ENABLE) {
    // When the uop cache is enabled, the next op to be consumed by the map stage
    // is from either the decode stage or the uop cache source.
    // The uop cache source is either the uop queue or the icache stage bypassing the uop queue.
    // The map stage may consume multiple ops in one cycle from both
    // the map stage and the uop cache source if allowed.
    ASSERT(map->proc_id, uopq_src_sd != NULL);
    if (dec_src_sd->op_count && dec_src_sd->ops[0]->op_num == map_stage_next_op_num) {
      consume_from_sd = dec_src_sd;
      consume_from_is_icache = FALSE;
      other_sd = uopq_src_sd;  //can only consume ALL ops from this stage if the other sd has them ready. Otherwise only the first few
      if (uopq_src_is_icache) {
        other_is_icache = TRUE;
      } else {
        other_is_icache = FALSE;
      }
    } else if (uopq_src_sd->op_count && uopq_src_sd->ops[0]->op_num == map_stage_next_op_num) {
      consume_from_sd = uopq_src_sd;
      if (uopq_src_is_icache) {
        consume_from_is_icache = TRUE;
      } else {
        consume_from_is_icache = FALSE;
      }
      other_sd = dec_src_sd;
      other_is_icache = FALSE;
    }
    fetch_from_both_srcs = UOC_IC_SWITCH_FRAG_DISABLE && cur->max_op_count >= consume_from_sd->op_count + other_sd->op_count;
  } else {
    // When the uop cache is disabled, the next op to be consumed by the map stage
    // is from the decode stage.
    ASSERT(map->proc_id, uopq_src_sd == NULL);
    if (dec_src_sd->op_count) {
      ASSERT(map->proc_id, dec_src_sd->ops[0]->op_num == map_stage_next_op_num);
      consume_from_sd = dec_src_sd;
    }
  }

  if(!map_off_path) {
    if(stall)
      STAT_EVENT(map->proc_id, MAP_STAGE_STALLED);
    else
      STAT_EVENT(map->proc_id, MAP_STAGE_NOT_STALLED);
    if(consume_from_sd == NULL)
      STAT_EVENT(map->proc_id, MAP_STAGE_STARVED);
    else
      STAT_EVENT(map->proc_id, MAP_STAGE_NOT_STARVED);
  }
  else
      STAT_EVENT(map->proc_id, MAP_STAGE_OFF_PATH);

  // Consume interleaved from both srcs if UOC_IC_SWITCH_FRAG_DISABLE.
  // Else, only consume from one src
  if (cur->op_count == 0 && consume_from_sd) {
    int cfsd_ii = 0;  // consume_from_sd idx
    int osd_ii  = 0;  // other_sd idx
    Flag fetched_cfsd = FALSE;
    Flag fetched_osd = FALSE;
    do {
      fetched_cfsd = map_fetch_fill_op(consume_from_sd, &cfsd_ii, consume_from_is_icache);
      if (fetch_from_both_srcs) {
        fetched_osd = map_fetch_fill_op(other_sd, &osd_ii, other_is_icache);
      }
    } while (fetched_cfsd || fetched_osd);

    for(int ii = 0; ii < cur->op_count; ii++) {
      Op* op = cur->ops[ii];
      if (op && op->off_path)
        map_off_path = 1;
    }
    // Probably should count number of on-path ops. 
    // Any stage can receive a mix of on/off-path ops in a single cycle.
    if (!map_off_path)
      STAT_EVENT(map->proc_id, MAP_STAGE_RECEIVED_OPS_0 + cur->op_count);
    ASSERT(map->proc_id, cur->op_count <= MAP_STAGE_RECEIVED_OPS_MAX);
  }
  // if op array not empty, shift ops to array start
  shift_ops_to_arr_start(consume_from_sd);
  shift_ops_to_arr_start(other_sd);

  /* if the last map stage is stalled, don't re-process the ops  */
  if(stall) {
    return;
  }
  /* now map the ops in the last map stage */
  for(ii = 0; ii < map->last_sd->op_count; ii++) {
    Op* op = map->last_sd->ops[ii];
    ASSERT(map->proc_id, op != NULL);
    stage_process_op(op);
  }

}


/**************************************************************************************/
/* map_process_op: */

static inline void stage_process_op(Op* op) {
  /* the map stage is currently responsible only for setting wake up lists */
  add_to_wake_up_lists(op, &op->oracle_info, model->wake_hook);
}


/**************************************************************************************/
/* map_fetch_fill_op: Fill the map stage with op from src at fetch_idx */

static inline Flag map_fetch_fill_op(Stage_Data* src_sd, int* fetch_idx, Flag src_is_icache) {
  Stage_Data* dest_sd = &map->sds[STAGE_MAX_DEPTH - 1];
  if (*fetch_idx == src_sd->max_op_count)
    return FALSE;

  Op* op = src_sd->ops[*fetch_idx];

  // The only case where icache sd can be consumed by the map stage is
  // when the instruction is fetched from uop and needs to bypass the uop queue.
  // So if the instruction is not fetched from uop, it cannot be used at this moment.

  // More detailed explanations:
  // Q: If `src_is_icache` is true, how can the op be `fetched_from_uop_cache`?
  // A: In the first cycle of the icache stage, we look up both icache and the uop cache.
  // No matter where the ops come from (icache or uop cache), they are added to the icache stage data,
  // and the way to distinguish them is to look at `fetched_from_uop_cache` of each op.
  // If in the first icache cycle the uop queue is NOT empty,
  // the ops fetched from the uop cache will be added to the uop queue in the next cycle;
  // otherwise, if the uop queue is empty,
  // the ops fetched from the uop cache can bypass the uop queue and go directly to the map stage in the next cycle.
  // This API is supposed to consume ops from the icache stage only in the latter case,
  // and the following if statements make it sure.
  // Those ops that fail the second following if-conditions are fetched from the icache and still need to be decoded,
  // so they are not ready yet.

  // Q: What if `UOC_IC_SWITCH_FRAG_DISABLE` is on?
  // A: When `UOC_IC_SWITCH_FRAG_DISABLE` is on,
  // we are allowed to fetch from both **decode stage** and **the uop cache source**,
  // where the **the uop cache source** is either the *uop queue* or the *icache stage data* when the uop queue is empty.
  // Still, when the **the uop cache source** is the *icache stage data*,
  // the ops that can be consumed are required to be fetched from the uop cache.
  // `UOC_IC_SWITCH_FRAG_DISABLE` being on does not change this fact.

  if (src_is_icache) {
    if (op && !op->fetched_from_uop_cache) {
      return FALSE;
    }
  }

  if (op && op->op_num == map_stage_next_op_num) {
    DEBUG(map->proc_id, "Fetching opnum=%llu from %s at idx=%i\n", op->op_num, src_sd->name, *fetch_idx);
    if (!op->decode_cycle) decode_stage_process_op(op);
    op->map_cycle = cycle_count;
    dest_sd->ops[dest_sd->op_count++] = op;
    src_sd->ops[*fetch_idx] = NULL;
    src_sd->op_count--;
    map_stage_next_op_num++;
    *fetch_idx = *fetch_idx + 1;
    return TRUE;
  }
  return FALSE;
}


/**************************************************************************************/
/* shift_ops_to_arr_start:  */

static inline void shift_ops_to_arr_start(Stage_Data* sd) {
  int insert_idx = 0;
  if (!sd || sd->op_count == 0)
    return;

  for (int ii = 0; ii < sd->max_op_count; ii++) {
    if (sd->ops[ii]) {
      sd->ops[insert_idx] = sd->ops[ii];
      if (ii != insert_idx)
        sd->ops[ii] = NULL;
      insert_idx++;
    }
  }
}

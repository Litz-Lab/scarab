/*
 * Copyright 2020 HPS/SAFARI Research Groups
 * Copyright 2025 Litz Lab
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
 * File         : dcache_stage.c
 * Author       : HPS Research Group, Litz Lab
 * Date         : 3/8/1999, 4/15/2025
 * Description  :
 ***************************************************************************************/

#include "dcache_stage.h"

#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "debug/debug.param.h"
#include "debug/debug_macros.h"
#include "debug/debug_print.h"

#include "core.param.h"
#include "memory/memory.param.h"
#include "prefetcher//stream.param.h"
#include "prefetcher/pref.param.h"

#include "bp/bp.h"
#include "prefetcher/l2l1pref.h"
#include "prefetcher/pref_common.h"
#include "prefetcher/stream_pref.h"

#include "cmp_model.h"
#include "map.h"
#include "model.h"
#include "statistics.h"
#include "thread.h"

/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_DCACHE_STAGE, ##args)
#define STAGE_MAX_OP_COUNT NUM_FUS

/**************************************************************************************/
/* Global Variables */

Dcache_Stage* dc = NULL;

/**************************************************************************************/
/* Prototypes for Inline Methods */

static inline Flag dcache_stage_addr_unready(Op* op);
static inline Flag dcache_stage_check_mem_type(Op* op);
static inline void dcache_stage_remove_src_op(Stage_Data* src_sd, int ii);

static inline void dcache_cacheline_hit(Op* op, Addr line_addr, Dcache_Data* line);
static inline void dcache_cacheline_miss(Op* op, Addr line_addr);

static inline void dcache_fill_wp_collect_stats(Dcache_Data* line, Mem_Req* req);
static inline void dcache_hit_wp_collect_stats(Dcache_Data* line, Op* op);
/* Load latency actually observed by the op: from the cycle its address was ready
   (earliest it could be submitted to the dcache) to the cycle its data is ready.
   Spans port/bank-conflict waits, OS_WAIT_DCACHE re-probes, replays and
   extra_ld_latency, so it is not bounded below by DCACHE_CYCLES. */
static inline void ld_served_stat(Op* op, Stat_Enum served, Stat_Enum lat) {
  Counter start = op_get_exec_cycle(op);
  Counter end = op_get_done_cycle(op);
  // Both are always set here: a replaying op with an unset exec_cycle is squished
  // out of the stage before this point, and both call sites run right after
  // done_cycle is stamped. end >= start because the addr-ready gate only lets the
  // op through once cycle_count >= exec_cycle. Skipping instead of asserting would
  // silently drop loads and break the LD_SERVED_* partition.
  ASSERT(op->proc_id, start != MAX_CTR);
  ASSERT(op->proc_id, end != MAX_CTR);
  ASSERT(op->proc_id, end >= start);
  STAT_EVENT(op->proc_id, served);
  INC_STAT_EVENT(op->proc_id, lat, end - start);
}

static inline Flag dcache_miss_new_mem_req(Op* op, Addr line_addr, Mem_Req_Type mem_req_type);
static inline void dcache_miss_extra_access(Op* op, Cache* cache, Addr line_addr, uns8 proc_id, uns8 cache_cycle);

static inline Dcache_Data* dcache_fill_get_cacheline(Mem_Req* req);
static inline void dcache_fill_process_cacheline(Mem_Req* req, Dcache_Data* data);

/**************************************************************************************/
/* External Interfaces for CMP Model */

void set_dcache_stage(Dcache_Stage* new_dc) {
  dc = new_dc;
}

void init_dcache_stage(uns8 proc_id, const char* name) {
  DEBUG(proc_id, "Initializing %s stage\n", name);

  ASSERT(0, dc);
  memset(dc, 0, sizeof(Dcache_Stage));

  dc->proc_id = proc_id;
  /* `name` is expected to be long-lived (caller passes string literals). */
  dc->sd.name = (char*)name;
  dc->sd.max_op_count = STAGE_MAX_OP_COUNT;
  dc->sd.ops = (Op**)malloc(sizeof(Op*) * STAGE_MAX_OP_COUNT);

  /* initialize the cache structure */
  init_cache(&dc->dcache, "DCACHE", DCACHE_SIZE, DCACHE_ASSOC, DCACHE_LINE_SIZE, sizeof(Dcache_Data), DCACHE_REPL);
  reset_dcache_stage();

  dc->ports = (Ports*)malloc(sizeof(Ports) * DCACHE_BANKS);
  for (uns ii = 0; ii < DCACHE_BANKS; ii++) {
    char name[MAX_STR_LENGTH + 1];
    snprintf(name, MAX_STR_LENGTH, "DCACHE BANK %d PORTS", ii);
    init_ports(&dc->ports[ii], name, DCACHE_READ_PORTS, DCACHE_WRITE_PORTS, FALSE);
  }

  dc->dcache.repl_pref_thresh = DCACHE_REPL_PREF_THRESH;

  if (DC_PREF_CACHE_ENABLE)
    init_cache(&dc->pref_dcache, "DC_PREF_CACHE", DC_PREF_CACHE_SIZE, DC_PREF_CACHE_ASSOC, DCACHE_LINE_SIZE,
               sizeof(Dcache_Data), DCACHE_REPL);

  memset(dc->rand_wb_state, 0, NUM_ELEMENTS(dc->rand_wb_state));
}

void reset_dcache_stage(void) {
  uns ii;
  for (ii = 0; ii < STAGE_MAX_OP_COUNT; ii++)
    dc->sd.ops[ii] = NULL;
  dc->sd.op_count = 0;
  dc->idle_cycle = 0;
}

void recover_dcache_stage() {
  uns ii;
  for (ii = 0; ii < NUM_FUS; ii++) {
    Op* op = dc->sd.ops[ii];
    if (op && IS_FLUSHING_OP(op)) {
      op_select_bp_pred_info(op, BP_PRED_MAIN);
      DEBUG(dc->proc_id, "Recovery op found in Dcache slot:%u op_num:%llu off_path:%u addr:0x%llx\n", ii,
            (unsigned long long)op->op_num, op->off_path, (unsigned long long)op->inst->addr);
    }
    if (op && op->op_num > bp_recovery_info->recovery_op_num) {
      DEBUG(dc->proc_id, "Dcache flushing op_num:%llu off_path:%u\n", (unsigned long long)op->op_num, op->off_path);
      ASSERT(dc->proc_id, op->off_path);
      dc->sd.ops[ii] = NULL;
      dc->sd.op_count--;
    }
  }
  dc->idle_cycle = cycle_count + 1;
}

void debug_dcache_stage() {
  DPRINTF("# %-10s  op_count:%d  busy: %d\n", dc->sd.name, dc->sd.op_count, dc->idle_cycle > cycle_count);
  print_op_array(GLOBAL_DEBUG_STREAM, dc->sd.ops, STAGE_MAX_OP_COUNT, STAGE_MAX_OP_COUNT);
}

void update_dcache_stage(Stage_Data* src_sd) {
  /* phase 1 - move ops into the dcache stage */
  ASSERT(dc->proc_id, src_sd->max_op_count == dc->sd.max_op_count);
  for (uns ii = 0; ii < src_sd->max_op_count; ii++) {
    Op* op = src_sd->ops[ii];
    Op* dc_op = dc->sd.ops[ii];

    // op just got told to replay this cycle (clobber it)
    if (op && cycle_count < op_get_rdy_cycle(op)) {
      ASSERTM(dc->proc_id, op->replay, "o:%s  rdy:%s", unsstr64(op->op_num), unsstr64(op_get_rdy_cycle(op)));
      dcache_stage_remove_src_op(src_sd, ii);
      op = NULL;
    }

    /* check if the op in the dcache_stage is stall */
    if (dc_op) {
      if (dc_op->state == OS_WAIT_DCACHE || (STALL_ON_WAIT_MEM && dc_op->state == OS_WAIT_MEM)) {
        ASSERT(dc->proc_id, cycle_count >= op_get_exec_cycle(dc->sd.ops[ii]));
        continue;
      }

      // unless the op stalled getting a dcache port, it's gone
      dc->sd.ops[ii] = NULL;
      dc->sd.op_count--;
      ASSERT(dc->proc_id, dc->sd.op_count >= 0);
    }

    /* check if the op from the src_stage is ready */
    if (!op)
      continue;

    // not ready due to address generation latency
    if (dcache_stage_addr_unready(op)) {
      continue;
    }

    // squash non-memory and prefetch (when software prefetching is not enabled) ops
    if (!dcache_stage_check_mem_type(op)) {
      dcache_stage_remove_src_op(src_sd, ii);
      continue;
    }

    /* if the op is valid, move it into the dcache stage */
    dc->sd.ops[ii] = op;
    dc->sd.op_count++;
    ASSERT(dc->proc_id, dc->sd.op_count <= dc->sd.max_op_count);
    dcache_stage_remove_src_op(src_sd, ii);
    ASSERTM(dc->proc_id, cycle_count >= op_get_exec_cycle(op), "o:%s  %s\n", unsstr64(op->op_num),
            Op_State_str(op->state));
  }

  /* phase 2 - check the dcache port availability and do dcache access */
  int start_op_count = dc->sd.op_count;
  Counter last_oldest_op_num = 0;
  for (uns ii = 0; ii < start_op_count; ii++) {
    /* update in program order (make things easier) */
    uns oldest_index = 0;
    Counter oldest_op_num = MAX_CTR;
    // TODO: adjust this O(n2) algorithm by getting the program order outside first
    for (uns jj = 0; jj < dc->sd.max_op_count; jj++) {
      if (dc->sd.ops[jj] && dc->sd.ops[jj]->op_num > last_oldest_op_num && dc->sd.ops[jj]->op_num < oldest_op_num) {
        oldest_op_num = dc->sd.ops[jj]->op_num;
        oldest_index = jj;
      }
    }
    last_oldest_op_num = oldest_op_num;

    ASSERT(dc->proc_id, oldest_op_num < MAX_CTR);
    Op* op = dc->sd.ops[oldest_index];

    // if the op is replaying, squish it
    if (op->replay && op_get_exec_cycle(op) == MAX_CTR) {
      dc->sd.ops[oldest_index] = NULL;
      dc->sd.op_count--;
      ASSERT(dc->proc_id, dc->sd.op_count >= 0);
      continue;
    }

    // OS_WAIT_MEM re-probe: the op already accessed the dcache and missed, and
    // new_mem_req found no in-flight request to merge with and no free mem-request
    // buffer. Re-looking-up the dcache can only re-derive the same known miss, so skip
    // the port acquisition and the tag lookup entirely and go straight to retrying the
    // mem request. A matching in-flight request that shows up later is still caught by
    // new_mem_req's queue search inside the miss handler. Set OS_SCHEDULED first (as the
    // fresh-access path below does) so the store-forward-hit case, which does not set
    // op->state, leaves the op removable instead of resident.
    if (op->state == OS_WAIT_MEM) {
      ASSERT(dc->proc_id, op_get_dcache_cycle(op) != MAX_CTR);
      op->state = OS_SCHEDULED;
      dcache_cacheline_miss(op, get_cache_line_addr(&dc->dcache, op->oracle_info.va));
      continue;
    }

    /* check on the availability of a read port for the given bank */
    // the bank bits are the lowest order cache index bits
    uns bank = BANK(op->oracle_info.va, DCACHE_BANKS, DCACHE_INTERLEAVE_FACTOR);
    DEBUG(dc->proc_id, "check_read and write port availiabilty mem_type:%s bank:%d \n",
          (op->uop->mem_type == MEM_ST) ? "ST" : "LD", bank);
    if (!PERFECT_DCACHE && ((op->uop->mem_type == MEM_ST && !get_write_port(&dc->ports[bank])) ||
                            (op->uop->mem_type != MEM_ST && !get_read_port(&dc->ports[bank])))) {
      op->state = OS_WAIT_DCACHE;
      STAT_EVENT(dc->proc_id, DCACHE_READ_PORT_UNAVAILABLE_ONPATH + op->off_path);
      continue;
    }
    // Record the op's first dcache access (dcache_cycle gates precommit). Every op
    // reaching here is making that first access: fresh from the scheduler, or an
    // OS_WAIT_DCACHE op that was only ever waiting for a cache port and never accessed.
    // OS_WAIT_MEM re-probes are short-circuited above, so a stamped op can no longer
    // re-enter this point (the sole path was OS_WAIT_MEM -> busy port -> OS_WAIT_DCACHE);
    // op_set_dcache_cycle is write-once and asserts dcache_cycle was unset.
    op_set_dcache_cycle(op, cycle_count);

    // memory ops are marked as scheduled so that they can be removed from the node->rdy_list
    op->state = OS_SCHEDULED;

    // ideal l2 l1 prefetcher bring l1 data immediately
    if (IDEAL_L2_L1_PREFETCHER)
      ideal_l2l1_prefetcher(op);

    /* now access the dcache with it */
    Addr line_addr;
    Dcache_Data* line = (Dcache_Data*)cache_access(&dc->dcache, op->oracle_info.va, &line_addr, TRUE);
    dc->idle_cycle = MAX2(dc->idle_cycle, cycle_count + DCACHE_CYCLES);

    if (op->uop->mem_type == MEM_ST)
      STAT_EVENT(op->proc_id, POWER_DCACHE_WRITE_ACCESS);
    else
      STAT_EVENT(op->proc_id, POWER_DCACHE_READ_ACCESS);

    // if the data hits dc_pref_cache then insert to the dcache immediately
    if (DC_PREF_CACHE_ENABLE && !line) {
      line = dc_pref_cache_access(op);
    }

    op->oracle_info.dcmiss = FALSE;
    if (PERFECT_DCACHE) {
      if (!op->off_path) {
        STAT_EVENT(op->proc_id, DCACHE_HIT);
        STAT_EVENT(op->proc_id, DCACHE_HIT_ONPATH);
      } else {
        STAT_EVENT(op->proc_id, DCACHE_HIT_OFFPATH);
      }

      op_set_done_cycle(op, cycle_count + DCACHE_CYCLES + op->uop->extra_ld_latency);
      if (op->uop->mem_type != MEM_ST) {
        op_set_wake_cycle(op, op_get_done_cycle(op));
        wake_up_ops(op, REG_DATA_DEP, model->wake_hook);
      }
      continue;
    }

    if (line) {
      dcache_cacheline_hit(op, line_addr, line);
      if (!op->off_path && op->uop->mem_type == MEM_LD)
        ld_served_stat(op, LD_SERVED_DCACHE, LD_LAT_DCACHE);
      continue;
    }
    dcache_cacheline_miss(op, line_addr);
  }

  /* prefetcher update */
  if (STREAM_PREFETCH_ON)
    update_pref_queue();
  if (L2WAY_PREF && !L1PREF_IMMEDIATE)
    update_l2way_pref_req_queue();
  if (L2MARKV_PREF_ON && !L1MARKV_PREF_IMMEDIATE)
    update_l2markv_pref_req_queue();
}

/**************************************************************************************/
/* External API for architectural cache */

Flag dcache_fill_line(Mem_Req* req) {
  set_dcache_stage(&cmp_model.dcache_stage[req->proc_id]);
  set_thread_data(&cmp_model.thread_data[req->proc_id]);
  set_map_data(&td->map_data);
  Counter old_cycle_count = cycle_count;  // FIXME HACK!
  cycle_count = freq_cycle_count(FREQ_DOMAIN_CORES[req->proc_id]);

  ASSERT(dc->proc_id, dc->proc_id == req->proc_id);
  ASSERT(dc->proc_id, req->op_count == req->op_ptrs.count);
  ASSERT(dc->proc_id, req->op_count == req->op_uniques.count);

  /* if it can't get a write port, fail */
  uns bank = BANK(req->addr, DCACHE_BANKS, DCACHE_INTERLEAVE_FACTOR);
  if (!get_write_port(&dc->ports[bank])) {
    cycle_count = old_cycle_count;
    STAT_EVENT(dc->proc_id, DCACHE_FILL_PORT_UNAVAILABLE_ONPATH + req->off_path);
    return FAILURE;
  }

  /* get new line in the cache */
  Dcache_Data* data = dcache_fill_get_cacheline(req);
  if (data == NULL) {  // if the line we need to replace is dirty
    /*
     * This is a hack to get around a deadlock issue.
     * It doesn't completely eliminate the deadlock, but makes it less likely...
     *
     * The deadlock occurs when all the mem_req buffers are used,
     * and all pending mem_reqs need to fill the dcache,
     * but the highest priority dcache fill ends up evicting a dirty line from the dcache,
     * which then needs to be written back to L1/MLC.
     *
     * This dcache fill will aquire a write port via get_write_port(), but then fail here,
     * because there are no more mem_req buffers available for dc wb req,
     * and new_mem_dc_wb_req() will return FALSE.
     *
     * If we don't release the write port, then all other mem_reqs,
     * which still need to fill the dcache, will fail, and we end up in a deadlock.
     * So instead, we release the write port below.
     *
     * HOWEVER, a deadlock is still possible if all pending mem_reqs fill the dcache
     * and all end up evicting a dirty line
     */

    ASSERT(dc->proc_id, 0 < dc->ports[bank].write_ports_in_use);
    dc->ports[bank].write_ports_in_use--;
    ASSERT(dc->proc_id, dc->ports[bank].write_ports_in_use < dc->ports->num_write_ports);

    /* TODO: fix this by using a new_cycle_count to avoid replacing cycle_count */
    cycle_count = old_cycle_count;
    return FAILURE;
  }

  /* update cacheline fields and wake up dependent ops */
  dcache_fill_process_cacheline(req, data);

  cycle_count = old_cycle_count;
  return SUCCESS;
}

Flag do_oracle_dcache_access(Op* op, Addr* line_addr) {
  Dcache_Data* hit;
  hit = (Dcache_Data*)cache_access(&dc->dcache, op->oracle_info.va, line_addr, FALSE);

  if (hit)
    return TRUE;
  else
    return FALSE;
}

/**************************************************************************************/
/* Inline Methods */

static inline void dcache_stage_remove_src_op(Stage_Data* src_sd, int ii) {
  src_sd->ops[ii] = NULL;
  src_sd->op_count--;
  ASSERT(dc->proc_id, src_sd->op_count >= 0);
}

static inline Flag dcache_stage_addr_unready(Op* op) {
  /*
   * this is a little screwy. if the addr gen time is more than one cycle, then the op
   * won't get cleared out of the exec stage, thus making it block the functional unit
   * (not for the henry mem system, which handles agen itself)
   */
  if (cycle_count >= op_get_exec_cycle(op))
    return FALSE;

  /*
   * the DCACHE_CYCLES == 0 check is to make a address + 0 cycle cache.
   * This stage will grab the op out of exec a cycle before normal,
   * so the wake up happens in the same cycle as execute
   */
  if (DCACHE_CYCLES == 0 && cycle_count + 1 == op_get_exec_cycle(op))
    return FALSE;

  return TRUE;
}

static inline Flag dcache_stage_check_mem_type(Op* op) {
  /* just squish non-memory ops */

  if (op->uop->mem_type == NOT_MEM) {
    return FALSE;
  }

  /* skip prefetch ops if software prefetching is disabled */
  if (op->uop->mem_type == MEM_PF && !ENABLE_SWPRF) {
    // The op completes here without a real dcache access; still stamp dcache/wake cycles (as the
    // normal completion paths do) so op_assert_cycles_set_at_retire holds when it retires.
    op_set_dcache_cycle(op, cycle_count);
    op_set_done_cycle(op, cycle_count + DCACHE_CYCLES);
    op_set_wake_cycle(op, op_get_done_cycle(op));
    op->state = OS_SCHEDULED;
    return FALSE;
  }

  return TRUE;
}

static inline void dcache_hit_wp_collect_stats(Dcache_Data* line, Op* op) {
  if (!WP_COLLECT_STATS)
    return;

  if (!line) {
    ASSERT(dc->proc_id, PERFECT_DCACHE);
    return;
  }

  if (op->off_path) {
    if (line->fetched_by_offpath) {
      STAT_EVENT(dc->proc_id, DCACHE_HIT_OFFPATH_SAT_BY_OFFPATH);
    } else {
      STAT_EVENT(dc->proc_id, DCACHE_HIT_OFFPATH_SAT_BY_ONPATH);
    }
    return;
  }

  if (!line->fetched_by_offpath) {
    STAT_EVENT(dc->proc_id, DCACHE_HIT_ONPATH_SAT_BY_ONPATH);
    STAT_EVENT(dc->proc_id, DCACHE_USE_ONPATH);
    return;
  }
  line->fetched_by_offpath = FALSE;

  STAT_EVENT(dc->proc_id, DCACHE_HIT_ONPATH_SAT_BY_OFFPATH);
  STAT_EVENT(dc->proc_id, DCACHE_USE_OFFPATH);
  STAT_EVENT(dc->proc_id, DIST_DCACHE_FILL_OFFPATH_USED);
  STAT_EVENT(dc->proc_id, DIST_REQBUF_OFFPATH_USED);
  STAT_EVENT(dc->proc_id, DIST2_REQBUF_OFFPATH_USED_FULL);

  L1_Data* l1_line = do_l1_access(op);
  if (l1_line) {
    if (l1_line->fetched_by_offpath) {
      STAT_EVENT(dc->proc_id, L1_USE_OFFPATH);
      STAT_EVENT(dc->proc_id, DIST_L1_FILL_OFFPATH_USED);
      STAT_EVENT(dc->proc_id, L1_USE_OFFPATH_DATA);
      l1_line->fetched_by_offpath = FALSE;
      l1_line->l0_modified_fetched_by_offpath = TRUE;
    }
  }

  DEBUG(0, "Dcache hit: On path hits off path. va:%s op:%s op:0x%s wp_op:0x%s opu:%s wpu:%s dist:%s%s\n",
        hexstr64s(op->oracle_info.va), disasm_op(op, TRUE), hexstr64s(op->inst->addr), hexstr64s(line->offpath_op_addr),
        unsstr64(op->unique_num), unsstr64(line->offpath_op_unique),
        op->unique_num > line->offpath_op_unique ? " " : "-",
        op->unique_num > line->offpath_op_unique ? unsstr64(op->unique_num - line->offpath_op_unique)
                                                 : unsstr64(line->offpath_op_unique - op->unique_num));
}

static inline void dcache_fill_wp_collect_stats(Dcache_Data* line, Mem_Req* req) {
  if (!WP_COLLECT_STATS)
    return;

  if ((req->type == MRT_WB) || (req->type == MRT_WB_NODIRTY) ||
      (req->type == MRT_DPRF)) /* for now we don't consider prefetches */
    return;

  if (req->off_path) {
    switch (req->type) {
      case MRT_DFETCH:
      case MRT_DSTORE:
        STAT_EVENT(dc->proc_id, DCACHE_FILL_OFFPATH);
        STAT_EVENT(dc->proc_id, DIST_DCACHE_FILL);
        break;
      default:
        break;
    }
  } else {
    switch (req->type) {
      case MRT_DFETCH:
      case MRT_DSTORE:
        STAT_EVENT(dc->proc_id, DCACHE_FILL_ONPATH);
        STAT_EVENT(dc->proc_id, DIST_DCACHE_FILL);
        if (req->onpath_match_offpath)
          STAT_EVENT(dc->proc_id, DIST_DCACHE_FILL_ONPATH_PARTIAL);
        else
          STAT_EVENT(dc->proc_id, DIST_DCACHE_FILL_ONPATH);
        break;
      default:
        break;
    }
  }
}

static inline void dcache_miss_extra_access(Op* op, Cache* cache, Addr line_addr, uns8 proc_id, uns8 cache_cycle) {
  Addr one_more_addr;
  Addr extra_line_addr;
  Dcache_Data* extra_line;

  one_more_addr = ((line_addr >> LOG2(cache->line_size)) & 1)
                      ? ((line_addr >> LOG2(cache->line_size)) - 1) << LOG2(cache->line_size)
                      : ((line_addr >> LOG2(cache->line_size)) + 1) << LOG2(cache->line_size);

  extra_line = (Dcache_Data*)cache_access(cache, one_more_addr, &extra_line_addr, FALSE);
  ASSERT(proc_id, one_more_addr == extra_line_addr);

  if (extra_line) {
    STAT_EVENT_ALL(ONE_MORE_DISCARDED_L0CACHE);
    return;
  }

  Flag ret = new_mem_req(MRT_DFETCH, proc_id, extra_line_addr, cache->line_size,
                         cache_cycle - 1 + op->uop->extra_ld_latency, NULL, NULL, op->unique_num, 0);
  if (ret)
    STAT_EVENT_ALL(ONE_MORE_SUCESS);
  else
    STAT_EVENT_ALL(ONE_MORE_DISCARDED_MEM_REQ_FULL);
}

static inline Flag dcache_miss_new_mem_req(Op* op, Addr line_addr, Mem_Req_Type mem_req_type) {
  Flag sent = new_mem_req((mem_req_type), dc->proc_id, line_addr, DCACHE_LINE_SIZE,
                          DCACHE_CYCLES - 1 + op->uop->extra_ld_latency, op, dcache_fill_line, op->unique_num, 0);
  Node_Stage* node = &cmp_model.node_stage[dc->proc_id];
  if (sent && node->mem_blocked) {
    node->mem_blocked = FALSE;
    STAT_EVENT(dc->proc_id, MEM_BLOCK_LENGTH_0 + MIN2(node->mem_block_length, 5000) / 100);
    node->mem_block_length = 0;
  }
  return sent;
}

static inline void dcache_cacheline_hit(Op* op, Addr line_addr, Dcache_Data* line) {
  /* prefetching handle */
  if (PREF_FRAMEWORK_ON && (PREF_UPDATE_ON_WRONGPATH || !op->off_path)) {
    // if framework is on use new prefetcher. otherwise old one
    if (line->HW_prefetch) {
      pref_dl0_pref_hit(line_addr, op->inst->addr, 0);  // CHANGEME
      line->HW_prefetch = FALSE;
      STAT_EVENT(dc->proc_id, PREF_DCACHE_TOTAL_USED);
      STAT_EVENT(dc->proc_id, CORE_PREF_DCACHE_USED);
    } else {
      pref_dl0_hit(line_addr, op->inst->addr);
    }
  } else if ((STREAM_TRAIN_ON_WRONGPATH || !op->off_path) && line->HW_prefetch) {
    // old prefetcher code
    STAT_EVENT(op->proc_id, DCACHE_PREF_HIT);
    STAT_EVENT(op->proc_id, STREAM_DCACHE_PREF_HIT);
    line->HW_prefetch = FALSE;  // not anymore prefetched data
    if (L2L1PREF_ON)
      l2l1pref_dcache(line_addr, op);
    if (STREAM_PREFETCH_ON && STREAM_PREF_INTO_DCACHE) {
      stream_dl0_hit_train(line_addr);
    }
  }
  if (L2L1PREF_ON && L2L1_DC_HIT_TRAIN) {
    l2l1pref_dcache(line_addr, op);
  }

  /* update stats */
  dcache_hit_wp_collect_stats(line, op);
  if (!op->off_path) {
    STAT_EVENT(op->proc_id, DCACHE_HIT);
    STAT_EVENT(op->proc_id, DCACHE_HIT_ONPATH);
  } else {
    STAT_EVENT(op->proc_id, DCACHE_HIT_OFFPATH);
  }

  /* update cacheline state */
  op_set_done_cycle(op, cycle_count + DCACHE_CYCLES + op->uop->extra_ld_latency);
  line->read_count[op->off_path] = line->read_count[op->off_path] + (op->uop->mem_type == MEM_LD);
  line->write_count[op->off_path] = line->write_count[op->off_path] + (op->uop->mem_type == MEM_ST);
  line->misc_state = (line->misc_state & 2) | op->off_path;
  if (!op->off_path) {
    line->dirty |= op->uop->mem_type == MEM_ST;
  }

  /* wake up source inst if the op is completed */
  if (op->uop->mem_type != MEM_ST) {
    op_set_wake_cycle(op, op_get_done_cycle(op));
    wake_up_ops(op, REG_DATA_DEP, model->wake_hook);
  }
}

static inline void dcache_cacheline_miss(Op* op, Addr line_addr) {
  if (op->uop->mem_type == MEM_ST)
    STAT_EVENT(op->proc_id, POWER_DCACHE_WRITE_MISS);
  else
    STAT_EVENT(op->proc_id, POWER_DCACHE_READ_MISS);

  if (CACHE_STAT_ENABLE)
    dc_miss_stat(op);

  Flag wrongpath_dcmiss = FALSE;

  switch (op->uop->mem_type) {
    case MEM_LD:
      // scan the store forwarding buffer
      if (scan_stores(op->oracle_info.va, op->oracle_info.mem_size)) {
        if (!op->off_path) {
          STAT_EVENT(op->proc_id, DCACHE_ST_BUFFER_HIT);
          STAT_EVENT(op->proc_id, DCACHE_ST_BUFFER_HIT_ONPATH);
        } else {
          STAT_EVENT(op->proc_id, DCACHE_ST_BUFFER_HIT_OFFPATH);
        }

        op_set_done_cycle(op, cycle_count + DCACHE_CYCLES + op->uop->extra_ld_latency);
        op_set_wake_cycle(op, cycle_count + DCACHE_CYCLES + op->uop->extra_ld_latency);
        wake_up_ops(op, REG_DATA_DEP, model->wake_hook);
        break;
      }

      if (!(model->mem == MODEL_MEM) || !dcache_miss_new_mem_req(op, line_addr, MRT_DFETCH)) {
        op->state = OS_WAIT_MEM;  // go into this state if no miss buffer is available
        cmp_model.node_stage[dc->proc_id].mem_blocked = TRUE;
        mem->uncores[dc->proc_id].mem_block_start = freq_cycle_count(FREQ_DOMAIN_L1);
        STAT_EVENT(op->proc_id, DCACHE_MISS_WAITMEM);
        break;
      }

      if (PREF_UPDATE_ON_WRONGPATH || !op->off_path) {
        pref_dl0_miss(line_addr, op->inst->addr);
      }

      if (ONE_MORE_CACHE_LINE_ENABLE) {
        dcache_miss_extra_access(op, &dc->dcache, line_addr, dc->proc_id, DCACHE_CYCLES);
      }

      if (!op->off_path) {
        STAT_EVENT(op->proc_id, DCACHE_MISS);
        STAT_EVENT(op->proc_id, DCACHE_MISS_ONPATH);
        STAT_EVENT(op->proc_id, DCACHE_MISS_LD_ONPATH);
        op->oracle_info.dcmiss = TRUE;
        STAT_EVENT(op->proc_id, DCACHE_MISS_LD);
      } else {
        STAT_EVENT(op->proc_id, DCACHE_MISS_OFFPATH);
        STAT_EVENT(op->proc_id, DCACHE_MISS_LD_OFFPATH);
        wrongpath_dcmiss = FALSE;
      }
      op->state = OS_MISS;
      op->engine_info.dcmiss = TRUE;
      break;

    case MEM_PF:
    case MEM_WH:
      if (!(model->mem == MODEL_MEM) || !dcache_miss_new_mem_req(op, line_addr, MRT_DPRF)) {
        op->state = OS_WAIT_MEM;  // go into this state if no miss buffer is available
        cmp_model.node_stage[dc->proc_id].mem_blocked = TRUE;
        mem->uncores[dc->proc_id].mem_block_start = freq_cycle_count(FREQ_DOMAIN_L1);
        STAT_EVENT(op->proc_id, DCACHE_MISS_WAITMEM);
        break;
      }

      if (ONE_MORE_CACHE_LINE_ENABLE) {
        dcache_miss_extra_access(op, &dc->dcache, line_addr, dc->proc_id, DCACHE_CYCLES);
      }

      if (!op->off_path) {
        STAT_EVENT(op->proc_id, DCACHE_MISS);
        STAT_EVENT(op->proc_id, DCACHE_MISS_ONPATH);
        STAT_EVENT(op->proc_id, DCACHE_MISS_LD_ONPATH);
        op->oracle_info.dcmiss = TRUE;
        STAT_EVENT(op->proc_id, DCACHE_MISS_LD);
      } else {
        STAT_EVENT(op->proc_id, DCACHE_MISS_OFFPATH);
        STAT_EVENT(op->proc_id, DCACHE_MISS_LD_OFFPATH);
        wrongpath_dcmiss = FALSE;
      }
      op->state = OS_MISS;
      if (PREFS_DO_NOT_BLOCK_WINDOW || op->uop->mem_type == MEM_PF) {
        op_set_done_cycle(op, cycle_count + DCACHE_CYCLES + op->uop->extra_ld_latency);
        op->state = OS_SCHEDULED;
      }
      break;

    case MEM_ST:
      if (!(model->mem == MODEL_MEM) || !dcache_miss_new_mem_req(op, line_addr, MRT_DSTORE)) {
        op->state = OS_WAIT_MEM;
        cmp_model.node_stage[dc->proc_id].mem_blocked = TRUE;
        mem->uncores[dc->proc_id].mem_block_start = freq_cycle_count(FREQ_DOMAIN_L1);
        STAT_EVENT(op->proc_id, DCACHE_MISS_WAITMEM);
        break;
      }

      if (ONE_MORE_CACHE_LINE_ENABLE) {
        dcache_miss_extra_access(op, &dc->dcache, line_addr, dc->proc_id, DCACHE_CYCLES);
      }

      if (!op->off_path) {
        STAT_EVENT(op->proc_id, DCACHE_MISS);
        STAT_EVENT(op->proc_id, DCACHE_MISS_ONPATH);
        STAT_EVENT(op->proc_id, DCACHE_MISS_ST_ONPATH);
        op->oracle_info.dcmiss = TRUE;
        STAT_EVENT(op->proc_id, DCACHE_MISS_ST);
      } else {
        STAT_EVENT(op->proc_id, DCACHE_MISS_OFFPATH);
        STAT_EVENT(op->proc_id, DCACHE_MISS_ST_OFFPATH);
        wrongpath_dcmiss = FALSE;
      }
      op->state = OS_MISS;
      if (STORES_DO_NOT_BLOCK_WINDOW) {
        op_set_done_cycle(op, cycle_count + DCACHE_CYCLES + op->uop->extra_ld_latency);
        op->state = OS_SCHEDULED;
      }
      break;

    default:
      ASSERT(dc->proc_id, FALSE);
      break;
  }

  if (STREAM_PREFETCH_ON && (op->oracle_info.dcmiss || (STREAM_TRAIN_ON_WRONGPATH && wrongpath_dcmiss))) {
    _DEBUG(dc->proc_id, DEBUG_STREAM_MEM, "dl0 miss : line_addr :%d op_count %lld  type :%d\n", (int)line_addr,
           op->op_num, (int)op->uop->mem_type);
    stream_dl0_miss(line_addr);
  }
}

static inline Dcache_Data* dcache_fill_get_cacheline(Mem_Req* req) {
  Dcache_Data* data;
  Addr line_addr, repl_line_addr;

  /* Prefetch */
  bool is_off_path = USE_CONFIRMED_OFF ? req->off_path_confirmed : req->off_path;
  bool is_prefetch = (req->type == MRT_DPRF);
  if (DC_PREF_CACHE_ENABLE && (is_off_path || is_prefetch)) {
    DEBUG(dc->proc_id,
          "Filling pref_dcache off_path:%d addr:0x%s  :%7d index:%7d "
          "op_count:%d oldest:%lld\n",
          req->off_path, hexstr64s(req->addr), (int)req->addr, (int)(req->addr >> LOG2(DCACHE_LINE_SIZE)),
          req->op_count, (req->op_count ? req->oldest_op_unique_num : -1));

    data = (Dcache_Data*)cache_insert(&dc->pref_dcache, dc->proc_id, req->addr, &line_addr, &repl_line_addr);
    ASSERT(dc->proc_id, req->emitted_cycle);
    ASSERT(dc->proc_id, cycle_count >= req->emitted_cycle);
    /*
     * mark the data as HW_prefetch if prefetch mark it as fetched_by_offpath if off_path
     * this is done downstairs
     */
    ASSERT(dc->proc_id, data != NULL);
    return data;
  }

  /*
   * Do not insert the line yet, just check which line we need to replace.
   * If that line is dirty, it's possible that we won't be able to insert the writeback into the memory system.
   */
  Flag repl_line_valid;
  data = (Dcache_Data*)get_next_repl_line(&dc->dcache, dc->proc_id, req->addr, &repl_line_addr, &repl_line_valid);

  /* Exclusive hierarchy: the victim moves down a level instead of vanishing, unless it is
     an unused prefetch and we were asked to drop those. This runs before the writeback so
     that a demotion memory refuses leaves nothing half-done and the fill just retries. */
  if (EXCLUSIVE_CACHES && repl_line_valid) {
    if (EXCLUSIVE_DROP_UNUSED_PREF && data->HW_prefetch) {
      STAT_EVENT(dc->proc_id, EXCL_DROP_UNUSED_PREF);
    } else if (!mem_demote_to_mlc(get_proc_id_from_cmp_addr(repl_line_addr), repl_line_addr, data->dirty,
                                  data->HW_prefetched, data->HW_prefetched && !data->HW_prefetch)) {
      return NULL;
    }
  }

  if (repl_line_valid && data->dirty) {
    /* need to do a write-back */
    uns repl_proc_id = get_proc_id_from_cmp_addr(repl_line_addr);
    DEBUG(dc->proc_id, "Scheduling writeback of addr:0x%s\n", hexstr64s(repl_line_addr));
    ASSERT(dc->proc_id, data->read_count[0] || data->read_count[1] || data->write_count[0] || data->write_count[1]);
    ASSERT(dc->proc_id, repl_line_addr || data->fetched_by_offpath || data->HW_prefetched);

    if (!new_mem_dc_wb_req(MRT_WB, repl_proc_id, repl_line_addr, DCACHE_LINE_SIZE, 1, NULL, NULL, unique_count, TRUE)) {
      return NULL;
    }
    STAT_EVENT(dc->proc_id, DCACHE_WB_REQ_DIRTY);
    STAT_EVENT(dc->proc_id, DCACHE_WB_REQ);
  }

  // The victim is final past the writeback check (a failed writeback returns
  // NULL above and the fill retries). A prefetched line evicted with
  // HW_prefetch still set was never demand-used.
  if (repl_line_valid && data->HW_prefetch) {
    STAT_EVENT(dc->proc_id, PREF_DCACHE_TOTAL_NOT_USED);
    STAT_EVENT(dc->proc_id, CORE_PREF_DCACHE_NOT_USED);
  }

  DEBUG(dc->proc_id,
        "Filling dcache  off_path:%d addr:0x%s  :%7d index:%7d op_count:%d "
        "oldest:%lld\n",
        req->off_path, hexstr64s(req->addr), (int)req->addr, (int)(req->addr >> LOG2(DCACHE_LINE_SIZE)), req->op_count,
        (req->op_count ? req->oldest_op_unique_num : -1));

  data = (Dcache_Data*)cache_insert(&dc->dcache, dc->proc_id, req->addr, &line_addr, &repl_line_addr);
  ASSERT(dc->proc_id, req->emitted_cycle);
  ASSERT(dc->proc_id, cycle_count >= req->emitted_cycle);
  ASSERT(dc->proc_id, ((int)req->mlc_hit + (int)req->l1_hit) < 2);

  STAT_EVENT(dc->proc_id, DCACHE_FILL);
  INC_STAT_EVENT(dc->proc_id, DATA_LD_CYCLES_ONPATH + req->off_path, cycle_count - req->emitted_cycle);
  if (req->mlc_hit) {
    STAT_EVENT(dc->proc_id, DATA_LD_MLC_ACCESSES_ONPATH + req->off_path);
    INC_STAT_EVENT(dc->proc_id, DATA_LD_MLC_CYCLES_ONPATH + req->off_path, cycle_count - req->emitted_cycle);
  } else if (req->l1_hit) {
    STAT_EVENT(dc->proc_id, DATA_LD_L1_ACCESSES_ONPATH + req->off_path);
    INC_STAT_EVENT(dc->proc_id, DATA_LD_L1_CYCLES_ONPATH + req->off_path, cycle_count - req->emitted_cycle);
  } else {
    STAT_EVENT(dc->proc_id, DATA_LD_MEM_ACCESSES_ONPATH + req->off_path);
    INC_STAT_EVENT(dc->proc_id, DATA_LD_MEM_CYCLES_ONPATH + req->off_path, cycle_count - req->emitted_cycle);
  }

  return data;
}

static inline void dcache_fill_process_cacheline(Mem_Req* req, Dcache_Data* data) {
  /* collect wp stat */
  dcache_fill_wp_collect_stats(data, req);

  /* set up dcache line fields */
  data->dirty = req->dirty_l0 ? TRUE : FALSE;
  data->prefetch = TRUE;
  data->read_count[0] = 0;
  data->read_count[1] = 0;
  data->write_count[0] = 0;
  data->write_count[1] = 0;
  data->misc_state = req->off_path | req->off_path << 1;
  data->fetched_by_offpath = USE_CONFIRMED_OFF ? req->off_path_confirmed : req->off_path;
  data->offpath_op_addr = req->oldest_op_addr;
  data->offpath_op_unique = req->oldest_op_unique_num;
  data->fetch_cycle = cycle_count;
  data->onpath_use_cycle = (req->type == MRT_DPRF || req->off_path) ? 0 : cycle_count;

  if (req->type == MRT_DPRF) {  // cmp FIXME
    data->HW_prefetch = TRUE;
    data->HW_prefetched = TRUE;
  } else {
    data->HW_prefetch = FALSE;
    data->HW_prefetched = FALSE;
  }

  /* process req op */
  Op** op_p = (Op**)list_start_head_traversal(&req->op_ptrs);
  Counter* op_unique = (Counter*)list_start_head_traversal(&req->op_uniques);
  for (; op_p;
       op_p = (Op**)list_next_element(&req->op_ptrs), op_unique = (Counter*)list_next_element(&req->op_uniques)) {
    Op* op = *op_p;
    ASSERT(dc->proc_id, op);
    ASSERT(dc->proc_id, op_unique);

    if (op->unique_num != *op_unique || !op->op_pool_valid) {
      continue;
    }

    /* Verify proc_id consistency only for valid ops we're going to process */
    ASSERT(dc->proc_id, dc->proc_id == op->proc_id);
    ASSERT(dc->proc_id, op->proc_id == req->proc_id);

    /* update cacheline metadata */
    if (!op->off_path && op->uop->mem_type == MEM_ST)
      ASSERT(dc->proc_id, data->dirty);

    data->prefetch &= op->uop->mem_type == MEM_PF || op->uop->mem_type == MEM_WH;
    data->read_count[op->off_path] += (op->uop->mem_type == MEM_LD);
    data->write_count[op->off_path] += (op->uop->mem_type == MEM_ST);

    DEBUG(dc->proc_id, "%s: %s line addr:0x%s: %7d\n", unsstr64(op->op_num), disasm_op(op, FALSE), hexstr64s(req->addr),
          (int)(req->addr >> LOG2(DCACHE_LINE_SIZE)));

    /* wake up dependent ops */
    DEBUG(dc->proc_id, "Awakening op_num:%lld %d %d\n", op->op_num, op->engine_info.l1_miss_satisfied, op->in_rdy_list);
    ASSERT(dc->proc_id, !op->in_rdy_list);

    // Record completion once (keeps done_cycle write-once). A store/prefetch that
    // missed with *_DO_NOT_BLOCK_WINDOW already completed early (lines ~684/715)
    // but still has this pending fill; keep its earlier done_cycle. Any other op
    // arriving here already-done is a bug.
    if (op_get_done_cycle(op) == MAX_CTR) {
      op_set_done_cycle(op, cycle_count + 1);
      if (!op->off_path && op->uop->mem_type == MEM_LD)
        ld_served_stat(op, req->mlc_hit ? LD_SERVED_MLC : (req->l1_hit ? LD_SERVED_LLC : LD_SERVED_MEM),
                       req->mlc_hit ? LD_LAT_MLC : (req->l1_hit ? LD_LAT_LLC : LD_LAT_MEM));
    } else {
      Mem_Type mt = op->uop->mem_type;
      ASSERT(dc->proc_id, mt == MEM_ST || mt == MEM_PF || mt == MEM_WH);
    }
    op->state = OS_SCHEDULED;

    if (op->uop->mem_type != MEM_ST) {
      op_set_wake_cycle(op, op_get_done_cycle(op));
      wake_up_ops(op, REG_DATA_DEP, model->wake_hook);
    }
  }

  /*
   * This write_count is missing all the stores that retired before this fill happened.
   * Still, we know at least one on-path write must have occurred if the line is dirty.
   */
  if (data->dirty && data->write_count[0] == 0)
    data->write_count[0] = 1;

  ASSERT(dc->proc_id, data->read_count[0] || data->read_count[1] || data->write_count[0] || data->write_count[1] ||
                          req->off_path || data->prefetch || data->HW_prefetch);
}

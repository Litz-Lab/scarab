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
 * File         : bp_targ_mech.c
 * Author       : HPS Research Group
 * Date         : 12/9/1998
 * Description  :
 ***************************************************************************************/

#include "bp/bp_targ_mech.h"

#include <string.h>

#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "debug/debug.param.h"
#include "debug/debug_macros.h"
#include "debug/debug_print.h"

#include "bp/bp.param.h"
#include "core.param.h"

#include "bp/bp.h"
#include "bp/btb.h"
#include "isa/isa_macros.h"
#include "libs/cache_lib.h"

#include "ft.h"
#include "statistics.h"

/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_BP, ##args)
#define DEBUG_CRS(proc_id, args...) _DEBUG(proc_id, DEBUG_CRS, ##args)
#define DEBUGU_CRS(proc_id, args...) _DEBUGU(proc_id, DEBUG_CRS, ##args)
#define DEBUG_BTB(proc_id, args...) _DEBUG(proc_id, DEBUG_BTB, ##args)

#define STAT_EVENT_BTB_OUTCOME(op, prefix, hit, pred_targ, fallthrough, tag_alias)                                   \
  do {                                                                                                               \
    if ((hit) && (pred_targ) == (op)->oracle_info.target) {                                                          \
      STAT_EVENT((op)->proc_id, (op)->off_path ? prefix##CORRECT_OFFPATH : prefix##CORRECT);                         \
    } else if (!(hit) && (fallthrough) == (op)->oracle_info.target) {                                                \
      STAT_EVENT((op)->proc_id, (op)->off_path ? prefix##INCORRECT_BUT_TARGET_CORRECT_OFFPATH                        \
                                               : prefix##INCORRECT_BUT_TARGET_CORRECT);                              \
    } else if ((hit) && (tag_alias)) {                                                                               \
      STAT_EVENT((op)->proc_id, (op)->off_path ? prefix##INCORRECT_TAG_ALIAS_OFFPATH : prefix##INCORRECT_TAG_ALIAS); \
    } else {                                                                                                         \
      STAT_EVENT((op)->proc_id, (op)->off_path ? prefix##INCORRECT_OFFPATH : prefix##INCORRECT);                     \
    }                                                                                                                \
  } while (0)

Addr _prev_stat_event_btb_addr = 0;
static inline void stat_event_btb_addr(uns proc_id, Addr addr) {
  Addr diff = addr ^ _prev_stat_event_btb_addr;
  for (uns ii = 0; ii < 64; ii++) {
    if ((diff >> ii) & 0b1)
      STAT_EVENT(proc_id, BTB_ADDR_0_ENTROPY + ii);
    STAT_EVENT(proc_id, BTB_ADDR_0_ZERO + 2 * ii + (addr >> ii & 0b1));
  }
  _prev_stat_event_btb_addr = addr;
}

#define STAT_EVENT_BTB_BANK(proc_id, level, case, bank_id)        \
  do {                                                            \
    STAT_EVENT(proc_id, BTB_##level##_##case##_BANK_0 + bank_id); \
  } while (0)

#define STAT_EVENT_IBTB_OUTCOME(op, prefix, correct)                                                               \
  do {                                                                                                             \
    STAT_EVENT((op)->proc_id, (op)->off_path ? ((correct) ? prefix##_CORRECT_OFFPATH : prefix##_INCORRECT_OFFPATH) \
                                             : ((correct) ? prefix##_CORRECT : prefix##_INCORRECT));               \
  } while (0)

/**************************************************************************************/
/* bp_crs_push: */

void bp_crs_push(Bp_Data* bp_data, Op* op) {
  Addr addr = ADDR_PLUS_OFFSET(op->inst->addr, op->inst->inst_size);

  Flag flag = op->off_path;
  Crs_Entry* ent = &bp_data->crs.entries[bp_data->crs.tail << 1 | flag];

  ASSERT(bp_data->proc_id, bp_data->proc_id == op->proc_id);

  ent->addr = addr;
  ent->op_num = op->op_num;
  bp_data->crs.off_path[bp_data->crs.tail] = op->off_path;
  bp_data->crs.tail = CIRC_INC2(bp_data->crs.tail, CRS_ENTRIES);

  if (bp_data->crs.depth == CRS_ENTRIES) {
    bp_data->crs.head = CIRC_INC2(bp_data->crs.head, CRS_ENTRIES);
    DEBUG_CRS(bp_data->proc_id, "CLOBBER    head:%d  tail:%d\n", bp_data->crs.head, bp_data->crs.tail_save);
    STAT_EVENT(bp_data->proc_id, CRS_CLOBBER);
  } else {
    bp_data->crs.depth++;
    ASSERTM(bp_data->proc_id, bp_data->crs.depth <= CRS_ENTRIES, "bp_data->crs_depth:%d\n", bp_data->crs.depth);
  }

  if (!op->off_path) {
    bp_data->crs.tail_save = bp_data->crs.tail;
    bp_data->crs.depth_save = bp_data->crs.depth;
  }

  DEBUG_CRS(bp_data->proc_id,
            "PUSH       head:%d  tail:%d  depth:%d  op:%s  addr:0x%s  type:%s  "
            "offpath:%d\n",
            bp_data->crs.head, bp_data->crs.tail, bp_data->crs.depth, unsstr64(op->op_num), hexstr64s(addr),
            cf_type_names[op->uop->cf_type], op->off_path);
}

/**************************************************************************************/
/* bp_crs_pop: */

Addr bp_crs_pop(Bp_Data* bp_data, Op* op) {
  uns new_tail = CIRC_DEC2(bp_data->crs.tail, CRS_ENTRIES);
  Flag flag = bp_data->crs.off_path[new_tail];
  Addr addr = bp_data->crs.entries[new_tail << 1 | flag].addr;
  Flag mispred;

  ASSERT(bp_data->proc_id, bp_data->proc_id == op->proc_id);
  if (bp_data->crs.depth == 0) {
    DEBUG_CRS(bp_data->proc_id, "UNDERFLOW  head:%d  tail:%d  offpath:%d\n", bp_data->crs.head, bp_data->crs.tail,
              op->off_path);
    STAT_EVENT(op->proc_id, CRS_MISS + PERFECT_CRS + 2 * op->off_path);
    return PERFECT_CRS ? op->oracle_info.target : convert_to_cmp_addr(bp_data->proc_id, 0);
  }
  bp_data->crs.tail = new_tail;
  bp_data->crs.depth--;
  ASSERT(bp_data->proc_id, bp_data->crs.depth >= 0);
  if (!op->off_path) {
    if (addr != op->oracle_info.npc)
      DEBUG_CRS(bp_data->proc_id, "MISS       addr:0x%s  true:0x%s\n", hexstr64s(addr), hexstr64s(op->oracle_info.npc));
    bp_data->crs.tail_save = bp_data->crs.tail;
    bp_data->crs.depth_save = bp_data->crs.depth;
  }

  DEBUG_CRS(bp_data->proc_id,
            "POP        head:%d  tail:%d  depth:%d  op:%s  addr:0x%s  type:%s  "
            "offpath:%d  true:0x%s  miss:%d\n",
            bp_data->crs.head, bp_data->crs.tail, bp_data->crs.depth,
            unsstr64(bp_data->crs.entries[bp_data->crs.tail << 1 | flag].op_num), hexstr64s(addr),
            cf_type_names[op->uop->cf_type], op->off_path, hexstr64s(op->oracle_info.npc), addr != op->oracle_info.npc);
  mispred = PERFECT_CRS ? 0 : addr != op->oracle_info.npc;
  STAT_EVENT(op->proc_id, CRS_MISS + !mispred + 2 * op->off_path);
  return PERFECT_CRS ? op->oracle_info.target : addr;
}

/**************************************************************************************/
/* bp_crs_recover: */

void bp_crs_recover(Bp_Data* bp_data) {
  uns8 ii;
  for (ii = 0; ii < CRS_ENTRIES; ii++)
    bp_data->crs.off_path[ii] = FALSE;
  bp_data->crs.tail = bp_data->crs.tail_save;
  bp_data->crs.depth = bp_data->crs.depth_save;
  DEBUG_CRS(bp_data->proc_id, "RECOVER    head:%d  tail:%d  depth:%d\n", bp_data->crs.head, bp_data->crs.tail_save,
            bp_data->crs.depth);
}

/**************************************************************************************/
/* bp_crs_realistic_push: */

void bp_crs_realistic_push(Bp_Data* bp_data, Op* op) {
  Addr addr = ADDR_PLUS_OFFSET(op->inst->addr, op->inst->inst_size);
  Crs_Entry* ent = &bp_data->crs.entries[bp_data->crs.next];

  ASSERT(bp_data->proc_id, bp_data->proc_id == op->proc_id);

  ent->addr = addr;
  ent->op_num = op->op_num;
  ent->nos = bp_data->crs.tos;
  bp_data->crs.off_path[bp_data->crs.next] = op->off_path;
  bp_data->crs.tos = bp_data->crs.next;
  ASSERT(bp_data->proc_id, bp_data->crs.tos < CRS_ENTRIES * 2);
  bp_data->crs.next = CIRC_INC2(bp_data->crs.next, CRS_ENTRIES);

  if (bp_data->crs.depth == CRS_ENTRIES) {
    DEBUG_CRS(bp_data->proc_id, "CLOBBER    next:%d  tos:%d  depth:%d\n", bp_data->crs.next, bp_data->crs.tos,
              bp_data->crs.depth);
    STAT_EVENT(bp_data->proc_id, CRS_CLOBBER);
  } else {
    bp_data->crs.depth++;
    ASSERTM(bp_data->proc_id, bp_data->crs.depth <= CRS_ENTRIES, "bp_data->crs_depth:%d\n", bp_data->crs.depth);
  }

  op->recovery_info.crs_next = bp_data->crs.next;
  op->recovery_info.crs_tos = bp_data->crs.tos;
  op->recovery_info.crs_depth = bp_data->crs.depth;

  DEBUG_CRS(bp_data->proc_id,
            "PUSH       next:%d  tos:%d  depth:%d  op:%s  addr:0x%s  type:%s  "
            "offpath:%d\n",
            bp_data->crs.next, bp_data->crs.tos, bp_data->crs.depth, unsstr64(op->op_num), hexstr64s(addr),
            cf_type_names[op->uop->cf_type], op->off_path);
}

/**************************************************************************************/
/* bp_crs_realistic_pop: */

Addr bp_crs_realistic_pop(Bp_Data* bp_data, Op* op) {
  uns new_next = CIRC_DEC2(bp_data->crs.next, CRS_ENTRIES);
  uns old_tos = bp_data->crs.tos;
  Addr addr;
  uns new_tos = bp_data->crs.entries[bp_data->crs.tos].nos;
  Flag mispred;

  UNUSED(old_tos);
  ASSERT(bp_data->proc_id, bp_data->proc_id == op->proc_id);

  switch (CRS_REALISTIC) {
    case 1:
      addr = bp_data->crs.entries[bp_data->crs.tos].addr;
      break;
    case 2:
      addr = bp_data->crs.entries[new_next].addr;
      break;
    default:
      old_tos = 0;
      // old_tos is messed with because of the stupid compiler warning about unused variables
      ASSERT(bp_data->proc_id, 0);
  }

  if (bp_data->crs.depth == 0) {
    DEBUG_CRS(bp_data->proc_id, "UNDERFLOW  next:%d  tos: %d  offpath:%d\n", bp_data->crs.next, bp_data->crs.tos,
              op->off_path);
    STAT_EVENT(op->proc_id, CRS_MISS + PERFECT_CRS + 2 * op->off_path);
    STAT_EVENT(op->proc_id, CRS_UNDERFLOW_ON_PATH + op->off_path); /*CHECK EXCLUSIVE UNDERFLOW*/
    return PERFECT_CRS ? op->oracle_info.target : convert_to_cmp_addr(bp_data->proc_id, 0);
  }

  if (CRS_REALISTIC == 2)
    bp_data->crs.next = new_next;
  bp_data->crs.depth--;
  ASSERT(bp_data->proc_id, bp_data->crs.depth >= 0);
  bp_data->crs.tos = new_tos;

  if (addr != op->oracle_info.npc)
    DEBUG_CRS(bp_data->proc_id, "MISS       addr:0x%s  true:0x%s\n", hexstr64s(addr), hexstr64s(op->oracle_info.npc));

  op->recovery_info.crs_next = bp_data->crs.next;
  op->recovery_info.crs_tos = bp_data->crs.tos;
  op->recovery_info.crs_depth = bp_data->crs.depth;

  DEBUG_CRS(bp_data->proc_id,
            "POP        next:%d  tos:%d  depth:%d  old_tos:%d  op:%s  "
            "addr:0x%s  type:%s  offpath:%d  true:0x%s  miss:%d\n",
            bp_data->crs.next, bp_data->crs.tos, bp_data->crs.depth, old_tos,
            unsstr64(bp_data->crs.entries[old_tos].op_num), hexstr64s(addr), cf_type_names[op->uop->cf_type],
            op->off_path, hexstr64s(op->oracle_info.npc), addr != op->oracle_info.npc);
  mispred = PERFECT_CRS ? 0 : addr != op->oracle_info.npc;
  STAT_EVENT(op->proc_id, CRS_MISS + !mispred + 2 * op->off_path);
  return PERFECT_CRS ? op->oracle_info.target : addr;
}

/**************************************************************************************/
/* bp_crs_realistic_recover: */

void bp_crs_realistic_recover(Bp_Data* bp_data, Recovery_Info* info) {
  bp_data->crs.next = info->crs_next;
  bp_data->crs.depth = info->crs_depth;
  bp_data->crs.tos = info->crs_tos;
  DEBUG_CRS(bp_data->proc_id, "RECOVER    next:%d  tos:%d  depth:%d\n", bp_data->crs.next, bp_data->crs.tos,
            bp_data->crs.depth);
}

/**************************************************************************************/
/* bp_crs_sync: */

void bp_crs_sync(Bp_Data* bp_data_src, Bp_Data* bp_data_dst) {
  memcpy(bp_data_dst->crs.entries, bp_data_src->crs.entries, sizeof(Crs_Entry) * CRS_ENTRIES * 2);
  memcpy(bp_data_dst->crs.off_path, bp_data_src->crs.off_path, sizeof(Flag) * CRS_ENTRIES);
  bp_data_dst->crs.depth = bp_data_src->crs.depth;
  bp_data_dst->crs.head = bp_data_src->crs.head;
  bp_data_dst->crs.tail = bp_data_src->crs.tail;
  bp_data_dst->crs.tail_save = bp_data_src->crs.tail_save;
  bp_data_dst->crs.depth_save = bp_data_src->crs.depth_save;
  bp_data_dst->crs.tos = bp_data_src->crs.tos;
  bp_data_dst->crs.next = bp_data_src->crs.next;
}

/**************************************************************************************/
/* btb_update_level: write (or update) one BTB-level cache entry for op. */

static void btb_update_level(Cache* cache, uns level, uns proc_id, Addr fetch_addr, Addr target) {
  ASSERT(proc_id, target != ADDR_INVALID);
  Addr intra_bank_addr, line_addr, repl_line_addr;
  uns bank_id;
  Flag tag_aliasing;

  if (level == BTB_L0) {
    bank_id = get_btb_bank_id(BTB_L0_BANKS, fetch_addr, &intra_bank_addr);
    STAT_EVENT_BTB_BANK(proc_id, L0, UPDATE, bank_id);
  } else if (level == BTB_L1) {
    bank_id = get_btb_bank_id(BTB_L1_BANKS, fetch_addr, &intra_bank_addr);
    STAT_EVENT_BTB_BANK(proc_id, L1, UPDATE, bank_id);
  } else if (level == BTB_MAIN) {
    bank_id = get_btb_bank_id(BTB_BANKS, fetch_addr, &intra_bank_addr);
    STAT_EVENT_BTB_BANK(proc_id, MAIN, UPDATE, bank_id);
  } else {
    ASSERT(proc_id, FALSE);
  }
  Addr* btb_line = (Addr*)cache_access_impl(&cache[bank_id], intra_bank_addr, &line_addr, &tag_aliasing, TRUE);

  if (!btb_line) {
    if (level == BTB_L0) {
      STAT_EVENT_BTB_BANK(proc_id, L0, INSERT, bank_id);
    } else if (level == BTB_L1) {
      STAT_EVENT_BTB_BANK(proc_id, L1, INSERT, bank_id);
    } else if (level == BTB_MAIN) {
      STAT_EVENT_BTB_BANK(proc_id, MAIN, INSERT, bank_id);
    } else {
      ASSERT(proc_id, FALSE);
    }
    btb_line = (Addr*)cache_insert(&cache[bank_id], proc_id, intra_bank_addr, &line_addr, &repl_line_addr);
  }
  *btb_line = target;
}

/**************************************************************************************/
/* bp_predict_btb: query the BTB and IBP once per branch and populate all
 * Btb_Pred_Info fields.  Must be called before any bp_predict_op() call for
 * the same op.  On entry op->btb_pred_info must be NULL; this function sets it
 * to &op->btb_pred and fills in every field so that bp_predict_op() is a pure
 * reader of btb_pred_info. */

void bp_predict_btb(Bp_Data* bp_data, Op* op) {
  ASSERT(bp_data->proc_id, !op->btb_pred_info);
  Btb_Pred_Info* btb_pred_info = &op->btb_pred;

  memset(btb_pred_info, 0, sizeof(*btb_pred_info));
  btb_pred_info->no_target = TRUE;

  if (!op->uop->cf_type)
    return;

  // Set pointer and init l0/l1 fields so bp_btb_gen_pred can populate them.
  op->btb_pred_info = btb_pred_info;
  btb_pred_info->btb_l0_hit = btb_pred_info->btb_l1_hit = btb_pred_info->btb_main_hit = FALSE;
  btb_pred_info->btb_l0_target = btb_pred_info->btb_l1_target = btb_pred_info->btb_main_target = 0;
  btb_pred_info->btb_pred_latency = MAX_UNS;

  const Addr pc_plus_offset = ADDR_PLUS_OFFSET(op->inst->addr, op->inst->inst_size);
  const Flag collect_btb_stats =
      op->uop->cf_type != CF_ICO && op->uop->cf_type != CF_RET && !(op->uop->bar_type & BAR_FETCH);

  /* Syscall: target is always known from oracle */
  if (op->uop->cf_type == CF_SYS) {
    btb_pred_info->no_target = FALSE;
    btb_pred_info->btb_pred_latency = BTB_L0_LATENCY;
    btb_pred_info->pred_target = convert_to_cmp_addr(bp_data->proc_id, op->oracle_info.npc);
    return;
  }

  /* Main BTB lookup (pred_func populates BTB hit/target fields as a side effect) */
  if (PERFECT_BTB) {
    ASSERT(bp_data->proc_id, op->oracle_info.target != ADDR_INVALID);
    if (BTB_L0_PRESENT) {
      btb_pred_info->btb_l0_hit = TRUE;
      btb_pred_info->btb_l0_target = op->oracle_info.target;
    }
    if (BTB_L1_PRESENT) {
      btb_pred_info->btb_l1_hit = TRUE;
      btb_pred_info->btb_l1_target = op->oracle_info.target;
    }
    btb_pred_info->btb_main_hit = TRUE;
    btb_pred_info->btb_main_target = op->oracle_info.target;
  } else {
    bp_data->bp_btb->pred_func(bp_data, op);
  }

  if (BTB_L0_PRESENT && btb_pred_info->btb_l0_hit && BTB_L0_LATENCY < btb_pred_info->btb_pred_latency) {
    btb_pred_info->btb_pred_latency = BTB_L0_LATENCY;
    btb_pred_info->pred_target = btb_pred_info->btb_l0_target;
  }
  if (BTB_L1_PRESENT && btb_pred_info->btb_l1_hit && BTB_L1_LATENCY < btb_pred_info->btb_pred_latency) {
    btb_pred_info->btb_pred_latency = BTB_L1_LATENCY;
    btb_pred_info->pred_target = btb_pred_info->btb_l1_target;
  }
  if (btb_pred_info->btb_main_hit && BTB_MAIN_LATENCY < btb_pred_info->btb_pred_latency) {
    btb_pred_info->btb_pred_latency = BTB_MAIN_LATENCY;
    btb_pred_info->pred_target = btb_pred_info->btb_main_target;
  }

  if (collect_btb_stats) {
    STAT_EVENT_BTB_OUTCOME(op, ALL_BTB_, btb_pred_info->btb_main_hit, btb_pred_info->btb_main_target, pc_plus_offset,
                           btb_pred_info->btb_main_tag_alias);
  }
  if (btb_pred_info->btb_pred_latency == MAX_UNS) {
    btb_pred_info->pred_target = pc_plus_offset;
    if (pc_plus_offset == op->oracle_info.target) {
      /* Fall-through equals the actual target: treat as no miss */
      btb_pred_info->no_target = FALSE;
      btb_pred_info->btb_pred_latency = BTB_L0_LATENCY;
    }
  }

  /* Per-BTB-level outcome stats */
  if (collect_btb_stats) {
    if (BTB_L0_PRESENT) {
      STAT_EVENT_BTB_OUTCOME(op, BTB_L0_, btb_pred_info->btb_l0_hit, btb_pred_info->btb_l0_target, pc_plus_offset,
                             btb_pred_info->btb_l0_tag_alias);
    }
    if (BTB_L1_PRESENT) {
      STAT_EVENT_BTB_OUTCOME(op, BTB_L1_, btb_pred_info->btb_l1_hit, btb_pred_info->btb_l1_target, pc_plus_offset,
                             btb_pred_info->btb_l1_tag_alias);
    }
    STAT_EVENT_BTB_OUTCOME(op, BTB_MAIN_, btb_pred_info->btb_main_hit, btb_pred_info->btb_main_target, pc_plus_offset,
                           btb_pred_info->btb_main_tag_alias);
  }

  /* PERFECT_CBR_BTB: use oracle target for conditional branches */
  if (PERFECT_CBR_BTB && (op->uop->cf_type == CF_CBR || op->uop->cf_type == CF_REP)) {
    btb_pred_info->no_target = FALSE;
    btb_pred_info->btb_pred_latency = BTB_L0_LATENCY;
    ASSERT(bp_data->proc_id, op->oracle_info.target != ADDR_INVALID);
    btb_pred_info->pred_target = op->oracle_info.target;
  }

  /* PERFECT_BP: for CBR/REP the direction is always known, so no_target = FALSE */
  if (PERFECT_BP && (op->uop->cf_type == CF_CBR || op->uop->cf_type == CF_REP)) {
    btb_pred_info->no_target = FALSE;
  }

  if (ENABLE_IBP && (op->uop->cf_type == CF_IBR || op->uop->cf_type == CF_ICALL)) {
    Addr ibp_target = bp_data->bp_ibtb->pred_func(bp_data, op);
    if (ibp_target) {
      btb_pred_info->pred_target = ibp_target;
      btb_pred_info->no_target = FALSE;
      btb_pred_info->btb_pred_latency = BTB_MAIN_LATENCY;
      btb_pred_info->ibp_miss = FALSE;
      STAT_EVENT_IBTB_OUTCOME(op, ALL_IBTB, ibp_target == op->oracle_info.target);
    } else {
      btb_pred_info->ibp_miss = TRUE;
      STAT_EVENT_IBTB_OUTCOME(op, ALL_IBTB, FALSE);
    }
  }

  if (!btb_pred_miss(btb_pred_info))
    btb_pred_info->no_target = FALSE;
  btb_pred_info->pred_target = convert_to_cmp_addr(bp_data->proc_id, btb_pred_info->pred_target);
}

/**************************************************************************************/
/* bp_btb_post_bp_predict: stores op's final pred dir in bp_data for next BTB access,
 * after ALL BPs made predictions. This must be coupled with bp_predict_btb(),
 * having any bp_predict_op()s in between. */
void bp_btb_post_bp_predict(Bp_Data* bp_data, Op* op) {
  if (op->uop->cf_type && op->uop->cf_type != CF_SYS) {
    // CF_SYS does not terminate an FT / basic block
    bp_data->prev_cf_pred = op->bp_pred_info->pred;
    bp_data->prev_cf_target = op->bp_pred_info->pred_npc;
  }
}

/**************************************************************************************/
/* bp_btb_init: */

void bp_btb_gen_init(Bp_Data* bp_data, Bp_Data* primary_bp) {
  // btb line size set to 1
  if (!bp_data->bp_id) {
    ASSERT(bp_data->proc_id, BTB_ENTRIES / BTB_BANKS >= BTB_ASSOC);
    for (uns ii = 0; ii < BTB_BANKS; ii++) {
      char name[MAX_STR_LENGTH + 1];
      snprintf(name, MAX_STR_LENGTH, "BTB BANK %d", ii);
      Cache_Index_Config idx_cfg = {.index_hash_id = BTB_HASH,
                                    .ei_num_track_bits = BTB_NUM_TRACK_BITS,
                                    .ei_interval_size = BTB_INTERVAL_SIZE,
                                    .ei_remap_rate = BTB_REMAP_RATE,
                                    .ei_switch_thresh_pct = BTB_SWITCH_THRESH_PCT,
                                    .ei_stat_base = BTB_MAIN_EI_SWITCHES,
                                    .csv_map_path = BTB_CSV_MAP_FILE};
      init_cache_impl(&bp_data->btb[ii], name, BTB_ENTRIES / BTB_BANKS, BTB_ASSOC, 1, BTB_TAG_BITS, sizeof(Addr),
                      REPL_TRUE_LRU, idx_cfg);
    }

    if (BTB_L0_PRESENT) {
      ASSERT(bp_data->proc_id, BTB_L0_ENTRIES / BTB_L0_BANKS >= BTB_L0_ASSOC);
      for (uns ii = 0; ii < BTB_L0_BANKS; ii++) {
        char name[MAX_STR_LENGTH + 1];
        snprintf(name, MAX_STR_LENGTH, "BTB_L0 BANK %d", ii);
        Cache_Index_Config idx_cfg = {.index_hash_id = BTB_L0_HASH,
                                      .ei_num_track_bits = BTB_L0_NUM_TRACK_BITS,
                                      .ei_interval_size = BTB_L0_INTERVAL_SIZE,
                                      .ei_remap_rate = BTB_L0_REMAP_RATE,
                                      .ei_switch_thresh_pct = BTB_L0_SWITCH_THRESH_PCT,
                                      .ei_stat_base = BTB_L0_EI_SWITCHES,
                                      .csv_map_path = BTB_L0_CSV_MAP_FILE};
        init_cache_impl(&bp_data->btb_l0[ii], name, BTB_L0_ENTRIES / BTB_L0_BANKS, BTB_L0_ASSOC, 1, BTB_L0_TAG_BITS,
                        sizeof(Addr), REPL_TRUE_LRU, idx_cfg);
      }
    }

    if (BTB_L1_PRESENT) {
      ASSERT(bp_data->proc_id, BTB_L1_ENTRIES / BTB_L1_BANKS >= BTB_L1_ASSOC);
      for (uns ii = 0; ii < BTB_L1_BANKS; ii++) {
        char name[MAX_STR_LENGTH + 1];
        snprintf(name, MAX_STR_LENGTH, "BTB_L1 BANK %d", ii);
        Cache_Index_Config idx_cfg = {.index_hash_id = BTB_L1_HASH,
                                      .ei_num_track_bits = BTB_L1_NUM_TRACK_BITS,
                                      .ei_interval_size = BTB_L1_INTERVAL_SIZE,
                                      .ei_remap_rate = BTB_L1_REMAP_RATE,
                                      .ei_switch_thresh_pct = BTB_L1_SWITCH_THRESH_PCT,
                                      .ei_stat_base = BTB_L1_EI_SWITCHES,
                                      .csv_map_path = BTB_L1_CSV_MAP_FILE};
        init_cache_impl(&bp_data->btb_l1[ii], name, BTB_L1_ENTRIES / BTB_L1_BANKS, BTB_L1_ASSOC, 1, BTB_L1_TAG_BITS,
                        sizeof(Addr), REPL_TRUE_LRU, idx_cfg);
      }
    }
  } else {
    // points to the primary BP's shared caches
    bp_data->btb = primary_bp->btb;
    bp_data->btb_l0 = primary_bp->btb_l0;
    bp_data->btb_l1 = primary_bp->btb_l1;
  }
}

/**************************************************************************************/
/* bp_btb_gen_pred: */

void bp_btb_gen_pred(Bp_Data* bp_data, Op* op) {
  ASSERT(bp_data->proc_id, op->uop->cf_type);

  Btb_Pred_Info* bpi = op->btb_pred_info;
  Addr intra_bank_addr;
  Addr line_addr;
  Flag tag_aliasing;
  Flag lru = bp_data->bp_id ? FALSE : TRUE;

  op->btb_pred_info->btb_index_addr = op->inst->addr;

  if (BTB_L0_PRESENT) {
    uns bank_id = get_btb_bank_id(BTB_L0_BANKS, op->inst->addr, &intra_bank_addr);
    STAT_EVENT_BTB_BANK(op->proc_id, L0, PRED, bank_id);
    Addr* e = (Addr*)cache_access_impl(&bp_data->btb_l0[bank_id], intra_bank_addr, &line_addr, &tag_aliasing, lru);
    if (e) {
      bpi->btb_l0_hit = TRUE;
      bpi->btb_l0_target = *e;
      bpi->btb_l0_tag_alias = tag_aliasing;
    }
  }

  if (BTB_L1_PRESENT) {
    uns bank_id = get_btb_bank_id(BTB_L1_BANKS, op->inst->addr, &intra_bank_addr);
    STAT_EVENT_BTB_BANK(op->proc_id, L1, PRED, bank_id);
    Addr* e = (Addr*)cache_access_impl(&bp_data->btb_l1[bank_id], intra_bank_addr, &line_addr, &tag_aliasing, lru);
    if (e) {
      bpi->btb_l1_hit = TRUE;
      bpi->btb_l1_target = *e;
      bpi->btb_l1_tag_alias = tag_aliasing;
    }
  }

  uns bank_id = get_btb_bank_id(BTB_BANKS, op->inst->addr, &intra_bank_addr);
  STAT_EVENT_BTB_BANK(op->proc_id, MAIN, PRED, bank_id);
  Addr* e = (Addr*)cache_access_impl(&bp_data->btb[bank_id], intra_bank_addr, &line_addr, &tag_aliasing, lru);
  if (e) {
    bpi->btb_main_hit = TRUE;
    bpi->btb_main_target = *e;
    bpi->btb_main_tag_alias = tag_aliasing;
  }
}

/**************************************************************************************/
/* bp_btb_gen_update: */

void bp_btb_gen_update(Bp_Data* bp_data, Op* op) {
  ASSERT(bp_data->proc_id, bp_data->proc_id == op->proc_id);
  ASSERT(bp_data->proc_id, bp_data->bp_id == 0);
  ASSERT(bp_data->proc_id, op->uop->cf_type);

  Addr fetch_addr = op->inst->addr;
  Addr intra_bank_addr;
  Addr *btb_line, btb_line_addr, repl_line_addr;
  Flag tag_aliasing;
  uns bank_id = get_btb_bank_id(BTB_BANKS, fetch_addr, &intra_bank_addr);

  // if it was a btb miss, it is time to write it into the btb
  if (btb_pred_miss(op->btb_pred_info) && op->oracle_info.dir == TAKEN) {
    ASSERT(bp_data->proc_id, op->oracle_info.target != ADDR_INVALID);
    if (BTB_OFF_PATH_WRITES || !op->off_path) {
      DEBUG_BTB(bp_data->proc_id, "Writing BTB  addr:0x%s  target:0x%s\n", hexstr64s(fetch_addr),
                hexstr64s(op->oracle_info.target));
      STAT_EVENT(op->proc_id, BTB_WRITE + op->off_path);
      STAT_EVENT_BTB_BANK(op->proc_id, MAIN, UPDATE, bank_id);
      stat_event_btb_addr(op->proc_id, fetch_addr);

      btb_line = (Addr*)cache_access_impl(&bp_data->btb[bank_id], intra_bank_addr, &btb_line_addr, &tag_aliasing, TRUE);
      if (!btb_line) {
        STAT_EVENT_BTB_BANK(op->proc_id, MAIN, INSERT, bank_id);
        btb_line = (Addr*)cache_insert(&bp_data->btb[bank_id], bp_data->proc_id, intra_bank_addr, &btb_line_addr,
                                       &repl_line_addr);
      }
      *btb_line = op->oracle_info.target;
      // FIXME: the exceptions to this assert are really about x86 vs Alpha
      ASSERT(bp_data->proc_id, (fetch_addr == btb_line_addr) || TRUE);
    }
  } else if (!btb_pred_miss(op->btb_pred_info) && op->oracle_info.dir == TAKEN) {
    ASSERT(bp_data->proc_id, op->oracle_info.target != ADDR_INVALID);
    // For jitted CF we want to update the BTB if the target changes, even on btb hit
    // or For indirects we want to update the BTB if the target changes, even on btb hit
    // The detection relies on the target stored in the btb

    btb_line = (Addr*)cache_access_impl(&bp_data->btb[bank_id], intra_bank_addr, &btb_line_addr, &tag_aliasing, FALSE);

    // The following assertion can fail (due to eviction?)
    // ASSERT(bp_data->proc_id, btb_entry);
    if (btb_line && *btb_line != op->oracle_info.target) {
      cache_access_impl(&bp_data->btb[bank_id], intra_bank_addr, &btb_line_addr, &tag_aliasing, TRUE);
      if (BTB_OFF_PATH_WRITES || !op->off_path) {
        DEBUG_BTB(bp_data->proc_id, "Writing BTB  addr:0x%s  target:0x%s\n", hexstr64s(fetch_addr),
                  hexstr64s(op->oracle_info.target));
        STAT_EVENT(op->proc_id, BTB_WRITE + op->off_path);
        STAT_EVENT_BTB_BANK(op->proc_id, MAIN, UPDATE, bank_id);
        stat_event_btb_addr(op->proc_id, fetch_addr);
        *btb_line = op->oracle_info.target;
        // FIXME: the exceptions to this assert are really about x86 vs Alpha
        ASSERT(bp_data->proc_id, (fetch_addr == btb_line_addr) || TRUE);
      }
      STAT_EVENT(bp_data->proc_id, BTB_UPDATE_BTB_HIT_JITTED_NOT_CF + op->uop->cf_type);
    }
  }

  // Update L0 BTB
  if (BTB_L0_PRESENT && (BTB_OFF_PATH_WRITES || !op->off_path) && op->oracle_info.dir == TAKEN) {
    btb_update_level(bp_data->btb_l0, BTB_L0, bp_data->proc_id, fetch_addr, op->oracle_info.target);
  }

  // Update L1 BTB
  if (BTB_L1_PRESENT && (BTB_OFF_PATH_WRITES || !op->off_path) && op->oracle_info.dir == TAKEN) {
    btb_update_level(bp_data->btb_l1, BTB_L1, bp_data->proc_id, fetch_addr, op->oracle_info.target);
  }
}

/**************************************************************************************/
/* bp_btb_gen_recover: */

void bp_btb_gen_recover(Bp_Data* bp_data, Recovery_Info* info) {
  return;
}

/**************************************************************************************/
/* bp_btb_block_init: */

void bp_btb_block_init(Bp_Data* bp_data, Bp_Data* primary_bp) {
  if (!bp_data->bp_id) {
    DEBUG_BTB(bp_data->proc_id, "Initializing BLOCK_BTB\n");
    ASSERT(bp_data->proc_id, BTB_NUM_BRSLOT > 0);
    ASSERT(bp_data->proc_id, BTB_BLOCK_SIZE > 0);
    ASSERT(bp_data->proc_id, (1 << LOG2(BTB_BLOCK_SIZE)) == BTB_BLOCK_SIZE);

    ASSERT(bp_data->proc_id, BTB_BANKS == 1);
    ASSERT(bp_data->proc_id, BTB_ENTRIES >= BTB_ASSOC);
    Cache_Index_Config idx_cfg = {.index_hash_id = BTB_HASH,
                                  .ei_num_track_bits = BTB_NUM_TRACK_BITS,
                                  .ei_interval_size = BTB_INTERVAL_SIZE,
                                  .ei_remap_rate = BTB_REMAP_RATE,
                                  .ei_switch_thresh_pct = BTB_SWITCH_THRESH_PCT,
                                  .ei_stat_base = BTB_MAIN_EI_SWITCHES,
                                  .csv_map_path = BTB_CSV_MAP_FILE};
    init_cache_impl(bp_data->btb, "B-BTB", BTB_ENTRIES, BTB_ASSOC, 1, BTB_TAG_BITS, BLK_BTB_ENTRY_SIZE, REPL_TRUE_LRU,
                    idx_cfg);
  } else  // points to the primary BP's shared BTB
    bp_data->btb = primary_bp->btb;
}

/**************************************************************************************/
/* bp_btb_block_pred: */

void bp_btb_block_pred(Bp_Data* bp_data, Op* op) {
  ASSERT(bp_data->proc_id, op->uop->cf_type);

  Btb_Pred_Info* bpi = op->btb_pred_info;

  Addr btb_index_addr = 0;
  // Actual BTB does not require this because the index addr to look up BTB is given,
  // but Scarab needs to memorize the previous target as it looks up BTB without knowing the index addr.
  if (bp_data->prev_cf_btb_index_addr == 0) {
    // Only for the very first access
    btb_index_addr = ft_get_ft_info(op->parent_FT).static_info.start;
  } else {
    if (bp_data->prev_cf_pred == TAKEN) {
      btb_index_addr = bp_data->prev_cf_target;
    } else {
      btb_index_addr = bp_data->prev_cf_btb_index_addr;
    }
  }
  ASSERT(bp_data->proc_id, btb_index_addr <= op->inst->addr);
  // Compute block-size-aligned (fall-through) address from the start of the first block, in case op is far away.
  btb_index_addr += (op->inst->addr - btb_index_addr) & ~(BTB_BLOCK_SIZE - 1);
  // Assert btb_index_addr <= op->inst->addr < btb_index_addr + BTB_BLOCK_SIZE
  ASSERT(bp_data->proc_id, btb_index_addr <= op->inst->addr);
  ASSERT(bp_data->proc_id, op->inst->addr < ADDR_PLUS_OFFSET(btb_index_addr, BTB_BLOCK_SIZE));

  // Store index for update
  op->btb_pred_info->btb_index_addr = btb_index_addr;

  // Prepare for next BTB lookup
  bp_data->prev_cf_btb_index_addr = btb_index_addr;

  STAT_EVENT_BTB_BANK(op->proc_id, MAIN, PRED, 0);
  Addr btb_line_addr;
  Flag tag_aliasing;
  Blk_Btb_BrSlot* br_slots =
      (Blk_Btb_BrSlot*)cache_access_impl(bp_data->btb, btb_index_addr, &btb_line_addr, &tag_aliasing, TRUE);
  bpi->btb_main_tag_alias = tag_aliasing;

  bpi->btb_main_hit = FALSE;
  if (br_slots) {
    for (uns ii = 0; ii < BTB_NUM_BRSLOT; ii++) {
      if (br_slots[ii].valid == TRUE && br_slots[ii].addr == op->inst->addr) {
        bpi->btb_main_hit = TRUE;
        bpi->btb_main_target = br_slots[ii].target;
        break;
      }
    }
  }
}

/**************************************************************************************/
/* bp_btb_block_update: */

void bp_btb_block_update(Bp_Data* bp_data, Op* op) {
  ASSERT(bp_data->proc_id, bp_data->proc_id == op->proc_id);
  ASSERT(bp_data->proc_id, bp_data->bp_id == 0);
  ASSERT(bp_data->proc_id, op->uop->cf_type);

  if (BTB_OFF_PATH_WRITES || !op->off_path) {
    Addr btb_index_addr = op->btb_pred_info->btb_index_addr;
    ASSERT(bp_data->proc_id, btb_index_addr <= op->inst->addr);
    ASSERT(bp_data->proc_id, op->inst->addr < ADDR_PLUS_OFFSET(btb_index_addr, BTB_BLOCK_SIZE));

    if (op->oracle_info.dir == TAKEN) {
      ASSERT(bp_data->proc_id, op->oracle_info.target != ADDR_INVALID);
      DEBUG_BTB(bp_data->proc_id, "Writing BTB  btb addr:0x%s  op addr:0x%s  target:0x%s\n", hexstr64s(btb_index_addr),
                hexstr64s(op->inst->addr), hexstr64s(op->oracle_info.target));
      STAT_EVENT(op->proc_id, BTB_WRITE + op->off_path);
      stat_event_btb_addr(op->proc_id, btb_index_addr);

      Blk_Btb_BrSlot br_slot;
      br_slot.addr = op->inst->addr;
      br_slot.type = op->uop->cf_type;
      br_slot.target = op->oracle_info.target;
      br_slot.valid = TRUE;

      STAT_EVENT_BTB_BANK(op->proc_id, MAIN, UPDATE, 0);
      Addr btb_line_addr, repl_line_addr;
      Flag tag_aliasing;
      Blk_Btb_BrSlot* br_slots =
          (Blk_Btb_BrSlot*)cache_access_impl(bp_data->btb, btb_index_addr, &btb_line_addr, &tag_aliasing, TRUE);

      if (!br_slots) {
        STAT_EVENT_BTB_BANK(op->proc_id, MAIN, INSERT, 0);
        br_slots = (Blk_Btb_BrSlot*)cache_insert(bp_data->btb, bp_data->proc_id, btb_index_addr, &btb_line_addr,
                                                 &repl_line_addr);
        br_slots[0] = br_slot;
        // Invalidate the remaining slots
        for (uns ii = 1; ii < BTB_NUM_BRSLOT; ii++)
          br_slots[ii].valid = FALSE;
      } else {
        uns insert_pos = BTB_NUM_BRSLOT;
        for (uns ii = 0; ii < BTB_NUM_BRSLOT; ii++) {
          if (br_slots[ii].valid) {
            // slot that has smaller addr (br_slots[ii].addr < op->inst->addr) will be just skipped
            if (br_slots[ii].addr == op->inst->addr) {
              br_slots[ii] = br_slot;
              break;
            } else if (br_slots[ii].addr > op->inst->addr) {
              // If there is no self-modifying code (e.g. SPEC 2017 int), op must be conditional in this case.
              // Enable this assertion when debugging.
              // ASSERT(bp_data->proc_id,
              //        op->uop->cf_type == CF_CBR || op->uop->cf_type == CF_REP);
              if (op->uop->cf_type == CF_CBR || op->uop->cf_type == CF_REP) {
                // If this op is NOT always-taken, it needs to be inserted, not just appended.
                insert_pos = ii;
              } else {
                // If this op is always-taken, invalidate the rest as the block ends here.
                // Only happens with self-modifying code.
                br_slots[ii] = br_slot;
                for (uns jj = ii + 1; jj < BTB_NUM_BRSLOT; jj++)
                  br_slots[jj].valid = FALSE;
              }
              break;
            }
          } else {
            // br_slots[ii] does not store a valid op yet.
            br_slots[ii] = br_slot;
            break;
          }
        }

        if (insert_pos < BTB_NUM_BRSLOT) {
          // Naive replacement policy: op with the largest addr will be discarded.
          for (uns ii = BTB_NUM_BRSLOT - 1; ii > insert_pos; ii--) {
            if (br_slots[ii - 1].valid)
              br_slots[ii] = br_slots[ii - 1];
          }
          br_slots[insert_pos] = br_slot;
        }
      }

      ASSERT(bp_data->proc_id, (btb_index_addr == btb_line_addr) || TRUE);
    }
  }
}

/**************************************************************************************/
/* bp_btb_recover: */

void bp_btb_block_recover(Bp_Data* bp_data, Recovery_Info* info) {
  DEBUG_BTB(bp_data->proc_id, "Recovering BLOCK_BTB prev cf to 0x%llx\n", info->op->inst->addr);
  bp_data->prev_cf_pred = info->op->oracle_info.dir;
  bp_data->prev_cf_target = info->op->oracle_info.target;
  bp_data->prev_cf_btb_index_addr = info->op->btb_pred_info->btb_index_addr;
}

/**************************************************************************************/
/* bp_tc_tagged_init: */

void bp_ibtb_tc_tagged_init(Bp_Data* bp_data, Bp_Data* primary_bp) {
  // line size set to 1
  if (!bp_data->bp_id) {
    init_cache_impl(bp_data->tc_tagged, "TC", TC_ENTRIES, TC_ASSOC, 1, TC_TAG_BITS, sizeof(Addr), REPL_TRUE_LRU,
                    get_default_cache_index_config());
  } else  // points to the primary BP's shared tc_tagged
    bp_data->tc_tagged = primary_bp->tc_tagged;
}

/**************************************************************************************/
/* bp_tc_tagged_pred: */

Addr bp_ibtb_tc_tagged_pred(Bp_Data* bp_data, Op* op) {
  Addr addr;
  uns32 hist;
  uns32 tc_index;
  Addr* tc_entry;
  Addr line_addr;
  Addr target;

  if (PERFECT_IBP)
    return op->oracle_info.target;

  /* branch history can be updated in one of two ways */
  /* 1. branch history (USE_PAT_HIST) */
  /* 2. path history */
  if (USE_PAT_HIST) {
    addr = op->inst->addr;
    bp_data->targ_hist = bp_data->global_hist; /* use global history from conditional branches */
    hist = bp_data->targ_hist;
    op->btb_pred.ibp_pred_targ_hist = bp_data->targ_hist;
    op->recovery_info.targ_hist = bp_data->targ_hist;
  } else {
    addr = op->inst->addr;
    hist = bp_data->targ_hist;
    op->btb_pred.ibp_pred_targ_hist = bp_data->targ_hist;
    bp_data->targ_hist >>= bp_data->target_bit_length;
    op->recovery_info.targ_hist =
        bp_data->targ_hist |
        (op->oracle_info.target >> 2 & N_BIT_MASK(bp_data->target_bit_length) << (32 - bp_data->target_bit_length));
    bp_data->targ_hist |= op->oracle_info.target >> 2 & N_BIT_MASK(bp_data->target_bit_length)
                                                            << (32 - bp_data->target_bit_length);
  }
  tc_index = hist ^ addr;
  if (IBTB_HASH_TOS)
    tc_index = tc_index ^ op->recovery_info.tos_addr;
  tc_entry = (Addr*)cache_access(bp_data->tc_tagged, tc_index, &line_addr, bp_data->bp_id ? FALSE : TRUE);  // TODO

  if (tc_entry)
    target = *tc_entry;
  else
    target = 0;

  if (!op->off_path)
    STAT_EVENT(op->proc_id, IBTB_TARG_CACHE_MISS + (target == op->oracle_info.npc));
  else
    STAT_EVENT(op->proc_id, IBTB_TARG_CACHE_MISS_OFFPATH + (target == op->oracle_info.npc));

  return target;
}

/**************************************************************************************/
/* bp_tc_tagged_update: */

void bp_ibtb_tc_tagged_update(Bp_Data* bp_data, Op* op) {
  Addr addr = op->inst->addr;
  uns32 hist = op->btb_pred_info->ibp_pred_targ_hist;
  uns32 tc_index = hist ^ addr;
  Addr* tc_line;
  Addr tc_line_addr;
  Addr repl_line_addr;

  ASSERT(bp_data->proc_id, !bp_data->bp_id);
  if (IBTB_HASH_TOS)
    tc_index = tc_index ^ op->recovery_info.tos_addr;

  DEBUG(bp_data->proc_id, "Writing target cache target for op_num:%s\n", unsstr64(op->op_num));
  tc_line = (Addr*)cache_access(bp_data->tc_tagged, tc_index, &tc_line_addr, bp_data->bp_id ? FALSE : TRUE);
  if (tc_line) {
    // ASSERT(bp_data->proc_id, !op->btb_pred_info->ibp_miss);
  } else {
    // ASSERT(bp_data->proc_id, op->btb_pred_info->ibp_miss);
    tc_line = (Addr*)cache_insert(bp_data->tc_tagged, bp_data->proc_id, tc_index, &tc_line_addr, &repl_line_addr);
  }
  *tc_line = op->oracle_info.target;

  STAT_EVENT(op->proc_id, IBTB_TARG_CACHE_WRITE + op->off_path);
}

/**************************************************************************************/
/* bp_tc_tagged_recover */

void bp_ibtb_tc_tagged_recover(Bp_Data* bp_data, Recovery_Info* info) {
  DEBUG(bp_data->proc_id, "Recovering target cache history\n");
  bp_data->targ_hist = info->targ_hist;
}

/**************************************************************************************/
/* bp_tc_tagless_init */

void bp_ibtb_tc_tagless_init(Bp_Data* bp_data, Bp_Data* primary_bp) {
  if (!bp_data->bp_id) {
    uns ii;
    bp_data->tc_tagless = (Addr*)malloc(sizeof(Addr) * (0x1 << IBTB_HIST_LENGTH));
    for (ii = 0; ii < 0x1 << IBTB_HIST_LENGTH; ii++)
      bp_data->tc_tagless[ii] = 0;
  } else {
    bp_data->tc_tagless = primary_bp->tc_tagless;
  }
}

/**************************************************************************************/
/* bp_tc_tagless_pred */

#define COOK_HIST_BITS(hist, untouched) ((hist) >> (32 - IBTB_HIST_LENGTH + untouched) << untouched)
#define COOK_ADDR_BITS(addr, addr_shift) (((addr) >> addr_shift) & (N_BIT_MASK(IBTB_HIST_LENGTH)))

Addr bp_ibtb_tc_tagless_pred(Bp_Data* bp_data, Op* op) {
  Addr addr;
  uns32 hist;
  uns32 cooked_hist;
  uns32 cooked_addr;
  uns32 tc_index;
  Addr tc_entry;

  if (PERFECT_IBP)
    return op->oracle_info.target;

  /* branch history can be updated in one of two ways */
  /* 1. branch history (USE_PAT_HIST) */
  /* 2. path history */
  if (USE_PAT_HIST) {
    addr = op->inst->addr;
    bp_data->targ_hist = bp_data->global_hist; /* use global history from conditional branches */
    hist = bp_data->targ_hist;
    op->btb_pred.ibp_pred_targ_hist = bp_data->targ_hist;
    op->recovery_info.targ_hist = bp_data->targ_hist;
  } else {
    addr = op->inst->addr;
    hist = bp_data->targ_hist;
    op->btb_pred.ibp_pred_targ_hist = bp_data->targ_hist;
    bp_data->targ_hist >>= bp_data->target_bit_length;
    op->recovery_info.targ_hist =
        bp_data->targ_hist |
        (op->oracle_info.target >> 2 & N_BIT_MASK(bp_data->target_bit_length) << (32 - bp_data->target_bit_length));
    bp_data->targ_hist |= op->oracle_info.target >> 2 & N_BIT_MASK(bp_data->target_bit_length)
                                                            << (32 - bp_data->target_bit_length);
  }
  cooked_hist = COOK_HIST_BITS(hist, 0);
  cooked_addr = COOK_ADDR_BITS(addr, 2);
  tc_index = cooked_hist ^ cooked_addr;
  tc_entry = bp_data->tc_tagless[tc_index];

  if (IBTB_HASH_TOS) {
    uns32 cooked_tos_addr;
    cooked_tos_addr = COOK_ADDR_BITS(op->recovery_info.tos_addr, 2);
    tc_index = tc_index ^ cooked_tos_addr;
    tc_entry = bp_data->tc_tagless[tc_index];
  }

  if (!op->off_path)
    STAT_EVENT(op->proc_id, IBTB_TARG_CACHE_MISS + (tc_entry == op->oracle_info.npc));
  else
    STAT_EVENT(op->proc_id, IBTB_TARG_CACHE_MISS_OFFPATH + (tc_entry == op->oracle_info.npc));

  return tc_entry;
}

/**************************************************************************************/
/* bp_tc_tagless_update */

void bp_ibtb_tc_tagless_update(Bp_Data* bp_data, Op* op) {
  Addr addr = op->inst->addr;
  uns32 hist = op->btb_pred_info->ibp_pred_targ_hist;
  uns32 cooked_hist = COOK_HIST_BITS(hist, 0);
  uns32 cooked_addr = COOK_ADDR_BITS(addr, 2);
  uns32 tc_index = cooked_hist ^ cooked_addr;

  if (IBTB_HASH_TOS) {
    uns32 cooked_tos_addr;
    cooked_tos_addr = COOK_ADDR_BITS(op->recovery_info.tos_addr, 2);
    tc_index = tc_index ^ cooked_tos_addr;
  }

  DEBUG(bp_data->proc_id, "Writing target cache target for op_num:%s\n", unsstr64(op->op_num));
  bp_data->tc_tagless[tc_index] = op->oracle_info.target;

  STAT_EVENT(op->proc_id, IBTB_TARG_CACHE_WRITE + op->off_path);
}

/**************************************************************************************/
/* bp_tc_tagless_recover */

void bp_ibtb_tc_tagless_recover(Bp_Data* bp_data, Recovery_Info* info) {
  DEBUG(bp_data->proc_id, "Recovering target cache history\n");
  bp_data->targ_hist = info->targ_hist;
}

typedef enum {
  TC_SELECTOR_TAGLESS_STRONG,  // 0
  TC_SELECTOR_TAGLESS_WEAK,    // 1
  TC_SELECTOR_TAGGED_WEAK,     // 2
  TC_SELECTOR_TAGGED_STRONG    // 3
} Tc_Selector_Table_entry_Value;

/**************************************************************************************/
/* bp_tc_hybrid_init: */

void bp_ibtb_tc_hybrid_init(Bp_Data* bp_data, Bp_Data* primary_bp) {
  uns ii;

  /* Init the meta-predictor */
  bp_data->tc_selector = (uns8*)malloc(sizeof(uns8) * (0x1 << IBTB_HIST_LENGTH));
  for (ii = 0; ii < 0x1 << IBTB_HIST_LENGTH; ii++)
    bp_data->tc_selector[ii] = TC_SELECTOR_TAGLESS_WEAK;

  /* Init the tagless predictor */
  bp_data->tc_tagless = (Addr*)malloc(sizeof(Addr) * (0x1 << IBTB_HIST_LENGTH));
  for (ii = 0; ii < 0x1 << IBTB_HIST_LENGTH; ii++)
    bp_data->tc_tagless[ii] = 0;

  /* Init the tagged predictor */
  // line size set to 1
  if (!bp_data->bp_id) {
    init_cache_impl(bp_data->tc_tagged, "TC", TC_ENTRIES, TC_ASSOC, 1, TC_TAG_BITS, sizeof(Addr), REPL_TRUE_LRU,
                    get_default_cache_index_config());
  } else  // points to the primary BP's shared tc_tagged
    bp_data->tc_tagged = primary_bp->tc_tagged;
}

/**************************************************************************************/
/* bp_tc_hybrid_pred: */

Addr bp_ibtb_tc_hybrid_pred(Bp_Data* bp_data, Op* op) {
  Addr target;
  Addr addr = op->inst->addr;
  uns32 hist = bp_data->global_hist;
  uns32 cooked_hist = COOK_HIST_BITS(hist, 0);
  uns32 cooked_addr = COOK_ADDR_BITS(addr, 2);
  uns32 sel_index = cooked_hist ^ cooked_addr;
  uns8 sel_entry = bp_data->tc_selector[sel_index];

  ASSERT(bp_data->proc_id, bp_data->proc_id == op->proc_id);

  if (IBTB_HASH_TOS) {
    uns32 cooked_tos_addr;
    cooked_tos_addr = COOK_ADDR_BITS(op->recovery_info.tos_addr, 2);
    sel_index = sel_index ^ cooked_tos_addr;
    sel_entry = bp_data->tc_selector[sel_index];
  }

  ASSERT(bp_data->proc_id, sel_entry <= TC_SELECTOR_TAGGED_STRONG);

  if (sel_entry >= TC_SELECTOR_TAGGED_WEAK) {
    target = bp_ibtb_tc_tagged_pred(bp_data, op);
  } else {
    target = bp_ibtb_tc_tagless_pred(bp_data, op);
  }

  op->btb_pred.ibp_pred_global_hist = bp_data->global_hist;
  op->btb_pred.ibp_pred_tc_selector_entry = sel_entry;

  return target;
}

/**************************************************************************************/
/* bp_tc_hybrid_update: */

void bp_ibtb_tc_hybrid_update(Bp_Data* bp_data, Op* op) {
  Addr addr = op->inst->addr;
  uns32 hist = op->btb_pred_info->ibp_pred_global_hist;
  uns32 cooked_hist = COOK_HIST_BITS(hist, 0);
  uns32 cooked_addr = COOK_ADDR_BITS(addr, 2);
  uns32 sel_index = cooked_hist ^ cooked_addr;
  uns8 sel_entry = bp_data->tc_selector[sel_index];
  Flag predicted_tagged = op->btb_pred_info->ibp_pred_tc_selector_entry >= TC_SELECTOR_TAGGED_WEAK;

  ASSERT(bp_data->proc_id, bp_data->proc_id == op->proc_id);

  if (IBTB_HASH_TOS) {
    uns32 cooked_tos_addr;
    cooked_tos_addr = COOK_ADDR_BITS(op->recovery_info.tos_addr, 2);
    sel_index = sel_index ^ cooked_tos_addr;
    sel_entry = bp_data->tc_selector[sel_index];
  }

  if (op->btb_pred_info->no_target) {  // branch was not predicted at all
    // Update both predictors
    // No change to selector
    bp_ibtb_tc_tagged_update(bp_data, op);
    bp_ibtb_tc_tagless_update(bp_data, op);
    if (!op->off_path)
      STAT_EVENT(op->proc_id, TARG_HYBRID_NO_PRED);
  } else if (op->bp_pred_info->recovery_point == RECOVER_AT_DECODE) {
    // Update the predictor that made the prediction
    // Change the selector so that it does not use this predictor again
    if (predicted_tagged) {  // predicted by tagged predictor
      bp_data->tc_selector[sel_index] = SAT_DEC(sel_entry, 0);
      bp_ibtb_tc_tagged_update(bp_data, op);
      if (!op->off_path)
        STAT_EVENT(op->proc_id, TARG_HYBRID_RECOVER_AT_EXEC_TAGGED);
    } else {  // predicted by tagless predictor
      bp_data->tc_selector[sel_index] = SAT_INC(sel_entry, TC_SELECTOR_TAGGED_STRONG);
      bp_ibtb_tc_tagless_update(bp_data, op);
      if (!op->off_path)
        STAT_EVENT(op->proc_id, TARG_HYBRID_RECOVER_AT_EXEC_TAGLESS);
    }
  } else {                   // branch was correctly predicted
    if (predicted_tagged) {  // correct pred by tagged predictor
      bp_data->tc_selector[sel_index] = SAT_INC(sel_entry, TC_SELECTOR_TAGGED_STRONG);
      bp_ibtb_tc_tagged_update(bp_data, op);
      if (!op->off_path)
        STAT_EVENT(op->proc_id, TARG_HYBRID_CORRECT_TAGGED);
    } else {  // correct pred by tagless predictor
      bp_data->tc_selector[sel_index] = SAT_DEC(sel_entry, 0);
      bp_ibtb_tc_tagless_update(bp_data, op);
      if (!op->off_path)
        STAT_EVENT(op->proc_id, TARG_HYBRID_CORRECT_TAGLESS);
    }
  }
}

/**************************************************************************************/
/* bp_tc_hybrid_recover */

void bp_ibtb_tc_hybrid_recover(Bp_Data* bp_data, Recovery_Info* info) {
  DEBUG(bp_data->proc_id, "Recovering target cache history\n");
  bp_data->targ_hist = info->targ_hist;
}

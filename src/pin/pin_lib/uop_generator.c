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
 ** File         : uop_generator.c
 ** Author       : HPS Research Group
 ** Date         : 1/3/2018
 ** Description  : API to extract uops out of ctype_pin_inst_struct instances.
 ****************************************************************************************/

#include "uop_generator.h"

#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "debug/debug.param.h"
#include "debug/debug_macros.h"
#include "debug/debug_print.h"

#include "bp/bp.param.h"
#include "general.param.h"

#include "bp/bp.h"
#include "isa/isa.h"
#include "libs/cpp_hash_lib_wrapper.h"
#include "libs/hash_lib.h"

#include "ctype_pin_inst.h"
#include "math.h"
#include "statistics.h"

/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_TRACE_READ, ##args)
#define DEBUG_PRINT(proc_id, args...) fprintf(GLOBAL_DEBUG_STREAM, ##args)
#define MAX_PUP 256
/**************************************************************************************/
/* Types */

struct Trace_Uop_struct {
  Op_Type op_type;    // type of operation
  Mem_Type mem_type;  // type of memory instruction
  Cf_Type cf_type;    // type of control flow instruction
  Bar_Type bar_type;  // type of barrier caused by instruction

  uns num_dest_regs;      // number of destination registers written
  uns num_src_regs;       // number of source registers read
  uns num_agen_src_regs;  // memory address calculation read for ztrace

  uns inst_size;  // instruction size
  Addr addr;

  Reg_Info srcs[MAX_SRCS];  // source register information
  Reg_Info dests[MAX_DESTS];

  // Dynamic (Runtime) Info
  uint64_t inst_uid;
  Flag actual_taken;
  Addr va;
  uns mem_size;  // number of bytes read/written by a memory instruction
  Addr target;
  Addr npc;
  Flag bom;
  Flag eom;
  Flag exit;
  //////////////////

  uns load_seq_num;
  uns store_seq_num;
  Flag is_from_gather_scater;

  Inst_Info* info;
  Static_Inst_Info* static_inst;  // per-macro-inst static info (interned)
  Static_Op_Info* static_op;      // per-uop static info (interned)
  Flag alu_uop;
};
typedef struct Trace_Uop_struct Trace_Uop;

/**************************************************************************************/
/* Global Variables */

extern int op_type_delays[NUM_OP_TYPES];
extern uns NEW_INST_TABLE_SIZE;  // TODO: what is this?

char dbg_print_buf[1024];

Trace_Uop*** trace_uop_bulk;
Flag* bom;
Flag* eom;
Flag* fetched_instruction;
uns* num_sending_uop;
uns* num_uops;
Addr* last_ga_va;

static Inst_Info fake_nop_template;
static Inst_Info fake_jmp_template;
/* A fake op's Static_Op_Info is byte-identical for every fake op of a kind (it holds no address),
   so they all share one, built once at init. */
static Static_Op_Info fake_nop_static_op;
static Static_Op_Info fake_jmp_static_op;
static Flag fake_templates_ready = FALSE;

/**************************************************************************************/
/* Local prototypes */

static uns generate_uops(uns8 proc_id, ctype_pin_inst* pi, Trace_Uop** trace_uop);
static void convert_pinuop_to_t_uop(uns8 proc_id, ctype_pin_inst* pi, Trace_Uop** trace_uop);
static void convert_t_uop_to_info(uns8 proc_id, Trace_Uop* t_uop, Inst_Info* info);
static void populate_static_inst_info(Static_Inst_Info* si, const Inst_Info* info, const ctype_pin_inst* pi);
static void populate_static_op_info(Static_Op_Info* so, const Inst_Info* info);
static void convert_dyn_uop(uns8 proc_id, Inst_Info* info, ctype_pin_inst* pi, Trace_Uop* trace_uop, uns mem_size,
                            Flag is_last_uop);

/**************************************************************************************/

static void print_inst_fields(uns proc_id, compressed_op* cop) {
  if (DEBUG_INST_FIELDS && DEBUG_RANGE_COND(proc_id)) {
    DEBUG_PRINT(proc_id, "=========== Compressed Op Information ===============\n");
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS inst addrs fake: %lx %lx %d %d\n", cop->instruction_addr,
                cop->instruction_next_addr, cop->fake_inst, cop->fake_inst_reason);
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS op: %s\n", cop->pin_iclass);
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS size op cf fp: %d %d %d %d %d\n", cop->size, cop->op_type, cop->cf_type,
                cop->is_fp, cop->actually_taken);
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS src regs:");
    for (int i = 0; i < cop->num_src_regs; ++i) {
      DEBUG_PRINT(proc_id, "%d ", cop->src_regs[i]);
    }
    DEBUG_PRINT(proc_id, "\n");
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS dst regs:");
    for (int i = 0; i < cop->num_dst_regs; ++i) {
      DEBUG_PRINT(proc_id, "%d ", cop->dst_regs[i]);
    }
    DEBUG_PRINT(proc_id, "\n");
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS ld1 addr regs:");
    for (int i = 0; i < cop->num_ld1_addr_regs; ++i) {
      DEBUG_PRINT(proc_id, "%d ", cop->ld1_addr_regs[i]);
    }
    DEBUG_PRINT(proc_id, "\n");
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS ld2 addr regs:");
    for (int i = 0; i < cop->num_ld2_addr_regs; ++i) {
      DEBUG_PRINT(proc_id, "%d ", cop->ld2_addr_regs[i]);
    }
    DEBUG_PRINT(proc_id, "\n");
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS st addr regs:");
    for (int i = 0; i < cop->num_st_addr_regs; ++i) {
      DEBUG_PRINT(proc_id, "%d ", cop->st_addr_regs[i]);
    }
    DEBUG_PRINT(proc_id, "\n");
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS simd: %d %d %d\n", cop->is_simd, cop->is_simd ? cop->num_simd_lanes : 0,
                cop->is_simd ? cop->lane_width_bytes : 0);
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS ld addrs:");
    for (int i = 0; i < cop->num_ld; ++i) {
      DEBUG_PRINT(proc_id, "%lx ", cop->ld_vaddr[i]);
    }
    DEBUG_PRINT(proc_id, "\n");
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS st addrs:");
    for (int i = 0; i < cop->num_st; ++i) {
      DEBUG_PRINT(proc_id, "%lx ", cop->st_vaddr[i]);
    }
    DEBUG_PRINT(proc_id, "\n");
    DEBUG_PRINT(proc_id, "DEBUG_INST_FIELDS ld st size: %d %d\n", cop->ld_size, cop->st_size);
  }
}

static void print_op_fields(uns proc_id, Op* op) {
  if (DEBUG_OP_FIELDS && DEBUG_RANGE_COND(proc_id)) {
    DEBUG_PRINT(proc_id, "----- Op Information -----\n");
    DEBUG_PRINT(proc_id, "DEBUG_OP_FIELDS inst addrs fake: %llx %d %d\n", op->inst->addr, op->inst->fake_inst,
                op->inst->fake_inst_reason);
    DEBUG_PRINT(proc_id, "DEBUG_OP_FIELDS op: %s\n", disasm_op(op, FALSE));
    DEBUG_PRINT(proc_id, "DEBUG_OP_FIELDS op cf dir: %d %d %d\n", op->uop->op_type, op->uop->cf_type,
                op->oracle_info.dir);
    DEBUG_PRINT(proc_id, "DEBUG_OP_FIELDS src regs: ");
    for (int i = 0; i < op->uop->num_src_regs; ++i) {
      DEBUG_PRINT(proc_id, "%d ", op->uop->srcs[i].id);
    }
    DEBUG_PRINT(proc_id, "\n");
    DEBUG_PRINT(proc_id, "DEBUG_OP_FIELDS dst regs: ");
    for (int i = 0; i < op->uop->num_dest_regs; ++i) {
      DEBUG_PRINT(proc_id, "%d ", op->uop->dests[i].id);
    }
    DEBUG_PRINT(proc_id, "\n");
    DEBUG_PRINT(proc_id, "DEBUG_OP_FIELDS simd: %d %d %d\n", op->inst->is_simd,
                op->inst->is_simd ? op->inst->num_simd_lanes : 0, op->inst->is_simd ? op->inst->lane_width_bytes : 0);
    DEBUG_PRINT(proc_id, "DEBUG_OP_FIELDS mem_type addr mem_size: %d %llx %d\n", op->uop->mem_type, op->oracle_info.va,
                op->uop->mem_size);
  }
}

void uop_generator_init(uint32_t num_cores) {
  trace_uop_bulk = (Trace_Uop***)malloc(num_cores * sizeof(Trace_Uop**));
  for (uns ii = 0; ii < num_cores; ii++) {
    trace_uop_bulk[ii] = (Trace_Uop**)malloc(MAX_PUP * sizeof(Trace_Uop*));
    for (uns jj = 0; jj < MAX_PUP; jj++) {
      trace_uop_bulk[ii][jj] = (Trace_Uop*)malloc(sizeof(Trace_Uop));
    }
  }

  bom = (Flag*)malloc(num_cores * sizeof(Flag));
  memset(bom, 1, num_cores * sizeof(Flag));
  eom = (Flag*)malloc(num_cores * sizeof(Flag));
  memset(eom, 1, num_cores * sizeof(Flag));
  fetched_instruction = (Flag*)malloc(num_cores * sizeof(Flag));
  memset(fetched_instruction, 1, num_cores * sizeof(Flag));
  num_uops = (uns*)malloc(num_cores * sizeof(uns));
  memset(num_uops, 0, num_cores * sizeof(uns));
  num_sending_uop = (uns*)malloc(num_cores * sizeof(uns));
  memset(num_sending_uop, 0, num_cores * sizeof(uns));

  last_ga_va = (Addr*)malloc(num_cores * sizeof(Addr));

  Trace_Uop** tuop = trace_uop_bulk[0];

  ctype_pin_inst nop_pi = create_dummy_nop(0, WPNM_FAKE_NOP, DUMMY_NOP_SIZE);
  uns nop_nuop = generate_uops(0, &nop_pi, tuop);
  ASSERT(0, nop_nuop == 1);
  convert_t_uop_to_info(0, tuop[0], &fake_nop_template);
  fake_nop_template.table_info.true_op_type = nop_pi.true_op_type;
  fake_nop_template.table_info.is_simd = nop_pi.is_simd;
  strcpy(fake_nop_template.table_info.name, nop_pi.pin_iclass);
  fake_nop_template.trace_info.num_uop = 1;
  fake_nop_template.uop_seq_num = 0;
  fake_nop_template.fake_inst = TRUE;
  fake_nop_template.fake_inst_reason = WPNM_FAKE_NOP;

  ctype_pin_inst jmp_pi = create_dummy_jump(0, 0, 0);
  uns jmp_nuop = generate_uops(0, &jmp_pi, tuop);
  ASSERT(0, jmp_nuop == 1);
  convert_t_uop_to_info(0, tuop[0], &fake_jmp_template);
  fake_jmp_template.table_info.true_op_type = jmp_pi.true_op_type;
  fake_jmp_template.table_info.is_simd = jmp_pi.is_simd;
  strcpy(fake_jmp_template.table_info.name, jmp_pi.pin_iclass);
  fake_jmp_template.trace_info.num_uop = 1;
  fake_jmp_template.uop_seq_num = 0;
  fake_jmp_template.fake_inst = TRUE;
  fake_jmp_template.fake_inst_reason = WPNM_FAKE_JMP;

  populate_static_op_info(&fake_nop_static_op, &fake_nop_template);
  populate_static_op_info(&fake_jmp_static_op, &fake_jmp_template);

  fake_templates_ready = TRUE;
}

Flag uop_generator_extract_op(uns proc_id, Op* op, compressed_op* cop) {
  if (uop_generator_get_bom(proc_id)) {
    uop_generator_get_uop(proc_id, op, cop);
    print_inst_fields(proc_id, cop);
  } else {
    uop_generator_get_uop(proc_id, op, NULL);
  }

  print_op_fields(proc_id, op);

  if (uop_generator_get_eom(proc_id)) {
    return TRUE;
  }

  return FALSE;
}

#if ENABLE_GLOBAL_DEBUG_PRINT
static void init_dbg_print_buf_to_empty() {
  dbg_print_buf[0] = '\0';
}
static void append_to_dbg_print_buf(uns proc_id, char* tmp_buf) {
  const int bytes_remaining_in_buf = sizeof(dbg_print_buf) - strlen(dbg_print_buf);
  ASSERT(proc_id, bytes_remaining_in_buf > strlen(tmp_buf));
  const char* ptr = strcat(dbg_print_buf, tmp_buf);
  ASSERT(proc_id, ptr == dbg_print_buf);
}

static void add_one_addr(uns proc_id, Flag ld_addr, uns mem_op_seq, ctype_pin_inst* inst) {
  char tmp_buf[64];
  int ret;
  if (ld_addr)
    ret = sprintf(tmp_buf, "ld_vaddr[%d]:0x%s ", mem_op_seq, hexstr64s(inst->ld_vaddr[mem_op_seq]));
  else {
    ret = sprintf(tmp_buf, "st_vaddr[%d]:0x%s ", mem_op_seq, hexstr64s(inst->st_vaddr[mem_op_seq]));
  }
  ASSERT(proc_id, 0 < ret);
  append_to_dbg_print_buf(proc_id, tmp_buf);
}

static char* ctype_pin_inst_ld_and_st_addrs(uns proc_id, ctype_pin_inst* inst) {
  ASSERT(proc_id, inst);
  init_dbg_print_buf_to_empty();

  for (uns ld = 0; ld < inst->num_ld; ld++) {
    add_one_addr(proc_id, TRUE, ld, inst);
  }
  for (uns st = 0; st < inst->num_st; st++) {
    add_one_addr(proc_id, FALSE, st, inst);
  }

  return dbg_print_buf;
}

#endif

void uop_generator_get_uop(uns proc_id, Op* op, ctype_pin_inst* inst) {
  Trace_Uop* trace_uop = NULL;
  Trace_Uop** trace_uop_array;
  uns ii;
  Inst_Info* info = NULL;

  // cmp: find the correct core
  trace_uop_array = trace_uop_bulk[proc_id];

  if (bom[proc_id]) {
    ASSERT(proc_id, inst != NULL);
    convert_pinuop_to_t_uop(proc_id, inst, trace_uop_array);

    op->bom = TRUE;
    trace_uop = trace_uop_array[0];
    info = trace_uop->info;
    num_uops[proc_id] = info->trace_info.num_uop;

    num_sending_uop[proc_id] = 1;
    eom[proc_id] = trace_uop->eom;
    fetched_instruction[proc_id] = inst->fetched_instruction;

    DEBUG(proc_id,
          "read pi, addr is 0x%s next_addr: 0x%s op_type:%s num_st:%d "
          "num_ld:%d is_fp:%d cf_type:%d size:%d branch_target:%s ld_size:%d "
          "st_size:%d %s taken:%d "
          "num_uop:%d eom:%d bar:%i\n",
          hexstr64s(inst->instruction_addr), hexstr64s(inst->instruction_next_addr), Op_Type_str(inst->op_type),
          inst->num_st, inst->num_ld, inst->is_fp, inst->cf_type, inst->size, hexstr64s(inst->branch_target),
          inst->ld_size, inst->st_size, ctype_pin_inst_ld_and_st_addrs(proc_id, inst), inst->actually_taken,
          num_uops[proc_id], eom[proc_id], info->table_info.bar_type);
  } else {
    trace_uop = trace_uop_array[num_sending_uop[proc_id]];
    ASSERTM(proc_id, trace_uop, "%i\n", num_sending_uop[proc_id]);
    num_sending_uop[proc_id]++;

    info = trace_uop->info;
    ASSERTM(proc_id, info, "%i\n", num_sending_uop[proc_id]);
    eom[proc_id] = trace_uop->eom;
  }

  if (eom[proc_id]) {
    bom[proc_id] = TRUE;
  } else
    bom[proc_id] = FALSE;

  op->op_num = op_count[proc_id];
  op->inst_uid = trace_uop->inst_uid;
  op->unique_num = unique_count;
  op->unique_num_per_proc = unique_count_per_core[proc_id];
  op->proc_id = proc_id;
  op->eom = trace_uop->eom;
  op->fetched_instruction = fetched_instruction[proc_id];
  op->inst = trace_uop->static_inst;
  op->uop = trace_uop->static_op;
  op->off_path = FALSE;
  op->state = OS_FETCHED;
  op->fu_num = -1;
  op->cycles.issue_cycle = MAX_CTR;
  op->cycles.map_cycle = MAX_CTR;
  op->cycles.rdy_cycle = 1;
  op->cycles.sched_cycle = MAX_CTR;
  op->cycles.exec_cycle = MAX_CTR;
  op->cycles.dcache_cycle = MAX_CTR;
  op->cycles.done_cycle = MAX_CTR;
  op->cycles.replay_cycle = MAX_CTR;
  op->cycles.retire_cycle = MAX_CTR;
  op->replay = FALSE;
  op->exec_count = 0;
  op->in_rdy_list = FALSE;
  op->in_node_list = FALSE;
  op->bp_pred_l0.recovery_sch = FALSE;
  op->bp_pred_main.recovery_sch = FALSE;

  op->req = NULL;
  op->marked = FALSE;

  op->parent_FT = NULL;
  op->parent_FT_off_path = NULL;

  /* pipelined scheduler fields */
  op->chkpt_num = MAX_CTR;
  // op->row_num = MAX_CTR;
  op->node_id = MAX_CTR;
  op->engine_info.l1_miss = FALSE;
  op->engine_info.l1_miss_satisfied = FALSE;
  op->engine_info.dep_on_l1_miss = FALSE;
  op->engine_info.was_dep_on_l1_miss = FALSE;

  /* multi path support */

  /* execute op */
  for (uns ii = 0; ii < info->table_info.num_src_regs; ii++) {
    op->src_val[ii] = trace_uop->srcs[ii].val;
  }
  for (uns ii = 0; ii < info->table_info.num_dest_regs; ii++) {
    op->dst_val[ii] = trace_uop->dests[ii].val;
  }

  if (op->uop->op_type == OP_CF) {
    if (op->uop->cf_type == CF_CBR || op->uop->cf_type == CF_REP) {
      op->oracle_info.dir = (trace_uop->actual_taken == 0) ? NOT_TAKEN : TAKEN;
    } else {
      /* assume that all CFs besides CBR are actually always taken. This fixes the
       * issue where fall-through PC == target. */
      op->oracle_info.dir = TAKEN;
    }
  } else
    op->oracle_info.dir = NOT_TAKEN;

  if ((op->uop->cf_type == CF_ICALL) || (op->uop->cf_type == CF_IBR) || (op->uop->cf_type == CF_ICO))
    op->oracle_info.dir = 1;  // FIXME Hack!! because of StringMOV

  /* removing proc_id from target before compare with zero */
  op->oracle_info.target = convert_to_cmp_addr(0, trace_uop->target) ? trace_uop->target : trace_uop->npc;
  op->oracle_info.va = trace_uop->va;
  op->oracle_info.npc = trace_uop->npc;
  if (op->proc_id)
    ASSERT(op->proc_id, op->oracle_info.npc);
  if (op->uop->cf_type == CF_BR || op->uop->cf_type == CF_CALL)
    ASSERT(op->proc_id, op->oracle_info.target == op->oracle_info.npc);
  op->oracle_info.mem_size = trace_uop->mem_size;
  // because of repeat move mem size is dynamic info  WRONG!!!!
  // op->uop->mem_size = trace_uop->mem_size;

  if (op->uop->mem_type && !(op->oracle_info.va)) {
    // QUESTION why? //TODO: Really why?
    op->oracle_info.va = last_ga_va[proc_id];
  } else if (op->oracle_info.va)
    last_ga_va[proc_id] = op->oracle_info.va;

  if ((op->eom && trace_read_done[proc_id]) || trace_uop->exit) {
    op->exit = TRUE;
  } else {
    op->exit = FALSE;
  }

  DEBUG(proc_id,
        "op_num:%s unique_num:%s pc:0x%s npc:0x%s va:0x%s mem_type:%d "
        "mem_size:%d cf_type:%d oracle_target:%s dir:%d\n",
        unsstr64(op->op_num), unsstr64(op->unique_num), hexstr64s(op->inst->addr), hexstr64s(op->oracle_info.npc),
        hexstr64s(op->oracle_info.va), op->uop->mem_type, op->oracle_info.mem_size, op->uop->cf_type,
        hexstr64s(op->oracle_info.target), op->oracle_info.dir);

  for (ii = 0; ii < op->uop->num_src_regs; ii++) {
    DEBUG(proc_id, "op_num:%s unique_num:%s pc:0x%s npc:0x%s, src(%d/%d):%s \n", unsstr64(op->op_num),
          unsstr64(op->unique_num), hexstr64s(op->inst->addr), hexstr64s(op->oracle_info.npc), ii + 1,
          op->uop->num_src_regs, disasm_reg(op->uop->srcs[ii].id));
  }

  for (ii = 0; ii < op->uop->num_dest_regs; ii++) {
    DEBUG(proc_id, "op_num:%s unique_num:%s pc:0x%s npc:0x%s, dest(%d/%d):%s \n", unsstr64(op->op_num),
          unsstr64(op->unique_num), hexstr64s(op->inst->addr), hexstr64s(op->oracle_info.npc), ii + 1,
          op->uop->num_dest_regs, disasm_reg(op->uop->dests[ii].id));
  }
}

Flag uop_generator_get_bom(uns proc_id) {
  return bom[proc_id];
}

Flag uop_generator_get_eom(uns proc_id) {
  return eom[proc_id];
}

void convert_t_uop_to_info(uns8 proc_id, Trace_Uop* t_uop, Inst_Info* info) {
  int ii;

  // build info
  // we can optimize to build this info only once
  // FIXME. at least a hash function based on the same table info.
  ASSERT(proc_id, info);

  info->table_info.op_type = t_uop->op_type;
  info->table_info.mem_type = t_uop->mem_type;
  info->table_info.cf_type = t_uop->cf_type;
  info->table_info.bar_type = t_uop->bar_type;
  info->table_info.num_dest_regs = t_uop->num_dest_regs;
  info->table_info.num_src_regs = t_uop->num_src_regs;
  info->table_info.mem_size = t_uop->mem_size;  // IGNORED FOR REP STRING instructions

  info->table_info.type = 0;        /* scarab internals, ignore */
  info->table_info.mask = 0;        /* scarab internals, ignore */
  info->table_info.dec_func = NULL; /* FIXME */
  info->table_info.src_func = NULL; /* FIXME */
  info->table_info.sim_func = NULL; /* FIXME */
  info->table_info.qualifiers = 0;  /* FIXME */

  /* op->inst_info */
  info->addr = t_uop->addr;
  info->trace_info.inst_size = (t_uop->inst_size);  // FIXME

  for (ii = 0; ii < info->table_info.num_src_regs; ii++) {
    info->srcs[ii].type = INT_REG;
    info->srcs[ii].id = t_uop->srcs[ii].id;
    info->srcs[ii].reg = t_uop->srcs[ii].reg;
    /* If an op sources a predicate, it is always the last source - here we
     * avoid sourcing the last source */
  }

  /* only one destination - temporary that is going to be read by the second t_uop */
  for (ii = 0; ii < info->table_info.num_dest_regs; ii++) {
    info->dests[ii].type = INT_REG;
    info->dests[ii].id = t_uop->dests[ii].id;
    info->dests[ii].reg = t_uop->dests[ii].reg;
  }

  info->latency = op_type_delays[t_uop->op_type];
  if (info->latency == 0)
    info->latency = 1; /* insure latency is not 0 */

  info->trace_info.load_seq_num = t_uop->load_seq_num;
  info->trace_info.store_seq_num = t_uop->store_seq_num;

  info->trigger_op_fetched_hook = FALSE; /* FIXME */
  info->extra_ld_latency = 0;
}

static void clear_t_uop(Trace_Uop* uop) {
  memset(uop, 0, sizeof(Trace_Uop));
}

static Flag is_reg_already_added(Reg_Id reg, Reg_Info* reg_array, uns num_regs) {
  for (uns i = 0; i < num_regs; ++i) {
    if (reg_array[i].id == reg)
      return 1;
  }
  return 0;
}

static void add_t_uop_src_reg(Trace_Uop* uop, Reg_Id reg) {
  if (is_reg_already_added(reg, uop->srcs, uop->num_src_regs))
    return;

  ASSERT(0, uop->num_src_regs < MAX_SRCS);
  uop->srcs[uop->num_src_regs].type = 0;
  uop->srcs[uop->num_src_regs].id = reg;
  uop->srcs[uop->num_src_regs].reg = reg;
  uop->num_src_regs++;
}

static void add_t_uop_dest_reg(Trace_Uop* uop, Reg_Id reg) {
  if (is_reg_already_added(reg, uop->dests, uop->num_dest_regs))
    return;

  ASSERT(0, uop->num_dest_regs < MAX_DESTS);
  uop->dests[uop->num_dest_regs].type = 0;
  uop->dests[uop->num_dest_regs].id = reg;
  uop->dests[uop->num_dest_regs].reg = reg;
  uop->num_dest_regs++;
}

static Flag is_stack_reg(Reg_Id reg) {
  return reg == REG_RSP || reg == REG_SS;
}

static void add_rep_uops(ctype_pin_inst* pi, Trace_Uop** trace_uop, uns* idx) {
  Flag add_rsi_add = FALSE;
  Flag add_rdi_add = FALSE;

  for (int i = 0; i < pi->num_ld1_addr_regs; ++i) {
    if (pi->ld1_addr_regs[i] == REG_RSI)
      add_rsi_add = TRUE;
    if (pi->ld1_addr_regs[i] == REG_RDI)
      add_rdi_add = TRUE;
  }
  for (int i = 0; i < pi->num_ld2_addr_regs; ++i) {
    if (pi->ld2_addr_regs[i] == REG_RSI)
      add_rsi_add = TRUE;
    if (pi->ld2_addr_regs[i] == REG_RDI)
      add_rdi_add = TRUE;
  }
  for (int i = 0; i < pi->num_st_addr_regs; ++i) {
    if (pi->st_addr_regs[i] == REG_RSI)
      add_rsi_add = TRUE;
    if (pi->st_addr_regs[i] == REG_RDI)
      add_rdi_add = TRUE;
  }

  if (add_rsi_add) {
    Trace_Uop* uop = trace_uop[*idx];
    clear_t_uop(uop);
    uop->op_type = OP_IADD;
    uop->alu_uop = TRUE;
    add_t_uop_src_reg(uop, REG_RSI);
    add_t_uop_dest_reg(uop, REG_RSI);
    *idx = *idx + 1;
  }

  if (add_rdi_add) {
    Trace_Uop* uop = trace_uop[*idx];
    clear_t_uop(uop);
    uop->op_type = OP_IADD;
    uop->alu_uop = TRUE;
    add_t_uop_src_reg(uop, REG_RDI);
    add_t_uop_dest_reg(uop, REG_RDI);
    *idx = *idx + 1;
  }

  if (pi->is_repeat) {
    // Decrement RCX micro-op
    Trace_Uop* uop = trace_uop[*idx];
    clear_t_uop(uop);
    uop->op_type = OP_IADD;
    uop->alu_uop = TRUE;
    add_t_uop_src_reg(uop, REG_RCX);
    add_t_uop_dest_reg(uop, REG_RCX);
    *idx = *idx + 1;
    // Control flow micro-op
    uop = trace_uop[*idx];
    clear_t_uop(uop);
    uop->op_type = OP_CF;
    uop->cf_type = CF_REP;
    STAT_EVENT(0, REP_FIRST);
    add_t_uop_src_reg(uop, REG_ZPS);
    *idx = *idx + 1;
  }
}

/* The uop decomposition depends on the number of memory operations performed by an instance,
 * which is not a property of the encoding. For example, a REP string instruction executed with
 * rcx == 0 has no memory accesses, while the same instruction at the same PC with rcx > 0 does.
 * Since generate_uops() runs only once per interning key, the operation counts must be part of it;
 * otherwise, the first decoded instance determines the uop shape for all subsequent instances. */
static inline uint16_t mem_shape_of(const ctype_pin_inst* pi) {
  return (uint16_t)((pi->num_ld << 8) | pi->num_st);
}

static Flag use_ld1_addr_regs(const uns8 proc_id, const compressed_op* pi, const uns load_seq_num) {
  if ((0 == load_seq_num) || pi->is_gather_scatter)
    return TRUE;
  else {
    ASSERT(proc_id, 1 == load_seq_num);
    return FALSE;
  }
}

static uns generate_uops(uns8 proc_id, ctype_pin_inst* pi, Trace_Uop** trace_uop) {
  /* Generating microinstructions for the trace instruction. The
   * general sequence of every instruction other than REP insts is:
   *     load_1, load_2, operate, store, control                 */

  uns idx = 0;
  Flag has_load = pi->num_ld > 0;
  Flag has_push = pi->has_push;
  Flag has_pop = pi->has_pop;
  Flag has_store = pi->num_st > 0;
  Flag has_control = pi->cf_type != NOT_CF;
  Flag has_alu = !(pi->is_move && (has_load || has_store)) &&  // not a simple LD/ST move
                 ((!has_control && !has_load && !has_store)    // not memory, not control, must be operate
                  || has_push || has_pop                       // need ALU for stack address generation
                  || pi->num_dst_regs > 0                      // if it writes to a registers, must be operate
                  || (has_load && has_store)               // must be read-modify-write operate (e.g. add $1, [%eax])
                  || (pi->op_type >= OP_PIPELINED_FAST &&  // special instructions always
                      pi->op_type <= OP_NOTPIPELINED_VERY_SLOW));  // need an alu uop

  /* both REP MOVS and REP STOS are is_rep_st, meaning alu uop is independent of mem uops */
  Flag is_rep_st = (pi->is_string) && has_store;

  /* Loads */
  for (uns i = 0; i < pi->num_ld; ++i) {
    Trace_Uop* uop = trace_uop[idx];
    idx += 1;
    clear_t_uop(uop);

    uop->mem_type = (pi->is_prefetch) ? MEM_PF : MEM_LD;
    uop->op_type = (pi->is_fp || pi->is_simd) ? OP_FLD : OP_ILD;
    uop->mem_size = pi->ld_size;
    uop->load_seq_num = i;

    for (uns j = 0; j < pi->num_ld1_addr_regs; ++j) {
      Reg_Id reg = (use_ld1_addr_regs(proc_id, pi, uop->load_seq_num) ? pi->ld1_addr_regs : pi->ld2_addr_regs)[j];
      add_t_uop_src_reg(uop, reg);
    }

    if ((has_alu && !has_push && !has_pop) || has_store || has_control) {  // load result used further down
      Reg_Id dest_reg = REG_TMP0 + i;
      ASSERT(proc_id, dest_reg <= REG_OTHER);
      add_t_uop_dest_reg(uop, dest_reg);
    } else {
      for (uns j = 0; j < pi->num_dst_regs; ++j) {
        Reg_Id dest_reg = pi->dst_regs[j];
        ASSERT(proc_id, dest_reg <= REG_OTHER);
        add_t_uop_dest_reg(uop, dest_reg);
      }
    }
  }

  /* Operate */
  if (has_alu) {
    Trace_Uop* uop = trace_uop[idx];
    idx += 1;
    clear_t_uop(uop);

    ASSERT(proc_id, pi->op_type < NUM_OP_TYPES);
    uop->op_type = pi->op_type;

    ASSERT(proc_id, uop->op_type != OP_INV);
    uop->alu_uop = TRUE;

    if (has_push || has_pop) {  // ALU op only changes the stack pointer in stack insts
      add_t_uop_src_reg(uop, REG_RSP);
      add_t_uop_dest_reg(uop, REG_RSP);
    } else if (!is_rep_st) {
      for (uns i = 0; i < pi->num_ld; ++i) {
        add_t_uop_src_reg(uop, REG_TMP0 + i);
      }
    }

    for (uns j = 0; j < pi->num_src_regs; ++j) {
      Reg_Id reg = pi->src_regs[j];

      if (has_push || has_pop) {
        if (is_stack_reg(reg))
          add_t_uop_src_reg(uop, reg);
      } else {
        add_t_uop_src_reg(uop, reg);
      }
    }

    if ((!has_push && !has_pop) && (has_store || has_control) && !is_rep_st) {
      add_t_uop_dest_reg(uop, REG_TMP2);
    }

    if (!has_push && !has_pop) {
      for (uns j = 0; j < pi->num_dst_regs; ++j) {
        Reg_Id reg = pi->dst_regs[j];
        add_t_uop_dest_reg(uop, reg);
      }
    }
  }

  /* Store */
  if (has_store) {
    for (uns i = 0; i < pi->num_st; ++i) {
      Trace_Uop* uop = trace_uop[idx];
      idx += 1;
      clear_t_uop(uop);

      uop->mem_type = MEM_ST;
      uop->op_type = (pi->is_fp || pi->is_simd) ? OP_FST : OP_IST;
      uop->mem_size = pi->st_size;
      uop->store_seq_num = i;

      if (pi->is_call) {
        // only storing (invisible) EIP on calls
      } else if (!has_alu || has_pop || has_push || is_rep_st) {
        for (uns i = 0; i < pi->num_ld; ++i) {
          add_t_uop_src_reg(uop, REG_TMP0 + i);
        }
      } else if (has_alu && !has_push && !has_pop) {
        add_t_uop_src_reg(uop, REG_TMP2);
      }

      for (uns j = 0; j < pi->num_st_addr_regs; ++j) {
        Reg_Id reg = pi->st_addr_regs[j];
        add_t_uop_src_reg(uop, reg);
      }

      for (uns j = 0; j < pi->num_src_regs; ++j) {
        Reg_Id reg = pi->src_regs[j];

        if (!has_load && (!has_alu || has_push))
          add_t_uop_src_reg(uop, reg);
      }
      // store has no dest regs
    }
  }

  /* Control */
  if (has_control) {
    Trace_Uop* uop = trace_uop[idx];
    idx += 1;
    clear_t_uop(uop);

    uop->cf_type = pi->cf_type;
    uop->op_type = OP_CF;

    if (has_load) {
      for (uns i = 0; i < pi->num_ld; ++i) {
        add_t_uop_src_reg(uop, REG_TMP0 + i);
      }
    } else {
      for (uns j = 0; j < pi->num_src_regs; ++j) {
        Reg_Id reg = pi->src_regs[j];

        // When calling/returning, the control op does not use the stack pointer
        if (!is_stack_reg(reg) || !(has_pop || has_push))
          add_t_uop_src_reg(uop, reg);
      }
    }
  }

  if (pi->is_string) {
    add_rep_uops(pi, trace_uop, &idx);
  }

  // make a nop if no ops were generated
  if (idx == 0) {
    Trace_Uop* uop = trace_uop[idx];
    idx += 1;
    clear_t_uop(uop);

    uop->op_type = OP_NOP;
    STAT_EVENT(0, STATIC_PIN_NOP);
  }

  return idx;
}

// Reusable per-proc scratch for fake ops. The fake Inst_Info is only read during decode (to
// populate op->inst / op->uop and read num_uop/bar_type here); the pipeline never touches it, so a
// single reused struct avoids leaking one heap Inst_Info per fake op (op->inst_info no longer holds
// and frees it after the split).
static Inst_Info fake_inst_info_scratch[MAX_NUM_PROCS];

void convert_pinuop_to_t_uop(uns8 proc_id, ctype_pin_inst* pi, Trace_Uop** trace_uop) {
  Flag new_entry = FALSE;
  Inst_Info* info;
  int ii;
  int num_uop = 0;

  if (pi->is_string) {
    pi->branch_target = pi->instruction_addr;
    pi->actually_taken = (pi->branch_target == pi->instruction_next_addr);
  }

  pi->instruction_addr = convert_to_cmp_addr(proc_id, pi->instruction_addr);
  pi->instruction_next_addr = convert_to_cmp_addr(proc_id, pi->instruction_next_addr);
  pi->branch_target = convert_to_cmp_addr(proc_id, pi->branch_target);
  for (uint ld = 0; ld < pi->num_ld; ld++) {
    pi->ld_vaddr[ld] = convert_to_cmp_addr(proc_id, pi->ld_vaddr[ld]);
  }
  for (uint st = 0; st < pi->num_st; st++) {
    pi->st_vaddr[st] = convert_to_cmp_addr(proc_id, pi->st_vaddr[st]);
  }

  if (pi->fake_inst) {
    ASSERT(proc_id, fake_templates_ready);
    info = &fake_inst_info_scratch[proc_id];
    if (pi->fake_inst_reason == WPNM_FAKE_JMP) {
      *info = fake_jmp_template;
    } else {
      *info = fake_nop_template;
    }
    info->addr = pi->instruction_addr;
    info->trace_info.inst_size = pi->size;
    info->fake_inst_reason = pi->fake_inst_reason;

    num_uop = 1;
    trace_uop[0]->info = info;
    convert_dyn_uop(proc_id, info, pi, trace_uop[0], info->table_info.mem_size, TRUE);
  } else {
    const uint16_t mem_shape = mem_shape_of(pi);
    info = cpp_hash_table_access_create(proc_id, pi->instruction_addr, pi->inst_binary_lsb, pi->inst_binary_msb, 0,
                                        mem_shape, &new_entry);
    info->fake_inst = FALSE;
    info->fake_inst_reason = WPNM_NOT_IN_WPNM;

    Flag need_to_gen_uops = new_entry || pi->is_gather_scatter;

    if (need_to_gen_uops) {
      num_uop = generate_uops(proc_id, pi, trace_uop);
      ASSERT(proc_id, num_uop > 0);

      info->trace_info.num_uop = num_uop;

      for (ii = 0; ii < num_uop; ii++) {
        if (ii > 0) {
          info = cpp_hash_table_access_create(proc_id, pi->instruction_addr, pi->inst_binary_lsb, pi->inst_binary_msb,
                                              ii, mem_shape, &new_entry);
          info->fake_inst = FALSE;
          info->fake_inst_reason = WPNM_NOT_IN_WPNM;
        }
        need_to_gen_uops = new_entry || pi->is_gather_scatter;
        ASSERT(proc_id, need_to_gen_uops);

        trace_uop[ii]->addr = pi->instruction_addr;
        trace_uop[ii]->inst_size = pi->size;

        if (ii == (num_uop - 1)) {
          if (pi->is_ifetch_barrier && !IGNORE_BAR_FETCH) {
            trace_uop[num_uop - 1]->bar_type = BAR_FETCH;
          }
        }

        convert_t_uop_to_info(proc_id, trace_uop[ii], info);
        trace_uop[ii]->info = info;

        info->table_info.true_op_type = pi->true_op_type;
        trace_uop[ii]->info->table_info.is_simd = pi->is_simd;
        trace_uop[ii]->info->uop_seq_num = ii;
        strcpy(trace_uop[ii]->info->table_info.name, pi->pin_iclass);
        if (trace_uop[ii]->alu_uop) {
          trace_uop[ii]->info->table_info.num_simd_lanes = pi->num_simd_lanes;
          trace_uop[ii]->info->table_info.lane_width_bytes = pi->lane_width_bytes;
        }
        trace_uop[ii]->info->trace_info.is_gather_scatter = pi->is_gather_scatter;

        ASSERT(proc_id, info->trace_info.inst_size == pi->size);

        Flag is_last_uop = (ii == (num_uop - 1));
        convert_dyn_uop(proc_id, info, pi, trace_uop[ii], info->table_info.mem_size, is_last_uop);
      }
    } else {
      num_uop = info->trace_info.num_uop;
      ASSERT(proc_id, !(pi->is_gather_scatter));

      for (ii = 0; ii < num_uop; ii++) {
        if (ii > 0) {
          info = cpp_hash_table_access_create(proc_id, pi->instruction_addr, pi->inst_binary_lsb, pi->inst_binary_msb,
                                              ii, mem_shape, &new_entry);
        }
        ASSERT(proc_id, !new_entry);

        trace_uop[ii]->info = info;
        trace_uop[ii]->eom = FALSE;
        ASSERT(proc_id, info->addr == pi->instruction_addr);
        ASSERT(proc_id, info->trace_info.inst_size == pi->size);

        Flag is_last_uop = (ii == (num_uop - 1));
        convert_dyn_uop(proc_id, info, pi, trace_uop[ii], info->table_info.mem_size, is_last_uop);
      }
    }
  }

  ASSERT(proc_id, num_uop > 0);
  trace_uop[num_uop - 1]->eom = TRUE;

  trace_uop[num_uop - 1]->npc = pi->instruction_next_addr;

  // Static_Inst_Info keeps a fixed-size uops[] array; bump STATIC_INST_MAX_UOPS if this ever fires.
  ASSERTM(proc_id, num_uop <= STATIC_INST_MAX_UOPS, "macro at 0x%llx cracks into %d uops (> %d)\n",
          (unsigned long long)pi->instruction_addr, num_uop, STATIC_INST_MAX_UOPS);

  // Intern (or, for fake ops, allocate) the split static structs and attach each uop to its
  // Trace_Uop. Real ops dedup the per-macro struct by {addr, binary}; fake ops mirror the calloc'd
  // Inst_Info above. si->uops[ii] is set right where the uop is first populated (so_new / fresh
  // fake), so the array is filled exactly once; has_load/has_store/has_cf derive from it on demand
  // (static_inst_has_*()).
  for (ii = 0; ii < num_uop; ii++) {
    Static_Inst_Info* si;
    Static_Op_Info* so;
    if (pi->fake_inst) {
      si = alloc_fake_static_inst();
      populate_static_inst_info(si, trace_uop[ii]->info, pi);
      // one shared Static_Op_Info per fake-op kind; uops[] beyond num_uop stays undefined, as the
      // struct's contract allows
      so = (pi->fake_inst_reason == WPNM_FAKE_JMP) ? &fake_jmp_static_op : &fake_nop_static_op;
      si->uops[ii] = so;
    } else {
      unsigned char si_new = 0, so_new = 0;
      si = cpp_static_inst_access_create(proc_id, pi->instruction_addr, pi->inst_binary_lsb, pi->inst_binary_msb,
                                         mem_shape_of(pi), &si_new);
      so = cpp_static_op_access_create(proc_id, pi->instruction_addr, pi->inst_binary_lsb, pi->inst_binary_msb, ii,
                                       mem_shape_of(pi), &so_new);
      if (si_new)
        populate_static_inst_info(si, trace_uop[ii]->info, pi);
      if (so_new) {
        populate_static_op_info(so, trace_uop[ii]->info);
        si->uops[ii] = so;
      }
    }
    trace_uop[ii]->static_inst = si;
    trace_uop[ii]->static_op = so;
  }
}

// Fill the shared per-macro-instruction static struct from the built Inst_Info + pi.
static void populate_static_inst_info(Static_Inst_Info* si, const Inst_Info* info, const ctype_pin_inst* pi) {
  si->addr = info->addr;
  si->opcode_lsb = pi->inst_binary_lsb;
  si->opcode_msb = pi->inst_binary_msb;
  si->inst_size = info->trace_info.inst_size;
  si->num_uop = info->trace_info.num_uop;
  si->branch_target = pi->branch_target;
  si->true_op_type = info->table_info.true_op_type;
  memcpy(si->name, info->table_info.name, sizeof(si->name));
  si->is_simd = info->table_info.is_simd;
  si->num_simd_lanes = info->table_info.num_simd_lanes;
  si->lane_width_bytes = info->table_info.lane_width_bytes;
  si->is_gather_scatter = info->trace_info.is_gather_scatter;
  si->fake_inst = info->fake_inst;
  si->fake_inst_reason = info->fake_inst_reason;
  si->type = info->table_info.type;
  si->qualifiers = info->table_info.qualifiers;
}

// Fill the per-uop static struct from the built Inst_Info (reg values stay off it).
static void populate_static_op_info(Static_Op_Info* so, const Inst_Info* info) {
  so->uop_seq_num = info->uop_seq_num;
  so->op_type = info->table_info.op_type;
  so->mem_type = info->table_info.mem_type;
  so->cf_type = info->table_info.cf_type;
  so->bar_type = info->table_info.bar_type;
  so->num_src_regs = info->table_info.num_src_regs;
  so->num_dest_regs = info->table_info.num_dest_regs;
  for (uns i = 0; i < info->table_info.num_src_regs; i++) {
    so->srcs[i].reg = info->srcs[i].reg;
    so->srcs[i].type = info->srcs[i].type;
    so->srcs[i].id = info->srcs[i].id;
  }
  for (uns i = 0; i < info->table_info.num_dest_regs; i++) {
    so->dests[i].reg = info->dests[i].reg;
    so->dests[i].type = info->dests[i].type;
    so->dests[i].id = info->dests[i].id;
  }
  so->mem_size = info->table_info.mem_size;
  so->latency = info->latency;
  so->extra_ld_latency = info->extra_ld_latency;
  so->load_seq_num = info->trace_info.load_seq_num;
  so->store_seq_num = info->trace_info.store_seq_num;
  so->trigger_op_fetched_hook = info->trigger_op_fetched_hook;
}

void convert_dyn_uop(uns8 proc_id, Inst_Info* info, ctype_pin_inst* pi, Trace_Uop* trace_uop, uns mem_size,
                     Flag is_last_uop) {
  trace_uop->inst_uid = pi->inst_uid;
  trace_uop->va = 0;
  trace_uop->mem_size = 0;

  for (uns ii = 0; ii < info->table_info.num_src_regs; ii++) {
    trace_uop->srcs[ii].val = 0;
    for (uns j = 0; j < pi->num_src_regs; j++) {
      if (pi->src_regs[j] == info->srcs[ii].id) {
        trace_uop->srcs[ii].val = pi->srcs[j].val;
        break;
      }
    }
  }
  for (uns ii = 0; ii < info->table_info.num_dest_regs; ii++) {
    trace_uop->dests[ii].val = 0;
    trace_uop->dests[ii].val_valid = FALSE;
    // A load's produced value comes from ld_data (MEM_LD block below), not the architectural
    // dst-reg match; skip it here so the destination value is written exactly once.
    if (info->table_info.mem_type == MEM_LD)
      continue;
    for (uns j = 0; j < pi->num_dst_regs; j++) {
      if (pi->dst_regs[j] == info->dests[ii].id) {
        ASSERT(proc_id, !trace_uop->dests[ii].val_valid);  // a destination value is written exactly once
        trace_uop->dests[ii].val = pi->dests[j].val;
        trace_uop->dests[ii].val_valid = TRUE;
        break;
      }
    }
  }

  if (info->table_info.cf_type) {
    trace_uop->actual_taken = pi->actually_taken;
    trace_uop->target = pi->branch_target;  // FIXME
  } else if (info->table_info.mem_type) {
    if (info->table_info.mem_type == MEM_ST) {
      trace_uop->store_seq_num = info->trace_info.store_seq_num;
      ASSERT(proc_id, trace_uop->store_seq_num < MAX_ST_NUM);
      ASSERT(proc_id, trace_uop->store_seq_num < pi->num_st);
      trace_uop->va = pi->st_vaddr[trace_uop->store_seq_num];
      DEBUG(proc_id,
            "Generating a store: inst @%llx opcode: %s num_ld: %i "
            "num_st: %u va: 0x%s store size: %u\n",
            (long long unsigned int)pi->instruction_addr, Op_Type_str(pi->op_type), pi->num_ld, pi->num_st,
            hexstr64s(trace_uop->va), pi->st_size);
      trace_uop->mem_size = mem_size;
    } else if (info->table_info.mem_type == MEM_LD) {
      trace_uop->load_seq_num = info->trace_info.load_seq_num;
      ASSERT(proc_id, trace_uop->load_seq_num < MAX_LD_NUM);
      trace_uop->va = pi->ld_vaddr[trace_uop->load_seq_num];
      // The loaded value is the memory data (captured at IPOINT_BEFORE). The dst-reg match above
      // deliberately skips load uops, so the value is written here exactly once -- val_valid must
      // be false (a cracked load-op writes the internal REG_TMP0, which PIN has no arch dest for).
      ASSERT(proc_id, trace_uop->load_seq_num < pi->num_ld);
      const uns64 ld_val = pi->ld_data[trace_uop->load_seq_num];
      for (uns ii = 0; ii < info->table_info.num_dest_regs; ii++) {
        ASSERT(proc_id, !trace_uop->dests[ii].val_valid);  // a destination value is written exactly once
        trace_uop->dests[ii].val = ld_val;
        trace_uop->dests[ii].val_valid = TRUE;
      }
      DEBUG(proc_id,
            "Generating a load: inst @%llx opcode: %s num_ld: %i "
            "num_st: %u va: 0x%s load size: %u\n",
            (long long unsigned int)pi->instruction_addr, Op_Type_str(pi->op_type), pi->num_ld, pi->num_st,
            hexstr64s(trace_uop->va), pi->ld_size);
      trace_uop->mem_size = mem_size;
    }
  }

  trace_uop->exit = is_last_uop ? pi->exit : 0;

  trace_uop->npc = trace_uop->info->addr;
}

void uop_generator_recover(uns8 proc_id) {
  bom[proc_id] = 1;
}

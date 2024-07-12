/* Copyright 2020 University of California Santa Cruz
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
 * File         : frontend/pt_memtrace/memtrace_fe.cc
 * Author       : Heiner Litz
 * Date         : 05/15/2020
 * Description  : Frontend to simulate traces in memtrace format
 ***************************************************************************************/

extern "C" {
#include "debug/debug.param.h"
#include "debug/debug_macros.h"
#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"
}

#include "bp/bp.h"
#include "bp/bp.param.h"
#include "ctype_pin_inst.h"
#include "frontend/pt_memtrace/memtrace_fe.h"
#include "isa/isa.h"
#include "pin/pin_lib/uop_generator.h"
#include "pin/pin_lib/x86_decoder.h"
#include "pin/pin_lib/gather_scatter_addresses.h"
#include "statistics.h"

#define DR_DO_NOT_DEFINE_int64

#include "frontend/pt_memtrace/memtrace_trace_reader_memtrace.h"
#include <unordered_map>
/**************************************************************************************/
/* Global Variables */

static char*    trace_files[MAX_NUM_PROCS];
TraceReader*    trace_readers[MAX_NUM_PROCS];
//TODO: Make per proc?
uint64_t        ins_id    = 0;
uint64_t        ins_id_fetched = 0;
uint64_t        prior_tid = 0;
uint64_t        prior_pid = 0;
uint64_t        rdptr = 0;
uint64_t        wrptr = 0;

extern scatter_info_map                          scatter_info_storage;

Flag roi_dump_began = FALSE;
Counter roi_dump_ID = 0;
std::vector<ctype_pin_inst> circ_buf;
std::unordered_map<Addr, Counter> buf_map;
const int CLINE = ~0x3F;

/**************************************************************************************/
/* Private Functions */
int memtrace_trace_read_internal(int proc_id, ctype_pin_inst* next_onpath_pi);
void buf_map_insert();
void buf_map_remove();

void fill_in_dynamic_info(ctype_pin_inst* info, const InstInfo* insi) {
  uint8_t ld = 0;
  uint8_t st = 0;

  // Note: should be overwritten for a taken control flow instruction
  info->instruction_addr      = insi->pc;
  info->instruction_next_addr = insi->target;
  info->actually_taken        = insi->taken;
  info->branch_target         = insi->target;
  info->inst_uid              = ins_id;
  info->last_inst_from_trace  = insi->last_inst_from_trace;
  info->fetched_instruction   = insi->fetched_instruction;

#ifdef PRINT_INSTRUCTION_INFO
  std::cout << std::hex << info->instruction_addr << " Next "
            << info->instruction_next_addr << " size " << (uint32_t)info->size
            << " taken " << (uint32_t)info->actually_taken << " target "
            << info->branch_target << " pid " << insi->pid << " tid "
            << insi->tid << " asm "
            << std::string(
                 xed_iclass_enum_t2str(xed_decoded_inst_get_iclass(insi->ins)))
            << " uid " << std::dec << info->inst_uid << std::endl;
#endif

  if(xed_decoded_inst_get_iclass(insi->ins) == XED_ICLASS_RET_FAR ||
     xed_decoded_inst_get_iclass(insi->ins) == XED_ICLASS_RET_NEAR)
    info->actually_taken = 1;

  for(uint8_t op = 0;
      op < xed_decoded_inst_number_of_memory_operands(insi->ins); op++) {
    // predicated true ld/st are handled just as regular ld/st
    if(xed_decoded_inst_mem_read(insi->ins, op) && !insi->mem_used[op]) {
      // Handle predicated stores specially?
      info->ld_vaddr[ld++] = insi->mem_addr[op];
    } else if(xed_decoded_inst_mem_read(insi->ins, op)) {
      info->ld_vaddr[ld++] = insi->mem_addr[op];
    }
    if(xed_decoded_inst_mem_written(insi->ins, op) && !insi->mem_used[op]) {
      // Handle predicated stores specially?
      info->st_vaddr[st++] = insi->mem_addr[op];
    } else if(xed_decoded_inst_mem_written(insi->ins, op)) {
      info->st_vaddr[st++] = insi->mem_addr[op];
    }
  }
}

int ffwd(const xed_decoded_inst_t* ins) {
  if(!FAST_FORWARD) {
    return 0;
  }
  if(XED_INS_Opcode(ins) == XED_ICLASS_XCHG &&
     XED_INS_OperandReg(ins, 0) == XED_REG_RCX &&
     XED_INS_OperandReg(ins, 1) == XED_REG_RCX) {
    return 0;
  }
  if((USE_FETCHED_COUNT ? ins_id_fetched : ins_id) == FAST_FORWARD_TRACE_INS) {
    return 0;
  }
  return 1;
}

int roi(const xed_decoded_inst_t* ins) {
  if(XED_INS_Opcode(ins) == XED_ICLASS_XCHG &&
     XED_INS_OperandReg(ins, 0) == XED_REG_RCX &&
     XED_INS_OperandReg(ins, 1) == XED_REG_RCX) {
    return 1;
  }
  return 0;
}

// inserts the inst written to write_ptr location
void buf_map_insert() {
  Addr line_addr = circ_buf[wrptr].instruction_addr & CLINE;
  auto it = buf_map.find(line_addr);
  if (it != buf_map.end()) {
    it->second++;
  }
  else {
    buf_map.insert(std::pair<Addr, Counter>(line_addr, 1));
  }
  wrptr = (wrptr + 1) % MEMTRACE_BUF_SIZE;
}

void buf_map_remove() {
  Addr line_addr = circ_buf[rdptr].instruction_addr & CLINE;
  auto it = buf_map.find(line_addr);
  assert (it != buf_map.end());
  if (it->second > 1) {
    it->second--;
  }
  else {
    buf_map.erase(it);
  }
  rdptr = (rdptr + 1) % MEMTRACE_BUF_SIZE;
}

bool buf_map_find(Addr line_addr) {
  return buf_map.find(line_addr) != buf_map.end();
}

int memtrace_trace_read(int proc_id, ctype_pin_inst* next_onpath_pi) {
  if (!MEMTRACE_BUF_SIZE) {
    return memtrace_trace_read_internal(proc_id, next_onpath_pi);
  }
  else {
    *next_onpath_pi = circ_buf[rdptr];
    buf_map_remove();
    int ret = memtrace_trace_read_internal(proc_id, &circ_buf[wrptr]);
    buf_map_insert();
    return ret;
  }
}

int memtrace_trace_read_internal(int proc_id, ctype_pin_inst* next_onpath_pi) {
  InstInfo* insi;

  do {
    insi = const_cast<InstInfo*>(trace_readers[proc_id]->nextInstruction());

    if(prior_pid == 0) {
      ASSERT(proc_id, prior_tid == 0);
      ASSERT(proc_id, insi->valid);
      prior_pid = insi->pid;
      prior_tid = insi->tid;
      ASSERT(proc_id, prior_tid);
      ASSERT(proc_id, prior_pid);
    }
    if(insi->valid) {
      ins_id++;
      if(insi->fetched_instruction) {
        ins_id_fetched++;
      }
    } else {
      return 0;  // end of trace
    }
  } while(insi->pid != prior_pid || insi->tid != prior_tid);

  memset(next_onpath_pi, 0, sizeof(ctype_pin_inst));
  fill_in_dynamic_info(next_onpath_pi, insi);
  fill_in_basic_info(next_onpath_pi, insi->ins);
  if(XED_INS_IsVgather(insi->ins) || XED_INS_IsVscatter(insi->ins)) {
    xed_category_enum_t category           = XED_INS_Category(insi->ins);
    scatter_info_storage[insi->pc] = add_to_gather_scatter_info_storage(
      insi->pc, XED_INS_IsVgather(insi->ins), XED_INS_IsVscatter(insi->ins), category);
  }
  uint32_t max_op_width = add_dependency_info(next_onpath_pi, insi->ins);
  fill_in_simd_info(next_onpath_pi, insi->ins, max_op_width);
  apply_x87_bug_workaround(next_onpath_pi, insi->ins);
  fill_in_cf_info(next_onpath_pi, insi->ins);
  print_err_if_invalid(next_onpath_pi, insi->ins);

  if (next_onpath_pi->scarab_marker_roi_begin == true) {
    assert(!roi_dump_began);
    // reset stats
    std::cout << "Reached roi dump begin marker, reset stats" << std::endl;
    reset_stats(TRUE);
    roi_dump_began = TRUE;
  } else if (next_onpath_pi->scarab_marker_roi_end == true) {
    assert(roi_dump_began);
    // dump stats
    std::cout << "Reached roi dump end marker, dump stats between" << std::endl;
    dump_stats(proc_id, TRUE, global_stat_array[proc_id], NUM_GLOBAL_STATS);
    roi_dump_began = FALSE;
    roi_dump_ID ++;
  }

  // End of ROI
  if(roi(insi->ins))
    return 0;

  return 1;
}


/**************************************************************************************/
/* trace_init() */

void memtrace_init(void) {
  uop_generator_init(NUM_CORES);
  init_x86_decoder(nullptr);
  init_x87_stack_delta();

  //next_onpath_pi = (ctype_pin_inst*)malloc(NUM_CORES * sizeof(ctype_pin_inst));

  /* temp variable needed for easy initialization syntax */
  char* tmp_trace_files[MAX_NUM_PROCS] = {
    CBP_TRACE_R0,  CBP_TRACE_R1,  CBP_TRACE_R2,  CBP_TRACE_R3,  CBP_TRACE_R4,
    CBP_TRACE_R5,  CBP_TRACE_R6,  CBP_TRACE_R7,  CBP_TRACE_R8,  CBP_TRACE_R9,
    CBP_TRACE_R10, CBP_TRACE_R11, CBP_TRACE_R12, CBP_TRACE_R13, CBP_TRACE_R14,
    CBP_TRACE_R15, CBP_TRACE_R16, CBP_TRACE_R17, CBP_TRACE_R18, CBP_TRACE_R19,
    CBP_TRACE_R20, CBP_TRACE_R21, CBP_TRACE_R22, CBP_TRACE_R23, CBP_TRACE_R24,
    CBP_TRACE_R25, CBP_TRACE_R26, CBP_TRACE_R27, CBP_TRACE_R28, CBP_TRACE_R29,
    CBP_TRACE_R30, CBP_TRACE_R31, CBP_TRACE_R32, CBP_TRACE_R33, CBP_TRACE_R34,
    CBP_TRACE_R35, CBP_TRACE_R36, CBP_TRACE_R37, CBP_TRACE_R38, CBP_TRACE_R39,
    CBP_TRACE_R40, CBP_TRACE_R41, CBP_TRACE_R42, CBP_TRACE_R43, CBP_TRACE_R44,
    CBP_TRACE_R45, CBP_TRACE_R46, CBP_TRACE_R47, CBP_TRACE_R48, CBP_TRACE_R49,
    CBP_TRACE_R50, CBP_TRACE_R51, CBP_TRACE_R52, CBP_TRACE_R53, CBP_TRACE_R54,
    CBP_TRACE_R55, CBP_TRACE_R56, CBP_TRACE_R57, CBP_TRACE_R58, CBP_TRACE_R59,
    CBP_TRACE_R60, CBP_TRACE_R61, CBP_TRACE_R62, CBP_TRACE_R63,
  };
  if(DUMB_CORE_ON) {
    // avoid errors by specifying a trace known to be good
    tmp_trace_files[DUMB_CORE] = tmp_trace_files[0];
  }

  for(uns proc_id = 0; proc_id < MAX_NUM_PROCS; proc_id++) {
    trace_files[proc_id] = tmp_trace_files[proc_id];
  }
  for(uns proc_id = 0; proc_id < NUM_CORES; proc_id++) {
    memtrace_setup(proc_id);
  }
}

void memtrace_setup(uns proc_id) {
  std::string path(trace_files[proc_id]);
  std::string trace(path);
  std::string binaries(MEMTRACE_MODULES_LOG);

  trace_readers[proc_id] = new TraceReaderMemtrace(trace, binaries, 1);

  if(FAST_FORWARD) {
    ASSERT(proc_id, !MEMTRACE_ROI_BEGIN && !MEMTRACE_ROI_END);
    uint64_t inst_count_to_use = USE_FETCHED_COUNT ?
                                  ins_id_fetched : ins_id;
    std::cout << "Enter fast forward " << inst_count_to_use << std::endl;
    // FFWD the first instruction and as many as later ffwding parameters specify.
    // insi is invalid once end of trace is reached.
    // Reaching the end of the trace breaks out of the loop and segfaults later in this function.
    const InstInfo *insi;
    do {
      insi = trace_readers[proc_id]->nextInstruction();
      ins_id++;
      if(insi->fetched_instruction) {
        ins_id_fetched++;
      }

      inst_count_to_use = USE_FETCHED_COUNT ? ins_id_fetched : ins_id;

      if((inst_count_to_use % 10000000) == 0)
        std::cout << "Fast forwarded " << inst_count_to_use << " instructions."
        << (insi->valid ? " Valid" : " Invalid") << " instr." << std::endl;
    } while(ffwd(insi->ins));
    std::cout << "Exit fast forward " << inst_count_to_use << std::endl;
  }

  if (MEMTRACE_BUF_SIZE) {
    circ_buf.resize(MEMTRACE_BUF_SIZE);
    rdptr = 0;
    wrptr = 0;
    for (uint i = 0; i < MEMTRACE_BUF_SIZE; i++) {
      memtrace_trace_read_internal(proc_id, &circ_buf[wrptr]);
      buf_map_insert();
    }
  }
}

/* Copyright 2020 University of Michigan (implemented by Tanvir Ahmed Khan)
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
 * File         : frontend/pt_memtrace/pt_fe.cc
 * Author       : Tanvir Ahmed Khan
 * Date         : 12/05/2020
 * Description  : Interface to simulate Intel processor trace
 ***************************************************************************************/
extern "C" {
#include "debug/debug.param.h"
#include "memory/memory.param.h"
#include "debug/debug_macros.h"
#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"
}

#include "bp/bp.h"
#include "statistics.h"
#include "bp/bp.param.h"
#include "ctype_pin_inst.h"
#include "frontend/pt_memtrace/pt_fe.h"
#include "isa/isa.h"
#include "pin/pin_lib/uop_generator.h"
#include "pin/pin_lib/x86_decoder.h"
#include <cmath>
#include <iomanip>
#include <random>

#define DR_DO_NOT_DEFINE_int64
#include "frontend/pt_memtrace/pt_trace_reader_pt.h"

/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_TRACE_READ, ##args)

/**************************************************************************************/
/* Global Variables for PT */

char* pt_trace_files[MAX_NUM_PROCS];
TraceReaderPT *pt_trace_readers[MAX_NUM_PROCS];
uint64_t pt_ins_id = 0;
uint64_t pt_prior_tid = 0;
uint64_t pt_prior_pid = 0;

std::mt19937 gen(0);
// Generate random addresses near the mean (1GB)
const uint64_t mean = 1000000000;
// Generate addresses where approximately 92% hit the L1 cache for DCACHE_SIZE=48KB
double sd = 14000;
std::normal_distribution<> d {mean, sd}; //generates an address of 1G +/-25K with 92% proability
const uint64_t offset = 0xFF0000; //ensure to generate no zero page address
/**************************************************************************************/
/* Private Functions for PT */

void pt_fill_in_dynamic_info(ctype_pin_inst* info, const InstInfo *insi) {
    uint8_t ld = 0;
    uint8_t st = 0;

    // Note: should be overwritten for a taken control flow instruction
    info->instruction_addr      = insi->pc;
    info->instruction_next_addr = insi->target;
    info->actually_taken = insi->taken;
    info->branch_target = insi->target;
    info->inst_uid = pt_ins_id;

#ifdef PRINT_INSTRUCTION_INFO
    std::cout << std::hex << info->instruction_addr << " Next " << info->instruction_next_addr
	      << " size " << (uint32_t)info->size << " taken " << (uint32_t)info->actually_taken
	      << " target " << info->branch_target << " asm "
	      << std::string(xed_iclass_enum_t2str(xed_decoded_inst_get_iclass(insi->ins))) <<std::endl;
#endif

    // TODO: Check whether we need to add this fix to memtrace fe
    if (xed_decoded_inst_get_iclass(insi->ins) == XED_ICLASS_RET_FAR ||
	xed_decoded_inst_get_iclass(insi->ins) == XED_ICLASS_RET_NEAR)
      info->actually_taken = 1;

    for (uint8_t op = 0; op < xed_decoded_inst_number_of_memory_operands(insi->ins); op++) {
        //generate random address according to normal distribution as PT does not contain memory addresses
        uint64_t fake_addr = std::round(d(gen)) + offset;
        //predicated true ld/st are handled just as regular ld/st
	if(xed_decoded_inst_mem_read(insi->ins, op) && !insi->mem_used[op]) {
          info->ld_vaddr[ld++] = fake_addr;
	}
	else if(xed_decoded_inst_mem_read(insi->ins, op)) {
          info->ld_vaddr[ld++] = fake_addr;
	}
	if(xed_decoded_inst_mem_written(insi->ins, op) && !insi->mem_used[op]) {
          info->st_vaddr[st++] = fake_addr;
	}
	else if(xed_decoded_inst_mem_written(insi->ins, op)) {
          info->st_vaddr[st++] = fake_addr;
	}
    }
}

int pt_ffwd(const xed_decoded_inst_t* ins) {
  if (!FAST_FORWARD) {
    return 0;
  }
  if(XED_INS_Opcode(ins) == XED_ICLASS_XCHG &&
     XED_INS_OperandReg(ins, 0) == XED_REG_RCX &&
     XED_INS_OperandReg(ins, 1) == XED_REG_RCX) {
    return 0;
  }
  return 1;
}

int pt_roi(const xed_decoded_inst_t* ins) {
  if(XED_INS_Opcode(ins) == XED_ICLASS_XCHG &&
     XED_INS_OperandReg(ins, 0) == XED_REG_RCX &&
     XED_INS_OperandReg(ins, 1) == XED_REG_RCX) {
    return 1;
  }
  return 0;
}

int pt_trace_read(int proc_id, ctype_pin_inst* pt_next_pi) {
  InstInfo *insi;

  do {
     insi = const_cast<InstInfo *>(pt_trace_readers[proc_id]->nextInstruction());
     pt_ins_id++;
     if (!insi->valid)
       return 0; //end of trace
  } while (insi->pid != pt_prior_pid || insi->tid != pt_prior_tid);

  memset(pt_next_pi, 0, sizeof(ctype_pin_inst));
  pt_fill_in_dynamic_info(pt_next_pi, insi);
  fill_in_basic_info(pt_next_pi, insi->ins);
  assert(pt_next_pi->instruction_next_addr && "instruction_next_addr not set");
  uint32_t max_op_width = add_dependency_info(pt_next_pi, insi->ins);
  fill_in_simd_info(pt_next_pi, insi->ins, max_op_width);
  apply_x87_bug_workaround(pt_next_pi, insi->ins);
  fill_in_cf_info(pt_next_pi, insi->ins);
  pt_next_pi->actually_taken = insi->taken;
  // TODO: This happens for PT for example when there is some traces missing,
  // Check whether this can also happen for memtrace fe
  if(insi->static_target) {
      // XED encoded target may not be right, so set it directly
      //std::cout << "setting branch target to: " << insi->static_target << " for PC: " << insi->pc << std::endl;
      pt_next_pi->branch_target = insi->static_target;
  }
  /* std::cout << "branch target for PC: " << pt_next_pi->instruction_addr << " is: " << pt_next_pi->branch_target << std::endl; */
  print_err_if_invalid(pt_next_pi, insi->ins);

  //End of ROI
  if (pt_roi(insi->ins))
    return 0;

  return 1;
}

void pt_init(void) {
  uop_generator_init(NUM_CORES);
  init_x86_decoder(nullptr);
  init_x87_stack_delta();

  //pt_next_pi = (ctype_pin_inst*)malloc(NUM_CORES * sizeof(ctype_pin_inst));
  for(int i = 0; i < MAX_NUM_PROCS; ++i) {
      pt_trace_readers[i] = nullptr;
  }

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
    pt_trace_files[proc_id] = tmp_trace_files[proc_id];
  }
  for(uns proc_id = 0; proc_id < NUM_CORES; proc_id++) {
    pt_setup(proc_id);
  }
}

void pt_setup(uns proc_id) {
  std::string path(pt_trace_files[proc_id]);
  std::string trace(path);

  pt_trace_readers[proc_id] = new TraceReaderPT(trace);

  //FFWD
  const InstInfo *insi = pt_trace_readers[proc_id]->nextInstruction();
  pt_ins_id++;

  if(FAST_FORWARD) {
    std::cout << "Enter fast forward " << pt_ins_id << std::endl;
  }

  while (!insi->valid || pt_ffwd(insi->ins)) {
    insi = pt_trace_readers[proc_id]->nextInstruction();
    pt_ins_id++;
    if ((pt_ins_id % 10000000) == 0)
      std::cout << "Fast forwarded " << pt_ins_id << " instructions." << std::endl;
    if (pt_ins_id >= FAST_FORWARD_TRACE_INS)
      break;
  }

  if(FAST_FORWARD) {
    std::cout << "Exit fast forward " << pt_ins_id << std::endl;
  }

  pt_prior_pid = insi->pid;
  pt_prior_tid = insi->tid;
  assert(pt_prior_tid);
  assert(pt_prior_pid);
}


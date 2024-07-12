#include "trace_fe.h"
#include "bp/bp.h"
#include "bp/bp.param.h"
#include "ctype_pin_inst.h"
#include "frontend/pt_memtrace/memtrace_fe.h"
#include "isa/isa.h"
#include "pin/pin_lib/uop_generator.h"
#include "pin/pin_lib/x86_decoder.h"
#include "statistics.h"
#include "sim.h"
#include <iostream>
#include <map>
#include <fstream>
#include <iomanip>
#include <limits>

#include "frontend/pt_memtrace/pt_fe.h"
#include "frontend/frontend_intf.h"

/**************************************************************************************/
/* Macros */

#include "debug/debug.param.h"
#include "debug/debug_macros.h"
#include "globals/assert.h"
//#include "globals/global_defs.h"
//#include "globals/global_types.h"
//#include "globals/global_vars.h"
//#include "globals/utils.h"
//#include "globals/global_types.h"


#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_TRACE_READ, ##args)

/* Globals */
static ctype_pin_inst next_onpath_pi[MAX_NUM_PROCS];
static ctype_pin_inst next_offpath_pi[MAX_NUM_PROCS];
static bool            off_path_mode[MAX_NUM_PROCS] = {false};
static uint64_t        off_path_addr[MAX_NUM_PROCS] = {0};
static std::unordered_map<uint64_t, ctype_pin_inst> pc_to_inst;

extern uint64_t ins_id;
extern uint64_t ins_id_fetched;

void off_path_generate_inst(uns proc_id, uint64_t *off_path_addr, ctype_pin_inst *inst) {
  auto op_iter = pc_to_inst.find(*off_path_addr);
  if (op_iter != pc_to_inst.end()) {
    *inst = op_iter->second;
    *off_path_addr += inst->size;
    DEBUG(proc_id, "Generate off-path inst:%lx inst_size:%i ",inst->instruction_addr, inst->size);
  }
  else {
    *inst = create_dummy_nop(*off_path_addr, WPNM_REASON_REDIRECT_TO_NOT_INSTRUMENTED);
    (*off_path_addr) += DUMMY_NOP_SIZE;
  }
}

Flag ctype_pin_inst_same_mem_vaddr(ctype_pin_inst inst_a, ctype_pin_inst inst_b) {
  for (uns i = 0; i < MAX_LD_NUM; i++) {
    if (inst_a.ld_vaddr[i] != inst_b.ld_vaddr[i]) {
      return FALSE;
    }
  }

  for (uns i = 0; i < MAX_ST_NUM; i++) {
    if (inst_a.st_vaddr[i] != inst_b.st_vaddr[i]) {
      return FALSE;
    }
  }

  return TRUE;
}

void assert_ctype_pin_inst_same(uns proc_id, ctype_pin_inst inst_a, ctype_pin_inst inst_b) {
  // uint64_t inst_uid;  // unique ID produced by the frontend

  ASSERT(proc_id, inst_a.instruction_addr == inst_b.instruction_addr);
  ASSERT(proc_id, inst_a.size == inst_b.size);
  ASSERT(proc_id, inst_a.inst_binary_msb == inst_b.inst_binary_msb);
  ASSERT(proc_id, inst_a.inst_binary_lsb == inst_b.inst_binary_lsb);
  ASSERT(proc_id, inst_a.op_type == inst_b.op_type);
  ASSERT(proc_id, inst_a.cf_type == inst_b.cf_type);
  ASSERT(proc_id,  inst_a.is_fp == inst_b.is_fp);
  ASSERT(proc_id,  inst_a.true_op_type == inst_b.true_op_type);
  ASSERT(proc_id,  inst_a.num_src_regs == inst_b.num_src_regs);
  ASSERT(proc_id,  inst_a.num_dst_regs == inst_b.num_dst_regs);
  ASSERT(proc_id,  inst_a.num_ld1_addr_regs == inst_b.num_ld1_addr_regs);
  ASSERT(proc_id,  inst_a.num_ld2_addr_regs == inst_b.num_ld2_addr_regs);
  ASSERT(proc_id,  inst_a.num_st_addr_regs == inst_b.num_st_addr_regs);

  for (uns i = 0; i < MAX_SRC_REGS_NUM; i++) {
    ASSERT(proc_id, inst_a.src_regs[i] == inst_b.src_regs[i]);
  }

  for (uns i = 0; i < MAX_DST_REGS_NUM; i++) {
    ASSERT(proc_id, inst_a.dst_regs[i] == inst_b.dst_regs[i]);
  }

  for (uns i = 0; i < MAX_MEM_ADDR_REGS_NUM; i++) {
    ASSERT(proc_id, inst_a.ld1_addr_regs[i] == inst_b.ld1_addr_regs[i]);
  }

  for (uns i = 0; i < MAX_MEM_ADDR_REGS_NUM; i++) {
    ASSERT(proc_id, inst_a.ld2_addr_regs[i] == inst_b.ld2_addr_regs[i]);
  }

  for (uns i = 0; i < MAX_MEM_ADDR_REGS_NUM; i++) {
    ASSERT(proc_id, inst_a.st_addr_regs[i] == inst_b.st_addr_regs[i]);
  }

  ASSERT(proc_id, inst_a.num_simd_lanes == inst_b.num_simd_lanes);
  ASSERT(proc_id, inst_a.lane_width_bytes == inst_b.lane_width_bytes);
  ASSERT(proc_id, inst_a.num_ld == inst_b.num_ld);
  ASSERT(proc_id, inst_a.num_st == inst_b.num_st);
  ASSERT(proc_id, inst_a.has_immediate == inst_b.has_immediate);

  for (uns i = 0; i < MAX_LD_NUM; i++) {
    ASSERT(proc_id, inst_a.ld_vaddr[i] == inst_b.ld_vaddr[i]);
  }

  for (uns i = 0; i < MAX_ST_NUM; i++) {
    ASSERT(proc_id, inst_a.st_vaddr[i] == inst_b.st_vaddr[i]);
  }

  ASSERT(proc_id, inst_a.ld_size == inst_b.ld_size);
  ASSERT(proc_id, inst_a.st_size == inst_b.st_size);
  ASSERT(proc_id, inst_a.branch_target == inst_b.branch_target);
  ASSERT(proc_id, inst_a.actually_taken == inst_b.actually_taken);

  ASSERT(proc_id, inst_a.is_string == inst_b.is_string);
  ASSERT(proc_id, inst_a.is_call == inst_b.is_call);
  ASSERT(proc_id, inst_a.is_move == inst_b.is_move);
  ASSERT(proc_id, inst_a.is_prefetch == inst_b.is_prefetch);

  ASSERT(proc_id, inst_a.has_push == inst_b.has_push);
  ASSERT(proc_id, inst_a.has_pop == inst_b.has_pop);
  ASSERT(proc_id, inst_a.is_ifetch_barrier == inst_b.is_ifetch_barrier);
  ASSERT(proc_id, inst_a.is_lock == inst_b.is_lock);

  ASSERT(proc_id, inst_a.is_repeat == inst_b.is_repeat);
  ASSERT(proc_id, inst_a.is_simd == inst_b.is_simd);
  ASSERT(proc_id, inst_a.is_gather_scatter == inst_b.is_gather_scatter);
  ASSERT(proc_id, inst_a.is_sentinel == inst_b.is_sentinel);

  ASSERT(proc_id, inst_a.fake_inst == inst_b.fake_inst);
  ASSERT(proc_id, inst_a.exit == inst_b.exit);
  ASSERT(proc_id, inst_a.fake_inst_reason == inst_b.fake_inst_reason);
  ASSERT(proc_id, inst_a.instruction_next_addr == inst_b.instruction_next_addr);

  for (uns i = 0; i < 16; i++) {
    ASSERT(proc_id, inst_a.pin_iclass[i] == inst_b.pin_iclass[i]);
  }
}

void ext_trace_fetch_op(uns proc_id, Op* op) {
  if(uop_generator_get_bom(proc_id)) {
    if (!off_path_mode[proc_id]) {
      uop_generator_get_uop(proc_id, op, &next_onpath_pi[proc_id]);
    }
    else {
      uop_generator_get_uop(proc_id, op, &next_offpath_pi[proc_id]);
    }
  } else {
    uop_generator_get_uop(proc_id, op, NULL);
  }

  if(uop_generator_get_eom(proc_id)) {
    if (!off_path_mode[proc_id]) {

      int success = false;
      if (FRONTEND == FE_PT)
        success = pt_trace_read(proc_id, &next_onpath_pi[proc_id]);
      else if (FRONTEND == FE_MEMTRACE)
        success = memtrace_trace_read(proc_id, &next_onpath_pi[proc_id]);
      if(!success) {
        trace_read_done[proc_id] = TRUE;
        reached_exit[proc_id]    = TRUE;
        op->exit = TRUE;
      }
      else {
        uint64_t addr = next_onpath_pi[proc_id].instruction_addr;
        auto find = pc_to_inst.find(addr);
        if(find == pc_to_inst.end()) {
          pc_to_inst.insert(std::pair<uint64_t, ctype_pin_inst>(addr, next_onpath_pi[proc_id]));
        }
        else if (next_onpath_pi[proc_id].inst_binary_lsb != find->second.inst_binary_lsb ||
                 next_onpath_pi[proc_id].inst_binary_msb != find->second.inst_binary_msb) {
          DEBUG(proc_id, "Previously seen PC references new instruction addr:%lx inst_size:%i lsb:%lx msb:%lx\n ",
                addr, next_onpath_pi[proc_id].size, next_onpath_pi[proc_id].inst_binary_lsb,
                next_onpath_pi[proc_id].inst_binary_msb);
          // Handle jitted code
          STAT_EVENT(proc_id, INST_MAP_UPDATE_JITTED);
          pc_to_inst.erase(addr);
          pc_to_inst.insert(std::pair<uint64_t, ctype_pin_inst>(addr, next_onpath_pi[proc_id]));
        }
        else if (next_onpath_pi[proc_id].instruction_next_addr != find->second.instruction_next_addr) {
          ASSERT(proc_id, next_onpath_pi[proc_id].op_type == find->second.op_type);
          if (next_onpath_pi[proc_id].cf_type) {
            ASSERT(proc_id, next_onpath_pi[proc_id].cf_type == find->second.cf_type);
            // This can fail for java pt traces
            // ASSERT(proc_id, next_onpath_pi[proc_id].cf_type == CF_CBR ||
            //                 next_onpath_pi[proc_id].cf_type >= CF_IBR ||
            //                 next_onpath_pi[proc_id].last_inst_from_trace);
          }
          STAT_EVENT(proc_id, INST_MAP_UPDATE_NPC_INV + next_onpath_pi[proc_id].op_type);
          pc_to_inst.erase(addr);
          pc_to_inst.insert(std::pair<uint64_t, ctype_pin_inst>(addr, next_onpath_pi[proc_id]));
        }
        else if (!ctype_pin_inst_same_mem_vaddr(next_onpath_pi[proc_id], find->second)) {
          ASSERT(proc_id, next_onpath_pi[proc_id].op_type == find->second.op_type);
          STAT_EVENT(proc_id, INST_MAP_UPDATE_MEM_INV + next_onpath_pi[proc_id].op_type);
          pc_to_inst.erase(addr);
          pc_to_inst.insert(std::pair<uint64_t, ctype_pin_inst>(addr, next_onpath_pi[proc_id]));
        }
        else {
          assert_ctype_pin_inst_same(proc_id, next_onpath_pi[proc_id], find->second);
        }
      }
    }
    else {
      off_path_generate_inst(proc_id, &off_path_addr[proc_id], &next_offpath_pi[proc_id]);
    }
  }
  DEBUG(proc_id, "Fetch op is_on_path:%i on_path:%lx off_path:%lx\n", off_path_mode[proc_id], next_onpath_pi[proc_id].instruction_addr, next_offpath_pi[proc_id].instruction_addr);
}

Flag ext_trace_can_fetch_op(uns proc_id) {
  return !(uop_generator_get_eom(proc_id) && trace_read_done[proc_id]);
}

void ext_trace_redirect(uns proc_id, uns64 inst_uid, Addr fetch_addr) {
  off_path_mode[proc_id] = true;
  off_path_addr[proc_id] = fetch_addr;
  off_path_generate_inst(proc_id, &off_path_addr[proc_id], &next_offpath_pi[proc_id]);
  DEBUG(proc_id, "Redirect on-path:%lx off-path:%lx", next_onpath_pi[proc_id].instruction_addr, next_offpath_pi[proc_id].instruction_addr);
}

void ext_trace_recover(uns proc_id, uns64 inst_uid) {
  Op dummy_op;
  off_path_mode[proc_id] = false;
  // Finish decoding of the current off-path inst before switching to on-path
  while (!uop_generator_get_eom(proc_id)) {
    uop_generator_get_uop(proc_id, &dummy_op, &next_offpath_pi[proc_id]);
  }
  DEBUG(proc_id, "Recover CF:%lx ", next_onpath_pi[proc_id].instruction_addr);
}

void ext_trace_retire(uns proc_id, uns64 inst_uid) {
  // Trace frontend does not need to communicate to PIN which instruction are
  // retired.
}

Addr ext_trace_next_fetch_addr(uns proc_id) {
  return next_onpath_pi[proc_id].instruction_addr;
}

void ext_trace_init() {
  memset(next_offpath_pi, 0, sizeof(next_offpath_pi));
  memset(next_onpath_pi, 0, sizeof(next_onpath_pi));

  if (FRONTEND == FE_PT) {
    pt_init();
    for(uns proc_id = 0; proc_id < NUM_CORES; proc_id++) {
      pt_trace_read(proc_id, &next_onpath_pi[proc_id]);
    }
  }
  else if (FRONTEND == FE_MEMTRACE) {
    memtrace_init();
    for(uns proc_id = 0; proc_id < NUM_CORES; proc_id++) {
      memtrace_trace_read(proc_id, &next_onpath_pi[proc_id]);
    }
  }
}

void ext_trace_done() {

}

// is also used to print footprint
uint64_t output_fingerprint(std::string file_name, std::map<uint64_t, uint64_t> fingerprint) {
  // output the map for this segment
  // make it a function?
  std::ofstream myfile;
  myfile.open(file_name, std::ofstream::out | std::ofstream::app);

  if (!myfile.is_open()) {
    std::cout << "open file failed: " << file_name << std::endl;
  }

  // std::cout << num_of_segments << "th fp dimensions: " << fingerprint.size() << std::endl;
  // fine if comment starting here
  std::map<uint64_t, uint64_t>::iterator freq;
  uint64_t instrs_count = 0;

  uint64_t nonzero_count = 0;
  // static std::vector<uint64> csv_line(counts_as_built.blocks, 0);

  for (freq = fingerprint.begin(); freq != fingerprint.end(); freq++) {
      instrs_count += freq->second;
      if(freq == fingerprint.begin()) {
        myfile << "T";
      }
      myfile << ":" << freq->first << ":" << freq->second << " ";

      // csv_line[freq->first] = freq->second;
      nonzero_count++;

      // if (freq->first + 1 == counts_as_built.blocks) {
      //     witness_total = true;
      // }
  }

  // ASSERT(proc_id, nonzero_count == fingerprint.size());

  myfile << std::endl;
  myfile.close();

  return instrs_count;
}

typedef struct bb_counts {
    uint64_t blocks;
    uint64_t total_size;
    uint64_t fetched_size;
} bb_counts;

typedef struct basic_block_info
{
  // instruction list contained in this basic block
  std::vector<ctype_pin_inst> ins_list;
  // fetched inst count in this basic block
  uint64_t inst_count_fetched;
  // the basic block id
  uint64_t bb_id;
  uint64_t freq;
  void clear() {
    inst_count_fetched = 0;
    ins_list.clear();
    bb_id = 0;
    freq = 0;
  }

  basic_block_info() {
    inst_count_fetched = 0;
    bb_id = 0;
    ins_list = std::vector<ctype_pin_inst>();
    freq = 0;
  }

  bool operator<(const basic_block_info& rhs) const
  {
    std::vector<ctype_pin_inst>::size_type size = ins_list.size() < rhs.ins_list.size()?
                                                  ins_list.size() : rhs.ins_list.size();
    for(unsigned i = 0 ; i < size; i++) {
      if(ins_list[i].instruction_addr < rhs.ins_list[i].instruction_addr) {
        return true;
      }
    }

    if(ins_list.size() < rhs.ins_list.size()) {
      return true;
    } else {
      return false;
    }
  }

  bool operator!=(const basic_block_info& bb) const
  {
    if(ins_list.size() != bb.ins_list.size()) {
      return true;
    }

    for(unsigned i = 0 ; i < this->ins_list.size(); i++) {
      if(this->ins_list[i].instruction_addr != bb.ins_list[i].instruction_addr) {
        return true;
      }
    }

    return false;
  }
} basic_block_info;

void output_counts(uint64_t num_of_segments,
  bb_counts counts_dynamic, bb_counts counts_as_built,
  uint64_t *op_taken_count,
  std::unordered_map<uint64_t, std::vector<basic_block_info>> bb_identity_map) {
  const char *op_type_strings[] = {
    "TRACE_INST_TAKEN_NOT_CF",
    "TRACE_INST_TAKEN_CF_BR",
    "TRACE_INST_TAKEN_CF_CBR",
    "TRACE_INST_TAKEN_CF_CALL",
    "TRACE_INST_TAKEN_CF_IBR",
    "TRACE_INST_TAKEN_CF_ICALL",
    "TRACE_INST_TAKEN_CF_ICO",
    "TRACE_INST_TAKEN_CF_RET",
    "TRACE_INST_TAKEN_CF_SYS",
  };

  printf("to be appened segment num %ld\n", num_of_segments);
  std::cout << counts_dynamic.total_size << std::endl;
  std::cout << counts_dynamic.fetched_size << std::endl;

  std::cout << "====================================\n";
  for(uint i = 0; i < NUM_CF_TYPES; i++) {
    std::cout << op_type_strings[i] << ":" << op_taken_count[i] << std::endl;
  }

  std::cout << "====================================\n";
  printf("identity within this segment\n");
  auto counter = 0;
  for(auto i = bb_identity_map.begin(); i != bb_identity_map.end(); i++) {
    if(i->second.size() > 1) {
      counter++;
      printf("%lu bbs with same first pc\n", i->second.size());
      for(unsigned j = 0; j < i->second.size(); j++) {
        printf("[%ld]: size %ld, freq %ld\n", i->second[j].bb_id, i->second[j].ins_list.size(), i->second[j].freq);
        for(unsigned k = 0; k < i->second[j].ins_list.size(); k++) {
          printf("%s: %u, %d\n",
                  hexstr64s(i->second[j].ins_list[k].instruction_addr),
                  i->second[j].ins_list[k].true_op_type, i->second[j].ins_list[k].last_inst_from_trace);
        }
      }
    }
  }

  printf("%d / %ld\n", counter, bb_identity_map.size());

  printf(
  "========================================================\n"
  "Number of blocks built : %ld\n"
  "     Average size      : %5.2lf instructions\n"
  "Number of blocks executed  : %ld\n"
  "     Average weighted size : %5.2lf instructions\n"
  "Number of total instructions  : %ld\n"
  "Number of fetched instruction : %ld\n"
  "========================================================\n"
  ,
  counts_as_built.blocks,
  counts_as_built.total_size / (double)counts_as_built.blocks,
  counts_dynamic.blocks,
  counts_dynamic.total_size / (double)counts_dynamic.blocks,
  counts_dynamic.total_size,
  counts_dynamic.fetched_size
  );
}

// ATTENTION: the string instruction expansion can inflate frenquency
// those abstract inst type..
void ext_trace_extract_basic_block_vectors() {
  uint64_t op_taken_count[NUM_CF_TYPES] = {0};

  std::vector<std::pair<std::string, uint64_t>> npc_op_count;

  ASSERT(0, NUM_CORES == 1);
  uns8 proc_id = 0;
  ASSERT(proc_id, (FRONTEND == FE_PT) || (FRONTEND == FE_MEMTRACE));

  // global counters for the entire trace
  bb_counts counts_dynamic{}, counts_as_built{};
  uint64_t num_of_segments = 0;

  // if SEGMENT_INSTR_COUNT is the default value zero,
  // output a single BBV for the entire trace
  if(SEGMENT_INSTR_COUNT == 0) {
    SEGMENT_INSTR_COUNT = std::numeric_limits<uns64>::max();
  }
  // segment instruction counter, reset every segment
  uint64_t cur_counter = 0;
  uint64_t cur_counter_fetched = 0;

  // unordered map to hash basic block identifier to the basic block info
  // the identifier: the first pc of the basic block
  // this map is used throughout the post-processing
  std::unordered_map<uint64_t, basic_block_info> bb_map;

  std::unordered_map<uint64_t, std::vector<basic_block_info>> bb_identity_map;

  // map to hash the basic block to the frequency counter
  // essentially the fingerprint for a segment
  // this map is cleared every SEGMENT_SIZE instruction
  // mode 1: the key is the basic block id, used for the whole trace
  // mode 2: the key is the first addr of the basic block, used for trace chunks
  std::map<uint64_t, uint64_t> fingerprint;

  // for instruction footprint analysis
  std::map<uint64_t, uint64_t> footprint;

  // maintain the current basic block
  basic_block_info cur_bb{};

  // the first trace entry was read during frontend initialization
  // assume the first read succeeded
  ctype_pin_inst* inst = &next_onpath_pi[proc_id];
  int success = true;

  printf("read from initialization: %p\n", (void *)(inst->instruction_addr));

  // continue till trace end
  while(success) {
    if(inst->actually_taken){
      ASSERT(proc_id, inst->cf_type);
      op_taken_count[inst->cf_type]++;
    }
    if(inst->instruction_next_addr != inst->instruction_addr + inst->size) {
      if(!inst->cf_type && !inst->is_repeat && !inst->last_inst_from_trace) {
        fprintf(stderr, "the cf change is not due to cf or rep or trace end at %p\n", (void *)inst->instruction_addr);
      }
      ASSERT(proc_id, inst->cf_type || inst->is_repeat || inst->last_inst_from_trace);
    }

    // add to current basic block
    // a sequence of rep of same pc is a bb executed multiple times
    cur_bb.ins_list.push_back(*inst);

    // increment unique inst frequency
    footprint[inst->instruction_addr]++;

    // increment fetched count if it is fetched
    if(inst->fetched_instruction) {
      cur_bb.inst_count_fetched++;
    }

    // read the next instruction from the trace, which overwrites inst
    if(FRONTEND == FE_PT)
      success = pt_trace_read(proc_id, inst);
    else if(FRONTEND == FE_MEMTRACE)
      success = memtrace_trace_read(proc_id, inst);

    if(cur_bb.ins_list.back().is_repeat && !inst->is_repeat) {
      ASSERT(proc_id, cur_bb.ins_list.back().instruction_addr != inst->instruction_addr);
    }

    // decide if it's the end of current basic block
    // - a cf
    // - (current is not a rep but) the next is rep (the first half might be redundant)
    // - a rep
    // - the last trace entry
    // note that at this point inst has the next instruction
    // note that the next inst is available if success
    // if so, find the frequency counter and increment

    if(cur_bb.ins_list.back().cf_type ||
        !success ||
        (!cur_bb.ins_list.back().is_repeat && inst->is_repeat) ||
        cur_bb.ins_list.back().is_repeat) {
      std::unordered_map<uint64_t, basic_block_info>::iterator map_lookup =
                                bb_map.find(cur_bb.ins_list.front().instruction_addr);

      // std::ofstream cinstf;
      // cinstf.open("cinst.log", std::ofstream::out | std::ofstream::app);
      // for (uint i = 0; i < cur_bb.ins_list.size(); i++) {
      //   cinstf << std::hex
      //          << std::setfill('0') << std::setw(sizeof(void *) * 2) << cur_bb.ins_list[i].instruction_addr
      //          << std::dec << std::setfill(' ') << std::endl;
      // }
      // cinstf.close();
      if(map_lookup != bb_map.end()) {
        // not the first time bb
        // the bb size might be cut if at boundary
        cur_bb.bb_id = map_lookup->second.bb_id;

        // sanity check: are they the same?
        auto bb_key = cur_bb.ins_list.front().instruction_addr;
        bool find = false;
        // bb_identity_map is cleared at segment boundary
        // so the sieze could be zero
        for(unsigned i = 0; i < bb_identity_map[bb_key].size(); i++) {
          if(cur_bb != bb_identity_map[bb_key][i]) {
          } else {
            find = true;
            bb_identity_map[bb_key][i].freq++;
            break;
          }
        }
        if(!find) {
          cur_bb.freq++;
          bb_identity_map[bb_key].push_back(cur_bb);

          if(bb_identity_map[bb_key].size() > 1) {
            for(uint i = 0; i < cur_bb.ins_list.size(); i++) {
              fprintf(stderr, "[%lu]: ad: %p, op: %d\n", map_lookup->second.bb_id, (void *)cur_bb.ins_list[i].instruction_addr, cur_bb.ins_list[i].true_op_type);
            }
            fprintf(stderr, "DUP bb detected%s\n", success? "" : " at trace end");
          }
        }
      } else {
        // the first rep string should be marked as fetched
        // in other words, counts_as_built.total_size == counts_as_built.fetched_size
        ASSERT(proc_id, cur_bb.ins_list.size() == cur_bb.inst_count_fetched);

        // a new bb, starting at 1
        counts_as_built.blocks++;
        cur_bb.bb_id = counts_as_built.blocks;
        counts_as_built.total_size += cur_bb.ins_list.size();
        counts_as_built.fetched_size += cur_bb.inst_count_fetched;

        // enter bb map
        bb_map[cur_bb.ins_list.front().instruction_addr] = cur_bb;
        // sanity check
        cur_bb.freq++;
        bb_identity_map[cur_bb.ins_list.front().instruction_addr].push_back(cur_bb);
        ASSERT(proc_id, bb_identity_map[cur_bb.ins_list.front().instruction_addr].size() == 1);

        // fprintf(stderr, "======================\n");
        static int bb_built_count = 0;
        for(uint i = 0; i < cur_bb.ins_list.size(); i++) {
          fprintf(stderr, "[%d]: ad: %p, op: %d\n", bb_built_count, (void *)cur_bb.ins_list[i].instruction_addr, cur_bb.ins_list[i].true_op_type);
        }
        bb_built_count++;
        fprintf(stderr, "new bb detected: %d\n", bb_built_count);
        // fprintf(stderr, "======================\n");
      }



      counts_dynamic.blocks++;
      counts_dynamic.total_size += cur_bb.ins_list.size();
      counts_dynamic.fetched_size += cur_bb.inst_count_fetched;
      cur_counter += cur_bb.ins_list.size();
      cur_counter_fetched += cur_bb.inst_count_fetched;

      // if TRACE_BBV_MODE_DISTRIBUTED,
      // the fetched counter always will not exceed SEGMENT_INSTR_COUNT,
      // as the frontend would only be provided that many instructions
      if(SIM_MODE == TRACE_BBV_DISTRIBUTED_MODE) {
        ASSERT(proc_id, cur_counter_fetched <= SEGMENT_INSTR_COUNT);
      }
      // furthermore,
      // if TRACE_BBV_MODE_DISTRIBUTED,
      // and if not the last segment,
      // (cur_counter_fetched == SEGMENT_INSTR_COUNT) <=> !success
      // since do not know if it is the last one,
      // (cur_counter_fetched == SEGMENT_INSTR_COUNT) -> !success
      if(cur_counter_fetched == SEGMENT_INSTR_COUNT) {
        ASSERT(proc_id, !success);
      }

      ASSERT(proc_id, cur_counter >= cur_counter_fetched);

      // same as dynamorio client
      uint64_t to_last_vector_count = 0;
      uint64_t to_new_vector_count = 0;
      uint64_t to_new_vector_count_fetched = 0;
      if((USE_FETCHED_COUNT ? cur_counter_fetched : cur_counter) > SEGMENT_INSTR_COUNT) {
        // if at a boundary (excluding perfect aligned boundary)
        // to_new_vector_count = cur_counter - SEGMENT_INSTR_COUNT;
        // to_last_vector_count = cur_bb.ins_list.size() - to_new_vector_count;

        // re-examine the counting
        // doing this because not knowing which one is fetched or not
        // but it could be easier as we only have rep string emulation?
        cur_counter -= cur_bb.ins_list.size();
        cur_counter_fetched -= cur_bb.inst_count_fetched;
        ASSERT(proc_id, (USE_FETCHED_COUNT ? cur_counter_fetched : cur_counter) < SEGMENT_INSTR_COUNT);
        bool to_new = false;
        for(uint i = 0; i < cur_bb.ins_list.size(); i++) {
          if(to_new) {
            to_new_vector_count++;
            to_new_vector_count_fetched++;
          } else {
            cur_counter++;
            if(cur_bb.ins_list[i].fetched_instruction) {
              cur_counter_fetched++;
            }
            to_last_vector_count++;
          }
          if((USE_FETCHED_COUNT ? cur_counter_fetched : cur_counter) == SEGMENT_INSTR_COUNT) {
            to_new = true;
          }
        }
        ASSERT(proc_id, to_new_vector_count >= to_new_vector_count_fetched);
        ASSERT(proc_id, to_new_vector_count > 0);
        ASSERT(proc_id, (USE_FETCHED_COUNT ? cur_counter_fetched : cur_counter) == SEGMENT_INSTR_COUNT);
      } else {
        ASSERT(proc_id, (USE_FETCHED_COUNT ? cur_counter_fetched : cur_counter) <= SEGMENT_INSTR_COUNT);
        to_last_vector_count = cur_bb.ins_list.size();
      }

      ASSERT(proc_id, to_last_vector_count + to_new_vector_count == cur_bb.ins_list.size());

      // the following are not true since the counters will not exceed SEGMENT_INSTR_COUNT
      // if(cur_counter_fetched > SEGMENT_INSTR_COUNT) {
      //   // it must be using the fetched count
      //   ASSERT(proc_id, USE_FETCHED_COUNT);
      //   // the termination of the current fp must be triggered
      //   ASSERT(proc_id, to_new_vector_count > 0);
      //   ASSERT(proc_id, to_new_vector_count_fetched > 0);
      // }
      // if(!USE_FETCHED_COUNT && cur_counter > SEGMENT_INSTR_COUNT) {
      //   // the termination of the current fp must be triggered
      //   ASSERT(proc_id, to_new_vector_count > 0);
      // }
      // ASSERT(proc_id, ((USE_FETCHED_COUNT ? cur_counter_fetched : cur_counter) > SEGMENT_INSTR_COUNT) == (to_new_vector_count > 0));

      if (SIM_MODE == TRACE_BBV_MODE) {
        fingerprint[cur_bb.bb_id] += to_last_vector_count;
      } else {
        // TRACE_BBV_DISTRIBUTED_MODE
        fingerprint[cur_bb.ins_list.front().instruction_addr] += to_last_vector_count;
      }

      // perfect alignment
      // two fingerprint output condition
      // - if reach segment limit
      // - if it is the end of trace
      // if two are both satisfied, will output twice
      if((USE_FETCHED_COUNT ? cur_counter_fetched : cur_counter) == SEGMENT_INSTR_COUNT) {
        num_of_segments++;
        output_counts(num_of_segments,
                      counts_dynamic, counts_as_built,
                      op_taken_count,
                      bb_identity_map);
        bb_identity_map.clear();

        cur_counter = 0;
        cur_counter_fetched = 0;

        std::string bbv_output(TRACE_BBV_OUTPUT);
        std::string footprint_output(TRACE_FOOTPRINT_OUTPUT);
        uint64_t instrs_count_bbv = output_fingerprint(bbv_output, fingerprint);
        if (!footprint_output.empty()) {
          uint64_t instrs_count_footprint = output_fingerprint(footprint_output, footprint);
          ASSERT(proc_id, instrs_count_bbv == instrs_count_footprint);
        }

        // clear for the next segment
        fingerprint.clear();
        footprint.clear();

        // record the residue
        // if to_new_vector_count > 0, the bb must have crossed the vector boundary
        if(to_new_vector_count > 0) {
            if (SIM_MODE == TRACE_BBV_MODE) {
              fingerprint.insert(std::make_pair(cur_bb.bb_id, to_new_vector_count));
            } else {
              // TRACE_BBV_DISTRIBUTED_MODE
              uint64_t bb_addr = cur_bb.ins_list.front().instruction_addr;
              fingerprint.insert(std::make_pair(bb_addr, to_new_vector_count));
            }

            cur_counter = to_new_vector_count;
            cur_counter_fetched = to_new_vector_count_fetched;
        }
      } else {
        ASSERT(proc_id, (USE_FETCHED_COUNT ? cur_counter_fetched : cur_counter) < SEGMENT_INSTR_COUNT);
      }

      // clear out current bb
      cur_bb.clear();

      if(!success && !fingerprint.empty()) {
        num_of_segments++;
        output_counts(num_of_segments,
                      counts_dynamic, counts_as_built,
                      op_taken_count,
                      bb_identity_map);

        // caution that ins_id and ins_id_fetched is only for memtrace
        ASSERT(proc_id, counts_dynamic.total_size == ins_id);
        ASSERT(proc_id, counts_dynamic.fetched_size == ins_id_fetched);

        std::string bbv_output(TRACE_BBV_OUTPUT);
        std::string footprint_output(TRACE_FOOTPRINT_OUTPUT);
        uint64_t instrs_count_bbv = output_fingerprint(bbv_output, fingerprint);
        if (!footprint_output.empty()) {
          uint64_t instrs_count_footprint = output_fingerprint(footprint_output, footprint);
          ASSERT(proc_id, instrs_count_bbv == instrs_count_footprint);
        }

        if(SIM_MODE == TRACE_BBV_DISTRIBUTED_MODE && USE_FETCHED_COUNT) {
          printf("The fingerprint outputting is triggered at the end of the trace."
                  "Is this the last segment?\n");
        }
      }
    }
  }
}

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
 * File         : frontend/pt_memtrace/memtrace_trace_reader_memtrace.h
 * Author       : Heiner Litz
 * Date         : 05/15/2020
 * Description  :
 ***************************************************************************************/

#include "frontend/pt_memtrace/memtrace_trace_reader_memtrace.h"

#include "elf.h"

#define warn(...) printf(__VA_ARGS__)
#define panic(...) printf(__VA_ARGS__)

// Trace + single binary
TraceReaderMemtrace::TraceReaderMemtrace(const std::string& _trace,
                                         const std::string& _binary,
                                         uint64_t _offset, uint32_t _bufsize) :
    TraceReader(_trace, _binary, _offset, _bufsize),
    mt_state_(MTState::INST),
    mt_use_next_ref_(true),
    mt_mem_ops_(0), mt_seq_(0), mt_prior_isize_(0), mt_using_info_a_(true),
    mt_warn_target_(0) {
  init(_trace);
}

// Trace + multiple binaries
TraceReaderMemtrace::TraceReaderMemtrace(const std::string& _trace,
                                         const std::string& _binary_group_path,
                                         uint32_t           _bufsize) :
    TraceReader(_trace, _binary_group_path, _bufsize),
    mt_state_(MTState::INST),
    mt_use_next_ref_(true),
    mt_mem_ops_(0), mt_seq_(0), mt_prior_isize_(0), mt_using_info_a_(true),
    mt_warn_target_(0) {
  binaryGroupPathIs(_binary_group_path);
  init(_trace);
}

TraceReaderMemtrace::~TraceReaderMemtrace() {
  if(mt_warn_target_ > 0) {
    warn("Set %lu conditional branches to 'not-taken' due to pid/tid gaps\n",
         mt_warn_target_);
  }
}

void TraceReaderMemtrace::init(const std::string& _trace) {
  mt_info_a_.custom_op = CustomOp::NONE;
  mt_info_b_.custom_op = CustomOp::NONE;
  mt_info_a_.valid     = true;
  mt_info_b_.valid     = true;
  TraceReader::init(_trace);
}

// TODO: Detect memtrace/module.log type dynamically
#ifdef ZSIM_USE_YT
/* Below is required to parse Google Memtraces that contain an extra column */
const char* TraceReaderMemtrace::parse_buildid_string(const char* src,
                                                      OUT void**  data) {
  // We just skip the string.  We don't store it as we're not using it here.
  const char* comma = strchr(src, ',');
  if(comma == nullptr)
    return nullptr;
  return comma + 1;
}
#endif

void TraceReaderMemtrace::binaryGroupPathIs(const std::string& _path) {
  clearBinaries();
  binary_ready_ = true;  // An absent binary is allowed
  if(!_path.empty()) {
    std::string info_name;
    info_name = _path + "/modules.log";
    std::ifstream info_file(info_name);
    if(!info_file.is_open()) {
      panic("Could not open binary collection info file '%s': %s",
            info_name.c_str(), strerror(errno));
    }
    if(_path.empty()) {
      panic("Module file path is missing");
      return;
    }
    dcontext_         = dr_standalone_init();
    std::string error = directory_.initialize_module_file(_path +
                                                          "/modules.log");
    if(!error.empty()) {
      panic("Failed to initialize directory: %s Cannot find a file named "
            "modules.log",
            error.c_str());
      return;
    }
    module_mapper_ = dynamorio::drmemtrace::module_mapper_t::create(directory_.modfile_bytes_,
#ifdef ZSIM_USE_YT
                                             parse_buildid_string,
#else
                                             nullptr,
#endif
                                             nullptr, nullptr, nullptr,
                                             knob_verbose_);
    module_mapper_->get_loaded_modules();
    error = module_mapper_->get_last_error();
    if(!error.empty()) {
      panic("Failed to load binaries: %s Check that module.log references the "
            "correct binary paths.",
            error.c_str());
      return;
    }
    binary_ready_ = true;
  }
}

bool TraceReaderMemtrace::initTrace() {
  std::vector<dynamorio::drmemtrace::scheduler_t::input_workload_t> sched_inputs;
  // memtrace region of interest provides a view of the trace only of interest
  // inst count satrt with 1
  // begin 0 is invalid
  // end is inclusive
  // end 0 is end of trace
  if(MEMTRACE_ROI_BEGIN) {
    ASSERT(0, MEMTRACE_ROI_BEGIN < MEMTRACE_ROI_END || MEMTRACE_ROI_END == 0);
    dynamorio::drmemtrace::scheduler_t::range_t roi(static_cast<uint64_t>(MEMTRACE_ROI_BEGIN), static_cast<uint64_t>(MEMTRACE_ROI_END));
    sched_inputs.emplace_back(trace_, std::vector<dynamorio::drmemtrace::scheduler_t::range_t>{roi});
  } else {
    sched_inputs.emplace_back(trace_);
  }

  if (scheduler.init(sched_inputs, 1, dynamorio::drmemtrace::scheduler_t::make_scheduler_serial_options()) !=
      dynamorio::drmemtrace::scheduler_t::STATUS_SUCCESS) {
      panic("failed to initialize scheduler: %s", scheduler.get_error_string().c_str());
      return false;
  }

  // Set info 'A' to the first complete instruction.
  // It will initially lack branch target information.
  getNextInstruction__(&mt_info_a_, &mt_info_b_);
  mt_using_info_a_ = false;
  return true;
}

bool TraceReaderMemtrace::getNextInstruction__(InstInfo* _info,
                                               InstInfo* _prior) {
  static bool first_instr = true;
  uint32_t prior_isize = mt_prior_isize_;
  bool     complete    = false;

  auto *stream = scheduler.get_stream(0);

  if(mt_use_next_ref_) {
    // start with the next entry
    mt_status_ = stream->next_record(mt_ref_);
  } else {
    // mt_use_next_ref_ is false following a REP BUG
    // start with the current entry because it needs to be processed
    // otherwise it will be skipped
    assert(mt_status_ != dynamorio::drmemtrace::scheduler_t::STATUS_EOF);
    assert(mt_state_ == MTState::INST);
    assert(type_is_instr(mt_ref_.instr.type));
    // will use the next entry the next time if not set to false again
    mt_use_next_ref_ = true;
  }
  while(mt_status_ != dynamorio::drmemtrace::scheduler_t::STATUS_EOF) {
    if(mt_status_ != dynamorio::drmemtrace::scheduler_t::STATUS_OK) {
      panic("scheduler failed to advance: %d", mt_status_);
    }
    // there can be mt_ref types other than inst and mem
    // the FSM will skip those if they appear within MTState::INST state
    switch(mt_state_) {
      case(MTState::INST):
        if(type_is_instr(mt_ref_.instr.type)) {
          if(first_instr) {
            // if this is the first instruction ever,
            // the file type marker of the trace should have been processed internally by DynamoRIO.
            // it is time to see if encodings are available.
            auto type = stream->get_filetype();
            trace_has_encodings_ = type & dynamorio::drmemtrace::OFFLINE_FILE_TYPE_ENCODINGS;
            first_instr = false;
          }
          processInst(_info);
          if(mt_mem_ops_ > 0) {
            mt_state_ = MTState::MEM1;
          } else {
            complete = true;
          }
        } else if(mt_ref_.instr.type == dynamorio::drmemtrace::TRACE_TYPE_INSTR_NO_FETCH) {
          // a repeated rep
          bool is_rep = std::get<MAP_REP>(xed_map_.at(_prior->pc));
          assert(is_rep && ((uint32_t)mt_ref_.instr.pid == _prior->pid) &&
                ((uint32_t)mt_ref_.instr.tid == _prior->tid) &&
                (mt_ref_.instr.addr == _prior->pc));
          // do not need to re-process
          *_info = *_prior;
          // flag this instruction as non-fetched
          _info->fetched_instruction = false;
          // mt_mem_ops_ set by the first rep occurance
          if(mt_mem_ops_ > 0) {
            mt_state_ = MTState::MEM1;
          } else {
            complete = true;
          }
        } else if(typeIsMem(mt_ref_.data.type)) {
          // Skip flush and thread exit types and
          // silently ignore memory operands of unknown instructions
          if(!_prior->unknown_type) {
            if(skipped_ == 0) {
              warn("Stray memory record detected at seq. %lu: PC: 0x%lx, "
                    "PID: %lu, TID: %lu, Addr: 0x%lx. "
                    "Suppressing further messages.\n",
                    mt_seq_, mt_ref_.data.pc, mt_ref_.data.pid,
                    mt_ref_.data.tid, mt_ref_.data.addr);
            }
            skipped_++;
          }
        }
        break;
      case(MTState::MEM1):
        if(typeIsMem(mt_ref_.data.type)) {
          if(((uint32_t)_info->pid == mt_ref_.data.pid) &&
             ((uint32_t)_info->tid == mt_ref_.data.tid) &&
             (_info->pc == mt_ref_.data.pc)) {
            _info->mem_addr[0] = mt_ref_.data.addr;
            _info->mem_used[0] = true;
            if(mt_mem_ops_ > 1) {
              mt_state_ = MTState::MEM2;
            } else {
              mt_state_ = MTState::INST;
              complete  = true;
            }
          } else {
            warn("Unexpected PID/TID/PC switch following 0x%lx\n", _info->pc);
            mt_state_ = MTState::INST;
          }
        } else if(type_is_instr(mt_ref_.instr.type)) {
          // REP Instructions with REP count 0
          warn("REP BUG: Data size does not match instruction 0x%lx - PATCHING "
               "size, success!\n",
               _info->pc);

          mt_mem_ops_ = 0;
          mt_state_ = MTState::INST;
          complete  = true;
          mt_use_next_ref_ = false;
          goto PATCH_REP;
        } else {
          warn("Expected data but found type '%s'\n",
               dynamorio::drmemtrace::trace_type_names[mt_ref_.data.type]);
          mt_state_ = MTState::INST;
        }
        break;
      case(MTState::MEM2):
        if(typeIsMem(mt_ref_.data.type)) {
          if(((uint32_t)_info->pid == mt_ref_.data.pid) &&
             ((uint32_t)_info->tid == mt_ref_.data.tid) &&
             (_info->pc == mt_ref_.data.pc)) {
            _info->mem_addr[1] = mt_ref_.data.addr;
            _info->mem_used[1] = true;
            assert(mt_mem_ops_ <= 2);
            mt_state_ = MTState::INST;
            complete  = true;
          } else {
            warn("Unexpected PID/TID/PC switch following 0x%lx\n", _info->pc);
            mt_state_ = MTState::INST;
          }
        } else {
          warn("Expected data2 but found type '%s'\n",
               dynamorio::drmemtrace::trace_type_names[mt_ref_.data.type]);
          mt_state_ = MTState::INST;
        }
        break;
    }
    mt_seq_++;
    if(complete) {
      break;
    }
    // advance to the next entry if the instruction has not yet completed
    mt_status_ = stream->next_record(mt_ref_);
  }
PATCH_REP:
  _info->valid &= complete;
  // Compute the branch target information for the prior instruction
  if(_info->valid) {
    bool is_rep = xed_map_.find(_prior->pc) != xed_map_.end() ? std::get<MAP_REP>(xed_map_.at(_prior->pc)) : 0;
    bool non_seq = _info->pc != (_prior->pc + prior_isize);

    if(_prior->taken) {          // currently set iif branch
      bool new_gid = (_prior->tid != _info->tid) || (_prior->pid != _info->pid);
      if(new_gid) {
        // TODO(granta): If there are enough of these, it may make sense to
        // delay conditional branch instructions until the thread resumes even
        // though this alters the apparent order of the trace.
        // (Seeking ahead to resolve the branch info is a non-starter.)
        if(mt_warn_target_ == 0) {
          warn("Detected a conditional branch preceding a pid/tid change "
              "at seq. %lu. Assuming not-taken. Suppressing further "
              "messages.\n",
              mt_seq_ - 1);
        }
        mt_warn_target_++;
        non_seq = false;
      }
      _prior->taken = non_seq;
    }
    else if (_prior->pc && non_seq && (!is_rep || (_prior->pc != _info->pc && (_prior->pc + prior_isize) != _info->pc))) {
      _prior->ins = createJmp(_info->pc - _prior->pc);
      _prior->target = _info->pc;
      _prior->taken = true;
      _prior->mem_used[0] = false;
      _prior->mem_used[1] = false;
      warn("Patching gap in trace by injecting a Jmp, prior PC: %lx next PC: %lx\n", _prior->pc, _info->pc);
    }
    _prior->target = _info->pc;  // TODO(granta): Invalid for pid/tid switch
  } else {
    // for the last instruction of the trace, the npc cannot be set by the next one
    // set the npc to itself because of the frontend assertion
    // bp_recovery_info->recovery_fetch_addr == frontend_next_fetch_addr(proc_id)
    // at /home/mxu61_bak/scarab_hlitz/src/decoupled_frontend.cc
    _prior->last_inst_from_trace = true;
    _prior->target = _prior->pc;
  }

  return complete;
}

void TraceReaderMemtrace::processInst(InstInfo* _info) {
  // Get the XED info from the cache, creating it if needed
  auto xed_map_iter = xed_map_.find(mt_ref_.instr.addr);
  if(xed_map_iter == xed_map_.end()) {
    if (trace_has_encodings_)
      fillCache(mt_ref_.instr.addr, mt_ref_.instr.size, mt_ref_.instr.encoding);
    else
      fillCache(mt_ref_.instr.addr, mt_ref_.instr.size);
    xed_map_iter = xed_map_.find(mt_ref_.instr.addr);
    assert((xed_map_iter != xed_map_.end()));
  }
  bool                unknown_type, cond_branch;
  xed_decoded_inst_t* xed_ins;
  auto&               xed_tuple = (*xed_map_iter).second;

  tie(mt_mem_ops_, unknown_type, cond_branch, std::ignore,
      std::ignore) = xed_tuple;
  mt_prior_isize_  = mt_ref_.instr.size;
  xed_ins          = std::get<MAP_XED>(xed_tuple).get();

  xed_category_enum_t category = xed_decoded_inst_get_category(xed_ins);
  _info->pc = mt_ref_.instr.addr;
  _info->ins = xed_ins;
  _info->pid = mt_ref_.instr.pid;
  _info->tid = mt_ref_.instr.tid;
  _info->target = 0;  // Set when the next instruction is evaluated
  // Set as taken if it's a branch.
  // Conditional branches are patched when the next instruction is evaluated.
  _info->taken = category == XED_CATEGORY_UNCOND_BR ||
                 category == XED_CATEGORY_COND_BR ||
                 category == XED_CATEGORY_CALL ||
                 category == XED_CATEGORY_RET;
  _info->mem_addr[0]  = 0;
  _info->mem_addr[1]  = 0;
  _info->mem_used[0]  = false;
  _info->mem_used[1]  = false;
  _info->unknown_type = unknown_type;
  // correct this later at getNextInstruction if it is the last instruction
  _info->last_inst_from_trace = false;
  // non-fetched instructions will be set within FSM
  _info->fetched_instruction = true;
}

bool TraceReaderMemtrace::typeIsMem(dynamorio::drmemtrace::trace_type_t _type) {
  return ((_type == dynamorio::drmemtrace::TRACE_TYPE_READ) || (_type == dynamorio::drmemtrace::TRACE_TYPE_WRITE) ||
          type_is_prefetch(_type));
}

const InstInfo* TraceReaderMemtrace::getNextInstruction() {
  InstInfo& info   = (mt_using_info_a_ ? mt_info_a_ : mt_info_b_);
  InstInfo& prior  = (mt_using_info_a_ ? mt_info_b_ : mt_info_a_);
  mt_using_info_a_ = !mt_using_info_a_;
  if(getNextInstruction__(&info, &prior)) {
    return &prior;
  } else if(prior.valid) {
    // the last instruction's npc cannot be set by the trace
    // but it is a valid instruction
    ASSERT(0, !info.valid);
    return &prior;
  } else {
    return &invalid_info_;
  }
}

bool TraceReaderMemtrace::locationForVAddr(uint64_t _vaddr, uint8_t** _loc,
                                           uint64_t* _size) {
  app_pc module_start;
  size_t module_size;

  *_loc = module_mapper_->find_mapped_trace_bounds(
    reinterpret_cast<app_pc>(_vaddr), &module_start, &module_size);
  *_size = reinterpret_cast<uint64_t>(module_size) -
           (reinterpret_cast<uint64_t>(*_loc) -
            reinterpret_cast<uint64_t>(module_start));
  if(!module_mapper_->get_last_error().empty()) {
    std::cout << "Failed to find mapped address: " << std::hex << _vaddr
              << " Error: " << module_mapper_->get_last_error() << std::endl;
    return false;
  }
  return true;
}

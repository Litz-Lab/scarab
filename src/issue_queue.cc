/*
 * Copyright 2026 University of California Santa Cruz
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
 * File         : issue_queue.cc
 * Author       : Yinyuan Zhao, Litz Lab
 * Date         : 4/2026
 * Description  :
 ***************************************************************************************/

#include "issue_queue.h"

extern "C" {
#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "debug/debug.param.h"
#include "debug/debug_macros.h"
#include "debug/debug_print.h"

#include "core.param.h"

#include "bp/bp.h"
#include "memory/memory.h"

#include "exec_ports.h"
#include "exec_stage.h"
#include "map_rename.h"
#include "node_stage.h"
}

#include <algorithm>
#include <deque>
#include <list>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_NODE_STAGE, ##args)

/**************************************************************************************/
/* Constexpr */

enum ISSUE_QUEUE_ENTRY_STATE {
  ISSUE_QUEUE_ENTRY_STATE_EMPTY,
  ISSUE_QUEUE_ENTRY_STATE_ALLOCATED,
  ISSUE_QUEUE_ENTRY_STATE_READY,
  ISSUE_QUEUE_ENTRY_STATE_NUM
};

/**************************************************************************************/
/* Static methods */

static inline Flag issue_queue_check_op_ready(Op* op) {
  ASSERT(node->proc_id, op != nullptr);

  // if the op is waiting for memory, check if it is still blocked by memory. If not, it becomes ready
  if (op->state == OS_WAIT_MEM) {
    if (node->mem_blocked) {
      return FALSE;
    }
    op->state = OS_READY;
  }

  if (op->state == OS_TENTATIVE || op->state == OS_WAIT_DCACHE) {
    return FALSE;
  }
  ASSERT(node->proc_id, op->state == OS_IN_RS || op->state == OS_READY || op->state == OS_WAIT_FWD);

  // if the op is waiting for forwarding, check if it can be forwarded in time to be ready in the next cycle
  if (cycle_count < op_get_rdy_cycle(op) - 1) {
    return FALSE;
  }
  ASSERT(node->proc_id, op_sources_not_rdy_is_clear(op));

  return TRUE;
}

/**************************************************************************************/
/* Structures */

struct IssueQueueEntry {
  const uns16 queue_id;
  const uns16 entry_id;

  Op* op = nullptr;
  uns64 op_fu_type = 0;

  uns32 bound_fu_id = MAX_UNS;
  ISSUE_QUEUE_ENTRY_STATE state = ISSUE_QUEUE_ENTRY_STATE_EMPTY;

  explicit IssueQueueEntry(uns16 queue_id, uns16 entry_id) : queue_id(queue_id), entry_id(entry_id) {}
  void clear();
  void fill(Op* op);
};

void IssueQueueEntry::clear() {
  op = nullptr;
  op_fu_type = 0;
}

void IssueQueueEntry::fill(Op* op) {
  this->op = op;
  op_fu_type = get_fu_type(op->uop->op_type, op->inst->is_simd);
}

/**************************************************************************************/
/* Virtual Interface */

class FunctionalUnitPicker;

/*
 * SchedulePolicy defines the relative priority between ready ops during selection. When
 * multiple ops compete for the same picker, the policy determines which op is preferred.
 *
 * Typical scheduling policies include oldest-first, criticality-based, and dependency-aware
 * prioritization.
 */
class SchedulePolicy {
 public:
  virtual ~SchedulePolicy() = default;

  // Return if the lhs entry is the preference
  virtual bool compare(const IssueQueueEntry* lhs, const IssueQueueEntry* rhs) const = 0;
};

/*
 * TraversalPolicy determines the picker traversal order during serial selection.
 * The selections made by earlier pickers are masked and then forwarded to the later pickers.
 */
class TraversalPolicy {
 public:
  virtual ~TraversalPolicy() = default;

  // Prepare the picker order for the current selection cycle
  virtual void build_picker_order(std::vector<size_t>& picker_order) = 0;
};

/*
 * BindPolicy assigns a functional unit picker to each op during early binding at dispatch. If multiple pickers
 * contain compatible FUs of the same type, the policy selects one to avoid conflicts.
 */
class BindPolicy {
 public:
  virtual ~BindPolicy() = default;
  virtual void bind(const std::vector<FunctionalUnitPicker>& connected_fu_pickers, IssueQueueEntry* entry) = 0;
  virtual void unbind(IssueQueueEntry* entry) = 0;
};

/**************************************************************************************/

/*
 * FunctionalUnitPicker selects a single op for a functional unit during the current
 * scheduling cycle.
 *
 * Each picker tracks the currently selected entry and updates the selection according to
 * the scheduling policy when competing ops are observed.
 *
 * In serial selection, a newly selected op may displace the current selection and forward
 * the displaced request to later pickers.
 *
 * In early binding, ops are bound to a single picker at dispatch, so a ready op only requests
 * the picker it was assigned to.
 */
class FunctionalUnitPicker {
 private:
  const uns proc_id;
  const uns16 queue_id;
  const uns32 fu_id;
  const uns64 fu_type;

  // current selection for this cycle
  IssueQueueEntry* picked_entry = nullptr;

 public:
  explicit FunctionalUnitPicker(uns proc_id, uns16 queue_id, uns32 fu_id, uns64 fu_type)
      : proc_id(proc_id), queue_id(queue_id), fu_id(fu_id), fu_type(fu_type) {}

  void pick(IssueQueueEntry*& candidate, const SchedulePolicy& sched_policy);
  void grant();
  bool is_compatible(uns64 op_fu_type) const;

  uns32 get_fu_id() const { return fu_id; }
  uns64 get_fu_type() const { return fu_type; }
};

/*
 * If the incoming request has higher priority than the current
 * selection, it replaces the existing selection according to the
 * scheduling policy.
 */
void FunctionalUnitPicker::pick(IssueQueueEntry*& request_entry, const SchedulePolicy& sched_policy) {
  ASSERT(proc_id, request_entry != nullptr && request_entry->queue_id == queue_id);
  if (!(picked_entry == nullptr || sched_policy.compare(request_entry, picked_entry))) {
    return;
  }

  // replace the current selection
  IssueQueueEntry* displaced_entry = picked_entry;
  picked_entry = request_entry;

  // forward the displaced request to later pickers
  request_entry = displaced_entry;
}

void FunctionalUnitPicker::grant() {
  // no valid selection this cycle
  if (picked_entry == nullptr) {
    return;
  }
  ASSERT(proc_id, picked_entry->state == ISSUE_QUEUE_ENTRY_STATE_READY);

  Op* op = picked_entry->op;
  ASSERT(proc_id, op != nullptr);
  ASSERT(proc_id, fu_id < (uns32)node->sd.max_op_count);
  op->fu_num = fu_id;

  // issue the op into the stage data
  ASSERT(proc_id, node->sd.ops[fu_id] == nullptr && node->sd.op_count < node->sd.max_op_count);
  node->sd.ops[op->fu_num] = op;
  node->last_scheduled_opnum = op->op_num;
  node->sd.op_count += 1;
  picked_entry = nullptr;
}

bool FunctionalUnitPicker::is_compatible(uns64 op_fu_type) const {
  return op_fu_type & fu_type;
}

/**************************************************************************************/
/*
 * The select logic is responsible for choosing ready ops for issue.
 *
 * It combines a SchedulePolicy, which defines the relative priority between competing ops,
 * a TraversalPolicy, which determines the priority of pickers during selection, and a
 * BindPolicy, which assigns ops to pickers when early binding is enabled.
 */

class SelectLogic {
 protected:
  const uns proc_id;
  const uns16 queue_id;

  std::vector<FunctionalUnitPicker> connected_fu_pickers;
  std::unique_ptr<SchedulePolicy> sched_policy;
  std::unique_ptr<TraversalPolicy> traversal_policy;
  std::unique_ptr<BindPolicy> bind_policy;

  std::vector<size_t> picker_order;  // picker traversal order generated each cycle
  std::list<IssueQueueEntry*> ready_list;

  // bitmask of ready but not yet issued op types in this cycle
  uns64 ready_not_issued_op_types = 0;
  std::vector<uns64> ready_not_issued_op_types_per_fu;

 public:
  explicit SelectLogic(uns proc_id, uns16 queue_id, std::vector<FunctionalUnitPicker> connected_fu_pickers,
                       std::unique_ptr<SchedulePolicy> sched_policy, std::unique_ptr<TraversalPolicy> traversal_policy,
                       std::unique_ptr<BindPolicy> bind_policy)
      : proc_id(proc_id),
        queue_id(queue_id),
        connected_fu_pickers(std::move(connected_fu_pickers)),
        sched_policy(std::move(sched_policy)),
        traversal_policy(std::move(traversal_policy)),
        bind_policy(std::move(bind_policy)),
        picker_order(this->connected_fu_pickers.size()) {}

  void bid();
  void grant(uns64 ready_not_issued_op_types_others);

  void bind(IssueQueueEntry* entry) { bind_policy->bind(connected_fu_pickers, entry); }
  void unbind(IssueQueueEntry* entry) { bind_policy->unbind(entry); }
  void request(IssueQueueEntry* entry) { ready_list.push_front(entry); }
  void release(IssueQueueEntry* entry) { ready_list.remove(entry); }

  bool has_ready_ops() const { return !ready_list.empty(); }
  uns64 get_ready_not_issued_op_types() const { return ready_not_issued_op_types; }

  void collect_entry_op_ready_stats(IssueQueueEntry* entry);
  void collect_entry_op_ready_not_issued_stats(IssueQueueEntry* entry);
};

/*
 * Serial selection is described in "Quantifying the complexity of superscalar processors", 1996.
 * The select logic consists of multiple picking blocks arranged in series and each block receives
 * request signals that are derived from the requests of the preceding block.
 *
 * When early bind is enabled, each dispatched op is bound to one compatible picker.
 */

// pick ready ops for issue according to the scheduling policy and picker traversal order
void SelectLogic::bid() {
  traversal_policy->build_picker_order(picker_order);
  ready_not_issued_op_types = 0;
  ready_not_issued_op_types_per_fu.assign(connected_fu_pickers.size(), 0);

  for (IssueQueueEntry* entry : ready_list) {
    // check if the op is ready (it may become not ready due to memory blocking or waiting for forwarding)
    if (entry->state != ISSUE_QUEUE_ENTRY_STATE_READY || !issue_queue_check_op_ready(entry->op)) {
      continue;
    }

    // the current request propagated through the serial picker chain
    IssueQueueEntry* request_entry = entry;
    collect_entry_op_ready_stats(request_entry);

    for (size_t i = 0; i < connected_fu_pickers.size(); ++i) {
      size_t picker_idx = picker_order[i];
      FunctionalUnitPicker& fu_picker = connected_fu_pickers[picker_idx];

      if (!fu_picker.is_compatible(request_entry->op_fu_type)) {
        continue;
      }

      // if early binding is enabled, the request can only be picked by the bound picker
      if (entry->bound_fu_id != MAX_UNS && entry->bound_fu_id != picker_idx) {
        continue;
      }

      fu_picker.pick(request_entry, *sched_policy);

      // the request has been consumed
      if (request_entry == nullptr) {
        break;
      }
    }

    // the entry is not picked or displaced
    if (request_entry != nullptr) {
      ready_not_issued_op_types |= request_entry->op_fu_type;
      if (request_entry->bound_fu_id != MAX_UNS) {
        ready_not_issued_op_types_per_fu[request_entry->bound_fu_id] |= request_entry->op_fu_type;
      }

      collect_entry_op_ready_not_issued_stats(request_entry);
    }
  }
}

// grant the picked ops into issue ports
void SelectLogic::grant(uns64 ready_not_issued_op_types_others) {
  for (size_t i = 0; i < connected_fu_pickers.size(); ++i) {
    // grant the pick after scanning the ready list
    FunctionalUnitPicker& fu_picker = connected_fu_pickers[picker_order[i]];
    fu_picker.grant();

    // track FU idle stats after scheduling
    uns32 fu_id = fu_picker.get_fu_id();
    if (node->sd.ops[fu_id] != NULL)
      continue;

    bool matching_unpick = false;
    if (ready_not_issued_op_types_others & fu_picker.get_fu_type()) {
      STAT_EVENT(proc_id, ISSUE_QUEUE_MATCHING_UNPICK_ACROSS_QUEUES);
      matching_unpick = true;
    }

    for (size_t j = 0; j < connected_fu_pickers.size(); ++j) {
      if (ready_not_issued_op_types_per_fu[j] & fu_picker.get_fu_type()) {
        STAT_EVENT(proc_id, ISSUE_QUEUE_MATCHING_UNPICK_WITHIN_QUEUE);
        matching_unpick = true;
        break;
      }
    }

    if (matching_unpick) {
      STAT_EVENT(proc_id, ISSUE_QUEUE_MATCHING_UNPICK);
    }
  }
}

void SelectLogic::collect_entry_op_ready_stats(IssueQueueEntry* entry) {
  STAT_EVENT(node->proc_id, RS_OP_READY);

  Op* op = entry->op;
  if (op->off_path) {
    STAT_EVENT(op->proc_id, ST_OP_OFFPATH_READY);
    STAT_EVENT(op->proc_id, ST_NOT_MEM_OFFPATH_READY + op->uop->mem_type);
    return;
  }

  STAT_EVENT(op->proc_id, ST_OP_ONPATH_READY);
  STAT_EVENT(op->proc_id, ST_OP_INV_READY + op->uop->op_type);
  STAT_EVENT(op->proc_id, ST_NOT_CF_READY + op->uop->cf_type);
  STAT_EVENT(op->proc_id, ST_BAR_NONE_READY + op->uop->bar_type);
  STAT_EVENT(op->proc_id, ST_NOT_MEM_READY + op->uop->mem_type);
}

void SelectLogic::collect_entry_op_ready_not_issued_stats(IssueQueueEntry* entry) {
  STAT_EVENT(node->proc_id, RS_OP_READY_NOT_ISSUED_TOTAL);
  STAT_EVENT(node->proc_id, RS_0_OP_READY_NOT_ISSUED + (queue_id < 8 ? queue_id : 8));

  Op* op = entry->op;
  if (op->off_path) {
    STAT_EVENT(op->proc_id, ST_OP_OFFPATH_READY_NOT_ISSUED);
    STAT_EVENT(op->proc_id, ST_NOT_MEM_OFFPATH_READY_NOT_ISSUED + op->uop->mem_type);
    return;
  }

  STAT_EVENT(op->proc_id, ST_OP_ONPATH_READY_NOT_ISSUED);
  STAT_EVENT(op->proc_id, ST_OP_INV_READY_NOT_ISSUED + op->uop->op_type);
  STAT_EVENT(op->proc_id, ST_NOT_CF_READY_NOT_ISSUED + op->uop->cf_type);
  STAT_EVENT(op->proc_id, ST_BAR_NONE_READY_NOT_ISSUED + op->uop->bar_type);
  STAT_EVENT(op->proc_id, ST_NOT_MEM_READY_NOT_ISSUED + op->uop->mem_type);
}

/**************************************************************************************/

/*
 * OldestFirstSchedulePolicy prioritize the older ops
 */
class OldestFirstSchedulePolicy : public SchedulePolicy {
 public:
  bool compare(const IssueQueueEntry* lhs, const IssueQueueEntry* rhs) const override {
    return lhs->op->op_num < rhs->op->op_num;
  }
};

/*
 * AMDBulldozerSchedulePolicy is introduced in "40-Entry Unified Out-of-Order Scheduler
 * and Integer Execution Unit for the AMD Bulldozer x86-64 Core", ISSCC 2011.
 * The oldest ready instruction will be picked first. Otherwise, a priority encoder scans ready
 * instructions in physical order.
 */
class AMDBulldozerSchedulePolicy : public SchedulePolicy {
 public:
  bool compare(const IssueQueueEntry* lhs, const IssueQueueEntry* rhs) const override {
    if (lhs->op->op_num == node->node_head->op_num)
      return true;

    if (rhs->op->op_num == node->node_head->op_num)
      return false;

    return lhs->entry_id < rhs->entry_id;
  }
};

/*
 * RandomSchedulePolicy selects ready ops in their physical order in the issue queue
 */
class RandomSchedulePolicy : public SchedulePolicy {
 public:
  bool compare(const IssueQueueEntry* lhs, const IssueQueueEntry* rhs) const override {
    return lhs->entry_id < rhs->entry_id;
  }
};

/**************************************************************************************/

// RoundRobinTraversalPolicy ensures fairness by rotating through pickers in a circular manner.
class RoundRobinTraversalPolicy : public TraversalPolicy {
 private:
  const size_t fu_num;
  int rr_idx = 0;  // circular queue idx for round-robin selection

 public:
  explicit RoundRobinTraversalPolicy(size_t fu_num) : fu_num(fu_num), rr_idx(fu_num - 1) {}
  void build_picker_order(std::vector<size_t>& picker_order) override {
    ASSERT(node->proc_id, picker_order.size() == fu_num);
    rr_idx = (rr_idx + 1) % fu_num;
    for (size_t i = 0; i < fu_num; ++i) {
      picker_order[i] = (rr_idx + i) % fu_num;
    }
  }
};

class PriorityTraversalPolicy : public TraversalPolicy {
 private:
  std::vector<size_t> priority_order;

 public:
  explicit PriorityTraversalPolicy(const std::string& priority_str, size_t fu_num) {
    std::istringstream ss(priority_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
      size_t idx = strtoull(token.c_str(), nullptr, 10);
      ASSERT(node->proc_id, idx < fu_num);
      ASSERT(node->proc_id, std::find(priority_order.begin(), priority_order.end(), idx) == priority_order.end());
      priority_order.push_back(idx);
    }
    ASSERT(node->proc_id, priority_order.size() == fu_num);
  }

  void build_picker_order(std::vector<size_t>& picker_order) override {
    ASSERT(node->proc_id, picker_order.size() == priority_order.size());
    for (size_t i = 0; i < priority_order.size(); ++i) {
      picker_order[i] = priority_order[i];
    }
  }
};

/**************************************************************************************/

/*
 * NoneBindPolicy does not perform early binding
 * Ops can be picked by any compatible picker during selection.
 */
class NoneBindPolicy : public BindPolicy {
 public:
  void bind(const std::vector<FunctionalUnitPicker>& connected_fu_pickers, IssueQueueEntry* entry) override {}
  void unbind(IssueQueueEntry* entry) override {}
};

/*
 * LeastBindPolicy is described in "Reconstructing Out-of-Order Issue Queue", MICRO 2022.
 * It binds ops to the compatible picker with the fewest assigned ops to avoid conflicts.
 */
class LeastBindPolicy : public BindPolicy {
 private:
  std::vector<size_t> bound_op_counts;

 public:
  explicit LeastBindPolicy(size_t fu_num) : bound_op_counts(fu_num, 0) {}

  void bind(const std::vector<FunctionalUnitPicker>& connected_fu_pickers, IssueQueueEntry* entry) override {
    ASSERT(node->proc_id, entry != nullptr);
    ASSERT(node->proc_id, entry->bound_fu_id == MAX_UNS);
    ASSERT(node->proc_id, connected_fu_pickers.size() == bound_op_counts.size());

    size_t least_picker_idx = MAX_UNS;
    size_t least_bound_ops = 0;

    for (size_t i = 0; i < connected_fu_pickers.size(); ++i) {
      const FunctionalUnitPicker& fu_picker = connected_fu_pickers[i];
      if (!fu_picker.is_compatible(entry->op_fu_type)) {
        continue;
      }

      if (least_picker_idx == MAX_UNS || bound_op_counts[i] < least_bound_ops) {
        least_picker_idx = i;
        least_bound_ops = bound_op_counts[i];
      }
    }

    ASSERT(node->proc_id, least_picker_idx != MAX_UNS);
    entry->bound_fu_id = least_picker_idx;
    bound_op_counts[least_picker_idx] += 1;
  }

  void unbind(IssueQueueEntry* entry) override {
    ASSERT(node->proc_id, entry != nullptr);
    if (entry->bound_fu_id == MAX_UNS) {
      return;
    }

    ASSERT(node->proc_id, entry->bound_fu_id < bound_op_counts.size());
    ASSERT(node->proc_id, bound_op_counts[entry->bound_fu_id] > 0);
    bound_op_counts[entry->bound_fu_id] -= 1;
    entry->bound_fu_id = MAX_UNS;
  }
};

/**************************************************************************************/
/* Factory */
// TODO: abstract factory

class IssueQueuePolicyFactory {
 public:
  std::unique_ptr<SchedulePolicy> make_schedule_policy() {
    using SchedulePolicyMaker = std::unique_ptr<SchedulePolicy> (*)();
    static const SchedulePolicyMaker schedule_policy_makers[ISSUE_QUEUE_SCHEDULE_POLICY_NUM] = {
        []() -> std::unique_ptr<SchedulePolicy> { return std::make_unique<OldestFirstSchedulePolicy>(); },
        []() -> std::unique_ptr<SchedulePolicy> { return std::make_unique<AMDBulldozerSchedulePolicy>(); },
        []() -> std::unique_ptr<SchedulePolicy> { return std::make_unique<RandomSchedulePolicy>(); },
    };

    ASSERT(node->proc_id, ISSUE_QUEUE_SCHEDULE_POLICY < ISSUE_QUEUE_SCHEDULE_POLICY_NUM);
    return schedule_policy_makers[ISSUE_QUEUE_SCHEDULE_POLICY]();
  }

  std::unique_ptr<TraversalPolicy> make_traversal_policy(uns16 queue_id, size_t fu_num) {
    using TraversalPolicyMaker = std::unique_ptr<TraversalPolicy> (*)(uns16, size_t);
    static const TraversalPolicyMaker traversal_policy_makers[ISSUE_QUEUE_TRAVERSAL_POLICY_NUM] = {
        [](uns16, size_t fu_num) -> std::unique_ptr<TraversalPolicy> {
          return std::make_unique<RoundRobinTraversalPolicy>(fu_num);
        },
        [](uns16 queue_id, size_t fu_num) -> std::unique_ptr<TraversalPolicy> {
          if (!ISSUE_QUEUE_TRAVERSAL_PRIORITY || !*ISSUE_QUEUE_TRAVERSAL_PRIORITY)
            return std::make_unique<RoundRobinTraversalPolicy>(fu_num);

          std::istringstream ss(ISSUE_QUEUE_TRAVERSAL_PRIORITY);
          std::string token;
          for (size_t i = 0; i <= queue_id && std::getline(ss, token, ';'); ++i) {
            if (i == queue_id && token != "-" && !token.empty())
              return std::make_unique<PriorityTraversalPolicy>(token, fu_num);
          }
          return std::make_unique<RoundRobinTraversalPolicy>(fu_num);
        },
    };

    ASSERT(node->proc_id, ISSUE_QUEUE_TRAVERSAL_POLICY < ISSUE_QUEUE_TRAVERSAL_POLICY_NUM);
    return traversal_policy_makers[ISSUE_QUEUE_TRAVERSAL_POLICY](queue_id, fu_num);
  }

  std::unique_ptr<BindPolicy> make_bind_policy(size_t fu_num) {
    using BindPolicyMaker = std::unique_ptr<BindPolicy> (*)(size_t);
    static const BindPolicyMaker bind_policy_makers[ISSUE_QUEUE_EARLY_BIND_POLICY_NUM] = {
        [](size_t) -> std::unique_ptr<BindPolicy> { return std::make_unique<NoneBindPolicy>(); },
        [](size_t fu_num) -> std::unique_ptr<BindPolicy> { return std::make_unique<LeastBindPolicy>(fu_num); },
    };

    ASSERT(node->proc_id, ISSUE_QUEUE_EARLY_BIND_POLICY < ISSUE_QUEUE_EARLY_BIND_POLICY_NUM);
    return bind_policy_makers[ISSUE_QUEUE_EARLY_BIND_POLICY](fu_num);
  }
};

/**************************************************************************************/
/*
 * The issue queue is organized as a random queue, where instructions are dispatched into free entries
 * without any particular ordering.
 */

class IssueQueue {
 private:
  const uns proc_id;
  const uns16 queue_id;

  std::vector<IssueQueueEntry> entries;
  std::deque<uns16> free_list;
  std::unique_ptr<SelectLogic> select_logic;

 public:
  explicit IssueQueue(uns proc_id, uns16 queue_id, uns16 size, std::vector<FunctionalUnitPicker> connected_fu_pickers);
  uns16 allocate_entry(Op* op);
  void free_entry(uns16 entry_id);
  size_t available_entries() const { return free_list.size(); }
  const std::vector<IssueQueueEntry>& get_entries() const { return entries; }

  void wakeup(uns16 entry_id);
  void issued(uns16 entry_id);
  void recover();

  void bid() { select_logic->bid(); }
  void grant(uns64 ready_not_issued_op_types_others) { select_logic->grant(ready_not_issued_op_types_others); }

  uns64 get_ready_not_issued_op_types() const { return select_logic->get_ready_not_issued_op_types(); }
  bool has_ready_ops() const { return select_logic->has_ready_ops(); }
};

IssueQueue::IssueQueue(uns proc_id, uns16 queue_id, uns16 size, std::vector<FunctionalUnitPicker> connected_fu_pickers)
    : proc_id(proc_id), queue_id(queue_id) {
  // initialize entries and free list
  entries.reserve(size);
  for (uns16 i = 0; i < size; ++i) {
    entries.emplace_back(queue_id, i);
    free_list.push_back(i);
  }
  DEBUG(proc_id, "Initializing Issue Queue %d with size %d\n", queue_id, size);

  // initialize select logic from the scheduling, traversal, and binding policies.
  IssueQueuePolicyFactory factory;
  size_t fu_num = connected_fu_pickers.size();
  select_logic =
      std::make_unique<SelectLogic>(proc_id, queue_id, std::move(connected_fu_pickers), factory.make_schedule_policy(),
                                    factory.make_traversal_policy(queue_id, fu_num), factory.make_bind_policy(fu_num));
}

uns16 IssueQueue::allocate_entry(Op* op) {
  // get an available entry from the free list
  ASSERT(proc_id, !free_list.empty());
  uns16 entry_id = free_list.front();
  free_list.pop_front();

  // fill the op info into the entry
  IssueQueueEntry& entry = entries[entry_id];
  ASSERT(proc_id, entry.state == ISSUE_QUEUE_ENTRY_STATE_EMPTY);
  entry.fill(op);
  entry.state = ISSUE_QUEUE_ENTRY_STATE_ALLOCATED;

  // bind the entry to a picker during dispatching when early bind is enabled
  select_logic->bind(&entry);

  return entry_id;
}

void IssueQueue::free_entry(uns16 entry_id) {
  IssueQueueEntry& entry = entries[entry_id];
  ASSERT(proc_id, entry.state != ISSUE_QUEUE_ENTRY_STATE_EMPTY);

  select_logic->unbind(&entry);

  entry.clear();
  entry.state = ISSUE_QUEUE_ENTRY_STATE_EMPTY;
  free_list.push_back(entry_id);
}

void IssueQueue::wakeup(uns16 entry_id) {
  ASSERT(proc_id, entry_id < entries.size());
  IssueQueueEntry& entry = entries[entry_id];
  ASSERT(proc_id, entry.state == ISSUE_QUEUE_ENTRY_STATE_ALLOCATED);

  Op* op = entry.op;
  ASSERT(proc_id, op != nullptr);

  op->in_rdy_list = TRUE;
  entry.state = ISSUE_QUEUE_ENTRY_STATE_READY;
  select_logic->request(&entry);
}

void IssueQueue::issued(uns16 entry_id) {
  ASSERT(proc_id, entry_id < entries.size());
  IssueQueueEntry& entry = entries[entry_id];
  ASSERT(proc_id, entry.state == ISSUE_QUEUE_ENTRY_STATE_READY && entry.queue_id == queue_id);

  Op* op = entry.op;
  ASSERT(proc_id, op != nullptr);

  STAT_EVENT(node->proc_id, OP_ISSUED);
  op->in_rdy_list = FALSE;
  select_logic->release(&entry);
  free_entry(entry.entry_id);
}

void IssueQueue::recover() {
  for (IssueQueueEntry& entry : entries) {
    Op* op = entry.op;
    if (op == nullptr || !op->off_path) {
      continue;
    }

    if (!FLUSH_OP(op)) {
      continue;
    }

    op->in_rdy_list = FALSE;
    select_logic->release(&entry);
    free_entry(entry.entry_id);
  }
}

/**************************************************************************************/

class IssueQueues {
 private:
  const uns proc_id;
  std::vector<IssueQueue> issue_queues;
  std::vector<uns16> fu_map;
  std::vector<uns64> fu_types;

  void update_mem_block();
  uns16 find_emptiest_queue(Op* op);

 public:
  explicit IssueQueues(uns proc_id);

  void dispatch();
  void schedule();
  void recover();

  void wakeup(Op* op);
  void issued(Op* op);
  bool has_ready_ops() const;
};

IssueQueues::IssueQueues(uns proc_id) : proc_id(proc_id) {
  auto split_tokens = [](const std::string& s) {
    std::vector<std::string> tokens;
    for (size_t pos = 0; pos < s.size();) {
      size_t next = s.find(' ', pos);
      tokens.emplace_back(s.substr(pos, next == std::string::npos ? s.size() - pos : next - pos));
      if (next == std::string::npos)
        break;
      pos = next + 1;
    }
    return tokens;
  };
  auto parse_mask = [](const std::string& s) -> uns64 {
    ASSERT(0, !s.empty());
    switch (s.front()) {
      case 'x':
        return static_cast<uns64>(std::stoull(s.substr(1), nullptr, 16));
      case 'b':
        return static_cast<uns64>(std::stoull(s.substr(1), nullptr, 2));
      default:
        return static_cast<uns64>(std::stoull(s, nullptr, 10));
    }
  };

  // initialize connection map from FUs to issue queues
  std::vector<std::string> connection_tokens = split_tokens(RS_CONNECTIONS);
  ASSERT(proc_id, connection_tokens.size() == NUM_RS);
  fu_map.assign(NUM_FUS, MAX_UNS16);
  for (size_t i = 0; i < NUM_RS; ++i) {
    for (uns64 mask = parse_mask(connection_tokens[i]); mask; mask &= (mask - 1)) {
      size_t fu_id = __builtin_ctzll(mask);
      ASSERT(proc_id, fu_id < fu_map.size() && fu_map[fu_id] == MAX_UNS16);
      fu_map[fu_id] = i;
    }
  }

  // initialize the connected FUs for each issue queue
  std::vector<std::string> fu_tokens = split_tokens(FU_TYPES);
  ASSERT(proc_id, fu_tokens.size() == NUM_FUS);
  std::vector<std::vector<FunctionalUnitPicker>> fu_connection(NUM_RS);
  fu_types.assign(NUM_RS, 0);
  std::vector<Flag> connected_to_int(NUM_RS, FALSE);
  std::vector<Flag> connected_to_fp(NUM_RS, FALSE);
  for (size_t i = 0; i < NUM_FUS; ++i) {
    uns16 queue_id = fu_map[i];
    ASSERT(proc_id, queue_id != MAX_UNS16);
    uns64 fu_type = parse_mask(fu_tokens[i]);
    fu_connection[queue_id].emplace_back(proc_id, queue_id, i, fu_type);
    fu_types[queue_id] |= fu_type;
    DEBUG(proc_id, "FU %ld, queue: %d, type: 0x%llx\n", i, queue_id, fu_type);

    connected_to_int[queue_id] |= is_alu_type(fu_type) || is_mul_or_div_type(fu_type);
    connected_to_fp[queue_id] |= is_fpu_type(fu_type);
  }

  // create issue queues based on configuration
  issue_queues.reserve(NUM_RS);
  POWER_TOTAL_RS_SIZE = 0;
  POWER_TOTAL_INT_RS_SIZE = 0;
  POWER_TOTAL_FP_RS_SIZE = 0;
  const char* p = RS_SIZES;
  for (uns16 i = 0; i < NUM_RS; ++i) {
    char* end = nullptr;
    uns64 size = strtoull(p, &end, 10);
    p = end;

    issue_queues.emplace_back(proc_id, i, size, std::move(fu_connection[i]));

    POWER_TOTAL_RS_SIZE += size;
    POWER_TOTAL_INT_RS_SIZE += connected_to_int[i] ? size : 0;
    POWER_TOTAL_FP_RS_SIZE += connected_to_fp[i] ? size : 0;
  }
}

/*
 * Fill the scheduling window (RS) with oldest available ops.
 * For each available op:
 *  - Allocate it to its designated reservation station.
 *  - If all source operands are ready, insert it into the ready list.
 */
void IssueQueues::dispatch() {
  Op* op = NULL;
  uns32 num_fill_rs = 0;

  for (op = node->next_op_into_rs; op; op = op->next_node) {
    ASSERT(proc_id, op->queue_id == MAX_UNS16 && op->queue_entry_id == MAX_UNS16);
    uns16 queue_id = find_emptiest_queue(op);
    if (queue_id == MAX_UNS16) {
      break;
    }

    IssueQueue& queue = issue_queues[queue_id];
    ASSERT(proc_id, queue.available_entries() > 0);
    op->queue_id = queue_id;
    op->queue_entry_id = queue.allocate_entry(op);
    num_fill_rs++;

    ASSERT(node->proc_id, op->state == OS_IN_ROB);
    op->state = OS_IN_RS;

    if (op_sources_not_rdy_is_clear(op)) {
      queue.wakeup(op->queue_entry_id);
      op->state = (cycle_count + 1 >= op_get_rdy_cycle(op) ? OS_READY : OS_WAIT_FWD);
    }

    // maximum number of operations to fill into the RS per cycle (0 = unlimited)
    if (RS_FILL_WIDTH && (num_fill_rs == RS_FILL_WIDTH)) {
      op = op->next_node;
      break;
    }
  }

  // mark the next node to continue filling in the next cycle
  node->next_op_into_rs = op;
}

/*
 * Schedule ready ops (ops that are currently in the ready list).
 *
 * Input:  ready_list, containing all ops that are ready to issue from each of the RSs.
 * Output: node->sd, containing ops thats being passed to the FUs.
 *
 * If a functional unit is available, it will accept the scheduled op,
 * which is then removed from the ready list. If no FU is available, the
 * op remains in the ready list to be considered in the next
 * scheduling cycle.
 */
void IssueQueues::schedule() {
  // the next stage is supposed to clear them out
  ASSERT(node->proc_id, node->sd.op_count == 0);

  // checks if any of the L1 MSHRs have become available
  update_mem_block();

  std::vector<uns64> ready_not_issued_op_types_total;
  for (IssueQueue& queue : issue_queues) {
    queue.bid();
    ready_not_issued_op_types_total.push_back(queue.get_ready_not_issued_op_types());
  }

  for (size_t queue_id = 0; queue_id < issue_queues.size(); ++queue_id) {
    uns64 ready_not_issued_op_types_others = 0;
    for (size_t other_queue_id = 0; other_queue_id < issue_queues.size(); ++other_queue_id) {
      if (other_queue_id != queue_id) {
        ready_not_issued_op_types_others |= ready_not_issued_op_types_total[other_queue_id];
      }
    }

    issue_queues[queue_id].grant(ready_not_issued_op_types_others);
  }
}

void IssueQueues::recover() {
  for (IssueQueue& queue : issue_queues) {
    queue.recover();
  }
}

void IssueQueues::wakeup(Op* op) {
  uns16 queue_id = op->queue_id;
  ASSERT(proc_id, queue_id < issue_queues.size());
  IssueQueue& queue = issue_queues[queue_id];
  ASSERT(proc_id, op->queue_entry_id != MAX_UNS16);
  queue.wakeup(op->queue_entry_id);
}

void IssueQueues::issued(Op* op) {
  uns16 queue_id = op->queue_id;
  ASSERT(proc_id, queue_id < issue_queues.size());
  IssueQueue& queue = issue_queues[queue_id];
  ASSERT(proc_id, op->queue_entry_id != MAX_UNS16);
  queue.issued(op->queue_entry_id);
}

bool IssueQueues::has_ready_ops() const {
  for (const IssueQueue& queue : issue_queues) {
    if (queue.has_ready_ops()) {
      return true;
    }
  }
  return false;
}

// Memory is blocked when there are no more MSHRs in the L1 Q (i.e., there is no way to handle a D-Cache miss)
void IssueQueues::update_mem_block() {
  // if we are stalled due to lack of MSHRs to the L1, check to see if there is space now
  if (node->mem_blocked && mem_can_admit_req(node->proc_id, MRT_DFETCH)) {
    node->mem_blocked = FALSE;
    STAT_EVENT(node->proc_id, MEM_BLOCK_LENGTH_0 + MIN2(node->mem_block_length, 5000) / 100);
    node->mem_block_length = 0;
  }

  INC_STAT_EVENT(node->proc_id, CORE_MEM_BLOCKED, node->mem_blocked);
  node->mem_block_length += node->mem_blocked;
}

uns16 IssueQueues::find_emptiest_queue(Op* op) {
  uns16 emptiest_queue_id = MAX_UNS16;
  size_t emptiest_queue_entries = 0;

  uns64 op_fu_type = get_fu_type(op->uop->op_type, op->inst->is_simd);
  for (size_t queue_id = 0; queue_id < issue_queues.size(); ++queue_id) {
    const IssueQueue& queue = issue_queues[queue_id];
    uns64 queue_fu_types = fu_types[queue_id];
    if (!(op_fu_type & queue_fu_types)) {
      continue;
    }

    size_t empty_entries = queue.available_entries();
    if (empty_entries == 0) {
      continue;
    }

    if (emptiest_queue_entries < empty_entries) {
      emptiest_queue_id = queue_id;
      emptiest_queue_entries = empty_entries;
    }
  }

  return emptiest_queue_id;
}

/**************************************************************************************/
/* Global Values */

static std::vector<IssueQueues> per_core_issue_queues;
IssueQueues* issue_queues = nullptr;

/**************************************************************************************/
/* External Function */

void issue_queue_update() {
  // dispatch ops from the reorder buffer to the corresponding issue queues
  issue_queues->dispatch();

  // schedule ready ops in the issue queue to the functional units
  issue_queues->schedule();
}

void issue_queue_wakeup(Op* op) {
  // wake up ops and insert them into the ready list for scheduling
  issue_queues->wakeup(op);
}

void issue_queue_issued(Op* op) {
  // release the entries of ops that are scheduled
  issue_queues->issued(op);
}

Flag issue_queue_has_ready_ops() {
  return issue_queues->has_ready_ops();
}

/**************************************************************************************/
/* Vanilla API */

void alloc_mem_issue_queue(uns num_cores) {
  per_core_issue_queues.reserve(num_cores);
  for (uns ii = 0; ii < num_cores; ii++) {
    per_core_issue_queues.emplace_back(ii);
  }
}

void set_issue_queue(uns8 proc_id) {
  issue_queues = &per_core_issue_queues[proc_id];
}

void recover_issue_queue() {
  ASSERT(node->proc_id, issue_queues != nullptr);
  issue_queues->recover();
}

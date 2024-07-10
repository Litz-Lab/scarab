// CPP implementation of a cache.

#include "libs/cpp_cache.h"
#include <iostream>
#include <list>
#include <unordered_map>
#include <vector>
#include "debug/debug_macros.h"
#include "debug/debug_print.h"

extern "C" {
#include "globals/assert.h"
#include "globals/utils.h"
#include "globals/global_vars.h"
#include "memory/memory.param.h"
}

#define CPPC_DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_CPP_CACHE, ##args)

template <typename User_Key_Type, typename User_Data_Type>
struct Entry {
  Flag valid;
  User_Key_Type key;
  User_Data_Type data;
  // for LRU replacement policy
  Counter accessed_cycle;
};

template <typename User_Key_Type, typename User_Data_Type>
class Set {
 public:
  std::vector<Entry<User_Key_Type, User_Data_Type>> entries;
  // for round-robin replacement policy
  uns next_evict;
};

template <typename User_Key_Type, typename User_Data_Type>
class Cpp_Cache {
 protected:
  std::vector<Set<User_Key_Type, User_Data_Type>> sets;
  Repl_Policy      repl_policy;
  uns              assoc;
  uns              num_sets;
  uns              line_bytes;

  // defines how to hash the key to the set index, need to be implemented by the user
  virtual uns set_idx_hash(User_Key_Type key) = 0;

  // replacement policy functions
  virtual void update_repl_states(Set<User_Key_Type, User_Data_Type>& set, uns hit_idx);
  virtual uns get_repl_idx(Set<User_Key_Type, User_Data_Type>& set);

 public:
  Cpp_Cache() = default;
  Cpp_Cache(uns nl, uns asc, uns lb, Repl_Policy rp) {
    assoc           = asc;
    num_sets        = nl / assoc;
    line_bytes      = lb;
    repl_policy     = rp;

    sets            = std::vector<Set<User_Key_Type, User_Data_Type>>(num_sets);
    for (uns i = 0; i < num_sets; i++) {
      sets[i].entries.resize(assoc);
    }
  }

  User_Data_Type* access(User_Key_Type key, bool update_repl);
  Entry<User_Key_Type, User_Data_Type> insert(User_Key_Type key, User_Data_Type data);
  Entry<User_Key_Type, User_Data_Type> invalidate(User_Key_Type key);
};

template <typename User_Key_Type, typename User_Data_Type>
void Cpp_Cache<User_Key_Type, User_Data_Type>::update_repl_states(Set<User_Key_Type, User_Data_Type>& set, uns hit_idx) {
  switch(repl_policy) {
    case REPL_TRUE_LRU: {
      set.entries[hit_idx].accessed_cycle = cycle_count;
    } break;
    case REPL_RANDOM: {
    } break;
    case REPL_ROUND_ROBIN: {
    } break;
    default:
      ASSERT(0, FALSE);  // unsupported
  }
}

template <typename User_Key_Type, typename User_Data_Type>
uns Cpp_Cache<User_Key_Type, User_Data_Type>::get_repl_idx(Set<User_Key_Type, User_Data_Type>& set) {
  // if there are invalid entries, replace them
  for (uns i = 0; i < assoc; i++) {
    Entry<User_Key_Type, User_Data_Type> entry = set.entries[i];
    if (!entry.valid) {
      return i;
    }
  }

  // otherwise, replace according to the policy
  uns repl_idx = 0;
  switch(repl_policy) {
    case REPL_TRUE_LRU: {
      Counter lru_cycle = std::numeric_limits<Counter>::max();
      for (uns i = 0; i < assoc; i++) {
        Entry<User_Key_Type, User_Data_Type> entry = set.entries[i];
        // find smallest access cycle
        if (entry.accessed_cycle < lru_cycle) {
          repl_idx = i;
          lru_cycle = entry.accessed_cycle;
        }
      }
    } break;
    case REPL_RANDOM: {
      repl_idx = rand() % assoc;
    } break;
    case REPL_ROUND_ROBIN: {
      repl_idx = (set.next_evict + 1) % assoc;
      set.next_evict = repl_idx;
    } break;
    default:
      ASSERT(0, FALSE);  // unsupported
  }
  return repl_idx;
}

// access: Looks up the cache based on key. Returns pointer to line data if found
template <typename User_Key_Type, typename User_Data_Type>
User_Data_Type* Cpp_Cache<User_Key_Type, User_Data_Type>::access(User_Key_Type key, bool update_repl) {
  User_Data_Type* data = NULL;
  uns set_idx = set_idx_hash(key);
  for (uns i = 0; i < assoc; i++) {
    Entry<User_Key_Type, User_Data_Type>& entry = sets[set_idx].entries[i];
    if (entry.valid && entry.key == key) {  // hit
      data = &entry.data;
      if (update_repl) {
        update_repl_states(sets[set_idx], i);
      }
      break;
    }
  }
  return data;
}

template <typename User_Key_Type, typename User_Data_Type>
Entry<User_Key_Type, User_Data_Type> Cpp_Cache<User_Key_Type, User_Data_Type>::insert(User_Key_Type key, User_Data_Type data) {
  // first check if line exists
  ASSERT(0, access(key, FALSE) == NULL);

  uns set_idx = set_idx_hash(key);
  // if the set is full, repl_idx will be overwritten;
  // if the set has vacancy, repl_idx will point to an invalid line
  uns repl_idx = get_repl_idx(sets[set_idx]);

  Entry<User_Key_Type, User_Data_Type> evicted_entry = sets[set_idx].entries[repl_idx];
  sets[set_idx].entries[repl_idx] = Entry<User_Key_Type, User_Data_Type>{TRUE, key, data, 0};
  update_repl_states(sets[set_idx], repl_idx);

  // the evicted entry is an eviciton victim if and only if it is valid
  return evicted_entry;
}

template <typename User_Key_Type, typename User_Data_Type>
Entry<User_Key_Type, User_Data_Type> Cpp_Cache<User_Key_Type, User_Data_Type>::invalidate(User_Key_Type key) {
  uns set_idx = set_idx_hash(key);
  Entry<User_Key_Type, User_Data_Type> invalidated_entry{};
  for (uns i = 0; i < assoc; i++) {
    Entry<User_Key_Type, User_Data_Type>& entry = sets[set_idx].entries[i];
    if (entry.valid && entry.key == key) {  // hit
      invalidated_entry = entry;
      entry.valid = FALSE;
      break;
    }
  }
  return invalidated_entry;
}
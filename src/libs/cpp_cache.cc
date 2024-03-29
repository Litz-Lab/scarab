// CPP implementation of a cache.
// Built-in support for multi-line units, e.g. PWs that take multiple lines.

#include "libs/cpp_cache.h"
#include <cstdlib>
#include <iostream>
#include <list>
#include <string>
#include <unordered_map>
#include <vector>
#include "debug/debug_macros.h"
#include "debug/debug_print.h"

extern "C" {
#include "globals/assert.h"
#include "globals/utils.h"
#include "memory/memory.param.h"
}

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_CPP_CACHE, ##args)

class Entry {
 public:
  Addr    addr;
  Addr    tag;
  int     num_lines;
  bool    priority;
  void*   data;
};

// List with custom capacity logging, enabling multiline Entries.
class Set {
 public:
  std::list<Entry> lst       = {};
  size_t           num_lines = 0;
};

class Cpp_Cache {
 private:
  std::vector<Set> sets;
  Repl_Policy      repl_policy;
  Counter          capacity_lines;
  uns              assoc;
  uns              line_bytes;
  bool             tag_incl_offset;
  int              data_size;

  uns offset_bits;
  uns offset_mask;
  uns set_mask;
  uns tag_mask;
  uns line_addr_mask;

 public:
  Cpp_Cache(uns nl, uns asc, uns lb, Repl_Policy rp, bool tio, int ds) {
    capacity_lines  = nl;
    assoc           = asc;
    line_bytes      = lb;
    repl_policy     = rp;
    tag_incl_offset = tio;
    data_size       = ds;

    uns num_sets = capacity_lines / assoc;
    sets         = std::vector<Set>(num_sets);
    offset_bits  = LOG2(line_bytes);
    offset_mask  = N_BIT_MASK(offset_bits);
    set_mask     = N_BIT_MASK(LOG2(num_sets)) << offset_bits;
    // If tag_incl_offset, we want to use non-LSB bits for set although cache is
    // byte-addressable
    tag_mask       = tag_incl_offset ? ~set_mask : ~(set_mask & offset_mask);
    line_addr_mask = tag_incl_offset ? N_BIT_MASK_64 : ~offset_mask;
  }

  uns   index(Addr addr, Addr* tag, Addr* line_addr);
  void* access(Addr addr, bool update_repl, bool upgrade_priority);
  void* insert(Addr addr, int lines_used, bool priority, Addr* evicted_entries);
  Addr  evict(Set& set);
  void  insert_entry_at_pos(Set& set, Entry entry,
                            std::list<Entry>::iterator pos);
};

std::unordered_map<std::string, Cpp_Cache> cache_map{};

// utility function for debugging
std::ostream& operator<<(std::ostream& os, const std::list<Entry>& list) {
  for(auto const& entry : list) {
    os << entry.tag << ", num_lines=" << entry.num_lines << "; ";
  }
  os << std::endl;
  return os;
}


static inline Cpp_Cache* get_cache(char* name) {
  std::string key = name;
  return &cache_map.at(key);
}

// returns the set index of the addr within this cache.
uns cpp_cache_index(char* name, Addr addr, Addr* tag, Addr* line_addr) {
  return get_cache(name)->index(addr, tag, line_addr);
}

void cpp_cache_create(char* name, uns num_lines, uns assoc, uns line_bytes,
                      Repl_Policy repl_policy, Flag tag_incl_offset,
                      int data_size) {
  Cpp_Cache   cache(num_lines, assoc, line_bytes, repl_policy,
                    tag_incl_offset == TRUE, data_size);
  std::string key = name;
  cache_map.insert({key, cache});
}

// Insert into cache, returning a void* for the callee to fill with data.
// evicted_entries must have enough allocated space to capture num_addr up to the cache assoc.
void* cpp_cache_insert(char* name, Addr addr, int lines_used, Flag priority, Addr* evicted_entries) {
  return get_cache(name)->insert(addr, lines_used, priority == TRUE, evicted_entries); //notify which lines evicted? Could be multiple! require an array to be passed in, size of assoc.
}

// Access cache. Upgrade priority if upgrade_priority and update_repl.
void* cpp_cache_access(char* name, Addr addr, Flag update_repl, Flag upgrade_priority) {
  return get_cache(name)->access(addr, update_repl == TRUE, upgrade_priority == TRUE);;
}

/////////// Cache Member Methods ////////////

uns Cpp_Cache::index(Addr addr, Addr* tag, Addr* line_addr) {
  // may need to return tag or line_addr as well.
  *tag       = addr & tag_mask;
  *line_addr = addr & line_addr_mask;
  return (addr & set_mask) >> offset_bits;
}

// access: Looks up the cache based on addr. Returns pointer to line data if
// found.
void* Cpp_Cache::access(Addr addr, bool update_repl, bool upgrade_priority) {
  Addr  tag;
  Addr  line_addr;
  void* data    = NULL;
  uns   set_idx = index(addr, &tag, &line_addr);
  for(std::list<Entry>::iterator entry_it = sets.at(set_idx).lst.begin();
      entry_it != sets.at(set_idx).lst.end(); ++entry_it) {
    Entry entry = *entry_it;
    if(entry.tag == tag) {  // hit
      data = entry.data;
      if(update_repl) {
        if (upgrade_priority) {
          entry.priority = true;
        }
        // Move to LRU position: remove entry from list and reinsert at Head
        sets.at(set_idx).lst.erase(entry_it);
        insert_entry_at_pos(sets.at(set_idx), entry,
                            sets.at(set_idx).lst.begin());
      }
      break;
    }
  }
  return data;
}

// Returns evicted line addr
Addr Cpp_Cache::evict(Set& set) {
  Addr evicted_line;
  ASSERT(0, !set.lst.empty());
  auto it_evicted_entry = --set.lst.end();  // last element
  set.num_lines -= it_evicted_entry->num_lines;
  evicted_line = it_evicted_entry->addr;
  free(it_evicted_entry->data);
  set.lst.erase(it_evicted_entry);
  return evicted_line;
}

// there are potentially Different policies for insert and access.
// A different insert policy would choose a different insert pos
// OR for insert, first insert at end, then update_repl for it.
void Cpp_Cache::insert_entry_at_pos(Set& set, Entry entry_to_insert,
                                    std::list<Entry>::iterator pos) {
  switch(repl_policy) {
    case REPL_TRUE_LRU:
      set.lst.insert(pos, entry_to_insert);
      break;
    case REPL_STICKY_PRIORITY_LINES: {
      if(rand() % 100 > PRIORITY_LINE_STICKINESS_PERCENT) {  // if not sticky
        set.lst.insert(pos, entry_to_insert);
        break;
      }
      // Find insert point.
      std::list<Entry>::iterator insert_pos = pos;
      while(insert_pos != set.lst.end() && !entry_to_insert.priority && insert_pos->priority) {  //what if all are prio?  *************
        ++insert_pos;
        // now insert_pos is at the element right after the insert position
      }
      set.lst.insert(insert_pos, entry_to_insert);
      for(auto cur_entry_it = insert_pos; cur_entry_it != set.lst.end();
          ++cur_entry_it) {
        auto prev_entry_it = std::prev(cur_entry_it);
        if(cur_entry_it->priority && !prev_entry_it->priority) {
          // move cur_entry left one element in the list
          set.lst.splice(prev_entry_it, set.lst, cur_entry_it);
        }
      }
    } break;
    default:
      ASSERT(0, FALSE);  // unsupported
  }
}

void* Cpp_Cache::insert(Addr addr, int lines_used, bool priority, Addr* evicted_entries) {
  Addr  tag;
  Addr  line_addr;
  void* data    = NULL;
  uns   set_idx = index(addr, &tag, &line_addr);
  // First check if line exists
  for(Entry entry : sets.at(set_idx).lst) {
    if(entry.tag == tag) {  // hit, do nothing
      return NULL;
    }
  }
  data = calloc(1, data_size);
  Entry new_entry{addr, tag, lines_used, priority, data};
  // Evict any entries required
  uns num_entries_evicted = 0;
  while(assoc < sets.at(set_idx).num_lines + new_entry.num_lines) {
    evicted_entries[num_entries_evicted++] = evict(sets.at(set_idx));
  }
  ASSERT(0, num_entries_evicted <= assoc);
  insert_entry_at_pos(sets.at(set_idx), new_entry,
                      sets.at(set_idx).lst.begin());
  sets.at(set_idx).num_lines += lines_used;
  ASSERT(0, sets.at(set_idx).num_lines <= assoc);
  return data;
}

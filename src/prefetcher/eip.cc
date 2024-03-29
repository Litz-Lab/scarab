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
 * File         : eip.cc
 * Date         : 7/3/2023
 * Description  : Entangling Instruction Prefetcher - Based on EIP ( ISCA'21 )
 * Cite: Alberto Ros and Alexandra Jimborean, "A Cost-Effective Entangling Prefetcher
 * for Instructions"
 ***************************************************************************************/

#include "debug/debug_macros.h"
#include "debug/debug_print.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"

#include "globals/assert.h"
#include "globals/utils.h"

#include "libs/hash_lib.h"
#include "libs/list_lib.h"
#include "prefetcher/eip.h"
#include "prefetcher/fdip_new.h"
#include "prefetcher/pref_common.h"
#include "statistics.h"

extern "C" {
#include "op.h"
#include "prefetcher/pref.param.h"
#include "memory/memory.param.h"
#include "debug/debug.param.h"
#include "memory/memory.h"
#include "general.param.h"
}

#include <iostream>
#include <vector>

using std::cout;
using std::endl;
using std::hex;
using std::dec;

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_EIP, ##args)

extern int per_cyc_ipref;
extern const int MAX_FTQ_ENTRY_CYC;

// To access cpu in my functions
uint32_t eip_proc_id;
uint32_t L1I_RQ_SIZE = 0;
uint32_t L1I_TIMING_MSHR_SIZE = 0;
uint32_t L1I_SET = 0;
uint32_t L1I_WAY = 0;

uint64_t l1i_last_basic_block;
uint32_t l1i_consecutive_count;
uint32_t l1i_basic_block_merge_diff;

bool debug = 0;

#define L1I_HIST_TABLE_ENTRIES 16

// LINE AND MERGE BASIC BLOCK SIZE

#define L1I_MERGE_BBSIZE_BITS 6
#define L1I_MERGE_BBSIZE_MAX_VALUE ((1 << L1I_MERGE_BBSIZE_BITS) - 1)

// TIME AND OVERFLOWS

#define L1I_TIME_DIFF_BITS 20
#define L1I_TIME_DIFF_OVERFLOW ((uint64_t)1 << L1I_TIME_DIFF_BITS)
#define L1I_TIME_DIFF_MASK (L1I_TIME_DIFF_OVERFLOW - 1)

#define L1I_TIME_BITS 12
#define L1I_TIME_OVERFLOW ((uint64_t)1 << L1I_TIME_BITS)
#define L1I_TIME_MASK (L1I_TIME_OVERFLOW - 1)

uint64_t l1i_get_latency(uint64_t cycle, uint64_t cycle_prev) {
  uint64_t cycle_masked = cycle & L1I_TIME_MASK;
  uint64_t cycle_prev_masked = cycle_prev & L1I_TIME_MASK;
  if (cycle_prev_masked > cycle_masked) {
    return (cycle_masked + L1I_TIME_OVERFLOW) - cycle_prev_masked;
  }
  return cycle_masked - cycle_prev_masked;
}

// ENTANGLED COMPRESSION FORMAT

#define L1I_ENTANGLED_MAX_FORMATS 7

// STATS
#define L1I_STATS_TABLE_INDEX_BITS 16
#define L1I_STATS_TABLE_ENTRIES (1 << L1I_STATS_TABLE_INDEX_BITS)
#define L1I_STATS_TABLE_MASK (L1I_STATS_TABLE_ENTRIES - 1)

typedef struct __l1i_stats_entry {
  uint64_t accesses;
  uint64_t misses;
  uint64_t hits;
  uint64_t late;
  uint64_t wrong; // early
} l1i_stats_entry;

//l1i_stats_entry l1i_stats_table[NUM_CPUS][L1I_STATS_TABLE_ENTRIES];
std::vector<l1i_stats_entry *> l1i_stats_table;
uint64_t l1i_stats_discarded_prefetches;
uint64_t l1i_stats_evict_entangled_j_table;
uint64_t l1i_stats_evict_entangled_k_table;
uint64_t l1i_stats_max_bb_size;
uint64_t l1i_stats_formats[L1I_ENTANGLED_MAX_FORMATS];
uint64_t l1i_stats_hist_lookups[L1I_HIST_TABLE_ENTRIES+2];
uint64_t l1i_stats_basic_blocks[L1I_MERGE_BBSIZE_MAX_VALUE+1];
uint64_t l1i_stats_entangled[L1I_ENTANGLED_MAX_FORMATS+1];
uint64_t l1i_stats_basic_blocks_ent[L1I_MERGE_BBSIZE_MAX_VALUE+1];

void l1i_init_stats_table() {
  for (int i = 0; i < L1I_STATS_TABLE_ENTRIES; i++) {
    l1i_stats_table[eip_proc_id][i].accesses = 0;
    l1i_stats_table[eip_proc_id][i].misses = 0;
    l1i_stats_table[eip_proc_id][i].hits = 0;
    l1i_stats_table[eip_proc_id][i].late = 0;
    l1i_stats_table[eip_proc_id][i].wrong = 0;
  }
  l1i_stats_discarded_prefetches = 0;
  l1i_stats_evict_entangled_j_table = 0;
  l1i_stats_evict_entangled_k_table = 0;
  l1i_stats_max_bb_size = 0;
  for (int i = 0; i < L1I_ENTANGLED_MAX_FORMATS; i++) {
    l1i_stats_formats[i] = 0;
  }
  for (int i = 0; i <= L1I_HIST_TABLE_ENTRIES; i++) {
    l1i_stats_hist_lookups[i] = 0;
  }
  for (int i = 0; i <= L1I_MERGE_BBSIZE_MAX_VALUE; i++) {
    l1i_stats_basic_blocks[i] = 0;
  }
  for (int i = 0; i <= L1I_ENTANGLED_MAX_FORMATS; i++) {
    l1i_stats_entangled[i] = 0;
  }
  for (int i = 0; i <= L1I_MERGE_BBSIZE_MAX_VALUE; i++) {
    l1i_stats_basic_blocks_ent[i] = 0;
  }
}

void l1i_print_stats_table() {
  cout << "IP accesses: ";
  uint64_t max = 0;
  uint64_t max_addr = 0;
  uint64_t total_accesses = 0;
  for (uint32_t i = 0; i < L1I_STATS_TABLE_ENTRIES; i++) {
    if (l1i_stats_table[eip_proc_id][i].accesses > max) {
      max = l1i_stats_table[eip_proc_id][i].accesses;
      max_addr = i;
    }
    total_accesses += l1i_stats_table[eip_proc_id][i].accesses;
  }
  cout << hex << max_addr << " " << (max_addr << LOG2(ICACHE_LINE_SIZE)) << dec << " " << max << " / " << total_accesses << endl;
  cout << "IP misses: ";
  max = 0;
  max_addr = 0;
  uint64_t total_misses = 0;
  for (uint32_t i = 0; i < L1I_STATS_TABLE_ENTRIES; i++) {
    if (l1i_stats_table[eip_proc_id][i].misses > max) {
      max = l1i_stats_table[eip_proc_id][i].misses;
      max_addr = i;
    }
    total_misses += l1i_stats_table[eip_proc_id][i].misses;
  }
  cout << hex << max_addr << " " << (max_addr << LOG2(ICACHE_LINE_SIZE)) << dec << " " << max << " / " << total_misses << endl;
  cout << "IP hits: ";
  max = 0;
  max_addr = 0;
  uint64_t total_hits = 0;
  for (uint32_t i = 0; i < L1I_STATS_TABLE_ENTRIES; i++) {
    if (l1i_stats_table[eip_proc_id][i].hits > max) {
      max = l1i_stats_table[eip_proc_id][i].hits;
      max_addr = i;
    }
    total_hits += l1i_stats_table[eip_proc_id][i].hits;
  }
  cout << hex << max_addr << " " << (max_addr << LOG2(ICACHE_LINE_SIZE)) << dec << " " << max << " / " << total_hits << endl;
  cout << "IP late: ";
  max = 0;
  max_addr = 0;
  uint64_t total_late = 0;
  for (uint32_t i = 0; i < L1I_STATS_TABLE_ENTRIES; i++) {
    if (l1i_stats_table[eip_proc_id][i].late > max) {
      max = l1i_stats_table[eip_proc_id][i].late;
      max_addr = i;
    }
    total_late += l1i_stats_table[eip_proc_id][i].late;
  }
  cout << hex << max_addr << " " << (max_addr << LOG2(ICACHE_LINE_SIZE)) << dec << " " << max << " / " << total_late << endl;
  cout << "IP wrong: ";
  max = 0;
  max_addr = 0;
  uint64_t total_wrong = 0;
  for (uint32_t i = 0; i < L1I_STATS_TABLE_ENTRIES; i++) {
    if (l1i_stats_table[eip_proc_id][i].wrong > max) {
      max = l1i_stats_table[eip_proc_id][i].wrong;
      max_addr = i;
    }
    total_wrong += l1i_stats_table[eip_proc_id][i].wrong;
  }
  cout << hex << max_addr << " " << (max_addr << LOG2(ICACHE_LINE_SIZE)) << dec << " " << max << " / " << total_wrong << endl;

  cout << "miss rate: " << ((double)total_misses / (double)total_accesses) << endl;
  cout << "coverage: " << ((double)total_hits / (double)(total_hits + total_misses)) << endl;
  cout << "coverage_late: " << ((double)(total_hits + total_late) / (double)(total_hits + total_misses)) << endl;
  cout << "accuracy: " << ((double)total_hits / (double)(total_hits + total_late + total_wrong)) << endl;
  cout << "accuracy_late: " << ((double)(total_hits + total_late) / (double)(total_hits + total_late + total_wrong)) << endl;
  cout << "discarded: " << l1i_stats_discarded_prefetches << endl;
  cout << "evicts entangled j table: " << l1i_stats_evict_entangled_j_table << endl;
  cout << "evicts entangled k table: " << l1i_stats_evict_entangled_k_table << endl;
  cout << "max bb size: " << l1i_stats_max_bb_size << endl;
  cout << "formats: ";
  for (uint32_t i = 0; i < L1I_ENTANGLED_MAX_FORMATS; i++) {
    cout << l1i_stats_formats[i] << " ";
  }
  cout << endl;
  cout << "hist_lookups: ";
  uint64_t total_hist_lookups = 0;
  for (uint32_t i = 0; i <= L1I_HIST_TABLE_ENTRIES+1; i++) {
    cout << l1i_stats_hist_lookups[i] << " ";
    total_hist_lookups += l1i_stats_hist_lookups[i];
  }
  cout << endl;
  cout << "hist_lookups_evict: " << (double)l1i_stats_hist_lookups[L1I_HIST_TABLE_ENTRIES] * 100 / (double)(total_hist_lookups) << " %" << endl;
  cout << "hist_lookups_shortlat: " << (double)l1i_stats_hist_lookups[L1I_HIST_TABLE_ENTRIES+1] * 100 / (double)(total_hist_lookups) << " %" << endl;

  cout << "bb_found_hist: ";
  uint64_t total_bb_found = 0;
  uint64_t total_bb_prefetches = 0;
  for (uint32_t i = 0; i <= L1I_MERGE_BBSIZE_MAX_VALUE; i++) {
    cout << l1i_stats_basic_blocks[i] << " ";
    total_bb_found += i * l1i_stats_basic_blocks[i];
    total_bb_prefetches += l1i_stats_basic_blocks[i];
  }
  cout << endl;
  cout << "bb_found_summary: " << total_bb_found << " " << total_bb_prefetches << " " << (double)total_bb_found / (double)total_bb_prefetches << endl;

  cout << "entangled_found_hist: ";
  uint64_t total_entangled_found = 0;
  uint64_t total_ent_prefetches = 0;
  for (uint32_t i = 0; i <= L1I_ENTANGLED_MAX_FORMATS; i++) {
    cout << l1i_stats_entangled[i] << " ";
    total_entangled_found += i * l1i_stats_entangled[i];
    total_ent_prefetches += l1i_stats_entangled[i];
  }
  cout << endl;
  cout << "entangled_found_summary: " << total_entangled_found << " " << total_ent_prefetches << " " << (double)total_entangled_found / (double)total_ent_prefetches << endl;

  cout << "bb_ent_found_hist: ";
  uint64_t total_bb_ent_found = 0;
  uint64_t total_bb_ent_prefetches = 0;
  for (uint32_t i = 0; i <= L1I_MERGE_BBSIZE_MAX_VALUE; i++) {
    cout << l1i_stats_basic_blocks_ent[i] << " ";
    total_bb_ent_found += i * l1i_stats_basic_blocks_ent[i];
    total_bb_ent_prefetches += l1i_stats_basic_blocks_ent[i];
  }
  cout << endl;
  cout << "bb_ent_found_summary: " << total_bb_ent_found << " " << total_bb_ent_prefetches << " " << (double)total_bb_ent_found / (double)total_bb_ent_prefetches << endl;
}

// HISTORY TABLE (BUFFER)

#define L1I_HIST_TABLE_MASK (L1I_HIST_TABLE_ENTRIES - 1)
#define L1I_BB_MERGE_ENTRIES 5
#define L1I_HIST_TAG_BITS 58
#define L1I_HIST_TAG_MASK (((uint64_t)1 << L1I_HIST_TAG_BITS) - 1)

typedef struct __l1i_hist_entry {
  uint64_t tag; // L1I_HIST_TAG_BITS bits
  uint64_t time_diff; // L1I_TIME_DIFF_BITS bits
  uint32_t bb_size; // L1I_MERGE_BBSIZE_BITS bits
} l1i_hist_entry;

//l1i_hist_entry l1i_hist_table[NUM_CPUS][L1I_HIST_TABLE_ENTRIES];
std::vector<l1i_hist_entry *> l1i_hist_table;
//uint64_t l1i_hist_table_head[NUM_CPUS]; // log_2 (L1I_HIST_TABLE_ENTRIES)
std::vector<uint64_t> l1i_hist_table_head; // log_2 (L1I_HIST_TABLE_ENTRIES)
//uint64_t l1i_hist_table_head_time[NUM_CPUS]; // 64 bits
std::vector<uint64_t> l1i_hist_table_head_time; // 64 bits

void l1i_init_hist_table() {
  l1i_hist_table_head[eip_proc_id] = 0;
  l1i_hist_table_head_time[eip_proc_id] = cycle_count;
  for (uint32_t i = 0; i < L1I_HIST_TABLE_ENTRIES; i++) {
    l1i_hist_table[eip_proc_id][i].tag = 0;
    l1i_hist_table[eip_proc_id][i].time_diff = 0;
    l1i_hist_table[eip_proc_id][i].bb_size = 0;
  }
}

uint64_t l1i_find_hist_entry(uint64_t line_addr) {
  uint64_t tag = line_addr & L1I_HIST_TAG_MASK; 
  for (uint32_t count = 0, i = (l1i_hist_table_head[eip_proc_id] + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES; count < L1I_HIST_TABLE_ENTRIES; count++, i = (i + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES) {
    if (l1i_hist_table[eip_proc_id][i].tag == tag) return i;
  }
  return L1I_HIST_TABLE_ENTRIES;
}

// It can have duplicated entries if the line was evicted in between
uint32_t l1i_add_hist_table(uint64_t line_addr) {
  // Insert empty addresses in hist not to have timediff overflows
  while(cycle_count - l1i_hist_table_head_time[eip_proc_id] >= L1I_TIME_DIFF_OVERFLOW) {
    l1i_hist_table[eip_proc_id][l1i_hist_table_head[eip_proc_id]].tag = 0;
    l1i_hist_table[eip_proc_id][l1i_hist_table_head[eip_proc_id]].time_diff = L1I_TIME_DIFF_MASK;
    l1i_hist_table[eip_proc_id][l1i_hist_table_head[eip_proc_id]].bb_size = 0;
    l1i_hist_table_head[eip_proc_id] = (l1i_hist_table_head[eip_proc_id] + 1) % L1I_HIST_TABLE_ENTRIES;
    l1i_hist_table_head_time[eip_proc_id] += L1I_TIME_DIFF_MASK;
  }

  // Allocate a new entry (evict old one if necessary)
  l1i_hist_table[eip_proc_id][l1i_hist_table_head[eip_proc_id]].tag = line_addr & L1I_HIST_TAG_MASK;
  l1i_hist_table[eip_proc_id][l1i_hist_table_head[eip_proc_id]].time_diff = (cycle_count - l1i_hist_table_head_time[eip_proc_id]) & L1I_TIME_DIFF_MASK;
  l1i_hist_table[eip_proc_id][l1i_hist_table_head[eip_proc_id]].bb_size = 0;
  uint32_t pos = l1i_hist_table_head[eip_proc_id];
  l1i_hist_table_head[eip_proc_id] = (l1i_hist_table_head[eip_proc_id] + 1) % L1I_HIST_TABLE_ENTRIES;
  l1i_hist_table_head_time[eip_proc_id] = cycle_count;
  return pos;
}

void l1i_add_bb_size_hist_table(uint64_t line_addr, uint32_t bb_size) {
  uint64_t index = l1i_find_hist_entry(line_addr);
  l1i_hist_table[eip_proc_id][index].bb_size = bb_size & L1I_MERGE_BBSIZE_MAX_VALUE;
}

uint32_t l1i_find_bb_merge_hist_table(uint64_t line_addr) {
  uint64_t tag = line_addr & L1I_HIST_TAG_MASK; 
  for (uint32_t count = 0, i = (l1i_hist_table_head[eip_proc_id] + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES; count < L1I_HIST_TABLE_ENTRIES; count++, i = (i + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES) {
    if (count >= L1I_BB_MERGE_ENTRIES) {
      return 0;
    }
    if (tag > l1i_hist_table[eip_proc_id][i].tag
        && (tag - l1i_hist_table[eip_proc_id][i].tag) <= l1i_hist_table[eip_proc_id][i].bb_size) {
      //&& (tag - l1i_hist_table[eip_proc_id][i].tag) == l1i_hist_table[eip_proc_id][i].bb_size) {
      return tag - l1i_hist_table[eip_proc_id][i].tag;
    }
    }
    ASSERT(eip_proc_id, false);
  }

  // return src-entangled pair
  uint64_t l1i_get_src_entangled_hist_table(uint64_t line_addr, uint32_t pos_hist, uint64_t latency, uint32_t skip = 0) {
    ASSERT(eip_proc_id, pos_hist < L1I_HIST_TABLE_ENTRIES);
    uint64_t tag = line_addr & L1I_HIST_TAG_MASK;
    ASSERT(eip_proc_id, tag);
    if (l1i_hist_table[eip_proc_id][pos_hist].tag != tag) {
      l1i_stats_hist_lookups[L1I_HIST_TABLE_ENTRIES]++;
      return 0; // removed
    }
    uint32_t next_pos = (pos_hist + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES;
    uint32_t first = (l1i_hist_table_head[eip_proc_id] + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES;
    uint64_t time_i = l1i_hist_table[eip_proc_id][pos_hist].time_diff;
    uint32_t num_skipped = 0;
    for (uint32_t count = 0, i = next_pos; i != first; count++, i = (i + L1I_HIST_TABLE_MASK) % L1I_HIST_TABLE_ENTRIES) {
      // Against the time overflow
      if (l1i_hist_table[eip_proc_id][i].tag == tag) {
        return 0; // Second time it appeared (it was evicted in between) or many for the same set. No entangle
      }
      if (l1i_hist_table[eip_proc_id][i].tag && time_i >= latency) {
        if (skip == num_skipped) {
          l1i_stats_hist_lookups[count]++;
          return l1i_hist_table[eip_proc_id][i].tag;
        } else {
          num_skipped++;
        }
      }
      time_i += l1i_hist_table[eip_proc_id][i].time_diff;
    }
    l1i_stats_hist_lookups[L1I_HIST_TABLE_ENTRIES+1]++;
    return 0;
  }

  // TIMING TABLES

#define L1I_SET_BITS 6
//#define L1I_TIMING_MSHR_SIZE (L1I_PQ_SIZE+L1I_MSHR_SIZE+64) // Not necessary +64 (perhaps +L1I_RQ_SIZE), just to track enough in-fligh requests 
#define L1I_TIMING_MSHR_TAG_BITS 42
#define L1I_TIMING_MSHR_TAG_MASK (((uint64_t)1 << L1I_HIST_TAG_BITS) - 1)
#define L1I_TIMING_CACHE_TAG_BITS (L1I_TIMING_MSHR_TAG_BITS - L1I_SET_BITS)
#define L1I_TIMING_CACHE_TAG_MASK (((uint64_t)1 << L1I_HIST_TAG_BITS) - 1)

  // We do not have access to the MSHR, so we aproximate it using this structure
  typedef struct __l1i_timing_mshr_entry {
    bool valid; // 1 bit
    uint64_t tag; // L1I_TIMING_MSHR_TAG_BITS bits
    uint32_t source_set; // 8 bits
    uint32_t source_way; // 6 bits
    uint64_t timestamp; // L1I_TIME_BITS bits // time when issued
    bool accessed; // 1 bit
    uint32_t pos_hist; // 1 bit
  } l1i_timing_mshr_entry;

  // We do not have access to the cache, so we aproximate it using this structure
  typedef struct __l1i_timing_cache_entry {
    bool valid; // 1 bit
    uint64_t tag; // L1I_TIMING_CACHE_TAG_BITS bits
    uint32_t source_set; // 8 bits
    uint32_t source_way; // 6 bits
    bool accessed; // 1 bit
  } l1i_timing_cache_entry;

  //l1i_timing_mshr_entry l1i_timing_mshr_table[NUM_CPUS][L1I_TIMING_MSHR_SIZE];
  std::vector<l1i_timing_mshr_entry*> l1i_timing_mshr_table;
  //l1i_timing_cache_entry l1i_timing_cache_table[NUM_CPUS][L1I_SET][L1I_WAY];
  std::vector<l1i_timing_cache_entry**> l1i_timing_cache_table;

  void l1i_init_timing_tables() {
    for (uint32_t i = 0; i < L1I_TIMING_MSHR_SIZE; i++) {
      l1i_timing_mshr_table[eip_proc_id][i].valid = 0;
    }
    for (uint32_t i = 0; i < L1I_SET; i++) {
      for (uint32_t j = 0; j < L1I_WAY; j++) {
        l1i_timing_cache_table[eip_proc_id][i][j].valid = 0;
      }
    }
  }

  uint64_t l1i_find_timing_mshr_entry(uint64_t line_addr) {
    for (uint32_t i = 0; i < L1I_TIMING_MSHR_SIZE; i++) {
      if (l1i_timing_mshr_table[eip_proc_id][i].tag == (line_addr & L1I_TIMING_MSHR_TAG_MASK)
          && l1i_timing_mshr_table[eip_proc_id][i].valid) return i;
    }
    return L1I_TIMING_MSHR_SIZE;
  }

  uint64_t l1i_find_timing_cache_entry(uint64_t line_addr) {
    uint64_t i = line_addr % L1I_SET;
    for (uint32_t j = 0; j < L1I_WAY; j++) {
      if (l1i_timing_cache_table[eip_proc_id][i][j].tag == ((line_addr >> L1I_SET_BITS) & L1I_TIMING_CACHE_TAG_MASK)
          && l1i_timing_cache_table[eip_proc_id][i][j].valid) return j;
    }
    return L1I_WAY;
  }

  uint32_t l1i_get_invalid_timing_mshr_entry() {
    for (uint32_t i = 0; i < L1I_TIMING_MSHR_SIZE; i++) {
      if (!l1i_timing_mshr_table[eip_proc_id][i].valid) return i;
    }
    ASSERT(eip_proc_id, false); // It must return a free entry
    return L1I_TIMING_MSHR_SIZE;  
  }

  uint32_t l1i_get_invalid_timing_cache_entry(uint64_t line_addr) {
    uint32_t i = line_addr % L1I_SET;
    DEBUG(eip_proc_id, "find a timing cache entry to invalidate for set %u\n", i);
    for (uint32_t j = 0; j < L1I_WAY; j++) {
      if (l1i_timing_cache_table[eip_proc_id][i][j].tag == ((line_addr >> L1I_SET_BITS) & L1I_TIMING_CACHE_TAG_MASK)) return j;
      if (!l1i_timing_cache_table[eip_proc_id][i][j].valid) return j;
    }
    ASSERT(eip_proc_id, false); // It must return a free entry
    return L1I_WAY;  
  }

  void l1i_add_timing_entry(uint64_t line_addr, uint32_t source_set, uint32_t source_way) {
    // First find for coalescing
    if (l1i_find_timing_mshr_entry(line_addr) < L1I_TIMING_MSHR_SIZE) return;
    if (l1i_find_timing_cache_entry(line_addr) < L1I_WAY) return;

    uint32_t i = l1i_get_invalid_timing_mshr_entry();
    l1i_timing_mshr_table[eip_proc_id][i].valid = true;
    l1i_timing_mshr_table[eip_proc_id][i].tag = line_addr & L1I_TIMING_MSHR_TAG_MASK;
    l1i_timing_mshr_table[eip_proc_id][i].source_set = source_set;
    l1i_timing_mshr_table[eip_proc_id][i].source_way = source_way;
    l1i_timing_mshr_table[eip_proc_id][i].timestamp = cycle_count & L1I_TIME_MASK;
    l1i_timing_mshr_table[eip_proc_id][i].accessed = false;
  }

  void l1i_invalid_timing_mshr_entry(uint64_t line_addr) {
    uint32_t index = l1i_find_timing_mshr_entry(line_addr);
    ASSERT(eip_proc_id, index < L1I_TIMING_MSHR_SIZE);
    l1i_timing_mshr_table[eip_proc_id][index].valid = false;
  }

  void l1i_move_timing_entry(uint64_t line_addr) {
    uint32_t index_mshr = l1i_find_timing_mshr_entry(line_addr); 
    if (index_mshr == L1I_TIMING_MSHR_SIZE) {
      uint32_t set = line_addr % L1I_SET;
      uint32_t index_cache = l1i_get_invalid_timing_cache_entry(line_addr);
      l1i_timing_cache_table[eip_proc_id][set][index_cache].valid = true;
      l1i_timing_cache_table[eip_proc_id][set][index_cache].tag = (line_addr >> L1I_SET_BITS) & L1I_TIMING_CACHE_TAG_MASK;
      l1i_timing_cache_table[eip_proc_id][set][index_cache].source_way = L1I_ENTANGLED_TABLE_WAYS;
      l1i_timing_cache_table[eip_proc_id][set][index_cache].accessed = true;
      DEBUG(eip_proc_id, "fill timing_cache_entry at set %u index %u for 0x%lx icache_line_addr 0x%lx\n", set, index_cache, line_addr, line_addr << LOG2(ICACHE_LINE_SIZE));
      return;
    }
    uint64_t set = line_addr % L1I_SET;
    uint64_t index_cache = l1i_get_invalid_timing_cache_entry(line_addr);
    l1i_timing_cache_table[eip_proc_id][set][index_cache].valid = true;
    l1i_timing_cache_table[eip_proc_id][set][index_cache].tag = (line_addr >> L1I_SET_BITS) & L1I_TIMING_CACHE_TAG_MASK;
    l1i_timing_cache_table[eip_proc_id][set][index_cache].source_set = l1i_timing_mshr_table[eip_proc_id][index_mshr].source_set;
    l1i_timing_cache_table[eip_proc_id][set][index_cache].source_way = l1i_timing_mshr_table[eip_proc_id][index_mshr].source_way;
    l1i_timing_cache_table[eip_proc_id][set][index_cache].accessed = l1i_timing_mshr_table[eip_proc_id][index_mshr].accessed;
    l1i_invalid_timing_mshr_entry(line_addr);
    DEBUG(eip_proc_id, "fill timing_cache_entry at set %lu index %lu for 0x%lx icache_line_addr 0x%lx\n", set, index_cache, line_addr, line_addr << LOG2(ICACHE_LINE_SIZE));
  }

  // returns if accessed
  bool l1i_invalid_timing_cache_entry(uint64_t line_addr, uint32_t &source_set, uint32_t &source_way) {
    uint32_t set = line_addr % L1I_SET;
    uint32_t way = l1i_find_timing_cache_entry(line_addr);
    ASSERT(eip_proc_id, way < L1I_WAY);
    l1i_timing_cache_table[eip_proc_id][set][way].valid = false;
    DEBUG(eip_proc_id, "invalidate timing_cache_entry at set %u index %u for 0x%lx icache_line_addr 0x%lx\n", set, way, line_addr, line_addr << LOG2(ICACHE_LINE_SIZE));
    source_set = l1i_timing_cache_table[eip_proc_id][set][way].source_set;
    source_way = l1i_timing_cache_table[eip_proc_id][set][way].source_way;
    return l1i_timing_cache_table[eip_proc_id][set][way].accessed;
  }

  void l1i_access_timing_entry(uint64_t line_addr, uint32_t pos_hist, uint32_t &source_set, uint32_t &source_way) {
    uint32_t index = l1i_find_timing_mshr_entry(line_addr);
    if (index < L1I_TIMING_MSHR_SIZE) {
      if (!l1i_timing_mshr_table[eip_proc_id][index].accessed) { // Prefetch accessed while in MSHR: late
        l1i_timing_mshr_table[eip_proc_id][index].accessed = true;
        l1i_timing_mshr_table[eip_proc_id][index].pos_hist = pos_hist;
        if (l1i_timing_mshr_table[eip_proc_id][index].source_way < L1I_ENTANGLED_TABLE_WAYS) {
          source_set = l1i_timing_mshr_table[eip_proc_id][index].source_set;
          source_way = l1i_timing_mshr_table[eip_proc_id][index].source_way;
          l1i_timing_mshr_table[eip_proc_id][index].source_set = 0;
          l1i_timing_mshr_table[eip_proc_id][index].source_way = L1I_ENTANGLED_TABLE_WAYS;
        }
      }
      return;
    }
    uint32_t set = line_addr % L1I_SET;
    uint32_t way = l1i_find_timing_cache_entry(line_addr);
    if (way < L1I_WAY) {
      l1i_timing_cache_table[eip_proc_id][set][way].accessed = true;
    }
  }

  bool l1i_is_accessed_timing_entry(uint64_t line_addr) {
    uint32_t index = l1i_find_timing_mshr_entry(line_addr);
    if (index < L1I_TIMING_MSHR_SIZE) {
      return l1i_timing_mshr_table[eip_proc_id][index].accessed;
    }
    uint32_t set = line_addr % L1I_SET;
    uint32_t way = l1i_find_timing_cache_entry(line_addr);
    if (way < L1I_WAY) {
      return l1i_timing_cache_table[eip_proc_id][set][way].accessed;
    }
    return false;
  }

  bool l1i_completed_request(uint64_t line_addr) {
    return l1i_find_timing_cache_entry(line_addr) < L1I_WAY;
  }

  bool l1i_ongoing_request(uint64_t line_addr) {
    DEBUG(eip_proc_id, "l1i_find_timing_mshr_entry: %ld, L1I_TIMING_MSHR_SIZE: %d, ongoing_request: %d\n", l1i_find_timing_mshr_entry(line_addr), L1I_TIMING_MSHR_SIZE, l1i_find_timing_mshr_entry(line_addr) < L1I_TIMING_MSHR_SIZE);
    return l1i_find_timing_mshr_entry(line_addr) < L1I_TIMING_MSHR_SIZE;
  }

  bool l1i_ongoing_accessed_request(uint64_t line_addr) {
    uint32_t index = l1i_find_timing_mshr_entry(line_addr);
    if (index == L1I_TIMING_MSHR_SIZE) return false;
    return l1i_timing_mshr_table[eip_proc_id][index].accessed;
  }

  uint64_t l1i_get_latency_timing_mshr(uint64_t line_addr, uint32_t &pos_hist) {
    uint32_t index = l1i_find_timing_mshr_entry(line_addr);
    if (index == L1I_TIMING_MSHR_SIZE) return 0;
    if (!l1i_timing_mshr_table[eip_proc_id][index].accessed) return 0;
    pos_hist = l1i_timing_mshr_table[eip_proc_id][index].pos_hist;
    return l1i_get_latency(cycle_count, l1i_timing_mshr_table[eip_proc_id][index].timestamp);
  }

  // ENTANGLED TABLE

  uint32_t L1I_ENTANGLED_FORMATS[L1I_ENTANGLED_MAX_FORMATS] = {58, 28, 18, 13, 10, 8, 6};
#define L1I_ENTANGLED_NUM_FORMATS 6

  uint32_t l1i_get_format_entangled(uint64_t line_addr, uint64_t entangled_addr) {
    for (uint32_t i = L1I_ENTANGLED_NUM_FORMATS; i != 0; i--) {
      if ((line_addr >> L1I_ENTANGLED_FORMATS[i-1]) == (entangled_addr >> L1I_ENTANGLED_FORMATS[i-1])) {
        return i;
      }
    }
    ASSERT(eip_proc_id, false);
  }

  uint64_t l1i_extend_format_entangled(uint64_t line_addr, uint64_t entangled_addr, uint32_t format) {
    return ((line_addr >> L1I_ENTANGLED_FORMATS[format-1]) << L1I_ENTANGLED_FORMATS[format-1])
      | (entangled_addr & (((uint64_t)1 << L1I_ENTANGLED_FORMATS[format-1]) - 1));
  }

  uint64_t l1i_compress_format_entangled(uint64_t entangled_addr, uint32_t format) {
    return entangled_addr & (((uint64_t)1 << L1I_ENTANGLED_FORMATS[format-1]) - 1);
  }

#define L1I_MAX_ENTANGLED_PER_LINE L1I_ENTANGLED_NUM_FORMATS

uint32_t L1I_ENTANGLED_TABLE_SETS;
uint32_t L1I_TAG_BITS;
uint32_t L1I_TAG_MASK;

#define L1I_CONFIDENCE_COUNTER_BITS 2
#define L1I_CONFIDENCE_COUNTER_MAX_VALUE ((1 << L1I_CONFIDENCE_COUNTER_BITS) - 1)
#define L1I_CONFIDENCE_COUNTER_THRESHOLD 1
#define L1I_TRIES_AVAIL_ENTANGLED 2

  typedef struct __l1i_entangled_entry {
    uint64_t tag; // L1I_TAG_BITS bits
    uint32_t format; // log2(L1I_ENTANGLED_NUM_FORMATS) bits
    uint64_t entangled_addr[L1I_MAX_ENTANGLED_PER_LINE]; // JUST DIFF
    uint32_t entangled_conf[L1I_MAX_ENTANGLED_PER_LINE]; // L1I_CONFIDENCE_COUNTER_BITS bits
    uint32_t bb_size; // L1I_MERGE_BBSIZE_BITS bits
  } l1i_entangled_entry;

  //l1i_entangled_entry l1i_entangled_table[NUM_CPUS][L1I_ENTANGLED_TABLE_SETS][L1I_ENTANGLED_TABLE_WAYS];
  std::vector<l1i_entangled_entry**> l1i_entangled_table;
  //uint32_t l1i_entangled_fifo[NUM_CPUS][L1I_ENTANGLED_TABLE_SETS]; // log2(L1I_ENTANGLED_TABLE_WAYS) * L1I_ENTANGLED_TABLE_SETS bits
  std::vector<uint32_t*> l1i_entangled_fifo; // log2(L1I_ENTANGLED_TABLE_WAYS) * L1I_ENTANGLED_TABLE_SETS bits

  uint64_t l1i_hash(uint64_t line_addr) {
    return line_addr ^ (line_addr >> 2) ^ (line_addr >> 5);
  }

  void l1i_init_entangled_table() {
    for (uint32_t i = 0; i < L1I_ENTANGLED_TABLE_SETS; i++) {
      for (uint32_t j = 0; j < L1I_ENTANGLED_TABLE_WAYS; j++) {
        l1i_entangled_table[eip_proc_id][i][j].tag = 0;
        l1i_entangled_table[eip_proc_id][i][j].format = 1;
        for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
          l1i_entangled_table[eip_proc_id][i][j].entangled_addr[k] = 0;
          l1i_entangled_table[eip_proc_id][i][j].entangled_conf[k] = 0;
        }
        l1i_entangled_table[eip_proc_id][i][j].bb_size = 0;
      }
      l1i_entangled_fifo[eip_proc_id][i] = 0;
    }
  }

  uint32_t l1i_get_way_entangled_table(uint64_t line_addr) {
    uint64_t tag = (l1i_hash(line_addr) >> L1I_ENTANGLED_TABLE_INDEX_BITS) & L1I_TAG_MASK; 
    uint32_t set = l1i_hash(line_addr) % L1I_ENTANGLED_TABLE_SETS;
    for (uint32_t i = 0; i < L1I_ENTANGLED_TABLE_WAYS; i++) {
      if (l1i_entangled_table[eip_proc_id][set][i].tag == tag) { // Found
        return i;
      }
    }
    return L1I_ENTANGLED_TABLE_WAYS;
  }

  void l1i_try_realocate_evicted_in_available_entangled_table(uint32_t set) {
    uint64_t way = l1i_entangled_fifo[eip_proc_id][set];
    bool dest_free_way = true;
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
        dest_free_way = false;
        break;
      }
    }
    if (dest_free_way && l1i_entangled_table[eip_proc_id][set][way].bb_size == 0) return;
    uint32_t free_way = way;
    bool free_with_size = false;
    for (uint32_t i = (way + 1) % L1I_ENTANGLED_TABLE_WAYS; i != way; i = (i + 1) % L1I_ENTANGLED_TABLE_WAYS) {
      bool dest_free = true;
      for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
        if (l1i_entangled_table[eip_proc_id][set][i].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
          dest_free = false;
          break;
        }
      }
      if (dest_free) {
        if (free_way == way) {
          free_way = i;
          free_with_size = (l1i_entangled_table[eip_proc_id][set][i].bb_size != 0);
        } else if (free_with_size && l1i_entangled_table[eip_proc_id][set][i].bb_size == 0) {
          free_way = i;
          free_with_size = false;
          break;
        }
      }
    }
    if (free_way != way && ((!free_with_size) || (free_with_size && !dest_free_way))) { // Only evict if it has more information 
      l1i_entangled_table[eip_proc_id][set][free_way].tag = l1i_entangled_table[eip_proc_id][set][way].tag;
      l1i_entangled_table[eip_proc_id][set][free_way].format = l1i_entangled_table[eip_proc_id][set][way].format;
      for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
        l1i_entangled_table[eip_proc_id][set][free_way].entangled_addr[k] = l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k];
        l1i_entangled_table[eip_proc_id][set][free_way].entangled_conf[k] = l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k];
      }
      l1i_entangled_table[eip_proc_id][set][free_way].bb_size = l1i_entangled_table[eip_proc_id][set][way].bb_size;
    }
  }

  void l1i_add_entangled_table(uint64_t line_addr, uint64_t entangled_addr) {
    uint64_t tag = (l1i_hash(line_addr) >> L1I_ENTANGLED_TABLE_INDEX_BITS) & L1I_TAG_MASK; 
    uint32_t set = l1i_hash(line_addr) % L1I_ENTANGLED_TABLE_SETS;
    uint32_t way = l1i_get_way_entangled_table(line_addr);
    if (way == L1I_ENTANGLED_TABLE_WAYS) {
      l1i_try_realocate_evicted_in_available_entangled_table(set);
      way = l1i_entangled_fifo[eip_proc_id][set];
      l1i_entangled_table[eip_proc_id][set][way].tag = tag;
      l1i_entangled_table[eip_proc_id][set][way].format = 1;
      for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
        l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k] = 0;
        l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] = 0;
      }
      l1i_entangled_table[eip_proc_id][set][way].bb_size = 0;
      l1i_entangled_fifo[eip_proc_id][set] = (l1i_entangled_fifo[eip_proc_id][set] + 1) % L1I_ENTANGLED_TABLE_WAYS;
    }
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD
          && l1i_extend_format_entangled(line_addr, l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k], l1i_entangled_table[eip_proc_id][set][way].format) == entangled_addr) {
        l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] = L1I_CONFIDENCE_COUNTER_MAX_VALUE;
        return;
      }
    }

    // Adding a new entangled
    uint32_t format_new = l1i_get_format_entangled(line_addr, entangled_addr);
    l1i_stats_formats[format_new-1]++;

    // Check for evictions
    while(true) {
      uint32_t min_format = format_new;
      uint32_t num_valid = 1;
      uint32_t min_value = L1I_CONFIDENCE_COUNTER_MAX_VALUE + 1;
      uint32_t min_pos = 0;
      for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
        if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
          num_valid++;
          uint32_t format_k = l1i_get_format_entangled(line_addr, l1i_extend_format_entangled(line_addr, l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k], l1i_entangled_table[eip_proc_id][set][way].format));
          if (format_k < min_format) {
            min_format = format_k;
          }
          if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] < min_value) {
            min_value = l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k];
            min_pos = k;
          }
        }
      }
      if (num_valid > min_format) { // Eviction is necessary. We chose the lower confidence one 
        l1i_stats_evict_entangled_k_table++;
        l1i_entangled_table[eip_proc_id][set][way].entangled_conf[min_pos] = 0;
      } else {
        // Reformat
        for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
          if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
            l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k] = l1i_compress_format_entangled(l1i_extend_format_entangled(line_addr, l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k], l1i_entangled_table[eip_proc_id][set][way].format), min_format);
          }
        }
        l1i_entangled_table[eip_proc_id][set][way].format = min_format;
        break;
      }
    }
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] < L1I_CONFIDENCE_COUNTER_THRESHOLD) {
        l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k] = l1i_compress_format_entangled(entangled_addr, l1i_entangled_table[eip_proc_id][set][way].format);
        l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] = L1I_CONFIDENCE_COUNTER_MAX_VALUE;
        return;
      }
    }
  }

  bool l1i_avail_entangled_table(uint64_t line_addr, uint64_t entangled_addr, bool insert_not_present) {
    uint32_t set = l1i_hash(line_addr) % L1I_ENTANGLED_TABLE_SETS;
    uint32_t way = l1i_get_way_entangled_table(line_addr);
    if (way == L1I_ENTANGLED_TABLE_WAYS) return insert_not_present;
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD
          && l1i_extend_format_entangled(line_addr, l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k], l1i_entangled_table[eip_proc_id][set][way].format) == entangled_addr) {
        return true;
      }
    }
    // Check for availability
    uint32_t min_format = l1i_get_format_entangled(line_addr, entangled_addr);
    uint32_t num_valid = 1;
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
        num_valid++;
        uint32_t format_k = l1i_get_format_entangled(line_addr, l1i_extend_format_entangled(line_addr, l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k], l1i_entangled_table[eip_proc_id][set][way].format));
        if (format_k < min_format) {
          min_format = format_k;
        }
      }
    }
    if (num_valid > min_format) { // Eviction is necessary
      return false;
    } else {
      return true;
    }
  }

  void l1i_add_bbsize_table(uint64_t line_addr, uint32_t bb_size) {
    uint64_t tag = (l1i_hash(line_addr) >> L1I_ENTANGLED_TABLE_INDEX_BITS) & L1I_TAG_MASK; 
    uint32_t set = l1i_hash(line_addr) % L1I_ENTANGLED_TABLE_SETS;
    uint32_t way = l1i_get_way_entangled_table(line_addr);
    if (way == L1I_ENTANGLED_TABLE_WAYS) {
      l1i_try_realocate_evicted_in_available_entangled_table(set);
      way = l1i_entangled_fifo[eip_proc_id][set];
      l1i_entangled_table[eip_proc_id][set][way].tag = tag;
      l1i_entangled_table[eip_proc_id][set][way].format = 1;
      for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
        l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k] = 0;
        l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] = 0;
      }
      l1i_entangled_table[eip_proc_id][set][way].bb_size = 0;
      l1i_entangled_fifo[eip_proc_id][set] = (l1i_entangled_fifo[eip_proc_id][set] + 1) % L1I_ENTANGLED_TABLE_WAYS;
    }
    if (bb_size > l1i_entangled_table[eip_proc_id][set][way].bb_size) {
      l1i_entangled_table[eip_proc_id][set][way].bb_size = bb_size & L1I_MERGE_BBSIZE_MAX_VALUE;
    }
    if (bb_size > l1i_stats_max_bb_size) {
      l1i_stats_max_bb_size = bb_size;
    }
  }

  uint64_t l1i_get_entangled_addr_entangled_table(uint64_t line_addr, uint32_t index_k, uint32_t &set, uint32_t &way) {
    set = l1i_hash(line_addr) % L1I_ENTANGLED_TABLE_SETS;
    way = l1i_get_way_entangled_table(line_addr);
    if (way < L1I_ENTANGLED_TABLE_WAYS) {
      if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[index_k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD) {
        return l1i_extend_format_entangled(line_addr, l1i_entangled_table[eip_proc_id][set][way].entangled_addr[index_k], l1i_entangled_table[eip_proc_id][set][way].format);
      }
    }
    return 0;
  }

  uint32_t l1i_get_bbsize_entangled_table(uint64_t line_addr) {
    uint32_t set = l1i_hash(line_addr) % L1I_ENTANGLED_TABLE_SETS;
    uint32_t way = l1i_get_way_entangled_table(line_addr);
    if (way < L1I_ENTANGLED_TABLE_WAYS) {
      return l1i_entangled_table[eip_proc_id][set][way].bb_size;
    }
    return 0;
  }

  void l1i_update_confidence_entangled_table(uint32_t set, uint32_t way, uint64_t entangled_addr, bool accessed) {
    if (way < L1I_ENTANGLED_TABLE_WAYS) {
      for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
        if (l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] >= L1I_CONFIDENCE_COUNTER_THRESHOLD
            && l1i_compress_format_entangled(l1i_entangled_table[eip_proc_id][set][way].entangled_addr[k], l1i_entangled_table[eip_proc_id][set][way].format) == l1i_compress_format_entangled(entangled_addr, l1i_entangled_table[eip_proc_id][set][way].format)) {
          if (accessed && l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] < L1I_CONFIDENCE_COUNTER_MAX_VALUE) {
            l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k]++;
          }
          if (!accessed && l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k] > 0) {
            l1i_entangled_table[eip_proc_id][set][way].entangled_conf[k]--;
          }
        }
      }
    }
  }

  // INTERFACE
void alloc_mem_eip(uns numCores) {
  if (!EIP_ENABLE)
    return;

  L1I_ENTANGLED_TABLE_SETS = (1 << L1I_ENTANGLED_TABLE_INDEX_BITS);
  L1I_TAG_BITS = (19 - L1I_ENTANGLED_TABLE_INDEX_BITS);
  L1I_TAG_MASK = (((uint64_t)1 << L1I_TAG_BITS) - 1);

  ASSERT(eip_proc_id, MEM_REQ_BUFFER_ENTRIES > 16);
  L1I_RQ_SIZE = QUEUE_L1_SIZE == 0 ? MEM_REQ_BUFFER_ENTRIES : QUEUE_L1_SIZE;
  ASSERT(eip_proc_id, L1I_RQ_SIZE > 0);
  L1I_SET = ICACHE_SIZE / ICACHE_LINE_SIZE / ICACHE_ASSOC;
  L1I_WAY = ICACHE_ASSOC;
  int L1I_MSHR_SIZE = (MEM_REQ_BUFFER_ENTRIES <= 64 && MEM_REQ_BUFFER_ENTRIES > 16) ? 16 : MEM_REQ_BUFFER_ENTRIES/4;
  L1I_TIMING_MSHR_SIZE = FE_FTQ_BLOCK_NUM + L1I_MSHR_SIZE + L1I_RQ_SIZE;
  DEBUG(eip_proc_id, "L1I_RQ_SIZE: %d, L1I_SET: %d, L1I_WAY: %d, L1I_TIMING_MSHR_SIZE: %d\n", L1I_RQ_SIZE, L1I_SET, L1I_WAY, L1I_TIMING_MSHR_SIZE);
  l1i_stats_table.resize(numCores);
  for (auto it = l1i_stats_table.begin(); it != l1i_stats_table.end(); ++it)
    *it = (l1i_stats_entry*)malloc(sizeof(l1i_stats_entry) * L1I_STATS_TABLE_ENTRIES);
  l1i_hist_table.resize(numCores);
  for (auto it = l1i_hist_table.begin(); it != l1i_hist_table.end(); ++it)
    *it = (l1i_hist_entry*)malloc(sizeof(l1i_hist_entry) * L1I_HIST_TABLE_ENTRIES);
  l1i_hist_table_head.resize(numCores);
  l1i_hist_table_head_time.resize(numCores);
  l1i_timing_mshr_table.resize(numCores);
  for (auto it = l1i_timing_mshr_table.begin(); it != l1i_timing_mshr_table.end(); ++it)
    *it = (l1i_timing_mshr_entry*)malloc(sizeof(l1i_timing_mshr_entry) * L1I_TIMING_MSHR_SIZE);
  l1i_timing_cache_table.resize(numCores);
  for (auto it = l1i_timing_cache_table.begin(); it != l1i_timing_cache_table.end(); ++it) {
    *it = (l1i_timing_cache_entry**)malloc(sizeof(l1i_timing_cache_entry*) * L1I_SET);
    for (uint32_t i = 0; i < L1I_SET; i++)
      (*it)[i] = (l1i_timing_cache_entry*)malloc(sizeof(l1i_timing_cache_entry) * L1I_WAY);
  }
  l1i_entangled_table.resize(numCores);
  for (auto it = l1i_entangled_table.begin(); it != l1i_entangled_table.end(); ++it) {
    *it = (l1i_entangled_entry**)malloc(sizeof(l1i_entangled_entry*) * L1I_ENTANGLED_TABLE_SETS);
    for (uint32_t i = 0; i < L1I_ENTANGLED_TABLE_SETS; i++)
      (*it)[i] = (l1i_entangled_entry*)malloc(sizeof(l1i_entangled_entry) * L1I_ENTANGLED_TABLE_WAYS);
  }
  l1i_entangled_fifo.resize(numCores);
  for (auto it = l1i_entangled_fifo.begin(); it != l1i_entangled_fifo.end(); ++it)
    *it = (uint32_t*)malloc(sizeof(uint32_t) * L1I_ENTANGLED_TABLE_SETS);
}

  void init_eip(uns proc_id) {
    if (!EIP_ENABLE)
      return;

    ASSERT(proc_id, WP_COLLECT_STATS);

    eip_proc_id = proc_id;
    l1i_init_stats_table();
    l1i_last_basic_block = 0;
    l1i_consecutive_count = 0;
    l1i_basic_block_merge_diff = 0;

    l1i_init_hist_table();
    l1i_init_timing_tables();
    l1i_init_entangled_table();
  }

  void eip_prefetch(uns proc_id, uint64_t v_addr, uint8_t cache_hit, uint8_t prefetch_hit, Flag off_path)
  {
    uint64_t line_addr = v_addr >> LOG2(ICACHE_LINE_SIZE);
    DEBUG(proc_id, "eip_prefetch 0x%lx, icache_line_addr 0x%lx line_addr 0x%lx, cache hit %d, prefetch_hit %d, l1i_find_timing_cache_entry %lu\n", v_addr, v_addr & ~0x3F, line_addr, cache_hit, prefetch_hit, l1i_find_timing_cache_entry(line_addr));

    if (!cache_hit) ASSERT(eip_proc_id, !prefetch_hit);
    if (!cache_hit) ASSERT(eip_proc_id, l1i_find_timing_cache_entry(line_addr) == L1I_WAY);
    if (cache_hit) ASSERT(eip_proc_id, l1i_find_timing_cache_entry(line_addr) < L1I_WAY);

    l1i_stats_table[proc_id][(line_addr & L1I_STATS_TABLE_MASK)].accesses++;
    if (!cache_hit) {
      l1i_stats_table[proc_id][(line_addr & L1I_STATS_TABLE_MASK)].misses++;
      if (l1i_ongoing_request(line_addr)
          && !l1i_is_accessed_timing_entry(line_addr)) {
        l1i_stats_table[proc_id][(line_addr & L1I_STATS_TABLE_MASK)].late++;
      }
    }
    if (prefetch_hit) {
      l1i_stats_table[proc_id][(line_addr & L1I_STATS_TABLE_MASK)].hits++;
    }

    bool consecutive = false;

    if (l1i_last_basic_block + l1i_consecutive_count == line_addr) { // Same
      return;
    } else if (l1i_last_basic_block + l1i_consecutive_count + 1 == line_addr) { // Consecutive
      l1i_consecutive_count++;
      consecutive = true;
    }
    if (!FDIP_ENABLE)
      per_cyc_ipref = 0;

    // Queue basic block prefetches
    uint32_t bb_size = l1i_get_bbsize_entangled_table(line_addr);
    if (bb_size) l1i_stats_basic_blocks[bb_size]++;
    for (uint32_t i = 1; i <= bb_size; i++) {
      uint64_t pf_addr = v_addr + i * (1<<LOG2(ICACHE_LINE_SIZE));
      if (!l1i_ongoing_request(pf_addr >> LOG2(ICACHE_LINE_SIZE))) {
        Flag success = FALSE;
        // TODO : limit per-cycle prefetches
        //if (per_cyc_ipref < MAX_FTQ_ENTRY_CYC)
        success = new_mem_req(MRT_IPRF, eip_proc_id, pf_addr, ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count, 0);
        if (success) {
          DEBUG(proc_id, "new_mem_req (BB pref) for v_addr 0x%lx,line_addr 0x%lx  pf_addr 0x%lx, unique_count: %llu\n", v_addr, v_addr & ~0x3F, pf_addr, unique_count);
          if (!off_path)
            l1i_add_timing_entry(pf_addr >> LOG2(ICACHE_LINE_SIZE), 0, L1I_ENTANGLED_TABLE_WAYS);
          //if (success == Mem_Queue_Req_Result::SUCCESS_NEW)
            //per_cyc_ipref++;
        }
      }
    }

    // Queue entangled and basic block of entangled prefetches
    uint32_t num_entangled = 0;
    for (uint32_t k = 0; k < L1I_MAX_ENTANGLED_PER_LINE; k++) {
      uint32_t source_set = 0;
      uint32_t source_way = L1I_ENTANGLED_TABLE_WAYS;
      uint64_t entangled_line_addr = l1i_get_entangled_addr_entangled_table(line_addr, k, source_set, source_way);
      if (entangled_line_addr && (entangled_line_addr != line_addr)) {
        num_entangled++;
        uint32_t bb_size = l1i_get_bbsize_entangled_table(entangled_line_addr);
        if (bb_size) l1i_stats_basic_blocks_ent[bb_size]++;
        for (uint32_t i = 0; i <= bb_size; i++) {
          uint64_t pf_line_addr = entangled_line_addr + i;
          if (!l1i_ongoing_request(pf_line_addr)) {
            Flag success = FALSE;
            // TODO : limit per-cycle prefetches
            //if (per_cyc_ipref < MAX_FTQ_ENTRY_CYC)
            success = new_mem_req(MRT_IPRF, eip_proc_id, pf_line_addr << LOG2(ICACHE_LINE_SIZE), ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count, 0);
            if (success) {
              DEBUG(proc_id, "new_mem_req (Entangled pref) for 0x%lx, unique_count: %llu\n", pf_line_addr << LOG2(ICACHE_LINE_SIZE), unique_count);
              if (!off_path)
                l1i_add_timing_entry(pf_line_addr, source_set, (i == 0) ? source_way : L1I_ENTANGLED_TABLE_WAYS);
              //if (success == Mem_Queue_Req_Result::SUCCESS_NEW)
                //per_cyc_ipref++;
            }
          }
        }
      }
    }
    if (num_entangled) l1i_stats_entangled[num_entangled]++; 

    if (!consecutive) { // New basic block found
      uint32_t max_bb_size = l1i_get_bbsize_entangled_table(l1i_last_basic_block);

      // Check for merging bb opportunities
      if (l1i_consecutive_count) { // single blocks no need to merge and are not inserted in the entangled table
        if (l1i_basic_block_merge_diff > 0) {
          l1i_add_bbsize_table(l1i_last_basic_block - l1i_basic_block_merge_diff, l1i_consecutive_count + l1i_basic_block_merge_diff);
          l1i_add_bb_size_hist_table(l1i_last_basic_block - l1i_basic_block_merge_diff, l1i_consecutive_count + l1i_basic_block_merge_diff);
        } else {
          l1i_add_bbsize_table(l1i_last_basic_block, std::max(max_bb_size, l1i_consecutive_count));
          l1i_add_bb_size_hist_table(l1i_last_basic_block, std::max(max_bb_size, l1i_consecutive_count));
        }
      }
    }

    if (!consecutive) { // New basic block found
      l1i_consecutive_count = 0;
      l1i_last_basic_block = line_addr;
    }  

    if (!consecutive) {
      l1i_basic_block_merge_diff = l1i_find_bb_merge_hist_table(l1i_last_basic_block);
    }

    // Add the request in the history buffer
    uint32_t pos_hist = L1I_HIST_TABLE_ENTRIES; 
    if (!consecutive && l1i_basic_block_merge_diff == 0) {
      if ((l1i_find_hist_entry(line_addr) == L1I_HIST_TABLE_ENTRIES)) {
        pos_hist = l1i_add_hist_table(line_addr);
      } else {
        if (!cache_hit && !l1i_ongoing_accessed_request(line_addr)) {
          pos_hist = l1i_add_hist_table(line_addr);      
        }
      }
    }

    // Add miss in the latency table
    if (!cache_hit && !l1i_ongoing_request(line_addr) && !off_path) {
      l1i_add_timing_entry(line_addr, 0, L1I_ENTANGLED_TABLE_WAYS);
    }

    uint32_t source_set = 0;
    uint32_t source_way = L1I_ENTANGLED_TABLE_WAYS;
    l1i_access_timing_entry(line_addr, pos_hist, source_set, source_way);

    // Update confidence if late
    if (source_way < L1I_ENTANGLED_TABLE_WAYS) {
      l1i_update_confidence_entangled_table(source_set, source_way, line_addr, false);
    }

  }

void set_eip(uns _proc_id) {
  eip_proc_id = _proc_id;
}

  void update_eip() {
    if (!EIP_ENABLE)
      return;
  }

  void eip_cache_fill(uns proc_id, uint64_t v_addr, uint64_t evicted_v_addr)
  {
    eip_proc_id = proc_id;
    uint64_t line_addr = (v_addr >> LOG2(ICACHE_LINE_SIZE));
    uint64_t evicted_line_addr = (evicted_v_addr >> LOG2(ICACHE_LINE_SIZE));
    DEBUG(proc_id, "eip_cache_fill 0x%lx, icache_line_addr 0x%lx line_addr 0x%lx, evicted_v_addr 0x%lx\n", v_addr, v_addr & ~0x3F, line_addr, evicted_v_addr);

    // Line is in cache
    if (evicted_v_addr) {
      uint32_t source_set = 0;
      uint32_t source_way = L1I_ENTANGLED_TABLE_WAYS;
      bool accessed = l1i_invalid_timing_cache_entry(evicted_line_addr, source_set, source_way);
      if (!accessed) {
        l1i_stats_table[proc_id][(evicted_line_addr & L1I_STATS_TABLE_MASK)].wrong++;
      }
      if (source_way < L1I_ENTANGLED_TABLE_WAYS) {
        // If accessed hit, but if not wrong
        l1i_update_confidence_entangled_table(source_set, source_way, evicted_line_addr, accessed);
      }
    }

    uint32_t pos_hist = L1I_HIST_TABLE_ENTRIES; 
    uint64_t latency = l1i_get_latency_timing_mshr(line_addr, pos_hist);

    l1i_move_timing_entry(line_addr);

    // Get and update entangled
    if (latency && pos_hist < L1I_HIST_TABLE_ENTRIES) {
      bool inserted = false;
      for (uint32_t i = 0; i < L1I_TRIES_AVAIL_ENTANGLED; i++) {
        uint64_t src_entangled = l1i_get_src_entangled_hist_table(line_addr, pos_hist, latency, i);
        if (src_entangled && line_addr != src_entangled) {
          if (l1i_avail_entangled_table(src_entangled, line_addr, false)) {
            l1i_add_entangled_table(src_entangled, line_addr);
            inserted = true;
            break;
          }
        }
      }
      if (!inserted) {
        uint64_t src_entangled = l1i_get_src_entangled_hist_table(line_addr, pos_hist, latency);
        if (src_entangled && line_addr != src_entangled) {
          l1i_add_entangled_table(src_entangled, line_addr);
        }
      }
    }
  }

  void print_eip_stats(uns proc_id)
  {
    l1i_print_stats_table();
  }

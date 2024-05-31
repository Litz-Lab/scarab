/***************************************************************************************
 * File         : uop_cache.h
 * Author       : Peter Braun
 * Date         : 10.28.2020
 * Description  : Interface for interacting with uop cache object.
 *                  Following Kotra et. al.'s MICRO 2020 description of uop cache baseline
 ***************************************************************************************/

#include "debug/debug_macros.h"
#include "debug/debug_print.h"
#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"
#include "isa/isa_macros.h"

#include "bp/bp.h"
#include "op_pool.h"

#include "core.param.h"
#include "debug/debug.param.h"
#include "general.param.h"
#include "statistics.h"

#include "libs/cache_lib.h"
#include "memory/memory.h"
#include "memory/memory.param.h"
#include "uop_cache.h"
#include "icache_stage.h"
#include "libs/cpp_cache.h"
#include "uop_queue_stage.h"

/**************************************************************************************/
/* Macros */

#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_UOP_CACHE, ##args)

// Uop cache is byte-addressable, so tag/set index are generated from full address (no offset)
// Uop cache uses icache tag + icache offset as full TAG
#define UOP_CACHE_LINE_SIZE       ICACHE_LINE_SIZE
#define UOP_QUEUE_SIZE            1000 // at least UOP_CACHE_ASSOC * ISSUE_WIDTH
#define UOP_CACHE_LINE_DATA_SIZE  sizeof(Uop_Cache_Data)
#define UOP_CACHE_NAME            "UOP_CACHE"

/**************************************************************************************/
/* Local Prototypes */

static inline Flag insert_uop_cache(void);
static inline Flag in_uop_cache_search(Addr search_addr, Flag update_repl, Flag offpath);

/**************************************************************************************/
/* Global Variables */

uns8 proc_id;
static Flag UOP_CACHE_ON;

// uop trace/bbl accumulation
static Uop_Cache_Data accumulating_pw = {0};
static Counter next_accum_op = 0;
static Op* uop_q[UOP_QUEUE_SIZE];

// k: instr addr
Hash_Table inf_size_uop_cache;
// indexed by start addr of PW
Hash_Table pc_to_pw;

Addr addr_following_resteer_bf = 0;  // Addr that follows a resteer or fetch barrier
Flag uop_cache_insert_enable = TRUE;

/**************************************************************************************/
/* init_uop_cache */

void init_uop_cache(uns8 pid) {
  proc_id = pid;
  UOP_CACHE_ON = UOP_CACHE_ENABLE && (UOP_CACHE_LINES || INF_SIZE_UOP_CACHE || ORACLE_PERFECT_UOP_CACHE);
  if (!UOP_CACHE_ON) {
    return;
  }
  if (INF_SIZE_UOP_CACHE || INF_SIZE_UOP_CACHE_PW_SIZE_LIM) {
    init_hash_table(&inf_size_uop_cache, "infinite sized uop cache", 15000000, sizeof(int));
  }

  // The cache library computes the number of entries from cache_size_bytes/cache_line_size_bytes,
  cpp_cache_create(UOP_CACHE_NAME, UOP_CACHE_LINES, UOP_CACHE_ASSOC, UOP_CACHE_LINE_SIZE,
                   UOP_CACHE_REPL, /*tag_incl_offset=*/TRUE, UOP_CACHE_LINE_DATA_SIZE);
}

Flag pw_insert(Uop_Cache_Data pw) {
  Uop_Cache_Data* cur_line_data = NULL;
  Addr line_addr;
  static Addr* evicted_pws = NULL;
  UNUSED(line_addr);
  pw.used = 0;

  if (evicted_pws == NULL)
    evicted_pws = (Addr*)calloc(UOP_CACHE_ASSOC, sizeof(Addr));

  int lines_needed = pw.n_uops / ISSUE_WIDTH;
  if (pw.n_uops % ISSUE_WIDTH) lines_needed++;
  ASSERT(0, lines_needed > 0);

  if (UOP_CACHE_INSERT_ONLY_ONPATH && pw.first_op_offpath) {
    STAT_EVENT(ic->proc_id, UOP_CACHE_PW_INSERT_FAILED_OFFPATH);
    return FALSE;
  } else if (lines_needed > UOP_CACHE_ASSOC) {  // Is the PW too big?
    STAT_EVENT(ic->proc_id, UOP_CACHE_PW_INSERT_FAILED_TOO_LONG);
    return FALSE;
  } else if (cpp_cache_access(UOP_CACHE_NAME, pw.first, FALSE, FALSE)) {
    STAT_EVENT(ic->proc_id, UOP_CACHE_PW_INSERT_FAILED_CACHE_HIT);
    return FALSE;
  } else {
    memset(evicted_pws, 0, UOP_CACHE_ASSOC*sizeof(*evicted_pws));
    cur_line_data = (Uop_Cache_Data*) cpp_cache_insert(UOP_CACHE_NAME, pw.first, lines_needed,
                                                       0, evicted_pws);
    *cur_line_data = pw;
    for (int i = 0; i < UOP_CACHE_ASSOC && evicted_pws[i]; i++) {
      DEBUG(ic->proc_id, "Evicted PW: %llx\n", evicted_pws[i]);
    }
    DEBUG(ic->proc_id,
          "PW inserted. off_path=%u, addr=0x%llx, set=%u, lines_needed=%i, ",
          pw.first_op_offpath, pw.first,
          cpp_cache_index(UOP_CACHE_NAME, pw.first, &line_addr, &line_addr),
          lines_needed);
    STAT_EVENT(0, UOP_CACHE_PWS_INSERTED_ONPATH + pw.first_op_offpath);
    INC_STAT_EVENT(0, UOP_CACHE_LINES_INSERTED_ONPATH + pw.first_op_offpath, lines_needed);
  }
  return TRUE;
}

/**************************************************************************************/
/* insert_uop_cache: private method, only called by accumulate_op
 *                   Drain buffer and insert. Return whether inserted.
 */
Flag insert_uop_cache() {
  // PW may span multiple cache entries. 1 entry per line. Additional terminating conditions per line:
  // 1. max uops per line
  // 2. max imm/disp per line
  // 3. max micro-coded instr per line (not simulated)

  // Invalidate after hitting maximum allowed number of lines, as in gem5
  // Each entry/pw indexed by physical addr of 1st instr, so insert each line using same addr
  // Invalidation: Let LRU handle it by placing PW in one line at a time.   
  
  ASSERT(0, accumulating_pw.n_uops);
  Flag success = FALSE;
  Flag new_entry;

  int lines_needed = accumulating_pw.n_uops / ISSUE_WIDTH;
  if (accumulating_pw.n_uops % ISSUE_WIDTH) lines_needed++;

  if (INF_SIZE_UOP_CACHE || (INF_SIZE_UOP_CACHE_PW_SIZE_LIM 
      && accumulating_pw.n_uops <= INF_SIZE_UOP_CACHE_PW_SIZE_LIM)) {
    for (int ii = 0; ii < accumulating_pw.n_uops; ii++) {
      Op* op = uop_q[ii];
      if (!op->off_path)
        hash_table_access_create(&inf_size_uop_cache, op->inst_info->addr, 
                                 &new_entry);
    }
    success = TRUE;
  } else if (INF_SIZE_UOP_CACHE_PW_SIZE_LIM 
            && accumulating_pw.n_uops > INF_SIZE_UOP_CACHE_PW_SIZE_LIM) {
    success = FALSE;
  } else if (UOP_CACHE_INSERT_ONLY_AFTER_RESTEER_UOP_QUEUE_NOT_FULL
             && !uop_cache_insert_enable) {
    success = FALSE;
  } else {
    success = pw_insert(accumulating_pw);
  }

  return success;
}

Uop_Cache_Line_Data* uop_cache_lookup_line(Addr line_addr, Addr ft_start, Addr ft_length, Flag update_repl) {
  if (!UOP_CACHE_ENABLE) {
    return NULL;
  }
  return NULL;
}

static inline Flag in_uop_cache_search(Addr search_addr, Flag update_repl, Flag offpath) {
  static Uop_Cache_Data cur_pw = {0};
  Addr line_addr;
  UNUSED(line_addr);
  Uop_Cache_Data* uoc_data = NULL;

  // First check if current pw has this search addr
  if (cur_pw.first && search_addr >= cur_pw.first
                   && search_addr <= cur_pw.last) {
    uoc_data = &cur_pw;
    if (uoc_data)
      DEBUG(ic->proc_id, "UOC hit (cur_pw). addr=0x%llx, set=%u, update_repl=%u\n",
            search_addr, cpp_cache_index(UOP_CACHE_NAME, search_addr, &line_addr, &line_addr),
            update_repl);
  } else {
    // Next try to access a new PW starting at this addr
    uoc_data = cpp_cache_access(UOP_CACHE_NAME, search_addr, update_repl, 0);
    // Only update state if this access should change state
    if (update_repl && uoc_data) {
      if (!uoc_data->used && !offpath) {
        STAT_EVENT(0, UOP_CACHE_LINES_INSERTED_ONPATH_USED_ONPATH + uoc_data->first_op_offpath);
      }
      uoc_data->used += 1;
      cur_pw = *uoc_data;
      DEBUG(ic->proc_id, "UOC hit (new PW). addr=0x%llx, set=%u\n",
            search_addr, cpp_cache_index(UOP_CACHE_NAME, search_addr, &line_addr, &line_addr));
    } else if (update_repl) {
      memset(&cur_pw, 0, sizeof(cur_pw));
      DEBUG(ic->proc_id, "UOC miss. addr=0x%llx, set=%u\n",  
            search_addr, cpp_cache_index(UOP_CACHE_NAME, search_addr, &line_addr, &line_addr));
      // maybe also add uop granularity, to make sure ending conditions are OK
    }
  }

  return uoc_data != NULL;
}

/**************************************************************************************/
/* in_uop_cache: Iterate over all possible ops and check if contained.
 *                    Other option: use cache to simulate capacity and maintain a map 
 *                      data structure for pcs
 */
Flag in_uop_cache(Addr pc, Flag update_repl, Flag offpath) {
  // A PW can span multiple cache entries. The next line used is either physically 
  // next in the set (use flag) or anywhere in the set (use pointer), depending on impl.
  // Here, don't care about order, just search all lines for pc in question
  if (!UOP_CACHE_ON) {
    return FALSE;
  }

  if (ORACLE_PERFECT_UOP_CACHE) {
    if (update_repl) {
      STAT_EVENT(0, UOP_CACHE_HIT);
      STAT_EVENT(ic->proc_id, UOP_CACHE_HIT_ONPATH + offpath);
    }
    return TRUE;
  } else if (INF_SIZE_UOP_CACHE || INF_SIZE_UOP_CACHE_PW_SIZE_LIM) {
    return hash_table_access(&inf_size_uop_cache, pc) != NULL;
  }

  Flag found = in_uop_cache_search(pc, update_repl, offpath);
  if (update_repl) {
    STAT_EVENT(0, UOP_CACHE_MISS + found);
    STAT_EVENT(ic->proc_id, UOP_CACHE_MISS_ONPATH + 2*found + offpath);
  }
  
  return found;
}

void end_accumulate(void) {
  if (!UOP_CACHE_ON) {
    return;
  }

  if (accumulating_pw.n_uops > 0) {
    Stat_Enum stat_to_inc = UOP_CACHE_PW_LENGTH_1 + (accumulating_pw.n_uops-1);
    if (stat_to_inc > UOP_CACHE_PW_LENGTH_MAX) stat_to_inc = UOP_CACHE_PW_LENGTH_MAX;
    STAT_EVENT(ic->proc_id, stat_to_inc);
    insert_uop_cache();
    memset(&accumulating_pw, 0, sizeof(accumulating_pw));
  }
}

/**************************************************************************************/
/* accumulate_op: accumulate into buffer. Insert into cache at end of PW. */
void accumulate_op(Op* op) {
  // Prediction Window termination conditions:
  // 1. end of icache line
  // 2. branch predicted taken
  // 3. predetermined number of branch NT (no limit in implementation)
  // 4. uop queue full
  // 5. too many uops to fit in entire set, even after evicting all entries

  // It is possible for an instr to be partially in 2 lines;
  // for pw termination purposes, assume it is in first line.
  if (!UOP_CACHE_ON) {
    return;
  }

  Addr cur_icache_line_addr = get_cache_line_addr(&ic->icache,
                                                  accumulating_pw.first);
  Addr icache_line_addr = get_cache_line_addr(&ic->icache, op->inst_info->addr);

  // Skipped ops means that there was a uop cache hit, so the accumulating_pw
  // may need to be flushed. This must be done in the same place accumulation is done.
  if (op->op_num > next_accum_op) {
    STAT_EVENT(0, UOP_CACHE_ICACHE_SWITCH_ONPATH + op->off_path);
    end_accumulate();
    cur_icache_line_addr = 0;
  }
  ASSERT(proc_id, op->op_num >= next_accum_op);
  if (!cur_icache_line_addr) {
    accumulating_pw.first = op->inst_info->addr;
    accumulating_pw.first_op_offpath = op->off_path;
    cur_icache_line_addr = icache_line_addr;
    next_accum_op = op->op_num + 1;
  } else {
    ASSERT(0, op->op_num == next_accum_op);
    next_accum_op++;
  }
  DEBUG(proc_id, "next_accum_op updated to %llu\n", next_accum_op);
  
  Flag end_of_icache_line = icache_line_addr != cur_icache_line_addr;
  Flag uop_q_full = (accumulating_pw.n_uops + 1 == UOP_QUEUE_SIZE);
  if (end_of_icache_line || uop_q_full) {
    end_accumulate();
  }

  if (accumulating_pw.n_uops == 0) {
    // occurs when THIS fxn call drains the uop queue (insertion into uop cache)
    accumulating_pw.first = op->inst_info->addr;
  }
  uop_q[accumulating_pw.n_uops] = op;
  accumulating_pw.last = op->inst_info->addr;
  accumulating_pw.last_op_num = op->op_num;
  accumulating_pw.n_uops++;

  // PW should be terminated at predicted-taken, but we terminate if actually-taken.
  Flag branch_taken = op->table_info->cf_type && op->oracle_info.dir == TAKEN;
  if (branch_taken) {
    end_accumulate();
  }
};

void recover_uop_cache(void) {
  if (accumulating_pw.last_op_num > bp_recovery_info->recovery_op_num) {
    memset(&accumulating_pw, 0, sizeof(accumulating_pw));
  } else {
    end_accumulate();
  }
  // Ops may be decoded out of order: e.g. when a previous op is slowly getting 
  // decoded, and the current op is (speculatively) fetched already-decoded from the uop cache.
  // So, ops preceding the recovering op may not have called accumulate_op yet.
  if (next_accum_op > bp_recovery_info->recovery_op_num) {
    next_accum_op = bp_recovery_info->recovery_op_num + 1;
    DEBUG(proc_id, "UOC recovery. next_accum_op reset to %llu\n", next_accum_op);
  }
  if (UOP_CACHE_INSERT_ONLY_AFTER_RESTEER_UOP_QUEUE_NOT_FULL && get_uop_queue_stage_length() < UOP_QUEUE_LENGTH) {
    set_uop_cache_insert_enable(TRUE);
  }
}

void set_uop_cache_insert_enable(Flag new_val) {
  uop_cache_insert_enable = new_val;
}

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
 * File         : icache_stage.h
 * Author       : HPS Research Group
 * Date         : 12/7/1998
 * Description  :
 ***************************************************************************************/

#ifndef __ICACHE_STAGE_H__
#define __ICACHE_STAGE_H__

#include "globals/global_types.h"
#include "libs/cache_lib.h"
#include "stage_data.h"
#include "decoupled_frontend.h"

#define IC_ISSUE_WIDTH  ISSUE_WIDTH - DECODE_PATH_WIDTH_NARROWER
#define UOPC_ISSUE_WIDTH  ISSUE_WIDTH

/**************************************************************************************/
/* Forward Declarations */

struct Inst_Info_struct;
struct Mem_Req_struct;

/**************************************************************************************/
/* Types */

/* name strings are in debug/debug_print.c */
typedef enum Icache_State_enum {
  SERVING_INIT,
  ICACHE_FINISHED_FT,
  ICACHE_FINISHED_FT_EXPECTING_NEXT,
  UOP_CACHE_FINISHED_FT,
  // icache serves immediately after the lookup
  ICACHE_LOOKUP_SERVING,
  ICACHE_NO_LOOKUP_SERVING,
  ICACHE_RETRY_MEM_REQ,
  UOP_CACHE_SERVING,
  WAIT_FOR_MISS,
  WAIT_FOR_EMPTY_ROB,
  WAIT_FOR_RENAME
} Icache_State;

// don't change this order without fixing stats in fetch.stat.def
typedef enum Break_Reason_enum {
  BREAK_DONT,          // don't break fetch yet
  BREAK_RENAME,        // break because of no free renaming physical register
  BREAK_FT_UNAVAILABLE, // break because the ft queue of the decoupled front-end is empty
  BREAK_ICACHE_TO_UOP_CACHE_SWITCH,  // break because in the same cycle switched to fetching from uop cache
  BREAK_ICACHE_MISS_REQ_SUCCESS,     // break because of an icache miss where the mem req succeeds
  BREAK_ICACHE_MISS_REQ_FAILURE,     // break because of an icache miss where the mem req fails
  BREAK_WAIT_FOR_MISS,
  BREAK_UOP_CACHE_READ_LIMIT,        // break because the uop cache has limited read capability
  BREAK_UOP_CACHE_READ_LIMIT_AND_ISSUE_WIDTH,       // break because the uop cache has limited read capability and the issue width has been reached
  BREAK_ICACHE_READ_LIMIT,           // break because the uop cache has limited read capability
  BREAK_ICACHE_READ_LIMIT_AND_ISSUE_WIDTH,          // break because the icache has limited read capability and the issue width has been reached
  BREAK_ISSUE_WIDTH,   // break because it's reached maximum issue width
  BREAK_BARRIER,       // break because of a system call or a fetch barrier instruction
  BREAK_WAIT_FOR_EMPTY_ROB,
  BREAK_APP_EXIT,      // break because the app exit has been reached
  BREAK_STALL          // break because the pipeline is stalled
} Break_Reason;

typedef struct Icache_Stage_struct {
  uns8       proc_id;
  /* two data paths: */
  /* uops fetched from uop cache go to uopc_sd, otherwise sd */
  Stage_Data sd; /* stage interface data */
  Stage_Data uopc_sd;

  Icache_State state; /* state that the ICACHE is in */
  Icache_State
    next_state; /* state that the ICACHE is going to be in next cycle */
  Icache_State after_waiting_state; /* state to transit to after waiting */
  uint64_t wait_for_miss_start; /* time when cache miss was observed */

  Inst_Info** line;   /* pointer to current line on a hit */
  Addr        line_addr;       /* address of the last cache line hit */
  Addr        fetch_addr;      /* address to fetch or fetching */
  FT_Info     current_ft_info; /* containing start, length, and termination reason of the current ft */
  Flag        off_path;        /* is the icache fetching on the correct path? */
  Flag back_on_path; /* did a recovery happen to put the machine back on path?
                      */

  Counter rdy_cycle; /* cycle that the henry icache will return data (only used
                        in henry model) */

  Cache icache;           /* the cache storage structure (caches Inst_Info *) */
  Cache icache_line_info; /* contains info about the icache lines */
  Cache
       pref_icache; /* Prefetcher cache storage structure (caches Inst_Info *) */
  char rand_wb_state[31]; /* State of random number generator for random
                             writeback */
} Icache_Stage;

typedef struct Icache_Data_struct {
  Flag fetched_by_offpath; /* fetched by an off_path op? */
  Addr offpath_op_addr;    /* PC of the off path op that fetched this line */
  Counter
       offpath_op_unique; /* unique of the off path op that fetched this line */
  uns  read_count[2];
  Flag HW_prefetch;
  uns FDIP_prefetch;
  uint64_t ghist;

  Counter fetch_cycle;
  Counter onpath_use_cycle;
} Icache_Data;


/**************************************************************************************/
/* External Variables */

extern Icache_Stage* ic;

/**************************************************************************************/
/* Prototypes */

/* vanilla hps model */
void set_icache_stage(Icache_Stage*);
void init_icache_stage(uns8, const char*);
Stage_Data* get_current_stage_data(void);
void reset_icache_stage(void);
void reset_all_ops_icache_stage(void);
void recover_icache_stage(void);
void redirect_icache_stage(void);
void debug_icache_stage(void);
void update_icache_stage(void);
Flag icache_fill_line(Mem_Req*);
void wp_process_icache_evicted(Icache_Data* line, Mem_Req* req, Addr* repl_line_addr);
void wp_process_icache_hit(Icache_Data* line, Addr fetch_addr);
void wp_process_icache_fill(Icache_Data* line, Mem_Req* req);
Flag icache_off_path(void);
void move_to_prev_op(void);
Flag instr_fill_line(Mem_Req* req);
Flag instr_fill_line(Mem_Req* req);

// For branch stat collection
Flag in_icache(Addr addr);

/**************************************************************************************/

#endif /* #ifndef __ICACHE_STAGE_H__ */

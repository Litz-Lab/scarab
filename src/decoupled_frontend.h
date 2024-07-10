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
 * Author       : Heiner Litz <hlitz@ucsc.edu>
 * Date         : 03/30/2023
 * Description  :
 ***************************************************************************************/

#ifndef __DECOUPLED_FE_H__
#define __DECOUPLED_FE_H__


#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>

#include "debug/debug_macros.h"
#include "debug/debug_print.h"
#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"

#include "bp/bp.h"
#include "bp/bp.param.h"

#include "frontend/frontend.h"
#include "frontend/pin_trace_fe.h"

#include "stage_data.h"

  typedef struct decoupled_fe_iter decoupled_fe_iter;
  
  struct decoupled_fe_iter {
    // the ft index
    uint64_t ft_pos;
    // the op index
    uint64_t op_pos;
    // the flattened op index, as if the ftq is an 1-d array
    uint64_t flattened_op_pos;
  };

  // Simulator API
  void alloc_mem_decoupled_fe(uns numProcs);
  void init_decoupled_fe(uns proc_id, const char*);
  void set_decoupled_fe(int proc_id);
  void reset_decoupled_fe();
  void debug_decoupled_fe();
  void update_decoupled_fe();
  // Icache/Core API
  void recover_decoupled_fe(int proc_id);
  void decoupled_fe_stall(Op *op);
  void decoupled_fe_retire(Op *op, int proc_id, uns64 inst_uid);
  bool decoupled_fe_current_ft_can_fetch_op(int proc_id);
  bool decoupled_fe_fill_icache_stage_data(int proc_id, int requested, Stage_Data *sd);
  bool decoupled_fe_can_fetch_ft(int proc_id);
  FT_Info decoupled_fe_fetch_ft(int proc_id);
  FT_Info decoupled_fe_peek_ft(int proc_id);
  // FTQ API
  decoupled_fe_iter* decoupled_fe_new_ftq_iter();
  /* Returns the Op at current iterator position or NULL if FTQ is empty or the end of FTQ was reached
     if end_of_ft is true the Op is the last one in a fetch target (cache-line boundary of taken branch)*/
  Op* decoupled_fe_ftq_iter_get(decoupled_fe_iter* iter, bool *end_of_ft);
  /* Increments iterator and returns the Op at iterator position or NULL if FTQ is empty or the end of FTQ was reached
     if end_of_ft is true the Op is the last one in a fetch target (cache-line boundary of taken branch)*/
  Op* decoupled_fe_ftq_iter_get_next(decoupled_fe_iter* iter, bool *end_of_ft);
  /* Returns iter flattened offset from the start of the FTQ, this offset gets incremented
     by advancing the iter and decremented by the icache consuming FTQ entries,
     and reset by flushes */
  uint64_t decoupled_fe_ftq_iter_offset(decoupled_fe_iter* iter);
  /* Returns iter ft offset from the start of the FTQ, this offset gets incremented
     by advancing the iter and decremented by the icache consuming FTQ entries,
     and reset by flushes */
  uint64_t decoupled_fe_ftq_iter_ft_offset(decoupled_fe_iter* iter);
  uint64_t decoupled_fe_ftq_num_ops();
  uint64_t decoupled_fe_ftq_num_fts();
  void decoupled_fe_set_ftq_num(int proc_id, uint64_t ftq_ft_num);
  uint64_t decoupled_fe_get_ftq_num(int proc_id);
#ifdef __cplusplus
}
#endif


#endif

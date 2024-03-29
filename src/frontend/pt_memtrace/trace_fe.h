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
 * File         : frontend/trace_fe.h
 * Author       : Heiner Litz
 * Date         :
 * Description  :
 ***************************************************************************************/

#ifndef __TRACE_FE_H__
#define __TRACE_FE_H__

#include "globals/global_types.h"

/**************************************************************************************/
/* Forward Declarations */

struct Trace_Uop_struct;
typedef struct Trace_Uop_struct Trace_Uop;
struct Op_struct;

/**************************************************************************************/
/* Prototypes */

#include "ctype_pin_inst.h"
//#include "pin/pin_lib/uop_generator.h"
//#include "pin/pin_lib/x86_decoder.h"


#ifdef __cplusplus
extern "C" {
#endif

void off_path_generate_inst(uns proc_id, uint64_t *off_path_addr, ctype_pin_inst *inst);

/* Implementing the frontend interface */
Addr ext_trace_next_fetch_addr(uns proc_id);
Flag ext_trace_can_fetch_op(uns proc_id);
void ext_trace_fetch_op(uns proc_id, Op* op);
void ext_trace_redirect(uns proc_id, uns64 inst_uid, Addr fetch_addr);
void ext_trace_recover(uns proc_id, uns64 inst_uid);
void ext_trace_retire(uns proc_id, uns64 inst_uid);
void ext_trace_init();
void ext_trace_done(void);
void ext_trace_extract_basic_block_vectors();
#ifdef __cplusplus
}
#endif

#endif

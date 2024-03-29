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
 * File         : frontend/memtrace_fe.h
 * Author       : Heiner Litz
 * Date         :
 * Description  :
 ***************************************************************************************/

#ifndef __MEMTRACE_FE_H__
#define __MEMTRACE_FE_H__

#include "globals/global_types.h"

/**************************************************************************************/
/* Forward Declarations */

struct Trace_Uop_struct;
typedef struct Trace_Uop_struct Trace_Uop;
struct Op_struct;

/**************************************************************************************/
/* Prototypes */

#ifdef __cplusplus
extern "C" {
#endif
  
void memtrace_init(void);
int  memtrace_trace_read(int proc_id, ctype_pin_inst* pt_next_pi);
void memtrace_setup(uns proc_id);
bool buf_map_find(uns64 line_addr);

#ifdef __cplusplus
}
#endif

#endif

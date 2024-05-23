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
 * File         : pref_markov.h
 * Author       : HPS Research Group
 * Date         : 05/01/2008
 * Description  :
 ***************************************************************************************/
#ifndef __PREF_MARKOV_H__
#define __PREF_MARKOV_H__

#include "pref_common.h"

typedef struct Markov_Table_Entry_Struct {
  Flag    valid;
  Addr    next_addr;
  Addr    tag;
  Counter count;
} Markov_Table_Entry;

typedef struct Pref_Markov_Struct {
  HWP_Info*            hwp_info;
  Markov_Table_Entry** markov_table;
  CacheLevel        type;
} Pref_Markov;

typedef struct{
  Pref_Markov* markov_hwp_core_ul1;
  Pref_Markov* markov_hwp_core_umlc;
  Addr*        last_miss_addr_core_ul1;
  Addr*        last_miss_addr_core_umlc;
} markov_prefetchers;
/*************************************************************/
/* HWP Interface */
void pref_markov_init(HWP* hwp);
void pref_markov_ul1_miss(uns8 proc_id, Addr lineAddr, Addr loadPC,
                          uns32 global_hist);
void pref_markov_ul1_prefhit(uns8 proc_id, Addr lineAddr, Addr loadPC,
                             uns32 global_hist);
void pref_markov_umlc_miss(uns8 proc_id, Addr lineAddr, Addr loadPC,
                          uns32 global_hist);
void pref_markov_umlc_prefhit(uns8 proc_id, Addr lineAddr, Addr loadPC,
                             uns32 global_hist);
/*************************************************************/
/* Internal function */
void init_markov(HWP* hwp, Pref_Markov* markov_hwp_core, Addr* last_miss_addr_core);
void pref_markov_update_table(Pref_Markov* markov_hwp, Addr* last_miss_addr_core, uns8 proc_id, Addr current_addr, Flag true_miss);
void pref_markov_send_prefetches(Pref_Markov* markov_hwp, uns8 proc_id, Addr miss_lineAddr);
/*************************************************************/

#endif /*  __PREF_MARKOV_H__*/

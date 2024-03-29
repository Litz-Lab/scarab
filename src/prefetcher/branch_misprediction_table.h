/***************************************************************************************
 * File         : branch_misprediction_table.h
 * Author       : Peter Braun
 * Date         : 10.28.2020
 * Description  : Store branch count and branch misprediction count for each branch.
 *                Used to identify candidates for dual path prefetching into the uop cache.
 *                8-way set associative.
 ***************************************************************************************/

#ifndef __BRANCH_MISPREDICTION_TABLE_H__
#define __BRANCH_MISPREDICTION_TABLE_H__

#include "op.h"

/**************************************************************************************/
/* Prototypes */

void init_branch_misprediction_table(uns8 proc_id);

float get_branch_misprediction_rate(Addr pc);

void increment_branch_count(Addr pc);
void increment_branch_mispredictions(Addr pc);


#endif /* #ifndef __BRANCH_MISPREDICTION_TABLE_H__ */

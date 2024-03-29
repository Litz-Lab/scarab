/***************************************************************************************
 * File         : branch_misprediction_table.cc
 * Author       : Peter Braun
 * Date         : 10.28.2020
 * Description  : Store branch count and branch misprediction count for each branch.
 *                Used to identify candidates for dual path prefetching into the uop cache.
 ***************************************************************************************/

#include "debug/debug_macros.h"
#include "debug/debug_print.h"
#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"
#include "globals/utils.h"
#include "statistics.h"

#include "libs/hash_lib.h"
#include "libs/cache_lib.h"
#include "prefetcher/pref.param.h"

#include "prefetcher/branch_misprediction_table.h"

typedef struct Bm_Info_struct {
    Counter branch_count;
    Counter branch_mispred_count;
} Bm_Info;

/**************************************************************************************/
/* Global Variables */

uns8 proc_id;

Hash_Table inf_size_bm_table;

/**************************************************************************************/
/* init_branch_misprediction_table */

void init_branch_misprediction_table(uns8 pid) {
  proc_id = pid;
  if (BRANCH_MISPREDICTION_TABLE_SIZE == 0) {
    init_hash_table(&inf_size_bm_table, "infinite sized", 15000000, sizeof(Bm_Info));
    // cpp version is not a lib yet. only one instance.
  }
}

float get_branch_misprediction_rate(Addr pc) {
    float rate = 0;
    if (BRANCH_MISPREDICTION_TABLE_SIZE == 0) {
        Bm_Info* info = (Bm_Info*)hash_table_access(&inf_size_bm_table, pc);
        if (info) {
            rate = info->branch_mispred_count / info->branch_count;
        }
    }
    return rate;
}

void increment_branch_count(Addr pc) {
    if (BRANCH_MISPREDICTION_TABLE_SIZE == 0) {
        Flag new_entry;
        Bm_Info* info = (Bm_Info*)hash_table_access_create(&inf_size_bm_table, pc, 
                        &new_entry);
        if (new_entry) memset(info, 0, sizeof(*info));
        info->branch_count++;
    }
}

void increment_branch_mispredictions(Addr pc) {
    if (BRANCH_MISPREDICTION_TABLE_SIZE == 0) {
        Flag new_entry;
        Bm_Info* info = (Bm_Info*)hash_table_access_create(&inf_size_bm_table, pc, 
                        &new_entry);
        if (new_entry) memset(info, 0, sizeof(*info));
        info->branch_mispred_count++;
    }
}
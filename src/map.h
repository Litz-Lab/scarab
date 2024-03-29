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
 * File         : map.h
 * Author       : HPS Research Group
 * Date         : 2/16/1999
 * Description  :
 ***************************************************************************************/

#ifndef __MAP_H__
#define __MAP_H__

#include "isa/isa_macros.h"
#include "libs/hash_lib.h"
#include "libs/list_lib.h"
#include "op.h"

/**************************************************************************************/
/* Merged Register File: The Hardware Implementation of Register Renaming */

// To be changed to configurable val
#define REG_RENAMING_TABLE_ENABLE           FALSE
#define REG_RENAMING_TABLE_REG_FILE_SIZE    1024

const static int REG_FILE_INVALID_REG_ID = -1;

// register state for releasing
typedef enum Reg_File_Entry_State_enum {
  REG_FILE_ENTRY_STATE_FREE,
  REG_FILE_ENTRY_STATE_ALLOC,
  REG_FILE_ENTRY_STATE_PRODUCED,
  REG_FILE_ENTRY_STATE_COMMIT,
  REG_FILE_ENTRY_STATE_DEAD,
  REG_FILE_ENTRY_STATE_NUM
} Reg_File_Entry_State;

typedef struct Reg_File_Entry_struct {
  // op info (the pointer of op + the deep copy of special val)
  Op       *op;
  Counter  op_num;
  Counter  unique_num;
  Flag     off_path;

  // register info
  int                  reg_arch_id;
  int                  reg_ptag;
  Reg_File_Entry_State reg_state;

  // tracking free physical register
  struct Reg_File_Entry_struct *next_free;

  // tracking the ops use the same architectural register
  int prev_same_arch_id;
} Reg_File_Entry;

typedef struct Merged_Reg_File_struct {
  /* map each architectural register to the latest physical register */
  int             reg_map_table[NUM_REG_IDS];

  /* map ptags to physical registers (register entries) for both speculative and committed op */
  Reg_File_Entry* reg_file;
  uns             reg_file_size;

  /* track all free physical registers */
  Reg_File_Entry* reg_free_list_head;
  uns             reg_free_num;
} Merged_Reg_File;

typedef struct Reg_Renaming_Table_struct {
  Merged_Reg_File *merged_rf;
} Reg_Renaming_Table;

/**************************************************************************************/
/* Types */

typedef struct Map_Entry_struct {
  Op*     op;         /* last op to write (invalid when committed) */
  Counter op_num;     /* op number of the last op to write (not cleared, only
                         overwritten) */
  Counter unique_num; /* unique number of the last op to write (not cleared,
                         only overwritten) */
} Map_Entry;

typedef struct Map_Data_struct {
  /* store information about the last op to write each register */
  uns8      proc_id;
  Map_Entry reg_map[NUM_REG_IDS * 2];
  Flag      map_flags[NUM_REG_IDS];

  Map_Entry last_store[2];
  Flag      last_store_flag;

  Hash_Table oracle_mem_hash;

  Wake_Up_Entry* free_list_head;
  uns            wake_up_entries;
  uns            active_wake_up_entries;

  /* register renaming implementation based on the hardware scheme */
  Reg_Renaming_Table *rename_table;
} Map_Data;


/**************************************************************************************/
/* External Variables */

extern Map_Data* map_data;


/**************************************************************************************/
/* Prototypes */

Map_Data* set_map_data(Map_Data*);
void      init_map(uns8);
void      recover_map(void);
void      rebuild_offpath_map(void);
void      reset_map(void);
void      map_op(Op*);
void      map_mem_dep(Op*);
void      wake_up_ops(Op*, Dep_Type, void (*)(Op*, Op*, uns8));
void      free_wake_up_list(Op*);
void      add_to_wake_up_lists(Op*, Op_Info*, void (*)(Op*, Op*, uns8));

void add_src_from_op(Op*, Op*, Dep_Type);
void add_src_from_map_entry(Op*, Map_Entry*, Dep_Type);

void simple_wake(Op*, Op*, uns8);
void delete_store_hash_entry(Op*);

void clear_not_rdy_bit(Op*, uns);
Flag test_not_rdy_bit(Op*, uns);
void set_not_rdy_bit(Op*, uns);

/* register renaming table */
void rename_table_init(void);
void rename_table_process(Op*);
void rename_table_produce(Op*);

Flag rename_table_available(void);
void rename_table_commit(Op*);
void rename_table_recover(Counter);

/**************************************************************************************/

#endif /* #ifndef __MAP_H__ */

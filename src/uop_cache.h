/***************************************************************************************
 * File         : uop_cache.h
 * Author       : Peter Braun
 * Date         : 10.28.2020
 * Description  : Interface for interacting with uop cache object.
 *                  Following Kotra et. al.'s MICRO 2020 description of uop cache baseline
 *                  Instr comes from icache. Theoretically
 *                  we have higher BW fetching direct with uop cache
 ***************************************************************************************/

#ifndef __UOP_CACHE_H__
#define __UOP_CACHE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "op.h"

// Uop Cache Data
typedef struct Uop_Cache_Data_struct {
  Addr line_start;
  // number of uops in this line
  uns n_uops;
  // the offset for calculating the next line
  Addr offset;
  FT_Info_Dynamic ft_info_dynamic;
  // is this line the end of the FT?
  Flag end_of_ft;

  Counter used;
  Flag priority;
} Uop_Cache_Data;

/**************************************************************************************/
/* Prototypes */

void alloc_mem_uop_cache(uns num_cores);
void init_uop_cache(uns8 proc_id);
void set_uop_cache(uns8 proc_id);
void recover_uop_cache(void);

Flag uop_cache_lookup_ft_and_fill_lookup_buffer(FT_Info ft_info, Flag offpath);
Uop_Cache_Data* uop_cache_get_line_from_lookup_buffer(void);
void uop_cache_clear_lookup_buffer(void);
Uop_Cache_Data* uop_cache_lookup_line(Addr line_start, FT_Info ft_info, Flag update_repl);

void clear_accumulation(Flag clear_line_only);
void end_line_accumulate(Flag last_line_of_ft);
/* accumulate uop into buffer. If terminating condition reached, call insert_uop_cache */
void accumulate_op(Op* op);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __UOP_CACHE_H__ */

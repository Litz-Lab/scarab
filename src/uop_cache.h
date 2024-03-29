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

#include "op.h"

/**************************************************************************************/
/* Macros */
#define UOP_CACHE_LATENCY 1  // Simulating a uop cache latency > 1 is unsupported

/**************************************************************************************/
/* Prototypes */

// only one instance of uop cache

void init_uop_cache(uns8 proc_id);
void recover_uop_cache(void);

/* return whether the instr pc is cached (this does not consider that the whole PW 
    could already have been fetched, potentially introducing 1 incorrect cycle of latency)*/
Flag in_uop_cache(Addr pc, Flag update_repl, Flag offpath); 

void end_accumulate(void);
/* accumulate uop into buffer. If terminating condition reached, call insert_uop_cache */
void accumulate_op(Op* op);

Flag pw_insert(Uop_Cache_Data pw);
void set_uop_cache_insert_enable(Flag new_val);
Uop_Cache_Data get_pw_lookahead_buffer(Addr addr);

#endif /* #ifndef __UOP_CACHE_H__ */

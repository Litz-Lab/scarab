// CPP implementation of a cache.
// Built-in support for multi-line units, e.g. PWs that take multiple lines.

#ifndef __CPP_CACHE_H__
#define __CPP_CACHE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "globals/global_types.h"
#include "cache_lib.h"

void cpp_cache_create(char* name, uns num_lines, uns assoc, uns line_bytes,
                      Repl_Policy repl_policy, Flag tag_incl_offset, int data_size);
uns cpp_cache_index(char* name, Addr addr, Addr* tag, Addr* line_addr);
void* cpp_cache_insert(char* name, Addr addr, int lines_used, Flag priority, Addr* evicted_entries);
void* cpp_cache_access(char* name, Addr addr, Flag update_repl, Flag upgrade_priority);



#ifdef __cplusplus
}
#endif

#endif
#ifndef __HASH_MAP_WRAPPER__
#define __HASH_MAP_WRAPPER__


#ifdef __cplusplus
extern "C" {
#endif

#include "inst_info.h"

  //class cpp_hash_lib_wrapper {

  Inst_Info * cpp_hash_table_access_create(int core, uint64_t addr, uint64_t lsb_bytes, uint64_t msb_bytes, uint8_t op_idx, unsigned char *new_entry);

#ifdef __cplusplus
}
#endif

#endif

#ifndef __HASH_MAP_WRAPPER__
#define __HASH_MAP_WRAPPER__

#ifdef __cplusplus
extern "C" {
#endif

#include "inst_info.h"
#include "static_inst_info.h"

// class cpp_hash_lib_wrapper {

Inst_Info *cpp_hash_table_access_create(int core, uint64_t addr, uint64_t lsb_bytes, uint64_t msb_bytes, uint8_t op_idx,
                                        uint16_t mem_shape, unsigned char *new_entry);

// Per-macro-instruction static info, interned by {addr, binary, mem_shape} (shared by all uops).
Static_Inst_Info *cpp_static_inst_access_create(int core, uint64_t addr, uint64_t lsb_bytes, uint64_t msb_bytes,
                                                uint16_t mem_shape, unsigned char *new_entry);

// Per-uop static info, interned by {addr, binary, op_idx, mem_shape}.
Static_Op_Info *cpp_static_op_access_create(int core, uint64_t addr, uint64_t lsb_bytes, uint64_t msb_bytes,
                                            uint8_t op_idx, uint16_t mem_shape, unsigned char *new_entry);

#ifdef __cplusplus
}
#endif

#endif

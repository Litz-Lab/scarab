#ifndef __FNLMMA_NEW_H__
#define __FNLMMA_NEW_H__


#ifdef __cplusplus
extern "C" {
#endif

  #include "icache_stage.h"

  // Interface
  void alloc_mem_fnlmma(uns numCores);
  void init_fnlmma(uns proc_id);
  void fnlmma_prefetch(uns proc_id, uint64_t v_addr, uint8_t cache_hit, uint8_t prefetch_hit);
  void set_fnlmma(uns proc_id);
  void update_fnlmma(uns proc_id, Addr fetch_addr, uint8_t branch_type, uint64_t branch_target);
  void fnlmma_prefetch(uns proc_id, uint64_t v_addr, uint8_t cache_hit, uint8_t prefetch_hit);
  void print_fnlmma_stats(uns proc_id);

#ifdef __cplusplus
}
#endif

#endif

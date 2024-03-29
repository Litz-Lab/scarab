#ifndef __EIP_NEW_H__
#define __EIP_NEW_H__


#ifdef __cplusplus
extern "C" {
#endif

  #include "icache_stage.h"

  // Interface
  void alloc_mem_eip(uns numCores);
  void init_eip(uns proc_id);
  void eip_prefetch(uns proc_id, uint64_t v_addr, uint8_t cache_hit, uint8_t prefetch_hit, Flag off_path);
  void set_eip(uns proc_id);
  void update_eip();
  void eip_cache_fill(uns proc_id, uint64_t v_addr, uint64_t evicted_v_addr);
  void print_eip_stats(uns proc_id);

#ifdef __cplusplus
}
#endif

#endif

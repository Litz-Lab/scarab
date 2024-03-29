#ifndef __DJOLT_NEW_H__
#define __DJOLT_NEW_H__


#ifdef __cplusplus
extern "C" {
#endif

  #include "icache_stage.h"

  // Interface
  void alloc_mem_djolt(uns numCores);
  void init_djolt(uns proc_id);
  void djolt_prefetch(uns proc_id, uint64_t v_addr, uint8_t cache_hit, uint8_t prefetch_hit);
  void set_djolt(uns proc_id);
  void update_djolt(uns proc_id, Addr fetch_addr, uint8_t branch_type, uint64_t branch_target);
  void djolt_prefetch(uns proc_id, uint64_t v_addr, uint8_t cache_hit, uint8_t prefetch_hit);
  void djolt_cycle_operate(uns proc_id);
  void print_djolt_stats(uns proc_id);

#ifdef __cplusplus
}
#endif

#endif

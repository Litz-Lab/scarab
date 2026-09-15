#ifndef __CACHE_INDEX_H__
#define __CACHE_INDEX_H__

#include "globals/global_defs.h"
#include "globals/global_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Cache_struct Cache;
typedef struct Csv_Index_State_struct Csv_Index_State;

/**************************************************************************************/
/* EntropyIndex (MICRO 2024) types */

#define ENTROPY_INDEX_MAX_TRACK_BITS 64

typedef enum Entropy_Index_Stat_enum {
  EI_STAT_SWITCH = 0,     /* index function switched */
  EI_STAT_SKIP,           /* reselection declined to switch */
  EI_STAT_SET_REMAPPED,   /* one set processed by the set pointer */
  EI_STAT_REMAP_EVICTION, /* line evicted due to a remap conflict */
  NUM_EI_STAT
} Entropy_Index_Stat;

typedef struct Entropy_Index_State_struct {
  uns num_track_bits; /* T: number of low block-address bits tracked */
  uns num_index_bits; /* N: log2(num_sets); width of the set index */

  uns flip_counts[ENTROPY_INDEX_MAX_TRACK_BITS]; /* EC[i]; # flips at bit i in current interval */
  Addr last_miss_blk_addr;                       /* address of the previous miss */
  Flag have_last_miss;                           /* whether last_miss_addr is valid */

  uns8 x_bits[ENTROPY_INDEX_MAX_TRACK_BITS];
  uns8 y_bits[ENTROPY_INDEX_MAX_TRACK_BITS];

  Flag remapping;
  uns8 x_bits_next[ENTROPY_INDEX_MAX_TRACK_BITS];
  uns8 y_bits_next[ENTROPY_INDEX_MAX_TRACK_BITS];

  uns set_ptr;          /* SP; sets [0, set_ptr) are already using the new function */
  uns remap_rate;       /* R; remap one set every R accesses */
  uns remap_access_ctr; /* accesses since the last set was remapped */

  uns interval_size;     /* M: accesses per reselection */
  uns interval_ctr;      /* accesses since the last reselection */
  uns switch_thresh_pct; /* require >= this % entropy gain to switch functions */

  /* For debugging. */
  int stat_base;               /* -1 = no stats */
  Counter num_switches;        /* index-function switches committed */
  Counter num_skipped;         /* reselections that did not switch */
  Counter num_sets_remapped;   /* sets processed by the set pointer */
  Counter num_evictions_remap; /* lines evicted due to remapping conflicts */
} Entropy_Index_State;

/**************************************************************************************/
/* Index-hash selection */

typedef enum Index_Hash_enum {
  ID_HASH,        /* Identity function */
  KNUTH_HASH,     /* Knuth multiplicative hash */
  SINGLE_XOR,     /* XOR only once */
  XOR_FOLDING,    /* XOR 64/set_bits times */
  PRIME_DISPLACE, /* Displace index by p * tag */
  SHA256_HASH,    /* SHA-256 of the block address, digest folded to the index */
  ENTROPY_INDEX,  /* MICRO 2024 EntropyIndex: dynamic entropy-based bit selection */
  CSV_MAP,        /* Fixed address->set map loaded from a CSV; misses go to the last set */
  NUM_INDEX_HASH
} Index_Hash_Id;

typedef struct Index_Hash_struct {
  Index_Hash_Id id;
  const char* name;
  Addr (*hash_func)(Cache*, Addr);
} Index_Hash;

/* Index_Hash_Id -> {name, hash_func} dispatch table */
extern Index_Hash index_hash_table[];

/**************************************************************************************/
/* Per-cache index state */

typedef struct Cache_Index_State_struct {
  Index_Hash* index_hash;             /* selected set-index hash function */
  Entropy_Index_State* entropy_state; /* non-NULL only when the index hash is ENTROPY_INDEX */
  Csv_Index_State* csv_state;         /* non-NULL only when the index hash is CSV_MAP */
} Cache_Index_State;

/**************************************************************************************/
/* Index-hash configuration passed to init (see init_cache_impl) */

typedef struct Cache_Index_Config_struct {
  Index_Hash_Id index_hash_id;

  /* EntropyIndex knobs; referenced only when index_hash_id == ENTROPY_INDEX */
  uns ei_num_track_bits;
  uns ei_interval_size;
  uns ei_remap_rate;
  int ei_switch_thresh_pct;
  int ei_stat_base;

  /* Path to the address->set CSV; referenced only when index_hash_id == CSV_MAP */
  const char* csv_map_path;
} Cache_Index_Config;

static inline Cache_Index_Config get_default_cache_index_config(void) {
  Cache_Index_Config idx_cfg = {
      .index_hash_id = ID_HASH,

      .ei_num_track_bits = ENTROPY_INDEX_MAX_TRACK_BITS,
      .ei_interval_size = 1000000,
      .ei_remap_rate = 80,
      .ei_switch_thresh_pct = -1,
      .ei_stat_base = -1,

      .csv_map_path = NULL,
  };
  return idx_cfg;
}

/**************************************************************************************/
/* Index API. cache_lib calls only these and never inspects the configured hash. */

/* Set up cache->cache_index_state from cfg. Call once at init, after the cache
 * geometry (set_bits, shift_bits, set_mask, num_sets, assoc) has been set. */
void cache_index_state_init(Cache* cache, Cache_Index_Config cfg);

/* Map an access address to its set index (shift + configured hash + set mask). */
uns cache_index_get_set(Cache* cache, Addr addr);

/* Notify the index of a demand access (hit = whether it hit) so an adaptive
 * index can train / remap. No-op for static index hashes. */
void cache_index_on_access(Cache* cache, Addr addr, Flag hit);

/* Reset online index-training state (e.g. on cache flush), keeping the current
 * index function. No-op for static index hashes. */
void cache_index_state_reset(Cache* cache);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __CACHE_INDEX_H__ */

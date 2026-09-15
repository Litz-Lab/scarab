#include "libs/cache_index.h"

#include <fstream>
#include <sstream>
#include <unordered_map>

#include "globals/assert.h"
#include "globals/global_defs.h"
#include "globals/utils.h"

#include "libs/cache_lib.h"
#include "libs/sha256.h"

#include "statistics.h"

/* Emit an EntropyIndex debug stat, offset from the state's stat_base (< 0 disables). */
#define ENTROPY_INDEX_STAT(state, stat_enum)           \
  do {                                                 \
    if ((state)->stat_base >= 0)                       \
      STAT_EVENT(0, (state)->stat_base + (stat_enum)); \
  } while (0)

/**************************************************************************************/
/* EntropyIndex (MICRO 2024). The set index is X ^ Y, where X is the N highest-entropy block-address bits and
 * Y is the next N (N = log2(num_sets)). "Entropy" is estimated online by a per-bit flip counter incremented when a bit
 * toggles between two consecutive misses. At each interval boundary the bits are re-ranked and, if the entropy gain
 * clears a threshold, the index function is switched and the cache is gradually remapped CEASER-style. */

static void entropy_index_init(Cache* cache, uns num_track_bits, uns interval_size, uns remap_rate,
                               int switch_thresh_pct, int stat_base) {
  /* NOTE: EntropyIndex selects bits of the line-addressable block address, so it
     should only be enabled for caches that are NOT byte-addressable (i.e. not
     tag_incl_offset). tag_incl_offset set after this point in init, so the
     caller is responsible for not pairing ENTROPY_INDEX with a uop-style cache. */

  Entropy_Index_State* s = (Entropy_Index_State*)calloc(1, sizeof(Entropy_Index_State));

  s->num_track_bits = MIN2(num_track_bits, ENTROPY_INDEX_MAX_TRACK_BITS - cache->shift_bits);
  s->num_index_bits = cache->set_bits;
  // The dual-sequence index needs 2N distinct tracked bits.
  ASSERT(0, s->num_track_bits >= 2 * s->num_index_bits);

  s->have_last_miss = FALSE;

  // Initial function: X = low N block-address bits, Y = next N bits.
  for (uns ii = 0; ii < s->num_index_bits; ii++) {
    s->x_bits[ii] = ii;
    s->y_bits[ii] = s->num_index_bits + ii;
  }

  ASSERT(0, cache->num_sets * remap_rate <= interval_size);  // Otherwise remapping will never complete
  s->remapping = FALSE;
  s->set_ptr = 0;
  s->remap_rate = remap_rate;
  s->remap_access_ctr = 0;
  s->interval_size = interval_size;
  s->interval_ctr = 0;
  if (switch_thresh_pct >= 0)
    s->switch_thresh_pct = switch_thresh_pct;
  else
    s->switch_thresh_pct = cache->assoc * 100 / remap_rate;

  s->stat_base = stat_base;

  cache->cache_index_state.entropy_state = s;
}

static Addr _entropy_index_apply(Entropy_Index_State* s, Addr blk_addr, const uns8* x_bits, const uns8* y_bits);
static void entropy_index_remap_one_set(Cache* cache);
static void entropy_index_reselect(Cache* cache);

/* At the end of each interval (M cache accesses), change index function if it will improve the hit rate.
 * If that's the case, in the next interval, gradually remap sets every R cache accesses. */
static void entropy_index_on_access(Cache* cache, Addr blk_addr, Flag hit) {
  Entropy_Index_State* s = cache->cache_index_state.entropy_state;

  // (1) Update per-bit flip ("entropy") counters on misses.
  if (!hit) {
    if (s->have_last_miss) {
      Addr diff = blk_addr ^ s->last_miss_blk_addr;
      for (uns ii = 0; ii < s->num_track_bits; ii++)
        s->flip_counts[ii] += (diff >> ii) & 0x1ULL;
    }
    s->last_miss_blk_addr = blk_addr;
    s->have_last_miss = TRUE;
  }

  // (2) Advance gradual (CEASER-style) remapping: one set per remap_rate accesses.
  if (s->remapping) {
    if (++s->remap_access_ctr >= s->remap_rate) {
      s->remap_access_ctr = 0;
      entropy_index_remap_one_set(cache);
    }
  }

  // (3) At each interval boundary, re-rank bits and possibly switch functions.
  if (++s->interval_ctr >= s->interval_size) {
    s->interval_ctr = 0;
    entropy_index_reselect(cache);
  }
}

static void entropy_index_reselect(Cache* cache) {
  Entropy_Index_State* s = cache->cache_index_state.entropy_state;

  /* Rank tracked bits by flip count, descending (ties broken by lower bit index
     for determinism). Selection sort over the small (<= 64) tracked-bit set. */
  uns8 order[ENTROPY_INDEX_MAX_TRACK_BITS];  // array of array index
  for (uns ii = 0; ii < s->num_track_bits; ii++)
    order[ii] = ii;
  for (uns ii = 0; ii < s->num_track_bits; ii++) {
    uns best = ii;
    for (uns jj = ii + 1; jj < s->num_track_bits; jj++) {
      if (s->flip_counts[order[jj]] > s->flip_counts[order[best]])
        best = jj;
      else if (s->flip_counts[order[jj]] == s->flip_counts[order[best]] && order[jj] < order[best])
        best = jj;
    }
    if (best != ii) {
      uns8 _tmp = order[ii];
      order[ii] = order[best];
      order[best] = _tmp;
    }
  }

  // Candidate function: X = top N bits, Y = next N bits.
  uns8 cand_x[ENTROPY_INDEX_MAX_TRACK_BITS], cand_y[ENTROPY_INDEX_MAX_TRACK_BITS];
  for (uns ii = 0; ii < s->num_index_bits; ii++) {
    cand_x[ii] = order[ii];
    cand_y[ii] = order[s->num_index_bits + ii];
  }

  // Compare total entropy (sum of flip counts) of the candidate against the current.
  uns64 curr_total_entropy = 0, cand_total_entropy = 0;
  for (uns ii = 0; ii < s->num_index_bits; ii++) {
    curr_total_entropy += s->flip_counts[s->x_bits[ii]] + s->flip_counts[s->y_bits[ii]];
    cand_total_entropy += s->flip_counts[cand_x[ii]] + s->flip_counts[cand_y[ii]];
  }
  // Switch only if the entropy gain clears the remap-cost threshold.
  if ((cand_total_entropy - curr_total_entropy) * 100 > (uns64)s->switch_thresh_pct * curr_total_entropy) {
    s->remapping = TRUE;
    for (uns ii = 0; ii < s->num_index_bits; ii++) {
      s->x_bits_next[ii] = cand_x[ii];
      s->y_bits_next[ii] = cand_y[ii];
    }
    s->set_ptr = 0;
    s->remap_access_ctr = 0;
    ENTROPY_INDEX_STAT(s, EI_STAT_SWITCH);
  } else {
    ENTROPY_INDEX_STAT(s, EI_STAT_SKIP);
  }

  /* Start a fresh entropy-measurement window. */
  for (uns ii = 0; ii < s->num_track_bits; ii++)
    s->flip_counts[ii] = 0;
  s->have_last_miss = FALSE;
}

static void entropy_index_remap_one_set(Cache* cache) {
  Entropy_Index_State* s = cache->cache_index_state.entropy_state;

  // The paper does not specify this, but assume all ways in a set are remapped at once.
  for (uns way = 0; way < cache->assoc; way++) {
    Cache_Entry* src_entry = &cache->entries[s->set_ptr][way];
    if (!src_entry->valid)
      continue;

    Addr blk_addr = src_entry->base >> cache->shift_bits;
    Addr dst = _entropy_index_apply(s, blk_addr, s->x_bits_next, s->y_bits_next) & cache->set_mask;
    if (dst == s->set_ptr)
      continue; /* line keeps its set under the new function */

    // Pick a victim way in dst (prefer an invalid way, else approximate LRU).
    // src-clean-dst-dirty case currently evicts dst.
    uns victim = 0;
    Counter lru_time = MAX_CTR;
    Flag found_invalid = FALSE;
    for (uns v = 0; v < cache->assoc; v++) {
      if (!cache->entries[dst][v].valid) {
        victim = v;
        found_invalid = TRUE;
        break;
      }
      if (cache->entries[dst][v].last_access_time < lru_time) {
        lru_time = cache->entries[dst][v].last_access_time;
        victim = v;
      }
    }
    Cache_Entry* dst_entry = &cache->entries[dst][victim];

    /* Swap the full entries so each slot keeps owning its own data buffer: the
       migrated line (with its data) moves to dst, while the evicted victim lands
       in src (s->set_ptr) and is invalidated. */
    Cache_Entry _tmp = *dst_entry;
    *dst_entry = *src_entry;
    *src_entry = _tmp;
    src_entry->valid = FALSE;

    if (!found_invalid && dst_entry->valid)
      ENTROPY_INDEX_STAT(s, EI_STAT_REMAP_EVICTION);
  }
  ENTROPY_INDEX_STAT(s, EI_STAT_SET_REMAPPED);

  if (++s->set_ptr >= cache->num_sets) {
    // Remapping complete: commit the new function and stop remapping.
    for (uns ii = 0; ii < s->num_index_bits; ii++) {
      s->x_bits[ii] = s->x_bits_next[ii];
      s->y_bits[ii] = s->y_bits_next[ii];
    }
    s->remapping = FALSE;
    s->set_ptr = 0;
  }
}

/**************************************************************************************/
/* CSV address->set map. Reads a CSV with (at least) "addr" and "set_index" columns (order not assumed).
 * `addr` values are full byte addresses and are keyed by their block address (addr >> shift_bits).
 * Addresses absent from the file map to the last set (num_sets - 1) via cache_index_csv() above. */

struct Csv_Index_State_struct {
  std::unordered_map<Addr, uns> addr_to_set;
  uns last_set = 0; /* default set for unlisted addresses */
};

/* Trim leading/trailing ASCII whitespace from s. */
static std::string _csv_trim(const std::string& s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return "";
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

static void csv_index_init(Cache* cache, const char* path) {
  ASSERTUM(0, path && path[0], "CSV_MAP index hash selected but no csv_map_path given for cache '%s'.\n", cache->name);

  std::ifstream file(path);
  ASSERTUM(0, file.is_open(), "Could not open cache index CSV '%s' for cache '%s'.\n", path, cache->name);

  Csv_Index_State* s = new Csv_Index_State();
  s->last_set = cache->num_sets - 1;

  std::string line;
  ASSERTUM(0, (bool)std::getline(file, line), "Cache index CSV '%s' is empty.\n", path);

  /* Locate the addr and set_index columns from the header. */
  int addr_col = -1, set_col = -1;
  {
    std::istringstream header(line);
    std::string cell;
    for (int col = 0; std::getline(header, cell, ','); col++) {
      std::string name = _csv_trim(cell);
      if (name == "addr")
        addr_col = col;
      else if (name == "set_index")
        set_col = col;
    }
  }
  ASSERTUM(0, addr_col >= 0 && set_col >= 0, "Cache index CSV '%s' must have 'addr' and 'set_index' columns.\n", path);

  /* Parse each data row. */
  while (std::getline(file, line)) {
    if (_csv_trim(line).empty())
      continue; /* skip blank lines */
    std::istringstream row(line);
    std::string cell, addr_cell, set_cell;
    for (int col = 0; std::getline(row, cell, ','); col++) {
      if (col == addr_col)
        addr_cell = cell;
      else if (col == set_col)
        set_cell = cell;
    }
    Addr addr = (Addr)std::stoull(_csv_trim(addr_cell), nullptr, 0); /* base 0: 0x-hex or decimal */
    uns set_index = (uns)std::stoul(_csv_trim(set_cell), nullptr, 0);
    ASSERTUM(0, set_index < cache->num_sets, "Cache index CSV '%s': set_index %u out of range (num_sets=%u).\n", path,
             set_index, cache->num_sets);
    s->addr_to_set[addr >> cache->shift_bits] = set_index;
  }

  cache->cache_index_state.csv_state = s;
}

/**************************************************************************************/
/* Set-index hash functions. Referenced only through index_hash_table below, so
 * they are file-local (static). `addr` is the block address (already shifted). */

static Addr cache_index_id_hash(Cache* cache, Addr addr) {
  return addr;
}

static Addr cache_index_knuth_hash(Cache* cache, Addr addr) {
  // Knuth multiplicative hash
  Addr tmp = addr * 11400714819323197440ULL;
  return (tmp << cache->set_bits) | (tmp >> (64 - cache->set_bits));
}

static Addr cache_index_single_xor(Cache* cache, Addr addr) {
  return (addr >> cache->set_bits) ^ addr;
}

static Addr cache_index_xor_folding(Cache* cache, Addr addr) {
  Addr index = addr;
  if (cache->set_bits < 64) {
    while ((index >> cache->set_bits) != 0) {
      index = (index & cache->set_mask) ^ (index >> cache->set_bits);
    }
  }
  return index & cache->set_mask;
}

static Addr cache_index_prime_displace(Cache* cache, Addr addr) {
  return ((addr & cache->set_mask) + (addr >> cache->set_bits) * 17) & cache->set_mask;
}

static Addr cache_index_sha256(Cache* cache, Addr addr) {
  return sha256_64bits(addr);
}

/* Compute X ^ Y for a given (x_bits, y_bits) selection over a block address. */
static Addr _entropy_index_apply(Entropy_Index_State* s, Addr blk_addr, const uns8* x_bits, const uns8* y_bits) {
  Addr x = 0, y = 0;
  for (uns ii = 0; ii < s->num_index_bits; ii++) {
    x |= ((blk_addr >> x_bits[ii]) & 0x1ULL) << ii;
    y |= ((blk_addr >> y_bits[ii]) & 0x1ULL) << ii;
  }
  return x ^ y;
}

static Addr cache_index_entropy(Cache* cache, Addr addr) {
  Entropy_Index_State* s = cache->cache_index_state.entropy_state;
  Addr current_set = _entropy_index_apply(s, addr, s->x_bits, s->y_bits);
  /* During gradual remapping a line is relocated once its old set has been
     processed by the set pointer; relocated lines live at their new set. */
  if (s->remapping && current_set < s->set_ptr)
    return _entropy_index_apply(s, addr, s->x_bits_next, s->y_bits_next);
  return current_set;
}

static Addr cache_index_csv(Cache* cache, Addr addr) {
  Csv_Index_State* s = (Csv_Index_State*)cache->cache_index_state.csv_state;
  auto it = s->addr_to_set.find(addr);
  return (it != s->addr_to_set.end()) ? it->second : s->last_set;
}

/* Index_Hash_Id -> {name, hash_func} dispatch table */
#include "libs/cache_index_table.def"

/**************************************************************************************/
/* Index API */

void cache_index_state_init(Cache* cache, Cache_Index_Config cfg) {
  cache->cache_index_state.index_hash = &index_hash_table[cfg.index_hash_id];
  cache->cache_index_state.entropy_state = NULL;
  cache->cache_index_state.csv_state = NULL;

  if (cfg.index_hash_id == ENTROPY_INDEX)
    entropy_index_init(cache, cfg.ei_num_track_bits, cfg.ei_interval_size, cfg.ei_remap_rate, cfg.ei_switch_thresh_pct,
                       cfg.ei_stat_base);
  else if (cfg.index_hash_id == CSV_MAP)
    csv_index_init(cache, cfg.csv_map_path);
}

uns cache_index_get_set(Cache* cache, Addr addr) {
  return cache->cache_index_state.index_hash->hash_func(cache, addr >> cache->shift_bits) & cache->set_mask;
}

void cache_index_on_access(Cache* cache, Addr addr, Flag hit) {
  if (cache->cache_index_state.entropy_state)
    entropy_index_on_access(cache, addr >> cache->shift_bits, hit);
}

void cache_index_state_reset(Cache* cache) {
  Entropy_Index_State* s = cache->cache_index_state.entropy_state;
  if (!s)
    return;
  // Clear EntropyIndex training/remapping state; keep the current index function.
  s->remapping = FALSE;
  s->set_ptr = 0;
  s->remap_access_ctr = 0;
  s->interval_ctr = 0;
  s->have_last_miss = FALSE;
  for (uns ii = 0; ii < s->num_track_bits; ii++)
    s->flip_counts[ii] = 0;
}

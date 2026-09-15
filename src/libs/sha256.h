/* Self-contained FIPS-180-4 SHA-256.
 * Used by cache_lib as an index-hash function, but the core is general purpose. */

#ifndef __SHA256_H__
#define __SHA256_H__

#include <stddef.h>

#include "globals/global_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Compute the SHA-256 digest of `len` bytes at `data`, writing 32 bytes to `digest`. */
void sha256(const uns8* data, size_t len, uns8 digest[32]);

/* Hash a single 64-bit value and fold the 256-bit digest down to 64 bits.
 * Convenience wrapper used for cache set indexing. */
uns64 sha256_64bits(uns64 value);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __SHA256_H__ */

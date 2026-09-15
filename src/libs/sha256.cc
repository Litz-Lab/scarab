#include "libs/sha256.h"

#include <array>
#include <cstring>

#include "globals/assert.h"
#include "globals/global_types.h"

namespace {

// Round constants: first 32 bits of the fractional parts of the cube roots of the first 64 primes.
constexpr std::array<uns32, 64> K = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

// Initial hash values: first 32 bits of the fractional parts of the square roots of the first 8 primes.
constexpr std::array<uns32, 8> InitialHash = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                              0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

constexpr uns32 rotr(uns32 x, unsigned n) {
  return (x >> n) | (x << (32 - n));
}
constexpr uns32 ch(uns32 x, uns32 y, uns32 z) {
  return (x & y) ^ (~x & z);
}
constexpr uns32 maj(uns32 x, uns32 y, uns32 z) {
  return (x & y) ^ (x & z) ^ (y & z);
}
constexpr uns32 bsig0(uns32 x) {
  return rotr(x, 2) ^ rotr(x, 13) ^ rotr(x, 22);
}
constexpr uns32 bsig1(uns32 x) {
  return rotr(x, 6) ^ rotr(x, 11) ^ rotr(x, 25);
}
constexpr uns32 ssig0(uns32 x) {
  return rotr(x, 7) ^ rotr(x, 18) ^ (x >> 3);
}
constexpr uns32 ssig1(uns32 x) {
  return rotr(x, 17) ^ rotr(x, 19) ^ (x >> 10);
}

// Run the 64-round compression over a fully-populated schedule and accumulate
// the result into the running hash state H.
void compress(std::array<uns32, 8>& H, const std::array<uns32, 64>& w) {
  uns32 a = H[0], b = H[1], c = H[2], d = H[3], e = H[4], f = H[5], g = H[6], h = H[7];

  for (std::size_t t = 0; t < 64; t++) {
    const uns32 t1 = h + bsig1(e) + ch(e, f, g) + K[t] + w[t];
    const uns32 t2 = bsig0(a) + maj(a, b, c);
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }

  H[0] += a;
  H[1] += b;
  H[2] += c;
  H[3] += d;
  H[4] += e;
  H[5] += f;
  H[6] += g;
  H[7] += h;
}

// Process one 512-bit block (64 bytes) into the running hash state H.
void sha256_block(std::array<uns32, 8>& H, const uns8* block) {
  std::array<uns32, 64> w{};
  for (std::size_t t = 0; t < 16; t++)
    w[t] = (static_cast<uns32>(block[t * 4]) << 24) | (static_cast<uns32>(block[t * 4 + 1]) << 16) |
           (static_cast<uns32>(block[t * 4 + 2]) << 8) | static_cast<uns32>(block[t * 4 + 3]);
  for (std::size_t t = 16; t < 64; t++)
    w[t] = ssig1(w[t - 2]) + w[t - 7] + ssig0(w[t - 15]) + w[t - 16];
  compress(H, w);
}

}  // namespace

void sha256(const uns8* data, std::size_t len, uns8 digest[32]) {
  ASSERT(0, data != nullptr || len == 0);
  ASSERT(0, digest != nullptr);

  std::array<uns32, 8> H = InitialHash;

  std::array<uns8, 64> block{};
  const uns64 bit_len = static_cast<uns64>(len) * 8;

  // Process all full 64-byte blocks.
  const std::size_t full_blocks = len / 64;
  for (std::size_t ii = 0; ii < full_blocks; ii++)
    sha256_block(H, data + ii * 64);

  // Build the final padded block(s) from the remaining (< 64) bytes.
  const std::size_t rem = len % 64;
  if (rem > 0)
    std::memcpy(block.data(), data + full_blocks * 64, rem);
  block[rem] = 0x80;  // append the '1' bit

  if (rem >= 56) {
    // Not enough room for the length; pad this block, then emit a length-only block.
    if (rem < 63)
      std::memset(block.data() + rem + 1, 0, 64 - rem - 1);
    sha256_block(H, block.data());
    std::memset(block.data(), 0, 56);
  } else {
    if (rem < 55)
      std::memset(block.data() + rem + 1, 0, 56 - rem - 1);
  }
  // Append the 64-bit big-endian message length.
  for (std::size_t ii = 0; ii < 8; ii++)
    block[56 + ii] = static_cast<uns8>(bit_len >> (56 - ii * 8));

  sha256_block(H, block.data());

  // Serialize the digest big-endian.
  for (std::size_t ii = 0; ii < 8; ii++) {
    digest[ii * 4] = static_cast<uns8>(H[ii] >> 24);
    digest[ii * 4 + 1] = static_cast<uns8>(H[ii] >> 16);
    digest[ii * 4 + 2] = static_cast<uns8>(H[ii] >> 8);
    digest[ii * 4 + 3] = static_cast<uns8>(H[ii]);
  }
}

/* Specialized SHA-256 for a fixed 8-byte (64-bit) message, as used for cache set indexing.
 * Such a message is a single 512-bit block whose 16 words are constant except the two that carry the input:
 * - w[0]/w[1] are the byte-swapped halves of `value`,
 * - w[2] = 0x80000000 (the pad '1' bit),
 * - w[3..14] = 0, and
 * - w[15] = 64 (the bit length).
 * Because w[2..15] are compile-time constants, the schedule terms sourced from them fold away
 * (SSIG of a constant is a constant, zero addends drop out), so we expand the schedule and
 * run the rounds inline here instead of the generic padding/compression path. */
uns64 sha256_64bits(uns64 value) {
  std::array<uns32, 8> H = InitialHash;
  const uns32 lo = static_cast<uns32>(value) & 0xffffffffu;  // input bytes 0..3 (little-endian)
  const uns32 hi = static_cast<uns32>(value >> 32);          // input bytes 4..7 (little-endian)

  // Schedule words are read big-endian, so each little-endian input half is
  // byte-swapped into w[0]/w[1]; w[2..15] are the constant padding block.
  std::array<uns32, 64> w{};
  w[0] =
      ((lo & 0x000000ffu) << 24) | ((lo & 0x0000ff00u) << 8) | ((lo & 0x00ff0000u) >> 8) | ((lo & 0xff000000u) >> 24);
  w[1] =
      ((hi & 0x000000ffu) << 24) | ((hi & 0x0000ff00u) << 8) | ((hi & 0x00ff0000u) >> 8) | ((hi & 0xff000000u) >> 24);
  w[2] = 0x80000000u;  // padding: the appended '1' bit
  w[15] = 64u;         // message length in bits (8 bytes * 8)

  /* Unrolled the message schedule. For t in [16, 31] at least one of the four source
   * words (w[t-2], w[t-7], w[t-15], w[t-16]) is still a constant padding word
   * (w[2] = 0x80000000, w[15] = 64, w[3..14] = 0), so those terms fold to compile-time
   * constants or drop out (+0). The recurrence is unrolled here with those parts folded in;
   * ssig(<non-zero-constant>) is left for the compiler to evaluate.
   * From t = 32 on every source word is input-derived, so the general recurrence runs in a loop. */
  w[16] = ssig0(w[1]) + w[0];                         // + ssig1(w[14]=0) + w[9]=0
  w[17] = ssig1(64u) + ssig0(0x80000000u) + w[1];     // w[15]=64, w[2];  + w[10]=0
  w[18] = ssig1(w[16]) + 0x80000000u;                 // w[2]; + w[11]=0  + ssig0(w[3]=0)
  w[19] = ssig1(w[17]);                               // + w[12]=0 + ssig0(w[4]=0) + w[3]=0
  w[20] = ssig1(w[18]);                               // + w[13]=0 + ssig0(w[5]=0) + w[4]=0
  w[21] = ssig1(w[19]);                               // + w[14]=0 + ssig0(w[6]=0) + w[5]=0
  w[22] = ssig1(w[20]) + 64u;                         // w[15]=64; + ssig0(w[7]=0) + w[6]=0
  w[23] = ssig1(w[21]) + w[16];                       // + ssig0(w[8]=0)  + w[7]=0
  w[24] = ssig1(w[22]) + w[17];                       // + ssig0(w[9]=0)  + w[8]=0
  w[25] = ssig1(w[23]) + w[18];                       // + ssig0(w[10]=0) + w[9]=0
  w[26] = ssig1(w[24]) + w[19];                       // + ssig0(w[11]=0) + w[10]=0
  w[27] = ssig1(w[25]) + w[20];                       // + ssig0(w[12]=0) + w[11]=0
  w[28] = ssig1(w[26]) + w[21];                       // + ssig0(w[13]=0) + w[12]=0
  w[29] = ssig1(w[27]) + w[22];                       // + ssig0(w[14]=0) + w[13]=0
  w[30] = ssig1(w[28]) + w[23] + ssig0(64u);          // w[15]=64; + w[14]=0
  w[31] = ssig1(w[29]) + w[24] + ssig0(w[16]) + 64u;  // w[15]=64 (last constant)
  for (std::size_t t = 32; t < 64; t++)
    w[t] = ssig1(w[t - 2]) + w[t - 7] + ssig0(w[t - 15]) + w[t - 16];

  compress(H, w);

  // XOR-fold the four big-endian 64-bit digest words, taken directly from the state
  return (static_cast<uns64>(H[0] ^ H[2] ^ H[4] ^ H[6]) << 32) | static_cast<uns64>(H[1] ^ H[3] ^ H[5] ^ H[7]);
}

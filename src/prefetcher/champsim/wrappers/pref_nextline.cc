/* Copyright 2020 HPS/SAFARI Research Groups (MIT license, see shim_import.h) */

/* ChampSim's next-line prefetcher (the official sequential/streaming baseline,
 * vendor: next_line.l2c, byte-for-byte). Enable by listing TYPE_NEXTLINE in
 * one of --pref_{dcache,mlc,l1}_prefetchers with its DEST_* token. */

#define SHIM_NAME nextline
#define SHIM_VENDOR "prefetcher/champsim/csenv/next_line.l2c.inc"
#define SHIM_API_L2C 1
#include "prefetcher/champsim/shim_import.h"

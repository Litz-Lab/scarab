/* Copyright 2020 HPS/SAFARI Research Groups */
/***************************************************************************************
 * File         : prefetcher/champsim/pref_champsim.h
 * Description  : Declarations of the Scarab hooks exported by ChampSim-shim
 *                prefetcher wrappers. Included by pref_common.c before pref_table.def.
 ***************************************************************************************/
#ifndef __PREF_CHAMPSIM_H__
#define __PREF_CHAMPSIM_H__

#include "globals/global_types.h"

#include "prefetcher/pref_common.h"

/* --- MLOP (vendor mlop_dpc3.l1d). ONE entry; enable by listing TYPE_MLOP in
 *     exactly one of --pref_{dcache,mlc,l1}_prefetchers with its DEST_* token.
 *     Multi-fill-level with --pref_mlop_route_by_fill 1. DL0 hooks take
 *     (lineAddr, loadPC). --- */
void pref_mlop_init(HWP* hwp);
void pref_mlop_per_core_done(uns proc_id);
void pref_mlop_dl0_miss(Addr lineAddr, Addr loadPC);
void pref_mlop_dl0_hit(Addr lineAddr, Addr loadPC);
void pref_mlop_dl0_pref_hit(Addr lineAddr, Addr loadPC);
void pref_mlop_mlc_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_mlop_mlc_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_mlop_mlc_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_mlop_l1_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_mlop_l1_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_mlop_l1_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);

/* --- IPCP (self-contained vendor ipcp_isca2020.l1d). ONE entry; enable by listing
 *     TYPE_IPCP in exactly one of --pref_{dcache,mlc,l1}_prefetchers with its DEST_*
 *     token (Scarab names: dcache=L1D/DL0, mlc=L2/UMLC, l1=LLC/UL1). The entry
 *     carries hooks for all three levels; the dispatchers only fire the listed
 *     training level's hooks. DL0 hooks take (lineAddr, loadPC). --- */
void pref_ipcp_init(HWP* hwp);
void pref_ipcp_per_core_done(uns proc_id);
void pref_ipcp_dl0_miss(Addr lineAddr, Addr loadPC);
void pref_ipcp_dl0_hit(Addr lineAddr, Addr loadPC);
void pref_ipcp_dl0_pref_hit(Addr lineAddr, Addr loadPC);
void pref_ipcp_mlc_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipcp_mlc_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipcp_mlc_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipcp_l1_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipcp_l1_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipcp_l1_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipcp_umlc_cache_fill(uns8 proc_id, Addr fill_addr, Flag prefetch, Addr evicted_addr, uns32 metadata);

/* --- FULL two-level IPCP: L1 IPCP at DL0 (dl0 hooks) + L2 IPCP at UMLC (umlc
 *     hooks), coupled by the L1->L2 metadata channel. One pref_table entry. --- */

/* --- SPP (vendor spp.l2c) at UL1/LLC; trained via ul1_cache_fill --- */
void pref_spp_init(HWP* hwp);
void pref_spp_per_core_done(uns proc_id);
void pref_spp_dl0_miss(Addr lineAddr, Addr loadPC);
void pref_spp_dl0_hit(Addr lineAddr, Addr loadPC);
void pref_spp_dl0_pref_hit(Addr lineAddr, Addr loadPC);
void pref_spp_mlc_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_spp_mlc_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_spp_mlc_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_spp_l1_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_spp_l1_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_spp_l1_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_spp_ul1_cache_fill(uns8 proc_id, Addr fill_addr, Flag prefetch, Addr evicted_addr, uns32 metadata);

/* --- IP-stride (vendor ip_stride.l2c, DPC2 baseline); trainable at any one level --- */
void pref_ipstride_init(HWP* hwp);
void pref_ipstride_per_core_done(uns proc_id);
void pref_ipstride_dl0_miss(Addr lineAddr, Addr loadPC);
void pref_ipstride_dl0_hit(Addr lineAddr, Addr loadPC);
void pref_ipstride_dl0_pref_hit(Addr lineAddr, Addr loadPC);
void pref_ipstride_mlc_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipstride_mlc_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipstride_mlc_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipstride_l1_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipstride_l1_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_ipstride_l1_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);

/* --- next-line (vendor next_line.l2c, ChampSim streaming baseline) --- */
void pref_nextline_init(HWP* hwp);
void pref_nextline_per_core_done(uns proc_id);
void pref_nextline_dl0_miss(Addr lineAddr, Addr loadPC);
void pref_nextline_dl0_hit(Addr lineAddr, Addr loadPC);
void pref_nextline_dl0_pref_hit(Addr lineAddr, Addr loadPC);
void pref_nextline_mlc_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_nextline_mlc_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_nextline_mlc_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_nextline_l1_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_nextline_l1_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);
void pref_nextline_l1_pref_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist);

#endif  // __PREF_CHAMPSIM_H__

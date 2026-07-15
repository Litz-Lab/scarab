# Universal ChampSim → Scarab Prefetcher Shim

Drop in an **unmodified** DPC3-era ChampSim prefetcher (`*.l2c_pref` / `*.l1d_pref`) and run it inside
Scarab's prefetch framework. Glue is written once; adding a prefetcher = drop in the vendor file + a
~60-line wrapper + a table entry. **Status: original goal met** — universal shim built and verified
with MLOP and IPCP (plus SPP). Three framework capabilities were added so the shim can match ChampSim
across cache levels, cooperating components, and eviction training.

## Layout

```
champsim/
  champsim_shim.h / .cc      # shim_issue (level routing), level_from_fill, tick, addr helpers, globals
  csenv/                     # fake ChampSim headers (cache.h, champsim.h, memory_class.h, ooo_cpu.h, spp.h)
                             # + byte-identical vendor sources as *.inc (diff vs Berti-Artifact = empty)
  wrappers/                  # one small .cc per import (3 template stubs + hand-written ipcp/mlop)
  pref_champsim.h            # hook declarations (included by ../pref_common.c before pref_table.def)
  pref_champsim.stat.def     # shared PREF_SHIM_* counters for all imports
  pref_<name>.param.def/.h    # only for imports with knobs (mlop, ipstride)
```

## Mechanism

- Each wrapper `#include`s the vendor `.inc` inside `namespace cs_<name>{}`; `csenv/cache.h` is a
  concrete `class CACHE`, so each prefetcher gets a distinct `cs_<name>::CACHE` — no ODR collision.
- ChampSim globals (`NUM_CPUS`, `FILL_*`, `warmup_complete[]`, `ooo_cpu[]`) stay global (pre-included);
  `ooo_cpu[].num_retired` is wired to Scarab's real `inst_count[]`.
- `CACHE::prefetch_line()` → `champsim::shim_issue(level, …)` → `pref_addto_{dl0,umlc,ul1}_req_queue`.
- Vendor TUs compiled with `-w` (CMake `set_source_files_properties`) so vendor warnings don't trip
  Scarab's `-Werror`. `csenv` is excluded from the build glob (the `.inc` never compile standalone).

## Framework capabilities added (all gated/opt-in; legacy prefetchers unaffected)

1. **Real L1D prefetch fill** — `--pref_dl0_fill_dcache` (default **off**; pass 1 whenever an
   instance uses `DEST_DCACHE`, else its dcache-queue misses forward to the LLC queue). When on,
   the DL0 prefetch drain issues
   `new_mem_req(MRT_DPRF, …, dcache_fill_line, dest=DEST_DCACHE)` so a DL0 prefetch truly fills the
   L1D (it previously forwarded to LLC). Also fixed a pre-existing dead-code bug in the DL0 drain
   (`pref_update_core` never freed queue slots). Files: `../pref_common.c`, `../pref.param.def`.
2. **`cache_fill` / eviction dispatch** — new `ul1_cache_fill` field on `HWP` (`../pref_common.h`),
   `pref_ul1_cache_fill` dispatch (`../pref_common.c`), call site at the UL1 fill (`../memory/memory.c`,
   after the L1 `cache_insert`). Mirrors ChampSim `*_prefetcher_cache_fill`; enables eviction-trained
   prefetchers (SPP, SPP+PPF).
3. **Inter-level metadata channel** — `_downstream_op` on the shim `CACHE`: an L1 prefetch forwards
   `(pf_addr, ip, metadata)` to a cooperating L2 component, reproducing ChampSim's L1→L2 `pf_metadata`
   handoff, for future cooperating two-level imports. (The experimental two-level IPCP that used
   it was dropped from the PR: its L2 component stays inert without per-line accuracy metadata.)

## Prefetchers (validated)

Enable via the framework instance lists (`--pref_framework_on 1` plus e.g.
`--pref_num_mlc_prefetchers 1 --pref_mlc_prefetchers TYPE_IPCP,DEST_MLC`): the list a type appears
in sets its training level, the DEST_* token its destination.

| prefetcher | vendor `.inc` | list token | notes |
|---|---|---|---|
| IPCP | `ipcp_isca2020.l1d` | `TYPE_IPCP` | per-class accuracy throttle wired to fills/pref-hits |
| MLOP | `mlop_dpc3.l1d` | `TYPE_MLOP` | fixed dest (default) or `--pref_mlop_route_by_fill 1` |
| SPP | `spp.l2c` | `TYPE_SPP` | eviction-trained via `ul1_cache_fill` |
| IP-stride | `ip_stride.l2c` | `TYPE_IPSTRIDE` | DPC2 baseline; `--pref_ipstride_route_by_fill 1` optional |
| next-line | `next_line.l2c` | `TYPE_NEXTLINE` | ChampSim's sequential/streaming baseline |

## Add a new ChampSim prefetcher FOO

1. Copy unmodified `foo.<lvl>_pref` → `csenv/foo.<lvl>.inc` (+ any private `.h` it includes).
2. Write the stub `wrappers/pref_foo.cc` — for a standard vendor this is the whole file:
   ```c
   #define SHIM_NAME    foo
   #define SHIM_VENDOR  "prefetcher/champsim/csenv/foo.l2c.inc"
   #define SHIM_API_L2C 1              /* or SHIM_API_L1D */
   #include "prefetcher/champsim/shim_import.h"
   ```
   Optional knobs before the include: `SHIM_EXTRA_INIT` / `SHIM_EXTRA_OP` (custom init/operate glue,
   see `pref_ipstride.cc`) and `SHIM_GEN_UL1_CACHE_FILL` (eviction-trained vendors, see `pref_spp.cc`).
   Prefetchers needing real custom glue keep hand-written wrappers (`pref_ipcp.cc` and
   `pref_mlop.cc` are the examples).
3. Declare the generated hooks in `pref_champsim.h` and add a `pref_table.def` row. Events count
   into the shared `PREF_SHIM_*` stats automatically; add `.param.def/.param.h` (registered in
   `../param_files.def`) only if FOO has knobs.
4. Enable it with the instance lists, e.g. `--pref_num_mlc_prefetchers 1
   --pref_mlc_prefetchers TYPE_FOO,DEST_MLC`.
5. Build (`sci --build-scarab`). Never edit vendor source or the shim core per prefetcher.

## Validated results (bwaves_r sp26004; baseline 0.674, stream 1.248)

| config | IPC | note |
|---|---|---|
| ipcp @ mlc | 1.241 | **matches hand-port 1.214** |
| ipcp @ dcache | 1.201 | real L1D fill (DCACHE_MISS 1.2M→39K) |
| ipcp @ l1 (LLC) | 1.187 | |
| mlop (collapsed L2, default) | 1.019 | |
| spp | 1.149 | `PREF_SHIM_CACHE_FILL`=406K (training fires) |

**Caveat:** Scarab IPC ≠ ChampSim/paper IPC (different simulator/baseline) — compare *relative* gains.
bwaves is a pure stream (stream's best case) and can't differentiate MLOP; use a multi-workload sweep
(mcf/omnetpp/mongodb/bwaves) for a real per-prefetcher comparison.

## Shim vs hand-port (freshly measured, same arch golden_cove, bwaves_r sp26004)

Hand-port = the prior hand-rewritten ports in `scarab_uop_segment/scarab_ll` (descriptor
`scarab-infra/json/mlop_handport_cmp.json`); shim = this tree.

| prefetcher | hand-port IPC (base 0.684) | shim IPC (base 0.674) | verdict |
|---|---|---|---|
| IPCP @ mlc | 1.232 (1.80x) | 1.241 (1.84x) | **match** (+0.7%) |
| MLOP @ mlc | 1.169 (1.71x) | 1.019 (1.51x, collapsed default; issued 671K) | shim ~13% lower |

- IPCP: verbatim self-contained IPCP reproduces the hand-written port.
- MLOP: shim runs the FULL verbatim MLOP (both FILL_L1+FILL_L2 selection tiers → over-prefetches 671K
  vs the hand-port's leaner L2-only 413K). Matching the hand-port would require de-tuning the vendor's
  hardcoded thresholds (breaks "verbatim") — an algorithm-faithfulness vs hand-port difference, not a
  shim defect.

## Known gap / future work

- **Two-level IPCP L2 is inert.** The metadata channel delivers (L2 driven 12M), but the L1 encodes
  `stride=0` whenever its accuracy < 75%, and accuracy is stuck at the default 60% because per-class
  `pref_filled`/`pref_useful` aren't populated. Making the L2 issue needs a per-line/per-request
  32-bit `pf_metadata` field threaded `Pref_Mem_Req`→`Mem_Req`→cache line, with counters updated on
  fill/pref-hit. Heavy core change; deferred (beyond the original shim goal).
- SPP+PPF: same pattern as SPP (drop in `ppf.l2c` + wrapper using `ul1_cache_fill`); PPF's perceptron
  also benefits from the same accuracy feedback.
- The three framework capabilities are general, low-risk Scarab improvements and could be PR'd to
  litz-lab/scarab_ll independently of the shim.

## Build gotchas
- `CMakeLists.txt` `scarab_dirs` includes `prefetcher/champsim` + `.../wrappers` (csenv excluded).
- Stale `pin/pin_exec/obj-intel64/pin_exec.d` can reference a removed `.stat.def` after deleting a
  prefetcher → "No rule to make target"; `rm -rf` that obj dir.

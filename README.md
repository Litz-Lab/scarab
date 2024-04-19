# Scarab Quick Start Guide
Install:
1. Install exact PIN version ([PIN 3.15](https://www.intel.com/content/www/us/en/developer/articles/tool/pin-a-binary-instrumentation-tool-downloads.html))
2. Export the following paths
  - `export PIN_ROOT=/path/to/pin-3.15`
  - `export SCARAB_ENABLE_PT_MEMTRACE=1`
3. `cd src && make`

Run:
1. Copy: `src/PARAMS.sunny_cove` into your run directory and rename to `PARAMS.in`
2. Run: `src/scarab --frontend memtrace --cbp_trace_r0=<MEMTRACE_FILE> --memtrace_modules_log=<MODULES_LOG_AND_BINARIES_DIR>`

# Scarab

Scarab is a cycle accurate simulator for state-of-the-art, high performance,
multicore chips. Scarab's goal is to be highly accurate, while also being
fast and easy to work with.

##### Simulator Features:
* Accurate: Scarab is detailed cycle accurate uArchitecture model
* Fast: 600 KIPS trace-driven, 100 KIPS exec-driven
* SimPoint Support: Checkpoints, Fast-Forward, Marker Instructions
* Execute-at-Fetch: Easier support for oracle features, faster development of new features

##### v.2.0 Release Features:
* Support for Dynomrio Memtrace and Intel Processor Trace (PT) frontends
* Wrong-path execution for trace-based frontends (instruction replay)

##### What Code Can Scarab Run?
* Single-threaded x86\_64 programs that can be run on Intel's [PIN](https://software.intel.com/en-us/articles/pin-a-dynamic-binary-instrumentation-tool)

##### Scarab uArchitecture:
* All typical pipeline stages and out-of-order structures (Fetch, Decode, Rename, Retire, ROB, R/S, and more...)
* Multicore 
* Wrong path simulation
* Cache Hierarchy (Private L1, Private MLC, Private/Shared LLC)
* Ramulator Memory Simulator (DDR3/4, LPDDR3/4, GDDR5, HBM, WideIO/2, and more...)  
* Interface to McPat and CACTI for system level power/energy modeling
* Support for DVFS
* Latest Branch Predictors and Data Prefetchers (TAGE-SC-L, Stride, Stream, 2dc, GHB, Markov, and more...)

##### v.2.0 uArchitecture Extensions:
* Decoupled Frontend
* Micro-op Cache
* Register Renaming (limited GRF/rename stalls)
* Updated branch predictor and increased recovery accuracy
* FDIP/UDP prefetcher

##### Code Limitations
* 32-bit binaries not supported (work in progress)
* Performance of System Code not modeled
* No cooperative multithreaded code

##### uArch Limitations
* No SMT
* No real OS virtual to physical address translation
* Shared bus interconnect only (ring, mesh, and others are in progress.)

##### Credits 
Scarab was created in collaboration with HPS and SAFARI. This project was sponsored by Intel Labs.
Scarab v.2.0 was created and is currently maintained by UCSC.

The Scarab v.2.0 artifact is the result of our UDP ISCA 2024 paper. If you are using Scarab v.2.0 in your research please cite:

```
@inproceedings{oh2024udp,
  author = {Oh, Surim and Xu, Mingsheng and Khan, Tanvir Ahmed and Kasikci, Baris and Litz, Heiner},
  title = {UDP: Utility-Driven Fetch Directed Instruction Prefetching},
  booktitle = {Proceedings of the 51st International Symposium on Computer Architecture (ISCA)},
  series = {ISCA 2024},
  year = {2024},
  month = jun,
}
```

## License & Copyright
Please see the [LICENSE](LICENSE) for more information.

## Getting Started

1. [System requirements and software prerequisites.](docs/system_requirements.md)
2. [Compiling Scarab.](docs/compiling-scarab.md)
3. [Setting up and running auto-verification on Scarab.](docs/verification.md)
4. Running a single program on Scarab.
5. Running multiple jobs locally or on a batch system. (coming soon!)
6. Viewing batch job status and results. (coming soon!)
7. [Simulating dynamorio memtraces](docs/memtrace.md)
8. Solutions to common Scarab problems.

## Contributing to Scarab

Found a bug? [File a bug report.](https://github.com/hpsresearchgroup/scarab/issues/new/choose)

Request a new feature? [File a feature request.](https://github.com/hpsresearchgroup/scarab/issues/new/choose)

Have code you would like to commit? [Create a pull request.](https://github.com/hpsresearchgroup/scarab/pulls)

## Other Resources


1) Auto-generated software documentation can be found [here](docs/doxygen/index.html).

* Please run this command in this directory to auto-generate documentation files.
> make -C docs

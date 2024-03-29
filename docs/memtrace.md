# Trace Frontends

## Memtrace Frontend
Scarab supports simulating instruction and memory address traces in the memtrace format,
obtained with dynamorio. Memtraces contain basic block (BBL) PC addresses of all executed
BBLs and memory addresses of all instructions accessing main memory. To simulate a
memtrace, scarab needs to be provided with the trace and a module.log file that contains
the absolute paths to the traced binary and all referenced libraries.

## PT Frontend
Scarab supports simulating instruction traces obtained with Intel Processor Trace (PT).
To simulate a PT trace, scarab needs to be provided with the trace file and the absolute
paths to the trace file.

##### Compiling the Trace Frontend
1. Install additional dependencies: snappy, dl, config++, z, rt, pthread
2. $ export SCARAB_ENABLE_PT_MEMTRACE=1
3. $ make (from the main scarab directory)

##### Capturing Memtraces
1. Trace a workload
$ <SCARAB_BUILD_DIR>/deps/dynamorio/bin64/drrun -c <SCARAB_BUILD_DIR>deps/dynamorio/clients/lib64/release/libdrmemtrace.so -offline -trace_after_instrs 1M -exit_after_tracing 1G -- <TRACED_WORKLOAD>
2. Copy binaries and shared libs and convert trace 
$ sh run_portabilize_trace.sh (run from the directory that contains the drmemtrace.* directory).
3. Fix absolute paths in modules.log (needs to be performed whenever the bin directory is moved)
$ sh run_update_trace.sh

##### Simulating Memtraces with Scarab
$ scarab
--frontend memtrace --fetch_off_path_ops 0
--cbp_trace_r0=<TRACE_DIRECTORY>
--memtrace_modules_log=<MODULES_LOG_FILE_DIRECTORY>

##### Simulating PT traces with Scarab
$ scarab
--frontend pt --fetch_off_path_ops 0
--cbp_trace_r0=<TRACE_DIRECTORY>

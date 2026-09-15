cmake_minimum_required(VERSION 3.13)

project(scarab C CXX ASM)

# Always emit build/<mode>/compile_commands.json. Consumed by:
#   - clangd / VS Code / any LSP-based editor for accurate include resolution.
#   - scarab-infra's `./sci --lint-scarab` (clang-tidy needs it to parse TUs).
#   - iwyu, clang-check, and other static analyzers.
# Cost is a single JSON file; does not affect compilation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SCARAB_ENABLE_LTO "Link-time optimization for the scarab executable" ON)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(flags_enable_pt_memtrace "")

if(DEFINED ENV{SCARAB_ENABLE_PT_MEMTRACE})
  set(flags_enable_pt_memtrace "-DENABLE_PT_MEMTRACE")
endif()

# SCARABOPT: -O3 portable x86-64 (make opt). OPT_AVX: same + -march=haswell (make opt-avx).
#
# OPT_AVX targets haswell (256-bit AVX2, FMA, BMI2) rather than skylake-avx512.
# -march=skylake-avx512 emits AVX-512 instructions, so the resulting binary dies
# with SIGILL ("Illegal instruction") on any host without AVX-512 -- including all
# AMD Zen hosts. haswell is the widest baseline that every x86-64 machine from
# Haswell/Zen1 onward can execute, which keeps one build usable across a
# heterogeneous cluster instead of silently restricting it to Intel nodes.
set(CMAKE_C_FLAGS_SCARABOPT "-O3 -g3 -DNO_DEBUG -DLINUX -DX86_64 ${flags_enable_pt_memtrace}")
set(CMAKE_CXX_FLAGS_SCARABOPT "-O3 -g3 -DNO_DEBUG -DLINUX -DX86_64 ${flags_enable_pt_memtrace}")
set(CMAKE_C_FLAGS_OPT_AVX "-O3 -g3 -march=haswell -DNO_DEBUG -DLINUX -DX86_64 ${flags_enable_pt_memtrace}")
set(CMAKE_CXX_FLAGS_OPT_AVX "-O3 -g3 -march=haswell -DNO_DEBUG -DLINUX -DX86_64 ${flags_enable_pt_memtrace}")

set(CMAKE_C_FLAGS_VALGRIND    "-O0 -g3 -DLINUX -DX86_64 ${flags_enable_pt_memtrace}")
set(CMAKE_CXX_FLAGS_VALGRIND  "-O0 -g3 -DLINUX -DX86_64 ${flags_enable_pt_memtrace}")
set(CMAKE_C_FLAGS_GPROF       "${CMAKE_CXX_FLAGS_SCARABOPT} -pg -g3 ${flags_enable_pt_memtrace}")
set(CMAKE_CXX_FLAGS_GPROF     "${CMAKE_CXX_FLAGS_SCARABOPT} -pg -g3 ${flags_enable_pt_memtrace}")
set(CMAKE_C_FLAGS_DEBUG       "-O0 -g3 -DLINUX -DX86_64 -fsanitize=address -fsanitize-address-use-after-scope ${flags_enable_pt_memtrace}")
set(CMAKE_CXX_FLAGS_DEBUG     "-O0 -g3 -DLINUX -DX86_64 -fsanitize=address -fsanitize-address-use-after-scope ${flags_enable_pt_memtrace}")

# Default ASan runtime options (embedded; ASAN_OPTIONS in the environment still overrides).
set(SCARAB_ASAN_DEFAULT_OPTIONS
  "check_initialization_order=true:detect_stack_use_after_return=true:strict_string_checks=true")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/support/asan_default_options.cc.in
  ${CMAKE_CURRENT_BINARY_DIR}/asan_default_options.cc
  @ONLY)

# Turn off doc generation before adding the subdirectory
set(BUILD_DOCS OFF CACHE BOOL "Disable DynamoRIO doc generation" FORCE)
#build dependencies with default warn flags, otherwise dynamorio will not build
add_subdirectory(deps)

set(warn_flags -Wall -Wunused -Wno-long-long
  -Wpointer-arith
  -Werror)
set(warn_c_flags ${warn_flags}
  -Wmissing-declarations
  -Wmissing-prototypes
  -Wimplicit -Wno-unused-but-set-variable -Wno-maybe-uninitialized)
set(warn_cxx_flags ${warn_flags})

add_compile_options(
  "$<$<COMPILE_LANGUAGE:C>:${warn_c_flags}>"
  "$<$<COMPILE_LANGUAGE:CXX>:${warn_cxx_flags}>"
)

add_subdirectory(ramulator)
add_subdirectory(pin/pin_lib)
add_subdirectory(pin/pin_exec/testing)

set(scarab_dirs bp debug bp/template_lib dvfs frontend globals isa libs memory power prefetcher confidence .)
if(DEFINED ENV{SCARAB_ENABLE_PT_MEMTRACE})
  set(scarab_dirs ${scarab_dirs} frontend/pt_memtrace)
endif()

set(srcs)
foreach(dir IN LISTS scarab_dirs) 
    file(GLOB dir_srcs CONFIGURE_DEPENDS
      ${dir}/*.h
      ${dir}/*.cc
      ${dir}/*.c
    )
    set(srcs ${srcs} ${dir_srcs})
endforeach()

add_executable(scarab 
    ${srcs}
)

target_sources(scarab PRIVATE
  $<$<CONFIG:Debug>:${CMAKE_CURRENT_BINARY_DIR}/asan_default_options.cc>
)

target_include_directories(scarab PRIVATE .)

target_link_libraries(scarab
    PRIVATE
        ramulator
        pin_lib_for_scarab
)
if(DEFINED ENV{SCARAB_ENABLE_PT_MEMTRACE})
  target_link_libraries(scarab PRIVATE dynamorio pt_memtrace)
endif()

if(SCARAB_ENABLE_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT _scarab_lto_ok OUTPUT _scarab_lto_reason LANGUAGES C CXX)
  if(_scarab_lto_ok AND CMAKE_C_COMPILER_ID STREQUAL "GNU")
    include(ProcessorCount)
    ProcessorCount(_scarab_lto_jobs)
    if(_scarab_lto_jobs EQUAL 0)
      set(_scarab_lto_jobs 1)
    endif()
    # -flto=N, not the plain -flto that INTERPROCEDURAL_OPTIMIZATION emits: the
    # serial link dominates an incremental build (2m55s vs 1m15s on 20 cores).
    # GCC only -- for clang the argument names a mode, not a job count.
    target_compile_options(scarab PRIVATE -flto=${_scarab_lto_jobs} -fno-fat-lto-objects)
    target_link_options(scarab PRIVATE -flto=${_scarab_lto_jobs})
  elseif(_scarab_lto_ok)
    set_property(TARGET scarab PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(STATUS "Scarab LTO not enabled: ${_scarab_lto_reason}")
  endif()
endif()

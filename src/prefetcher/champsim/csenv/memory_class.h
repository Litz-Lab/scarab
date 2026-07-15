/* Universal ChampSim shim: minimal fake of ChampSim's inc/memory_class.h.
 * Only the access TYPE codes that DPC3 prefetcher sources branch on. */
#ifndef MEMORY_CLASS_H
#define MEMORY_CLASS_H

#include "champsim.h"

#define LOAD                 0
#define RFO                  1
#define PREFETCH             2
#define WRITEBACK            3
#define LOAD_TRANSLATION     4
#define PREFETCH_TRANSLATION 5
#define TRANSLATION_FROM_L1D 6
#define NUM_TYPES            7

#endif  // MEMORY_CLASS_H

/***************************************************************************************
 * File         : D_JOLT.cc
 * Date         : 2/20/2024
 * Description  : D-JOLT: Distant Jolt Prefetcher from IPC-1
 * Cite: Nakamura et al., "D-JOLT: Distant Jolt Prefetcher"
 ***************************************************************************************/
#include "debug/debug_macros.h"
#include "debug/debug_print.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"

#include "globals/assert.h"
#include "globals/utils.h"

#include "prefetcher/D_JOLT.h"

extern "C" {
#include "op.h"
#include "prefetcher/pref.param.h"
#include "memory/memory.param.h"
#include "debug/debug.param.h"
#include "memory/memory.h"
#include "general.param.h"
}

#include <iostream>
#include <array>
#include <vector>
#include <algorithm>
#include <utility>
#include <memory>
#include <numeric>

uint32_t djolt_proc_id;
#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_DJOLT, ##args)
// ============================================================
//  D-JOLT parameters.
// ============================================================
static constexpr size_t UpperBitPtrBits = 5; // This determines that the number of upper bit variations that D-JOLT can handle.
static constexpr size_t UpperBitMask = 0xffffffffff000000; // This defines where upper bit is.

static constexpr size_t SignatureBits = 23; // Note: if you change this, you may need to re-tune the hash function in siggens.

#define LongRangePrefetcherSiggen Siggen_FifoRetCnt<7>
static constexpr size_t LongRangePrefetcherDistance = 15;
//static constexpr size_t LongRangePrefetcher_N_Sets = 2048;
static constexpr size_t LongRangePrefetcher_N_Sets = 64; // For 8K budget (to compare with UDP 8K)
static constexpr size_t LongRangePrefetcher_N_Ways = 4;
static constexpr size_t LongRangePrefetcher_N_Vectors = 2;
static constexpr size_t LongRangePrefetcher_VectorSize = 8;
static constexpr size_t LongRangePrefetcher_TagBits = 12;
// -------------------------------------
//  miss table of long-range prefetcher
// -------------------------------------
// 4 way, 2048 Sets (8192 entry)
// Budget:
//  Signature Tag :      (23 - 11) bits X 8192 entry =  98304 bits
//  miss vector : (5 + 18 + 8) bits X 2 X 8192 entry = 507904 bits
//     compressed upper address :  5 bits
//     lower address            : 18 bits
//     bit vector               :  8 bits
//  lru bit :                    2 bits X 8192 entry =  16384 bits
//                                               Total 622592 bits

// ----------------------------------------------
//  signature generator of long-range prefetcher
// ----------------------------------------------
// Budget:
//  7 entry Queue
//   address : 32 bits X 7 entry = 224 bits
//   head pointer :                  3 bits
//   return counter :               32 bits
//                           Total 259 bits

// ----------------------------------------------
//  signature queue of long-range prefetcher
// ----------------------------------------------
// Budget:
//  15 entry Queue
//   signature : 23 bits X 15 entry = 345 bits
//   head pointer :                     4 bits
//                              Total 349 bits

#define ShortRangePrefetcherSiggen Siggen_FifoRetCnt<4>
static constexpr size_t ShortRangePrefetcherDistance = 4;
//static constexpr size_t ShortRangePrefetcher_N_Sets = 1024;
static constexpr size_t ShortRangePrefetcher_N_Sets = 32; // For 8K budget (to compare with UDP 8K)
static constexpr size_t ShortRangePrefetcher_N_Ways = 4;
static constexpr size_t ShortRangePrefetcher_N_Vectors = 2;
static constexpr size_t ShortRangePrefetcher_VectorSize = 8;
static constexpr size_t ShortRangePrefetcher_TagBits = 13;
// --------------------------------------
//  miss table of short-range prefetcher
// --------------------------------------
// 4 way, 1024 Sets (4096 entry)
// Budget:
//  Signature Tag :      (23 - 10) bits X 4096 entry =  53248 bits
//  miss vector : (5 + 18 + 8) bits X 2 X 4096 entry = 253952 bits
//     compressed upper address :  5bits
//     lower address            : 18bits
//     bit vector               :  8bits
//  lru bit :                     2bits X 4096 entry =   8192 bits
//                                               Total 315392 bits
 
// ----------------------------------------------
//  signature generator of short-range prefetcher
// ----------------------------------------------
// Budget:
//  4 entry Queue
//   address : 3 2bits X 4 entry = 128 bits
//   head pointer :                  2 bits
//   return counter :               32 bits
//                           Total 162 bits

// ----------------------------------------------
//  signature queue of short-range prefetcher
// ----------------------------------------------
// Budget:
//  4 entry Queue
//   signature : 23 bits X 4 entry = 92 bits
//   head pointer :                   2 bits
//                             Total 94 bits

static constexpr size_t ExtraMissTable_N_Sets = 256;
static constexpr size_t ExtraMissTable_N_Ways = 4;
static constexpr size_t ExtraMissTable_N_Vectors = 2;
static constexpr size_t ExtraMissTable_VectorSize = 8;
static constexpr size_t ExtraMissTable_TagBits = 15;
// ---------------------
//  extra miss table
// ---------------------
// 4 way, 256 Sets (1024 entry)
// Budget:
//  signature Tag :         (23 - 8) bits X 1024 entry = 15360 bits
//  miss vector :   (5 + 18 + 8) bits X 2 X 1024 entry = 63488 bits
//     compressed upper address :  5 bits
//     lower address            : 18 bits
//     bit vector               :  8 bits
//  lru bit :                      2 bits X 1024 entry =  2048 bits
//                                                 Total 80896 bits

// ============================================================

// This is a compressed expression of upper bit.
struct UpperBitPtr {
    size_t ptr;
    bool operator==(const UpperBitPtr& rhs) const noexcept { return ptr == rhs.ptr; }
    bool operator!=(const UpperBitPtr& rhs) const noexcept { return ptr != rhs.ptr; }
};

// This is a compressed representation of line address.
struct CompressedLineAddress {
    UpperBitPtr upper_part;
    uint64_t lower_part;
    bool isValid() const noexcept { return upper_part.ptr != 0; }
};

// This table records the correspondence between the compressed expression and the original expression.
class UpperBitTable {
    struct Entry { bool valid; uint64_t upper_bits; };
    std::array<Entry, (1ull << UpperBitPtrBits) - 1> table = {};
public:
    std::pair<bool, CompressedLineAddress> compress(uint64_t full_address) {
        const uint64_t upper_bits = full_address & UpperBitMask;
        const uint64_t lower_bits = (full_address & ~UpperBitMask) >> LOG2(ICACHE_LINE_SIZE);

        const auto exists_pos = std::find_if(table.begin(), table.end(), [upper_bits](const Entry& e) noexcept { return e.valid && upper_bits == e.upper_bits; });
        const bool entry_exists = exists_pos != table.end();

        if (entry_exists) {
            return { true, { static_cast<size_t>(exists_pos - table.begin()) + 1, lower_bits } };
        } else {
            const auto invalid_pos = std::find_if(table.begin(), table.end(), [](const Entry& e) noexcept { return !e.valid; });
            const bool invalid_entry_found = invalid_pos != table.end();

            if (invalid_entry_found) {
                (*invalid_pos) = { true, upper_bits };
                return { true, { static_cast<size_t>(invalid_pos - table.begin()) + 1, lower_bits } };
            } else {
                return { false, {} };
            }
        }
    }

    uint64_t decompress(CompressedLineAddress cla) const {
        return table.at(cla.upper_part.ptr - 1).upper_bits + (cla.lower_part << LOG2(ICACHE_LINE_SIZE));
    }
};

// ---------------------
//  upper bit table
// ---------------------
// 31 Sets Fully-asociative table
// Budget:
//  upper bit : 40 bits X 31 entry = 1240 bits
//  Valid :      1 bit  X 31 entry =   31 bits
//                             Total 1271 bits

// utility functions

template<size_t N>
std::array<size_t, N> make_initialized_lru_order() {
    std::array<size_t, N> ret;
    std::iota(ret.begin(), ret.end(), 0);
    return ret;
}

template<class T>
void update_lru_order(T& lru_order, size_t touch_pos) noexcept {
    ASSERT(djolt_proc_id, touch_pos < lru_order.size());
    for (auto& e : lru_order) {
        if (e < lru_order.at(touch_pos)) { ++e; }
    }
    lru_order.at(touch_pos) = 0;
}

template<size_t HistLen>
class Siggen_FifoRetCnt {
    std::array<uint32_t, HistLen> ghist = {};
    size_t head = 0;
    size_t return_count = 0;

    uint32_t makeSig() const noexcept {
        uint32_t sig = 0;
        for (size_t i = head; i < head + HistLen; ++i) {
            const uint32_t pc = ghist.at(i % HistLen);
            sig = (sig << (SignatureBits - 5)) | (sig >> 5);
            sig ^= pc ^ pc >> 2; // work well on both A64/x86
            sig &= ((1ull << SignatureBits) - 1);
        }
        sig ^= return_count * 0xabcd;
        return sig & ((1ull << SignatureBits) - 1);
    }
public:
    uint32_t onReturnInstruction(uint64_t, uint64_t) {
        ++return_count;
        return makeSig();
    }
    uint32_t onCallInstruction(uint64_t ip, uint64_t) {
        return_count = 0;
        ghist.at(head) = ip;
        head = (head + 1) % HistLen;
        return makeSig();
    }
};

template<size_t VectorSize>
class MissInfo {
    CompressedLineAddress base_address = {};
    std::array<bool, VectorSize> bit_vector = {};
public:
    bool add(CompressedLineAddress address) noexcept {
        if (base_address.isValid()) {
            if (base_address.upper_part.ptr != address.upper_part.ptr) { return false; }
            const int64_t diff = address.lower_part - base_address.lower_part;
            if (diff < 0) {
                return false;
            } else if (diff == 0) {
                return true;
            } else if (static_cast<size_t>(diff-1) < bit_vector.size()) {
                bit_vector.at(diff-1) = true;
                return true;
            } else {
                return false;
            }
        } else {
            base_address = address;
            return true;
        }
    }
    bool isValid() const noexcept { return base_address.isValid(); }
    std::vector<CompressedLineAddress> getAddresses() const {
        ASSERT(djolt_proc_id, isValid());
        std::vector<CompressedLineAddress> ret;
        ret.push_back(base_address);
        for (size_t i = 0; i < bit_vector.size(); ++i) {
            if (bit_vector.at(i)) { CompressedLineAddress tmp = base_address; tmp.lower_part += (i+1); ret.push_back(tmp); }
        }
        return ret;
    }
};

template<size_t N_Vectors, size_t VectorSize>
class MissTableEntry {
    std::array<MissInfo<VectorSize>, N_Vectors> elems = {};
public:
    bool insert_but_do_not_evict(CompressedLineAddress address) {
        for (size_t i = 0; i < N_Vectors; ++i) {
            const bool success = elems.at(i).add(address);
            if (success) { return true; }
        }
        return false;
    }
    std::vector<MissInfo<VectorSize>> getValidEntries() const {
        std::vector<MissInfo<VectorSize>> ret;
        for (const auto& e : elems) {
            if (e.isValid()) { ret.push_back(e); }
        }
        return ret;
    }
};

template<size_t N_Ways, class T, class U, class Hasher>
class FullyAssociativeLRUTable {
    struct Entry {
        size_t tag;
        U value;
        bool valid;
        Entry() : tag(0), value(), valid(false) {}
    };
    std::array<Entry, N_Ways> table = {};
    std::array<size_t, N_Ways> lru_order = make_initialized_lru_order<N_Ways>();

    size_t find_index_of(const T& key) const {
        const size_t tag = Hasher{}(key);
        return std::find_if(table.begin(), table.end(), [tag](const Entry& entry) noexcept { return entry.valid && entry.tag == tag; }) - table.begin();
    }
public:
    const U& operator[](const T& key) const {
        ASSERT(djolt_proc_id, contains(key));
        return table.at(find_index_of(key)).value;
    }
    U& operator[](const T& key) {
        ASSERT(djolt_proc_id, contains(key));
        return table.at(find_index_of(key)).value;
    }

    void touch(const T& key) {
        ASSERT(djolt_proc_id, contains(key));
        update_lru_order(lru_order, find_index_of(key));
    }
    void insert(const T& key, const U& elem) {
        if (contains(key)) {
            const size_t index = find_index_of(key);
            table.at(index).value = elem;
            touch(key);
        } else {
            const size_t victim_index = std::max_element(lru_order.begin(), lru_order.end()) - lru_order.begin();
            table.at(victim_index).tag = Hasher{}(key);
            table.at(victim_index).value = elem;
            table.at(victim_index).valid = true;
            touch(key);
        }
    }
    bool contains(const T& key) const {
        return find_index_of(key) != N_Ways;
    }
};

template<size_t N_Ways, class T>
class FullyAssociativeLRUSet {
    std::array<T, N_Ways> table = {};
    std::array<size_t, N_Ways> lru_order = make_initialized_lru_order<N_Ways>();
public:
    T& at(size_t i) { return table.at(i); }
    const T& at(size_t i) const { return table.at(i); }
    void touch(size_t i) { update_lru_order(lru_order, i); }
    size_t find_lru_index() { return static_cast<size_t>(std::max_element(lru_order.begin(), lru_order.end()) - lru_order.begin()); }
};

template<size_t N_Sets, size_t N_Ways, class T, class U, class Hasher>
class SetAssociativeLRUTable {
    struct HasherForIndex { size_t operator()(const T& key) const { return Hasher{}(key) % N_Sets; } };
    struct HasherForTag { size_t operator()(const T& key) const { return Hasher{}(key) / N_Sets; } };
    using Entry = FullyAssociativeLRUTable<N_Ways, T, U, HasherForTag>;
    std::array<Entry, N_Sets> table = {};
public:
    const U& operator[](const T& key) const { return table.at(HasherForIndex{}(key))[key]; }
    U& operator[](const T& key) { return table.at(HasherForIndex{}(key))[key]; }
    void touch(const T& key) { table.at(HasherForIndex{}(key)).touch(key); }
    void insert(const T& key, const U& elem) { table.at(HasherForIndex{}(key)).insert(key, elem); }
    bool contains(const T& key) const { return table.at(HasherForIndex{}(key)).contains(key); }
};

class WindowBasedStreamPrefetcher {
    static constexpr size_t TrainingThreshold = 3;
    static constexpr size_t WindowSize = 2;
    static constexpr size_t Distance = 2;
    static constexpr size_t Degree = 2;
    static constexpr size_t TrainingTableSize = 16;
    static constexpr size_t MonitoringTableSize = 16;

    struct TrainingStreamEntry {
        bool valid;
        uint64_t start_line_address;
        size_t count;
    };

    struct MonitoringStreamEntry {
        bool valid;
        uint64_t start_line_address;
    };

    FullyAssociativeLRUSet<TrainingTableSize, TrainingStreamEntry> training_table = {};
    FullyAssociativeLRUSet<MonitoringTableSize, MonitoringStreamEntry> monitoring_table = {};
// ---------------------
//  train table
// ---------------------
// 16 entry fully-asociative table
// Budget:
//  line address : 58 bits X 16 entry = 928 bits
//  valid :         1 bit  X 16 entry =  16 bits
//  counter :       2 bits X 16 entry =  32 bits
//  lru bit :       4 bits X 16 entry =  64 bits
//                               Total 1040 bits

// ---------------------
//  monitor table
// ---------------------
// 16 entry fully-asociative table
// Budget:
//  line address : 58 bits X 16 entry = 928 bits
//  valid :         1 bit  X 16 entry =  16 bits
//  lru bit :       4 bits X 16 entry =  64 bits
//                               Total 1008 bits

    // Is line_address in [start_line_adress, start_line_adress + range_size)?
    static bool is_in_range(uint64_t line_address, uint64_t start_line_address, size_t range_size) noexcept {
        return start_line_address <= line_address && line_address < start_line_address + range_size;
    }

    void allocate_training_stream(uint64_t line_address) {
        const size_t lru_index = training_table.find_lru_index();
        training_table.at(lru_index) = TrainingStreamEntry { /* valid = */true, /* start_line_address = */ line_address, /* count = */0 };
        training_table.touch(lru_index);
    }

    bool update_training_stream_and_prefetch(uint64_t line_address) {
        for (size_t i = 0; i < TrainingTableSize; ++i) {
            TrainingStreamEntry& stream = training_table.at(i);
            if (!stream.valid) { continue; }

            if (!is_in_range(line_address, stream.start_line_address, WindowSize)) { continue; }

            ++stream.count;

            if (stream.count >= TrainingThreshold) {
                prefetch_initial_stream(line_address, stream);
                allocate_monitoring_stream(MonitoringStreamEntry { /* valid = */ true, /* start_line_address = */ line_address }); // line_address, line_address+1, ... will be prefetched.
                stream.valid = false;
            } else {
                training_table.touch(i);
            }
            return true;
        }

        return false;
    }

    void allocate_monitoring_stream(MonitoringStreamEntry entry) {
        const size_t lru_index = monitoring_table.find_lru_index();
        monitoring_table.at(lru_index) = std::move(entry);
        monitoring_table.touch(lru_index);
    }

    bool update_monitoring_stream_and_prefetch(uint64_t line_address) {
        for (size_t i = 0; i < MonitoringTableSize; ++i) {
            MonitoringStreamEntry& stream = monitoring_table.at(i);
            if (!stream.valid) { continue; }

            // Check a missed address is in a prefetch window.
            if (!is_in_range(line_address, stream.start_line_address, Distance)) { continue; }

            // Update, issue prefetch, and touch a entry
            for (size_t j = 0; j < Degree; ++j) {
                const uint64_t pf_addr = (stream.start_line_address + Distance) << LOG2(ICACHE_LINE_SIZE);

                new_mem_req(MRT_IPRF, djolt_proc_id, pf_addr, ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count, 0);
                INC_STAT_EVENT(0, DJOLT_PREFETCH_ENTRY, 1);
                ++stream.start_line_address;
            }
            monitoring_table.touch(i);
            return true;
        }
        return false;
    }

    void prefetch_initial_stream(uint64_t line_address, const TrainingStreamEntry& stream) {
        // i == 0 is not needed since it is the same line as the demand access.
        for (size_t i = 1; i < Distance; ++i) {
            const uint64_t pf_addr = (line_address + i) << LOG2(ICACHE_LINE_SIZE);
            new_mem_req(MRT_IPRF, djolt_proc_id, pf_addr, ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count, 0);
            INC_STAT_EVENT(0, DJOLT_PREFETCH_INITIAL, 1);
        }
    }

public:
    void cache_operate(uint64_t address, uint8_t cache_hit, uint8_t prefetch_hit) {
        const bool virtual_miss = cache_hit == 0 || prefetch_hit == 1;
        const uint64_t line_address = address >> LOG2(ICACHE_LINE_SIZE);

        const bool already_monitored_stream = update_monitoring_stream_and_prefetch(line_address);

        if (virtual_miss) {
            const bool already_training_stream = update_training_stream_and_prefetch(line_address);

            if (!already_monitored_stream && !already_training_stream) {
                allocate_training_stream(line_address);
            }
        }
    }
};

template<size_t N>
class SignatureQueue {
    std::array<uint32_t, N> queue = {};
    size_t head = 0;
public:
    uint32_t back() const noexcept { return queue.at((head + 1) % N); }
    void insert(uint32_t x) noexcept { queue.at((head + 1) % N) = x; head = (head + 1) % N; }
};
    

class D_JOLT_PREFETCHER {
    uns proc_id;
    struct SigHasher { size_t operator()(const uint32_t& x) const noexcept { return x; } };

    ShortRangePrefetcherSiggen siggen_1 = {};
    LongRangePrefetcherSiggen siggen_2 = {};

    SignatureQueue<ShortRangePrefetcherDistance> sig_history_1 = {};
    SignatureQueue<LongRangePrefetcherDistance> sig_history_2 = {};

    SetAssociativeLRUTable<ShortRangePrefetcher_N_Sets, ShortRangePrefetcher_N_Ways, uint32_t, MissTableEntry<ShortRangePrefetcher_N_Vectors, ShortRangePrefetcher_VectorSize>, SigHasher> miss_table_1 = {};
    SetAssociativeLRUTable<LongRangePrefetcher_N_Sets, LongRangePrefetcher_N_Ways, uint32_t, MissTableEntry<LongRangePrefetcher_N_Vectors, LongRangePrefetcher_VectorSize>, SigHasher> miss_table_2 = {};
    SetAssociativeLRUTable<ExtraMissTable_N_Sets, ExtraMissTable_N_Ways, uint32_t, MissTableEntry<ExtraMissTable_N_Vectors, ExtraMissTable_VectorSize>, SigHasher> extra_miss_table = {};

    WindowBasedStreamPrefetcher stream_prefetcher = {};

    UpperBitTable upper_bit_table = {};

    template<class T>
    void print_parameter(const std::string& str, T& param) {
        std::cout << str << ": " << param << std::endl;
    }

public:
    D_JOLT_PREFETCHER(uns proc_id);

    template<class Table>
    void prefetch_with_sig(const Table& table, uint32_t sig);
    template<class Table>
    void learn_with_sig(Table& table, uint32_t sig, CompressedLineAddress c_address);
    void branch_operate(uint64_t ip, uint8_t branch_type, uint64_t branch_target);
    void cache_operate(uint64_t addr, uint8_t cache_hit, uint8_t prefetch_hit);
    void cycle_operate();
    void final_stats();
    void cache_fill(uint64_t v_addr, uint64_t evicted_v_addr);
};

std::vector<D_JOLT_PREFETCHER *> per_core_d_jolt_prefetcher;

D_JOLT_PREFETCHER::D_JOLT_PREFETCHER(uns proc_id) : proc_id(proc_id) {
    std::cout << "L1I D-JOLT instruction prefetcher has been constructed!" << std::endl;
}

template<class Table>
void D_JOLT_PREFETCHER::prefetch_with_sig(const Table& table, uint32_t sig) {
    if (table.contains(sig)) {
        for (const auto& v : table[sig].getValidEntries()) {
            for (const auto& address : v.getAddresses()) {
                const uint64_t pf_addr = upper_bit_table.decompress(address);
                new_mem_req(MRT_IPRF, proc_id, pf_addr, ICACHE_LINE_SIZE, 0, NULL, instr_fill_line, unique_count, 0);
                INC_STAT_EVENT(0, DJOLT_PREFETCH_SIG, 1);
            }
        }
    }
}

void D_JOLT_PREFETCHER::branch_operate(uint64_t ip, uint8_t branch_type, uint64_t branch_target) {
    uint32_t sig_1;
    uint32_t sig_2;

    if (branch_type == CF_CBR || branch_type == CF_IBR) {
        sig_1 = siggen_1.onCallInstruction(ip, branch_target);
        sig_2 = siggen_2.onCallInstruction(ip, branch_target);
    } else if (branch_type == CF_RET) {
        sig_1 = siggen_1.onReturnInstruction(ip, branch_target);
        sig_2 = siggen_2.onReturnInstruction(ip, branch_target);
    } else {
        return;
    }

    // Make sure storage limits are adhered to...
    ASSERT(proc_id, sig_1 < (1ull<<ExtraMissTable_TagBits) * ExtraMissTable_N_Sets);
    ASSERT(proc_id, sig_2 < (1ull<<ExtraMissTable_TagBits) * ExtraMissTable_N_Sets);
    //ASSERT(proc_id, sig_1 < (1ull<<ShortRangePrefetcher_TagBits) * ShortRangePrefetcher_N_Sets);
    //ASSERT(proc_id, sig_2 < (1ull<<LongRangePrefetcher_TagBits) * LongRangePrefetcher_N_Sets);

    sig_history_1.insert(sig_1);
    sig_history_2.insert(sig_2);

    prefetch_with_sig(miss_table_1, sig_1);
    prefetch_with_sig(extra_miss_table, sig_1);
    prefetch_with_sig(miss_table_2, sig_2);
    prefetch_with_sig(extra_miss_table, sig_2);
}

template<class Table>
void D_JOLT_PREFETCHER::learn_with_sig(Table& table, uint32_t sig, CompressedLineAddress c_address) {
    if (!table.contains(sig)) {
        table.insert(sig, {});
    } else {
        table.touch(sig);
    }
    const bool success = table[sig].insert_but_do_not_evict(c_address);
    if (!success) {
        if (!extra_miss_table.contains(sig)) {
            extra_miss_table.insert(sig, {});
        } else {
            extra_miss_table.touch(sig);
        }
        extra_miss_table[sig].insert_but_do_not_evict(c_address);
    } else if (extra_miss_table.contains(sig)) {
        extra_miss_table.touch(sig);
    }
}

void D_JOLT_PREFETCHER::cache_operate(uint64_t addr, uint8_t cache_hit, uint8_t prefetch_hit) {
    const bool miss = cache_hit == 0;
    //const bool virtual_miss = cache_hit == 0 || prefetch_hit == 1;
    stream_prefetcher.cache_operate(addr, cache_hit, prefetch_hit);

    if (miss) {
        const auto compress_result = upper_bit_table.compress(addr);
        const bool compress_success = compress_result.first;
        const CompressedLineAddress c_address = compress_result.second;

        if (!compress_success) { return; }

        learn_with_sig(miss_table_1, sig_history_1.back(), c_address);
        learn_with_sig(miss_table_2, sig_history_2.back(), c_address);
    }
}

void D_JOLT_PREFETCHER::cycle_operate()
{
}

void D_JOLT_PREFETCHER::final_stats()
{
}

void D_JOLT_PREFETCHER::cache_fill(uint64_t v_addr, uint64_t evicted_v_addr)
{
}


void alloc_mem_djolt(uns numCores) {
  per_core_d_jolt_prefetcher.resize(numCores);
}

void init_djolt(uns proc_id) {
  if (!DJOLT_ENABLE)
    return;
  per_core_d_jolt_prefetcher[proc_id] = new D_JOLT_PREFETCHER(proc_id);
}

void set_djolt(uns proc_id) {
  djolt_proc_id = proc_id;
}

void update_djolt(uns proc_id, Addr fetch_addr, uint8_t branch_type, uint64_t branch_target) {
  if (!DJOLT_ENABLE)
    return;
  per_core_d_jolt_prefetcher[proc_id]->branch_operate(fetch_addr, branch_type, branch_target);
}

void djolt_prefetch(uns proc_id, uint64_t v_addr, uint8_t cache_hit, uint8_t prefetch_hit) {
  per_core_d_jolt_prefetcher[proc_id]->cache_operate(v_addr, cache_hit, prefetch_hit);
}

void djolt_cycle_operate(uns proc_id) {
  per_core_d_jolt_prefetcher[proc_id]->cycle_operate();
}

void print_djolt_stats(uns proc_id) {
  per_core_d_jolt_prefetcher[proc_id]->final_stats();
}

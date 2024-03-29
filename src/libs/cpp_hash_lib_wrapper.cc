#include "libs/cpp_hash_lib_wrapper.h"
#include <unordered_map>
#include <vector>
#include <iostream>

struct key {
  uint64_t addr;
  uint64_t lsb_bytes;
  uint64_t msb_bytes;
  uint8_t op_idx;

  key () : addr(0), lsb_bytes(0), msb_bytes(0), op_idx(0) {};
  key ( uint64_t _addr, uint64_t _lsb_bytes, uint64_t _msb_bytes, uint8_t _op_idx) :
  addr(_addr), lsb_bytes(_lsb_bytes), msb_bytes(_msb_bytes), op_idx(_op_idx) {};

  bool operator==(const key &p) const {
    return addr == p.addr && lsb_bytes == p.lsb_bytes && msb_bytes == p.msb_bytes && op_idx == p.op_idx;
  }
};

// The specialized hash function for `unordered_map` keys
struct hash_fn
{
  //template <class T1, class T2>
    std::size_t operator() (const key &key) const
    {
        std::size_t h1 = std::hash<uint64_t>()(key.addr);
        std::size_t h2 = std::hash<uint64_t>()(key.lsb_bytes);
        std::size_t h3 = std::hash<uint64_t>()(key.msb_bytes);
        std::size_t h4 = std::hash<uint8_t>()(key.op_idx);
        return h1 ^ h2 ^ h3 ^ h4;
    }
};

std::unordered_map<key, Inst_Info*, hash_fn> hash_map;

  Inst_Info *cpp_hash_table_access_create(int core, uint64_t addr, uint64_t lsb_bytes, uint64_t msb_bytes, uint8_t op_idx, unsigned char *new_entry) {
    *new_entry = false;
    key _key(addr, lsb_bytes, msb_bytes, op_idx);
    auto lookup = hash_map.find(_key);
    if (lookup != hash_map.end()) {
      return lookup->second;
    }
    else {
      Inst_Info * info = new Inst_Info(); //&vec.back();
      hash_map.insert(std::pair<key, Inst_Info*>(_key, info));
      *new_entry = true;
      return info;
    }
  }


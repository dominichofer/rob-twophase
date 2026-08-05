#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "cubie.h"

namespace move
{
  const int COUNT = 18;

  extern std::string names[COUNT];
  extern cubie::cube cubes[COUNT];
  extern int inv[COUNT];

  extern uint64_t next[COUNT]; // successor moves that should be explored
  extern uint64_t next_p1p2[COUNT]; // `next` for phase1 to phase 2 transition

  extern uint64_t p1_mask; // phase 1 moves
  extern uint64_t p2_mask; // phase 2 moves

  inline uint64_t bit(int m) { return uint64_t(1) << m; }
  inline bool in(int m, uint64_t mm) { return mm & bit(m); }

  // Convert solution to AXHT; especially useful when solving in AXQT
  std::string compress(const std::vector<int>& mseq);

  /* Compute solution length */
  int len_ht(const std::vector<int>& mseq);

  void init();
}

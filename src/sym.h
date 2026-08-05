#pragma once
#include "coord.h"
#include "cubie.h"
#include "move.h"

namespace sym {

  const int COUNT = 48;

  const int COUNT_SUB = 16;
  const int N_FLIP_SLICE1 = 64430;
  const int N_CORNERS = 2768;
  const int ROT = 16; // 120 degree rotation around axis through URF and DLB corner

  extern std::array<cubie::cube, COUNT> cubes;
  extern std::array<int, COUNT> inv;
  extern std::array<std::array<int, 3>, COUNT> effect;

  extern std::array<std::array<int, COUNT>, move::COUNT> conj_move;
  extern std::array<std::array<uint16_t, COUNT_SUB>, coord::N_TWIST> conj_twist;
  extern std::array<std::array<uint16_t, COUNT_SUB>, coord::N_UD_EDGES2> conj_ud_edges2;

  extern std::array<uint32_t, coord::N_FLIP_SLICE1> fslice1_sym;
  extern std::array<uint32_t, coord::N_CORNERS> corners_sym;
  extern std::array<uint32_t, N_FLIP_SLICE1> fslice1_raw;
  extern std::array<uint16_t, N_CORNERS> corners_raw;
  extern std::array<uint16_t, N_FLIP_SLICE1> fslice1_selfs;
  extern std::array<uint16_t, N_CORNERS> corners_selfs;

  inline bool eff_inv(int eff) { return eff & 1; }
  inline bool eff_flip(int eff) { return eff & 2; }
  inline int eff_shift(int eff) { return eff >> 2; }
  inline int coord_c(int coord) { return coord / COUNT_SUB; }
  inline int coord_s(int coord) { return coord % COUNT_SUB; }

  void init();

}

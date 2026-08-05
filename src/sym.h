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

  extern cubie::cube cubes[COUNT];
  extern int inv[COUNT];
  extern int effect[COUNT][3];

  extern int conj_move[move::COUNT][COUNT];
  extern uint16_t conj_twist[coord::N_TWIST][COUNT_SUB];
  extern uint16_t conj_ud_edges2[coord::N_UD_EDGES2][COUNT_SUB];

  extern uint32_t fslice1_sym[coord::N_FLIP_SLICE1];
  extern uint32_t corners_sym[coord::N_CORNERS];
  extern uint32_t fslice1_raw[N_FLIP_SLICE1];
  extern uint16_t corners_raw[N_CORNERS];
  extern uint16_t fslice1_selfs[N_FLIP_SLICE1];
  extern uint16_t corners_selfs[N_CORNERS];

  inline bool eff_inv(int eff) { return eff & 1; }
  inline bool eff_flip(int eff) { return eff & 2; }
  inline int eff_shift(int eff) { return eff >> 2; }
  inline int coord_c(int coord) { return coord / COUNT_SUB; }
  inline int coord_s(int coord) { return coord % COUNT_SUB; }

  void init();

}

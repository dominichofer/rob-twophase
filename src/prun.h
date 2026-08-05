#pragma once
#include <array>
#include <cstdint>

#include "coord.h"
#include "sym.h"

namespace prun {

  const int N_FS1TWIST = sym::N_FLIP_SLICE1 * coord::N_TWIST; // 64430 * 3^(8 - 1) = 140'908'410
  const int N_CORNUD2 = sym::N_CORNERS * coord::N_UD_EDGES2; // 2768 * 8! = 111'605'760
  const int N_CSLICE2 = coord::N_CORNERS * coord::N_SLICE2; // 8! * 4! = 967'680

  extern std::array<uint32_t, N_FS1TWIST> phase1;
  extern std::array<uint8_t, N_CORNUD2> phase2;
  extern std::array<uint8_t, N_CSLICE2> precheck;

  int get_phase1(int flip, int slice, int twist, int togo, uint64_t& next);
  int get_phase2(int corners, int ud_edges);
  int get_precheck(int corners, int slice);

  bool init(bool file = true);

}

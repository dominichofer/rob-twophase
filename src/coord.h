#pragma once
#include <array>
#include "cubie.h"
#include "move.h"

namespace coord {

  const int N_FLIP = 2048; // 2^(12 - 1)
  const int N_TWIST = 2187; // 3^(8 - 1)
  const int N_SLICE1 = 495; // binom(12, 4)
  const int N_FLIP_SLICE1 = 1013760; // N_FLIP * N_SLICE

  const int N_SLICE = 11880; // 12! / 8!
  const int N_U_EDGES = 11880; // 12! / 8!
  const int N_D_EDGES = 11880; // 12! / 8!

  const int N_SLICE2 = 24; // 4!
  const int N_UD_EDGES2 = 40320; // 8!
  const int N_CORNERS = 40320; // 8!

  const int SLICE1_SOLVED = 494; // SLICE1 is not 0 at the end of phase 1

  extern std::array<std::array<uint16_t, move::COUNT>, N_FLIP> move_flip;
  extern std::array<std::array<uint16_t, move::COUNT>, N_TWIST> move_twist;
  extern std::array<std::array<uint16_t, move::COUNT>, N_SLICE> move_edges4;
  extern std::array<std::array<uint16_t, move::COUNT>, N_CORNERS> move_corners;
  extern std::array<std::array<uint16_t, move::COUNT>, N_UD_EDGES2> move_ud_edges2; // primarily for faster phase 2 table generation

  int get_flip(const cubie::cube& c);
  int get_twist(const cubie::cube& c);
  int get_slice(const cubie::cube& c);
  int get_u_edges(const cubie::cube& c);
  int get_d_edges(const cubie::cube& c);
  int get_corners(const cubie::cube& c);

  void set_flip(cubie::cube& c, int flip);
  void set_twist(cubie::cube& c, int twist);
  void set_slice(cubie::cube& c, int slice);
  void set_u_edges(cubie::cube& c, int u_edges);
  void set_d_edges(cubie::cube& c, int d_edges);
  void set_corners(cubie::cube& c, int corners);

  int get_slice1(const cubie::cube& c); // faster table generation
  void set_slice1(cubie::cube& c, int slice1);
  int get_ud_edges2(const cubie::cube& c);
  void set_ud_edges2(cubie::cube& c, int ud_edges2);
  inline int merge_ud_edges2(int u_edges, int d_edges) { return 24 * u_edges + (d_edges % 24); };

  inline int slice_to_slice1(int slice) { return slice / 24; }
  inline int slice1_to_slice(int slice1) { return 24 * slice1; }
  inline int slice_to_slice2(int slice) { return slice - N_SLICE2 * SLICE1_SOLVED; }
  inline int slice2_to_slice(int slice2) { return slice2 + N_SLICE2 * SLICE1_SOLVED; }
  inline int fslice1(int flip, int slice1) { return N_FLIP * slice1 + flip; }
  inline int fslice1_to_flip(int fslice1) { return fslice1 % N_FLIP; }
  inline int fslice1_to_slice1(int fslice1) { return fslice1 / N_FLIP; }

  void init();

}

#include "coord.h"

#include <algorithm>
#include <bit>
#include <cstring>
#include <functional>
#include <span>

#include "cubie.h"

namespace coord {

  const int N_C12K4 = 495; // binom(12, 4)
  const int N_PERM4 = 24; // 4!

  std::array<std::array<uint16_t, move::COUNT>, N_FLIP> move_flip;
  std::array<std::array<uint16_t, move::COUNT>, N_TWIST> move_twist;
  std::array<std::array<uint16_t, move::COUNT>, N_SLICE> move_edges4;
  std::array<std::array<uint16_t, move::COUNT>, N_CORNERS> move_corners;
  std::array<std::array<uint16_t, move::COUNT>, N_UD_EDGES2> move_ud_edges2;

  /* Used for en-/decoding pos-perm coords */
  std::array<uint8_t, 1 << (4 * 2)> enc_perm; // encode 4-elem perm as 8 bits
  std::array<uint8_t, N_PERM4> dec_perm;
  std::array<uint16_t, 1 << 12> enc_comb; // encode 4-elem comb as 12-bit mask with exactly 4 bits on
  std::array<uint16_t, N_C12K4> dec_comb;

  int binarize_perm(const std::array<int, 4>& perm) {
    int bin = 0;
    for (int i = 3; i >= 0; i--)
      bin = (bin << 2) | perm[i];
    return bin;
  }

  void init_encdec() {
    std::array<int, 4> perm = {0, 1, 2, 3};
    for (int i = 0; i < N_PERM4; i++) {
      int bin = binarize_perm(perm);
      enc_perm[bin] = i;
      dec_perm[i] = bin;
      std::next_permutation(perm.begin(), perm.end());
    }

    int i = 0;
    for (uint32_t comb = 0; comb < (1 << cubie::edge::COUNT); comb++) {
      if (std::popcount(comb) == 4) {
        enc_comb[comb] = i;
        dec_comb[i] = comb;
        i++;
      }
    }
  }

  template <std::size_t N>
  int get_ori(const std::array<int, N>& oris, int n_oris) {
    int val = 0;
    for (std::size_t i = 0; i < N - 1; i++) // last ori can be reconstructed by parity
      val = n_oris * val + oris[i];
    return val;
  }

  template <std::size_t N>
  void set_ori(int val, std::array<int, N>& oris, int n_oris) {
    int par = 0;
    for (int i = static_cast<int>(N) - 2; i >= 0; i--) {
      oris[i] = val % n_oris;
      par += oris[i];
      val /= n_oris;
    }
    // Ori parity must always be 0
    oris[N - 1] = (n_oris - par % n_oris) % n_oris;
  }

  // `mask` indicates which 4 edges to compute the coordinate for
  template <std::size_t N>
  int get_combperm(const std::array<int, N>& cubies, int mask) {
    int min_cubie = ffs(mask) - 1;

    int comb = 0;
    int perm = 0;

    for (int i = static_cast<int>(N) - 1; i >= 0; i--) {
      if (mask & (1 << cubies[i])) {
        comb |= 1 << i;
        perm = (perm << 2) | (cubies[i] - min_cubie);
      }
    }

    return N_PERM4 * enc_comb[comb] + enc_perm[perm];
  }

  template <std::size_t N>
  void set_combperm(int comb, int perm, std::array<int, N>& cubies, int min_cubie) {
    comb = dec_comb[comb];
    perm = dec_perm[perm];

    int cubie = 0;
    for (int i = 0; i < static_cast<int>(N); i++) {
      if (cubie == min_cubie)
        cubie += 4;
      if (comb & (1 << i)) {
        cubies[i] = (perm & 0x3) + min_cubie;
        perm >>= 2;
      } else
        cubies[i] = cubie++;
    }
  }

  /* Faster than using `*_comperm()` twice */

  int get_perm8(std::span<const int, 8> cubies) {
    int comb1 = 0;
    int perm1 = 0;
    int perm2 = 0;

    for (int i = 7; i >= 0; i--) {
      if (cubies[i] < 4) {
        comb1 |= 1 << i;
        perm1 = (perm1 << 2) | cubies[i];
      } else
        perm2 = (perm2 << 2) | (cubies[i] - 4);
    }

    comb1 = enc_comb[comb1];
    perm1 = enc_perm[perm1];
    perm2 = enc_perm[perm2];
    return N_PERM4 * (N_PERM4 * comb1 + perm1) + perm2;
  }

  void set_perm8(int perm8, std::span<int, 8> cubies) {
    int perm2 = dec_perm[perm8 % N_PERM4];
    int comb1 = dec_comb[(perm8 / N_PERM4) / N_PERM4];
    int perm1 = dec_perm[(perm8 / N_PERM4) % N_PERM4];

    for (int i = 0; i < 8; i++) {
      if (comb1 & (1 << i)) {
        cubies[i] = perm1 & 0x3;
        perm1 >>= 2;
      } else {
        cubies[i] = (perm2 & 0x3) + 4;
        perm2 >>= 2;
      }
    }
  }

  int get_twist(const cubie::cube& c) {
    return get_ori(c.c_ori, 3);
  }

  void set_twist(cubie::cube& c, int twist) {
    set_ori(twist, c.c_ori, 3);
  }

  int get_flip(const cubie::cube& c) {
    return get_ori(c.e_ori, 2);
  }

  void set_flip(cubie::cube& c, int flip) {
    set_ori(flip, c.e_ori, 2);
  }

  int get_slice(const cubie::cube& c) {
    return get_combperm(c.e_prm, 0xf00);
  }

  void set_slice(cubie::cube& c, int slice) {
    set_combperm(slice / N_PERM4, slice % N_PERM4, c.e_prm, cubie::edge::FR);
  }

  int get_u_edges(const cubie::cube& c) {
    return get_combperm(c.e_prm, 0x00f);
  }

  void set_u_edges(cubie::cube& c, int u_edges) {
    set_combperm(u_edges / N_PERM4, u_edges % N_PERM4, c.e_prm, cubie::edge::UR);
  }

  int get_d_edges(const cubie::cube& c) {
    return get_combperm(c.e_prm, 0x0f0);
  }

  void set_d_edges(cubie::cube& c, int d_edges) {
    set_combperm(d_edges / N_PERM4, d_edges % N_PERM4, c.e_prm, cubie::edge::DR);
  }

  int get_corners(const cubie::cube& c) {
    return get_perm8(c.c_prm);
  }

  void set_corners(cubie::cube& c, int corners) {
    set_perm8(corners, c.c_prm);
  }

  /* Dedicated methods again more efficient than `*_posperm()` */

  int get_slice1(const cubie::cube& c) {
    int slice1 = 0;
    for (int i = cubie::edge::COUNT - 1; i >= 0; i--) {
      if (c.e_prm[i] >= cubie::edge::FR)
        slice1 |= 1 << i;
    }
    return enc_comb[slice1];
  }

  void set_slice1(cubie::cube& c, int slice1) {
    slice1 = dec_comb[slice1];
    int j = cubie::edge::FR;
    int cubie = 0;
    for (int i = 0; i < cubie::edge::COUNT; i++)
      c.e_prm[i] = (slice1 & (1 << i)) ? j++ : cubie++;
  }

  int get_ud_edges2(const cubie::cube& c) {
    return get_perm8(std::span(c.e_prm).first<8>());
  }

  void set_ud_edges2(cubie::cube& c, int ud_edges2) {
    set_perm8(ud_edges2, std::span(c.e_prm).first<8>());
  }

  // Computing only exactly the moves that are needed and storing them tightly would only make things more complicated
  // during solving (in exchange for completely negligible setup/memory-gains)
  void init_move(
    std::span<std::array<uint16_t, move::COUNT>> move_coord,
    std::function<int(const cubie::cube&)> get_coord,
    std::function<void(cubie::cube&, int)> set_coord,
    std::function<void(const cubie::cube&, const cubie::cube&, cubie::cube&)> mul,
    bool phase2 = false
  ) {
    cubie::cube c1 = cubie::SOLVED_CUBE; // coords only affect perm or ori -> one would be uninitialized
    cubie::cube c2;

    for (int coord = 0; coord < static_cast<int>(move_coord.size()); coord++) {
      set_coord(c1, coord);

      if (phase2) { // Ud_edges2 is only defined for phase 2 moves
        for (uint64_t moves = move::p2_mask; moves; moves &= moves - 1) {
          int m = std::countr_zero(moves);
          mul(c1, move::cubes[m], c2);
          move_coord[coord][m] = get_coord(c2);
        }
      } else {
        for (int m = 0; m < move::COUNT; m++) {
          mul(c1, move::cubes[m], c2);
          move_coord[coord][m] = get_coord(c2);
        }
      }
    }
  }

  void init() {
    init_encdec();

    init_move(move_flip, get_flip, set_flip, cubie::edge::mul);
    init_move(move_twist, get_twist, set_twist, cubie::corner::mul);
    init_move(move_edges4, get_slice, set_slice, cubie::edge::mul);
    init_move(move_corners, get_corners, set_corners, cubie::corner::mul);
    init_move(move_ud_edges2, get_ud_edges2, set_ud_edges2, cubie::edge::mul, true);
  }

}

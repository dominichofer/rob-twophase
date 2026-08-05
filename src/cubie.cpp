#include "cubie.h"

#include <algorithm>
#include <random>

#include "coord.h"

namespace cubie {

  /* Faster than tricky if-else sequences for handling mirrored states */
  std::array<std::array<int, 6>, 6> mul_c_oris = {{
    {0, 1, 2, 3, 4, 5},
    {1, 2, 0, 4, 5, 3},
    {2, 0, 1, 5, 3, 4},
    {3, 5, 4, 0, 2, 1},
    {4, 3, 5, 1, 0, 2},
    {5, 4, 3, 2, 1, 0}
  }};
  std::array<int, 6> inv_c_ori = {
    0, 2, 1, 3, 4, 5
  };

  std::random_device device;
  std::mt19937 gen(device());

  void corner::mul(const cubie::cube& c1, const cubie::cube& c2, cubie::cube& into) {
    for (int i = 0; i < corner::COUNT; i++) {
      into.c_prm[i] = c1.c_prm[c2.c_prm[i]];
      into.c_ori[i] = mul_c_oris[c1.c_ori[c2.c_prm[i]]][c2.c_ori[i]];
    }
  }

  void edge::mul(const cubie::cube& c1, const cubie::cube& c2, cubie::cube& into) {
    for (int i = 0; i < edge::COUNT; i++) {
      into.e_prm[i] = c1.e_prm[c2.e_prm[i]];
      into.e_ori[i] = (c1.e_ori[c2.e_prm[i]] + c2.e_ori[i]) & 1;
    }
  }

  // Permutation parity = #inversions % 2
  template <std::size_t N>
  bool parity(const std::array<int, N>& perm) {
    int par = 0;
    for (std::size_t i = 0; i < N; i++) {
      for (std::size_t j = 0; j < i; j++) {
        if (perm[j] > perm[i])
          par++;
      }
    }
    return par & 1;
  }

  void mul(const cubie::cube& c1, const cubie::cube& c2, cubie::cube& into) {
    corner::mul(c1, c2, into);
    edge::mul(c1, c2, into);
  }

  void inv(const cube& c, cube& into) {
    for (int corner = 0; corner < corner::COUNT; corner++)
      into.c_prm[c.c_prm[corner]] = corner; // inv[a[i]] = i
    for (int edge = 0; edge < edge::COUNT; edge++)
      into.e_prm[c.e_prm[edge]] = edge;
    for (int i = 0; i < corner::COUNT; i++)
      into.c_ori[i] = inv_c_ori[c.c_ori[into.c_prm[i]]];
    for (int i = 0; i < edge::COUNT; i++)
      into.e_ori[i] = c.e_ori[into.e_prm[i]];
  }

  template <std::size_t N>
  bool is_permutation(std::array<int, N> prm)
  {
      std::sort(prm.begin(), prm.end());

      for (std::size_t i = 0; i < N; ++i)
          if (prm[i] != static_cast<int>(i))
              return false;

      return true;
  }

  template <std::size_t N>
  bool valid_orientation(const std::array<int, N>& ori, int count)
  {
      int sum = 0;

      for (int o : ori) {
          if (o < 0 || o >= count)
              return false;
          sum += o;
      }

      return sum % count == 0;
  }

  int check(const cube& c)
  {
      if (!is_permutation(c.c_prm))
          return 1; // invalid corner permutation

      if (!valid_orientation(c.c_ori, 3))
          return 2; // invalid corner orientation

      if (!is_permutation(c.e_prm))
          return 3; // invalid edge permutation

      if (!valid_orientation(c.e_ori, 2))
          return 4; // invalid edge orientation

      if (parity(c.c_prm) != parity(c.e_prm))
          return 5; // corner and edge permutation parity mismatch

      return 0;
  }

  void shuffle(cube& c) {
    for (int i = 0; i < corner::COUNT; i++)
      c.c_prm[i] = i;
    for (int i = 0; i < edge::COUNT; i++)
      c.e_prm[i] = i;

    coord::set_corners(c, std::uniform_int_distribution<int>(0, coord::N_CORNERS)(gen));
    std::shuffle(c.e_prm.begin(), c.e_prm.end(), gen); // no coordinate for all edges
    if (parity(c.c_prm) != parity(c.e_prm))
      std::swap(c.c_prm[corner::COUNT - 2], c.c_prm[corner::COUNT - 1]); // flip parity

    coord::set_twist(c, std::uniform_int_distribution<int>(0, coord::N_TWIST - 1)(gen));
    coord::set_flip(c, std::uniform_int_distribution<int>(0, coord::N_FLIP - 1)(gen));
  }
}

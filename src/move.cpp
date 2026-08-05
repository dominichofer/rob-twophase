#include "move.h"

#include <span>

namespace move {

  using namespace cubie::corner;
  using namespace cubie::edge;

  /* Select moves and order according to used metric */
  const std::array<int, 45> map = {
    0, 1, 2, 3, 4, 5,
    -1, -1, -1, -1, -1, -1, -1, -1, -1,
    6, 7, 8, 9, 10, 11,
    -1, -1, -1, -1, -1, -1, -1, -1, -1,
    12, 13, 14, 15, 16, 17,
    -1, -1, -1, -1, -1, -1, -1, -1, -1
  };

  std::array<std::string, COUNT> names;
  std::array<cubie::cube, COUNT> cubes;
  std::array<int, COUNT> inv;

  std::array<uint64_t, COUNT> next;
  std::array<uint64_t, COUNT> next_p1p2;

  uint64_t p1_mask = bit(45) - 1;
  uint64_t p2_mask = 0x10482097fff; // 000010000 010010 000010000 010010 111111111 111111;

  // For full set of 45 moves no matter the solving mode
  std::array<std::string, 45> names1;
  std::array<std::array<int, 45>, 45> merge;
  std::array<int, COUNT> unmap;

  // Translate bitmask from full moveset to configured one
  uint64_t reindex(uint64_t mm) {
    uint64_t mm1 = 0;
    for (int m = 0; m < 45; m++) {
      if (map[m] != -1 && in(m, mm)) // drop unmapped moves
        mm1 |= bit(map[m]);
    }
    return mm1;
  }

  // Build full moveset first, then remap to configured one
  void init() {
    for (int m = 0; m < 45; m++) {
      if (map[m] != -1)
        unmap[map[m]] = m;
    }

    std::array<cubie::cube, 45> cubes1;
    std::array<int, 45> inv1;
    // Not initializing the following arrays apparently causes problems on MacOS
    std::array<uint64_t, 45> next1 = {};

    std::array<std::string, 6> fnames = {"U", "D", "R", "L", "F", "B"};
    std::array<std::string, 3> pnames = {"", "2", "'"};
    std::array<cubie::cube, 6> fcubes = {{
      { // U
        {UBR, URF, UFL, ULB, DFR, DLF, DBL, DRB},
        {UB, UR, UF, UL, DR, DF, DL, DB, FR, FL, BL, BR},
        {}, {}
      },
      { // D
        {URF, UFL, ULB, UBR, DLF, DBL, DRB, DFR},
        {UR, UF, UL, UB, DF, DL, DB, DR, FR, FL, BL, BR},
        {}, {}
      },
      { // R
        {DFR, UFL, ULB, URF, DRB, DLF, DBL, UBR},
        {FR, UF, UL, UB, BR, DF, DL, DB, DR, FL, BL, UR},
        {2, 0, 0, 1, 1, 0, 0, 2}, {}
      },
      { // L
        {URF, ULB, DBL, UBR, DFR, UFL, DLF, DRB},
        {UR, UF, BL, UB, DR, DF, FL, DB, FR, UL, DL, BR},
        {0, 1, 2, 0, 0, 2, 1, 0}, {}
      },
      { // F
        {UFL, DLF, ULB, UBR, URF, DFR, DBL, DRB},
        {UR, FL, UL, UB, DR, FR, DL, DB, UF, DF, BL, BR},
        {1, 2, 0, 0, 2, 1, 0, 0},
        {0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0}
      },
      { // B
        {URF, UFL, UBR, DRB, DFR, DLF, ULB, DBL},
        {UR, UF, UL, BR, DR, DF, DL, BL, FR, FL, UB, DB},
        {0, 0, 1, 2, 0, 0, 2, 1},
        {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1}
      }
    }};

    for (int ax = 0; ax < 3; ax++) {
      int i1 = 15 * ax; // index to start first face moves
      int i2 = 15 * ax + 3; // index to start of second face moves
      int i3 = 15 * ax + 6; // index to start of axial moves

      int f1 = 2 * ax; // first face
      int f2 = 2 * ax + 1; // second face

      for (int cnt = 0; cnt < 3; cnt++) {
        int m = i1 + cnt;
        names1[m] = fnames[f1] + pnames[cnt];
        if (cnt == 0)
          cubes1[m] = fcubes[f1];
        else
          cubie::mul(cubes1[m - 1], fcubes[f1], cubes1[m]);
        inv1[m] = i1 + (2 - cnt);
        next1[m] |= uint64_t(0x7) << i1; // block any moves on same face
        next1[m] |= uint64_t(0x1ff) << i3; // block all axial moves
      }
      for (int cnt = 0; cnt < 3; cnt++) {
        int m = i2 + cnt;
        names1[m] = fnames[f2] + pnames[cnt];
        if (cnt == 0)
          cubes1[m] = fcubes[f2];
        else
          cubie::mul(cubes1[m - 1], fcubes[f2], cubes1[m]);
        inv1[m] = i2 + (2 - cnt);
        next1[m] |= uint64_t(0x3f) << i1; // block all simple moves on both faces
        next1[m] |= uint64_t(0x1ff) << i3;
      }
      for (int cnt1 = 0; cnt1 < 3; cnt1++) {
        for (int cnt2 = 0; cnt2 < 3; cnt2++) {
          int m = i3 + 3 * cnt1 + cnt2;
          names1[m] = "(" + names1[i1 + cnt1] + " " + names1[i2 + cnt2] + ")";
          cubie::mul(cubes1[i1 + cnt1], cubes1[i2 + cnt2], cubes1[m]);
          inv1[m] = i3 + 3 * (2 - cnt1) + (2 - cnt2);
          next1[m] |= uint64_t(0x7fff) << 15 * ax; // block all simple and axial moves
        }
      }
    }
    // Half-slice moves commute
    next1[25] |= bit(10);
    next1[40] |= bit(10) | bit(25);
    // Was built by blocking moves, but should actually indicate permitted ones
    for (int m = 0; m < 45; m++)
      next1[m] = ~next1[m];

    for (int m = 0; m < 45; m++) {
      if (map[m] == -1)
        continue;
      int i = map[m];

      names[i] = names1[m];
      cubes[i] = cubes1[m];
      inv[i] = map[inv1[m]];
      next[i] = reindex(next1[m]);
    }

    p1_mask = reindex(p1_mask);
    p2_mask = reindex(p2_mask);

    for (int m = 0; m < COUNT; m++) {
      next_p1p2[m] = next[m]; // we can do normal blocking for phase 2 moves
    }

    cubie::cube c;
    for (int m1 = 0; m1 < 45; m1++) {
      for (int m2 = 0; m2 < 45; m2++) {
        merge[m1][m2] = -1;
        cubie::mul(cubes1[m1], cubes1[m2], c);
        for (int i = 0; i < 45; i++) {
          if (c == cubes1[i]) {
            merge[m1][m2] = i;
            break;
          }
        }
      }
    }
  }

  void compress1(const std::vector<int>& mseq, std::vector<int>& into) {
    into.clear();
    for (int m : mseq) {
      m = unmap[m];
      if (into.size() == 0 || merge[into.back()][m] == -1)
        into.push_back(m);
      else {
        int tmp = into.back();
        into.pop_back();
        into.push_back(merge[tmp][m]);
      }
    }
  }

  std::string compress(const std::vector<int>& mseq) {
    std::vector<int> comp;
    compress1(mseq, comp);

    // Faster string building probably not worth it in a function like this
    std::string s;
    for (int i = 0; i < comp.size(); i++) {
      s += names1[comp[i]];
      if (i != comp.size() - 1)
        s += " ";
    }
    return s;
  }

  int len(const std::vector<int>& mseq, std::span<const int> cost) {
    std::vector<int> comp;
    compress1(mseq, comp);

    int res = 0;
    for (int m : comp)
      res += cost[m];
    return res;
  }

  int len_ht(const std::vector<int>& mseq) {
    std::array<int, 45> cost = {
      1, 1, 1, 1, 1, 1,
      2, 2, 2, 2, 2, 2, 2, 2, 2,
      1, 1, 1, 1, 1, 1,
      2, 2, 2, 2, 2, 2, 2, 2, 2,
      1, 1, 1, 1, 1, 1,
      2, 2, 2, 2, 2, 2, 2, 2, 2
    };
    return len(mseq, cost);
  }
}

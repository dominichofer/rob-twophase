#pragma once
#include <array>
#include <string>

namespace cubie {

  /* All cubie definitions are plain integers for making uniform handling much easier */

  namespace corner { // definition of corner cubies
    const int COUNT = 8;

    const int URF = 0;
    const int UFL = 1;
    const int ULB = 2;
    const int UBR = 3;
    const int DFR = 4;
    const int DLF = 5;
    const int DBL = 6;
    const int DRB = 7;

    const std::array<std::string, COUNT> NAMES = {
      "URF", "UFL", "ULB", "UBR", "DFR", "DLF", "DBL", "DRB"
    };
  }
  using namespace corner;

  namespace edge { // definition of edge cubies
    const int COUNT = 12;

    const int UR = 0;
    const int UF = 1;
    const int UL = 2;
    const int UB = 3;
    const int DR = 4;
    const int DF = 5;
    const int DL = 6;
    const int DB = 7;
    // SLICE-edges last s.t. Ud_edges2 is easier to handle
    const int FR = 8;
    const int FL = 9;
    const int BL = 10;
    const int BR = 11;

    const std::array<std::string, COUNT> NAMES = {
      "UR", "UF", "UL", "UB", "DR", "DF", "DL", "DB", "FR", "FL", "BL", "BR"
    };
  }
  using namespace edge;

  struct cube {
    std::array<int, corner::COUNT> c_prm; // corner cubie permutation
    std::array<int, edge::COUNT>   e_prm; // edge cubie permutation
    std::array<int, corner::COUNT> c_ori; // corner cubie orientation; 0 if U/D-facelet on U/D-face; 1 clockwise rot; 2 c-clock
    std::array<int, edge::COUNT>   e_ori; // edge cubie orientation; 0 if U/D-facelet on U/D-face or same for F/B for slice edges

    bool operator==(const cube&) const = default;
    bool operator!=(const cube&) const = default;
    cube operator*(const cube& other) const;

    cube inverse() const;
    int check() const; // check a cube for being solvable
  };

  const cube SOLVED_CUBE = {
    {URF, UFL, ULB, UBR, DFR, DLF, DBL, DRB},
    {UR, UF, UL, UB, DR, DF, DL, DB, FR, FL, BL, BR},
    {}, {}
  }; // cubie-cube in solved state

  /* Explicitly pass result cube to avoid unnecessary copying during table generation */

  namespace corner {
    void mul(const cube& c1, const cube& c2, cube& into); // multiply only corner cubies
  }
  namespace edge {
    void mul(const cube& c1, const cube& c2, cube& into); // multiply only edge cubies
  }

  void shuffle(cube& c); // generate a uniformly random cube
}

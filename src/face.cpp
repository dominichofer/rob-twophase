#include "face.h"

#include <string>
#include <unordered_map>

namespace face {

  // True modulo function that also works properly for negative numbers
  int mod(int a, int m) {
    return a > 0 ? a % m : (a % m + m) % m;
  }

  // Converts a cubelet and its orientation into a single unique code-number
  int encode(const std::string& cubelet, int ori) {
    int res = 0;
    for (int i = 0; i < cubelet.size(); i++)
      res = color::COUNT * res + color::FROM_NAME.at(cubelet[mod(i - ori, cubelet.size())]);
    return res;
  }

  // Map code to cubie ID and orientation
  std::unordered_map<int, std::pair<int, int>> corners;
  std::unordered_map<int, std::pair<int, int>> edges;

  void init() {
    for (int corner = 0; corner < cubie::corner::COUNT; corner++) {
      for (int ori = 0; ori < 3; ori++)
        corners[encode(cubie::corner::NAMES[corner], ori)] = std::make_pair(corner, ori);
    }
    for (int edge = 0; edge < cubie::edge::COUNT; edge++) {
      for (int ori = 0; ori < 2; ori++)
        edges[encode(cubie::edge::NAMES[edge], ori)] = std::make_pair(edge, ori);
    }
  }

  int to_cubie(const std::string& s, cubie::cube& c) {
    for (int i = 0; i < N_FACELETS; i++) {
      if (color::FROM_NAME.find(s[i]) == color::FROM_NAME.end())
        return 1; // invalid color
      if ((i - 4) % 9 == 0 && color::FROM_NAME.at(s[i]) != i / 9)
        return 2; // invalid center facelet (they are always fixed)
    }

    for (int corner = 0; corner < cubie::corner::COUNT; corner++) {
      std::array<char, 3> cornlet;
      for (int i = 0; i < 3; i++)
        cornlet[i] = s[CORNLETS[corner][i]];
      auto tmp = corners.find(encode(std::string(cornlet.data(), 3), 0));
      if (tmp == corners.end())
        return 3; // invalid corner cubie
      c.c_prm[corner] = tmp->second.first;
      c.c_ori[corner] = tmp->second.second;
    }

    for (int edge = 0; edge < cubie::edge::COUNT; edge++) {
      std::array<char, 2> edgelet;
      for (int i = 0; i < 2; i++)
        edgelet[i] = s[EDGELETS[edge][i]];
      auto tmp = edges.find(encode(std::string(edgelet.data(), 2), 0));
      if (tmp == edges.end())
        return 4; // invalid edge cubie
      c.e_prm[edge] = tmp->second.first;
      c.e_ori[edge] = tmp->second.second;
    }

    return 0;
  }

  // Assumes the given cube to be valid
  std::string from_cubie(const cubie::cube& c) {
    std::array<char, N_FACELETS> s;

    for (int color = 0; color < color::COUNT; color++)
      s[9 * color + 4] = color::NAMES[color];
    for (int corner = 0; corner < cubie::corner::COUNT; corner++) {
      for (int i = 0; i < 3; i++)
        // Corner twist is defined clockwise
        s[CORNLETS[corner][i]] = cubie::corner::NAMES[c.c_prm[corner]][mod(i - c.c_ori[corner], 3)];
    }
    for (int edge = 0; edge < cubie::edge::COUNT; edge++) {
      for (int i = 0; i < 2; i++)
        s[EDGELETS[edge][i]] = cubie::edge::NAMES[c.e_prm[edge]][mod(i - c.e_ori[edge], 2)];
    }

    return std::string(s.data(), N_FACELETS);
  }

}

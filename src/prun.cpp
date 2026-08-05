#include "prun.h"

#include <bitset>
#include <iostream>
#include <cstring>

namespace prun {
  const std::string SAVE = "twophase-ht.tbl";

  const int EMPTY = 0xff;

  const int BITS_PER_AX = 8;
  const int BITS_PER_M = 1;
  const int N_SIMP = 6;

  // Used to remap symmetry ext. phase 1 table entries back to actual situation
  std::array<std::array<std::array<uint64_t, 1 << BITS_PER_AX>, 16>, 2> remap;

  std::array<uint32_t, N_FS1TWIST> phase1;
  std::array<uint8_t, N_CORNUD2> phase2;
  std::array<uint8_t, N_CSLICE2> precheck;

  inline int ones(int count) { return (1 << count) - 1; }

  int rev(int movec, int count, int off = 0, int step = BITS_PER_M) {
    movec >>= step * off;

    int rev = 0;
    for (int i = 0; i < step * count; i += step) {
      rev = (rev << step) | (movec & ones(step));
      movec >>= step;
    }
    return rev << step * off;
  }

  int inv(int mask) {
    int n_per_face = N_SIMP / 2;

    int inv = rev(mask, n_per_face) | rev(mask, n_per_face, n_per_face);
    return inv;
  }
  
  int flip(int mask) {
    int per_face = N_SIMP / 2;
    int flipped = rev(mask, 2, 0, BITS_PER_M * per_face);
    return flipped;
  }

  void init_base() {
    for (int eff = 0; eff < 16; eff++) {
      for (int mask = 0; mask < 256; mask++) {
        int mask1 = mask & 0xf;
        int mask2 = (mask & 0xf0) >> 4;

        if (sym::eff_inv(eff)) {
          mask1 = rev(mask1, 3, 1) | (mask1 & 1);
          mask2 = rev(mask2, 3, 1) | (mask2 & 1);
        }
        if (sym::eff_flip(eff))
          std::swap(mask1, mask2);

        remap[0][eff][mask] = ((mask1 & 1) ? 0 : ~(mask1 >> 1) & 0x7) << 6 * sym::eff_shift(eff);
        remap[1][eff][mask] = ((mask1 & 1) ? ~(mask1 >> 1) & 0x7 : 0x7) << 6 * sym::eff_shift(eff);
        remap[0][eff][mask] |= ((mask2 & 1) ? 0 : ~(mask2 >> 1) & 0x7) << 6 * sym::eff_shift(eff) + 3;
        remap[1][eff][mask] |= ((mask2 & 1) ? ~(mask2 >> 1) & 0x7 : 0x7) << 6 * sym::eff_shift(eff) + 3;
      }
    }
    return;
  }

  void init_phase1() {
    int n_moves = std::bitset<64>(move::p1_mask).count(); // make sure not to consider B-moves in F5-mode

    std::fill(phase1.begin(), phase1.end(), EMPTY);

    phase1[coord::N_TWIST * sym::coord_c(sym::fslice1_sym[coord::fslice1(0, coord::SLICE1_SOLVED)])] = 0;
    int count = 0;
    int dist = 0;

    while (count < N_FS1TWIST) {
      int coord = 0;

      for (int fs1sym = 0; fs1sym < sym::N_FLIP_SLICE1; fs1sym++) {
        int fslice1 = sym::fslice1_raw[fs1sym];
        int flip = coord::fslice1_to_flip(fslice1);
        int slice = coord::slice1_to_slice(coord::fslice1_to_slice1(fslice1));

        for (int twist = 0; twist < coord::N_TWIST; twist++) {
          if ((phase1[coord] & 0xff) == dist) {
            count++;
            std::array<int, move::COUNT> deltas; // easier encoding if B-face always exists (F5-mode ignores it anyways)

            for (int m = 0; m < n_moves; m++) {
              int slice11 = coord::slice_to_slice1(coord::move_edges4[slice][m]);
              int fslice11 = coord::fslice1(coord::move_flip[flip][m], slice11);
              int tmp = sym::fslice1_sym[fslice11];
              int twist1 = sym::conj_twist[coord::move_twist[twist][m]][sym::coord_s(tmp)];
              int fs1sym1 = sym::coord_c(tmp);
              int coord1 = coord::N_TWIST * fs1sym1 + twist1;

              if (phase1[coord1] == EMPTY)
                phase1[coord1] = dist + 1;
              deltas[m] = (phase1[coord1] & 0xff) - dist;
              coord1 -= twist1; // only TWIST part changes below

              int selfs = sym::fslice1_selfs[fs1sym1] >> 1;
              for (int s = 1; selfs > 0; s++) { // bit 0 is always on
                if (selfs & 1) {
                  int coord2 = coord1 + sym::conj_twist[twist1][s];
                  if (phase1[coord2] == EMPTY)
                    phase1[coord2] = dist + 1;
                }
                selfs >>= 1;
              }
            }

            uint32_t prun = 0;
            const int n_ax = 3;
            /* Encode from left to right to preserve indexing of moves */
            for (int ax = n_ax - 1; ax >= 0; ax--) {
              bool away = false; // first bit of axis encoding (whether any move brings us further from the goal)
              for (int i = ax * (BITS_PER_AX - 1); i < (ax + 1) * (BITS_PER_AX - 1); i++) {
                if (deltas[i] != 0) {
                  if (deltas[i] > 0)
                    away = true;
                  break; // stop immediately once we found a value != 0
                }
              }

              int tmp = 0;
              for (int i = (ax + 1) * (BITS_PER_AX - 1) - 1; i >= ax * (BITS_PER_AX - 1); i--)
                tmp = (tmp | (away ? deltas[i] : deltas[i] + 1)) << 1;
              tmp |= away;

              prun = (prun << BITS_PER_AX) | tmp;
            }
            phase1[coord] |= prun << 8;
          }
          coord++;
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }
  }

  void init_phase2() {
    std::fill(phase2.begin(), phase2.end(), EMPTY);

    phase2[0] = 0;
    int count = 0;
    int dist = 0;

    while (count < N_CORNUD2) {
      int coord = 0;

      for (int csym = 0; csym < sym::N_CORNERS; csym++) {
        int corners = sym::corners_raw[csym];

        for (int ud_edges2 = 0; ud_edges2 < coord::N_UD_EDGES2; ud_edges2++) {
          if (phase2[coord] == dist) {
            count++;

            for (uint64_t moves = move::p2_mask; moves; moves &= moves - 1) {
              int m = std::countr_zero(moves);

              int dist1 = dist + 1;

              int corners1 = coord::move_corners[corners][m];
              int ud_edges21 = coord::move_ud_edges2[ud_edges2][m];
              int tmp = sym::corners_sym[corners1];
              ud_edges21 = sym::conj_ud_edges2[ud_edges21][sym::coord_s(tmp)];
              int csym1 = sym::coord_c(tmp);
              int coord1 = coord::N_UD_EDGES2 * csym1 + ud_edges21;

              if (phase2[coord1] <= dist1)
                continue;
              phase2[coord1] = dist1;
              coord1 -= ud_edges21;

              int selfs = sym::corners_selfs[csym1] >> 1;
              for (int s = 1; selfs > 0; s++) {
                if (selfs & 1) {
                  int coord2 = coord1 + sym::conj_ud_edges2[ud_edges21][s];
                  if (phase2[coord2] > dist1)
                    phase2[coord2] = dist1;
                }
                selfs >>= 1;
              }
            }
          }
          coord++;
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }
  }

  void init_precheck() {
    std::fill(precheck.begin(), precheck.end(), EMPTY);

    precheck[0] = 0;
    int dist = 0;
    int count = 0;

    while (count < N_CSLICE2) {
      int coord = 0;

      for (int corners = 0; corners < coord::N_CORNERS; corners++) {
        for (int slice2 = 0; slice2 < coord::N_SLICE2; slice2++) {
          if (precheck[coord] == dist) {
            count++;
            int slice = coord::slice2_to_slice(slice2);

            for (uint64_t moves = move::p2_mask; moves; moves &= moves - 1) {
              int m = std::countr_zero(moves);

              int dist1 = dist + 1;

              int corners1 = coord::move_corners[corners][m];
              int slice21 = coord::slice_to_slice2(coord::move_edges4[slice][m]);

              int coord1 = coord::N_SLICE2 * corners1 + slice21;
              if (precheck[coord1] > dist1)
                precheck[coord1] = dist1;
            }
          }
          coord++;
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }
  }

  int get_phase1(int flip, int slice, int twist, int togo, uint64_t& next) {
    int tmp = sym::fslice1_sym[coord::fslice1(flip, coord::slice_to_slice1(slice))];
    int s = sym::coord_s(tmp);
    uint32_t prun = phase1[coord::N_TWIST * sym::coord_c(tmp) + sym::conj_twist[twist][s]];

    int dist = prun & 0xff;
    int delta = togo - dist;

    // `delta` < 0 case can never happen during a real search
    if (delta > 1)
      next = move::p1_mask; // all moves are possible
    else {
      prun >>= 8; // get rid of dist
      next = 0;
      for (int ax = 0; ax < 3; ax++) {
        next |= remap[delta][sym::effect[s][ax]][prun & ones(BITS_PER_AX)];
        prun >>= BITS_PER_AX;
      }
    }

    return dist;
  }

  int get_phase2(int corners, int ud_edges) {
    int tmp = sym::corners_sym[corners];
    return phase2[coord::N_UD_EDGES2 * sym::coord_c(tmp) + sym::conj_ud_edges2[ud_edges][sym::coord_s(tmp)]];
  }

  int get_precheck(int corners, int slice) {
    return precheck[coord::N_SLICE2 * corners + coord::slice_to_slice2(slice)];
  }

  bool init(bool file) {
    init_base();

    if (!file) {
      init_phase1();
      init_phase2();
      init_precheck();
      return true;
    }

    FILE *f = fopen(SAVE.c_str(), "rb");
    int err = 0;

    if (f == NULL) {
      init_phase1();
      init_phase2();
      init_precheck();

      f = fopen(SAVE.c_str(), "wb");
      if (fwrite(phase1.data(), sizeof(uint32_t), N_FS1TWIST, f) != N_FS1TWIST)
        err = 1;
      if (fwrite(phase2.data(), sizeof(uint8_t), N_CORNUD2, f) != N_CORNUD2)
        err = 1;
      if (fwrite(precheck.data(), sizeof(uint8_t), N_CSLICE2, f) != N_CSLICE2)
        err = 1;
      if (err)
        remove(SAVE.c_str()); // delete file if there was some error writing it
    } else {
      if (fread(phase1.data(), sizeof(uint32_t), N_FS1TWIST, f) != N_FS1TWIST)
        err = 1;
      if (fread(phase2.data(), sizeof(uint8_t), N_CORNUD2, f) != N_CORNUD2)
        err = 1;
      if (fread(precheck.data(), sizeof(uint8_t), N_CSLICE2, f) != N_CSLICE2)
        err = 1;
    }

    fclose(f);
    return err;
  }

}

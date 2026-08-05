#include <array>
#include <bitset>
#include <chrono>
#include <functional>
#include <iostream>

#include "coord.h"
#include "cubie.h"
#include "move.h"
#include "prun.h"
#include "sym.h"

inline void ok() { std::cout << "Ok." << std::endl; }
inline void error() { std::cout << "Error." << std::endl; }

void test_cubie() {
  std::cout << "Testing cubie level ..." << std::endl;
  cubie::cube c = cubie::SOLVED_CUBE;

  cubie::cube tmp1, tmp2;
  cubie::inv(c, tmp1);
  if (c != tmp1)
    error();
  tmp2 = c * tmp1;
  if (c != tmp2)
    error();

  cubie::shuffle(c);
  cubie::inv(c, tmp1);
  tmp2 = c * tmp1;
  if (tmp2 != cubie::SOLVED_CUBE)
    error();

  for (int i = 0; i < 100; i++) {
    cubie::shuffle(c);
    if (cubie::check(c) != 0)
      error();
  }
  cubie::shuffle(c);
  std::swap(c.c_prm[0], c.c_prm[1]);
  if (check(c) == 0)
    error();

  ok();
}

void test_getset(std::function<int(const cubie::cube&)> get_coord, std::function<void(cubie::cube&, int)> set_coord, int count) {
  cubie::cube c;
  for (int i = 0; i < count; i++) {
    set_coord(c, i);
    if (get_coord(c) != i)
      error();
  }
  ok();
}

void test_movecoord(std::array<uint16_t, move::COUNT>* move_coord, int n_coord, uint64_t moves = move::p1_mask | move::p2_mask) {
  for (int coord = 0; coord < n_coord; coord++) {
    for (; moves; moves &= moves - 1) {
      int m = std::countr_zero(moves);
      if (move_coord[move_coord[coord][m]][move::inv[m]] != coord)
        error();
      if (move_coord[move_coord[coord][move::inv[m]]][m] != coord)
        error();
    }
  }
  ok();
}

void test_coord() {
  std::cout << "Testing coord level ..." << std::endl;
  test_getset(coord::get_flip, coord::set_flip, coord::N_FLIP);
  test_getset(coord::get_twist, coord::set_twist, coord::N_TWIST);
  test_getset(coord::get_slice, coord::set_slice, coord::N_SLICE);
  test_getset(coord::get_u_edges, coord::set_u_edges, coord::N_U_EDGES);
  test_getset(coord::get_d_edges, coord::set_d_edges, coord::N_D_EDGES);
  test_getset(coord::get_corners, coord::set_corners, coord::N_CORNERS);

  test_getset(coord::get_slice1, coord::set_slice1, coord::N_SLICE1);
  test_getset(coord::get_ud_edges2, coord::set_ud_edges2, coord::N_UD_EDGES2);

  test_movecoord(coord::move_flip.data(), coord::N_FLIP);
  test_movecoord(coord::move_twist.data(), coord::N_TWIST);
  test_movecoord(coord::move_edges4.data(), coord::N_SLICE);
  test_movecoord(coord::move_corners.data(), coord::N_CORNERS);
  test_movecoord(coord::move_ud_edges2.data(), coord::N_UD_EDGES2, move::p2_mask);
}

void test_move() {
  std::cout << "Testing move level ..." << std::endl;

  cubie::cube c;
  for (int m = 0; m < move::COUNT; m++) {
    if (move::inv[move::inv[m]] != m)
      error();
    c = move::cubes[m] * move::cubes[move::inv[m]];
    if (c != cubie::SOLVED_CUBE)
      error();
    c = move::cubes[move::inv[m]] * move::cubes[m];
    if (c != cubie::SOLVED_CUBE)
      error();
  }
  ok();

  std::cout << "Phase 1: ";
  for (int m = 0; m < move::COUNT; m++) {
    if (move::in(m, move::p1_mask))
      std::cout << move::names[m] << " ";
  }
  std::cout << std::endl;
  std::cout << "Phase 2: ";
  for (int m = 0; m < move::COUNT; m++) {
    if (move::in(m, move::p2_mask))
      std::cout << move::names[m] << " ";
  }
  std::cout << std::endl;

  std::cout << "Forbidden:" << std::endl;
  for (int m = 0; m < move::COUNT; m++) {
    std::cout << move::names[m] << ": ";
    for (int m1 = 0; m1 < move::COUNT; m1++) {
      if (!move::in(m1, move::next[m]))
        std::cout << move::names[m1] << " ";
    }
    std::cout << std::endl;
  }

}

void test_conj(std::array<uint16_t, sym::COUNT_SUB>* conj_coord, int n_coord) {
  for (int coord = 0; coord < n_coord; coord++) {
    for (int s = 0; s < sym::COUNT_SUB; s++) {
      if (conj_coord[conj_coord[coord][s]][sym::inv[s]] != coord)
        error();
      if (conj_coord[conj_coord[coord][sym::inv[s]]][s] != coord)
        error();
    }
  }
  ok();
}

void test_sym() {
  std::cout << "Testing sym level ..." << std::endl;
  test_conj(sym::conj_twist.data(), coord::N_TWIST);
  test_conj(sym::conj_ud_edges2.data(), coord::N_UD_EDGES2);
}

void test_prun() {
  std::cout << "Testing pruning ..." << std::endl;

  srand(0);
  int n_moves = std::popcount(move::p1_mask);

  for (int i = 0; i < 1000; i++) {
    int flip = rand() % coord::N_FLIP;
    int slice = rand() % coord::N_SLICE;
    int twist = rand() % coord::N_TWIST;

    uint64_t next;
    uint64_t next1;
    uint64_t tmp;
    int togo = prun::get_phase1(flip, slice, twist, 100, tmp);

    int dist = prun::get_phase1(flip, slice, twist, togo, next);
    next &= move::p1_mask;

    next1 = 0;
    for (int m = 0; m < n_moves; m++) {
      int flip1 = coord::move_flip[flip][m];
      int slice1 = coord::move_edges4[slice][m];
      int twist1 = coord::move_twist[twist][m];
      if (prun::get_phase1(flip1, slice1, twist1, 100, tmp) < dist)
        next1 |= move::bit(m);
    }
    if (next1 != next)
      error();

    dist = prun::get_phase1(flip, slice, twist, togo + 1, next);
    next &= move::p1_mask;

    next1 = 0;
    for (int m = 0; m < n_moves; m++) {
      int flip1 = coord::move_flip[flip][m];
      int slice1 = coord::move_edges4[slice][m];
      int twist1 = coord::move_twist[twist][m];
      if (prun::get_phase1(flip1, slice1, twist1, 100, tmp) <= dist)
        next1 |= move::bit(m);
    }
    if (next1 != next)
      error();
  }

  ok();
}

bool check(const cubie::cube &c, const std::vector<int>& sol) {
  cubie::cube c1;
  cubie::cube c2;

  c1 = c;
  for (int m : sol) {
    c2 = c1 * move::cubes[m];
    std::swap(c1, c2);
  }

  return c1 == cubie::SOLVED_CUBE;
}

int main() {
  auto tick = std::chrono::high_resolution_clock::now();
  move::init();
  coord::init();
  sym::init();
  prun::init();
  std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tick).count() / 1000. << "ms" << std::endl;

  test_cubie();
  test_coord();
  test_move();
  test_sym();
  test_prun();

  return 0;
}

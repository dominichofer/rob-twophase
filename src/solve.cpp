#include "solve.h"

#include <algorithm>
#include <bit>
#include <cstring>
#include <thread>

#include "prun.h"
#include "sym.h"

namespace solve {

  class Search
  {
    int dir; // ID of search direction
    const coordc& cube; // starting position
    int p1_depth; // phase 1 search depth
    uint64_t d0_moves; // mask for initial moves to consider
    bool& done; // when to terminate the search
    int& len_limit; // only find strictly shorter solutions
    Engine& solver; // report solutions to

    /* Keep track of reconstructed edges that remain valid in the current search path */
    std::array<int, 50> u_edges;
    std::array<int, 50> d_edges;
    int edges_depth;

    std::array<int, 50> moves; // current (partial) solution

  private:
    // phase 1 search; iterates through all solution with exactly `togo` moves
    void phase1(int depth, int togo, int flip, int slice, int twist, int corners, uint64_t next);

    // phase 2 search; returns once any solution is found
    bool phase2(int depth, int togo, int slice, int ud_edges2, int corners, uint64_t next);

  public:
    Search(int dir, const coordc& cube, int p1_depth, uint64_t d0_moves, bool& done, int& len_limit, Engine& solver)
      : dir(dir), cube(cube), p1_depth(p1_depth), d0_moves(d0_moves), done(done), len_limit(len_limit), solver(solver)
    {};

    void run();
  };

  void Search::run() {
    u_edges[0] = cube.u_edges;
    d_edges[0] = cube.d_edges;
    edges_depth = 0;

    uint64_t next;
    prun::get_phase1(cube.flip, cube.slice, cube.twist, p1_depth, next);
    next &= move::p1_mask & d0_moves; // block B-moves in F5 mode here and select current search split
    phase1(0, p1_depth, cube.flip, cube.slice, cube.twist, cube.corners, next);
  }

  void Search::phase1(int depth, int togo, int flip, int slice, int twist, int corners, uint64_t next)
  {
    if (done)
      return;

    if (togo == 0) {
      int tmp = prun::get_precheck(corners, slice);
      if (tmp >= len_limit - depth) // phase 2 precheck, only reconstruct edges if successful
        return;

      for (int i = edges_depth + 1; i <= depth; i++) {
        u_edges[i] = coord::move_edges4[u_edges[i - 1]][moves[i - 1]];
        d_edges[i] = coord::move_edges4[d_edges[i - 1]][moves[i - 1]];
      }
      edges_depth = depth - 1;
      int ud_edges2 = coord::merge_ud_edges2(u_edges[depth], d_edges[depth]);

      int delta = 1;
      for (int togo1 = std::max(prun::get_phase2(corners, ud_edges2), tmp); togo1 < len_limit - depth; togo1 += delta) {
        if (phase2(depth, togo1, slice, ud_edges2, corners, move::p2_mask & move::next_p1p2[moves[depth - 1]]))
          return; // once we have found a phase 2 solution, there cannot be any shorter ones -> quit
      }
      return;
    }

    depth++;
    togo--;
    while (next) {
      int m = std::countr_zero(next);
      next &= next - 1;

      int flip1 = coord::move_flip[flip][m];
      int slice1 = coord::move_edges4[slice][m];
      int twist1 = coord::move_twist[twist][m];
      uint64_t next1;
      int dist1 = prun::get_phase1(flip1, slice1, twist1, togo, next1);

      // Check inside loop to avoid unnecessary recursion unwinds
      if (dist1 == togo || dist1 + togo >= 5) { // Rokicki optimization
        int corners1 = coord::move_corners[corners][m];
        moves[depth - 1] = m;

        next1 &= move::p1_mask & move::next[m];
        phase1(depth, togo, flip1, slice1, twist1, corners1, next1);
      }
    }

    // We always want to maintain the maximum number of already reconstructed EDGES coordinates, hence we only
    // decrement when the depth level gets lower than the current valid index (note that we will typically also
    // visit other deeper branches in between that might not have any effect on this)
    if (edges_depth == depth - 1)
      edges_depth--;
  }

  bool Search::phase2(int depth, int togo, int slice, int ud_edges2, int corners, uint64_t next)
  {
    if (togo == 0) {
      if (slice != coord::N_SLICE2 * coord::SLICE1_SOLVED) // check if SLICE2 is also solved
        return false;

      searchres sol = {std::vector<int>(depth), dir };
      for (int i = 0; i < depth; i++)
        sol.first[i] = moves[i];
      solver.report_sol(sol);

      return true; // we will not find any shorter solutions
    }

    while (next) {
      int m = std::countr_zero(next);
      next &= next - 1;

      int slice1 = coord::move_edges4[slice][m];
      int ud_edges21 = coord::move_ud_edges2[ud_edges2][m];
      int corners1 = coord::move_corners[corners][m];

      if (prun::get_phase2(corners1, ud_edges21) < togo) {
        moves[depth] = m;
        if (phase2(depth + 1, togo - 1, slice1, ud_edges21, corners1, move::p2_mask & move::next[m]))
          return true; // return as soon as we have a solution
      }
    }

    return false;
  }

  Engine::Engine(int n_threads, int tlim, int n_sols, int max_len, int n_splits)
    : n_threads(n_threads), tlim(tlim), n_sols(n_sols), max_len(max_len), n_splits(n_splits)
  {
    int tmp = (move::COUNT + n_splits - 1) / n_splits; // ceil to make sure that we always include all moves
    for (int i = 0; i < n_splits; i++)
      masks[i] = (uint64_t(1) << tmp) - 1 << tmp * i;
    done = true; // make sure that the first `prepare()` will actually do something
  }

  void Engine::thread() {
    int mindir = 0;
    do {
      /* Select next job to execute; don't forget to lock */
      job_mtx.lock();
      for (int dir = 0; dir < N_DIRS; dir++) {
        if (depths[dir] < depths[mindir])
          mindir = dir;
      }
      int split = splits[mindir]++;
      int togo = depths[mindir];
      if (splits[mindir] == n_splits) {
        depths[mindir]++;
        splits[mindir] = 0;
      }
      job_mtx.unlock();

      Search search(mindir, dirs[mindir], togo, masks[split], done, len_limit, *this);
      search.run();
    } while (!done); // we should never actually get to the truly optimal depth anyways in general
  }

  void Engine::prepare() {
    if (!done) // avoid double preparation
      return;
    finish();

    job_mtx.lock(); // make spawned threads wait for initialization of the cube to be solved
    for (int i = 0; i < n_threads; i++)
      threads.push_back(std::thread([&]() { this->thread(); }));

    done = false;
    len_limit = max_len > 0 ? max_len + 1: 50; // only search for strictly shorter solutions than this
    // `sols` is always emptied after a solve
  }

  void Engine::solve(const cubie::cube& c, std::vector<std::vector<int>>& res) {
    prepare(); // make sure we are prepared; will do nothing if that should already be the case

    cubie::cube tmp1, tmp2;
    cubie::cube invc = c.inverse();

    for (int dir = 0; dir < N_DIRS; dir++) {
      const cubie::cube& c1 = (dir & 1) ? invc : c; // reference is enough, we do not need to copy
      int rot = sym::ROT * (dir / 2);
      tmp1 = sym::cubes[sym::inv[rot]] * c1;
      tmp2 = tmp1 * sym::cubes[rot];

      dirs[dir].flip = coord::get_flip(tmp2);
      dirs[dir].slice = coord::get_slice(tmp2);
      dirs[dir].twist = coord::get_twist(tmp2);
      dirs[dir].u_edges = coord::get_u_edges(tmp2);
      dirs[dir].d_edges = coord::get_d_edges(tmp2);
      dirs[dir].corners = coord::get_corners(tmp2);

      uint64_t tmp; // simply ignore, makes no sense anyways without proper `togo`
      depths[dir] = prun::get_phase1(dirs[dir].flip, dirs[dir].slice, dirs[dir].twist, 100, tmp);
      splits[dir] = 0;
    }

    job_mtx.unlock(); // start solving

    { // timeout
      std::unique_lock<std::mutex> lock(tout_mtx);
      tout_cvar.wait_for(lock, std::chrono::milliseconds(tlim), [&]{ return done; });
      if (!done)
        done = true; // if we get here, this was a timeout
    }
    std::lock_guard<std::mutex> lock(sol_mtx); // make sure no thread is writing any more solutions

    res.resize(sols.size());
    for (int i = 0; i < res.size(); i++) {
      const searchres& sol = sols.top();
      res[i].resize(sol.first.size());

      int rot = sym::ROT * (sol.second / 2);
      for (int j = 0; j < res[i].size(); j++) // undo rotation
        res[i][j] = sym::conj_move[sol.first[j]][rot];
      if (sol.second & 1) { // undo inversion
        for (int j = 0; j < res[i].size(); j++)
          res[i][j] = move::inv[res[i][j]];
        std::reverse(res[i].begin(), res[i].end());
      }

      sols.pop();
    }
    std::reverse(res.begin(), res.end()); // return solutions in order of increasing length
  }

  void Engine::report_sol(searchres& sol) {
    std::lock_guard<std::mutex> lock(sol_mtx);

    if (done) // prevent any type of reporting after the solver has terminated (important for threading)
      return;

    sols.push(sol); // usually we only get here if we actually have a solution that will be added
    if (sols.size() > n_sols)
      sols.pop();
    if (sols.size() == n_sols) {
      len_limit = sols.top().first.size(); // only search for strictly shorter solutions

      if (len_limit <= max_len) { // already found a solution that is short enough
        done = true; // end searching
        // Wake up timeout
        std::lock_guard<std::mutex> lock(tout_mtx);
        tout_cvar.notify_one();
      }
    }
  }

  void Engine::finish() {
    for (std::thread& t : threads) // wait for all existing threads to actually finish
      t.join();
    threads.clear(); // they are now invalid
  }

}

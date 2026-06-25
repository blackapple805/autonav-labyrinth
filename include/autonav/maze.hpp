#pragma once
#include <vector>
#include <cstdint>
#include <random>
#include <algorithm>
#include "autonav/vec2.hpp"
#include "autonav/occupancy_grid.hpp"
#include "autonav/maze_photo_data.hpp"

// Maze maps for the navigation challenge.
//
// Two sources, both producing a fully-connected OccupancyGrid the A* planner
// can solve and the DWA controller can drive:
//
//   1. load_photo_maze()  — the EXACT Beaulieu hedge-maze from the reference
//      photograph, traced into the grid. Deterministic: same map every run.
//   2. generate_maze(seed) — a procedural perfect maze (recursive backtracker).
//      Fully reproducible from its integer seed, so a given seed is also an
//      "exact" maze for repeatable testing, while different seeds give variety.
//
// Both expose suggested start/goal poses that are guaranteed to sit in open
// space with clearance for the robot's radius.

namespace autonav {

struct MazeMap {
    OccupancyGrid grid;
    Vec2 start;
    Vec2 goal;
};

namespace detail {

// Distance (in cells) from each free cell to the nearest wall, via a simple
// two-pass chamfer transform. Used to pick start/goal cells with clearance.
inline std::vector<int> clearance_field(const OccupancyGrid& g) {
    int W = g.width(), H = g.height();
    const int INF = 1 << 29;
    std::vector<int> d(static_cast<size_t>(W) * H, 0);
    auto at = [&](int x, int y) -> int& { return d[static_cast<size_t>(y) * W + x]; };
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            at(x, y) = g.occupied(x, y) ? 0 : INF;
    // forward
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            if (at(x, y) == 0) continue;
            int best = at(x, y);
            if (x > 0) best = std::min(best, at(x - 1, y) + 1);
            if (y > 0) best = std::min(best, at(x, y - 1) + 1);
            at(x, y) = best;
        }
    // backward
    for (int y = H - 1; y >= 0; --y)
        for (int x = W - 1; x >= 0; --x) {
            if (at(x, y) == 0) continue;
            int best = at(x, y);
            if (x < W - 1) best = std::min(best, at(x + 1, y) + 1);
            if (y < H - 1) best = std::min(best, at(x, y + 1) + 1);
            at(x, y) = best;
        }
    return d;
}

}  // namespace detail

// ---- 1. Photo-traced maze ------------------------------------------------

inline MazeMap load_photo_maze() {
    const int W = kPhotoMazeW, H = kPhotoMazeH;
    OccupancyGrid grid(W, H, kPhotoMazeRes);

    // Row 0 of the data is the TOP of the image. World y points up, so map
    // data row r to grid row gy = H-1-r. This makes the rendered maze match
    // the orientation of the original photograph.
    for (int r = 0; r < H; ++r) {
        const char* row = kPhotoMazeRows[r];
        int gy = H - 1 - r;
        for (int gx = 0; gx < W; ++gx)
            if (row[gx] == '#')
                grid.set_occupied(gx, gy, true);
    }

    // Start and goal are precomputed (bottom-left / top-right corners, each on
    // an open cell with robot-radius clearance, verified A*-reachable through
    // the inflated map). Embedding them keeps the map deterministic.
    MazeMap m{std::move(grid), {}, {}};
    m.start = {kPhotoStartX, kPhotoStartY};
    m.goal  = {kPhotoGoalX,  kPhotoGoalY};
    return m;
}

// ---- 2. Seeded procedural maze -------------------------------------------

// Recursive-backtracker perfect maze on a coarse cell lattice, then scaled up
// into the fine occupancy grid so corridors are several cells wide (room for
// the robot). Deterministic for a given seed.
inline MazeMap generate_maze(uint32_t seed,
                             int cols = 13, int rows = 9,
                             int corridor = 11, int wall = 4) {
    // Fine grid dimensions derived from the lattice.
    int W = cols * (corridor + wall) + wall;
    int H = rows * (corridor + wall) + wall;
    const double res = 0.1;
    OccupancyGrid grid(W, H, res);

    // Start everything as wall, carve passages out.
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            grid.set_occupied(x, y, true);

    auto carve_cell = [&](int cx, int cy) {
        int x0 = wall + cx * (corridor + wall);
        int y0 = wall + cy * (corridor + wall);
        for (int y = y0; y < y0 + corridor; ++y)
            for (int x = x0; x < x0 + corridor; ++x)
                grid.set_occupied(x, y, false);
    };
    auto carve_wall_between = [&](int ax, int ay, int bx, int by) {
        int x0 = wall + ax * (corridor + wall);
        int y0 = wall + ay * (corridor + wall);
        if (bx > ax)      { for (int y = y0; y < y0 + corridor; ++y) for (int x = x0 + corridor; x < x0 + corridor + wall; ++x) grid.set_occupied(x, y, false); }
        else if (bx < ax) { for (int y = y0; y < y0 + corridor; ++y) for (int x = x0 - wall;     x < x0;                    ++x) grid.set_occupied(x, y, false); }
        else if (by > ay) { for (int x = x0; x < x0 + corridor; ++x) for (int y = y0 + corridor; y < y0 + corridor + wall; ++y) grid.set_occupied(x, y, false); }
        else              { for (int x = x0; x < x0 + corridor; ++x) for (int y = y0 - wall;     y < y0;                    ++y) grid.set_occupied(x, y, false); }
    };

    std::vector<uint8_t> visited(static_cast<size_t>(cols) * rows, 0);
    auto vis = [&](int x, int y) -> uint8_t& { return visited[static_cast<size_t>(y) * cols + x]; };

    std::mt19937 rng(seed);
    std::vector<std::pair<int,int>> stack;
    stack.push_back({0, 0});
    vis(0, 0) = 1;
    carve_cell(0, 0);

    while (!stack.empty()) {
        auto [cx, cy] = stack.back();
        int dirs[4] = {0, 1, 2, 3};
        std::shuffle(dirs, dirs + 4, rng);
        bool advanced = false;
        for (int i = 0; i < 4; ++i) {
            int nx = cx, ny = cy;
            if (dirs[i] == 0) nx++;
            else if (dirs[i] == 1) nx--;
            else if (dirs[i] == 2) ny++;
            else ny--;
            if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
            if (vis(nx, ny)) continue;
            vis(nx, ny) = 1;
            carve_cell(nx, ny);
            carve_wall_between(cx, cy, nx, ny);
            stack.push_back({nx, ny});
            advanced = true;
            break;
        }
        if (!advanced) stack.pop_back();
    }

    MazeMap m{std::move(grid), {}, {}};
    m.start = m.grid.grid_to_world(wall + corridor / 2, wall + corridor / 2);
    m.goal  = m.grid.grid_to_world(wall + (cols - 1) * (corridor + wall) + corridor / 2,
                                   wall + (rows - 1) * (corridor + wall) + corridor / 2);
    return m;
}

}  // namespace autonav

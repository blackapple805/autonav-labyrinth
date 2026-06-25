// Records a frontier-exploration run as JSON for the web visualizer: the true
// maze walls, the goal, the robot's trajectory, and periodic snapshots of the
// robot's OWN discovered map (so the page can show the fog of war lifting as
// the robot explores). The robot is given no map and no goal location.
#include <iostream>
#include <iomanip>
#include <vector>
#include "autonav/maze.hpp"
#include "autonav/frontier_explorer.hpp"
#include "autonav/lidar.hpp"

using namespace autonav;

int main() {
    MazeMap m = generate_maze(7, 20, 14, 9, 3);
    const int W = m.grid.width(), H = m.grid.height();
    const double res = m.grid.resolution();

    FrontierExplorer explorer(W, H, res, 4.0);
    DiffDrive robot({m.start, 0.0});
    Lidar lidar(64, 2.0 * kPi, 4.0);

    // We export the map at a coarser cell size to keep the file small: group
    // the fine grid into blocks and report each block's known-state.
    const int block = 3;  // 3x3 fine cells per exported map cell
    const int MW = (W + block - 1) / block;
    const int MH = (H + block - 1) / block;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "{\n";
    std::cout << "  \"world\": {\"w\": " << W * res << ", \"h\": " << H * res
              << ", \"res\": " << res << "},\n";
    std::cout << "  \"map\": {\"cols\": " << MW << ", \"rows\": " << MH
              << ", \"cell\": " << block * res << "},\n";
    std::cout << "  \"start\": [" << m.start.x << "," << m.start.y << "],\n";
    std::cout << "  \"goal\": [" << m.goal.x << "," << m.goal.y << "],\n";

    // True maze walls (for an optional faint ground-truth underlay), as coarse
    // blocks: a block is a wall if any fine cell in it is occupied.
    std::cout << "  \"walls\": [";
    bool first = true;
    for (int my = 0; my < MH; ++my)
        for (int mx = 0; mx < MW; ++mx) {
            bool wall = false;
            for (int dy = 0; dy < block && !wall; ++dy)
                for (int dx = 0; dx < block && !wall; ++dx) {
                    int gx = mx * block + dx, gy = my * block + dy;
                    if (gx < W && gy < H && m.grid.occupied(gx, gy)) wall = true;
                }
            if (wall) {
                if (!first) std::cout << ",";
                first = false;
                std::cout << "[" << mx << "," << my << "]";
            }
        }
    std::cout << "],\n";

    // Step the sim, recording a frame every few ticks: robot pose plus the
    // newly-known coarse cells since the last frame (to keep the file compact,
    // we emit deltas: which blocks became known this frame and their state).
    std::cout << "  \"frames\": [\n";
    const double dt = 0.05;
    int ticks = 0;
    bool found = false;
    std::vector<uint8_t> exported(static_cast<size_t>(MW) * MH, 0);  // 0 unknown
    bool firstFrame = true;

    auto block_state = [&](int mx, int my) -> int {
        // 2 = wall if any known-wall in block, else 1 = free if any known-free,
        // else 0 = still unknown.
        bool wall = false, free = false;
        for (int dy = 0; dy < block; ++dy)
            for (int dx = 0; dx < block; ++dx) {
                int gx = mx * block + dx, gy = my * block + dy;
                if (gx >= W || gy >= H) continue;
                auto c = explorer.map().at(gx, gy);
                if (c == KnownMap::Wall) wall = true;
                else if (c == KnownMap::Free) free = true;
            }
        return wall ? 2 : (free ? 1 : 0);
    };

    while (ticks < 20000) {
        auto scan = lidar.scan(robot.pose(), m.grid);
        auto cmd = explorer.step(robot.pose(), scan);
        robot.step(cmd.v, cmd.w, dt);

        if (ticks % 6 == 0) {  // ~3.3 Hz frames
            if (!firstFrame) std::cout << ",\n";
            firstFrame = false;
            const Pose& p = robot.pose();
            std::cout << "    {\"x\":" << p.pos.x << ",\"y\":" << p.pos.y
                      << ",\"t\":" << p.theta << ",\"reveal\":[";
            // Emit blocks whose state changed since last export.
            bool rf = true;
            for (int my = 0; my < MH; ++my)
                for (int mx = 0; mx < MW; ++mx) {
                    int s = block_state(mx, my);
                    size_t idx = static_cast<size_t>(my) * MW + mx;
                    if (s != 0 && exported[idx] != s) {
                        exported[idx] = static_cast<uint8_t>(s);
                        if (!rf) std::cout << ",";
                        rf = false;
                        std::cout << "[" << mx << "," << my << "," << s << "]";
                    }
                }
            std::cout << "]}";
        }

        if (explorer.goal_found(m.goal)) { found = true; break; }
        if (cmd.v == 0.0 && cmd.w == 0.0) break;
        ++ticks;
    }
    std::cout << "\n  ],\n";
    std::cout << "  \"found\": " << (found ? "true" : "false") << ",\n";
    std::cout << "  \"explore_time\": " << ticks * dt << "\n";
    std::cout << "}\n";

    std::cerr << "exploration " << (found ? "FOUND goal" : "ended")
              << " in " << ticks * dt << "s\n";
    return 0;
}

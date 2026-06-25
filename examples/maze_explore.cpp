// Frontier-exploration demo: the robot solves a procedurally generated maze
// with NO prior map and NO knowledge of where the goal is. It builds its own
// map from lidar and explores until it discovers the exit.
//
// Build:  make maze_explore
// Run:    ./build/maze_explore
#include <iostream>
#include <iomanip>
#include "autonav/maze.hpp"
#include "autonav/frontier_explorer.hpp"
#include "autonav/lidar.hpp"

using namespace autonav;

int main() {
    // A clean grid maze (recursive backtracker) in the style of a real hedge
    // maze: uniform corridors, a single solution, dense branching.
    MazeMap m = generate_maze(7, 20, 14, 9, 3);

    FrontierExplorer explorer(m.grid.width(), m.grid.height(),
                              m.grid.resolution(), 4.0);
    DiffDrive robot({m.start, 0.0});
    Lidar lidar(64, 2.0 * kPi, 4.0);  // 360-degree scan for mapping

    std::cout << "Maze " << m.grid.width() << "x" << m.grid.height()
              << "  start (" << m.start.x << ", " << m.start.y << ")"
              << "  goal (" << m.goal.x << ", " << m.goal.y << ")\n";
    std::cout << "Robot has NO map and does not know where the goal is.\n\n";

    const double dt = 0.05;
    const int total_cells = m.grid.width() * m.grid.height();
    int ticks = 0;
    bool found = false;

    while (ticks < 20000) {
        auto scan = lidar.scan(robot.pose(), m.grid);
        auto cmd = explorer.step(robot.pose(), scan);
        robot.step(cmd.v, cmd.w, dt);

        if (explorer.goal_found(m.goal)) { found = true; break; }
        if (cmd.v == 0.0 && cmd.w == 0.0) break;  // fully explored, no goal

        if (ticks % 1000 == 0) {
            double pct = 100.0 * explorer.map().known_count() / total_cells;
            std::cout << std::fixed << std::setprecision(0)
                      << "  t=" << ticks * dt << "s  explored "
                      << pct << "% of the maze\n";
        }
        ++ticks;
    }

    std::cout << "\n";
    if (found) {
        std::cout << std::fixed << std::setprecision(1)
                  << "Goal discovered after " << ticks * dt
                  << "s of exploration.\n";
    } else {
        std::cout << "Exploration ended without finding the goal.\n";
    }
    return 0;
}

// Console demo: runs the autonomy stack and prints the robot navigating an
// office-like map to the goal, with a final ASCII rendering of its path.
//
// Build:  make
// Run:    ./build/autonav_demo
#include <iostream>
#include <iomanip>
#include <vector>
#include "autonav/simulator.hpp"

using namespace autonav;

static OccupancyGrid build_office_map() {
    // 20m x 14m world at 0.1m resolution -> 200 x 140 cells.
    OccupancyGrid grid(200, 140, 0.1);
    // Outer walls.
    grid.add_box({0, 0}, {20, 0.2});
    grid.add_box({0, 13.8}, {20, 14});
    grid.add_box({0, 0}, {0.2, 14});
    grid.add_box({19.8, 0}, {20, 14});
    // Interior obstacles: shelves / desks forming a small maze.
    grid.add_box({5, 0}, {5.4, 9});
    grid.add_box({9, 5}, {9.4, 14});
    grid.add_box({13, 0}, {13.4, 9});
    grid.add_box({13, 8.6}, {17, 9});
    return grid;
}

int main() {
    OccupancyGrid grid = build_office_map();
    Pose start{{1.5, 1.5}, 0.0};
    Vec2 goal{18.0, 12.0};

    Simulator sim(grid, start, goal);
    if (!sim.has_path()) {
        std::cerr << "No path found to goal.\n";
        return 1;
    }

    std::cout << "Planned path waypoints: " << sim.path().size() << "\n";

    const double dt = 0.05;  // 20 Hz control loop
    int ticks = 0;
    std::vector<Vec2> trace;
    while (sim.step(dt)) {
        trace.push_back(sim.pose().pos);
        if (++ticks > 4000) { std::cerr << "Timeout.\n"; break; }
    }

    const Pose& end = sim.pose();
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Reached (" << end.pos.x << ", " << end.pos.y << ") in "
              << ticks * dt << " s of sim time.\n\n";

    // ASCII map: '#' obstacle, '.' free, '*' path traced by the robot,
    // 'S' start, 'G' goal.
    const int cols = 80, rows = 28;
    std::vector<std::string> canvas(rows, std::string(cols, ' '));
    auto plot = [&](const Vec2& w, char c) {
        int cx = static_cast<int>(w.x / 20.0 * (cols - 1));
        int cy = static_cast<int>(w.y / 14.0 * (rows - 1));
        if (cx >= 0 && cx < cols && cy >= 0 && cy < rows)
            canvas[rows - 1 - cy][cx] = c;
    };
    for (int gy = 0; gy < grid.height(); ++gy)
        for (int gx = 0; gx < grid.width(); ++gx)
            if (grid.occupied(gx, gy)) plot(grid.grid_to_world(gx, gy), '#');
    for (const auto& p : trace) plot(p, '*');
    plot(start.pos, 'S');
    plot(goal, 'G');

    for (const auto& line : canvas) std::cout << line << "\n";
    return 0;
}

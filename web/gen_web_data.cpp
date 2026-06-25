// Runs the same simulation as the console demo but serializes every frame
// (pose + lidar scan) to JSON on stdout, so the browser visualizer can
// replay the run with no C++ runtime in the browser. The brain stays in C++.
#include <iostream>
#include <iomanip>
#include <vector>
#include "autonav/simulator.hpp"

using namespace autonav;

static OccupancyGrid build_office_map() {
    OccupancyGrid grid(200, 140, 0.1);
    grid.add_box({0, 0}, {20, 0.2});
    grid.add_box({0, 13.8}, {20, 14});
    grid.add_box({0, 0}, {0.2, 14});
    grid.add_box({19.8, 0}, {20, 14});
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

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "{\n";
    std::cout << "  \"world\": {\"w\": 20.0, \"h\": 14.0, \"res\": 0.1},\n";

    // Obstacle cells (run-length-free, just list occupied cell rects coarsely).
    std::cout << "  \"obstacles\": [";
    bool first = true;
    for (int gy = 0; gy < grid.height(); ++gy)
        for (int gx = 0; gx < grid.width(); ++gx)
            if (grid.occupied(gx, gy)) {
                if (!first) std::cout << ",";
                first = false;
                Vec2 w = grid.grid_to_world(gx, gy);
                std::cout << "[" << w.x << "," << w.y << "]";
            }
    std::cout << "],\n";

    // Planned global path.
    std::cout << "  \"path\": [";
    for (size_t i = 0; i < sim.path().size(); ++i) {
        if (i) std::cout << ",";
        std::cout << "[" << sim.path()[i].x << "," << sim.path()[i].y << "]";
    }
    std::cout << "],\n";
    std::cout << "  \"goal\": [" << goal.x << "," << goal.y << "],\n";

    // Per-frame trajectory + lidar.
    std::cout << "  \"frames\": [\n";
    const double dt = 0.05;
    int ticks = 0;
    bool firstFrame = true;
    while (sim.step(dt) && ticks < 4000) {
        if (ticks % 2 == 0) {  // downsample to ~10 Hz for a lighter file
            if (!firstFrame) std::cout << ",\n";
            firstFrame = false;
            const Pose& p = sim.pose();
            std::cout << "    {\"x\":" << p.pos.x << ",\"y\":" << p.pos.y
                      << ",\"t\":" << p.theta << ",\"scan\":[";
            const auto& scan = sim.last_scan();
            for (size_t i = 0; i < scan.size(); ++i) {
                if (i) std::cout << ",";
                std::cout << "[" << scan[i].angle << ","
                          << scan[i].range << ","
                          << (scan[i].hit ? 1 : 0) << "]";
            }
            std::cout << "]}";
        }
        ++ticks;
    }
    std::cout << "\n  ]\n}\n";
    return 0;
}

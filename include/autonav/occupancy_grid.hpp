#pragma once
#include <vector>
#include <cstdint>
#include "autonav/vec2.hpp"

namespace autonav {

// An occupancy grid: the standard world representation in mobile robotics.
// Each cell is either free (false) or occupied (true). Real robots build
// this from sensor data via SLAM; here we treat it as the known map for
// planning, while the lidar model re-discovers obstacles for local avoidance.
class OccupancyGrid {
public:
    OccupancyGrid(int width, int height, double resolution)
        : width_(width), height_(height), resolution_(resolution),
          cells_(static_cast<size_t>(width) * height, false) {}

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }

    bool in_bounds(int gx, int gy) const {
        return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
    }

    bool occupied(int gx, int gy) const {
        if (!in_bounds(gx, gy)) return true;  // out of bounds == wall
        return cells_[index(gx, gy)];
    }

    void set_occupied(int gx, int gy, bool v) {
        if (in_bounds(gx, gy)) cells_[index(gx, gy)] = v;
    }

    // Stamp a filled rectangle of obstacle (world coordinates).
    void add_box(const Vec2& min_w, const Vec2& max_w) {
        int x0, y0, x1, y1;
        world_to_grid(min_w, x0, y0);
        world_to_grid(max_w, x1, y1);
        for (int gy = y0; gy <= y1; ++gy)
            for (int gx = x0; gx <= x1; ++gx)
                set_occupied(gx, gy, true);
    }

    void world_to_grid(const Vec2& w, int& gx, int& gy) const {
        gx = static_cast<int>(w.x / resolution_);
        gy = static_cast<int>(w.y / resolution_);
    }

    Vec2 grid_to_world(int gx, int gy) const {
        return {(gx + 0.5) * resolution_, (gy + 0.5) * resolution_};
    }

    // Is the straight segment a->b clear? Uses supersampled stepping along
    // the ray. Used by the lidar model and by path validation.
    bool segment_clear(const Vec2& a, const Vec2& b) const {
        double d = a.dist(b);
        int steps = static_cast<int>(d / (resolution_ * 0.5)) + 1;
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            Vec2 p = a + (b - a) * t;
            int gx, gy;
            world_to_grid(p, gx, gy);
            if (occupied(gx, gy)) return false;
        }
        return true;
    }

    // Return a copy with every obstacle grown by `cells` in each direction.
    // This is "configuration-space inflation": by padding obstacles by the
    // robot's radius before planning, the planner treats the robot as a point
    // yet still produces a route with real clearance. Standard practice in
    // every production navigation stack (ROS calls it the inflation layer).
    OccupancyGrid inflated(int cells) const {
        OccupancyGrid out(width_, height_, resolution_);
        for (int y = 0; y < height_; ++y)
            for (int x = 0; x < width_; ++x)
                if (occupied(x, y))
                    for (int dy = -cells; dy <= cells; ++dy)
                        for (int dx = -cells; dx <= cells; ++dx)
                            out.set_occupied(x + dx, y + dy, true);
        return out;
    }

private:
    size_t index(int gx, int gy) const {
        return static_cast<size_t>(gy) * width_ + gx;
    }

    int width_, height_;
    double resolution_;
    std::vector<bool> cells_;
};

}  // namespace autonav

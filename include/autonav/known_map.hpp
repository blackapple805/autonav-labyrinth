#pragma once
#include <vector>
#include <cstdint>
#include <cmath>
#include "autonav/vec2.hpp"
#include "autonav/occupancy_grid.hpp"
#include "autonav/diff_drive.hpp"
#include "autonav/lidar.hpp"

namespace autonav {

// The robot's OWN evolving belief about the maze. Unlike OccupancyGrid (the
// ground-truth world the simulator owns), this starts entirely Unknown and is
// filled in only by what the robot's lidar has actually seen. Frontier-based
// exploration plans over THIS map, never the real one, so the robot genuinely
// discovers the maze rather than being handed it.
class KnownMap {
public:
    enum Cell : uint8_t { Unknown = 0, Free = 1, Wall = 2 };

    KnownMap(int width, int height, double resolution)
        : w_(width), h_(height), res_(resolution),
          cells_(static_cast<size_t>(width) * height, Unknown) {}

    int width() const { return w_; }
    int height() const { return h_; }
    double resolution() const { return res_; }

    bool in_bounds(int gx, int gy) const {
        return gx >= 0 && gx < w_ && gy >= 0 && gy < h_;
    }
    Cell at(int gx, int gy) const {
        if (!in_bounds(gx, gy)) return Wall;
        return cells_[idx(gx, gy)];
    }
    void set(int gx, int gy, Cell c) {
        if (in_bounds(gx, gy)) cells_[idx(gx, gy)] = c;
    }

    void world_to_grid(const Vec2& wld, int& gx, int& gy) const {
        gx = static_cast<int>(wld.x / res_);
        gy = static_cast<int>(wld.y / res_);
    }
    Vec2 grid_to_world(int gx, int gy) const {
        return {(gx + 0.5) * res_, (gy + 0.5) * res_};
    }

    // Integrate one lidar scan taken at `pose`. Each beam marks every cell it
    // passes through as Free up to its hit point, and the hit cell as Wall.
    // This is a simple ray-cast inverse sensor model -- exactly how a real
    // robot turns range readings into an occupancy map.
    void integrate(const Pose& pose, const std::vector<LidarReturn>& scan,
                   double max_range) {
        for (const auto& r : scan) {
            double a = pose.theta + r.angle;
            double reach = r.hit ? r.range : max_range;
            Vec2 dir{std::cos(a), std::sin(a)};
            // Walk the ray in small steps, marking free space.
            double step = res_ * 0.5;
            for (double d = 0.0; d < reach; d += step) {
                Vec2 p = pose.pos + dir * d;
                int gx, gy;
                world_to_grid(p, gx, gy);
                if (in_bounds(gx, gy) && cells_[idx(gx, gy)] == Unknown)
                    cells_[idx(gx, gy)] = Free;
            }
            if (r.hit) {
                Vec2 p = pose.pos + dir * r.range;
                int gx, gy;
                world_to_grid(p, gx, gy);
                set(gx, gy, Wall);  // the obstacle that stopped the beam
            }
        }
    }

    // Count of cells discovered (Free or Wall), for progress reporting.
    int known_count() const {
        int n = 0;
        for (auto c : cells_) if (c != Unknown) ++n;
        return n;
    }

private:
    size_t idx(int gx, int gy) const {
        return static_cast<size_t>(gy) * w_ + gx;
    }
    int w_, h_;
    double res_;
    std::vector<Cell> cells_;
};

}  // namespace autonav

#pragma once
#include <vector>
#include "autonav/vec2.hpp"
#include "autonav/occupancy_grid.hpp"
#include "autonav/diff_drive.hpp"

namespace autonav {

// A single lidar return: angle relative to robot heading, and measured range.
struct LidarReturn {
    double angle;  // radians, robot-relative
    double range;  // meters; == max_range if nothing was hit
    bool hit;      // true if a real obstacle was detected
};

// Simulated 2D lidar (planar laser scanner). This is the workhorse sensor
// of indoor autonomy. We cast N evenly spaced rays across the field of view
// and find the first occupied cell along each. Real lidars add noise; this
// one is clean, which keeps the controller logic the focus of the demo.
class Lidar {
public:
    Lidar(int num_beams, double fov, double max_range)
        : num_beams_(num_beams), fov_(fov), max_range_(max_range) {}

    std::vector<LidarReturn> scan(const Pose& pose,
                                  const OccupancyGrid& grid) const {
        std::vector<LidarReturn> returns;
        returns.reserve(num_beams_);

        double start = -fov_ / 2.0;
        double step = (num_beams_ > 1) ? fov_ / (num_beams_ - 1) : 0.0;

        for (int i = 0; i < num_beams_; ++i) {
            double rel = start + step * i;
            double world_angle = pose.theta + rel;
            Vec2 dir{std::cos(world_angle), std::sin(world_angle)};

            LidarReturn r{rel, max_range_, false};
            // March along the ray until we hit something or run out of range.
            double res = grid.resolution() * 0.5;
            for (double d = 0.0; d <= max_range_; d += res) {
                Vec2 p = pose.pos + dir * d;
                int gx, gy;
                grid.world_to_grid(p, gx, gy);
                if (grid.occupied(gx, gy)) {
                    r.range = d;
                    r.hit = true;
                    break;
                }
            }
            returns.push_back(r);
        }
        return returns;
    }

    double max_range() const { return max_range_; }

private:
    int num_beams_;
    double fov_;
    double max_range_;
};

}  // namespace autonav

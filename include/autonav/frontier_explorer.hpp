#pragma once
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include "autonav/vec2.hpp"
#include "autonav/known_map.hpp"
#include "autonav/diff_drive.hpp"
#include "autonav/lidar.hpp"

namespace autonav {

// Frontier-based exploration: the standard algorithm for a robot mapping an
// unknown space. The robot has NO map and does NOT know where the goal is. Each
// cycle it:
//   1. Senses (lidar) and updates its KnownMap.
//   2. Finds frontiers -- known-free cells that border Unknown space. These are
//      the edges of the explored region, i.e. where new information lives.
//   3. Picks the nearest reachable frontier (BFS over known-free cells).
//   4. Drives toward it, sensing as it goes.
// It stops when the goal cell enters the known-free region (the robot has
// "seen" the exit). Because planning happens entirely over what the robot has
// discovered, it cannot beeline to the goal -- it must explore to find it.
//
// This is the honest "solve the maze by trial and error" the project was after.
class FrontierExplorer {
public:
    struct Command { double v; double w; };

    FrontierExplorer(int w, int h, double res, double max_range)
        : map_(w, h, res), max_range_(max_range) {}

    const KnownMap& map() const { return map_; }

    // Has the robot discovered the goal cell yet?
    bool goal_found(const Vec2& goal) const {
        int gx, gy;
        map_.world_to_grid(goal, gx, gy);
        return map_.at(gx, gy) == KnownMap::Free;
    }

    // One control step. Returns a velocity command. The robot senses, updates
    // its map, (re)selects a frontier target if needed, and steers toward it.
    Command step(const Pose& pose, const std::vector<LidarReturn>& scan) {
        map_.integrate(pose, scan, max_range_);

        // If we have no current target, or we've reached it, pick a new one.
        int rgx, rgy;
        map_.world_to_grid(pose.pos, rgx, rgy);
        bool need_target = !have_target_ ||
                           pose.pos.dist(target_) < replan_radius_;
        if (need_target) {
            if (!select_frontier(rgx, rgy)) {
                have_target_ = false;
                return {0.0, 0.0};  // nothing left to explore
            }
        }

        // Steer toward the current frontier target, slowing near obstacles seen
        // dead ahead so the robot doesn't scrape hedges while exploring.
        Vec2 to = target_ - pose.pos;
        double desired = std::atan2(to.y, to.x);
        double err = wrap_angle(desired - pose.theta);
        double w = std::clamp(2.5 * err, -2.2, 2.2);

        double front = front_clearance(scan);
        double v = forward_speed_ * std::max(0.2, 1.0 - std::fabs(err) / kPi);
        if (front < 0.6) v *= std::max(0.15, front / 0.6);
        return {v, w};
    }

private:
    // Find frontiers and choose the nearest reachable one by BFS over known-
    // free cells from the robot's position. Returns false if no frontier
    // exists (map fully explored).
    bool select_frontier(int rgx, int rgy) {
        const int W = map_.width(), H = map_.height();
        std::vector<int> dist(static_cast<size_t>(W) * H, -1);
        auto id = [W](int x, int y) { return y * W + x; };

        std::queue<std::pair<int,int>> q;
        // Seed BFS from the robot's cell (or nearest free cell if it's on a
        // boundary). We expand only through known-free cells -- the robot can
        // only plan through space it has confirmed is clear.
        if (map_.at(rgx, rgy) != KnownMap::Free) {
            // Snap to a nearby free cell.
            bool found = false;
            for (int rad = 1; rad <= 5 && !found; ++rad)
                for (int dy = -rad; dy <= rad && !found; ++dy)
                    for (int dx = -rad; dx <= rad && !found; ++dx)
                        if (map_.at(rgx + dx, rgy + dy) == KnownMap::Free) {
                            rgx += dx; rgy += dy; found = true;
                        }
            if (!found) return false;
        }

        dist[id(rgx, rgy)] = 0;
        q.push({rgx, rgy});
        const int dx4[4] = {1, -1, 0, 0};
        const int dy4[4] = {0, 0, 1, -1};

        int best_d = 1 << 29;
        int best_x = -1, best_y = -1;

        while (!q.empty()) {
            auto [cx, cy] = q.front(); q.pop();
            int d = dist[id(cx, cy)];

            // Is this cell a frontier? (free and adjacent to unknown)
            if (is_frontier(cx, cy)) {
                // Prefer the nearest frontier with a small bias toward larger
                // open frontiers so the robot heads for real openings.
                if (d < best_d) { best_d = d; best_x = cx; best_y = cy; }
            }

            for (int k = 0; k < 4; ++k) {
                int nx = cx + dx4[k], ny = cy + dy4[k];
                if (!map_.in_bounds(nx, ny)) continue;
                if (dist[id(nx, ny)] != -1) continue;
                if (map_.at(nx, ny) != KnownMap::Free) continue;  // only known-free
                dist[id(nx, ny)] = d + 1;
                q.push({nx, ny});
            }
        }

        if (best_x < 0) return false;
        target_ = map_.grid_to_world(best_x, best_y);
        have_target_ = true;
        return true;
    }

    // A frontier cell is known-free and orthogonally adjacent to Unknown space.
    bool is_frontier(int x, int y) const {
        if (map_.at(x, y) != KnownMap::Free) return false;
        const int dx4[4] = {1, -1, 0, 0};
        const int dy4[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4; ++k)
            if (map_.at(x + dx4[k], y + dy4[k]) == KnownMap::Unknown)
                return true;
        return false;
    }

    static double front_clearance(const std::vector<LidarReturn>& scan) {
        double m = 1e9;
        for (const auto& r : scan)
            if (std::fabs(r.angle) < 0.4 && r.hit) m = std::min(m, r.range);
        return (m < 1e8) ? m : 4.0;
    }

    KnownMap map_;
    double max_range_;
    Vec2 target_;
    bool have_target_ = false;
    double replan_radius_ = 0.5;   // re-pick a frontier within this of target
    double forward_speed_ = 1.0;
};

}  // namespace autonav

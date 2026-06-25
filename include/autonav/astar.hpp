#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include "autonav/vec2.hpp"
#include "autonav/occupancy_grid.hpp"

namespace autonav {

// A* global path planner on the occupancy grid. A* is the standard global
// planner in robotics navigation stacks (it is what ROS move_base and
// countless game/robot systems use under the hood): optimal, complete, and
// fast when the heuristic is admissible. We use octile distance, the correct
// heuristic for 8-connected grids.
class AStarPlanner {
public:
    explicit AStarPlanner(const OccupancyGrid& grid) : grid_(grid) {}

    // Returns a list of world-space waypoints from start to goal, or an empty
    // vector if no path exists. The raw grid path is then string-pulled into
    // a shorter set of waypoints the controller can follow smoothly.
    // When smooth_path is false, the raw 8-connected grid path is returned
    // (one waypoint per cell). The dense path hugs the corridor and is what a
    // tight-clearance path follower should track; string-pulling is great for
    // open space but cuts corners that matter in a narrow maze.
    std::vector<Vec2> plan(const Vec2& start_w, const Vec2& goal_w,
                           bool smooth_path = true) const {
        int sx, sy, gx, gy;
        grid_.world_to_grid(start_w, sx, sy);
        grid_.world_to_grid(goal_w, gx, gy);

        if (grid_.occupied(sx, sy) || grid_.occupied(gx, gy))
            return {};

        const int W = grid_.width();
        auto id = [W](int x, int y) { return y * W + x; };

        std::priority_queue<Node, std::vector<Node>, std::greater<>> open;
        std::unordered_map<int, double> g_cost;
        std::unordered_map<int, int> came_from;

        open.push({sx, sy, 0.0, heuristic(sx, sy, gx, gy)});
        g_cost[id(sx, sy)] = 0.0;

        // 8-connected neighborhood with correct diagonal costs.
        const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        bool found = false;
        while (!open.empty()) {
            Node cur = open.top();
            open.pop();

            if (cur.x == gx && cur.y == gy) { found = true; break; }

            int cur_id = id(cur.x, cur.y);
            // Skip stale queue entries (we never decrease-key; we re-push).
            if (cur.g > g_cost[cur_id] + 1e-9) continue;

            for (int k = 0; k < 8; ++k) {
                int nx = cur.x + dx[k];
                int ny = cur.y + dy[k];
                if (grid_.occupied(nx, ny)) continue;

                // Prevent corner-cutting through diagonally-adjacent walls.
                if (dx[k] != 0 && dy[k] != 0) {
                    if (grid_.occupied(cur.x + dx[k], cur.y) &&
                        grid_.occupied(cur.x, cur.y + dy[k]))
                        continue;
                }

                double step = (dx[k] != 0 && dy[k] != 0) ? 1.41421356 : 1.0;
                double ng = cur.g + step;
                int nid = id(nx, ny);

                auto it = g_cost.find(nid);
                if (it == g_cost.end() || ng < it->second) {
                    g_cost[nid] = ng;
                    came_from[nid] = cur_id;
                    open.push({nx, ny, ng, ng + heuristic(nx, ny, gx, gy)});
                }
            }
        }

        if (!found) return {};

        // Reconstruct grid path from goal back to start.
        std::vector<Vec2> path;
        int cur = id(gx, gy);
        int start_id = id(sx, sy);
        std::vector<int> rev;
        while (cur != start_id) {
            rev.push_back(cur);
            cur = came_from[cur];
        }
        rev.push_back(start_id);
        std::reverse(rev.begin(), rev.end());

        for (int node_id : rev) {
            int x = node_id % W;
            int y = node_id / W;
            path.push_back(grid_.grid_to_world(x, y));
        }

        return smooth_path ? smooth(path) : path;
    }

private:
    struct Node {
        int x, y;
        double g, f;
        bool operator>(const Node& o) const { return f > o.f; }
    };

    // Octile distance: the admissible heuristic for 8-connected grids.
    static double heuristic(int x0, int y0, int x1, int y1) {
        double dx = std::abs(x1 - x0);
        double dy = std::abs(y1 - y0);
        return (dx + dy) + (1.41421356 - 2.0) * std::min(dx, dy);
    }

    // String-pulling: collapse runs of collinear-ish waypoints by keeping a
    // point only when the line of sight from the last kept point is broken.
    // This turns a jagged grid path into a handful of clean waypoints.
    std::vector<Vec2> smooth(const std::vector<Vec2>& path) const {
        if (path.size() <= 2) return path;
        std::vector<Vec2> out;
        out.push_back(path.front());
        size_t anchor = 0;
        for (size_t i = 2; i < path.size(); ++i) {
            if (!grid_.segment_clear(path[anchor], path[i])) {
                out.push_back(path[i - 1]);
                anchor = i - 1;
            }
        }
        out.push_back(path.back());
        return out;
    }

    const OccupancyGrid& grid_;
};

}  // namespace autonav

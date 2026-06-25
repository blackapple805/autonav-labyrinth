#pragma once
#include <vector>
#include "autonav/vec2.hpp"
#include "autonav/occupancy_grid.hpp"
#include "autonav/diff_drive.hpp"
#include "autonav/lidar.hpp"
#include "autonav/astar.hpp"
#include "autonav/dwa_planner.hpp"

namespace autonav {

// Ties the whole autonomy stack into one fixed-timestep loop:
//   plan (A*) -> sense (lidar) -> decide (DWA) -> act (kinematics).
// This mirrors the sense-plan-act architecture every real robot runs. The
// global planner (A*) finds a route; the local planner (DWA) follows it while
// guaranteeing collision-free motion against what the lidar currently sees.
class Simulator {
public:
    Simulator(OccupancyGrid grid, const Pose& start, const Vec2& goal)
        : Simulator(std::move(grid), start, goal,
                    DWAPlanner::Limits{}, DWAPlanner::Weights{}) {}

    // Overload taking explicit DWA tuning, used by the evolutionary runner so
    // each generation drives with a different (possibly bad) parameter set.
    // `inflate_radius_m` sets the configuration-space padding used for global
    // planning; it should be about the robot's footprint radius. Tight mazes
    // need a smaller value than open offices, hence it is a parameter.
    Simulator(OccupancyGrid grid, const Pose& start, const Vec2& goal,
              const DWAPlanner::Limits& limits, const DWAPlanner::Weights& weights,
              double inflate_radius_m = 0.5)
        : grid_(std::move(grid)),
          robot_(start),
          goal_(goal),
          lidar_(72, 2.0 * kPi * 0.75, 4.0),  // 72 beams, 270deg FOV, 4m
          controller_(limits, weights) {
        // Plan on an inflated copy of the map so the global route keeps the
        // robot's radius of clearance from walls. This keeps the local planner
        // (DWA) from having to fight a path that hugs obstacles, which is the
        // main cause of corner oscillation and local-minimum traps.
        int inflate_cells = static_cast<int>(inflate_radius_m / grid_.resolution());
        OccupancyGrid planning_grid = grid_.inflated(inflate_cells);
        AStarPlanner planner(planning_grid);
        path_ = planner.plan(start.pos, goal);
        controller_.set_path(path_);
    }

    bool has_path() const { return !path_.empty(); }
    const std::vector<Vec2>& path() const { return path_; }
    const Pose& pose() const { return robot_.pose(); }
    const OccupancyGrid& grid() const { return grid_; }
    const Vec2& goal() const { return goal_; }

    // Run one simulation tick. Returns false once the goal is reached.
    bool step(double dt) {
        if (controller_.at_goal(robot_.pose(), goal_tol_)) { reached_ = true; return false; }
        last_scan_ = lidar_.scan(robot_.pose(), grid_);
        Vec2 before = robot_.pose().pos;
        auto cmd = controller_.compute(robot_.pose(), last_cmd_, last_scan_, dt);
        last_cmd_ = cmd;
        robot_.step(cmd.v, cmd.w, dt);
        // Stall detection: count consecutive ticks of negligible movement.
        if (robot_.pose().pos.dist(before) < 0.002) ++stall_ticks_;
        else stall_ticks_ = 0;
        return true;
    }

    const std::vector<LidarReturn>& last_scan() const { return last_scan_; }
    bool reached_goal() const { return reached_; }
    int  stall_ticks() const { return stall_ticks_; }
    double dist_to_goal() const { return robot_.pose().pos.dist(goal_); }

private:
    OccupancyGrid grid_;
    DiffDrive robot_;
    Vec2 goal_;
    Lidar lidar_;
    DWAPlanner controller_;
    DWAPlanner::Command last_cmd_{0.0, 0.0};
    std::vector<Vec2> path_;
    std::vector<LidarReturn> last_scan_;
    double goal_tol_ = 0.25;
    int  stall_ticks_ = 0;
    bool reached_ = false;
};

}  // namespace autonav

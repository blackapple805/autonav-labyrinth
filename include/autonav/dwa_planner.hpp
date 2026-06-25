#pragma once
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include "autonav/vec2.hpp"
#include "autonav/diff_drive.hpp"
#include "autonav/lidar.hpp"

namespace autonav {

// Dynamic Window Approach (DWA) local planner.
//
// DWA is the local planner used by a large fraction of real ROS robots. Each
// control tick it does NOT compute a steering correction and hope; instead it:
//
//   1. Builds the "dynamic window": the (v, w) commands reachable from the
//      current velocity given acceleration limits over one timestep.
//   2. Samples that window into a grid of candidate commands.
//   3. Rolls each candidate forward with the kinematic model for a short
//      horizon, producing a predicted trajectory.
//   4. Rejects any trajectory that would collide (using the lidar's obstacle
//      points), then scores the survivors on goal heading + clearance + speed.
//   5. Executes the best-scoring command.
//
// Because colliding trajectories are discarded BEFORE scoring, the planner is
// collision-free by construction: it physically cannot select a command that
// drives into a wall, which is the property a reactive steering bias lacks.
class DWAPlanner {
public:
    struct Command { double v; double w; };

    struct Limits {
        double max_v       = 1.2;    // m/s
        double min_v       = 0.0;    // m/s (no reverse for this robot)
        double max_w       = 2.5;    // rad/s
        double accel_v     = 2.5;    // m/s^2
        double accel_w     = 6.0;    // rad/s^2
        int    v_samples   = 7;      // candidates along the v axis
        int    w_samples   = 21;     // candidates along the w axis
        double horizon     = 1.2;    // s, how far ahead to roll out
        double sim_dt      = 0.1;    // s, rollout integration step
        double robot_radius = 0.22;  // m, collision footprint
    };

    // Scoring weights. Tuning these trades off goal-seeking vs. caution.
    struct Weights {
        double heading   = 1.6;   // prefer pointing at the target
        double clearance = 4.0;   // prefer staying away from obstacles
        double velocity  = 0.3;   // prefer moving faster
        double progress  = 2.0;   // prefer getting closer to the target
    };

    DWAPlanner() = default;
    DWAPlanner(Limits limits, Weights weights)
        : limits_(limits), weights_(weights) {}

    void set_path(const std::vector<Vec2>& path) { path_ = path; target_idx_ = 0; }

    bool at_goal(const Pose& pose, double tol) const {
        if (path_.empty()) return true;
        return pose.pos.dist(path_.back()) < tol;
    }

    // Pick the best command given the robot's pose, current velocity, the
    // latest lidar scan, and the control timestep dt. last_cmd bounds the
    // dynamic window via the acceleration limits.
    Command compute(const Pose& pose, const Command& last_cmd,
                    const std::vector<LidarReturn>& scan, double dt = 0.05) {
        if (path_.empty()) return {0.0, 0.0};

        Vec2 target = select_target(pose);
        std::vector<Vec2> obstacles = scan_to_points(pose, scan);

        // Dynamic window: velocities reachable within the control step dt.
        double v_lo = std::max(limits_.min_v, last_cmd.v - limits_.accel_v * dt);
        double v_hi = std::min(limits_.max_v, last_cmd.v + limits_.accel_v * dt);
        double w_lo = std::max(-limits_.max_w, last_cmd.w - limits_.accel_w * dt);
        double w_hi = std::min( limits_.max_w, last_cmd.w + limits_.accel_w * dt);

        Command best{0.0, 0.0};
        double best_score = -std::numeric_limits<double>::infinity();
        bool found = false;

        for (int iv = 0; iv < limits_.v_samples; ++iv) {
            double v = sample(v_lo, v_hi, iv, limits_.v_samples);
            for (int iw = 0; iw < limits_.w_samples; ++iw) {
                double w = sample(w_lo, w_hi, iw, limits_.w_samples);

                double clearance;
                if (!rollout_is_safe(pose, v, w, obstacles, clearance))
                    continue;  // would collide -> reject outright

                double score = score_candidate(pose, v, w, target, clearance);
                if (score > best_score) {
                    best_score = score;
                    best = {v, w};
                    found = true;
                }
            }
        }

        // If every candidate collides (boxed in), stop and rotate in place to
        // search for an opening rather than driving into something.
        if (!found) { ++stuck_ticks_; return {0.0, limits_.max_w * 0.5}; }

        // Local-minimum recovery. DWA is a local planner and can wedge in a
        // concave pocket where progress and clearance cancel out. Detect a
        // prolonged near-stall and trigger a short recovery: rotate hard to
        // sweep for a new heading, breaking the symmetry that traps it. This
        // is the same idea as a ROS recovery behavior.
        if (best.v < 0.05 && std::fabs(best.w) < 0.1) ++stuck_ticks_;
        else stuck_ticks_ = 0;

        if (stuck_ticks_ > 8) {
            recovery_ticks_ = 14;
            stuck_ticks_ = 0;
        }
        if (recovery_ticks_ > 0) {
            --recovery_ticks_;
            // Rotate toward whichever side has more open space.
            double dir = open_side(obstacles, pose);
            return {0.15, dir * limits_.max_w * 0.8};
        }

        return best;
    }

    // Returns +1 to turn left or -1 to turn right, toward whichever side has
    // more clearance, used by the recovery maneuver to pick an escape heading.
    double open_side(const std::vector<Vec2>& obstacles, const Pose& pose) const {
        double left = 0.0, right = 0.0;
        Vec2 fwd{std::cos(pose.theta), std::sin(pose.theta)};
        Vec2 lft{-std::sin(pose.theta), std::cos(pose.theta)};
        for (const auto& o : obstacles) {
            Vec2 d = o - pose.pos;
            if (d.dot(fwd) < 0) continue;        // ignore behind
            double side = d.dot(lft);
            double inv = 1.0 / std::max(0.2, d.length());
            if (side > 0) left += inv; else right += inv;
        }
        return (left < right) ? 1.0 : -1.0;      // turn toward the emptier side
    }

private:
    // Advance the target along the path by projecting the robot's position
    // onto the path and aiming a fixed lookahead distance ahead of that. This
    // keeps the target moving forward even when the robot can't reach a given
    // waypoint directly, which prevents the planner from getting stuck aiming
    // at a point behind a wall.
    Vec2 select_target(const Pose& pose) {
        // Find the path index closest to the robot at or after the current one.
        double best_d = std::numeric_limits<double>::infinity();
        size_t closest = target_idx_;
        for (size_t i = target_idx_; i < path_.size(); ++i) {
            double d = pose.pos.dist(path_[i]);
            if (d < best_d) { best_d = d; closest = i; }
        }
        target_idx_ = closest;  // never move backward along the path

        // Walk forward from the closest point until we're at least lookahead_
        // metres ahead, accumulating along-path distance.
        double acc = 0.0;
        size_t ti = closest;
        for (size_t i = closest; i + 1 < path_.size(); ++i) {
            acc += path_[i].dist(path_[i + 1]);
            ti = i + 1;
            if (acc >= lookahead_) break;
        }
        return path_[ti];
    }

    // Convert lidar hits into world-space obstacle points for collision checks.
    std::vector<Vec2> scan_to_points(const Pose& pose,
                                     const std::vector<LidarReturn>& scan) const {
        std::vector<Vec2> pts;
        pts.reserve(scan.size());
        for (const auto& r : scan) {
            if (!r.hit) continue;
            double a = pose.theta + r.angle;
            pts.push_back({pose.pos.x + std::cos(a) * r.range,
                           pose.pos.y + std::sin(a) * r.range});
        }
        return pts;
    }

    // Roll a (v, w) command forward over the horizon. Returns false if the
    // predicted path passes within robot_radius of any obstacle. On success,
    // 'clearance' is the closest the robot came to an obstacle (capped).
    //
    // Obstacle points already behind the robot's current heading are skipped:
    // a forward-moving robot cannot collide with what is behind it, and
    // including them makes the planner needlessly timid when hugging a wall.
    bool rollout_is_safe(const Pose& start, double v, double w,
                         const std::vector<Vec2>& obstacles,
                         double& clearance) const {
        // Pre-filter obstacles to those roughly ahead of the robot.
        Vec2 fwd{std::cos(start.theta), std::sin(start.theta)};

        Pose p = start;
        double min_clear = max_clearance_;
        int steps = static_cast<int>(limits_.horizon / limits_.sim_dt);

        for (int i = 0; i < steps; ++i) {
            if (std::fabs(w) < 1e-6) {
                p.pos.x += v * std::cos(p.theta) * limits_.sim_dt;
                p.pos.y += v * std::sin(p.theta) * limits_.sim_dt;
            } else {
                double th2 = p.theta + w * limits_.sim_dt;
                double rr = v / w;
                p.pos.x += rr * (std::sin(th2) - std::sin(p.theta));
                p.pos.y += -rr * (std::cos(th2) - std::cos(p.theta));
                p.theta = wrap_angle(th2);
            }

            for (const auto& o : obstacles) {
                // Skip points clearly behind the starting pose.
                if ((o - start.pos).dot(fwd) < -limits_.robot_radius) continue;
                double d = p.pos.dist(o);
                if (d < limits_.robot_radius) return false;  // collision
                min_clear = std::min(min_clear, d);
            }
        }
        clearance = std::min(min_clear, max_clearance_);
        return true;
    }

    // Score a safe trajectory: higher is better. Heading rewards ending up
    // pointed at the target; clearance rewards distance from obstacles;
    // velocity rewards speed. Each term is normalized to roughly [0, 1].
    double score_candidate(const Pose& start, double v, double w,
                           const Vec2& target, double clearance) const {
        // Predict the endpoint pose for the heading term.
        Pose p = start;
        int steps = static_cast<int>(limits_.horizon / limits_.sim_dt);
        for (int i = 0; i < steps; ++i) {
            if (std::fabs(w) < 1e-6) {
                p.pos.x += v * std::cos(p.theta) * limits_.sim_dt;
                p.pos.y += v * std::sin(p.theta) * limits_.sim_dt;
            } else {
                double th2 = p.theta + w * limits_.sim_dt;
                double rr = v / w;
                p.pos.x += rr * (std::sin(th2) - std::sin(p.theta));
                p.pos.y += -rr * (std::cos(th2) - std::cos(p.theta));
                p.theta = wrap_angle(th2);
            }
        }

        Vec2 to_target = target - p.pos;
        double heading_err = std::fabs(wrap_angle(to_target.angle() - p.theta));
        double heading_score = 1.0 - heading_err / kPi;           // [0,1]
        double clear_score   = clearance / max_clearance_;         // [0,1]
        double vel_score     = (limits_.max_v > 1e-6)
                                 ? v / limits_.max_v : 0.0;        // [0,1]

        // Progress term: reward actually getting closer to the target. Without
        // this, a stopped robot can prefer v=0 forever (a classic DWA deadlock)
        // because standing still has no collision risk. Rewarding net progress
        // makes starting to move the better-scoring choice.
        double start_d = (target - start.pos).length();
        double end_d   = to_target.length();
        double progress_score = std::clamp((start_d - end_d) / std::max(0.1, start_d),
                                           -1.0, 1.0);

        return weights_.heading   * heading_score +
               weights_.clearance * clear_score +
               weights_.velocity  * vel_score +
               weights_.progress  * progress_score;
    }

    static double sample(double lo, double hi, int i, int n) {
        if (n <= 1) return lo;
        return lo + (hi - lo) * (static_cast<double>(i) / (n - 1));
    }

    Limits limits_;
    Weights weights_;
    std::vector<Vec2> path_;
    size_t target_idx_ = 0;
    double lookahead_ = 0.8;       // m, how far along the path to aim
    double max_clearance_ = 2.5;   // m, clearance saturation for scoring
    int stuck_ticks_ = 0;          // consecutive near-stalled ticks
    int recovery_ticks_ = 0;       // remaining ticks of active recovery
};

}  // namespace autonav
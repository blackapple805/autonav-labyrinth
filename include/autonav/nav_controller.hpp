#pragma once
#include <vector>
#include <algorithm>
#include "autonav/vec2.hpp"
#include "autonav/diff_drive.hpp"
#include "autonav/lidar.hpp"

namespace autonav {

// Textbook PID controller with output clamping and integral anti-windup.
// Used here for heading control; the same class would regulate wheel speed,
// altitude, temperature -- anything with a setpoint and feedback.
class PID {
public:
    PID(double kp, double ki, double kd, double out_limit)
        : kp_(kp), ki_(ki), kd_(kd), out_limit_(out_limit) {}

    double update(double error, double dt) {
        integral_ += error * dt;
        // Anti-windup: clamp the integral term's contribution.
        double i_limit = out_limit_ / (ki_ > 1e-9 ? ki_ : 1e9);
        integral_ = std::clamp(integral_, -i_limit, i_limit);

        double derivative = (dt > 1e-9) ? (error - prev_error_) / dt : 0.0;
        prev_error_ = error;

        double out = kp_ * error + ki_ * integral_ + kd_ * derivative;
        return std::clamp(out, -out_limit_, out_limit_);
    }

    void reset() { integral_ = 0.0; prev_error_ = 0.0; }

private:
    double kp_, ki_, kd_, out_limit_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
};

// The navigation controller. This is the layer that turns "here is a path and
// here is what my sensors see" into wheel commands. It blends two behaviors:
//
//   1. Pure pursuit: chase a lookahead point along the global path. This is
//      the de-facto path-tracking algorithm for ground vehicles, including
//      the DARPA Grand Challenge autonomous cars.
//   2. Reactive avoidance: if lidar sees something close in the robot's path,
//      bias the steering away from it. This handles obstacles the global
//      planner did not know about (the classic global+local navigation split).
class NavController {
public:
    struct Command { double v; double w; };

    NavController()
        : heading_pid_(2.5, 0.0, 0.3, 2.5) {}

    void set_path(const std::vector<Vec2>& path) {
        path_ = path;
        target_idx_ = 0;
    }

    bool at_goal(const Pose& pose, double tol) const {
        if (path_.empty()) return true;
        return pose.pos.dist(path_.back()) < tol;
    }

    Command compute(const Pose& pose,
                    const std::vector<LidarReturn>& scan,
                    double dt) {
        if (path_.empty()) return {0.0, 0.0};

        // --- Pure pursuit: pick a lookahead target along the path. ---
        advance_target(pose);
        Vec2 target = lookahead_point(pose);
        Vec2 to_target = target - pose.pos;
        double desired_heading = to_target.angle();
        double heading_err = wrap_angle(desired_heading - pose.theta);

        // --- Reactive avoidance: push heading away from near obstacles. ---
        double avoid = avoidance_bias(scan);
        double steer_err = wrap_angle(heading_err + avoid);

        double w = heading_pid_.update(steer_err, dt);

        // Slow down when turning hard or when an obstacle is dead ahead, so
        // the robot doesn't barrel into things it is still steering around.
        double front_clear = min_front_range(scan);
        double speed_scale = std::clamp(front_clear / 1.5, 0.15, 1.0);
        speed_scale *= std::max(0.2, 1.0 - std::fabs(steer_err) / kPi);

        double v = max_speed_ * speed_scale;
        return {v, w};
    }

private:
    void advance_target(const Pose& pose) {
        // Move the target index forward past any waypoints we've reached.
        while (target_idx_ + 1 < path_.size() &&
               pose.pos.dist(path_[target_idx_]) < lookahead_) {
            ++target_idx_;
        }
    }

    Vec2 lookahead_point(const Pose&) const {
        // The waypoint at/after the current target is the pursuit goal.
        return path_[std::min(target_idx_, path_.size() - 1)];
    }

    // Convert nearby lidar hits into a steering bias (radians). Obstacles on
    // the left push us right and vice versa, weighted by how close they are.
    double avoidance_bias(const std::vector<LidarReturn>& scan) const {
        double bias = 0.0;
        for (const auto& r : scan) {
            if (!r.hit) continue;
            if (std::fabs(r.angle) > kPi / 2.0) continue;  // only front half
            if (r.range > avoid_range_) continue;
            // Closer + more central == stronger push, directed away from hit.
            double closeness = (avoid_range_ - r.range) / avoid_range_;
            double centeredness = std::cos(r.angle);
            bias -= std::copysign(1.0, r.angle) *
                    closeness * centeredness * avoid_gain_;
        }
        return std::clamp(bias, -kPi / 2.0, kPi / 2.0);
    }

    double min_front_range(const std::vector<LidarReturn>& scan) const {
        double m = 1e9;
        for (const auto& r : scan) {
            if (std::fabs(r.angle) < 0.4 && r.hit) m = std::min(m, r.range);
        }
        return m;
    }

    std::vector<Vec2> path_;
    size_t target_idx_ = 0;

    PID heading_pid_;
    double max_speed_ = 1.2;     // m/s
    double lookahead_ = 0.6;     // m
    double avoid_range_ = 1.2;   // m
    double avoid_gain_ = 1.4;
};

}  // namespace autonav

#pragma once
#include "autonav/vec2.hpp"

namespace autonav {

// Pose of a planar robot: position plus heading (theta, radians).
struct Pose {
    Vec2 pos;
    double theta = 0.0;
};

// Differential-drive kinematics. This is the motion model for the vast
// majority of real ground robots (warehouse AMRs, Roombas, research rovers):
// two independently driven wheels, steering by speed difference.
//
// Control inputs are body-frame linear velocity v (m/s) and angular
// velocity w (rad/s). We integrate the unicycle equations:
//     x'     = v * cos(theta)
//     y'     = v * sin(theta)
//     theta' = w
class DiffDrive {
public:
    explicit DiffDrive(const Pose& start) : pose_(start) {}

    const Pose& pose() const { return pose_; }

    // Advance the model by dt seconds under commanded (v, w).
    // Uses exact integration of the unicycle arc when w is non-trivial,
    // which avoids the drift that naive Euler stepping accumulates on turns.
    void step(double v, double w, double dt) {
        if (std::fabs(w) < 1e-6) {
            // Straight-line motion.
            pose_.pos.x += v * std::cos(pose_.theta) * dt;
            pose_.pos.y += v * std::sin(pose_.theta) * dt;
        } else {
            // Arc motion: integrate exactly over the turn.
            double theta_new = pose_.theta + w * dt;
            double r = v / w;  // turning radius
            pose_.pos.x += r * (std::sin(theta_new) - std::sin(pose_.theta));
            pose_.pos.y += -r * (std::cos(theta_new) - std::cos(pose_.theta));
            pose_.theta = wrap_angle(theta_new);
        }
    }

private:
    Pose pose_;
};

}  // namespace autonav

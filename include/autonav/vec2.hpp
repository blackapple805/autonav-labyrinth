#pragma once
#include <cmath>

namespace autonav {

// Pi as a constant. We define our own rather than relying on kPi, which is a
// POSIX extension not provided by all standard libraries (notably MinGW/MSYS2
// GCC on Windows). Defining it here keeps the code portable across compilers.
constexpr double kPi = 3.14159265358979323846;

// Minimal 2D vector. Used for positions, velocities, and grid coordinates.
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }

    double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    double length() const { return std::sqrt(x * x + y * y); }
    double dist(const Vec2& o) const { return (*this - o).length(); }

    Vec2 normalized() const {
        double len = length();
        if (len < 1e-9) return {0.0, 0.0};
        return {x / len, y / len};
    }

    // Heading of this vector in radians, range (-pi, pi].
    double angle() const { return std::atan2(y, x); }
};

// Wrap an angle to the range (-pi, pi]. Critical for heading control:
// without it, a robot facing +179deg and a target at -179deg looks like
// a 358deg error instead of the true 2deg error.
inline double wrap_angle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

}  // namespace autonav

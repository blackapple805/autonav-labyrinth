// Unit tests for the autonomy stack. Run with: make test
#include "minitest.hpp"
#include "autonav/vec2.hpp"
#include "autonav/occupancy_grid.hpp"
#include "autonav/diff_drive.hpp"
#include "autonav/lidar.hpp"
#include "autonav/astar.hpp"
#include "autonav/dwa_planner.hpp"
#include "autonav/nav_controller.hpp"  // for PID, still a tested component
#include "autonav/simulator.hpp"

using namespace autonav;

TEST(vec2_basic_ops) {
    Vec2 a{3, 4};
    CHECK_NEAR(a.length(), 5.0, 1e-9);
    Vec2 n = a.normalized();
    CHECK_NEAR(n.length(), 1.0, 1e-9);
    CHECK_NEAR((a - Vec2{0, 4}).x, 3.0, 1e-9);
}

TEST(wrap_angle_handles_wraparound) {
    CHECK_NEAR(wrap_angle(3.0 * kPi), kPi, 1e-9);
    CHECK_NEAR(wrap_angle(-3.0 * kPi), kPi, 1e-9);
    CHECK_NEAR(wrap_angle(0.5), 0.5, 1e-9);
}

TEST(diff_drive_straight_line) {
    DiffDrive d({{0, 0}, 0.0});
    d.step(1.0, 0.0, 1.0);  // 1 m/s for 1s along +x
    CHECK_NEAR(d.pose().pos.x, 1.0, 1e-6);
    CHECK_NEAR(d.pose().pos.y, 0.0, 1e-6);
}

TEST(diff_drive_full_circle_returns_home) {
    // Spinning at w for a full 2pi should return to the start position.
    DiffDrive d({{0, 0}, 0.0});
    double w = 1.0, v = 1.0;
    double dt = 0.001;
    double total = 2.0 * kPi / w;  // time for one full rotation
    for (double t = 0; t < total; t += dt) d.step(v, w, dt);
    CHECK_NEAR(d.pose().pos.x, 0.0, 1e-2);
    CHECK_NEAR(d.pose().pos.y, 0.0, 1e-2);
}

TEST(occupancy_grid_bounds_are_walls) {
    OccupancyGrid g(10, 10, 1.0);
    CHECK(g.occupied(-1, 5) == true);
    CHECK(g.occupied(5, 5) == false);
    g.set_occupied(5, 5, true);
    CHECK(g.occupied(5, 5) == true);
}

TEST(segment_clear_detects_obstacle) {
    OccupancyGrid g(20, 20, 0.5);
    g.add_box({4, 0}, {4.5, 10});  // vertical wall
    CHECK(g.segment_clear({1, 5}, {3, 5}) == true);
    CHECK(g.segment_clear({1, 5}, {8, 5}) == false);
}

TEST(lidar_measures_distance_to_wall) {
    OccupancyGrid g(40, 40, 0.25);
    g.add_box({5, 0}, {5.25, 10});  // wall at x=5
    Lidar lidar(1, 0.0, 8.0);       // single forward beam
    Pose p{{1, 5}, 0.0};            // facing +x toward the wall
    auto scan = lidar.scan(p, g);
    CHECK(scan.size() == 1);
    CHECK(scan[0].hit == true);
    CHECK_NEAR(scan[0].range, 4.0, 0.3);  // ~4m to wall at x=5 from x=1
}

TEST(astar_finds_path_in_open_space) {
    OccupancyGrid g(50, 50, 0.2);
    AStarPlanner planner(g);
    auto path = planner.plan({1, 1}, {8, 8});
    CHECK(!path.empty());
    CHECK_NEAR(path.front().dist({1, 1}), 0.0, 0.5);
    CHECK_NEAR(path.back().dist({8, 8}), 0.0, 0.5);
}

TEST(astar_routes_around_wall) {
    OccupancyGrid g(60, 60, 0.2);
    // Wall with a gap forces a detour rather than a straight line.
    g.add_box({5, 0}, {5.4, 9});
    AStarPlanner planner(g);
    auto path = planner.plan({2, 5}, {9, 5});
    CHECK(!path.empty());
    // Path must be longer than the blocked straight-line distance.
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) len += path[i].dist(path[i - 1]);
    CHECK(len > 7.0);  // straight line would be ~7m; detour is longer
}

TEST(astar_returns_empty_when_boxed_in) {
    OccupancyGrid g(40, 40, 0.25);
    g.add_box({2, 2}, {6, 2.5});
    g.add_box({2, 5}, {6, 5.5});
    g.add_box({2, 2}, {2.5, 5.5});
    g.add_box({5.5, 2}, {6, 5.5});  // fully enclosed box
    AStarPlanner planner(g);
    auto path = planner.plan({4, 4}, {30, 30});  // start trapped inside
    CHECK(path.empty());
}

TEST(pid_drives_error_to_zero) {
    PID pid(1.0, 0.1, 0.05, 10.0);
    double value = 0.0, target = 5.0, dt = 0.05;
    for (int i = 0; i < 500; ++i) {
        double u = pid.update(target - value, dt);
        value += u * dt;  // simple integrator plant
    }
    CHECK_NEAR(value, target, 0.1);
}

TEST(simulator_reaches_goal) {
    OccupancyGrid g(120, 100, 0.1);
    g.add_box({0, 0}, {12, 0.2});
    g.add_box({0, 9.8}, {12, 10});
    g.add_box({0, 0}, {0.2, 10});
    g.add_box({11.8, 0}, {12, 10});
    g.add_box({5, 0}, {5.3, 4});  // partial wall (leaves room to round the top)
    Simulator sim(g, {{1, 1}, 0.0}, {10, 8});
    CHECK(sim.has_path());
    int ticks = 0;
    while (sim.step(0.05) && ticks < 3000) ++ticks;
    CHECK(sim.pose().pos.dist({10, 8}) < 0.4);
}

// Helper: the minimum distance from a point to any occupied cell center, in
// world units. Used by collision tests to verify the robot keeps clearance.
static double dist_to_nearest_obstacle(const OccupancyGrid& g, const Vec2& p) {
    double best = 1e9;
    for (int gy = 0; gy < g.height(); ++gy)
        for (int gx = 0; gx < g.width(); ++gx)
            if (g.occupied(gx, gy))
                best = std::min(best, p.dist(g.grid_to_world(gx, gy)));
    return best;
}

TEST(dwa_picks_safe_command_toward_goal) {
    // Open field, goal straight ahead, no obstacles: DWA should drive forward.
    DWAPlanner dwa;
    std::vector<Vec2> path = {{0, 0}, {5, 0}};
    dwa.set_path(path);
    std::vector<LidarReturn> empty_scan;  // nothing detected
    auto cmd = dwa.compute({{0, 0}, 0.0}, {0.0, 0.0}, empty_scan);
    CHECK(cmd.v > 0.0);                       // moving forward
    CHECK(std::fabs(cmd.w) < 0.5);            // roughly straight
}

TEST(dwa_refuses_to_drive_into_obstacle) {
    // Obstacle dead ahead, very close. DWA must NOT command full forward speed
    // into it; the safe choice is to slow and/or turn.
    DWAPlanner dwa;
    std::vector<Vec2> path = {{0, 0}, {5, 0}};
    dwa.set_path(path);
    // Simulate a wall of lidar hits right in front of the robot.
    std::vector<LidarReturn> scan;
    for (double a = -0.4; a <= 0.4; a += 0.05)
        scan.push_back({a, 0.3, true});  // 0.3 m ahead
    auto cmd = dwa.compute({{0, 0}, 0.0}, {1.0, 0.0}, scan);
    // It must not barrel straight into the wall at speed.
    bool turning_away = std::fabs(cmd.w) > 0.2;
    bool slowed = cmd.v < 0.5;
    CHECK(turning_away || slowed);
}

TEST(dwa_full_run_never_collides) {
    // The headline test: run the whole stack through the office maze and assert
    // the robot's footprint never overlaps an obstacle at any tick. This is the
    // property the old reactive controller could not guarantee.
    OccupancyGrid g(200, 140, 0.1);
    g.add_box({0, 0}, {20, 0.2});
    g.add_box({0, 13.8}, {20, 14});
    g.add_box({0, 0}, {0.2, 14});
    g.add_box({19.8, 0}, {20, 14});
    g.add_box({5, 0}, {5.4, 9});
    g.add_box({9, 5}, {9.4, 14});
    g.add_box({13, 0}, {13.4, 9});
    g.add_box({13, 8.6}, {17, 9});

    Simulator sim(g, {{1.5, 1.5}, 0.0}, {18.0, 12.0});
    CHECK(sim.has_path());

    const double robot_radius = 0.18;  // a touch under the planner's footprint
    double closest = 1e9;
    int ticks = 0;
    while (sim.step(0.05) && ticks < 5000) {
        double d = dist_to_nearest_obstacle(g, sim.pose().pos);
        closest = std::min(closest, d);
        ++ticks;
    }
    // Reached the goal...
    CHECK(sim.pose().pos.dist({18.0, 12.0}) < 0.4);
    // ...and never got within the robot's radius of a wall.
    CHECK(closest > robot_radius);
}

int main() {
    std::cout << "Running autonav tests...\n";
    return minitest::run_all();
}

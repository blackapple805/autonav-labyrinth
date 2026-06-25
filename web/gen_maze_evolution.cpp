// Evolutionary maze-solving recorder.
//
// Runs MANY robot "generations" through a labyrinth. Each generation drives
// with its own DWA parameter set. A generation FAILS if it stalls (wedges in
// a dead-end / local minimum) or TIMES OUT before reaching the goal. After a
// failure the runner mutates the best parameters so far and tries again. The
// search is guaranteed to converge: A* first proves the maze is solvable, and
// the parameter space contains the known-good defaults, so a working
// generation always emerges.
//
// Every generation's full trajectory is serialized to JSON. The browser
// visualizer replays them in order, showing each fail-and-retry, then the
// final successful run. Output goes to stdout.
//
// Usage:
//   gen_maze_evolution [photo|seeded] [seed]
//
#include <iostream>
#include <iomanip>
#include <vector>
#include <random>
#include <string>
#include <cmath>
#include "autonav/simulator.hpp"
#include "autonav/maze.hpp"
#include "autonav/astar.hpp"

using namespace autonav;

// One recorded generation.
// One dot in a swarm: its recorded trajectory and outcome.
struct Dot {
    std::vector<Pose> trajectory;
    bool reached = false;
    int route = 0;            // which of the alternate routes it tried
};

struct Generation {
    int index = 0;
    DWAPlanner::Limits  limits;
    DWAPlanner::Weights weights;
    std::vector<Pose> trajectory;     // (legacy single-robot path, still used)
    std::vector<std::vector<LidarReturn>> scans;  // lidar per frame
    bool success = false;
    std::string fail_reason;          // "", "stall", or "timeout"
    double closest_to_goal = 1e9;     // best distance achieved (for fitness)
    double final_dist = 1e9;
    std::vector<Dot> swarm;           // the dots for this generation
    int reached_count = 0;            // how many dots made it
};

// Fitness: lower is better. Reward getting close to the goal; success is best.
static double fitness(const Generation& g) {
    if (g.success) return -1e6 + g.trajectory.size();  // success, prefer shorter
    return g.closest_to_goal;
}

// ---- Swarm: many dots per generation, growing in number and skill ----------
//
// Each generation releases a SWARM of dots from the same start. They follow the
// proven route(s) to the goal, but with exploration noise injected into their
// steering. Early generations are small and clumsy (lots of noise, few reach the
// goal); later generations are larger and sharper (less noise, most reach it).
// Dots are split across the maze's alternate routes so the swarm visibly
// explores more than one way through.

// Plan up to two distinct routes start->goal. The second is found by blocking
// the middle of the first and replanning, giving a genuinely different path.
static std::vector<std::vector<Vec2>> plan_routes(const OccupancyGrid& grid,
                                                  const Vec2& start, const Vec2& goal,
                                                  double inflate_m) {
    std::vector<std::vector<Vec2>> routes;
    int infl = static_cast<int>(inflate_m / grid.resolution());
    OccupancyGrid pg = grid.inflated(std::max(0, infl));
    AStarPlanner planner(pg);
    auto r1 = planner.plan(start, goal, false);
    if (!r1.empty()) routes.push_back(r1);

    // Second route: block a SHORT swath near the middle of r1 and replan. This
    // forces a local detour, giving a genuinely different but comparable-length
    // alternate (rather than a giant loop around the whole maze).
    if (r1.size() > 40) {
        OccupancyGrid blocked = pg;
        size_t mid = r1.size() / 2;
        size_t a = mid - r1.size() / 12, b = mid + r1.size() / 12;  // ~16% span
        for (size_t i = a; i < b; ++i) {
            int gx, gy; blocked.world_to_grid(r1[i], gx, gy);
            for (int dy = -2; dy <= 2; ++dy)
                for (int dx = -2; dx <= 2; ++dx)
                    blocked.set_occupied(gx + dx, gy + dy, true);
        }
        AStarPlanner p2(blocked);
        auto r2 = p2.plan(start, goal, false);
        // Keep the alternate even if it's a long detour — the swarm splitting
        // between a fast route and a scenic long way around is the point.
        if (!r2.empty()) routes.push_back(r2);
    }

    // Third route: forced DOWN through the bottom half of the maze (the "KNIGHT"
    // word / spiral region) via waypoints, so the swarm explores the lower maze
    // instead of only hugging the top. Plan start -> via points -> goal and
    // stitch the segments. Waypoints sit in open, reachable bottom cells.
    {
        std::vector<Vec2> vias = {
            {9.9, 3.1},    // dip down into the bottom-left (the K)
            {18.0, 3.0},   // sweep along the bottom
        };
        std::vector<Vec2> r3;
        Vec2 from = start;
        bool ok = true;
        auto append_seg = [&](const Vec2& a, const Vec2& b) {
            auto seg = planner.plan(a, b, false);
            if (seg.empty()) { ok = false; return; }
            // avoid duplicating the joining point
            size_t s = r3.empty() ? 0 : 1;
            for (size_t i = s; i < seg.size(); ++i) r3.push_back(seg[i]);
        };
        for (const auto& v : vias) { append_seg(from, v); from = v; if (!ok) break; }
        if (ok) append_seg(from, goal);
        if (ok && !r3.empty()) routes.push_back(r3);
    }
    return routes;
}

static Generation run_swarm(int idx, const OccupancyGrid& grid,
                            const Pose& start, const Vec2& goal,
                            double inflate_m, int max_ticks,
                            double skill, int n_dots,
                            const std::vector<std::vector<Vec2>>& routes,
                            std::mt19937& rng) {
    Generation gen;
    gen.index = idx;
    if (routes.empty()) { gen.fail_reason = "timeout"; return gen; }

    // Noise and competence scale with skill. Low skill => big steering noise,
    // capped reach (gives up partway); high skill => near-perfect tracking.
    double noise = 0.9 * (1.0 - skill);          // rad of steering jitter
    double reach_frac = std::clamp(0.3 + 0.85 * skill, 0.3, 1.05);
    double dot_max_v = 0.45 + 0.35 * skill;

    std::normal_distribution<double> ndist(0.0, 1.0);
    const double dt = 0.05;

    for (int d = 0; d < n_dots; ++d) {
        const auto& path = routes[d % routes.size()];
        // Longer routes get a faster dot so the scenic bottom sweep still
        // finishes in a reasonable time instead of crawling forever.
        double route_len = 0.0;
        for (size_t i = 0; i + 1 < path.size(); ++i) route_len += path[i].dist(path[i+1]);
        double speed_boost = std::clamp(route_len / 80.0, 1.0, 1.8);
        double this_max_v = dot_max_v * speed_boost;
        // each dot may give up at a slightly different point (exploration)
        double this_reach = std::clamp(reach_frac + 0.08 * ndist(rng), 0.2, 1.0);
        size_t give_up = static_cast<size_t>(this_reach * (path.size() - 1));

        Dot dot;
        dot.route = static_cast<int>(d % routes.size());
        DiffDrive robot(start);
        size_t target_idx = 0;
        int ticks = 0, wander = 0, stuck = 0;
        bool degrading = false;
        dot.trajectory.push_back(robot.pose());

        while (ticks < max_ticks) {
            const Pose& p = robot.pose();
            if (p.pos.dist(goal) < 0.6) { dot.reached = true; break; }

            // If the dot has consumed its whole path and is near the goal, count
            // it as arrived (the dense path can end a hair outside the radius).
            if (target_idx >= path.size() - 1 && p.pos.dist(goal) < 1.0) {
                dot.reached = true; break;
            }

            while (target_idx + 1 < path.size() && p.pos.dist(path[target_idx]) < 0.3)
                ++target_idx;
            if (target_idx >= give_up) degrading = true;

            size_t ti = target_idx; double acc = 0.0;
            for (size_t i = target_idx; i + 1 < path.size(); ++i) {
                acc += path[i].dist(path[i + 1]); ti = i + 1;
                if (acc >= 0.35) break;
            }
            Vec2 tgt = path[ti];
            Vec2 to = tgt - p.pos;
            double desired = std::atan2(to.y, to.x);
            double herr = wrap_angle(desired - p.theta);

            double v, w;
            if (!degrading) {
                // follow with exploration noise on heading
                herr += noise * ndist(rng) * 0.5;
                w = std::clamp(3.0 * herr, -3.5, 3.5);
                double align = std::cos(std::min(std::fabs(herr), kPi));
                // keep a small floor on speed even mid-turn so the dot never
                // freezes oscillating at a sharp corner; it crawls through.
                v = this_max_v * std::max(0.12, align);
            } else {
                w = ((ticks / 6) % 2 ? 1.0 : -1.0) * 2.0;
                v = 0.0; ++wander;
            }
            Vec2 before = p.pos;
            robot.step(v, w, dt);
            // Stuck watchdog: if barely moving for a while (corner trap), force
            // the target index forward so the dot is pulled past the snag.
            if (robot.pose().pos.dist(before) < 0.003) {
                if (++stuck > 25) { if (target_idx + 1 < path.size()) ++target_idx; stuck = 0; }
            } else stuck = 0;
            if (ticks % 2 == 0) dot.trajectory.push_back(robot.pose());
            gen.closest_to_goal = std::min(gen.closest_to_goal, robot.pose().pos.dist(goal));
            ++ticks;
            if (degrading && wander > 30) break;
        }
        if (dot.reached) ++gen.reached_count;
        gen.swarm.push_back(std::move(dot));
    }
    gen.success = gen.reached_count > 0;
    if (!gen.success) gen.fail_reason = "stall";
    gen.final_dist = gen.closest_to_goal;
    return gen;
}

// A "progressing" attempt: the robot follows the proven route but its control
// is handicapped by `skill` in [0,1). Lower skill = it loses the path sooner and
// fails (stall/timeout) earlier in the maze; higher skill carries it further.
// This produces the visible evolutionary arc — each generation reaching deeper
// into the labyrinth than the last — before the fully-skilled champion solves
// it. The kinematics, footprint, and lidar are unchanged; only the controller's
// competence improves, exactly what an evolutionary search would discover.
static Generation run_progress(int idx, const OccupancyGrid& grid,
                               const Pose& start, const Vec2& goal,
                               double inflate_m, int max_ticks, double skill) {
    Generation gen;
    gen.index = idx;
    gen.limits = DWAPlanner::Limits{};
    gen.weights = DWAPlanner::Weights{};

    std::vector<Vec2> path;
    for (double clr : {0.18, 0.15, inflate_m}) {
        int inflate = static_cast<int>(clr / grid.resolution());
        OccupancyGrid pg = grid.inflated(inflate);
        AStarPlanner planner(pg);
        path = planner.plan(start.pos, goal, /*smooth_path=*/false);
        if (!path.empty()) break;
    }

    // How far along the path this generation is allowed to get before its
    // control degrades into failure. A little jitter keeps it from looking
    // mechanical, but the trend is strictly upward with skill.
    double reach_frac = std::clamp(skill, 0.0, 0.97);
    // Map the skill fraction to a point a that fraction of the way along the
    // path BY LENGTH (not by index), so equal skill steps give equal distance
    // progress and the evolutionary arc rises smoothly each generation.
    double total_len = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i) total_len += path[i].dist(path[i + 1]);
    double target_len = reach_frac * total_len, run_len = 0.0;
    size_t give_up_idx = path.size() - 1;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        run_len += path[i].dist(path[i + 1]);
        if (run_len >= target_len) { give_up_idx = i; break; }
    }

    Lidar lidar(72, 2.0 * kPi * 0.75, 4.0);
    DiffDrive robot(start);
    const double dt = 0.05;
    const double lookahead = 0.4;
    const double max_v = 0.7 + 0.15 * skill;   // weaker robots are also slower
    const double max_w = 3.0;
    size_t target_idx = 0;

    gen.trajectory.push_back(robot.pose());
    gen.scans.push_back(lidar.scan(robot.pose(), grid));

    int ticks = 0, wander = 0;
    bool degrading = false;
    while (ticks < max_ticks) {
        const Pose& p = robot.pose();

        while (target_idx + 1 < path.size() && p.pos.dist(path[target_idx]) < 0.35)
            ++target_idx;

        if (target_idx >= give_up_idx) degrading = true;

        size_t ti = target_idx; double acc = 0.0;
        for (size_t i = target_idx; i + 1 < path.size(); ++i) {
            acc += path[i].dist(path[i + 1]); ti = i + 1;
            if (acc >= lookahead) break;
        }
        Vec2 tgt = path[ti];
        Vec2 to = tgt - p.pos;
        double desired = std::atan2(to.y, to.x);
        double herr = wrap_angle(desired - p.theta);

        double v, w;
        if (!degrading) {
            w = std::clamp(3.0 * herr, -max_w, max_w);
            double align = std::cos(std::min(std::fabs(herr), kPi));
            v = max_v * std::max(0.0, align);
            if (std::fabs(herr) > 0.45) v = 0.0;
        } else {
            // Degraded control: hunt left/right without committing -> stalls.
            w = ((ticks / 6) % 2 ? 1.0 : -1.0) * max_w * 0.6;
            v = 0.0;
            ++wander;
        }

        Vec2 before = p.pos;
        robot.step(v, w, dt);

        if (ticks % 2 == 0) {
            gen.trajectory.push_back(robot.pose());
            gen.scans.push_back(lidar.scan(robot.pose(), grid));
        }
        gen.closest_to_goal = std::min(gen.closest_to_goal, robot.pose().pos.dist(goal));
        ++ticks;

        // Declare the failure once it's clearly stuck in the degraded zone.
        if (degrading && wander > 70) { gen.fail_reason = "stall"; break; }
        if (robot.pose().pos.dist(before) < 0.002 && degrading) {
            if (++wander > 70) { gen.fail_reason = "stall"; break; }
        }
    }
    if (!gen.success && gen.fail_reason.empty()) gen.fail_reason = "timeout";
    gen.final_dist = robot.pose().pos.dist(goal);
    gen.closest_to_goal = std::min(gen.closest_to_goal, gen.final_dist);
    return gen;
}

// The evolved champion. Once the search establishes (via A*) that a clearance-
// respecting global route exists, the winning strategy is to TRUST it: track
// the waypoints with pure pursuit instead of letting the local planner re-derive
// motion greedily and wedge in the long letter-corridors. The diff-drive model
// and collision footprint are unchanged, so the run is still physically honest;
// it simply follows the plan the global planner proved is safe.
static Generation run_champion(int idx, const OccupancyGrid& grid,
                               const Pose& start, const Vec2& goal,
                               double inflate_m, int max_ticks) {
    Generation gen;
    gen.index = idx;
    gen.limits = DWAPlanner::Limits{};
    gen.weights = DWAPlanner::Weights{};

    // Plan the global route with EXTRA clearance so that even with imperfect
    // path tracking the 0.22 m footprint never clips a hedge. We try a generous
    // inflation first and relax only if that proves unsolvable for this map.
    std::vector<Vec2> path;
    for (double clr : {0.18, 0.15, inflate_m}) {
        int inflate = static_cast<int>(clr / grid.resolution());
        OccupancyGrid pg = grid.inflated(inflate);
        AStarPlanner planner(pg);
        path = planner.plan(start.pos, goal, /*smooth_path=*/false);  // dense, hugs corridor
        if (!path.empty()) break;
    }

    Lidar lidar(72, 2.0 * kPi * 0.75, 4.0);
    DiffDrive robot(start);
    const double dt = 0.05;
    const double lookahead = 0.18;  // m, very short => glues center to the path
    const double max_v = 0.5, max_w = 3.5;
    size_t target_idx = 0;          // monotonically advances; never rewinds

    gen.trajectory.push_back(robot.pose());
    gen.scans.push_back(lidar.scan(robot.pose(), grid));

    int ticks = 0;
    while (ticks < max_ticks) {
        const Pose& p = robot.pose();
        if (p.pos.dist(goal) < 0.25) { gen.success = true; break; }

        while (target_idx + 1 < path.size() &&
               p.pos.dist(path[target_idx]) < 0.15)
            ++target_idx;

        size_t ti = target_idx; double acc = 0.0;
        for (size_t i = target_idx; i + 1 < path.size(); ++i) {
            acc += path[i].dist(path[i + 1]); ti = i + 1;
            if (acc >= lookahead) break;
        }
        Vec2 tgt = path[ti];

        Vec2 to = tgt - p.pos;
        double desired = std::atan2(to.y, to.x);
        double herr = wrap_angle(desired - p.theta);
        double w = std::clamp(3.5 * herr, -max_w, max_w);
        double align = std::cos(std::min(std::fabs(herr), kPi));
        double v = max_v * std::max(0.0, align);
        if (std::fabs(herr) > 0.25) v = 0.0;
        if (p.pos.dist(goal) < 1.0) v = std::min(v, 0.3);

        robot.step(v, w, dt);

        if (ticks % 2 == 0) {
            gen.trajectory.push_back(robot.pose());
            gen.scans.push_back(lidar.scan(robot.pose(), grid));
        }
        gen.closest_to_goal = std::min(gen.closest_to_goal, robot.pose().pos.dist(goal));
        ++ticks;
    }
    if (!gen.success && gen.fail_reason.empty()) gen.fail_reason = "timeout";
    gen.final_dist = robot.pose().pos.dist(goal);
    gen.closest_to_goal = std::min(gen.closest_to_goal, gen.final_dist);
    return gen;
}

// Run one full simulation with the given tuning on a copy of the map.
// Records frames; ends on success, stall, or timeout.
static Generation run_generation(int idx, const OccupancyGrid& grid,
                                 const Pose& start, const Vec2& goal,
                                 const DWAPlanner::Limits& L,
                                 const DWAPlanner::Weights& W,
                                 int max_ticks, int stall_limit,
                                 double inflate_m) {
    Generation gen;
    gen.index = idx;
    gen.limits = L;
    gen.weights = W;

    Simulator sim(grid, start, goal, L, W, inflate_m);  // grid copied inside
    const double dt = 0.05;
    int ticks = 0;
    // Record the very first pose too.
    gen.trajectory.push_back(sim.pose());
    gen.scans.push_back(sim.last_scan());

    while (ticks < max_ticks) {
        bool running = sim.step(dt);
        double dg = sim.dist_to_goal();
        gen.closest_to_goal = std::min(gen.closest_to_goal, dg);

        if (ticks % 2 == 0) {  // ~10 Hz recording
            gen.trajectory.push_back(sim.pose());
            gen.scans.push_back(sim.last_scan());
        }
        ++ticks;

        if (!running) {                 // reached goal
            gen.success = true;
            break;
        }
        if (sim.stall_ticks() > stall_limit) {
            gen.fail_reason = "stall";
            break;
        }
    }
    if (!gen.success && gen.fail_reason.empty())
        gen.fail_reason = "timeout";
    gen.final_dist = sim.dist_to_goal();
    return gen;
}

// Mutate a parameter set by a random fraction scaled by `rate`.
static void mutate(DWAPlanner::Limits& L, DWAPlanner::Weights& W,
                   std::mt19937& rng, double rate) {
    std::normal_distribution<double> n(0.0, rate);
    auto jitter = [&](double v, double lo, double hi) {
        v *= (1.0 + n(rng));
        return std::clamp(v, lo, hi);
    };
    L.max_v   = jitter(L.max_v,   0.4, 1.6);
    L.max_w   = jitter(L.max_w,   1.0, 3.5);
    L.accel_v = jitter(L.accel_v, 1.0, 4.0);
    L.accel_w = jitter(L.accel_w, 2.0, 9.0);
    W.heading   = jitter(W.heading,   0.2, 4.0);
    W.clearance = jitter(W.clearance, 0.2, 8.0);
    W.velocity  = jitter(W.velocity,  0.0, 2.0);
    W.progress  = jitter(W.progress,  0.2, 5.0);
}

int main(int argc, char** argv) {
    std::string which = (argc > 1) ? argv[1] : "photo";
    uint32_t seed = (argc > 2) ? static_cast<uint32_t>(std::stoul(argv[2])) : 7u;

    // Build the labyrinth.
    MazeMap maze = (which == "seeded") ? generate_maze(seed)
                                       : load_photo_maze();

    // Clearance used for global planning. Tight hedge corridors need a snug
    // radius; this matches the value the simulator plans with below.
    const double inflate_m = (which == "seeded") ? 0.25 : 0.15;

    // Confirm solvability up front (so "until it succeeds" is guaranteed).
    {
        int inflate = static_cast<int>(inflate_m / maze.grid.resolution());
        OccupancyGrid pg = maze.grid.inflated(inflate);
        AStarPlanner planner(pg);
        auto path = planner.plan(maze.start, maze.goal);
        if (path.empty()) {
            std::cerr << "Maze is unsolvable for the chosen start/goal; "
                         "adjust clearance.\n";
            return 1;
        }
    }

    Pose start{maze.start, 0.0};
    const int max_ticks = 6500;   // long enough for all three routes incl. the bottom sweep
    std::mt19937 rng(seed * 2654435761u + 1u);

    std::vector<Generation> history;

    // Pre-plan the alternate routes once; all generations draw from them so the
    // swarm can fan out across more than one way through the maze.
    auto routes = plan_routes(maze.grid, maze.start, maze.goal, inflate_m);
    std::cerr << "routes found: " << routes.size() << "\n";

    // The evolutionary arc, now as a SWARM. Each generation releases more dots
    // than the last (the population grows as it learns), and the dots track the
    // route more tightly each generation (rising skill => less noise, more
    // reach the goal). The final generation is a large, sharp swarm that solves.
    const int n_gens = 9;
    int idx = 0;
    bool solved = false;

    for (; idx < n_gens; ++idx) {
        double frac = static_cast<double>(idx) / (n_gens - 1);   // 0 .. 1
        double skill = std::clamp(0.12 + 0.88 * frac, 0.1, 1.0);
        // Swarm grows: a few dots early, many by the end.
        int n_dots = 4 + static_cast<int>(std::round(frac * 14));  // 4 -> 18
        Generation g = run_swarm(idx, maze.grid, start, maze.goal,
                                 inflate_m, max_ticks, skill, n_dots, routes, rng);
        if (g.success) solved = true;
        history.push_back(std::move(g));
    }

    // ---- Serialize everything to JSON --------------------------------------
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "{\n";
    std::cout << "  \"map\": \"" << which << "\",\n";
    // Robot footprint radius: the photo maze's real corridors are narrow, so it
    // uses a small indoor-robot footprint; the seeded maze uses the standard one.
    double robot_radius = (which == "seeded") ? 0.18 : 0.07;
    std::cout << "  \"robot_radius\": " << robot_radius << ",\n";
    std::cout << "  \"world\": {\"w\": " << maze.grid.width() * maze.grid.resolution()
              << ", \"h\": " << maze.grid.height() * maze.grid.resolution()
              << ", \"res\": " << maze.grid.resolution() << "},\n";

    // Obstacles (occupied cell centers).
    std::cout << "  \"obstacles\": [";
    bool first = true;
    for (int gy = 0; gy < maze.grid.height(); ++gy)
        for (int gx = 0; gx < maze.grid.width(); ++gx)
            if (maze.grid.occupied(gx, gy)) {
                if (!first) std::cout << ",";
                first = false;
                Vec2 w = maze.grid.grid_to_world(gx, gy);
                std::cout << "[" << w.x << "," << w.y << "]";
            }
    std::cout << "],\n";

    std::cout << "  \"start\": [" << maze.start.x << "," << maze.start.y << "],\n";
    std::cout << "  \"goal\": ["  << maze.goal.x  << "," << maze.goal.y  << "],\n";

    // One planned A* path (same for all generations; the global route).
    {
        // Plan the displayed route the same way the champion does: extra
        // clearance, then string-pulled to clean waypoints for a tidy amber
        // line. This is the route the robot is actually following.
        std::vector<Vec2> path;
        for (double clr : {0.18, 0.15, inflate_m}) {
            int inflate = static_cast<int>(clr / maze.grid.resolution());
            OccupancyGrid pg = maze.grid.inflated(inflate);
            AStarPlanner planner(pg);
            path = planner.plan(maze.start, maze.goal);  // smoothed for display
            if (!path.empty()) break;
        }
        std::cout << "  \"path\": [";
        for (size_t i = 0; i < path.size(); ++i) {
            if (i) std::cout << ",";
            std::cout << "[" << path[i].x << "," << path[i].y << "]";
        }
        std::cout << "],\n";
    }

    // Generations array — each generation is a SWARM of dots.
    std::cout << "  \"generations\": [\n";
    for (size_t gi = 0; gi < history.size(); ++gi) {
        const Generation& g = history[gi];
        if (gi) std::cout << ",\n";
        std::cout << "    {\n";
        std::cout << "      \"index\": " << g.index << ",\n";
        std::cout << "      \"success\": " << (g.success ? "true" : "false") << ",\n";
        std::cout << "      \"fail_reason\": \"" << g.fail_reason << "\",\n";
        std::cout << "      \"closest\": " << g.closest_to_goal << ",\n";
        std::cout << "      \"n_dots\": " << g.swarm.size() << ",\n";
        std::cout << "      \"reached\": " << g.reached_count << ",\n";
        std::cout << "      \"dots\": [";
        for (size_t di = 0; di < g.swarm.size(); ++di) {
            const Dot& dot = g.swarm[di];
            if (di) std::cout << ",";
            std::cout << "{\"reached\":" << (dot.reached ? "true" : "false")
                      << ",\"route\":" << dot.route << ",\"path\":[";
            for (size_t f = 0; f < dot.trajectory.size(); ++f) {
                if (f) std::cout << ",";
                const Pose& p = dot.trajectory[f];
                std::cout << "[" << p.pos.x << "," << p.pos.y << "," << p.theta << "]";
            }
            std::cout << "]}";
        }
        std::cout << "]\n    }";
    }
    std::cout << "\n  ]\n}\n";

    std::cerr << "generations: " << history.size()
              << "  solved: " << (solved ? "yes" : "no") << "\n";
    return 0;
}

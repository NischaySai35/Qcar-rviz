// Offline scenario test for shield_core.hpp -- no ROS, no car.
//   g++ -O2 -std=c++17 -I../src test_shield_core.cpp -o /tmp/test_shield && /tmp/test_shield
//
// A small world: the car (bicycle model, accel/brake limited, motor command
// applied after a delay), obstacles made of points (people = small discs that
// can walk/run/stop, walls = point lines), a LiDAR sampled at 20 Hz with
// range noise.  Nav2 is modelled as "keep driving at the requested speed"
// whenever the shield is idle.  The test fails if the car touches anything
// it was not pinned against, or ends up doing the wrong thing.
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <random>

#include "shield_core.hpp"

using shield::Point;

struct Body
{
    // Disc (person/pole) when radius > 0, else a wall segment a->b.
    double x, y, radius;
    double ax, ay, bx, by;
    std::function<void(Body &, double t, double dt)> move;
};

struct Car
{
    double x = 0, y = 0, th = 0, v = 0, steer = 0;
};

struct Result
{
    double min_gap = 99;          // over the run, to every body
    double contact_t = -1, clear_t = -1;
    double final_x = 0, final_y = 0;
    bool trapped_seen = false, escaped = false;
    int trapped_count = 0;
    double max_speed = 0;         // fastest the car went (|v|)
    // SMOOTHNESS (the 2026-10-09 complaint): how the COMMAND behaves.
    double max_step = 0;          // largest scan-to-scan speed-command change, outside emergencies
    int flips = 0;                // forward<->reverse command reversals
    int mode_changes = 0;         // idle/hold/escape switches
    double steer_travel = 0;      // total steering command movement (rad)
    std::string last_reason;
};

static const double kRealBrake = 1.8;        // pessimistic: model assumes 2.0
static const double kCmdDelay = 0.06;        // command -> wheels

// Gap from a body to the car rectangle, in world.
static double body_gap(const shield::Config & c, const Car & car, const Body & b)
{
    const double ct = std::cos(car.th), st = std::sin(car.th);
    auto local = [&](double wx, double wy, double & lx, double & ly) {
        lx = ct * (wx - car.x) + st * (wy - car.y);
        ly = -st * (wx - car.x) + ct * (wy - car.y);
    };
    if (b.radius > 0)
    {
        double lx, ly;
        local(b.x, b.y, lx, ly);
        return std::max(0.0, shield::rect_gap(c, lx, ly) - b.radius);
    }
    double best = 99;
    for (int i = 0; i <= 200; ++i)
    {
        const double f = i / 200.0;
        double lx, ly;
        local(b.ax + f * (b.bx - b.ax), b.ay + f * (b.by - b.ay), lx, ly);
        best = std::min(best, shield::rect_gap(c, lx, ly));
    }
    return best;
}

// LiDAR at base_link (0.1, 0): 720 beams, ray-cast against bodies, noise.
// Returns beam bearings (base_link) and ranges; no return = 0 (as the
// RPLIDAR does inside its ~0.15 m blind zone).
static void scan(const Car & car, const std::vector<Body> & bodies, std::mt19937 & rng,
                 std::vector<double> & angles, std::vector<double> & ranges)
{
    std::normal_distribution<double> noise(0.0, 0.012);
    // People are not rigid discs: legs swing, so what the LiDAR sees of a
    // person shifts by a couple of cm every scan.  Without this the tracker
    // would look better here than it can on the car.
    std::normal_distribution<double> sway(0.0, 0.02);
    std::vector<Body> seen = bodies;
    for (auto & b : seen)
        if (b.radius > 0)
        {
            b.x += sway(rng);
            b.y += sway(rng);
        }
    angles.clear();
    ranges.clear();
    const double lx0 = car.x + 0.1 * std::cos(car.th), ly0 = car.y + 0.1 * std::sin(car.th);
    for (int i = 0; i < 720; ++i)
    {
        angles.push_back(i * 2 * M_PI / 720);
        ranges.push_back(0.0);
        const double a = car.th + i * 2 * M_PI / 720;
        const double dx = std::cos(a), dy = std::sin(a);
        double r = 1e9;
        for (const auto & b : seen)
        {
            if (b.radius > 0)
            {
                const double fx = lx0 - b.x, fy = ly0 - b.y;
                const double bb = fx * dx + fy * dy, cc = fx * fx + fy * fy - b.radius * b.radius;
                const double disc = bb * bb - cc;
                if (disc >= 0)
                {
                    const double t = -bb - std::sqrt(disc);
                    if (t > 0) r = std::min(r, t);
                }
            }
            else
            {
                const double ex = b.bx - b.ax, ey = b.by - b.ay;
                const double den = dx * ey - dy * ex;
                if (std::abs(den) < 1e-9) continue;
                const double t = ((b.ax - lx0) * ey - (b.ay - ly0) * ex) / den;
                const double u = ((b.ax - lx0) * dy - (b.ay - ly0) * dx) / den;
                if (t > 0 && u >= 0 && u <= 1) r = std::min(r, t);
            }
        }
        if (r > 6.0 || r < 0.15)
            continue;                         // out of range / LiDAR blind zone
        ranges.back() = r + noise(rng);
    }
}

// Called every sim step with the time, the car and the bodies (for checks
// at particular moments of a scripted scene).
using Probe = std::function<void(double, const Car &, const std::vector<Body> &)>;

static Result run(const char * name, std::vector<Body> bodies, double nav_speed, double seconds,
                  std::vector<int> pinned = {}, Probe probe = nullptr)
{
    shield::Core core;
    shield::ScanMemory memory;
    std::vector<double> angles, ranges;
    const auto & c = core.config();
    // Every scene is inside a 8 x 8 m room, so beams have walls to return
    // from (that is what clears remembered points).
    const size_t n_scene = bodies.size();
    bodies.push_back({0, 0, 0, -4, -4, 4, -4, nullptr});
    bodies.push_back({0, 0, 0, 4, -4, 4, 4, nullptr});
    bodies.push_back({0, 0, 0, 4, 4, -4, 4, nullptr});
    bodies.push_back({0, 0, 0, -4, 4, -4, -4, nullptr});
    for (size_t i = n_scene; i < bodies.size(); ++i)
        pinned.push_back(static_cast<int>(i));
    Car car;
    car.v = nav_speed;
    std::mt19937 rng(7);
    Result res;
    double contact_speed = -1.0;
    double cmd_v = nav_speed, cmd_s = 0;
    struct Pending { double t, v, s; };
    std::deque<Pending> pending;
    double next_scan = 0;
    shield::Decision d;
    bool have_prev = false;
    double prev_v = 0.0, prev_s = 0.0;
    shield::Mode prev_mode = shield::Mode::Idle;
    const double dt = 0.005;
    for (double t = 0; t < seconds; t += dt)
    {
        for (auto & b : bodies)
            if (b.move) b.move(b, t, dt);
        if (t >= next_scan)
        {
            next_scan += 0.05;
            scan(car, bodies, rng, angles, ranges);
            const auto pts = memory.update(t, car.x, car.y, car.th, 0.1, 0.0, angles, ranges);
            d = core.update(t, car.x, car.y, car.th, pts, {}, car.v, car.steer, nav_speed != 0.0);
            double v = nav_speed, s = 0;
            if (d.mode != shield::Mode::Idle)
            {
                v = d.speed;
                s = d.steering;
            }
            if (have_prev)
            {
                // The shield's own commands only: a hand-back to Nav2 is ramped
                // by the hardware on the car (the sim's Nav2 just jumps).
                if (!d.emergency && d.mode != shield::Mode::Idle && prev_mode != shield::Mode::Idle)
                    res.max_step = std::max(res.max_step, std::abs(v - prev_v));
                if (v * prev_v < 0.0)
                    ++res.flips;
                if (d.mode != prev_mode)
                    ++res.mode_changes;
                res.steer_travel += std::abs(s - prev_s);
            }
            have_prev = true;
            prev_v = v;
            prev_s = s;
            prev_mode = d.mode;
            if (std::getenv("TRACE") && std::string(name).rfind(std::getenv("TRACE"), 0) == 0)
                std::printf("  t=%.2f car x=%.3f v=%.2f st=%.2f | gap %.3f app %.2f movers %d tgt %.2f -> %s v=%.2f s=%.2f\n", t,
                            car.x, car.v, car.steer, d.min_gap, d.max_approach, d.movers, d.target_speed, d.reason.c_str(), d.speed, d.steering);
            if (d.trapped) { res.trapped_seen = true; ++res.trapped_count; }
            if (d.mode == shield::Mode::Escape) res.escaped = true;
            res.last_reason = d.reason;
            pending.push_back({t + kCmdDelay, v, s});
        }
        while (!pending.empty() && pending.front().t <= t)
        {
            cmd_v = pending.front().v;
            cmd_s = pending.front().s;
            pending.pop_front();
        }
        // Wheels: brake/accelerate toward command; servo slews 4 rad/s.
        car.v += std::clamp(cmd_v - car.v, -kRealBrake * dt, kRealBrake * dt);
        res.max_speed = std::max(res.max_speed, std::abs(car.v));
        car.steer += std::clamp(cmd_s - car.steer, -4.0 * dt, 4.0 * dt);
        const double rx = car.x - 0.12765 * std::cos(car.th);
        const double ry = car.y - 0.12765 * std::sin(car.th);
        const double nrx = rx + car.v * std::cos(car.th) * dt, nry = ry + car.v * std::sin(car.th) * dt;
        car.th += car.v * std::tan(car.steer) / c.wheelbase * dt;
        car.x = nrx + 0.12765 * std::cos(car.th);
        car.y = nry + 0.12765 * std::sin(car.th);
        if (probe)
            probe(t, car, bodies);
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            bool is_pinned = false;
            for (int p : pinned) if (p == (int)i) is_pinned = true;
            if (!is_pinned)
            {
                const double g = body_gap(c, car, bodies[i]);
                if (g <= 0.0 && contact_speed < 0.0)
                {
                    contact_speed = std::abs(car.v);
                    res.contact_t = t;
                }
                if (res.contact_t >= 0.0 && res.clear_t < 0.0 && g >= c.margin)
                    res.clear_t = t;           // back outside the margin after contact
                res.min_gap = std::min(res.min_gap, g);
            }
        }
    }
    res.final_x = car.x;
    res.final_y = car.y;
    std::printf("%-46s min gap %5.3f m  end (%5.2f,%5.2f) v=%5.2f  %strapped x%-3d last=%s", name, res.min_gap,
                car.x, car.y, car.v, res.escaped ? "ESCAPED " : "", res.trapped_count,
                res.last_reason.c_str());
    if (contact_speed >= 0.0)
        std::printf("  CONTACT at car speed %.2f m/s", contact_speed);
    std::printf("\n%-46s   smooth: max step %.2f m/s  flips %d  mode changes %d  steer travel %.2f rad\n", "",
                res.max_step, res.flips, res.mode_changes, res.steer_travel);
    return res;
}

static Body person(double x, double y, double vx = 0, double vy = 0, double stop_after = 99)
{
    Body b{x, y, 0.12, 0, 0, 0, 0, nullptr};
    b.move = [vx, vy, stop_after](Body & me, double t, double dt) {
        if (t < stop_after) { me.x += vx * dt; me.y += vy * dt; }
    };
    return b;
}
static Body wall(double ax, double ay, double bx, double by)
{
    return Body{0, 0, 0, ax, ay, bx, by, nullptr};
}
// Appears suddenly at time `when` (jumps in).
static Body jumper(double x, double y, double when)
{
    Body b{100, 100, 0.12, 0, 0, 0, 0, nullptr};
    b.move = [x, y, when](Body & me, double t, double) {
        if (t >= when) { me.x = x; me.y = y; }
    };
    return b;
}

int main()
{
    int fails = 0;
    auto expect = [&](bool ok, const char * what) {
        if (!ok) { std::printf("   FAIL: %s\n", what); ++fails; }
    };
    // Front bumper is at x = +0.22, rear at -0.21, sides at |y| = 0.12.
    // Person radius 0.12, so centre x = 0.22 + gap + 0.12.

    // 1. Car at full nav speed, someone jumps in ahead of the bumper.
    //    (car is at x = 0.45*0.5 when they appear)
    {
        auto r = run("1 jump 15cm ahead @0.45 m/s", {jumper(0.22 + 0.15 + 0.12 + 0.45 * 0.5, 0, 0.5)}, 0.45, 4);
        expect(r.min_gap > 0.0, "touched the person");
        expect(r.escaped, "never escaped");
        // 5 cm at 0.45 m/s: the car covers 5 cm in the ~0.11 s it takes to
        // even see them and start braking -- reported, not asserted.
        // What it MUST do: back straight off, without calling itself
        // trapped (it is not), and be outside the margin within 1 s.
        auto rb = run("1b jump 5cm ahead @0.45 m/s (physics limit)", {jumper(0.22 + 0.05 + 0.12 + 0.45 * 0.5, 0, 0.5)}, 0.45, 4);
        expect(rb.trapped_count == 0, "called itself trapped after the jump-in");
        expect(rb.clear_t > 0 && rb.clear_t - rb.contact_t < 1.0, "did not back off within 1 s of contact");
        if (rb.contact_t >= 0)
            std::printf("   contact at %.2f s, back past 8 cm at %.2f s (%.2f s later)\n", rb.contact_t, rb.clear_t,
                        rb.clear_t - rb.contact_t);
    }
    // 2. Person walks at the parked car at 1 m/s and keeps coming 1.2 s.
    {
        auto r = run("2 walk at it 1.0 m/s", {person(1.2, 0, -1.0, 0, 1.2)}, 0.0, 4);
        expect(r.min_gap > 0.0, "touched the person");
        expect(r.final_x < -0.2, "did not back away");
    }
    // 3. Runner at 3 m/s from 2.5 m away, stops 1 m later.
    {
        auto r = run("3 run at it 3.0 m/s", {person(2.5, 0, -3.0, 0, 0.55)}, 0.0, 4);
        expect(r.min_gap > 0.0, "touched the runner");
    }
    // 4. Walk at it with a wall 0.40 m behind the rear bumper.
    {
        auto r = run("4 walk at it, wall 0.40 behind", {person(1.0, 0, -0.8, 0, 0.9), wall(-0.61, -2, -0.61, 2)}, 0.0, 5);
        expect(r.min_gap > 0.0, "touched something");
        auto rw = run("4b  (gap to wall only)", {wall(-0.61, -2, -0.61, 2), person(1.0, 0, -0.8, 0, 0.9)}, 0.0, 5, {1});
        expect(rw.min_gap >= 0.06, "went closer than ~8 cm to the wall behind");
    }
    // 5. Front-right diagonal approach.
    {
        auto r = run("5 diagonal from front-right", {person(0.8, -0.6, -0.6, 0.45, 1.3)}, 0.0, 4);
        expect(r.min_gap > 0.0, "touched the person");
    }
    // 6. Front and back both inside 5 cm: nowhere to go.
    {
        auto r = run("6 front + back touching", {person(0.22 + 0.04 + 0.12, 0), person(-0.21 - 0.04 - 0.12, 0)}, 0.0, 3, {0, 1});
        expect(r.trapped_seen, "did not report trapped");
        expect(std::abs(r.final_x) < 0.02, "moved while boxed in");
    }
    // 7. Boxed on all four sides.
    {
        auto r = run("7 all sides", {person(0.40, 0), person(-0.39, 0), person(0, 0.27), person(0, -0.27)}, 0.0, 3, {0, 1, 2, 3});
        expect(r.trapped_seen, "did not report trapped");
    }
    // 8. Boxed front+back, then the back person walks away at t=1 s.
    {
        Body back = person(-0.21 - 0.04 - 0.12, 0);
        back.move = [](Body & me, double t, double dt) { if (t > 1.0) me.y += 1.0 * dt; };
        auto r = run("8 front+back, back leaves at 1 s", {person(0.22 + 0.04 + 0.12, 0), back}, 0.0, 4, {0, 1});
        expect(r.final_x < -0.03, "did not move back once the back opened");
    }
    // 9. Nav driving at a still wall at 0.45 m/s (clearance cap absent).
    {
        auto r = run("9 drive at a wall 0.45 m/s", {wall(0.9, -2, 0.9, 2)}, 0.45, 4);
        expect(r.min_gap > 0.0, "hit the wall");
    }
    // 10. Person ALREADY walking briskly (0.8 m/s) at the middle of the car's
    //     side from only 73 cm away.  The car must move its whole length
    //     (~0.42 m) out of their path in ~0.95 s, starting ~0.2 s late (it
    //     needs a few scans to see them walking): at the physical limit of a
    //     0.6 m/s car -- reported, not asserted.
    run("10 from the left side 0.8 m/s, from 73cm (physics limit)", {person(0.0, 1.0, 0, -0.8, 1.0)}, 0.0, 4);
    // 10b. The realistic version: the same brisk walk into the side, starting
    //      1.5 m out -- the car starts easing away early and must not be
    //      touched.
    {
        auto r = run("10b from the left side 0.8 m/s, from 1.5m", {person(0.0, 1.75, 0, -0.8, 1.95)}, 0.0, 5);
        expect(r.min_gap > 0.0, "touched the person");
    }
    // 10c. Same from the right, aimed at the front corner (a weak spot).
    {
        auto r = run("10c from the right at the front corner", {person(0.25, -1.75, 0, 0.8, 1.95)}, 0.0, 5);
        expect(r.min_gap > 0.0, "touched the person");
    }
    // 10d. Aimed at the rear-left corner.
    {
        auto r = run("10d from the left at the rear corner", {person(-0.25, 1.75, 0, -0.8, 1.95)}, 0.0, 5);
        expect(r.min_gap > 0.0, "touched the person");
    }
    // 11. Nothing near: shield must stay idle at nav speed.
    {
        auto r = run("11 open room, cruising", {wall(5, -5, 5, 5)}, 0.3, 2);
        expect(!r.escaped, "escaped from nothing");
    }
    // 12. Front + right walk in together; wall 0.5 m behind.
    {
        auto r = run("12 front + right, wall behind", {person(0.9, 0, -0.7, 0, 0.9), person(0.0, -0.9, 0, 0.7, 0.85), wall(-0.71, -2, -0.71, 2)}, 0.0, 4);
        expect(r.min_gap > 0.0, "touched something");
    }
    // 13. Cruising down a corridor with 23 cm each side: no false alarm.
    {
        auto r = run("13 corridor 23cm each side @0.3", {wall(-2, 0.35, 4, 0.35), wall(-2, -0.35, 4, -0.35)}, 0.3, 3);
        expect(!r.escaped, "ran from corridor walls");
        expect(r.min_gap > 0.2, "left its lane");
    }
    // 14. Something standing still 20 cm in front of the parked car: inside
    //     the parked front buffer (30 cm), so a calm creep back to ~30 cm --
    //     never a run.
    {
        auto r = run("14 something still 20cm ahead", {person(0.22 + 0.20 + 0.12, 0)}, 0.0, 4);
        const double gap_end = 0.20 - r.final_x;
        std::printf("   final gap %.2f m, max speed %.2f\n", gap_end, r.max_speed);
        expect(gap_end > 0.30 && gap_end < 0.42, "did not settle at the parked buffer (35 cm)");
        expect(r.max_speed < 0.20, "moved briskly for a small adjustment");
    }
    // 17. THE 2026-10-09 SCREENSHOT: someone walks up slowly and stands
    //     12 cm in front of the parked car.  It must back off -- smoothly --
    //     to the person buffer (30 cm front) and stop; not freeze, not run.
    {
        auto r = run("17 walk up, stand 12cm in front", {person(1.0, 0, -0.3, 0, 1.8)}, 0.0, 6);
        const double gap_end = (1.0 - 0.3 * 1.8 - 0.12) - (r.final_x + 0.22);
        std::printf("   final gap %.2f m, car moved %.2f m, max speed %.2f\n", gap_end, -r.final_x, r.max_speed);
        expect(r.min_gap > 0.0, "touched the person");
        expect(gap_end > 0.35 && gap_end < 0.60, "did not settle at the person buffer in front (40 cm)");
        expect(r.max_speed < 0.40, "too brisk for a slow walk-up");
    }
    // 18. Same from behind: drives forward.
    {
        auto r = run("18 walk up, stand 12cm behind", {person(-1.0, 0, 0.3, 0, 1.8)}, 0.0, 6);
        const double gap_end = (r.final_x - 0.21) - (-1.0 + 0.3 * 1.8 + 0.12);
        std::printf("   final gap %.2f m, car moved %.2f m\n", gap_end, r.final_x);
        expect(r.min_gap > 0.0, "touched the person");
        expect(gap_end > 0.35 && gap_end < 0.60, "did not settle at the person buffer behind (40 cm)");
    }
    // 19. Walk at it from the front while it sits in a narrow corridor
    //     (walls 10 cm either side): reversing must not be choked by the
    //     side walls (it was: "bad at going back").
    {
        auto r = run("19 front walker, walls 10cm each side",
                     {person(1.0, 0, -0.4, 0, 1.6), wall(-3, 0.22, 2, 0.22), wall(-3, -0.22, 2, -0.22)}, 0.0, 5);
        expect(r.min_gap > 0.0, "touched something");
        expect(r.final_x < -0.25, "did not back away down the corridor");
    }
    // 20. Parked with a wall 9 cm beside it: at most a small, calm nudge.
    {
        auto r = run("20 parked, wall 9cm beside", {wall(-2, 0.21, 2, 0.21)}, 0.0, 4);
        expect(r.min_gap > 0.05, "went closer to the wall");
        expect(std::hypot(r.final_x, r.final_y) < 0.30, "wandered off");
    }
    // 21. THE OPERATOR'S SQUEEZE EXAMPLE (2026-10-09): wall 45 cm behind the
    //     parked car.  A person walks in and keeps pressing toward it, then
    //     stands; steps back 10 cm; another 10 cm; then 1 m.  Expected:
    //     never touched; while squeezed the car sits HALF-WAY between person
    //     and wall; each 10 cm step back brings it ~5 cm forward; once the
    //     person is gone it keeps the parked buffer from the wall.
    {
        Body walker{1.2, 0, 0.12, 0, 0, 0, 0, nullptr};
        // Script: walk in at 0.3 m/s to x = 0.30 (surface 0.08 in front of
        // where the bumper started), stand, back 0.10, stand, back 0.10,
        // stand, then back 1 m.
        walker.move = [](Body & me, double t, double dt) {
            auto toward = [&](double goal, double speed) {
                me.x += std::clamp(goal - me.x, -speed * dt, speed * dt);
            };
            if (t < 4.0) toward(0.30, 0.3);
            else if (t < 7.0) toward(0.40, 0.2);
            else if (t < 10.0) toward(0.50, 0.2);
            else toward(1.6, 0.5);
        };
        const double wall_x = -0.21 - 0.45;
        double mid_err = 99, x_at_7 = 0, x_at_10 = 0, x_at_4 = 0, wall_gap_end = 0;
        auto probe = [&](double t, const Car & car, const std::vector<Body> & b) {
            const double front = (b[0].x - 0.12) - (car.x + 0.22);
            const double back = (car.x - 0.21) - wall_x;
            if (std::abs(t - 3.95) < 0.003) { mid_err = std::abs(front - back); x_at_4 = car.x; }
            if (std::abs(t - 6.95) < 0.003) x_at_7 = car.x;
            if (std::abs(t - 9.95) < 0.003) x_at_10 = car.x;
            wall_gap_end = back;
        };
        auto r = run("21 squeeze: person in front, wall behind", {walker, wall(wall_x, -2, wall_x, 2)}, 0.0, 14,
                     {}, probe);
        std::printf("   squeezed: front-vs-back gap difference %.3f m; step back 10cm -> car +%.3f m; "
                    "again -> +%.3f m; finally %.2f m from the wall\n",
                    mid_err, x_at_7 - x_at_4, x_at_10 - x_at_7, wall_gap_end);
        expect(r.min_gap > 0.0, "touched the person or the wall");
        expect(mid_err < 0.04, "did not sit half-way while squeezed");
        expect(x_at_7 - x_at_4 > 0.02 && x_at_7 - x_at_4 < 0.09, "did not come ~5 cm forward after a 10 cm step back");
        expect(wall_gap_end > 0.30 && wall_gap_end < 0.42, "did not keep the parked buffer (35 cm) from the wall");
    }
    // 16. THE 2026-10-09 LIVE CASE: walk slowly (0.3 m/s) at the parked car,
    //     then stop.  Must back off gently -- nowhere near full speed -- and
    //     stop about keep_gap from the person, not run on.
    {
        auto r = run("16 slow walk 0.3 m/s then stop", {person(1.0, 0, -0.3, 0, 2.5)}, 0.0, 6);
        // Person ends at x = 0.25 (surface 0.13); car must end ~0.2 m clear.
        const double gap_end = (0.25 - 0.12) - (r.final_x + 0.22);
        std::printf("   max car speed %.2f m/s, final gap %.2f m, car moved %.2f m\n", r.max_speed, gap_end,
                    -r.final_x);
        expect(r.min_gap > 0.0, "touched the walker");
        expect(r.max_speed < 0.45, "backed off much faster than a slow walk needs");
        // It answers early (buffer zone), so it holds roughly where the
        // walker's approach was first matched -- what matters is that it
        // STOPS when they stop and never runs on.
        expect(gap_end > 0.35 && gap_end < 0.60, "did not hold a sensible distance from the person");
    }
    // 15. Someone follows the car at 0.5 m/s for 3 s in open space.
    {
        auto r = run("15 follower 0.5 m/s for 3 s", {person(0.9, 0, -0.5, 0, 3.0)}, 0.0, 5);
        expect(r.min_gap > 0.0, "the follower caught it");
    }
    std::printf(fails ? "\n%d FAILURES\n" : "\nALL PASSED\n", fails);
    return fails ? 1 : 0;
}

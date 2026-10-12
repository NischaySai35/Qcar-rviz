// SAFETY SHIELD -- decision logic (no ROS, so it can be tested offline:
// see test/test_shield_core.cpp).
//
// Goal set by the operator (2026-10-09): during navigation the car must not
// touch ANYTHING -- a wall, a table, a person standing still, a person who
// walks, runs or jumps at it -- and it must get out of the way SMOOTHLY:
// sense what is coming from far off, ease away early, hold a steady distance
// from people, and never jerk, jitter or slam.
//
// Version 2 (same day, after the first live tests).  Version 1 was a trigger
// plus a fresh discrete choice every scan: nothing until a person crossed a
// line, then full reaction; it needed ~0.3 s of history before it believed
// anything was approaching; tiny score differences flipped speed and
// steering scan to scan (158 mode switches in 3 minutes, command jumping
// 0.12 -> 0.60 -> 0.46 -> 0.60 -> 0.23 -> hard brake in 0.4 s); and every
// "safe again" was a hard stop -- wheel slip that smeared the map.
//
// Now, every scan:
//   1. TRACK   -- LiDAR points are grouped into objects (a person, a leg, a
//                 chair) and followed scan to scan in the odom frame with an
//                 alpha-beta filter, so each object has a smooth velocity
//                 within ~2 scans, however far away it is.  Big objects
//                 (walls) are never tracked -- they cannot walk.  An object
//                 that has really moved is a MOVER for as long as it is seen.
//   2. FIELD   -- every obstacle point pushes the car away from it with a
//                 strength that rises SMOOTHLY through a buffer zone, not at
//                 a line: for a mover from keep_gap + mover_zone (further
//                 out the faster it approaches -- it is judged by where it
//                 will be look_ahead seconds from now) to full at keep_gap;
//                 for anything still, only inside margin + still_zone.
//                 Approaching movers add their own speed, so the car keeps
//                 pace.  Pushes from every side add up: front and back
//                 cancel, a person front-right pushes back-left.
//   3. MOTION  -- the push becomes a speed (its fore-aft part) and a
//                 steering angle (its sideways part: on this Ackermann car
//                 steering toward a side moves the car to that side whether
//                 it drives forward or back).  Speed is capped so the car can
//                 always stop before the margin to anything STILL in its
//                 path (the wall behind), and an arc that would graze
//                 something still is swapped for the nearest one that does
//                 not.
//   4. SMOOTH  -- the command follows that target at limited acceleration
//                 and deceleration, steering through a filter and a rate
//                 limit.  It eases to a stop; the hard brake (emergency) is
//                 reserved for contact that is about to happen.
//   5. HAND-BACK -- once the target and the command have both been zero for
//                 release_hold, Nav2 drives again.
//
// Everything is in base_link: x forward, y left, steering positive = left.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace shield
{

struct Point
{
    double x, y;
};

struct Config
{
    // Car outline (base_link), from the Nav2 footprint.
    double front = 0.22;
    double rear = -0.21;
    double half_width = 0.12;
    double wheelbase = 0.25725;
    double rear_axle_x = -0.12765;
    double max_steer = 0.5236;          // mechanical stop
    // Distances.
    double margin = 0.08;               // never closer than this to anything (m)
    // Car-shaped buffer -- see keep_distance().
    // (Raised after the 4th live test: "not moving away far enough".)
    // (Raised after the 4th and 5th live tests: "not far enough".)
    double keep_front_mover = 0.50;     // people: front/back (m)
    double keep_side_mover = 0.30;      // people: sides
    double keep_front_still = 0.35;     // anything else, car parked: front/back
    // anything else: sides.  Just above the margin: driving along a wall
    // never widens the gap beside it, so a bigger side buffer would only send
    // the car off along the wall forever (offline test, parked 9 cm from one).
    double keep_side_still = 0.085;
    double keep_front_driving = 0.14;   // anything else while Nav2 drives: front/back
    double keep_gap = 0.20;             // reference distance (speed scaling, steering score)
    // ... and it starts EARLY: a person walking in is answered from ~1 m,
    // gently at first (smooth), fully by the keep distance.
    double mover_zone = 0.45;           // a mover's push starts this far beyond its keep distance ...
    double look_ahead = 0.30;           // ... judged by where it will be in this (s)
    double look_ahead_fast = 0.40;      // + this much more for a fast approach
    // Speeds -- smooth and sensible, not fast (operator, 2026-10-09: "smoothness
    // and smartness is important, not fastness; just fast processing").
    double v_push = 0.20;               // speed from the distance push at full strength (m/s)
    double centre_gain = 3.0;           // squeezed: m/s per metre off the middle
    double centre_deadband = 0.01;      // squeezed: this close to the middle is the middle
    double squeeze_range = 0.55;        // front/back: "near the other end" for a squeeze (m)
    double squeeze_range_side = 0.30;   // sides
    double approach_calm = 0.15;        // below this nobody counts as still coming (m/s)
    double side_move_min = 0.08;        // a purely sideways push below this is ignored (m/s)
    double v_creep = 0.10;              // slowest speed the motor holds smoothly (m/s)
    double side_room_min = 0.35;        // side escape: less room than this that way -> try the other
    int flip_scans = 5;                 // the other direction must win this many scans in a row
    double flip_push = 0.10;            // ... unless a front/back push this strong says so
    double creep_start = 0.04;          // a correction this strong is worth starting a creep for
    double room_weight = 0.10;          // steering: value of free room at the end of an arc
    // x approach speed of a mover inside the zone.  A little under 1: the gap
    // then closes gently toward the keep distance while they walk in, and
    // the car does not drift on after they stop (the speed estimate needs
    // ~0.25 s to see the stop).
    double approach_match = 0.85;
    double approach_match_fast = 1.2;   // ... rising to this for a fast approach (get clear)
    double v_max = 0.45;                // cap for a calm approach (= Nav2's own top speed)
    double accel_up = 0.8;              // command acceleration (m/s^2), below the odometry slip cap
    // Speed depends on the situation: someone coming FAST may raise the cap
    // and the acceleration, smoothly with their speed, up to these.
    double v_max_fast = 0.60;
    double accel_up_fast = 1.5;
    double fast_from = 0.5;             // approach (m/s) where it starts to scale up ...
    double fast_full = 1.0;             // ... and where it is fully scaled
    double decel = 1.2;                 // command deceleration, normal
    double decel_emergency = 3.0;       // command deceleration, contact imminent
    double steer_filter = 0.35;         // low-pass per scan toward the target steering
    double steer_rate = 2.5;            // rad/s
    double release_hold = 0.50;         // calm this long before handing back (s)
    double trapped_after = 0.40;        // stuck this long before it counts (and is said)
    // Stopping model for the "never into the wall behind" cap.
    double brake_decel = 1.4;           // m/s^2, deliberately pessimistic
    double reaction = 0.12;             // scan + compute + motor loop latency (s)
    double soft_decel = 1.0;            // normal (non-emergency) braking, assumed
    double horizon = 0.6;               // arc check length (s)
    double step = 0.05;
    double range = 2.5;                 // points further than this are ignored
    // Tracker.
    double cluster_link = 0.12;         // neighbouring points closer than this = same object
    double max_object = 0.9;            // objects bigger than this are walls: never movers
    double track_gate = 0.30;           // association radius (+ predicted travel)
    double track_window = 0.30;         // s of centroids in the velocity line fit
    double track_timeout = 0.3;         // s unseen -> forgotten
    double mover_memory = 3.0;          // s a lost person's spot is remembered
    double mover_distance = 0.15;       // net travel that makes an object a mover
    double mover_speed = 0.25;          // ... or this steady speed over a full window
    double speed_deadband = 0.12;       // tracked speeds below this are noise
};

enum class Mode
{
    Idle = 0,     // Nav2 drives
    Hold = 1,     // shield holds the car still / easing to a stop
    Escape = 2,   // shield drives (speed, steering)
};

struct Decision
{
    Mode mode = Mode::Idle;
    double speed = 0.0;                 // signed m/s (smoothed command)
    double steering = 0.0;              // rad, + = left
    double target_speed = 0.0;          // what the field asks for (unsmoothed)
    bool trapped = false;
    bool emergency = false;             // hard brake (else the gentle one)
    double min_gap = 99.0;
    double max_approach = 0.0;          // fastest mover approach (m/s)
    int movers = 0;                     // tracked moving objects in range
    bool side_front = false, side_back = false, side_left = false, side_right = false;
    std::string reason;
};

// LIDAR BLIND ZONE MEMORY.
// The LiDAR sits 0.12 m behind the front bumper and returns nothing closer
// than ~0.15 m, so anything within a few cm of the front bumper or the front
// corners VANISHES from the scan -- exactly when it matters most.  Points
// seen near the car are remembered in the odom frame (so they stay put while
// the car moves) and dropped only when a beam proves the spot is empty: a
// valid return further out along the same bearing.  A beam with no return
// does not clear (it may be the object itself, too close to measure).
class ScanMemory
{
public:
    double near_radius = 0.45;     // remember points this close to the LiDAR
    double blind_radius = 0.17;    // closer than this the LiDAR returns nothing
    // Only ever fills the BLIND zone, and briefly: the operator wants the
    // shield on live LiDAR, and a remembered point carries the odometry's
    // error -- during a wheel-spin (2026-10-09) stale points could ride along
    // with the car and keep it "escaping".
    double max_age = 1.5;          // s
    double clear_margin = 0.05;    // m, a return this far beyond clears

    // pose: base_link in odom.  lidar_x/y: LiDAR origin in base_link.
    // angles: beam bearings in BASE_LINK; ranges: <= 0 or non-finite = none.
    std::vector<Point> update(double t, double px, double py, double pth, double lidar_x, double lidar_y,
                              const std::vector<double> & angles, const std::vector<double> & ranges,
                              int * remembered_out = nullptr)
    {
        static constexpr int kBins = 720;
        std::array<double, kBins> far;          // furthest valid return per 0.5 deg bin
        far.fill(-1.0);
        auto bin_of = [](double a) {
            a = std::fmod(a, 2.0 * M_PI);
            if (a < 0.0)
                a += 2.0 * M_PI;
            return std::min(kBins - 1, static_cast<int>(a / (2.0 * M_PI) * kBins));
        };
        std::vector<Point> out;
        const double c = std::cos(pth), s = std::sin(pth);
        std::vector<Mem> fresh;
        // RANGE DENOISING: median of up to 5 neighbouring beams on the same
        // surface (within 5 cm of this one -- edges are kept sharp).  The
        // shield acts on the NEAREST of hundreds of points, and the nearest
        // of many noisy readings is biased ~2.5 sigma too close: a wall 9 cm
        // away read as 6 cm, inside the margin, and the parked car drove off
        // along it (offline test 2026-10-09).
        const size_t n_beams = std::min(angles.size(), ranges.size());
        std::vector<double> smooth(n_beams, 0.0);
        for (size_t i = 0; i < n_beams; ++i)
        {
            const double r = ranges[i];
            if (!std::isfinite(r) || r <= 0.0)
                continue;
            double w[5];
            int k = 0;
            for (int d = -2; d <= 2; ++d)
            {
                const double q = ranges[(i + n_beams + d) % n_beams];
                if (std::isfinite(q) && q > 0.0 && std::abs(q - r) < 0.05)
                    w[k++] = q;
            }
            std::nth_element(w, w + k / 2, w + k);
            smooth[i] = (k >= 3) ? w[k / 2] : r;
        }
        for (size_t i = 0; i < n_beams; ++i)
        {
            const double r = smooth[i];
            if (!std::isfinite(r) || r <= 0.0)
                continue;
            const int b = bin_of(angles[i]);
            far[b] = std::max(far[b], r);
            const double bx = lidar_x + r * std::cos(angles[i]);
            const double by = lidar_y + r * std::sin(angles[i]);
            out.push_back({bx, by});
            if (r < near_radius)
                fresh.push_back({px + c * bx - s * by, py + s * bx + c * by, t});
        }

        // Keep remembered points the scan cannot account for.
        std::vector<Mem> kept;
        int remembered = 0;
        for (const auto & m : mem_)
        {
            if (t - m.t > max_age)
                continue;
            const double dx = m.x - px, dy = m.y - py;
            const double bx = c * dx + s * dy, by = -s * dx + c * dy;
            const double lx = bx - lidar_x, ly = by - lidar_y;
            const double r = std::hypot(lx, ly);
            if (r > near_radius + 0.1)
                continue;
            // Cleared if a neighbouring beam (+-1 deg) saw past it.
            const int b = bin_of(std::atan2(ly, lx));
            bool cleared = false;
            for (int k = -2; k <= 2 && !cleared; ++k)
            {
                const double f = far[(b + k + kBins) % kBins];
                cleared = f > r + clear_margin;
            }
            if (cleared)
                continue;
            // In the visible band and the beam returned something nearer:
            // the scan already shows what is there now.
            const double f = far[b];
            if (r > blind_radius && f > 0.0 && f < r - clear_margin)
                continue;
            kept.push_back(m);
            if (r > blind_radius)
                continue;                        // the LiDAR sees this band live
            out.push_back({bx, by});
            ++remembered;
        }
        // Merge fresh points (2 cm grid) so memory does not grow per scan.
        for (const auto & m : fresh)
        {
            bool dup = false;
            for (auto & k : kept)
                if (std::abs(k.x - m.x) < 0.02 && std::abs(k.y - m.y) < 0.02)
                {
                    k = m;
                    dup = true;
                    break;
                }
            if (!dup)
                kept.push_back(m);
        }
        mem_.swap(kept);
        if (remembered_out)
            *remembered_out = remembered;
        return out;
    }

    void clear() { mem_.clear(); }

private:
    struct Mem
    {
        double x, y, t;
    };
    std::vector<Mem> mem_;
};

// Distance from p to the car rectangle (0 on or inside it), plus the unit
// vector from the nearest rectangle point to p.
inline double rect_gap(const Config & c, double px, double py, double * ux = nullptr, double * uy = nullptr)
{
    const double qx = std::clamp(px, c.rear, c.front);
    const double qy = std::clamp(py, -c.half_width, c.half_width);
    const double dx = px - qx, dy = py - qy;
    const double d = std::hypot(dx, dy);
    if (ux && uy)
    {
        if (d > 1e-6)
        {
            *ux = dx / d;
            *uy = dy / d;
        }
        else
        {
            // On/inside: push outward through the nearest edge.
            const double e[4] = {c.front - px, px - c.rear, c.half_width - py, py + c.half_width};
            const int k = static_cast<int>(std::min_element(e, e + 4) - e);
            const double vx[4] = {1, -1, 0, 0}, vy[4] = {0, 0, 1, -1};
            *ux = vx[k];
            *uy = vy[k];
        }
    }
    return d;
}

// Smooth 0 -> 1 as x goes 0 -> 1 (zero slope at both ends: no kick on entry).
inline double smoothstep(double x)
{
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

class Core
{
public:
    static constexpr int kSectors = 24;

    explicit Core(const Config & config = Config()) : c_(config) {}

    const Config & config() const { return c_; }
    bool active() const { return active_; }

    // points: LiDAR obstacles in base_link.
    // fixed:  obstacles that only ever count as still (kept for testing; the
    //         node passes none -- LiDAR only, operator's rule).
    // (px, py, pth): base_link pose in odom, to tell obstacle motion from
    // the car's own.  car_speed: measured, signed.  car_steer: current
    // steering command.
    // nav_driving: Nav2 is commanding motion right now (smaller buffer for
    // still things -- see keep_distance()).
    Decision update(double t, double px, double py, double pth, const std::vector<Point> & points,
                    const std::vector<Point> & fixed, double car_speed, double car_steer,
                    bool nav_driving = false)
    {
        Decision out;
        nav_driving_ = nav_driving;
        const double dt = last_t_ < 0.0 ? 0.05 : std::clamp(t - last_t_, 0.0, 0.2);
        last_t_ = t;

        prepare(points, fixed);
        track(t, dt, px, py, pth);

        // ---- 2. FIELD
        Field f = field(car_speed);
        out.min_gap = f.min_gap;
        out.max_approach = f.max_approach;
        out.movers = f.movers;
        mark_sides(out);
        // 0 for a calm approach, rising smoothly to 1 for a fast one.
        const double urgency = smoothstep((f.max_approach - c_.fast_from) / (c_.fast_full - c_.fast_from));

        // ---- 3. MOTION target
        double v_target = 0.0, steer_target = 0.0;
        const double push = std::hypot(f.ex, f.ey);
        if (push > 0.02)
        {
            double dir;
            double mag;
            if (std::abs(f.ex) >= 0.35 * std::abs(f.ey))
            {
                dir = f.ex > 0.0 ? 1.0 : -1.0;
                mag = std::abs(f.ex);
            }
            else
            {
                // Pushed sideways (someone at the flank): drive whichever way
                // has more room, steering away -- an Ackermann car cannot
                // step sideways.  Keep the previous choice unless the other
                // side is clearly roomier, so it does not dither.
                // Any front/back push at all decides: go the way it points
                // (choosing by "room" alone sent the car FORWARD into a
                // person in front -- the room check ignores people coming).
                // Room each way counts PEOPLE in line too (still_gap_ahead
                // leaves out whoever is coming, so a person in front looked
                // like open space and the car drove at them -- offline test).
                const double room_f = std::min(still_gap_ahead(1.0, 0.0, 0.0, 0.0), f.gap_front);
                const double room_b = std::min(still_gap_ahead(-1.0, 0.0, 0.0, 0.0), f.gap_back);
                if (std::abs(f.ex) > 0.03)
                    dir = f.ex > 0.0 ? 1.0 : -1.0;
                else
                {
                    // Someone at the flank: get the car OUT OF THEIR LINE by
                    // the shortest way -- they are nearer the nose -> back
                    // up, nearer the tail -> drive on (live test: side
                    // approaches were the weak spot; choosing by room alone
                    // often drove the long way past them).  Only if that way
                    // has no room does it take the other.  Near the middle,
                    // keep the previous choice (no dithering).
                    const double cx = 0.5 * (c_.front + c_.rear);
                    if (std::abs(f.side_x - cx) > 0.05 || side_dir_ == 0.0)
                        dir = (f.side_x > cx) ? -1.0 : 1.0;
                    else
                        dir = side_dir_;
                    const double room_dir = dir > 0.0 ? room_f : room_b;
                    const double room_other = dir > 0.0 ? room_b : room_f;
                    if (room_dir < c_.side_room_min && room_other > room_dir + 0.10)
                        dir = -dir;
                }
                mag = std::max(std::abs(f.ey), std::abs(f.ex));
                // A small purely sideways push is not worth a drive: the car
                // cannot step sideways, only creep along.
                if (std::abs(f.ex) <= 0.03 && mag < c_.side_move_min)
                    mag = 0.0;
            }
            // COMMIT to a direction.  Someone at the middle of the side makes
            // forward and back equally good; the choice flipped every few
            // scans (+0.55, -0.57, +0.49 ...), each flip a brake-and-reverse,
            // and the car never got out of their way -- the "struggling to
            // decide" seen live.  While moving, the opposite direction must
            // win flip_scans scans in a row (or the current way run out of
            // room) before the car turns round.
            // (A CLEAR push the other way -- something actually ahead or
            // behind -- overrides at once: commitment is against noise, not
            // against a person in front.)
            const bool clear_push = f.ex * dir > 0.0 && std::abs(f.ex) > c_.flip_push;
            if (std::abs(v_out_) > 0.05 && dir * v_out_ < 0.0 && !clear_push)
            {
                const double room_now = still_gap_ahead(v_out_, 0.0, 0.0, 0.0);
                if (++flip_votes_ < c_.flip_scans && stop_speed(room_now) >= c_.v_creep)
                    dir = (v_out_ > 0.0) ? 1.0 : -1.0;
                else
                    flip_votes_ = 0;
            }
            else
            {
                flip_votes_ = 0;
            }
            side_dir_ = dir;
            v_target = dir * std::min(mag, c_.v_max + urgency * (c_.v_max_fast - c_.v_max));
            steer_target = c_.max_steer * std::clamp(2.0 * f.ey / (std::abs(f.ex) + std::abs(f.ey)), -1.0, 1.0);
        }
        else
        {
            side_dir_ = 0.0;
        }

        // Never toward something still faster than the car can stop from,
        // and never along an arc that grazes it.
#ifdef SHIELD_DEBUG
        std::printf("      field ex %.3f ey %.3f -> v_target %.3f steer %.3f (room ahead %.3f)\n", f.ex, f.ey, v_target,
                    steer_target, v_target != 0.0 ? still_gap_ahead(v_target, 0.0, 0.0, 0.0) : -1.0);
#endif
        if (v_target != 0.0)
        {
            const double cap = stop_speed(still_gap_ahead(v_target, 0.0, 0.0, 0.0));
            v_target = std::copysign(std::min(std::abs(v_target), cap), v_target);
            if (std::abs(v_target) < 0.03)
                v_target = 0.0;
            else
                steer_target = safe_steer(v_target, steer_target);
            if (std::isnan(steer_target))
            {
                v_target = 0.0;
                steer_target = 0.0;
            }
        }

        // CREEP, NOT STOP-AND-GO.  A small correction asked for 0.04 m/s,
        // which this motor cannot hold: it lurched, stopped, lurched (live
        // log 2026-10-09).  Below v_creep the car either does not move, or
        // -- once a correction is worth making, or it is already moving that
        // way -- moves at a steady v_creep until the distance is restored.
        if (v_target != 0.0 && std::abs(v_target) < c_.v_creep)
        {
            const bool same_way = v_out_ * v_target > 0.0 && std::abs(v_out_) > 0.02;
            const bool room = stop_speed(still_gap_ahead(v_target, 0.0, 0.0, 0.0)) >= c_.v_creep;
            if (!room)
                ;                                   // near a wall: keep the slower, stoppable speed
            else if (same_way || std::abs(v_target) >= c_.creep_start)
                v_target = std::copysign(c_.v_creep, v_target);
            else
                v_target = 0.0;
        }

        // ---- trapped / emergency
        // Trapped = it has to stay put although something is too close, and
        // that has LASTED trapped_after: in passing (a scan or two while an
        // escape turns) it is not, and must not make the car say "cannot
        // move, please move away".
        const bool in_danger = f.min_gap < c_.margin || f.inside_keep_mover;
        const bool stuck = in_danger && std::abs(v_target) < 0.03 && f.pressure > 0.02;
        if (!stuck)
            stuck_since_ = -1.0;
        else if (stuck_since_ < 0.0)
            stuck_since_ = t;
        out.trapped = stuck && t - stuck_since_ >= c_.trapped_after;
        out.emergency = f.emergency;

        // ---- 4. SMOOTH
        if (!active_)
        {
            if (std::abs(v_target) < 0.06 && !out.trapped && !out.emergency)
            {
                out.mode = Mode::Idle;
                out.reason = "clear";
                out.target_speed = v_target;
                return out;
            }
            active_ = true;
            calm_since_ = -1.0;
            // Take over from whatever Nav2 was doing -- no jump.
            v_out_ = car_speed;
            steer_out_ = car_steer;
        }
        const bool slowing = std::abs(v_target) < std::abs(v_out_) || v_target * v_out_ < 0.0;
        const double rate = slowing ? (out.emergency ? c_.decel_emergency : c_.decel)
                                    : c_.accel_up + urgency * (c_.accel_up_fast - c_.accel_up);
        // Through zero only via zero: decelerate to 0 first, then build up.
        const double goal = (v_target * v_out_ < 0.0) ? 0.0 : v_target;
        v_out_ += std::clamp(goal - v_out_, -rate * dt, rate * dt);
        if (std::abs(v_out_) < 0.02 && goal == 0.0)
            v_out_ = 0.0;
        if (std::abs(v_out_) > 0.02 || v_target != 0.0)
        {
            const double want = steer_out_ + c_.steer_filter * (steer_target - steer_out_);
            steer_out_ += std::clamp(want - steer_out_, -c_.steer_rate * dt, c_.steer_rate * dt);
        }

        out.target_speed = v_target;
        out.speed = v_out_;
        out.steering = steer_out_;

        // ---- 5. HAND-BACK
        if (v_target == 0.0 && v_out_ == 0.0 && !out.trapped && !out.emergency)
        {
            if (calm_since_ < 0.0)
                calm_since_ = t;
            if (t - calm_since_ >= c_.release_hold)
            {
                active_ = false;
                out.mode = Mode::Idle;
                out.reason = "released";
                return out;
            }
        }
        else
        {
            calm_since_ = -1.0;
        }
        // One mode while the shield has the wheels -- its speed may be zero.
        // (Switching hold <-> escape every time the target touched zero was
        // the "mode changes" the live log counted by the hundred.)
        out.mode = Mode::Escape;
        out.reason = out.trapped ? "trapped" : (std::abs(v_out_) > 0.02 ? "escape" : "hold");
        return out;
    }

    void reset()
    {
        active_ = false;
        tracks_.clear();
        calm_since_ = -1.0;
        v_out_ = steer_out_ = 0.0;
        last_t_ = -1.0;
    }

private:
    struct Obstacle
    {
        double x, y;          // base_link
        double gap;
        double ux, uy;        // outward normal: from the car toward the point
        double vx, vy;        // its velocity (base_link axes), 0 unless a mover
        double approach;      // m/s toward the car (>= 0)
        bool mover;
        bool fixed;
        int sector;
    };
    struct Sample
    {
        double t, x, y;
    };
    struct Track
    {
        double x = 0, y = 0, vx = 0, vy = 0;   // odom, from the line fit
        double bx = 0, by = 0;                 // where it was first seen
        double last_t = 0;
        int hits = 0;
        int steady = 0;                        // scans in a row at a walking speed
        bool mover = false;
        std::vector<Sample> hist;              // recent centroids
    };

    // Least-squares line through the track's recent centroids: velocity, and
    // the position on that line now (less noisy than the last centroid).
    static void fit(Track & tr)
    {
        const auto & h = tr.hist;
        const double n = static_cast<double>(h.size());
        double mt = 0, mx = 0, my = 0;
        for (const auto & s : h)
        {
            mt += s.t;
            mx += s.x;
            my += s.y;
        }
        mt /= n;
        mx /= n;
        my /= n;
        double stt = 0, stx = 0, sty = 0;
        for (const auto & s : h)
        {
            stt += (s.t - mt) * (s.t - mt);
            stx += (s.t - mt) * (s.x - mx);
            sty += (s.t - mt) * (s.y - my);
        }
        if (h.size() < 2 || stt < 1e-6)
        {
            tr.x = h.back().x;
            tr.y = h.back().y;
            tr.vx = tr.vy = 0.0;
            return;
        }
        tr.vx = stx / stt;
        tr.vy = sty / stt;
        tr.x = mx + tr.vx * (h.back().t - mt);
        tr.y = my + tr.vy * (h.back().t - mt);
    }
    struct Field
    {
        double ex = 0.0, ey = 0.0;
        double min_gap = 99.0;
        double max_approach = 0.0;
        int movers = 0;
        bool inside_keep_mover = false;
        bool emergency = false;
        double pressure = 0.0;          // strongest single push (before front/back cancel)
        double side_x = 0.0;            // along the car, where the strongest SIDE push comes from
        double gap_front = 99.0;        // nearest anything directly ahead (in the car's lane)
        double gap_back = 99.0;         // ... directly behind
    };

    int sector_of(double x, double y) const
    {
        const double cx = 0.5 * (c_.front + c_.rear);
        double a = std::atan2(y, x - cx);
        if (a < 0.0)
            a += 2.0 * M_PI;
        return std::min(kSectors - 1, static_cast<int>(a / (2.0 * M_PI) * kSectors));
    }

    void prepare(const std::vector<Point> & points, const std::vector<Point> & fixed)
    {
        obs_.clear();
        auto add = [this](const Point & p, bool is_fixed) {
            double ux, uy;
            const double g = rect_gap(c_, p.x, p.y, &ux, &uy);
            if (g > c_.range)
                return;
            obs_.push_back({p.x, p.y, g, ux, uy, 0.0, 0.0, 0.0, false, is_fixed, sector_of(p.x, p.y)});
        };
        for (const auto & p : points)
            add(p, false);
        for (const auto & p : fixed)
            add(p, true);
    }

    // ---- 1. TRACK: cluster -> associate -> alpha-beta filter (odom frame).
    void track(double t, double dt, double px, double py, double pth)
    {
        const double c = std::cos(pth), s = std::sin(pth);
        // Order the LiDAR points by bearing around the car so neighbours on
        // one surface are consecutive.
        std::vector<int> idx;
        for (int i = 0; i < static_cast<int>(obs_.size()); ++i)
            if (!obs_[i].fixed)
                idx.push_back(i);
        const double cx = 0.5 * (c_.front + c_.rear);
        std::sort(idx.begin(), idx.end(), [&](int a, int b) {
            return std::atan2(obs_[a].y, obs_[a].x - cx) < std::atan2(obs_[b].y, obs_[b].x - cx);
        });
        std::vector<std::vector<int>> clusters;
        for (size_t k = 0; k < idx.size(); ++k)
        {
            const auto & p = obs_[idx[k]];
            if (!clusters.empty())
            {
                const auto & q = obs_[clusters.back().back()];
                if (std::hypot(p.x - q.x, p.y - q.y) < c_.cluster_link)
                {
                    clusters.back().push_back(idx[k]);
                    continue;
                }
            }
            clusters.push_back({idx[k]});
        }
        // The bearing order wraps around at +-180 deg.
        if (clusters.size() > 1)
        {
            const auto & a = obs_[clusters.back().back()];
            const auto & b = obs_[clusters.front().front()];
            if (std::hypot(a.x - b.x, a.y - b.y) < c_.cluster_link)
            {
                clusters.front().insert(clusters.front().begin(), clusters.back().begin(), clusters.back().end());
                clusters.pop_back();
            }
        }
        // STANDALONE only.  Something standing in the room (a person, a
        // leg, a chair leg) is IN FRONT of what is behind it: at both of its
        // edges the next LiDAR return is clearly further away (or there is
        // none).  Where beams meet a wall at a shallow angle the wall breaks
        // into 2-3-point fragments with more wall right beside them at the
        // same depth -- those slide along with the car and were being tracked
        // as walking people (corridor test, 2026-10-09).
        std::vector<int> pos(obs_.size(), -1);
        for (size_t k = 0; k < idx.size(); ++k)
            pos[idx[k]] = static_cast<int>(k);
        const int n_idx = static_cast<int>(idx.size());
        auto range_of = [&](int i) { return std::hypot(obs_[i].x - cx, obs_[i].y); };
        auto bearing_of = [&](int i) { return std::atan2(obs_[i].y, obs_[i].x - cx); };
        auto edge_free = [&](int end, int neighbour) {
            if (neighbour == end)
                return true;
            double gap = std::abs(bearing_of(neighbour) - bearing_of(end));
            gap = std::min(gap, 2.0 * M_PI - gap);
            return gap > 0.09 || range_of(neighbour) > range_of(end) + 0.15;   // 5 deg, 15 cm
        };
        std::vector<bool> standalone(clusters.size(), false);
        for (size_t k = 0; k < clusters.size(); ++k)
        {
            const int first = clusters[k].front(), last = clusters[k].back();
            const int before = idx[(pos[first] - 1 + n_idx) % n_idx];
            const int after = idx[(pos[last] + 1) % n_idx];
            standalone[k] = edge_free(first, before) && edge_free(last, after);
        }

        // Where every track should be now (for association only; last_t
        // stays "last SEEN" so unseen tracks expire).
        std::vector<Point> predicted;
        for (const auto & tr : tracks_)
        {
            const double h = t - tr.last_t;
            predicted.push_back({tr.x + tr.vx * h, tr.y + tr.vy * h});
        }
        std::vector<bool> used(tracks_.size(), false);
        std::vector<Track> born;
        born.reserve(clusters.size());           // pointers into it stay valid
        for (size_t ci = 0; ci < clusters.size(); ++ci)
        {
            const auto & cl = clusters[ci];
            if (!standalone[ci])
                continue;
            double mx = 0.0, my = 0.0, size = 0.0;
            for (int i : cl)
            {
                mx += obs_[i].x;
                my += obs_[i].y;
            }
            mx /= cl.size();
            my /= cl.size();
            for (int i : cl)
                size = std::max(size, std::hypot(obs_[i].x - mx, obs_[i].y - my));
            if (2.0 * size > c_.max_object || cl.size() < 2)
                continue;                        // a wall / single stray return
            // A straight, thin run of points is a piece of WALL, never a
            // person (legs and bodies are small curved blobs).  As the car
            // drives along a wall the LiDAR samples a slightly different
            // piece each scan, so such a piece seems to move WITH the car: it
            // was tracked as a "mover", the car ran from corridor walls, and
            // -- worse -- left that wall out of its don't-back-into-it check
            // (offline test 2026-10-09).
            if (cl.size() >= 4)
            {
                double sxx = 0.0, syy = 0.0, sxy = 0.0;
                for (int i : cl)
                {
                    const double dx = obs_[i].x - mx, dy = obs_[i].y - my;
                    sxx += dx * dx;
                    syy += dy * dy;
                    sxy += dx * dy;
                }
                const double tr = sxx + syy, det = sxx * syy - sxy * sxy;
                const double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));
                const double major = tr / 2.0 + disc, minor = tr / 2.0 - disc;
                const double length = 2.0 * std::sqrt(std::max(0.0, major / cl.size())) * 1.73;
                if (length > 0.20 && minor < 0.04 * major)
                    continue;
            }
            const double ox = px + c * mx - s * my, oy = py + s * mx + c * my;
            // Nearest unused track inside the gate.
            int best = -1;
            double best_d = 1e9;
            for (size_t k = 0; k < tracks_.size(); ++k)
            {
                if (used[k])
                    continue;
                const double gate = c_.track_gate + std::hypot(tracks_[k].vx, tracks_[k].vy) * dt;
                const double d = std::hypot(ox - predicted[k].x, oy - predicted[k].y);
                if (d < gate && d < best_d)
                {
                    best = static_cast<int>(k);
                    best_d = d;
                }
            }
            Track * tr;
            if (best >= 0)
            {
                used[best] = true;
                tr = &tracks_[best];
            }
            else
            {
                born.push_back(Track{});
                tr = &born.back();
                tr->bx = ox;
                tr->by = oy;
                // A person the tracker lost for a moment (legs merging,
                // standing very close) is still a person when found again
                // -- otherwise someone who walked up and stopped turned into
                // "furniture" and the car stopped keeping its distance
                // (live test 2026-10-09).
                for (const auto & g : lost_movers_)
                    if (t - g.t < c_.mover_memory && std::hypot(ox - g.x, oy - g.y) < 0.4)
                        tr->mover = true;
            }
            tr->last_t = t;
            ++tr->hits;
            // Velocity = least-squares line through the last track_window
            // seconds of centroids.  Legs swing a couple of cm every scan;
            // a filter that chases each scan turned that into +-0.3 m/s of
            // fake motion (a person standing still got run from, offline
            // test 2026-10-09).  A line fit over ~6 scans is ~+-0.1 m/s and
            // still answers a real walk within a few scans.
            tr->hist.push_back({t, ox, oy});
            while (!tr->hist.empty() && t - tr->hist.front().t > c_.track_window)
                tr->hist.erase(tr->hist.begin());
            fit(*tr);
            // Mover: has really TRAVELLED (net, from where it was first seen),
            // or the line fit over a full window shows a steady walk -- so a
            // person walking in is answered early, not after 25 cm.
            const double speed = std::hypot(tr->vx, tr->vy);
            const bool full_window = tr->hist.size() >= 5 &&
                                     tr->hist.back().t - tr->hist.front().t > 0.8 * c_.track_window;
            tr->steady = (full_window && speed > c_.mover_speed) ? tr->steady + 1 : 0;
            if (std::hypot(tr->x - tr->bx, tr->y - tr->by) > c_.mover_distance || tr->steady >= 3)
                tr->mover = true;
            // Velocity in base_link axes, trusted once there is a line to fit.
            double vx = 0.0, vy = 0.0;
            if (tr->hist.size() >= 3 && speed > c_.speed_deadband)
            {
                vx = c * tr->vx + s * tr->vy;
                vy = -s * tr->vx + c * tr->vy;
            }
            for (int i : cl)
            {
                auto & o = obs_[i];
                o.mover = tr->mover;
                o.vx = tr->mover ? vx : 0.0;
                o.vy = tr->mover ? vy : 0.0;
                o.approach = std::max(0.0, -(o.vx * o.ux + o.vy * o.uy));
            }
        }
        // Forget tracks not seen for a while (remembering where people were);
        // adopt the new ones.
        std::vector<Track> keep;
        for (size_t k = 0; k < tracks_.size(); ++k)
        {
            if (used[k] || t - tracks_[k].last_t < c_.track_timeout)
                keep.push_back(tracks_[k]);
            else if (tracks_[k].mover)
                lost_movers_.push_back({t, tracks_[k].x, tracks_[k].y});
        }
        lost_movers_.erase(std::remove_if(lost_movers_.begin(), lost_movers_.end(),
                                          [&](const Sample & g) { return t - g.t > c_.mover_memory; }),
                           lost_movers_.end());
        for (auto & b : born)
            keep.push_back(b);
        tracks_.swap(keep);
    }

    // ---- 2. FIELD: per sector, the strongest push; summed over sectors.
    // CAR-SHAPED BUFFER (operator, 2026-10-09: "it's a car, not a cube --
    // front and back need more space than the sides").  The distance kept
    // from a point depends on which side of the car it is on: an oval with
    // the front/back distance along the car and the side distance across it,
    // blended smoothly for the corners.
    //   people (movers, moving or stopped) ........ 0.30 front/back, 0.15 sides
    //   anything else, car parked .................. 0.22 front/back, 0.10 sides
    //   anything else, while Nav2 is driving ....... 0.14 front/back, 0.10 sides
    // The last one is small on purpose: Nav2 plans around furniture itself
    // (and the hardware's clearance cap slows it); a large still-object
    // buffer would make the shield fight Nav2 near every table.
    double keep_distance(const Obstacle & o) const
    {
        double fb, side;
        if (o.mover)
        {
            fb = c_.keep_front_mover;
            side = c_.keep_side_mover;
        }
        else if (nav_driving_)
        {
            fb = c_.keep_front_driving;
            side = c_.keep_side_still;
        }
        else
        {
            fb = c_.keep_front_still;
            side = c_.keep_side_still;
        }
        const double k = 1.0 / std::sqrt((o.ux / fb) * (o.ux / fb) + (o.uy / side) * (o.uy / side));
        return std::max(k, c_.margin + 0.005);
    }

    Field field(double car_speed) const
    {
        Field f;
        double fwd = 0.0, back = 0.0, left = 0.0, right = 0.0;
        double gap_front = 99.0, gap_back = 99.0, gap_left = 99.0, gap_right = 99.0;
        double side_best = 0.0;
        for (const auto & o : obs_)
        {
            f.min_gap = std::min(f.min_gap, o.gap);
            // Nearest thing at each end / side (pushing or not), for
            // centring in a squeeze below -- only what is IN LINE with the
            // car: directly ahead/behind within its width, directly beside
            // within its length.  (By direction alone, a corridor wall
            // running past the front corner counted as "in front" and the
            // car centred itself between pieces of its own side walls.)
            const double lane = c_.half_width + 0.04;
            if (std::abs(o.y) < lane && o.x > c_.front)
                gap_front = std::min(gap_front, o.gap);
            else if (std::abs(o.y) < lane && o.x < c_.rear)
                gap_back = std::min(gap_back, o.gap);
            if (o.x < c_.front && o.x > c_.rear)
            {
                if (o.y > c_.half_width)
                    gap_left = std::min(gap_left, o.gap);
                else if (o.y < -c_.half_width)
                    gap_right = std::min(gap_right, o.gap);
            }
            const double approach = o.approach;
            if (o.mover)
            {
                ++f.movers;
                f.max_approach = std::max(f.max_approach, approach);
            }
            // Two parts, both rising smoothly (no line to cross):
            //  MATCH -- a mover coming at the car is matched in speed, faded
            //    in over the buffer zone, judged by where it will be in
            //    look_ahead (so a person still far off but walking in is
            //    already being answered, gently).  It stops when they stop.
            //  HOLD  -- the distance itself: only once actually inside the
            //    distance kept (keep_gap for people, the margin for anything
            //    still), and only enough to restore it -- no running on.
            double w = 0.0;          // 0..1, how much the car answers this point
            double p = 0.0;          // the push, m/s
            const double keep = keep_distance(o);
            // Straight-line inside the buffer: every cm of missing distance
            // gets its share of push (a smoothstep was so flat at the edge
            // that a person stepping back 10 cm moved the car under 2 cm).
            // Smoothness comes from the command's accel/decel limits.
            const double hold = std::clamp((keep - o.gap) / (keep - 0.5 * c_.margin), 0.0, 1.0);
            if (o.mover)
            {
                // Faster approach -> look further ahead (start earlier): a
                // brisk walker into the car's SIDE needs the whole car moved
                // out of their path, which only works if it starts early;
                // a slow walker keeps the short look and is not fled from.
                const double fast = smoothstep((approach - c_.fast_from) / (c_.fast_full - c_.fast_from));
                const double look = c_.look_ahead + c_.look_ahead_fast * fast;
                const double g_eff = o.gap - approach * look;
                const double match = smoothstep((keep + c_.mover_zone - g_eff) / c_.mover_zone);
                const double factor = c_.approach_match + fast * (c_.approach_match_fast - c_.approach_match);
                p = match * approach * factor + hold * c_.v_push;
                w = std::max(match, hold);
            }
            else
            {
                p = hold * c_.v_push;
                w = hold;
            }
            if (o.gap < c_.margin)                         // inside the margin: firmer
                p += c_.v_push * (c_.margin - o.gap) / c_.margin;
            // Something STILL right beside the car (a wall alongside) never
            // makes it drive: driving along it cannot widen the gap beside
            // it -- the car only wandered off along the wall (offline test,
            // most noise patterns).  It still counts everywhere else: the
            // corner-sweep check, squeezes, trapped.  People beside the car
            // do push (driving out of their path does help).
            if (!o.mover && std::abs(o.uy) > 0.7)
            {
                p = 0.0;
                w = 0.0;
            }
            if (o.mover && o.gap < keep)
                f.inside_keep_mover = true;
            // Emergency: contact within the reaction time + a gentle stop.
            const double own = std::max(0.0, car_speed * o.ux);
            const double closing = approach + own;
            if (o.gap - closing * c_.reaction - own * own / (2.0 * c_.soft_decel) < 0.5 * c_.margin ||
                (o.mover && approach > 1.0 && o.gap - approach * c_.look_ahead < c_.margin))
                f.emergency = true;
            if (w <= 0.0 && p <= 0.0)
                continue;
#ifdef SHIELD_DEBUG
            if (p > 0.1)
                std::printf("        push %.2f from (%.2f,%.2f) gap %.3f mover %d u(%.2f,%.2f)\n", p, o.x, o.y, o.gap,
                            o.mover, o.ux, o.uy);
#endif
            // The push on the car is AWAY from the point: -u.
            back = std::max(back, p * std::max(0.0, o.ux));      // something ahead pushes back
            fwd = std::max(fwd, p * std::max(0.0, -o.ux));       // something behind pushes forward
            // Sideways: also remember WHERE along the car the strongest side
            // pusher is (nearer the nose or the tail) -- see the motion step.
            const double side = p * std::abs(o.uy);
            if (side > side_best)
            {
                side_best = side;
                f.side_x = o.x;
            }
            right = std::max(right, p * std::max(0.0, o.uy));    // something left pushes right
            left = std::max(left, p * std::max(0.0, -o.uy));
        }
        // The STRONGEST push each way, not a sum: one person covers several
        // points and angles, and summing them pushed 2-3x too hard (a slow
        // walker sent the car off at full speed, offline test 2026-10-09).
        f.pressure = std::max(std::max(fwd, back), std::max(left, right));
        f.gap_front = gap_front;
        f.gap_back = gap_back;
        f.ex = fwd - back;
        f.ey = left - right;
        // SQUEEZED (pushed from both ends at once -- you in front, the wall
        // behind): balance DISTANCES, not push strengths.  The car settles
        // half-way between them: step back 10 cm and it comes forward 5
        // (operator's own example, 2026-10-09).  Same across the car.
        // A squeeze = something pushing at one end AND something near the
        // other end too.  Then sit half-way between them -- toward the side
        // with MORE room (+ = forward / left).  Once one end is clear (the
        // person steps a metre back) it is no squeeze: the car just keeps
        // its normal buffer from what is left.
        // Only once nobody is still coming: while someone walks in, the car
        // backs off at THEIR pace (the stopping-room cap keeps it off the
        // wall behind); centring at its own gentle pace meanwhile let two
        // people walking in catch it (offline test).
        const bool calm = f.max_approach < c_.approach_calm;
        if (calm && (fwd > 0.02 || back > 0.02) && gap_front < c_.squeeze_range && gap_back < c_.squeeze_range)
            f.ex = centre(gap_front - gap_back);
        if (calm && (left > 0.02 || right > 0.02) && gap_left < c_.squeeze_range_side &&
            gap_right < c_.squeeze_range_side)
            f.ey = centre(gap_left - gap_right);
        return f;
    }

    // Speed toward the middle of a squeeze: `excess` = how much more room
    // the + side (front / left) has than the other; half of it is the
    // distance to the middle.
    double centre(double excess) const
    {
        const double off = 0.5 * excess;
        if (std::abs(off) < c_.centre_deadband)
            return 0.0;
        return std::clamp(c_.centre_gain * off, -c_.v_push, c_.v_push);
    }

    // Steering for `speed`, chosen by SIMULATING each angle: the car's whole
    // outline (corners included) over the horizon, with moving obstacles
    // carried along their tracked velocity.  Still obstacles must stay
    // outside min(margin, current gap); among the angles that manage that,
    // the one keeping the biggest gap to EVERYTHING wins, ties going to the
    // angle the push asked for (so it stays smooth).  NaN if no angle is
    // safe.  Needed because steering only by the push direction ignored the
    // corners: reversing with left steer swings the FRONT to the right --
    // straight into someone walking in from the right (offline test).
    double safe_steer(double speed, double wanted) const
    {
        double best = std::nan("");
        double best_score = -1e9;
        for (int k = -4; k <= 5; ++k)
        {
            const double s = (k == 5) ? wanted : c_.max_steer * k / 4.0;
            double worst, room, mean;
            if (!arc_clear(speed, s, &worst, &room, &mean))
                continue;
            // + OPEN SPACE: an arc that ends pointing into free room beats
            // one that ends nose-first into the wall behind -- with space
            // around, the car steers out of a squeeze instead of only
            // backing into it.  + keep the side it already chose, so the
            // wheels do not hunt left-right between near-equal arcs.
            // + the MEAN gap over the arc: when someone fast comes from the
            // side every arc ends close to them, the worst-gap term ties,
            // and the room bonus alone swung the car toward them (offline
            // test) -- the arc that stays away longer must win.
            const double cap = c_.keep_gap + c_.mover_zone;
            double score = 0.5 * std::min(worst, cap) + 0.5 * mean +
                           c_.room_weight * std::min(room, 1.0) -
                           0.03 * std::abs(s - wanted) / c_.max_steer;
            if (last_steer_choice_ * s > 0.0)
                score += 0.02;
            if (score > best_score)
            {
                best_score = score;
                best = s;
            }
        }
        if (!std::isnan(best))
            last_steer_choice_ = (std::abs(best) < 0.05) ? 0.0 : best;
        return best;
    }

    // Driving `speed` along `steer` for the horizon (capped by the stopping
    // rule as it goes), does the car stay at least min(margin, current gap)
    // from everything still?  `worst`: the smallest gap to anything on the
    // way, movers moved along their velocity.  `room`: at the end of the
    // arc, how much free lane is left ahead in the direction of travel.
    // `mean`: the average over the arc of the step's smallest gap (capped).
    bool arc_clear(double speed, double steer, double * worst = nullptr, double * room = nullptr,
                   double * mean = nullptr) const
    {
        double rx = c_.rear_axle_x, ry = 0.0, th = 0.0;
        const double off = -c_.rear_axle_x;
        const int n = static_cast<int>(std::round(c_.horizon / c_.step));
        const double cap = c_.keep_gap + c_.mover_zone;
        double v = speed;
        double low = 99.0, sum = 0.0;
        for (int i = 1; i <= n; ++i)
        {
            double step_low = 99.0;
            const double bx0 = rx + off * std::cos(th), by0 = ry + off * std::sin(th);
            v = std::copysign(std::min(std::abs(speed), stop_speed(still_gap_ahead(speed, bx0, by0, th))), speed);
            rx += v * std::cos(th) * c_.step;
            ry += v * std::sin(th) * c_.step;
            th += v * std::tan(steer) / c_.wheelbase * c_.step;
            const double bx = rx + off * std::cos(th), by = ry + off * std::sin(th);
            const double ct = std::cos(th), st = std::sin(th);
            const double time = i * c_.step;
            for (const auto & o : obs_)
            {
                const double ox = o.x + o.vx * time, oy = o.y + o.vy * time;
                const double lx = ct * (ox - bx) + st * (oy - by);
                const double ly = -st * (ox - bx) + ct * (oy - by);
                const double g = rect_gap(c_, lx, ly);
                step_low = std::min(step_low, g);
                if (!coming(o) && g < std::min(c_.margin, o.gap) - 0.01)
                    return false;
            }
            low = std::min(low, step_low);
            sum += std::min(step_low, cap);
        }
        if (worst)
            *worst = low;
        if (mean)
            *mean = sum / n;
        if (room)
            *room = still_gap_ahead(speed, rx + off * std::cos(th), ry + off * std::sin(th), th);
        return true;
    }

    // Nearest gap to a still obstacle on the side the car is driving toward
    // (ahead half plus the flanks), car at pose (bx, by, th).
    double still_gap_ahead(double speed, double bx, double by, double th) const
    {
        if (speed == 0.0)
            return 99.0;
        // Only what is IN THE CAR'S LANE beyond the bumper it is driving
        // toward.  It used to count everything in that half of the car --
        // including things level with its SIDES, which a straight move
        // never gets closer to -- so a wall beside the car capped the
        // reverse speed to a crawl ("bad at going back", live test
        // 2026-10-09).  Sweeping a corner into something beside the car is
        // the arc check's job (arc_clear).
        const double ct = std::cos(th), st = std::sin(th);
        const double bumper = speed > 0.0 ? c_.front : c_.rear;
        // Straight travel sweeps only the car's own width: +4 cm.  (12 cm
        // put walls running alongside "in the lane" and the car called
        // itself trapped in a corridor -- offline test.)
        const double lane = c_.half_width + 0.04;
        double best = 99.0;
        for (const auto & o : obs_)
        {
            if (coming(o))
                continue;
            const double lx = ct * (o.x - bx) + st * (o.y - by);
            const double ly = -st * (o.x - bx) + ct * (o.y - by);
            if (std::copysign(1.0, speed) * (lx - bumper) < -0.02 || std::abs(ly) > lane)
                continue;                         // beside or behind, or outside the lane
            best = std::min(best, rect_gap(c_, lx, ly));
        }
        return best;
    }

    // Left out of the "never into it" checks ONLY when it is actually coming
    // at the car (that is what the car is getting away from).  Anything else
    // -- still, tracked or not -- is something the car must not drive into.
    static bool coming(const Obstacle & o) { return o.mover && o.approach > 0.10; }

    // Fastest speed from which the car still stops before the margin, given
    // `gap` of room, with a latency allowance.
    double stop_speed(double gap) const
    {
        const double room = gap - c_.margin - 0.03;
        if (room <= 0.0)
            return 0.0;
        return std::sqrt(2.0 * c_.brake_decel * room);
    }

    void mark_sides(Decision & out) const
    {
        for (const auto & o : obs_)
        {
            const double near = keep_distance(o);
            if (o.gap > near)
                continue;
            if (o.x > c_.front - 0.03)
                out.side_front = true;
            else if (o.x < c_.rear + 0.03)
                out.side_back = true;
            if (o.y > c_.half_width - 0.02 && o.x <= c_.front + 0.05 && o.x >= c_.rear - 0.05)
                out.side_left = true;
            else if (o.y < -c_.half_width + 0.02 && o.x <= c_.front + 0.05 && o.x >= c_.rear - 0.05)
                out.side_right = true;
        }
    }

    Config c_;
    std::vector<Obstacle> obs_;
    std::vector<Track> tracks_;
    std::vector<Sample> lost_movers_;        // where recently lost people were (t, x, y)
    bool active_ = false;
    double calm_since_ = -1.0;
    double v_out_ = 0.0, steer_out_ = 0.0;
    double side_dir_ = 0.0;
    double last_t_ = -1.0;
    bool nav_driving_ = false;
    double stuck_since_ = -1.0;
    int flip_votes_ = 0;                       // scans in a row the other direction won
    mutable double last_steer_choice_ = 0.0;   // safe_steer's previous pick (hysteresis)
};

}  // namespace shield

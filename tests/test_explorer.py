#!/usr/bin/python3
"""Offline checks for qcar2_explorer frontier detection and goal choice.

Builds synthetic occupancy grids and runs the node's real methods against
them. Never constructs a Node / calls rclpy.init().
"""
import math
import os
import sys
import types

import numpy as np

sys.path.insert(0, os.path.expanduser(
    '~/Desktop/Qcar-rviz/qcar2_ws/src/qcar2_rviz_gui/scripts'))
import qcar2_explorer as ex  # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'{"PASS" if cond else "FAIL"}  {name}' + (f'   {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


def make_grid(array, res=0.05, ox=0.0, oy=0.0):
    h, w = array.shape
    info = types.SimpleNamespace(
        width=w, height=h, resolution=res,
        origin=types.SimpleNamespace(position=types.SimpleNamespace(x=ox, y=oy)))
    header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=id(array) % 10**6, nanosec=0))
    return types.SimpleNamespace(info=info, header=header, data=array.flatten().tolist())


class Stub:
    # Mirrors the node's own declared defaults so the test cannot drift into
    # asserting behaviour the shipped configuration does not actually have.
    min_cells = 8              # min_frontier_cells
    blacklist_radius = 0.5
    max_failures = 3
    min_goal_distance = 0.6
    max_goal_distance = 8.0
    distance_weight = 0.35
    wall_clearance = 0.30
    unknown_backoff = 0.35
    approach_radius = 1.5
    blacklist_ttl = 120.0
    find_frontiers = ex.Explorer.find_frontiers
    choose = ex.Explorer.choose
    rank = ex.Explorer.rank
    blacklisted = ex.Explorer.blacklisted
    strike = ex.Explorer.strike
    clearance_mask = ex.Explorer.clearance_mask
    approach_goal = ex.Explorer.approach_goal

    def __init__(self, grid):
        self.map = grid
        self.blacklist = []
        self._clearance = None


def test_no_frontier_in_fully_known_map():
    """A fully explored room has nothing left to go to -> run should end."""
    a = np.zeros((40, 40), dtype=np.int16)
    a[0, :] = a[-1, :] = a[:, 0] = a[:, -1] = 100
    f = Stub(make_grid(a)).find_frontiers()
    check('fully known map yields no frontiers', f == [], f'{len(f)} found')


def test_frontier_found_at_unknown_boundary():
    a = np.zeros((60, 60), dtype=np.int16)
    a[:, 30:] = -1
    f = Stub(make_grid(a, res=0.1)).find_frontiers()
    check('frontier detected at the free/unknown boundary', len(f) >= 1, f'{len(f)} clusters')
    if f:
        cx, _cy, _size = max(f, key=lambda c: c[2])
        check('frontier centroid sits on the boundary', abs(cx - 2.9) < 0.15, f'cx={cx:.2f}')


def test_origin_offset_applied():
    """Map origin must be honoured or every goal is offset by the map corner."""
    a = np.zeros((60, 60), dtype=np.int16)
    a[:, 30:] = -1
    f = Stub(make_grid(a, res=0.1, ox=-5.0, oy=-3.0)).find_frontiers()
    cx, cy, _ = max(f, key=lambda c: c[2])
    check('origin offset applied to frontier x', abs(cx - (2.9 - 5.0)) < 0.15, f'cx={cx:.2f}')
    check('origin offset applied to frontier y', cy < 0, f'cy={cy:.2f}')


def test_tiny_speckle_ignored():
    """A couple of stray unknown pixels must not become a goal."""
    a = np.zeros((60, 60), dtype=np.int16)
    a[10, 10] = -1
    f = Stub(make_grid(a)).find_frontiers()
    check('sub-min_cells speckle ignored', f == [], f'{len(f)} clusters')


def test_walls_are_not_frontiers():
    """Unknown behind a wall is not reachable free space -> not a frontier."""
    a = np.zeros((60, 60), dtype=np.int16)
    a[:, 30] = 100
    a[:, 31:] = -1
    f = Stub(make_grid(a, res=0.1)).find_frontiers()
    check('unknown behind a wall is not a frontier', f == [], f'{len(f)} clusters')


def test_choose_prefers_big_and_close():
    frontiers = [(2.0, 0.0, 10), (3.0, 0.0, 400), (7.5, 0.0, 420)]
    best = Stub(None).choose(frontiers, (0.0, 0.0))
    check('big-and-close frontier chosen',
          best is not None and abs(best[0] - 3.0) < 1e-6, f'{best}')


def test_choose_rejects_too_close():
    """Ackermann: a goal under the turning radius is unreachable, not a goal."""
    best = Stub(None).choose([(0.3, 0.0, 500)], (0.0, 0.0))
    check('sub-minimum-distance frontier rejected (cannot turn in place)',
          best is None, f'{best}')


def test_blacklist_after_repeated_failure():
    s = Stub(None)
    target = [(3.0, 0.0, 200)]
    check('reachable before any failures', s.choose(target, (0.0, 0.0)) is not None)
    for _ in range(3):
        s.strike(3.0, 0.0)
    check('blacklisted after max_failures strikes',
          s.choose(target, (0.0, 0.0)) is None, f'blacklist={s.blacklist}')


def test_blacklist_is_local():
    """Blacklisting one dead end must not poison a different frontier."""
    s = Stub(None)
    for _ in range(3):
        s.strike(3.0, 0.0)
    other = s.choose([(3.0, 4.0, 200)], (0.0, 0.0))
    check('a different frontier stays reachable', other is not None, f'{other}')


def test_frontier_found_with_real_cartographer_values():
    """REGRESSION: the car explored nothing in the real room.

    The earlier tests used idealised grids where free space is exactly 0.
    Cartographer does not publish that. Its default miss_probability is 0.49,
    so a cell seen empty ONCE is published as 49, and it takes ~27 LiDAR passes
    to drift below 25. The frontier -- the edge of what has been seen -- is by
    definition the least-observed part of the map, so it sits in that 30..49
    band. With free defined as <= 25 no frontier was ever found, exploration
    "completed" instantly, and the car drove "home" to where it already was.
    """
    a = np.full((60, 60), 10, dtype=np.int16)      # well-observed floor
    a[:, 24:30] = 47                               # seen only a few times
    a[:, 30:] = -1                                 # never seen
    f = Stub(make_grid(a, res=0.1)).find_frontiers()
    check('frontier found where floor was seen only a few times (value ~47)',
          len(f) >= 1, f'{len(f)} clusters')


def test_once_seen_cells_count_as_free():
    """Exactly Cartographer's first-observation value, 49."""
    a = np.full((60, 60), 49, dtype=np.int16)
    a[:, 30:] = -1
    f = Stub(make_grid(a, res=0.1)).find_frontiers()
    check('cells observed once (value 49) still form a frontier', len(f) >= 1,
          f'{len(f)} clusters')


def test_uncertain_band_is_not_free():
    """50..64 is 'no better than a coin flip' -- must not be treated as free."""
    a = np.full((60, 60), 10, dtype=np.int16)
    a[:, 28:30] = 58
    a[:, 30:] = -1
    f = Stub(make_grid(a, res=0.1)).find_frontiers()
    check('an uncertain (50-64) band does not create a frontier', f == [],
          f'{len(f)} clusters')


# ------------------------------------------------------ goal placement

def room(res=0.1):
    """Walled free area (x 0..3 m) opening onto unknown at x >= 3 m."""
    a = np.full((60, 60), 45, dtype=np.int16)      # realistic lightly-seen floor
    a[:, 30:] = -1
    a[0, :30] = a[-1, :30] = a[:, 0] = 100          # walls on three sides
    return a


def test_goal_is_pulled_back_from_the_unknown_edge():
    s = Stub(make_grid(room(), res=0.1))
    fx, fy, _ = max(s.find_frontiers(), key=lambda c: c[2])
    g = s.approach_goal(fx, fy)
    check('a safe approach goal exists', g is not None, f'{g}')
    if g:
        # Unknown starts at x = 3.0 m; the goal must sit back from it.
        check('goal is backed off from the unknown edge (>= 0.35 m)',
              3.0 - g[0] >= 0.34, f'goal x={g[0]:.2f}, edge at 3.00')
        check('goal is still close to the frontier (within 1.5 m)',
              math.hypot(g[0] - fx, g[1] - fy) <= 1.5 + 1e-6,
              f'{math.hypot(g[0] - fx, g[1] - fy):.2f} m')


def test_goal_keeps_clear_of_walls():
    """A frontier running along a wall must not put the goal against it."""
    a = room()
    s = Stub(make_grid(a, res=0.1))
    g = s.approach_goal(2.9, 0.15)                 # frontier point hugging y=0 wall
    check('goal found for a frontier beside a wall', g is not None, f'{g}')
    if g:
        check('goal keeps >= 0.30 m from the wall', g[1] >= 0.30 - 1e-6, f'goal y={g[1]:.2f}')


def test_no_safe_parking_returns_none():
    """A frontier seen through a gap too narrow to park in -> no goal, not a crash goal."""
    a = np.full((60, 60), 100, dtype=np.int16)
    a[28:32, 20:30] = 45                           # 0.4 m-wide slot
    a[28:32, 30:] = -1
    s = Stub(make_grid(a, res=0.1))
    check('narrow slot yields no approach goal', s.approach_goal(2.9, 3.0) is None)


def test_blacklist_expires():
    """Failed frontiers get retried once the map has had time to change."""
    s = Stub(None)
    for _ in range(3):
        s.strike(3.0, 0.0)
    check('blacklisted right after failing', s.blacklisted(3.0, 0.0))
    s.blacklist = [(bx, by, st, t - 999) for bx, by, st, t in s.blacklist]
    check('eligible again after the TTL', not s.blacklisted(3.0, 0.0))


def test_rank_orders_candidates():
    s = Stub(None)
    ranked = s.rank([(2.0, 0.0, 10), (3.0, 0.0, 400), (7.5, 0.0, 420)], (0.0, 0.0))
    check('rank returns best first', ranked and abs(ranked[0][0] - 3.0) < 1e-6, f'{ranked}')
    check('rank keeps the fallbacks too', len(ranked) == 3, f'{len(ranked)}')


for fn in (test_goal_is_pulled_back_from_the_unknown_edge, test_goal_keeps_clear_of_walls,
           test_no_safe_parking_returns_none, test_blacklist_expires, test_rank_orders_candidates,
           test_frontier_found_with_real_cartographer_values,
           test_once_seen_cells_count_as_free, test_uncertain_band_is_not_free,
           test_no_frontier_in_fully_known_map, test_frontier_found_at_unknown_boundary,
           test_origin_offset_applied, test_tiny_speckle_ignored,
           test_walls_are_not_frontiers, test_choose_prefers_big_and_close,
           test_choose_rejects_too_close, test_blacklist_after_repeated_failure,
           test_blacklist_is_local):
    print(f'\n--- {fn.__name__} ---')
    fn()

print('\n' + ('ALL PASSED' if not FAIL else f'{len(FAIL)} FAILED: {FAIL}'))
sys.exit(1 if FAIL else 0)

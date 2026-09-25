#!/usr/bin/python3
"""Offline checks for qcar2_object_nav name matching and standoff planning.

Run with scripts/run_tests.sh. Imports the node module but never constructs a
Node and never calls rclpy.init(), so it cannot disturb a running stack.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'qcar2_ws', 'src', 'qcar2_rviz_gui', 'scripts'))
import qcar2_object_nav as on  # noqa: E402

FAIL = []


def check(name, cond, detail=''):
    print(f'{"PASS" if cond else "FAIL"}  {name}' + (f'   {detail}' if detail else ''))
    if not cond:
        FAIL.append(name)


def test_normalize():
    cases = {
        'go to the air cooler': 'air cooler',
        'Go To The Air Cooler': 'air cooler',
        'please drive to the fridge': 'fridge',
        'move to the sofa': 'sofa',
        'navigate towards the tv': 'tv',
        'take me to the bed': 'bed',
        'air cooler': 'air cooler',
        'the chair': 'chair',
    }
    for raw, want in cases.items():
        got = on.normalize(raw)
        check(f'normalize({raw!r})', got == want, f'-> {got!r} want {want!r}')


class MatchStub:
    match = on.ObjectNav.match

    def __init__(self, labels, synonyms):
        self.objects = [{'label': l, 'x': 0.0, 'y': 0.0} for l in labels]
        self.synonyms = {k.lower(): v for k, v in synonyms.items()}


def test_match():
    labels = ['air cooler', 'television', 'sofa', 'refrigerator', 'office chair']
    syn = {'cooler': 'air cooler', 'ac': 'air cooler', 'tv': 'television',
           'couch': 'sofa', 'fridge': 'refrigerator', 'air cooler': 'air cooler',
           'television': 'television', 'sofa': 'sofa', 'refrigerator': 'refrigerator'}
    m = MatchStub(labels, syn)
    cases = {
        'go to the air cooler': 'air cooler',
        'go to the cooler': 'air cooler',          # synonym
        'drive to the ac': 'air cooler',           # short alias
        'go to the tv': 'television',              # synonym
        'move to the couch': 'sofa',               # synonym
        'go to the fridge': 'refrigerator',        # synonym
        'go to the aircooler': 'air cooler',       # speech-recogniser run-together
        'go to the televison': 'television',       # misspelling -> fuzzy
        'go to the chair': 'office chair',         # substring
    }
    for raw, want in cases.items():
        got = m.match(raw)
        check(f'match({raw!r})', got == want, f'-> {got!r} want {want!r}')
    check('unknown object returns None', m.match('go to the helicopter') is None)


class GoalStub:
    standoff, min_standoff, max_standoff = 0.70, 0.45, 1.60
    standoff_goal = on.ObjectNav.standoff_goal

    def __init__(self, car=None, blocked=()):
        self._car = car
        self._blocked = blocked

    def car_xy(self):
        return self._car

    def cell_free(self, x, y):
        return all(math.hypot(x - bx, y - by) > r for bx, by, r in self._blocked)


def test_standoff_basic():
    """Goal sits short of the object, on the car's side, facing the object."""
    obj = {'x': 3.0, 'y': 0.0}
    g = GoalStub(car=(0.0, 0.0)).standoff_goal(obj)
    check('standoff goal produced', g is not None)
    if g is None:
        return
    gx, gy, yaw, _dist = g
    d = math.hypot(gx - obj['x'], gy - obj['y'])
    check('goal is ~standoff from the object', abs(d - 0.70) < 1e-6, f'{d:.3f} m')
    check('goal is on the CAR side of the object', gx < obj['x'], f'gx={gx:.3f}')
    want = math.atan2(obj['y'] - gy, obj['x'] - gx)
    check('goal heading faces the object',
          abs(math.atan2(math.sin(yaw - want), math.cos(yaw - want))) < 1e-9, f'yaw={yaw:.3f}')


def test_standoff_never_inside_object():
    """The goal must never be the object's own coordinate (= drive into it)."""
    obj = {'x': 2.0, 'y': 1.0}
    gx, gy, _, _ = GoalStub(car=(0.0, 0.0)).standoff_goal(obj)
    sep = math.hypot(gx - obj['x'], gy - obj['y'])
    check('goal is NOT the object coordinate', sep > 0.4, f'separation {sep:.3f} m')


def test_standoff_routes_around_blockage():
    """Cooler in a corner: straight-in cell blocked, so sweep around it."""
    obj = {'x': 3.0, 'y': 0.0}
    blocked = [(3.0 - 0.70, 0.0, 0.35)]
    g = GoalStub(car=(0.0, 0.0), blocked=blocked).standoff_goal(obj)
    check('blocked approach still finds a goal', g is not None)
    if g is None:
        return
    gx, gy, _, _ = g
    check('chosen goal avoids the blocked cell',
          math.hypot(gx - blocked[0][0], gy - blocked[0][1]) > blocked[0][2],
          f'goal=({gx:.2f}, {gy:.2f})')


def test_standoff_all_blocked():
    """Fully enclosed object -> report failure rather than a bad goal."""
    g = GoalStub(car=(0.0, 0.0), blocked=[(3.0, 0.0, 5.0)]).standoff_goal({'x': 3.0, 'y': 0.0})
    check('fully blocked object yields no goal', g is None, f'-> {g}')


def test_no_costmap_still_works():
    """Before the costmap arrives, navigation by name must not be dead."""
    g = GoalStub(car=(0.0, 0.0)).standoff_goal({'x': 1.5, 'y': -2.0})
    check('works with no costmap yet', g is not None)


TESTS = (test_normalize, test_match, test_standoff_basic,
         test_standoff_never_inside_object, test_standoff_routes_around_blockage,
         test_standoff_all_blocked, test_no_costmap_still_works)

if __name__ == '__main__':
    for fn in TESTS:
        print(f'\n--- {fn.__name__} ---')
        fn()
    print('\n' + ('ALL PASSED' if not FAIL else f'{len(FAIL)} FAILED: {FAIL}'))
    sys.exit(1 if FAIL else 0)
